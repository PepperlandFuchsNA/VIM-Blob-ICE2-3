"""
run_parallel_ports.py

Simultaneous multi-port BLOB collection runner for a Pepperl+Fuchs ICE3 IO-Link
master with Balluff BCM0003 sensors.

This runner is designed for the existing project structure where
Balluff_blob_functions.py uses a module-level ICE_ISDU object. For parallel
collection, a normal shared global client is unsafe, so this script replaces
BF.ICE_ISDU with a thread-local proxy. Each worker thread gets its own ISDU
client connected to a different physical IO-Link port.

Start with two ports first before scaling to all 8 ports.
"""
from __future__ import annotations

import inspect
import os
import threading
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional

try:
    from dotenv import load_dotenv
except ImportError:  # python-dotenv is optional
    load_dotenv = None

import modbus_ISDU as ISDU
import Balluff_blob_functions as BF
from blob_state_machine import MultiBlobStateMachineResult, run_blob_state_machine_multi


# ---------------------------------------------------------------------------
# Optional .env loading
# ---------------------------------------------------------------------------
if load_dotenv is not None:
    load_dotenv()


# ---------------------------------------------------------------------------
# Thread-local ICE_ISDU proxy
# ---------------------------------------------------------------------------
class ThreadLocalISDUProxy:
    """
    Provides BF.ICE_ISDU.read_isdu(...) / write_isdu(...) using the client
    assigned to the current thread.

    This prevents thread races caused by replacing one global ICE_ISDU object
    while multiple port state machines are running at the same time.
    """

    def __init__(self) -> None:
        self._local = threading.local()

    def set_client(self, client: ISDU.ISDU) -> None:
        self._local.client = client

    def get_client(self) -> ISDU.ISDU:
        client = getattr(self._local, "client", None)
        if client is None:
            raise RuntimeError(
                "No thread-local ISDU client is configured for this thread. "
                "Call THREAD_LOCAL_ICE.set_client(...) before using Balluff functions."
            )
        return client

    def read_isdu(self, *args, **kwargs):
        return self.get_client().read_isdu(*args, **kwargs)

    def write_isdu(self, *args, **kwargs):
        return self.get_client().write_isdu(*args, **kwargs)


THREAD_LOCAL_ICE = ThreadLocalISDUProxy()

# Install the proxy once. All Balluff helper functions now use per-thread clients.
BF.ICE_ISDU = THREAD_LOCAL_ICE


# ---------------------------------------------------------------------------
# Configuration helpers
# ---------------------------------------------------------------------------
def _env_str(name: str, default: str) -> str:
    return os.getenv(name, default).strip()


def _env_int(name: str, default: int) -> int:
    return int(_env_str(name, str(default)))


def _env_float(name: str, default: float) -> float:
    return float(_env_str(name, str(default)))


def _env_bool(name: str, default: bool) -> bool:
    text = _env_str(name, "1" if default else "0").lower()
    return text in {"1", "true", "yes", "y", "on"}


def _env_optional_int(name: str, default: Optional[int] = None) -> Optional[int]:
    text = os.getenv(name)
    if text is None or not text.strip():
        return default
    return int(text.strip())


def _parse_int_list(text: str) -> list[int]:
    output: list[int] = []
    for part in text.split(","):
        part = part.strip()
        if not part:
            continue
        value = int(part)
        if not 1 <= value <= 8:
            raise ValueError(f"IO-Link port must be 1..8, got {value}")
        output.append(value)

    if not output:
        raise ValueError("No IO-Link ports configured")

    return output


def _parse_blob_types(text: str) -> Optional[list[str]]:
    text = text.strip()
    if not text or text.lower() in {"auto", "none", "null"}:
        return None

    blob_types = [part.strip() for part in text.split(",") if part.strip()]
    if not blob_types:
        return None

    invalid = [blob_type for blob_type in blob_types if blob_type not in BF.BLOB_TYPE_TO_ID]
    if invalid:
        raise ValueError(f"Unsupported BLOB type(s): {invalid}")

    return blob_types


def _filter_supported_kwargs(func, kwargs: dict) -> dict:
    """
    Lets this runner work with slightly older/newer versions of the state-machine
    function by only passing keyword arguments supported by the installed file.
    """
    signature = inspect.signature(func)
    supported = set(signature.parameters)
    return {key: value for key, value in kwargs.items() if key in supported}


@dataclass(frozen=True)
class ParallelRunConfig:
    host: str
    tcp_port: int
    unit_id: int
    iol_ports: list[int]
    max_workers: int
    blob_types: Optional[list[str]]
    timeout_s: float
    poll_s: float
    packet_poll_s: float
    dptg_value: int
    radptm_value: int
    dcas_value: int
    dct_value: int
    save_csv: bool
    output_root: str
    word_byte_order: str
    object_endian: str
    wait_for_write_responses: bool
    restart_before_config: bool
    cleanup_before_acquisition: bool
    cleanup_on_error: bool
    force_reconfigure: bool
    reuse_ready_data: bool
    strict_transfer: bool
    validate_packet_counter: bool
    validate_expected_length: bool
    blob_info_payload_length_offset: Optional[int]
    blob_info_payload_length_size: int
    trim_to_expected_length: bool


def load_config() -> ParallelRunConfig:
    ports = _parse_int_list(_env_str("ICE_IOL_PORTS", "1,2,3"))
    max_workers = _env_int("PARALLEL_MAX_WORKERS", len(ports))
    max_workers = max(1, min(max_workers, len(ports)))

    return ParallelRunConfig(
        host=_env_str("ICE_HOST", "192.168.137.21"),
        tcp_port=_env_int("ICE_TCP_PORT", 502),
        unit_id=_env_int("ICE_UNIT_ID", 1),
        iol_ports=ports,
        max_workers=max_workers,
        blob_types=_parse_blob_types(_env_str("BLOB_TYPES", "rawX,rawY,rawZ")),
        timeout_s=_env_float("BLOB_TIMEOUT_S", 180.0),
        poll_s=_env_float("BLOB_POLL_S", 0.2),
        packet_poll_s=_env_float("BLOB_PACKET_POLL_S", 0.05),
        dptg_value=_env_int("DPTG_VALUE", 2),
        radptm_value=_env_int("RADPTM_VALUE", 0),
        dcas_value=_env_int("DCAS_VALUE", 6),
        dct_value=_env_int("DCT_VALUE", 0),
        save_csv=_env_bool("SAVE_CSV", True),
        output_root=_env_str("BLOB_OUTPUT_ROOT", "blob_csv_parallel"),
        word_byte_order=_env_str("WORD_BYTE_ORDER", "big"),
        object_endian=_env_str("OBJECT_ENDIAN", "big"),
        wait_for_write_responses=_env_bool("WAIT_FOR_WRITE_RESPONSES", False),
        restart_before_config=_env_bool("RESTART_BEFORE_CONFIG", True),
        cleanup_before_acquisition=_env_bool("CLEANUP_BEFORE_ACQUISITION", True),
        cleanup_on_error=_env_bool("CLEANUP_ON_ERROR", True),
        force_reconfigure=_env_bool("FORCE_RECONFIGURE", False),
        reuse_ready_data=_env_bool("REUSE_READY_DATA", True),
        strict_transfer=_env_bool("STRICT_TRANSFER", True),
        validate_packet_counter=_env_bool("VALIDATE_PACKET_COUNTER", True),
        validate_expected_length=_env_bool("VALIDATE_EXPECTED_LENGTH", True),
        blob_info_payload_length_offset=_env_optional_int("BLOB_INFO_LENGTH_OFFSET", None),
        blob_info_payload_length_size=_env_int("BLOB_INFO_LENGTH_SIZE", 4),
        trim_to_expected_length=_env_bool("TRIM_TO_EXPECTED_LENGTH", True),
    )


# ---------------------------------------------------------------------------
# Port worker
# ---------------------------------------------------------------------------
def run_one_port(iol_port: int, config: ParallelRunConfig) -> MultiBlobStateMachineResult:
    thread_name = threading.current_thread().name
    output_dir = str(Path(config.output_root) / f"port_{iol_port}")

    print(f"\n[{thread_name}] Starting IO-Link port {iol_port}")

    client = ISDU.ISDU(
        host=config.host,
        iol_port=iol_port,
        tcp_port=config.tcp_port,
        unit_id=config.unit_id,
    )
    THREAD_LOCAL_ICE.set_client(client)

    kwargs = {
        "blob_types": config.blob_types,
        "timeout_s": config.timeout_s,
        "poll_s": config.poll_s,
        "packet_poll_s": config.packet_poll_s,
        "DPTG_value": config.dptg_value,
        "RADPTM_value": config.radptm_value,
        "DCAS_value": config.dcas_value,
        "DCT_value": config.dct_value,
        "save_csv": config.save_csv,
        "output_dir": output_dir,
        "word_byte_order": config.word_byte_order,
        "object_endian": config.object_endian,
        "wait_for_write_responses": config.wait_for_write_responses,
        # These are supported by the newer production state machine.
        # They are automatically filtered out if your local function lacks them.
        "restart_before_config": config.restart_before_config,
        "cleanup_before_acquisition": config.cleanup_before_acquisition,
        "cleanup_on_error": config.cleanup_on_error,
        "force_reconfigure": config.force_reconfigure,
        "reuse_ready_data": config.reuse_ready_data,
        "strict_transfer": config.strict_transfer,
        "validate_packet_counter": config.validate_packet_counter,
        "validate_expected_length": config.validate_expected_length,
        "blob_info_payload_length_offset": config.blob_info_payload_length_offset,
        "blob_info_payload_length_size": config.blob_info_payload_length_size,
        "trim_to_expected_length": config.trim_to_expected_length,
    }

    result = run_blob_state_machine_multi(
        **_filter_supported_kwargs(run_blob_state_machine_multi, kwargs)
    )

    print(f"[{thread_name}] Finished IO-Link port {iol_port}: success={result.success}")
    return result


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main() -> int:
    config = load_config()

    print("\nParallel ICE3/Balluff BLOB collection")
    print("=====================================")
    print(f"ICE host: {config.host}")
    print(f"TCP port: {config.tcp_port}")
    print(f"Unit ID: {config.unit_id}")
    print(f"IO-Link ports: {config.iol_ports}")
    print(f"Max workers: {config.max_workers}")
    print(f"BLOB types: {config.blob_types if config.blob_types is not None else 'auto'}")
    print(f"Output root: {config.output_root}")
    print(f"Strict transfer: {config.strict_transfer}")
    print(f"Reuse ready data: {config.reuse_ready_data}")
    print(f"Force reconfigure: {config.force_reconfigure}")
    print("\nTip: start with ICE_IOL_PORTS=1,2 before running all 8 ports.\n")

    started = time.monotonic()
    results: dict[int, MultiBlobStateMachineResult] = {}
    failures: dict[int, str] = {}

    with ThreadPoolExecutor(
        max_workers=config.max_workers,
        thread_name_prefix="iol-port",
    ) as executor:
        future_to_port = {
            executor.submit(run_one_port, iol_port, config): iol_port
            for iol_port in config.iol_ports
        }

        for future in as_completed(future_to_port):
            iol_port = future_to_port[future]

            try:
                result = future.result()
                results[iol_port] = result
                if not result.success:
                    failures[iol_port] = result.error or "Unknown state-machine failure"

            except KeyboardInterrupt:
                print("\nKeyboard interrupt received. Stopping...")
                raise

            except Exception as exc:
                failures[iol_port] = str(exc)
                print(f"\nPort {iol_port} crashed: {exc}")

    elapsed = time.monotonic() - started

    print("\nParallel run summary")
    print("====================")
    for iol_port in config.iol_ports:
        result = results.get(iol_port)

        if result is None:
            print(f"Port {iol_port}: FAILED - {failures.get(iol_port, 'No result')}")
            continue

        print(f"Port {iol_port}: {'OK' if result.success else 'FAILED'}")
        for blob_type, transfer in result.results.items():
            diag = transfer.diagnostics
            print(
                f"  {blob_type}: success={transfer.success}, "
                f"bytes={len(transfer.payload)}, packets={transfer.packet_count}, "
                f"csv={transfer.csv_path}"
            )
            if diag.markers_seen:
                print(f"    markers={diag.markers_seen}")
            if diag.counter_mismatches:
                print(f"    counter_mismatches={diag.counter_mismatches}")
            if diag.validation_warnings:
                print(f"    warnings={diag.validation_warnings}")

        if not result.success and result.error:
            print(f"  Error: {result.error}")

    print(f"\nElapsed: {elapsed:.1f} s")

    if failures:
        print("\nOne or more ports failed.")
        return 1

    print("\nAll ports completed successfully.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
