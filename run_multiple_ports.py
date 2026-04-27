from __future__ import annotations

import logging
import os
from datetime import datetime
from pathlib import Path
from typing import Optional

try:
    from dotenv import load_dotenv
    load_dotenv()
except Exception:
    # python-dotenv is optional. PowerShell/CMD environment variables still work.
    pass

import Balluff_blob_functions as BF
from Balluff_blob_functions import get_ice_connection_info, set_ice_isdu_client
from blob_state_machine import run_blob_state_machine_multi


def env_str(name: str, default: str) -> str:
    return os.getenv(name, default).strip()


def env_int(name: str, default: int) -> int:
    return int(env_str(name, str(default)))


def env_float(name: str, default: float) -> float:
    return float(env_str(name, str(default)))


def env_bool(name: str, default: bool) -> bool:
    text = env_str(name, "1" if default else "0").lower()
    return text in {"1", "true", "yes", "y", "on"}


def env_optional_int(name: str, default: Optional[int] = None) -> Optional[int]:
    text = os.getenv(name)
    if text is None or not text.strip():
        return default
    return int(text.strip())


def parse_ports_from_env(default: str = "1") -> list[int]:
    """
    Read IO-Link ports from ICE_IOL_PORTS.

    Examples:
      ICE_IOL_PORTS=1,2,3
      ICE_IOL_PORTS=1
    """
    raw = os.getenv("ICE_IOL_PORTS", default)
    ports: list[int] = []

    for item in raw.replace(";", ",").split(","):
        item = item.strip()
        if not item:
            continue

        port = int(item)

        if port < 1 or port > 8:
            raise ValueError(f"IO-Link port must be 1..8; got {port}")

        ports.append(port)

    if not ports:
        raise ValueError("No IO-Link ports configured")

    return ports


def parse_blob_types_from_env(default: str = "rawX,rawY,rawZ") -> Optional[list[str]]:
    raw = env_str("BLOB_TYPES", default)
    if not raw or raw.lower() in {"auto", "none", "null"}:
        return None

    blob_types = [item.strip() for item in raw.replace(";", ",").split(",") if item.strip()]
    invalid = [blob_type for blob_type in blob_types if blob_type not in BF.BLOB_TYPE_TO_ID]
    if invalid:
        raise ValueError(f"Unsupported BLOB type(s): {invalid}")

    return blob_types


def configure_logging() -> None:
    log_dir = Path(os.getenv("BLOB_LOG_DIR", "blob_logs"))
    log_dir.mkdir(parents=True, exist_ok=True)

    log_path = log_dir / f"blob_run_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"

    logging.basicConfig(
        level=os.getenv("LOG_LEVEL", "INFO").upper(),
        format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
        handlers=[
            logging.FileHandler(log_path, encoding="utf-8"),
            logging.StreamHandler(),
        ],
    )

    logging.info("Logging to %s", log_path)


def main() -> int:
    configure_logging()

    ice_host = env_str("ICE_HOST", "192.168.137.21")
    ice_tcp_port = env_int("ICE_TCP_PORT", 502)
    ice_unit_id = env_int("ICE_UNIT_ID", 1)

    ports_to_test = parse_ports_from_env(default="1")

    timeout_s = env_float("BLOB_TIMEOUT_S", 180.0)
    poll_s = env_float("BLOB_POLL_S", 0.2)
    packet_poll_s = env_float("BLOB_PACKET_POLL_S", 0.05)

    blob_types = parse_blob_types_from_env(default="rawX,rawY,rawZ")
    dptg_value = env_int("DPTG_VALUE", 2)
    radptm_value = env_int("RADPTM_VALUE", 0)
    dcas_value = env_int("DCAS_VALUE", 6)
    dct_value = env_int("DCT_VALUE", 0)

    save_csv = env_bool("SAVE_CSV", True)
    output_root = env_str("BLOB_OUTPUT_ROOT", "blob_csv")
    word_byte_order = env_str("WORD_BYTE_ORDER", "big")
    object_endian = env_str("OBJECT_ENDIAN", "big")
    wait_for_write_responses = env_bool("WAIT_FOR_WRITE_RESPONSES", False)
    restart_before_config = env_bool("RESTART_BEFORE_CONFIG", True)
    cleanup_before_acquisition = env_bool("CLEANUP_BEFORE_ACQUISITION", True)
    cleanup_on_error = env_bool("CLEANUP_ON_ERROR", True)
    continue_on_blob_error = env_bool("CONTINUE_ON_BLOB_ERROR", False)
    force_reconfigure = env_bool("FORCE_RECONFIGURE", False)
    reuse_ready_data = env_bool("REUSE_READY_DATA", True)

    strict_transfer = env_bool("STRICT_TRANSFER", True)
    validate_packet_counter = env_bool("VALIDATE_PACKET_COUNTER", True)
    validate_expected_length = env_bool("VALIDATE_EXPECTED_LENGTH", True)
    blob_info_payload_length_offset = env_optional_int("BLOB_INFO_LENGTH_OFFSET", None)
    blob_info_payload_length_size = env_int("BLOB_INFO_LENGTH_SIZE", 4)
    trim_to_expected_length = env_bool("TRIM_TO_EXPECTED_LENGTH", True)

    print("\nSequential ICE3/Balluff BLOB collection")
    print("=======================================")
    print(f"ICE host: {ice_host}")
    print(f"IO-Link ports: {ports_to_test}")
    print(f"BLOB types: {blob_types if blob_types is not None else 'auto'}")
    print(f"Output root: {output_root}")
    print(f"Strict transfer: {strict_transfer}")
    print(f"Reuse ready data: {reuse_ready_data}")
    print(f"Force reconfigure: {force_reconfigure}")

    overall_success = True

    for iol_port in ports_to_test:
        print("\n====================================")
        print(f"Running BLOB state machine on IO-Link port {iol_port}")
        print("====================================")

        try:
            set_ice_isdu_client(
                host=ice_host,
                iol_port=iol_port,
                tcp_port=ice_tcp_port,
                unit_id=ice_unit_id,
            )

            logging.info("Active ICE connection: %s", get_ice_connection_info())

            result = run_blob_state_machine_multi(
                blob_types=blob_types,
                timeout_s=timeout_s,
                poll_s=poll_s,
                packet_poll_s=packet_poll_s,
                DPTG_value=dptg_value,
                RADPTM_value=radptm_value,
                DCAS_value=dcas_value,
                DCT_value=dct_value,
                save_csv=save_csv,
                output_dir=f"{output_root}/port_{iol_port}",
                word_byte_order=word_byte_order,
                object_endian=object_endian,
                wait_for_write_responses=wait_for_write_responses,
                restart_before_config=restart_before_config,
                cleanup_before_acquisition=cleanup_before_acquisition,
                cleanup_on_error=cleanup_on_error,
                continue_on_blob_error=continue_on_blob_error,
                force_reconfigure=force_reconfigure,
                reuse_ready_data=reuse_ready_data,
                strict_transfer=strict_transfer,
                validate_packet_counter=validate_packet_counter,
                validate_expected_length=validate_expected_length,
                blob_info_payload_length_offset=blob_info_payload_length_offset,
                blob_info_payload_length_size=blob_info_payload_length_size,
                trim_to_expected_length=trim_to_expected_length,
            )

            if result.success:
                print(f"\nPort {iol_port} completed successfully.")
            else:
                overall_success = False
                print(f"\nPort {iol_port} failed.")
                print(f"Error: {result.error}")
                logging.error("Port %s failed: %s", iol_port, result.error)

            for blob_type, transfer in result.results.items():
                diag = transfer.diagnostics
                print(
                    f"{blob_type}: success={transfer.success}, "
                    f"bytes={len(transfer.payload)}, packets={transfer.packet_count}, "
                    f"csv={transfer.csv_path}"
                )
                if diag.markers_seen:
                    print(f"  markers={diag.markers_seen}")
                if diag.counter_mismatches:
                    print(f"  counter_mismatches={diag.counter_mismatches}")
                if diag.validation_warnings:
                    print(f"  warnings={diag.validation_warnings}")

        except KeyboardInterrupt:
            print("\nInterrupted by user.")
            logging.warning("Run interrupted by user")
            return 130

        except Exception as exc:
            overall_success = False
            print(f"\nPort {iol_port} crashed with exception:")
            print(exc)
            logging.exception("Port %s crashed", iol_port)

    return 0 if overall_success else 1


if __name__ == "__main__":
    raise SystemExit(main())
