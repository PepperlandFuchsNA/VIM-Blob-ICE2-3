from __future__ import annotations

import logging
import os
from datetime import datetime
from pathlib import Path

try:
    from dotenv import load_dotenv
    load_dotenv()
except Exception:
    # python-dotenv is optional. PowerShell/CMD environment variables still work.
    pass

from Balluff_blob_functions import get_ice_connection_info, set_ice_isdu_client
from blob_state_machine import run_blob_state_machine_multi


def parse_ports_from_env(default: str = "1,2,3") -> list[int]:
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

    ice_host = os.getenv("ICE_HOST", "192.168.137.21")
    ice_tcp_port = int(os.getenv("ICE_TCP_PORT", "502"))
    ice_unit_id = int(os.getenv("ICE_UNIT_ID", "1"))

    ports_to_test = parse_ports_from_env(default="1,2,3")

    timeout_s = float(os.getenv("BLOB_TIMEOUT_S", "180"))
    poll_s = float(os.getenv("BLOB_POLL_S", "0.2"))
    packet_poll_s = float(os.getenv("BLOB_PACKET_POLL_S", "0.05"))

    # Production-safe default for your current raw X/Y/Z test.
    blob_types = ["rawX", "rawY", "rawZ"]
    dptg_value = 2
    radptm_value = 0
    dcas_value = 6
    dct_value = 0

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

                save_csv=True,
                output_dir=f"blob_csv/port_{iol_port}",

                word_byte_order="big",
                object_endian="big",

                wait_for_write_responses=False,
                restart_before_config=True,
                cleanup_before_acquisition=True,
                continue_on_blob_error=False,
            )

            if result.success:
                print(f"\nPort {iol_port} completed successfully.")

                for blob_type, transfer in result.results.items():
                    print(f"{blob_type}: {transfer.csv_path}")

            else:
                overall_success = False
                print(f"\nPort {iol_port} failed.")
                print(f"Error: {result.error}")
                logging.error("Port %s failed: %s", iol_port, result.error)

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
