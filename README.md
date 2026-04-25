# ICE3 Balluff BCM0003 BLOB Data Collector

Python tooling for collecting vibration BLOB data from a **Balluff BCM0003** condition monitoring sensor connected to a **Pepperl+Fuchs ICE3 IO-Link master** over **Modbus/TCP**.

The project configures the Balluff data provider through IO-Link ISDU, triggers data collection, transfers BLOB data from the Balluff BLOB channel, parses the payload, and exports timestamped CSV files.

---

## Features

- ICE3 ISDU communication over Modbus/TCP.
- Sequential collection from IO-Link ports `1..8`.
- Balluff BCM0003 BLOB configuration, trigger, transfer, finish, abort, and restart handling.
- Raw acceleration export for `rawX`, `rawY`, and `rawZ`.
- Optional amplitude and envelope spectrum BLOB support.
- Timestamped CSV output per BLOB and per port.
- Recovery handling for interrupted or stuck BLOB transfers.
- Console and file logging for commissioning and troubleshooting.

---

## Project Structure

```text
.
├── ModbusClientWrapper.py      # pyModbusTCP wrapper
├── modbus_ISDU.py              # ICE3 ISDU client
├── Balluff_blob_functions.py   # Balluff BCM0003 commands, constants, and parsers
├── blob_state_machine.py       # Acquisition and BLOB transfer state machine
├── run_multiple_ports.py       # Sequential multi-port runner
└── README.md
```

---

## Requirements

- Python 3.10 or newer
- Pepperl+Fuchs ICE3 IO-Link master with Modbus/TCP enabled
- Balluff BCM0003 connected to an ICE3 IO-Link port
- Python package: `pyModbusTCP`

Install dependencies:

```bash
pip install -r requirements.txt
```

---

## Configuration

Create a `.env` file in the project folder:

```env
ICE_HOST=192.168.137.21
ICE_TCP_PORT=502
ICE_UNIT_ID=1
ICE_IOL_PORTS=1,2,3

BLOB_TIMEOUT_S=180
BLOB_POLL_S=0.2
BLOB_PACKET_POLL_S=0.05

BLOB_LOG_DIR=blob_logs
LOG_LEVEL=INFO
```

Minimum required values:

| Variable | Description |
|---|---|
| `ICE_HOST` | IP address of the ICE3 master. |
| `ICE_TCP_PORT` | Modbus/TCP port, usually `502`. |
| `ICE_UNIT_ID` | Modbus unit ID, usually `1`. |
| `ICE_IOL_PORTS` | Comma-separated IO-Link ports to run sequentially. |

---

## Quick Start

Run one port first:

```powershell
$env:ICE_IOL_PORTS="1"
python run_multiple_ports.py
```

Run ports 1, 2, and 3 sequentially:

```powershell
$env:ICE_IOL_PORTS="1,2,3"
python run_multiple_ports.py
```

The default runner is intended to collect raw acceleration BLOBs:

```python
blob_types = ["rawX", "rawY", "rawZ"]
DPTG_value = 2      # ISDU trigger
RADPTM_value = 0    # Raw acceleration starts after trigger
DCAS_value = 6      # X, Y, Z axes
DCT_value = 0       # Raw acceleration only
```

---

## Running from Python

### Single port, raw X/Y/Z

```python
from Balluff_blob_functions import set_ice_isdu_client
from blob_state_machine import run_blob_state_machine_multi

set_ice_isdu_client(
    host="192.168.137.21",
    iol_port=1,
    tcp_port=502,
    unit_id=1,
)

result = run_blob_state_machine_multi(
    blob_types=["rawX", "rawY", "rawZ"],
    timeout_s=180.0,
    poll_s=0.2,
    packet_poll_s=0.05,
    DPTG_value=2,
    RADPTM_value=0,
    DCAS_value=6,
    DCT_value=0,
    save_csv=True,
    output_dir="blob_csv/port_1",
    wait_for_write_responses=False,
)

for blob_type, transfer in result.results.items():
    print(blob_type, transfer.success, transfer.csv_path)
```

### Transfer from already-ready data

Use this after an acquisition has already completed and the BLOB status is ready:

```python
from blob_state_machine import collect_ready_blobs_multi

result = collect_ready_blobs_multi(
    blob_types=["rawY", "rawZ"],
    timeout_s=180.0,
    save_csv=True,
    output_dir="blob_csv/port_1_extra",
    verify_ready_status=True,
)
```

---

## Output

CSV files are saved under the selected output directory:

```text
blob_csv/
└── port_1/
    ├── rawX_YYYYMMDD_HHMMSS_mmm.csv
    ├── rawY_YYYYMMDD_HHMMSS_mmm.csv
    └── rawZ_YYYYMMDD_HHMMSS_mmm.csv
```

Raw acceleration CSV columns:

| Column | Description |
|---|---|
| `timestamp_ms` | Trigger timestamp from the Balluff payload. |
| `axis` | X, Y, or Z. |
| `sample_index` | Sample number. |
| `raw_int16` | Signed raw acceleration value. |
| `accel_mg` | Acceleration in mg. |
| `accel_g` | Acceleration in g. |

---

## Important Notes

- Keep `wait_for_write_responses=False` unless your ICE3 response behavior is confirmed stable. Writes can succeed even when immediate write-response polling times out.
- Do not manually read Balluff `BLOB_CH` index `50` during an active transfer. Each read can advance the BLOB stream.
- Run ports sequentially. The project switches the active ISDU client per port and is not designed for parallel collection.
- Start validation with `rawX` on one port before running all axes or multiple ports.

---

## Troubleshooting

### Cannot connect to ICE3

Check the ICE3 IP address, subnet, Modbus/TCP enable setting, firewall rules, and TCP port `502`.

### Timeout waiting for BLOB status

Confirm the sensor is connected to the selected IO-Link port, the port is in IO-Link mode, and the Balluff feature restarted correctly.

### Timeout during BLOB transfer

Run cleanup, then retry the acquisition:

```python
from blob_state_machine import cleanup_active_blob_transfer

cleanup_active_blob_transfer(strategy="abort", quiet=False)
```

### CSV parser error

Start with `rawX` only. Parser errors usually indicate an interrupted transfer, unexpected packet content, or a mismatch between selected BLOB type and payload.

---

## Safety

This project writes configuration and trigger commands to an IO-Link sensor. Validate all behavior on a bench setup before using it near active machinery or production equipment.
