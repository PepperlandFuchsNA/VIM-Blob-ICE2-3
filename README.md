# ICE3 Balluff BCM0003 BLOB Data Collector

Python tooling for collecting vibration BLOB data from a **Balluff BCM0003** condition monitoring sensor connected to a **Pepperl+Fuchs ICE3 IO-Link master** over **Modbus/TCP**.

The project configures the Balluff data provider through IO-Link ISDU, triggers data collection, transfers BLOB payloads, parses the data, and exports timestamped CSV files.

---

## Features

- ICE3 ISDU communication over Modbus/TCP.
- Sequential collection from IO-Link ports `1..8`.
- Simultaneous multi-port collection using threaded per-port ISDU clients.
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
├── run_parallel_ports.py       # Simultaneous threaded multi-port runner
└── README.md
```

---

## Requirements

- Python 3.10 or newer
- Pepperl+Fuchs ICE3 IO-Link master with Modbus/TCP enabled
- Balluff BCM0003 connected to one or more ICE3 IO-Link ports
- Python package: `pyModbusTCP`

Install dependencies:

```bash
pip install -r requirements.txt
```

If you are using a `.env` file, install `python-dotenv` as well:

```bash
pip install python-dotenv
```

---

## Configuration

Create a `.env` file in the project folder:

```env
ICE_HOST=192.168.137.21
ICE_TCP_PORT=502
ICE_UNIT_ID=1
ICE_IOL_PORTS=1,2,3

BLOB_TYPES=rawX,rawY,rawZ
BLOB_TIMEOUT_S=180
BLOB_POLL_S=0.2
BLOB_PACKET_POLL_S=0.05

BLOB_OUTPUT_ROOT=blob_csv
BLOB_LOG_DIR=blob_logs
LOG_LEVEL=INFO

PARALLEL_MAX_WORKERS=3
```

Minimum required values:

| Variable | Description |
|---|---|
| `ICE_HOST` | IP address of the ICE3 master. |
| `ICE_TCP_PORT` | Modbus/TCP port, usually `502`. |
| `ICE_UNIT_ID` | Modbus unit ID, usually `1`. |
| `ICE_IOL_PORTS` | Comma-separated IO-Link ports to collect from. |
| `BLOB_TYPES` | Comma-separated BLOB types, usually `rawX,rawY,rawZ`. |

---

## Quick Start

### Sequential collection

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

Sequential collection is the safest commissioning mode because only one IO-Link port is active at a time.

---

### Simultaneous collection

Run two ports in parallel first:

```powershell
$env:ICE_IOL_PORTS="1,2"
$env:PARALLEL_MAX_WORKERS="2"
python run_parallel_ports.py
```

Run ports 1 through 8 in parallel:

```powershell
$env:ICE_IOL_PORTS="1,2,3,4,5,6,7,8"
$env:PARALLEL_MAX_WORKERS="8"
python run_parallel_ports.py
```

The parallel runner creates an independent ISDU client per worker thread. Do not use `set_ice_isdu_client()` for parallel collection because that function changes a shared global client and is intended for sequential use only.

---

## Default Acquisition

The default collection target is raw acceleration for all three axes:

```python
blob_types = ["rawX", "rawY", "rawZ"]
DPTG_value = 2      # ISDU trigger
RADPTM_value = 0    # Raw acceleration starts after trigger
DCAS_value = 6      # X, Y, Z axes
DCT_value = 0       # Raw acceleration only
```

Start with raw acceleration before enabling spectrum BLOBs.

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
- Use `run_multiple_ports.py` for sequential collection.
- Use `run_parallel_ports.py` for simultaneous collection.
- For parallel collection, each worker must use its own ISDU client. Avoid shared global client switching inside threads.
- Start validation with `rawX` on one port before running all axes, multiple ports, or parallel collection.

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

### Parallel collection is unstable

Reduce the number of workers and test two ports first:

```powershell
$env:ICE_IOL_PORTS="1,2"
$env:PARALLEL_MAX_WORKERS="2"
python run_parallel_ports.py
```

If two ports are stable, increase one port at a time.

### CSV parser error

Start with `rawX` only. Parser errors usually indicate an interrupted transfer, unexpected packet content, or a mismatch between selected BLOB type and payload.

---

## Safety

This project writes configuration and trigger commands to an IO-Link sensor. Validate all behavior on a bench setup before using it near active machinery or production equipment.
