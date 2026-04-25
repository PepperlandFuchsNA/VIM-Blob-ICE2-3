# ICE3 Modbus/TCP Balluff BCM0003 BLOB Data Collector

Professional Python tooling for collecting vibration BLOB data from a **Balluff BCM0003 condition monitoring sensor** connected to a **Pepperl+Fuchs ICE3 IO-Link master** over **Modbus/TCP**.

The project configures the Balluff data provider through IO-Link ISDU, triggers data collection, transfers BLOB payloads through the Balluff `BLOB_CH` channel, parses raw acceleration/spectrum data, and exports timestamped CSV files.

---

## Features

- Communicates with Pepperl+Fuchs ICE3 IO-Link masters over Modbus/TCP.
- Supports physical IO-Link ports `1..8`.
- Performs IO-Link ISDU read/write operations using the ICE3 Modbus register interface.
- Configures Balluff BCM0003 BLOB data collection.
- Supports Balluff BLOB transfer commands:
  - `0xF0` — BLOB Abort
  - `0xF1` — BLOB Start
  - `0xF2` — BLOB Finish
- Supports BLOB types:
  - `rawX`, `rawY`, `rawZ`
  - `AmpSpecX`, `AmpSpecY`, `AmpSpecZ`
  - `EnvSpecX`, `EnvSpecY`, `EnvSpecZ`
- Reads the full Balluff data-provider status map from index `8607`.
- Exports each BLOB to a unique timestamped CSV file.
- Includes recovery logic for stuck or interrupted BLOB transfers.
- Supports sequential multi-port runs from one script.
- Adds logging to file and console for production troubleshooting.

---

## Hardware Assumptions

This project assumes the following setup:

```text
PC running Python
   |
   | Modbus/TCP, port 502
   v
Pepperl+Fuchs ICE3 IO-Link Master
   |
   | IO-Link physical port 1..8
   v
Balluff BCM0003 condition monitoring sensor
```

Before running the scripts, confirm:

1. The PC can ping the ICE3 master IP address.
2. Modbus/TCP is enabled on the ICE3 master.
3. The Balluff BCM0003 is connected to the expected IO-Link port.
4. The ICE3 port is configured for IO-Link operation.
5. Firewalls allow TCP communication to the ICE3 Modbus/TCP port, normally `502`.

---

## Project Structure

```text
.
├── ModbusClientWrapper.py      # Thin pyModbusTCP wrapper with read/write success handling
├── modbus_ISDU.py              # ICE3 ISDU read/write client over Modbus/TCP
├── Balluff_blob_functions.py   # Balluff BCM0003 BLOB indexes, commands, parsers, CSV helpers
├── blob_state_machine.py       # Acquisition and BLOB transfer state machine
├── run_multiple_ports.py       # Production runner for sequential IO-Link port collection
└── README.md
```

### File Responsibilities

| File | Purpose |
|---|---|
| `ModbusClientWrapper.py` | Handles low-level Modbus holding-register reads/writes using `pyModbusTCP`. |
| `modbus_ISDU.py` | Builds ICE3 ISDU command frames, writes request registers, polls response registers, and returns structured `ISDUResponse` objects. |
| `Balluff_blob_functions.py` | Contains Balluff BCM0003 constants, BLOB command helpers, configuration writes, status reads, restart/abort/finish helpers, and CSV parsers. |
| `blob_state_machine.py` | Runs the full BLOB acquisition/transfer sequence with timeout handling, packet-counter checks, cleanup, and CSV export. |
| `run_multiple_ports.py` | Runs the state machine sequentially for one or more ICE3 physical IO-Link ports. |

---

## Python Requirements

Recommended Python version:

```text
Python 3.10+
```

Install dependencies:

```bash
pip install pyModbusTCP python-dotenv
```

`python-dotenv` is optional but recommended. Without it, environment variables must be set manually in the terminal.

---

## Environment Configuration

Create a `.env` file in the same folder as the scripts:

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

### Environment Variables

| Variable | Default | Description |
|---|---:|---|
| `ICE_HOST` | `192.168.137.21` | IP address of the Pepperl+Fuchs ICE3 master. |
| `ICE_TCP_PORT` | `502` | Modbus/TCP port. Usually `502`. |
| `ICE_UNIT_ID` | `1` | Modbus unit ID. Usually `1`. |
| `ICE_IOL_PORTS` | `1,2,3` | Comma-separated physical IO-Link ports to run sequentially. |
| `BLOB_TIMEOUT_S` | `180` | Timeout for acquisition and BLOB transfer states. |
| `BLOB_POLL_S` | `0.2` | Poll interval for Balluff data-provider status. |
| `BLOB_PACKET_POLL_S` | `0.05` | Poll interval while reading BLOB packets. |
| `BLOB_LOG_DIR` | `blob_logs` | Folder for run logs. |
| `LOG_LEVEL` | `INFO` | Python logging level. |

### Windows PowerShell Alternative

```powershell
$env:ICE_HOST="192.168.137.21"
$env:ICE_IOL_PORTS="1,2,3"
python run_multiple_ports.py
```

---

## Quick Start

### 1. Test one IO-Link port first

Use only one port until the setup is confirmed:

```powershell
$env:ICE_IOL_PORTS="1"
python run_multiple_ports.py
```

### 2. Run ports 1, 2, and 3 sequentially

```powershell
$env:ICE_IOL_PORTS="1,2,3"
python run_multiple_ports.py
```

The runner intentionally processes ports **sequentially**, not in parallel. The project uses a shared active ISDU client, so parallel port execution is not recommended without refactoring to instance-based clients.

---

## Default Acquisition Behavior

`run_multiple_ports.py` currently collects raw acceleration data for all three axes:

```python
blob_types = ["rawX", "rawY", "rawZ"]
dptg_value = 2      # Data provider triggered by ISDU
radptm_value = 0    # Raw acceleration starts after trigger
dcas_value = 6      # X, Y, Z axes
dct_value = 0       # Raw acceleration data only
```

This is the recommended first production test because raw acceleration BLOBs are simpler to validate than spectrum BLOBs.

---

## Output Files

CSV files are saved under a port-specific folder:

```text
blob_csv/
└── port_1/
    ├── rawX_20260425_151533_522.csv
    ├── rawY_20260425_151534_017.csv
    └── rawZ_20260425_151534_481.csv
```

Log files are saved under:

```text
blob_logs/
└── blob_run_YYYYMMDD_HHMMSS.log
```

---

## CSV Format

### Raw Acceleration CSV

Raw acceleration BLOBs export these columns:

| Column | Description |
|---|---|
| `timestamp_ms` | Trigger timestamp from the Balluff payload. |
| `axis` | `X`, `Y`, or `Z`. |
| `sample_index` | Sample number within the BLOB payload. |
| `raw_int16` | Signed raw acceleration value. |
| `accel_mg` | Acceleration in milligravity. Calculated as `raw_int16 * 0.488`. |
| `accel_g` | Acceleration in g. |

### Spectrum CSV

Amplitude and envelope spectrum BLOBs export:

| Column | Description |
|---|---|
| `timestamp_ms` | Trigger timestamp from the Balluff payload. |
| `axis` | `X`, `Y`, or `Z`. |
| `spectrum_type` | `amplitude` or `envelope`. |
| `bin_index` | Spectrum bin number. |
| `value_g` | FLOAT32 spectrum value in g. |
| `frequency_hz` | Optional, only added if `frequency_resolution_hz` is provided. |

---

## Running from Python Code

### Single BLOB, one port

```python
from Balluff_blob_functions import set_ice_isdu_client
from blob_state_machine import run_blob_state_machine

set_ice_isdu_client(
    host="192.168.137.21",
    iol_port=1,
    tcp_port=502,
    unit_id=1,
)

result = run_blob_state_machine(
    blob_type="rawX",
    timeout_s=180.0,
    poll_s=0.2,
    packet_poll_s=0.05,
    DPTG_value=2,
    RADPTM_value=0,
    DCAS_value=0,
    DCT_value=0,
    save_csv=True,
    output_dir="blob_csv/port_1_test",
)

print(result.success)
print(result.csv_path)
```

### Raw X/Y/Z on one port

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
    restart_before_config=True,
    cleanup_before_acquisition=True,
)

for blob_type, transfer in result.results.items():
    print(blob_type, transfer.success, transfer.csv_path)
```

### Transfer additional BLOBs from already-ready data

After one acquisition has already completed and the Balluff status is `3`, you can fetch additional BLOBs without rerunning the full acquisition:

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

## Balluff BCM0003 BLOB Configuration Reference

### Data Provider Configuration: Index `8603 / 0x219B`

| Subindex | Name | Common Value |
|---:|---|---:|
| `1` | `DPTG` — Data provider trigger source | `2` = ISDU trigger |
| `2` | `RADPTM` — Raw acceleration trigger mode | `0` = starts after trigger |
| `3` | `DCAS` — Axis selection | `6` = X, Y, Z |
| `4` | `DCT` — Data collection target | `0` = raw only |

### Trigger: Index `8605 / 0x219D`

Writes `1` to trigger data collection when `DPTG=2`.

### Restart Feature: Index `8606 / 0x219E`

Used before a new run to recover from old or stuck data-provider states.

### Status: Index `8607 / 0x219F`

Status values:

| Value | Meaning |
|---:|---|
| `0` | Data collection disabled |
| `1` | Waiting for trigger |
| `2` | Data provider preparing data |
| `3` | Data ready for BLOB transfer |

Status subindex mapping:

| BLOB Type | Status Byte Position |
|---|---:|
| `rawX` | `1` |
| `rawY` | `2` |
| `rawZ` | `3` |
| `AmpSpecX` | `4` |
| `AmpSpecY` | `5` |
| `AmpSpecZ` | `6` |
| `EnvSpecX` | `7` |
| `EnvSpecY` | `8` |
| `EnvSpecZ` | `9` |

---

## BLOB Type IDs

Positive IDs are used for parsing and export labels. Negative IDs are used in the `BLOB_Start` command when reading from the device.

| BLOB Type | Positive ID | Read ID Sent in BLOB_Start |
|---|---:|---:|
| `rawX` | `4096` | `-4096` |
| `rawY` | `4097` | `-4097` |
| `rawZ` | `4098` | `-4098` |
| `AmpSpecX` | `4099` | `-4099` |
| `AmpSpecY` | `4100` | `-4100` |
| `AmpSpecZ` | `4101` | `-4101` |
| `EnvSpecX` | `4102` | `-4102` |
| `EnvSpecY` | `4103` | `-4103` |
| `EnvSpecZ` | `4104` | `-4104` |

Example BLOB start payload for `rawX`:

```text
F1 F0 00
```

Where:

```text
F1    = BLOB_Start
F0 00 = -4096 as signed INT16 big-endian
```

---

## ICE3 ISDU Register Addressing

The ISDU client uses the following ICE3 Modbus register pattern:

```text
response_addr = iol_port * 1000 + 100
request_addr  = iol_port * 1000 + 300
```

For example, physical IO-Link port `1` uses:

```text
response_addr = 1100
request_addr  = 1300
```

The project defaults to base-0 Modbus addressing because `pyModbusTCP` uses base-0 addressing in this setup.

---

## Error Recovery Behavior

The production state machine includes recovery logic:

1. Before acquisition, it can send a best-effort `BLOB_Abort`.
2. Before configuration, it can restart the Balluff raw-data feature.
3. If a BLOB transfer fails, it can send a cleanup command using `BLOB_Abort`.
4. The multi-port runner catches exceptions per port, logs the failure, and continues to the next port.

These defaults are controlled by:

```python
restart_before_config=True
cleanup_before_acquisition=True
cleanup_on_error=True
continue_on_blob_error=False
```

For normal production use, keep these safety options enabled.

---

## Important Operational Notes

### Do not read `BLOB_CH` casually

Reading Balluff `BLOB_CH` index `50` advances the BLOB stream. During an active transfer, every read may consume the next packet. Avoid manual reads while the state machine is running.

### Keep write-response polling disabled unless proven stable

The project defaults to:

```python
wait_for_write_responses=False
```

This is intentional. In testing, ICE/ISDU writes may succeed even when waiting for the write response times out. Reads still wait for proper ISDU responses.

### Run ports sequentially

The active ISDU client is switched by `set_ice_isdu_client(...)`. This is safe for sequential testing but not for multithreaded parallel polling.

---

## Troubleshooting

### Cannot connect to ICE3

Check:

- Correct `ICE_HOST` IP address.
- PC network adapter is on the same subnet.
- Modbus/TCP is enabled on ICE3.
- Port `502` is not blocked by firewall.
- You can ping the ICE3 master.

### Timeout waiting for status `1`

Possible causes:

- Balluff sensor not connected to the selected IO-Link port.
- Wrong IO-Link port number.
- Sensor not in IO-Link mode.
- Feature did not restart cleanly.
- Incorrect configuration values.

Try:

```python
from Balluff_blob_functions import Restart_Raw_Data_Feature, Read_Blob_Status_Map, Format_Blob_Status_Map

Restart_Raw_Data_Feature(wait_for_response=False)
print(Format_Blob_Status_Map(Read_Blob_Status_Map()))
```

### Timeout during BLOB transfer

Possible causes:

- BLOB transfer was interrupted.
- Another script/manual read consumed `BLOB_CH` packets.
- Packet polling is too fast or too slow for the setup.
- Sensor returned an unexpected marker.

Try:

```python
from blob_state_machine import cleanup_active_blob_transfer

cleanup_active_blob_transfer(strategy="abort", quiet=False)
```

Then rerun the acquisition.

### CSV parser error: invalid payload length

Possible causes:

- Packet loss or interrupted BLOB transfer.
- Wrong BLOB type selected.
- `include_0x30_payload` may need adjustment depending on observed device behavior.
- Endianness setting may be wrong.

Start with `rawX` only before testing all axes.

---

## Recommended Validation Sequence

Use this sequence when commissioning a new sensor or ICE3 port:

1. Confirm Modbus/TCP connectivity to the ICE3 master.
2. Read a known simple ISDU parameter from the Balluff device.
3. Read Balluff BLOB status map.
4. Run `rawX` only.
5. Run `rawX`, `rawY`, `rawZ` on one port.
6. Run multiple ports sequentially.
7. Test spectrum BLOBs only after raw BLOBs are stable.

---

## Disclaimer

This software writes configuration and trigger commands to an IO-Link device. Review the configuration values before use, test on a safe bench setup first, and do not run automated collection against production machinery until the behavior has been validated for your exact ICE3, Balluff BCM0003, and network setup.
