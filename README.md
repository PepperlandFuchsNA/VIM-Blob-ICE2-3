# ICE3 Balluff BCM0003 BLOB Data Collector

Production-oriented Python tooling for collecting vibration BLOB data from a **Balluff BCM0003** condition monitoring sensor connected to a **Pepperl+Fuchs ICE3 IO-Link master** over **Modbus/TCP**.

The project configures the Balluff data provider through IO-Link ISDU, triggers a capture, transfers one or more BLOB payloads, validates the transfer, parses the payload, and exports timestamped CSV files.

---

## What this project does

- Communicates with the ICE3 IO-Link master over Modbus/TCP.
- Reads and writes IO-Link ISDU parameters.
- Configures Balluff BCM data-provider settings.
- Starts a capture through ISDU trigger mode.
- Transfers BLOB payloads through BLOB channel index `50`.
- Supports raw acceleration and amplitude/envelope spectrum BLOBs.
- Exports timestamped CSV files per BLOB and per IO-Link port.
- Supports sequential multi-port collection.
- Supports parallel multi-port collection using thread-local ISDU clients.
- Adds recovery handling for stuck or interrupted BLOB transfers.
- Adds stricter transfer diagnostics: markers seen, packet counter mismatches, payload length information, final marker, and CRC/end packet capture.

---

## Project structure

```text
.
├── ModbusClientWrapper.py      # pyModbusTCP wrapper
├── modbus_ISDU.py              # ICE3 ISDU client and typed ISDUResponse
├── Balluff_blob_functions.py   # Balluff BCM commands, constants, and payload parsers
├── blob_state_machine.py       # Status-aware acquisition and strict BLOB transfer logic
├── run_multiple_ports.py       # Sequential multi-port runner
├── run_parallel_ports.py       # Simultaneous threaded multi-port runner
├── requirements.txt            # Minimal runtime dependencies
├── .env.example                # Example environment configuration
└── README.md
```

---

## Requirements

- Python 3.10 or newer
- Pepperl+Fuchs ICE3 IO-Link master with Modbus/TCP enabled
- Balluff BCM0003 connected to one or more IO-Link ports
- Network access to TCP port `502` on the ICE3 master

Install dependencies:

```bash
pip install -r requirements.txt
```

---

## Configuration

Copy the example environment file:

```bash
copy .env.example .env
```

On macOS/Linux:

```bash
cp .env.example .env
```

Edit `.env` for your setup:

```env
ICE_HOST=192.168.137.21
ICE_TCP_PORT=502
ICE_UNIT_ID=1
ICE_IOL_PORTS=1

BLOB_TYPES=rawX
DPTG_VALUE=2
RADPTM_VALUE=0
DCAS_VALUE=6
DCT_VALUE=0

BLOB_TIMEOUT_S=180
BLOB_POLL_S=0.2
BLOB_PACKET_POLL_S=0.05

SAVE_CSV=true
BLOB_OUTPUT_ROOT=blob_csv
WAIT_FOR_WRITE_RESPONSES=false
```

Use `ICE_IOL_PORTS`, not `PORTS_TO_TEST`.

---

## Recommended commissioning sequence

Start small and scale up only after each step is stable.

### 1. One port, raw X only

```powershell
$env:ICE_IOL_PORTS="1"
$env:BLOB_TYPES="rawX"
python run_multiple_ports.py
```

### 2. One port, raw X/Y/Z

```powershell
$env:ICE_IOL_PORTS="1"
$env:BLOB_TYPES="rawX,rawY,rawZ"
python run_multiple_ports.py
```

### 3. Two ports sequentially

```powershell
$env:ICE_IOL_PORTS="1,2"
$env:BLOB_TYPES="rawX,rawY,rawZ"
python run_multiple_ports.py
```

### 4. Two ports in parallel

```powershell
$env:ICE_IOL_PORTS="1,2"
$env:PARALLEL_MAX_WORKERS="2"
python run_parallel_ports.py
```

Only after this is stable should you try more ports or all 8 ports.

---

## Status-aware acquisition behavior

The state machine now checks the current Balluff data-provider status before deciding what to do.

```text
0 = data collection disabled       -> configure, wait for 1, trigger, wait for 3
1 = waiting for trigger            -> trigger, wait for 3
2 = preparing data                 -> wait for 3
3 = data ready for BLOB transfer   -> reuse data or force a new capture
```

Useful options:

```env
FORCE_RECONFIGURE=false
REUSE_READY_DATA=true
```

Use `FORCE_RECONFIGURE=true` when you want a brand-new capture every run.

---

## Transfer validation behavior

The BLOB transfer logic now records and validates more information:

- `0x10` info packet seen
- `0x20` to `0x2F` data packet counters
- `0x30` final data packet marker
- `0x40` CRC/end packet
- markers seen during transfer
- packet counter mismatches
- expected and actual payload length when known
- validation warnings

Default validation options:

```env
STRICT_TRANSFER=true
VALIDATE_PACKET_COUNTER=true
VALIDATE_EXPECTED_LENGTH=true
TRIM_TO_EXPECTED_LENGTH=true
```

Important: expected-length validation is only enforced when the length is trusted. A length is trusted when you provide either:

```env
BLOB_INFO_LENGTH_OFFSET=<confirmed byte offset in the 0x10 packet>
```

or when the caller provides an explicit expected payload length in code.

Without a confirmed offset, the code records a best-effort length estimate for diagnostics but does not use that estimate to fail or trim the transfer.

---

## Sequential collection

Run:

```bash
python run_multiple_ports.py
```

This is the safest production/commissioning mode because each IO-Link port is handled one at a time.

---

## Parallel collection

Run:

```bash
python run_parallel_ports.py
```

The parallel runner uses a thread-local ISDU proxy so each worker thread gets its own client. This avoids unsafe global-client switching during simultaneous collection.

Start with:

```env
ICE_IOL_PORTS=1,2
PARALLEL_MAX_WORKERS=2
```

Then increase one port at a time.

---

## Output

CSV files are saved under the selected output root:

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

## Troubleshooting

### First run works, second run fails

This usually means the sensor is no longer in status `0`. The status-aware state machine handles this by accepting status `1`, `2`, or `3` as valid entry states.

For repeated testing, use:

```env
REUSE_READY_DATA=true
FORCE_RECONFIGURE=false
```

For a brand-new capture every run, use:

```env
REUSE_READY_DATA=false
FORCE_RECONFIGURE=true
```

### Timeout waiting for status `1`

Check:

- correct IO-Link port
- Balluff sensor connected and online
- ICE3 port in IO-Link mode
- `DPTG_VALUE`, `RADPTM_VALUE`, `DCAS_VALUE`, `DCT_VALUE`
- whether cleanup/restart commands are enabled

### Timeout during transfer

Run cleanup and retry:

```python
from blob_state_machine import cleanup_active_blob_transfer

cleanup_active_blob_transfer(strategy="abort", quiet=False)
```

### Packet counter mismatch

With `STRICT_TRANSFER=true`, this fails the transfer instead of saving questionable CSV data. For debugging only, you can temporarily set:

```env
STRICT_TRANSFER=false
```

### Parallel mode unstable

Reduce workers:

```env
ICE_IOL_PORTS=1,2
PARALLEL_MAX_WORKERS=2
```

Then increase one port at a time.

---

## Safety

This project writes configuration and trigger commands to an IO-Link sensor. Validate all behavior on a bench setup before using it near active machinery or production equipment.
