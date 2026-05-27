# BCM BLOB Transfer Troubleshooting

## What Changed

- Added a safe BLOB_CH transaction path that issues only one state-advancing read request per segment.
- Added response-mailbox polling after a `0x3001` result; polling reads registers only and does not advance the BLOB state machine.
- Made verified transfers strict: duplicate, stale, skipped, or unexpected flow counters abort immediately.
- Added timeline logging for BLOB_CH reads, including timestamps, control/status, response length, header, flow, payload hash, and stale-payload comparisons.
- Added no-save diagnostics and length/delay probes.

## Diagnostic Modes

- `--diagnose-blob-read`: resets BLOB state, starts a fresh recording, reads BLOB_Info, then attempts a short strict segment prefix. It logs Modbus and BLOB timeline details and aborts without saving.
- `--probe-lengths 201,101,51,33,17`: tests each max ISDU length with a clean BLOB_Start and strict first-segment prefix.
- `--probe-delays 0.05,0.1,0.25,0.5,1.0`: tests quiet gaps between stateful BLOB_CH reads, using length `101` unless combined with `--probe-lengths`.
- `--isdu-mode vim_compatible`: uses the older VIM one-write, fixed-delay, one-read pacing for comparison while still rejecting failed `0x3001` data.

## Interpreting 0x2001 and 0x3001

The ICE response control word is parsed as status in bits 12-15 and ISDU type in bits 0-3.

- `0x2001`: status `2` success, type `1` read. This can be accepted only if the response is fresh and matches BLOB_CH index/subindex.
- `0x3001`: status `3` failure, type `1` read. Do not parse or append its payload. In the observed BCM/ICE logs it can show stale bytes from the prior successful segment.

## Mailbox Format Used

The local VIM reference in git history uses the same ICE2/ICE3 Modbus mailbox format as the BCM script:

- ISDU read request: `[1, index, subindex, requested_length]`
- BLOB_CH read request for 200 bytes of segment data: `[1, 50, 0, 201]`
- ISDU write request: `[2, index, subindex, payload_length, data_words...]`
- Response register count: `4 + ceil(requested_length / 2)`

For BLOB_CH, length `201` means one BLOB header byte plus up to 200 body bytes.

## Recommended First Commands

```powershell
python .\balluff_bcm_blob_reader.py --diagnose-blob-read --max-isdu-len 101 --blob-segment-gap-s 0.25 --blob-read-retries 0 --log-file blob_diag.log
python .\balluff_bcm_blob_reader.py --probe-lengths 201,101,51,33,17 --blob-segment-gap-s 0.25 --blob-read-retries 0
python .\balluff_bcm_blob_reader.py --probe-delays 0.05,0.1,0.25,0.5,1.0 --blob-read-retries 0
```

If all probes fail, keep the log and report the last successful flow, expected next flow, last control/status, last response hash, and the first `0x3001` location printed by the script.
