# VIM-Blob-ICE2-3
Starter code and Guide to acquire raw g values from VIM32PP-E7DC8-0RE-IO-1V1401 using Blob connected to ICE2/3 IO-link master and Modbus TCP

The settings for Blob being used for these runs are below.

![image](https://github.com/user-attachments/assets/52520290-51f8-4dbf-b86a-061ab18c45dc)

Results:
Few runs were made with VIM connected to a desktop fan. 
One with speed being very high
Other test with low speed
Last test with fan being turned off

![image](https://github.com/user-attachments/assets/424985cc-4e74-4a2e-a036-be4f6dc9d58d)

![image](https://github.com/user-attachments/assets/1d59a231-4488-4008-b2f4-a0681b550255)

![image](https://github.com/user-attachments/assets/8b981a9c-aeed-4608-bcde-6b03a05621e0)

![image](https://github.com/user-attachments/assets/8eee18f8-cad4-4654-8e1b-bdd10a847f30)

![image](https://github.com/user-attachments/assets/00db635c-21cc-48c7-a412-aee745ce648d)

![image](https://github.com/user-attachments/assets/c52207b2-282b-425c-8e4f-6de0a259ddf8)

## Safe BCM BLOB Transfer Strategy

The BCM reader treats `BLOB_CH` reads on index `50 / 0x0032` as stateful. A read request can advance the device/master BLOB state machine even when the ICE2/ICE3 response mailbox later reports `0x3001` and still shows stale payload bytes. Because of that, the production path never retries a failed `BLOB_CH` read by issuing another index-50 read inside the same active transfer.

Safe behavior in `balluff_bcm_blob_reader.py`:

- `0x2001` means the ICE mailbox reports a successful ISDU read response.
- `0x3001` means status nibble `3` with read type `1`; treat it as failed/not usable for BLOB data.
- Stale `0x3001` payload is never appended.
- Flow counters are strict; any duplicate, skipped, or unexpected flow aborts the transfer.
- `.bin` and `.csv` files are saved only after exact BLOB length and IO-Link BLOB CRC32 both pass.

Useful first diagnostics:

```powershell
python .\balluff_bcm_blob_reader.py --diagnose-blob-read --max-isdu-len 101 --blob-segment-gap-s 0.25 --blob-read-retries 0 --log-file blob_diag.log
python .\balluff_bcm_blob_reader.py --probe-lengths 201,101,51,33,17 --blob-segment-gap-s 0.25 --blob-read-retries 0
python .\balluff_bcm_blob_reader.py --probe-delays 0.05,0.1,0.25,0.5,1.0 --blob-read-retries 0
```

Use `--isdu-mode vim_compatible` only as a comparison test for the older VIM pacing model. It still rejects non-`0x2001` BLOB_CH responses and keeps strict flow/CRC validation.
