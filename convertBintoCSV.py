from pathlib import Path
import csv

INPUT_FILE = "blob_payload.bin"
OUTPUT_FILE = "g_values_from_bin.csv"

SCALE = 1969.3568
OFFSET = 50.0

payload = Path(INPUT_FILE).read_bytes()

print(f"Read {len(payload)} bytes from {INPUT_FILE}")

usable_len = len(payload) - (len(payload) % 4)

if usable_len != len(payload):
    print(f"Warning: ignoring {len(payload) - usable_len} leftover byte(s)")

g_values = []

for i in range(0, usable_len, 4):
    raw_bytes = payload[i:i + 4]

    raw_value = int.from_bytes(
        raw_bytes,
        byteorder="big",
        signed=False,
    )

    g_value = (raw_value / SCALE) - OFFSET
    g_values.append(g_value)

with open(OUTPUT_FILE, "w", newline="") as file:
    writer = csv.writer(file)
    writer.writerow(["sample_index", "raw_value", "g_value"])

    for sample_index in range(len(g_values)):
        raw_bytes = payload[sample_index * 4:sample_index * 4 + 4]
        raw_value = int.from_bytes(raw_bytes, byteorder="big", signed=False)
        writer.writerow([sample_index, raw_value, g_values[sample_index]])

print(f"Wrote {len(g_values)} samples to {OUTPUT_FILE}")
print("First 10 g-values:")

for value in g_values[:10]:
    print(value)