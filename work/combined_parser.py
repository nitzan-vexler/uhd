#!/usr/bin/env python3
"""Parse packed Combined Mode measurement records into CSV.
Current 72-byte format: <q32sdddd.
The first field is USRP device-time microseconds, NOT necessarily Unix UTC.
"""
import argparse
import csv
import os
import struct

RECORD = struct.Struct('<q32sdddd')

def convert(input_path, output_path):
    fmt = RECORD
    size = os.path.getsize(input_path)
    if size % fmt.size:
        raise ValueError(f'Input size {size} is not a multiple of {fmt.size} bytes. Expected the current 72-byte record format.')
    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    count = 0
    with open(input_path, 'rb') as source, open(output_path, 'w', newline='', encoding='utf-8') as dest:
        writer = csv.writer(dest)
        writer.writerow(['device_timestamp_us', 'gps_fix_utc', 'latitude', 'longitude', 'altitude_m', 'rx_dbfs'])
        while True:
            raw = source.read(fmt.size)
            if not raw:
                break
            timestamp, gps, lat, lon, alt, dbfs = fmt.unpack(raw)
            writer.writerow([timestamp, gps.split(b'\0', 1)[0].decode('utf-8', errors='replace'), lat, lon, alt, dbfs])
            count += 1
    return count

if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input', required=True)
    p.add_argument('--output', required=True)
    a = p.parse_args()
    print(f'Converted {convert(a.input, a.output)} records to {a.output}')
