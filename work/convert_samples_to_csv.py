#!/usr/bin/env python3

import argparse
import csv
import os
import struct
import sys


# Must match the C++ binary layout exactly:
#
# uint64_t system_time_us   -> Q
# char gps_time[32]         -> 32s
# double latitude           -> d
# double longitude          -> d
# double altitude           -> d
# float rx_dbfs             -> f
#
# "<" means little-endian with no alignment padding.
RECORD_FORMAT = "<Q32sdddf"
RECORD_SIZE = struct.calcsize(RECORD_FORMAT)


def decode_gps_time(raw_gps_time: bytes) -> str:
    """
    Remove trailing null bytes from the fixed-size C string
    and decode it as UTF-8.
    """
    return raw_gps_time.split(b"\0", 1)[0].decode(
        "utf-8",
        errors="replace",
    )


def convert_binary_to_csv(input_path: str, output_path: str) -> int:
    if not os.path.isfile(input_path):
        raise FileNotFoundError(
            f"Input file does not exist: {input_path}"
        )

    file_size = os.path.getsize(input_path)

    if file_size == 0:
        raise ValueError("Input file is empty.")

    complete_records = file_size // RECORD_SIZE
    remaining_bytes = file_size % RECORD_SIZE

    if remaining_bytes:
        print(
            f"Warning: input file contains {remaining_bytes} "
            "trailing bytes that do not form a complete record.",
            file=sys.stderr,
        )

    output_directory = os.path.dirname(
        os.path.abspath(output_path)
    )

    os.makedirs(output_directory, exist_ok=True)

    records_written = 0

    with open(input_path, "rb") as binary_file, open(
        output_path,
        "w",
        newline="",
        encoding="utf-8",
    ) as csv_file:

        writer = csv.writer(csv_file)

        writer.writerow([
            "measurement",
            "system_time_us",
            "system_time_seconds",
            "gps_time",
            "latitude",
            "longitude",
            "altitude",
            "rx_dbfs",
        ])

        measurement_index = 0

        while True:
            record_data = binary_file.read(RECORD_SIZE)

            if not record_data:
                break

            if len(record_data) != RECORD_SIZE:
                print(
                    "Warning: ignoring incomplete final record.",
                    file=sys.stderr,
                )
                break

            (
                system_time_us,
                raw_gps_time,
                latitude,
                longitude,
                altitude,
                rx_dbfs,
            ) = struct.unpack(RECORD_FORMAT, record_data)

            gps_time = decode_gps_time(raw_gps_time)

            writer.writerow([
                measurement_index,
                system_time_us,
                f"{system_time_us / 1_000_000.0:.6f}",
                gps_time,
                f"{latitude:.10f}",
                f"{longitude:.10f}",
                f"{altitude:.3f}",
                f"{rx_dbfs:.3f}",
            ])

            measurement_index += 1
            records_written += 1

    print(f"Record size: {RECORD_SIZE} bytes")
    print(f"Input size: {file_size} bytes")
    print(f"Complete records found: {complete_records}")
    print(f"CSV records written: {records_written}")
    print(f"Output file: {output_path}")

    return records_written


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Convert RFNoC Combined Mode binary "
            "measurements to CSV."
        )
    )

    parser.add_argument(
        "--input",
        required=True,
        help="Input binary .dat file",
    )

    parser.add_argument(
        "--output",
        help=(
            "Output CSV path. By default, the CSV is "
            "created next to the input file."
        ),
    )

    args = parser.parse_args()

    input_path = os.path.abspath(
        os.path.expanduser(args.input)
    )

    if args.output:
        output_path = os.path.abspath(
            os.path.expanduser(args.output)
        )
    
    else:
        input_directory = os.path.dirname(input_path)
        input_filename = os.path.basename(input_path)
        input_stem, _ = os.path.splitext(input_filename)
        
        output_directory = os.path.join(input_directory,"csv_output",)

        output_path = os.path.join(
            output_directory,
            f"{input_stem}_measurements.csv",
        )

    try:
        convert_binary_to_csv(
            input_path,
            output_path,
        )
    except Exception as error:
        print(
            f"Conversion failed: {error}",
            file=sys.stderr,
        )
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
