import struct
import csv
import os
from datetime import datetime, timedelta

# Configuration
BINARY_FILE = "usrp_samples.dat"  # Binary file name
OUTPUT_FOLDER = "csv_output"  # Folder for CSV files
TIMESTAMP_SIZE = 8  # int64_t (8 bytes)
GPS_TIME_SIZE = 20  # Fixed-size GPS time string (20 bytes)
LATITUDE_SIZE = 8  # double (8 bytes)
LONGITUDE_SIZE = 8  # double (8 bytes)
ALTITUDE_SIZE = 8  # double (8 bytes)
AMPLITUDE_SIZE = 8  # double (8 bytes)
ROWS_PER_FILE = 1000000  # Max rows per CSV file
UTC_OFFSET_HOURS = 2  # Adjust to UTC+2

def read_samples_binary_to_csv(binary_file, output_folder, rows_per_file):
    """Reads the binary file and converts it to a CSV file including GPS data"""
    os.makedirs(output_folder, exist_ok=True)  # Ensure output directory exists

    file_index = 1  # File numbering
    row_count = 0
    prev_timestamp = None  # Store previous timestamp to calculate time difference

    csv_file_path = os.path.join(output_folder, f"output_{file_index}.csv")

    with open(binary_file, "rb") as bin_file:
        csv_file = open(csv_file_path, "w", newline="")
        writer = csv.writer(csv_file)
        writer.writerow([
            "System Time (µs)", "Parsed System Time (UTC+2)", "GPS Time",
            "Time Diff (µs)", "Latitude", "Longitude", "Altitude", "Amplitude (dBm)"
        ])

        while True:
            # Read timestamp (int64_t)
            timestamp_bytes = bin_file.read(TIMESTAMP_SIZE)
            if not timestamp_bytes:
                break  # End of file

            # Read GPS time (fixed 20-byte string)
            gps_time_bytes = bin_file.read(GPS_TIME_SIZE)
            if not gps_time_bytes:
                break  # End of file
            gps_time = gps_time_bytes.decode("utf-8").strip("\x00")  # Decode and remove null terminators

            # Read latitude (double)
            latitude_bytes = bin_file.read(LATITUDE_SIZE)
            if not latitude_bytes:
                break  # End of file

            # Read longitude (double)
            longitude_bytes = bin_file.read(LONGITUDE_SIZE)
            if not longitude_bytes:
                break  # End of file

            # Read altitude (double)
            altitude_bytes = bin_file.read(ALTITUDE_SIZE)
            if not altitude_bytes:
                break  # End of file

            # Read amplitude (double)
            amplitude_bytes = bin_file.read(AMPLITUDE_SIZE)
            if not amplitude_bytes:
                break  # End of file

            # Unpack timestamp (Little-endian int64_t)
            timestamp = struct.unpack("<q", timestamp_bytes)[0]

            # Compute time difference from previous sample
            time_diff = timestamp - prev_timestamp if prev_timestamp is not None else 0
            prev_timestamp = timestamp  # Update previous timestamp

            # Convert timestamp (system time) from microseconds to readable format
            parsed_system_time = datetime.utcfromtimestamp(timestamp / 1_000_000.0) + timedelta(hours=UTC_OFFSET_HOURS)
            parsed_system_time_str = parsed_system_time.strftime("%Y-%m-%d %H:%M:%S.%f")  # Keep microseconds

            # Unpack values (Little-endian doubles)
            latitude = struct.unpack("<d", latitude_bytes)[0]
            longitude = struct.unpack("<d", longitude_bytes)[0]
            altitude = struct.unpack("<d", altitude_bytes)[0]
            amplitude = struct.unpack("<d", amplitude_bytes)[0]

            # Handle missing GPS values
            if abs(latitude) > 90:
                latitude = "No Latitude"
            if abs(longitude) > 180:
                longitude = "No Longitude"
            if altitude < -500 or altitude > 9000:
                altitude = "No Altitude"

            # Write row to CSV
            writer.writerow([timestamp, parsed_system_time_str, gps_time, time_diff, latitude, longitude, altitude, amplitude])
            row_count += 1

            # If max row count reached, create a new file
            if row_count >= rows_per_file:
                csv_file.close()
                file_index += 1
                row_count = 0  # Reset row counter
                csv_file_path = os.path.join(output_folder, f"output_{file_index}.csv")
                csv_file = open(csv_file_path, "w", newline="")
                writer = csv.writer(csv_file)
                writer.writerow([
                    "System Time (µs)", "Parsed System Time (UTC+2)", "GPS Time",
                    "Time Diff (µs)", "Latitude", "Longitude", "Altitude", "Amplitude (dBm)"
                ])

        # Close last file
        csv_file.close()

    print(f"Conversion complete! {file_index} CSV files saved in '{output_folder}'.")

if __name__ == "__main__":
    read_samples_binary_to_csv(BINARY_FILE, OUTPUT_FOLDER, ROWS_PER_FILE)

