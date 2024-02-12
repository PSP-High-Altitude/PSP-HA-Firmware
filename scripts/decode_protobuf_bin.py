import struct
import time
import sys

import pandas as pd

def decode_protobuf_file(file_path, protobuf_class):
    messages = []

    start_time = time.time()

    with open(file_path, 'rb') as file:
        # Read and print the header
        header = file.read(64)
        print("Firmware specifier:", header.decode('utf-8'))

        while True:
            # Read the length byte
            length_byte = file.read(1)
            if not length_byte:
                break  # End of file

            # Unpack the length
            message_length = struct.unpack('B', length_byte)[0]

            # Read the message based on the length
            message_data = file.read(message_length)

            # Create an instance of the provided protobuf class and parse the message
            protobuf_message = protobuf_class()
            protobuf_message.ParseFromString(message_data)

            messages.append(protobuf_message)

    end_time = time.time()
    elapsed_time = end_time - start_time
    print(f"Decoded {len(messages)} elements in {elapsed_time:.6f} seconds")

    # Convert messages to DataFrame
    data = [
        [
            getattr(message, field.name) 
            for field in message.DESCRIPTOR.fields
        ]
        for message in messages
    ]
    column_names = [field.name for field in protobuf_class.DESCRIPTOR.fields]
    df = pd.DataFrame(data, columns=column_names)

    return df

def write_dataframe_to_csv(dataframe, csv_output_path):
    start_time = time.time()

    dataframe.to_csv(csv_output_path, index=False, float_format="%.3f")

    end_time = time.time()
    elapsed_time = end_time - start_time
    print(f"Written {len(dataframe)} elements to CSV in {elapsed_time:.6f} seconds")

def unpack_gps_validity_flags(dataframe):
    # Define the masks for each flag
    masks = {
        "date_valid": 1 << 0,
        "time_valid": 1 << 1,
        "time_resolved": 1 << 2,
        "fix_type": (0b111 << 3),
        "fix_valid": 1 << 8,
        "diff_used": 1 << 9,
        "psm_state": (0b111 << 10),
        "hdg_veh_valid": 1 << 14,
        "carrier_phase": (0b11 << 15),
        "invalid_llh": 1 << 19
    }

    # Create new columns for each flag
    for flag, mask in masks.items():
        # Extract bits using the mask
        dataframe[flag] = (dataframe['valid_flags'] & mask).apply(lambda x: x >> mask.bit_length() - bin(mask).count('1'))

    return dataframe

if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("Usage: python decode_protobuf_bin.py [sensor | gps | state] input_bin output_csv")
        sys.exit(1)

    frame_kind = sys.argv[1]
    input_bin_path = sys.argv[2]
    output_csv_path = sys.argv[3]

    if frame_kind == "sensor":
        from proto.sensor_pb2 import SensorFrame
        protobuf_class = SensorFrame
    elif frame_kind == "gps":
        from proto.gps_pb2 import GpsFrame
        protobuf_class = GpsFrame
    elif frame_kind == "state":
        from proto.state_pb2 import StateFrame
        protobuf_class = StateFrame
    else:
        print("Usage: python decode_protobuf_bin.py [sensor | gps | state] input_bin output_csv")
        sys.exit(1)

    # Decode the protobuf messages
    sensor_frames = decode_protobuf_file(input_bin_path, protobuf_class)

    # Do any required processing
    if frame_kind == "gps":
        sensor_frames = unpack_gps_validity_flags(sensor_frames)

    # Write the decoded values to the CSV file
    write_dataframe_to_csv(sensor_frames, output_csv_path)
