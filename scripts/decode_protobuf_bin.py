import struct
import csv
import time
import sys

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
    print(f"Decoded {len(messages)} {protobuf_class.__name__} messages in {elapsed_time:.6f} seconds")

    return messages

def write_protobuf_messages_to_csv(messages, csv_output_path):
    start_time = time.time()

    with open(csv_output_path, 'w', newline='') as csv_file:
        # Create a CSV writer
        csv_writer = csv.writer(csv_file)

        # Write the CSV header based on the protobuf class fields
        csv_writer.writerow(messages[0].DESCRIPTOR.fields_by_name.keys())

        for message in messages:
            # Write the values to the CSV file
            csv_writer.writerow([getattr(message, field.name) for field in message.DESCRIPTOR.fields])

    end_time = time.time()
    elapsed_time = end_time - start_time
    print(f"Written {len(messages)} elements to CSV in {elapsed_time:.6f} seconds")

if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("Usage: python decode_protobuf_bin.py [sensor | gps] input_bin output_csv")
        sys.exit(1)

    frame_kind = sys.argv[1]
    input_bin_path = sys.argv[2]
    output_csv_path = sys.argv[3]

    from proto.sensor_pb2 import SensorFrame
    from proto.gps_pb2 import GpsFrame

    if frame_kind == "sensor":
        protobuf_class = SensorFrame
    elif frame_kind == "gps":
        protobuf_class = GpsFrame
    else:
        print("Usage: python decode_protobuf_bin.py [sensor | gps] input_bin output_csv")
        sys.exit(1)

    # Decode the protobuf messages
    sensor_frames = decode_protobuf_file(input_bin_path, protobuf_class)

    # Write the decoded values to the CSV file
    write_protobuf_messages_to_csv(sensor_frames, output_csv_path)
