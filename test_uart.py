import serial
import time
import os
import csv

def uint24_to_int24(unsigned_int24):
    # Python does not actually have a 3-byte integer object. So it'll give us four bytes
    # This line ensures that the Most Significant byte is zeroed out
    unsigned_int24 &= 0x00FFFFFF

    # If this bit is set, the integer is actually a negative number
    three_byte_sign_bit = 0x00800000

    # Convert our unsigned int to a signed int
    signed_int24 = unsigned_int24
    if signed_int24 & three_byte_sign_bit:
        signed_int24 = (signed_int24 & ~three_byte_sign_bit) - three_byte_sign_bit

    return signed_int24

def write_to_csv(data, filename):
    
    # Check if the file exists to determine whether to write the header
    write_header = not os.path.exists(filename)

    with open(filename, 'a', newline='') as csvfile:
        writer = csv.writer(csvfile)
        
        # Write header if the file is being created
        if write_header:
            writer.writerow(['X-axis', 'Y-axis', 'Z-axis'])

        # Write data
        writer.writerow(data)

def generate_filename(dir):
    current_time =  int(time.time())
    filename = f"{current_time}_mag_data.csv"
    filename = os.path.join(dir, filename)
    return filename


# Define the serial port and baud rate
ser = serial.Serial(
  port='/dev/ttyAMA0', # Change this according to connection methods, e.g. /dev/ttyUSB0
  baudrate = 115200,
  parity=serial.PARITY_NONE,
  stopbits=serial.STOPBITS_ONE,
  bytesize=serial.EIGHTBITS,
  timeout=1
)

directory = "mag_data"

x_scaling = 1.0/75.0
y_scaling = 1.0/75.0
z_scaling = 1.0/75.0

data_counter = 0
max_data_points = 1000

if not os.path.exists(directory):
    os.makedirs(directory)

current_filename = generate_filename(directory)

while True:
    received_data = ser.read(9)
    
    received_integer_bytes = [int(byte) for byte in received_data]

    if data_counter > max_data_points:
        data_counter = 0
        current_filename = generate_filename(directory)

    for i in range(0, len(received_integer_bytes), 9):
        
        # Extract x-axis values (first 3 bytes)
        x_values = received_integer_bytes[i:i+3]
        x_mag_unsigned = int.from_bytes(x_values, "big")
        x_mag_int = uint24_to_int24(x_mag_unsigned)
        x_mag_value = x_mag_int * x_scaling

        # Extract y-axis values (next 3 bytes)
        y_values = received_integer_bytes[i+3:i+6]
        y_mag_unsigned = int.from_bytes(y_values, "big")
        y_mag_int = uint24_to_int24(y_mag_unsigned)
        y_mag_value = y_mag_int * y_scaling

        # Extract z-axis values (last 3 bytes)
        z_values = received_integer_bytes[i+6:i+9]
        z_mag_unsigned = int.from_bytes(z_values, "big")
        z_mag_int = uint24_to_int24(z_mag_unsigned)
        z_mag_value = z_mag_int * z_scaling

        mag_data_arr = [x_mag_value, y_mag_value, z_mag_value]
        print(f"data: {mag_data_arr}")

        write_to_csv(mag_data_arr, current_filename)

        data_counter += 1





ser.close()
