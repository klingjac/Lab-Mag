import serial
import time
import socket
import os
import csv
import RPi.GPIO as GPIO
import threading

GPIO.setmode(GPIO.BCM)
gpio_pin = 27
GPIO.setup(gpio_pin, GPIO.OUT)

class DataProcessor:
    def __init__(self, max_file_size=10000):
        self.ser = serial.Serial(
            port='/dev/ttyAMA0',  # Change this according to connection methods, e.g. /dev/ttyUSB0
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1
        )

        self.x_scaling = 1.0 / 75.0
        self.y_scaling = 1.0 / 75.0
        self.z_scaling = 1.0 / 75.0

        # Initialize empty lists to store data
        self.x_data = None
        self.y_data_x = None
        self.y_data_y = None
        self.y_data_z = None

        self.t = threading.Thread(target=self.reading_thread)
        self.t.start()

        self.directory = "mag_data_37"
        if not os.path.exists(self.directory):
            os.makedirs(self.directory)
        self.directory = "mag_data_75"
        if not os.path.exists(self.directory):
            os.makedirs(self.directory)
        self.directory = "mag_data_18"
        if not os.path.exists(self.directory):
            os.makedirs(self.directory)

        self.directory = "mag_data_37"

        self.max_file_size = max_file_size
        self.current_file_writes = 0
        self.filename = None

        

        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_host = 'localhost' 
        self.udp_port = 5005 

        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_host = 'localhost'  
        self.tcp_port = 5006        
        self.tcp_socket.bind((self.tcp_host, self.tcp_port))
        self.tcp_socket.listen(1)

        self.tcp_thread = threading.Thread(target=self.handle_tcp_client)
        self.tcp_thread.start()

    def send_data_udp(self, data):
        message = f"{data[0]},{data[1]},{data[2]},{data[3]}"
        #print(message)
        self.udp_socket.sendto(message.encode(), (self.udp_host, self.udp_port))
    
    def uint24_to_int24(self, unsigned_int24):
        unsigned_int24 &= 0x00FFFFFF
        three_byte_sign_bit = 0x00800000
        signed_int24 = unsigned_int24
        if signed_int24 & three_byte_sign_bit:
            signed_int24 = (signed_int24 & ~three_byte_sign_bit) - three_byte_sign_bit
        return signed_int24

    def read_data(self):

        while not self.ser.in_waiting > 0:
            time.sleep(0.01)

        received_data = self.ser.read(9)
        received_integer_bytes = [int(byte) for byte in received_data]

        x_values = received_integer_bytes[0:3]
        y_values = received_integer_bytes[3:6]
        z_values = received_integer_bytes[6:9]

        x_mag_unsigned = int.from_bytes(x_values, "big")
        y_mag_unsigned = int.from_bytes(y_values, "big")
        z_mag_unsigned = int.from_bytes(z_values, "big")

        x_mag_int = self.uint24_to_int24(x_mag_unsigned)
        y_mag_int = self.uint24_to_int24(y_mag_unsigned)
        z_mag_int = self.uint24_to_int24(z_mag_unsigned)

        x_mag_value = x_mag_int * self.x_scaling
        y_mag_value = y_mag_int * self.y_scaling
        z_mag_value = z_mag_int * self.z_scaling

        current_time = time.time()
        
        data = [current_time, x_mag_value, y_mag_value, z_mag_value]
        self.write_to_csv(data)

        

        return current_time, x_mag_value, y_mag_value, z_mag_value

    def write_to_csv(self, data):
        
        if self.filename == None:
            self.generate_filename()

        if self.current_file_writes >= self.max_file_size:
            self.generate_filename()
            self.current_file_writes = 0
        
        write_header = not os.path.exists(self.filename)

        with open(self.filename, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            
            if write_header:
                writer.writerow(['Time unix', 'X-axis', 'Y-axis', 'Z-axis'])

            writer.writerow(data)

        self.current_file_writes += 1

    def generate_filename(self):
        current_time =  int(time.time())
        filename = f"{current_time}_mag_data.csv"
        filename = os.path.join(self.directory, filename)
        
        self.filename = filename
    
    def update_plot(self, readings):

        current_time = time.time()

        self.x_data = [current_time]
        self.y_data_x = [readings[0]]
        self.y_data_y = [readings[1]]
        self.y_data_z = [readings[2]]


    def reading_thread(self):
        while True:
            #print(f"in thread loop")
            try:
                data = self.read_data()
                #print(f"reading: {data}")
                self.send_data_udp(data)
                self.update_plot(data)
                time.sleep(0.01)
            except Exception as e:
                print(f"error: {e}")
    

    def set_frequency(self, freq):
        data_to_send = b''
        if freq == 18:
            data_to_send = b'18'
            print('18')
            self.filename = None
            self.directory = "mag_data_18"
        elif freq == 37:
            data_to_send = b'37'
            print('37')
            self.filename = None
            self.directory = "mag_data_37"
        elif freq == 75:
            data_to_send = b'75'
            print('75')
            self.filename = None
            self.directory = "mag_data_75"

        GPIO.output(gpio_pin, GPIO.HIGH)
        time.sleep(0.1)
        self.ser.write(data_to_send)
        time.sleep(0.1)
        GPIO.output(gpio_pin, GPIO.LOW)

    def handle_tcp_client(self):
        while True:
            # Accept a new connection
            conn, addr = self.tcp_socket.accept()
            print(f'Connected by {addr}')
            try:
                with conn:
                    while True:
                        # Receive data from the client
                        data = conn.recv(1024)
                        if not data:
                            break  # Exit loop if data is not received, i.e., client disconnects
                        freq = int(data.decode())
                        print(f"Received frequency: {freq}")
                        self.set_frequency(freq)
            except Exception as e:
                print(f"Connection error: {e}")


if __name__ == '__main__':
    data_processor = DataProcessor()