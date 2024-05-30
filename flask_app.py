from flask import Flask, render_template, jsonify, g
import serial
import time
import socket
import os
import csv
import matplotlib.pyplot as plt
import RPi.GPIO as GPIO
import threading
import logging



app = Flask(__name__)

# Initialize GPIO
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
        self.x_data = []
        self.y_data_x = []
        self.y_data_y = []
        self.y_data_z = []

        self.t = threading.Thread(target=self.reading_thread)
        self.t.start()

        # Set up the plot
        plt.figure(figsize=(10, 6))
        plt.xlabel('Time (s)')
        plt.ylabel('Value')
        plt.title('Real-time Magnetometer Data')

        self.directory = "mag_data_37"
        self.max_file_size = max_file_size
        self.current_file_writes = 0
        self.filename = None

        self.setup_udp_socket()
        self.setup_tcp_server()


        if not os.path.exists(self.directory):
            os.makedirs(self.directory)

        
        self.tcp_thread = threading.Thread(target=data_processor.handle_tcp_client)
        self.tcp_thread.start()


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
        #print(f"readings: {x_mag_value}, {y_mag_value}, {z_mag_value}")
        data = [x_mag_value, y_mag_value, z_mag_value]
        self.write_to_csv(data)

        return x_mag_value, y_mag_value, z_mag_value

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
                writer.writerow(['X-axis', 'Y-axis', 'Z-axis'])

            writer.writerow(data)

        self.current_file_writes += 1

    def generate_filename(self):
        current_time =  int(time.time())
        filename = f"{current_time}_mag_data.csv"
        filename = os.path.join(self.directory, filename)
        
        self.filename = filename
    
    def update_plot(self, readings):
        #Update underlying data arrays for the data processor

        current_time = time.time()

        # Append new data points to existing data lists
        self.x_data.append(current_time)
        self.y_data_x = [readings[0]]
        self.y_data_y = [readings[1]]
        self.y_data_z = [readings[2]]

        # if len(self.x_data) > 0:
        #     self.y_data_x.pop(0)
        #     self.y_data_y.pop(0)
        #     self.y_data_z.pop(0)
        #     self.x_data.pop(0)

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
        logging.info('a frequency button clicked')
        if freq == 18:
            data_to_send = b'18'
            logging.info('18')
        elif freq == 37:
            data_to_send = b'37'
            logging.info('37')
        elif freq == 75:
            data_to_send = b'75'
            logging.info('75')

        GPIO.output(gpio_pin, GPIO.HIGH)
        time.sleep(0.1)
        self.ser.write(data_to_send)
        time.sleep(0.1)
        GPIO.output(gpio_pin, GPIO.LOW)

    


@app.before_request
def before_request():
    if 'data_processor' not in g:
        g.data_processor = DataProcessor()

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/plot_data')
def plot_data():
    x_data = data_processor.x_data
    y_data_x = data_processor.y_data_x
    y_data_y = data_processor.y_data_y
    y_data_z = data_processor.y_data_z

    #print(y_data_z)
    
    return jsonify(x_data=x_data, y_data_x=y_data_x, y_data_y=y_data_y, y_data_z=y_data_z)


@app.route('/start')
def start():
    data_processor.start()
    return jsonify(success=True)

@app.route('/get_point')
def new_point():
    if len(data_processor.x_data) > 0:
        # index = len(data_processor.x_data) - 1
        # print(f"index: {index}")
        time = data_processor.x_data
        x_axis = data_processor.y_data_x
        y_axis = data_processor.y_data_y
        z_axis = data_processor.y_data_z
        print(f"data point: {y_axis}")
    else:
        time = [0]
        x_axis = [0]
        y_axis = [0]
        z_axis = [0]

    return jsonify(x_data=time,y_data_x=x_axis,y_data_y=y_axis,y_data_z=z_axis )

@app.route('/stop')
def stop():
    data_processor.stop()
    return jsonify(success=True)

@app.route('/set_frequency/<int:frequency>',  methods=['GET', 'POST'])
def set_frequency(frequency):
    data_processor.set_frequency(frequency)
    return jsonify(success=True)

if __name__ == '__main__':
    data_processor = DataProcessor()
    
    app.run(debug=True, host='0.0.0.0')