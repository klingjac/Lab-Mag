import serial
import time
import os
import csv
import matplotlib.pyplot as plt
import tkinter as tk
import RPi.GPIO as GPIO
import threading

GPIO.setmode(GPIO.BCM)
gpio_pin = 27
GPIO.setup(gpio_pin, GPIO.OUT)

class DataProcessor:
    def __init__(self, max_file_size=100):
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

        # Set up the plot
        plt.figure(figsize=(10, 6))
        plt.xlabel('Time (s)')
        plt.ylabel('Value')
        plt.title('Real-time Magnetometer Data')

        self.directory = "mag_data"
        self.max_file_size = max_file_size
        self.current_file_writes = 0
        self.filename = None

        if not os.path.exists(self.directory):
            os.makedirs(self.directory)

        self.gui_thread = threading.Thread(target = self.gui_sensor_manager)
        self.gui_thread.start()

    def uint24_to_int24(self, unsigned_int24):
        unsigned_int24 &= 0x00FFFFFF
        three_byte_sign_bit = 0x00800000
        signed_int24 = unsigned_int24
        if signed_int24 & three_byte_sign_bit:
            signed_int24 = (signed_int24 & ~three_byte_sign_bit) - three_byte_sign_bit
        return signed_int24

    def read_data(self):
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

        print(f"Data: {data}")

        return x_mag_value, y_mag_value, z_mag_value

    def update_plot(self, readings):

        current_time = time.time()

        # Append new data points to existing data lists
        self.x_data.append(current_time)
        self.y_data_x.append(readings[0])
        self.y_data_y.append(readings[1])
        self.y_data_z.append(readings[2])

        if len(self.x_data) >= 100:
            self.y_data_x.pop(0)
            self.y_data_y.pop(0)
            self.y_data_z.pop(0)
            self.x_data.pop(0)


        # Clear the plot and plot the updated data
        plt.clf()
        plt.plot(self.x_data, self.y_data_x, label='X-axis')
        plt.plot(self.x_data, self.y_data_y, label='Y-axis')
        plt.plot(self.x_data, self.y_data_z, label='Z-axis')
        plt.legend()
        plt.xlabel('Time (s)')
        plt.ylabel('Value')
        plt.title('Real-time Magnetometer Data')
        plt.grid(True)
        plt.tight_layout()
        plt.pause(0.01)

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

    class GPIOControlApp:
        def __init__(self, master):
            self.master = master
            master.title("GPIO Control")
            self.button_st = tk.Button(master, text="Start", command=self.start)
            self.button_stp = tk.Button(master, text="Stop", command=self.stop)
            self.button0 = tk.Button(master, text="Set Mag 18Hz", command=self.set_18hz)
            self.button = tk.Button(master, text="Set Mag 37Hz", command=self.set_37hz)
            self.button2 = tk.Button(master, text="Set Mag 75Hz", command=self.set_75hz)

            self.ser = serial.Serial(
                port='/dev/ttyAMA0',  # Change this according to connection methods, e.g. /dev/ttyUSB0
                baudrate=115200,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=1
            )
            self.button_st.pack()
            self.button_stp.pack()
            self.button0.pack()
            self.button.pack()
            self.button2.pack()

        def drive_gpio_high(self):
            # Drive GPIO pin high
            GPIO.output(gpio_pin, GPIO.HIGH)
            time.sleep(0.1)
            data_to_send = b'Hello from PySerial!'
            self.ser.write(data_to_send)
        def drive_gpio_low(self):
            GPIO.output(gpio_pin, GPIO.LOW)
        
        def write_uart_gpio(self, data_to_send):
            GPIO.output(gpio_pin, GPIO.HIGH)
            time.sleep(0.1)
            self.ser.write(data_to_send)
            time.sleep(0.1)
            GPIO.output(gpio_pin, GPIO.LOW)

        def start(self):
            data_to_send = b'start'
            self.write_uart_gpio(data_to_send)

        def stop(self):
            data_to_send = b'stop'
            self.write_uart_gpio(data_to_send)

        def set_18hz(self):
            data_to_send = b'18'
            self.write_uart_gpio(data_to_send)

        def set_37hz(self):
            data_to_send = b'37'
            self.write_uart_gpio(data_to_send)
        
        def set_75hz(self):
            data_to_send = b'75'
            self.write_uart_gpio(data_to_send)

    def gui_sensor_manager(self):
        root = tk.Tk()
        app = self.GPIOControlApp(root)
        root.mainloop()

        



def main():
    data_processor = DataProcessor()
    passed = 0
    x_series_values = []
    y_series_values = []
    z_series_values = []


    while True:
        if passed == 37:
            x_avg = sum(x_series_values)/len(x_series_values)
            y_avg = sum(y_series_values)/len(y_series_values)
            z_avg = sum(z_series_values)/len(z_series_values)
            avgs = [x_avg, y_avg, z_avg]
            data_processor.update_plot(avgs)
            passed = 0
            x_series_values = []
            y_series_values = []
            z_series_values = []
        else:
            data = data_processor.read_data()
            x_series_values.append(data[0])
            y_series_values.append(data[1])
            z_series_values.append(data[2])


        passed += 1


if __name__ == '__main__':
    main()
