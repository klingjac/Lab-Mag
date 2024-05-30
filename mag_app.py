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

x_data = 0
y_data_x = 0
y_data_y = 0
y_data_z = 0

def udp_listener(app, host='localhost', port=5005):
    with app.app_context():  # This makes sure the Flask context is carried into the thread
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_socket.bind((host, port))

        while True:
            data, addr = udp_socket.recvfrom(1024)  # Buffer size is 1024 bytes
            data = data.decode('utf-8')
            current_time, x_mag_value, y_mag_value, z_mag_value = map(float, data.split(','))

            #print(f"received -- time:{current_time}, x:{x_mag_value}, y:{y_mag_value}, z:{z_mag_value} ")
            
            global x_data, y_data_x, y_data_y, y_data_z
            x_data = [current_time]
            y_data_x = [x_mag_value]
            y_data_y = [y_mag_value]
            y_data_z = [z_mag_value]
            
def send_tcp_command(host, port, command):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((host, port))
        s.sendall(command.encode('utf-8'))


@app.route('/')
def index():
    return render_template('index.html')

@app.route('/start')
def start():
    return jsonify(success=True)

@app.route('/get_point')
def new_point():
    global x_data, y_data_x, y_data_y, y_data_z
    #print(f"sending vals x_data:{x_data}, x_mag:{y_data_x}, y_mag:{y_data_y}, z_mag:{y_data_z} ")
    return jsonify(x_data=x_data,y_data_x=y_data_x,y_data_y=y_data_y,y_data_z=y_data_z )

@app.route('/stop')
def stop():
    return jsonify(success=True)

@app.route('/set_frequency/<int:frequency>',  methods=['GET', 'POST'])
def set_frequency(frequency):
    host = 'localhost'
    port = 5006
    print(f"sending frequency: {frequency}")
    send_tcp_command(host,port, str(frequency))

    return jsonify(success=True)

if __name__ == '__main__':

    
    thread = threading.Thread(target=udp_listener, args=(app,))
    thread.start()
    app.run(debug=False, host='0.0.0.0')