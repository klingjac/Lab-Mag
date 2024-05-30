#!/bin/bash

sleep 10

source env/bin/activate
python3 data_processor.py &
sleep 10
python3 mag_app.py &
