from pni_rm3100 import PniRm3100
import time
import RPi.GPIO as GPIO
import queue
import threading
import csv

class Mag():

    def __init__(self, frequency=37):

        self.reserve_queue = queue.Queue(maxsize=10) #Queue to store values between writes
        self.queue_condition = threading.Condition() #CV to check if the queue has values in it or not
        
        self.mag_addr = 0x23

        self.magnetometer = PniRm3100()
        self.magnetometer.assign_device_addr(self.mag_addr)

        self.magnetometer.print_status_statements = False
        self.magnetometer.print_debug_statements = False

        self.magnetometer.assign_xyz_ccr(x_ccr_in=self.magnetometer.CcrRegister.CCR_DEFAULT, 
                                     y_ccr_in=self.magnetometer.CcrRegister.CCR_DEFAULT, 
                                     z_ccr_in=self.magnetometer.CcrRegister.CCR_DEFAULT)

        #Set the magnetometer frequency here

        if frequency == 37:
            self.magnetometer.assign_tmrc(self.magnetometer.TmrcRegister.TMRC_37HZ)
        else:
            self.magnetometer.assign_tmrc(self.magnetometer.TmrcRegister.TMRC_75HZ)
        
    
        #self.magnetometer.assign_tmrc(self.magnetometer.TmrcRegister.TMRC_37HZ)
        self.magnetometer.write_ccr()
        self.magnetometer.write_config()

        self.header = ["MagX", "MagY", "MagZ"]

        self.filename = "Test.csv"

        with open(self.filename, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(self.header)

        self.write_thread = threading.Thread(target=self.write_file)
        self.read_thread = threading.Thread(target=self.read_mag)

        self.write_thread.start()
        self.read_thread.start()

        self.read_thread.join()
        self.write_thread.join()

        


    def write_file(self):
        while True:
            with self.queue_condition:
                if self.reserve_queue.empty():
                    with open(self.filename, 'a', newline='') as file: 
                        writer = csv.writer(file)
                        writer.writerow("Waiting on queue")
                    self.queue_condition.wait()

                arr = self.reserve_queue.get(block=False, timeout=None)

                with open(self.filename, 'a', newline='') as file: 
                    writer = csv.writer(file)
                    writer.writerow(arr)


    def read_mag(self):
        while True:
            with self.queue_condition:
                readings = self.magnetometer.read_meas()

                magX = readings[0]
                magY = readings[1]
                magZ = readings[2]
                xb = readings[4]
                print(f"XB: {xb}")

                arr = [magX, magY, magZ]

                #print(f"MagX:{magX}, MagY:{magY}, MagZ:{magZ}")

                

                try:
                    self.reserve_queue.put(arr, block=False)
                except queue.Full:
                    print("Queue is full, could not add additional mag array")

                self.queue_condition.notify()

                print("adding element to the queue")

                time.sleep(1/37)

    #def read_mag_intr(self):



def main():
    magnetometer = Mag()
    

if __name__=="__main__":
    main()

        
