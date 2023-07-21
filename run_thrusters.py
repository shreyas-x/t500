import can
import time
import random
# from tqdm import tqdm
import statistics as st
import serial
import re
import progressbar
import sys
import os

uint8_from_int32 = lambda f: [f >> 24 & 0xff, f >> 16 & 0xff, f >> 8 & 0xff, f & 0xff]

#color = ['RED', 'GREEN', 'YELLOW', 'BLUE', 'MAGENTA', 'CYAN', 'WHITE']
color = ['GREEN']

def getMessage(motor_id, output):
    #Current control 
    new_id = motor_id | (1<<8)
    motor_data = uint8_from_int32(output)
    return can.Message(
        arbitration_id=new_id, data=motor_data, dlc=4, is_extended_id=True
    )

def stopMessage(motor_id):
    new_id = 0x00000800 | motor_id 
    return can.Message(
        arbitration_id=new_id, data=[0x01, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00 ], dlc=7, is_extended_id=True
    )

def send(bus, msg):
    try:
        bus.send(msg)
    except can.CanError:
        print("Message NOT sent")

def getPercent(current_time, total_time):
    n = ((time.time() - current_time)/total_time)*100
    return max(0, min(n, 100))

def getValueFromRawLoadCell(txt):
    pass

def loop_thrusters():
    # Create instance of can bus
    # bus = can.interface.Bus(bustype='pcan', channel='PCAN_USBBUS1', bitrate=500000)
    
    # Init load cell
    # ser = serial.Serial('COM3', 9600)
    time.sleep(0.1)
    ser.write(b'Z')
    
    current_vals = [7500,10000,15000,17000,19000,20000,22000,25000,27900,29000]
    # current_vals = [27900]
    motor_id = 4
    run_time = 2.5
    stop_time = 3.5 * 60
    # stop_time = 8
    acq_start_time = 1.5

    total_time = run_time + stop_time

    thrusts = []
    
    
    #Loop through list
    for current in current_vals:
        forces = []
        time.sleep(0.5)
        # print("\nZeroing load cell")
        ser.write(b'Z')
        
        widgets = ["Current: " + str(current) + " mA ", progressbar.Percentage(),
                   ' ', progressbar.Bar(marker='█', left='[', right=']'),
                   ' ', progressbar.Counter(), '/100',
                   ' ', progressbar.Timer(),
                   ' ', progressbar.ETA()]
        bar = progressbar.ProgressBar(widgets=widgets, max_value=100, redirect_stdout=True)
        
        #Running
        start_time = time.time()
        p_start = time.time()
        
        ser.read_all()
        ser.reset_input_buffer()
        
        while time.time() - start_time < run_time:
            send(bus, getMessage(motor_id, current))
            bar.update(getPercent(p_start, total_time))
            
            # data acquisition
            loadCellData = ser.readline().decode()    
            
            loadCellData = loadCellData[2:-2]
            try:
                force = float(loadCellData)
            except ValueError:
                force = 0.0
            if time.time() - start_time > acq_start_time:
                forces.append(force)
            
            time.sleep(0.01)
            
        
    
        stdev = st.pstdev(forces)
        mean = st.mean(forces)
        # print("\nMean: ", f'{mean:.3f}', "Standard Deviation: ", f'{stdev:.3f}')
        # eliminate measurements that are more than 2 standard deviations away from the mean
        forces = [f if (f < mean+(2*stdev) or f > mean-(2*stdev))  else 0 for f in forces]
        median = st.median(forces)
        print("Median thrust for " + str(current) + " mA: ", f'{median:.3f}')
        thrusts.append(median)
        
        time.sleep(0.2)
        

        #Stop
        start_time = time.time()
        while time.time() - start_time < stop_time:
            send(bus, stopMessage(motor_id))
            time.sleep(0.2)
            bar.update(getPercent(p_start, total_time))
            
        
    print("All thrust forces: ", thrusts)
    bus.shutdown()
    ser.close()

if __name__ == "__main__":
    try:
        bus = can.interface.Bus(bustype='pcan', channel='PCAN_USBBUS1', bitrate=500000)
        ser = serial.Serial('COM3', 9600)
        loop_thrusters()
    except KeyboardInterrupt:
        print("Interrupted, shutting down...")
        bus.shutdown()
        ser.close()
        try:
            sys.exit(130)
        except SystemExit:
            os._exit(130)