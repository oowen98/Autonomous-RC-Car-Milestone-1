#Sending Throttle and Steering Values to Arduino over i2c
#Jetson Nano: SDA = Pin 27, SCL = Pin 28
#Arduino Nano: SDA = A4, SCL = A5
#i2cdetect -y -r 0
from datetime import datetime
import smbus
import time
from mpu6050 import mpu6050
import csv

bus = smbus.SMBus(0) #i2c Bus 0
#IMU = mpu6050(address=0x68, bus=0)


i2c_Address_Arduino = 0x40 #Address specified in Arduino Code

#Sending a Single Value to the Arduino
def writeValue(Value):
    bus.write_byte(i2c_Address_Arduino, Value)
    return -1

#Sending a list of integers to Arduino for Throttle and Steering Values
def writeSteeringThrottle(data):
    bus.write_block_data(i2c_Address_Arduino,0x00, list(map(int,data)))
    return -1

#Receiving data from the Arduino
def readNumber():
    number = bus.read_byte(i2c_Address_Arduino)
    return number

def IMU_data():
    accel_data = IMU.get_accel_data() #Returns accel data in m/s^2
    gyro_data = IMU.get_gyro_data()
    #print([accel_data, gyro_data])
    return ([accel_data, gyro_data])
    

def writeIMU_Data(elapsed_time):
    accel_data = IMU.get_accel_data() #Returns accel data in m/s^2
    gyro_data = IMU.get_gyro_data()

    f = open("IMUData.csv", "a+", newline="")
    c = csv.writer(f)

    c.writerow([elapsed_time,' ', 'Accelerometer Data m/s^2 (x,y,z)', *accel_data,' ', 'Gyro Data deg/s (x,y,z)', *gyro_data])
    f.close()


def i2c_process(data_q, i2c_break_q):
    now = datetime.now()
    print("Starting i2c process")
    while True:
        #now = datetime.now()
        #current_time = now.strftime("%H:%M:%S:%f")
        #print(current_time)
        if(data_q.empty() == 0):
            data = data_q.get()
            writeSteeringThrottle(data)

            #print("Sent Data i2c")
            #print(current_time, data)

            #print(current_time, "Arduino Return: ", readNumber())

        if(i2c_break_q.empty() == 0):
            print("Breaking out of i2c process")
            break

#Testing before integrating into main.py
'''
data = []
while True:
    #Value = input("Enter Throttle: ")
    #Value2 = input ("Enter Steering: ")
    #print(Value, "," ,Value2)

    #data = [Value, Value2]
    data = [110, 30]
    print(list(map(int,data)))
    #print((data))
    writeSteeringThrottle(data)
    time.sleep(0.5)
    #writeValue(int(Value)) #Sends to Arduino the Value

    #number = readNumber() #Value Returned from the Arduino
    #print(number)
'''