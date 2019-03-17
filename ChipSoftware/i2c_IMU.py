from digitalio import DigitalInOut, Direction, Pull
import time
import board
import busio
import adafruit_lsm9ds1
import os

# I2C connection:
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)

north = [-0.192, 0.237, -0.339]

# ONLY GYRO/
temp_list = []


# Built in LEDs
led = DigitalInOut(board.LED)
led.direction = Direction.OUTPUT


######################### MAIN LOOP ##############################
delay = 1
i = 0
while True:

while True:
    print(i)
    led.value = 1
    time.sleep(delay)  # make bigger to slow down
    led.value = 0
    time.sleep(delay)  # make bigger to slow down
    i += 1
    # Read acceleration, magnetometer, gyroscope, temperature.
    accel_x, accel_y, accel_z = sensor.acceleration
    mag_x, mag_y, mag_z = sensor.magnetic
    gyro_x, gyro_y, gyro_z = sensor.gyro
    temp = sensor.temperature
    # Print values.
    print(
        'Acceleration (m/s^2): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(accel_x, accel_y, accel_z))
    print('Magnetometer (gauss): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(
        mag_x, mag_y, mag_z))
    print(
        'Gyroscope (degrees/sec): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(gyro_x, gyro_y, gyro_z))
    print('Temperature: {0:0.3f}C'.format(temp))
    # with open('file.txt', 'a') as f:
    # f.write(bytes(temp))
    # Delay for a second.
    time.sleep(1)
