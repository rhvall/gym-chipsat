from digitalio import DigitalInOut, Direction, Pull
import time
import board
import math
import busio
import adafruit_lsm9ds1
import os

######################### Functions ##############################


def mat_mult(a, b):
    result = []
    result1 = []

    while len(a) > 0:

        d = 0
        a1 = a[:1:]
        # print(a1)
        c = True

        while d < len(a1):
            for x in b:
                for x1 in x:
                    # print("x1 is", x1)
                      # print("y1 is",y1)
                    result.append(x1*a1[0][d])
                d = d+1

        a.pop(0)

    result = [result[i:i+len(b[0])] for i in range(0, len(result), len(b[0]))]

    # print(result)
    sum = 0
    # mi basta fare append per avere tutto in una lista e sommare tutto con ultima funzione che ho fatto
    while len(result) > 0:

        for X in range(len(result[0])):
            for Y in range(len(b)):
                sum = sum+result[Y][X]
            result1.append(sum)

            # print(result1)
            sum = 0
        for s in range(len(b)):
            result.pop(0)

    result1 = [result1[i:i+len(b[0])]
               for i in range(0, len(result1), len(b[0]))]
    return (result1)


def cross(a, b):
    c = [a[1]*b[2] - a[2]*b[1],
         a[2]*b[0] - a[0]*b[2],
         a[0]*b[1] - a[1]*b[0]]

    return c


def magnitude(x): return math.sqrt(sum(map(lambda a: a[0]*a[1], zip(x, x))))


def normalise(x): return [a/magnitude(x) for a in x]


def transpose(m): return list(zip(*m))


def bodyMatrixTranspose(grav, mag):
    grav = normalise(grav)
    mag = normalise(mag)
    cross1 = normalise(cross(grav, mag))
    cross2 = normalise(cross(cross1, grav))

    return [cross2, cross1, grav]


def toDegrees(angle):
    return angle * 180 / math.pi


detect_north = False

# I2C connection:
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)

north = [-0.192, 0.237, -0.339]

g_n = [0, 0, -9.8]

# ONLY GYRO/
temp_list = []


# Built in LEDs
led = DigitalInOut(board.LED)
led.direction = Direction.OUTPUT


######################### MAIN LOOP ##############################
delay = 1
i = 0

while True:

    Tn = [[0.46162, -0.777016, -0.427964],
          [-0.569812, -0.629481,  0.528268],
          [-0.679868, -0., -0.733334]]
    # print(i)
    i += 1
    # Read acceleration, magnetometer, gyroscope, temperature.
    accel_x, accel_y, accel_z = sensor.acceleration
    accel = [accel_x, accel_y, accel_z]
    mag_x, mag_y, mag_z = sensor.magnetic
    mag = [mag_x, mag_y, mag_z]
    # gyro_x, gyro_y, gyro_z = sensor.gyro
    # temp = sensor.temperature
    # # Print values.
    # print(
    #     'Acceleration (m/s^2): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(accel_x, accel_y, accel_z))
    # print('Magnetometer (gauss): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(
    #     mag_x, mag_y, mag_z))
    # print(
    #     'Gyroscope (degrees/sec): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(gyro_x, gyro_y, gyro_z))
    # print('Temperature: {0:0.3f}C'.format(temp))
    # # with open('file.txt', 'a') as f:
    # # f.write(bytes(temp))
    # # Delay for a second.

    # mseg = sum([(n - m)**2 for n, m in zip(g_n, sensor.acceleration)])
    # mse = sum([(n - m)**2 for n, m in zip(north, sensor.magnetic)])
    # print(mseg / 1000)
    # print(mse)
    # led.value = 1
    # time.sleep(mse + mseg / 1000)  # make bigger to slow down
    # led.value = 0
    # time.sleep(mse + mseg / 1000)  # make bigger to slow down

    # print(mseg / 1000)
    rotationMatrix = mat_mult(Tn, bodyMatrixTranspose(accel, mag))
    Tn = [[0.46162, -0.777016, -0.427964],
          [-0.569812, -0.629481,  0.528268],
          [-0.679868, -0., -0.733334]]
    # print(Tn)
    theta_x = math.trunc(toDegrees(math.atan2(
        rotationMatrix[2][1], rotationMatrix[2][2])))
    print(theta_x)
    led.value = 1
    time.sleep(0.1)  # make bigger to slow down
    led.value = 0
    # time.sleep(1)  # make bigger to slow down
    # print(transpose(mag))
    # print(mag)
    # def calculate_rotation(gb, mb, mbxgb):
    #     gn = [0, 0, 1]
    #     rn = [0.4, 0.7, 0]
    #     gnxrn = cross_product(gn, rn)
    #     return calculate_inverse([gb, mb, mbxgb], [gn, rn, gnxrn])
    # if detect_north:
    #     g = sensor.gyro
    #     m = sensor.magnetic
    #     gb = normalize(g)
    #     mb = normalize(m)
    #     cross_mb_gb = cross_product(gb, mb)
    #     rotation_matrix = calculate_rotation(gb, mb, mb)
    #     mse = get_metric(rotation_matrix)
    #     led.value = 1
    #     time.sleep(mse)  # make bigger to slow down
    #     led.value = 0
    #     time.sleep(mse)  # make bigger to slow down
