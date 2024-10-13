# """
# ===================================================================
# |                                                                 |
# |  COPYRIGHT NOTICE                                               |
# |  Developer: Farhad Shamsfakhr, PhD                              |
# |  EMAIL : fshamsfakhr@gmail.com                                  | 
# |  © [2024] Farhad Shamsfakhr, PhD.                               |
# |                                                                 |
# ===================================================================
# """
import time
import can
import struct
import serial
import re
from dwm1001_systemDefinitions import SYS_DEFS
from dwm1001_apiCommands import DWM1001_API_COMMANDS
from sensor_msgs.msg import Imu, MagneticField, Range
from geometry_msgs.msg import Pose
import numpy as np
import math

class ANValue:
    def __init__(self, name, a, b, c, d, e):
        self.name = name
        self.a = a
        self.b = b
        self.c = c
        self.d = d
        self.e = e
        
class SensorReader:
    def __init__(self):
        # IMU and Magnetometer setup
        self.imu_serial_port = '/dev/ttyUSB0'  # Adjust the port based on your device
        self.imu_baud_rate = 115200  # Adjust baud rate if necessary

        # UWB setup
        self.uwb_serial_port_name = '/dev/ttyACM0'
        self.uwb_baud_rate = 115200
        self.uwb_serial_port = serial.Serial(
            port=self.uwb_serial_port_name,
            baudrate=self.uwb_baud_rate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1
        )

        # Constants
        self.ACCEL_SENSITIVITY = 0.122 * 1e-3  # g/LSB
        self.GYRO_SENSITIVITY = 17.5 * 1e-3    # dps/LSB
        self.G = 9.81  # m/s²
        self.DT = 0.1  # Time step (seconds), this should be adjusted based on actual sampling rate
        self.ALPHA = 0.98  # Complementary filter constant

        # Calibration offsets (replace these with your actual calibration values)
        self.GYRO_X_OFFSET = 0.0
        self.GYRO_Y_OFFSET = 0.0
        self.GYRO_Z_OFFSET = 0.0

        # Initial estimates
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0

        # CAN setup
        self.bus = can.interface.Bus(channel='can0', bustype='socketcan')

    def convert_accelerometer(self, raw_acc):
        acc_g = np.array([raw_acc.x, raw_acc.y, raw_acc.z]) * self.ACCEL_SENSITIVITY
        acc_mps2 = acc_g * self.G
        return acc_mps2

    def convert_gyroscope(self, raw_gyro):
        # Apply calibration offsets
        gyro_dps = (np.array([raw_gyro.x, raw_gyro.y, raw_gyro.z]) - np.array([self.GYRO_X_OFFSET, self.GYRO_Y_OFFSET, self.GYRO_Z_OFFSET])) * self.GYRO_SENSITIVITY
        gyro_rps = np.deg2rad(gyro_dps)
        return gyro_rps

    def calculate_acc_angles(self, acc_mps2):
        acc_x, acc_y, acc_z = acc_mps2
        pitch = np.arctan2(acc_y, np.sqrt(acc_x**2 + acc_z**2))
        roll = np.arctan2(-acc_x, acc_z)
        return pitch, roll

    def complementary_filter(self, pitch_acc, roll_acc, yaw_gyro, gyro_rps, pitch, roll, yaw):
        gyro_x, gyro_y, gyro_z = gyro_rps

        # Integrate gyroscope data
        pitch_gyro = pitch + gyro_x * self.DT
        roll_gyro = roll + gyro_y * self.DT
        yaw_gyro += gyro_z * self.DT

        # Apply complementary filter
        pitch = self.ALPHA * pitch_gyro + (1 - self.ALPHA) * pitch_acc
        roll = self.ALPHA * roll_gyro + (1 - self.ALPHA) * roll_acc

        return pitch, roll, yaw_gyro

    def calibrate_gyroscope(self, imu_ser, num_samples=100):
        print("Calibrating gyroscope...")
        gyro_x_total, gyro_y_total, gyro_z_total = 0.0, 0.0, 0.0
        for _ in range(num_samples):
            if imu_ser.in_waiting > 0:
                imu_data = imu_ser.readline().decode().strip()
                imu_lines = imu_data.split('\n')
                for line in imu_lines:
                    first_word = line.split(':')[0].strip()
                    if first_word == 'Gyroscope':
                        numbers = re.findall(r'[-+]?\d*\.\d+|\d+', line)
                        gyro_x_total += float(numbers[0])
                        gyro_y_total += float(numbers[1])
                        gyro_z_total += float(numbers[2])
        return gyro_x_total / num_samples, gyro_y_total / num_samples, gyro_z_total / num_samples




    def publish_data(self):
        try:
            # Open the IMU serial port
            with serial.Serial(self.imu_serial_port, self.imu_baud_rate, timeout=1) as imu_ser:
                print("Reading from IMU ...")

                # Calibrate the gyroscope
                GYRO_X_OFFSET, GYRO_Y_OFFSET, GYRO_Z_OFFSET = self.calibrate_gyroscope(imu_ser)
                print(f"Gyroscope calibration offsets: X={GYRO_X_OFFSET}, Y={GYRO_Y_OFFSET}, Z={GYRO_Z_OFFSET}")

                # Open the UWB serial port
                self.uwb_serial_port.close()
                time.sleep(1)
                self.uwb_serial_port.open()
                if not self.uwb_serial_port.is_open:
                    raise serial.SerialException("Can't open UWB port: " + str(self.uwb_serial_port.name))

                self.initialize_dwm1001_api()
                time.sleep(2)
                self.uwb_serial_port.write(DWM1001_API_COMMANDS.LEC)
                self.uwb_serial_port.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
                print("Reading from IMU and UWB ports...")

                imu_msg = Imu()
                mag_msg = MagneticField()
                # these values should be replaced with the one get from the magnetometer (for global atittude estimation), but here I just initialized with zero (for local atittude estimation)
                yaw = self.yaw
                pitch = self.pitch
                roll = self.roll
                while True:
                    try:
                        # Read data from the IMU serial port
                        if imu_ser.in_waiting > 0:
                            imu_data = imu_ser.readline().decode().strip()  # Read a line of data and decode it
                            imu_lines = imu_data.split('\n')
                            for line in imu_lines:
                                first_word = line.split(':')[0].strip()
                                if first_word == 'Accelerometer':
                                    numbers = re.findall(r'[-+]?\d*\.\d+|\d+', line)
                                    imu_msg.linear_acceleration.x = float(numbers[0])
                                    imu_msg.linear_acceleration.y = float(numbers[1])
                                    imu_msg.linear_acceleration.z = float(numbers[2])
                                if first_word == 'Gyroscope':
                                    numbers = re.findall(r'[-+]?\d*\.\d+|\d+', line)
                                    imu_msg.angular_velocity.x = float(numbers[0])
                                    imu_msg.angular_velocity.y = float(numbers[1])
                                    imu_msg.angular_velocity.z = float(numbers[2])
                                if first_word == 'X axis raw value':
                                    numbers = re.findall(r'[-+]?\d*\.\d+|\d+', line)
                                    mag_msg.magnetic_field.x = float(numbers[0])
                                    mag_msg.magnetic_field.y = float(numbers[1])
                                    mag_msg.magnetic_field.z = float(numbers[2])

                            acc = imu_msg.linear_acceleration
                            gyro = imu_msg.angular_velocity
                            mag = mag_msg.magnetic_field
                            acc_mps2 = self.convert_accelerometer(acc)
                            gyro_rps = self.convert_gyroscope(gyro)
                            pitch_acc, roll_acc = self.calculate_acc_angles(acc_mps2)
                            pitch, roll, yaw = self.complementary_filter(pitch_acc, roll_acc, yaw, gyro_rps, pitch, roll, yaw)

                            # Print IMU data
                            print("IMU data:")
                            print("  Pitch Increments:", pitch)
                            print("  ROLL Increments:", roll)
                            print("  YAW Increments", yaw)

                        # Read data from the UWB serial port
                        if self.uwb_serial_port.in_waiting > 0:
                            serial_read_line = self.uwb_serial_port.read_until().decode('utf-8')
                            ans_values = {}
                            current_an = None
                            values = serial_read_line.split(',')
                            for value in values:
                                if value.startswith("AN") or value.startswith("POS"):
                                    current_an = value
                                    ans_values[current_an] = ANValue(name=current_an, a=None, b=None, c=None, d=None, e=None)
                                elif current_an is not None:
                                    if ans_values[current_an].a is None:
                                        ans_values[current_an].a = value
                                    elif ans_values[current_an].b is None:
                                        ans_values[current_an].b = value
                                    elif ans_values[current_an].c is None:
                                        ans_values[current_an].c = value
                                    elif ans_values[current_an].d is None:
                                        ans_values[current_an].d = value
                                    elif ans_values[current_an].e is None:
                                        ans_values[current_an].e = value

                            # Create CAN messages from UWB data
                            for key, value in ans_values.items():
                                if "POS" in value.name:

                                    # Create CAN messages from UWB data
                                    try:
                                        x = (float(value.a))
                                        y = (float(value.b))
                                        z = (float(value.c))
                                    except ValueError:
                                        print("Error: UWB data is not in a valid format")
                                        continue  # Skip creating CAN message for this data

                                    # # Create CAN message
                                    # uwb_can_message = can.Message(arbitration_id=0x456, data=[x, y, z], is_extended_id=False)
                                    # self.bus.send(uwb_can_message)
                                    # print("UWB Position: X:", value.a, "Y:", value.b, "Z:", value.c)
                                    numbers = [x, y, z, pitch, roll, yaw]
                                    data = struct.pack('6f', *numbers)

                                    # Split data into 2 CAN messages (first 8 bytes, remaining 4 bytes)
                                    data1 = data[:8]
                                    data2 = data[8:16]
                                    data3 = data[16:]

                                    # Create CAN messages
                                    can_message1 = can.Message(arbitration_id=0x123, data=data1)
                                    can_message2 = can.Message(arbitration_id=0x124, data=data2)
                                    can_message3 = can.Message(arbitration_id=0x125, data=data3)

                                    # Send the messages
                                    self.bus.send(can_message1)
                                    self.bus.send(can_message2)
                                    self.bus.send(can_message3)
                                    print("uwb pose:", numbers)
                                    # self.bus.shutdown()

                    except serial.SerialException as e:
                        print("Serial Exception during reading:", e)
                        break

        except serial.SerialException as e:
            print("Serial Exception during setup:", e)

    def initialize_dwm1001_api(self):
        self.uwb_serial_port.write(DWM1001_API_COMMANDS.RESET)
        self.uwb_serial_port.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
        time.sleep(0.5)
        self.uwb_serial_port.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
        time.sleep(0.5)
        self.uwb_serial_port.write(DWM1001_API_COMMANDS.SINGLE_ENTER)

if __name__ == "__main__":
    sensor_reader = SensorReader()
    sensor_reader.publish_data()
