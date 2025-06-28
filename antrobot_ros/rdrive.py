# Copyright (C) 2021 Dan Novischi. All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v3 or any later version.

from smbus2 import SMBus
import struct

class RDrive:
    __DCM_NO_LOAD_RPM = 8
    __ENC_CPR_REG = 13
    __DD_EN_DIS = 15
    __DD_WHEEL_SEPARATION = 16
    __DD_WHEEL_RADIUS = 17
    __DD_PID_GAINS = 20
    __DD_VEL_CMD = 21
    __DD_ODOM = 22
    __DD_POSE = 23

    def __init__(
            self, pose=None, wheel_radius=0.03, wheel_separation=0.219,
            encoder_resolution=2100, i2c_port=1, i2c_addr=0x70
    ):
        # Set 2D robot pose
        if pose is None:
            pose = [0.0, 0.0, 0.0]
        self.pose = pose

        # Set i2c com rpi to drive parameters
        self.i2c_port = i2c_port
        self.i2c_addr = i2c_addr

        # Set drive mechanical parameters
        self.wheel_radius = wheel_radius
        self.wheel_separation = wheel_separation

        # Set drive sensors parameters
        self.encoder_resolution = encoder_resolution

        # Set drive status
        self.drive_enable = False
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

    def enable(self):
        with SMBus(self.i2c_port) as bus:
            try:
                bus.write_i2c_block_data(self.i2c_addr, self.__DD_EN_DIS, [1])
                self.drive_enable = True
            except IOError:
                print("enable(): IOError in i2c write.")
                self.drive_enable = False
        return self.drive_enable

    def disable(self):
        with SMBus(self.i2c_port) as bus:
            try:
                bus.write_i2c_block_data(self.i2c_addr, self.__DD_EN_DIS, [0])
                self.drive_enable = False
            except IOError:
                print("disable(): IOError in i2c write.")
        return not self.drive_enable

    def cmd_vel(self, v, omega):
        msg = [byte for data_item in [v, omega] for byte in struct.pack('<f', data_item)]
        with SMBus(self.i2c_port) as bus:
            try:
                bus.write_i2c_block_data(self.i2c_addr, self.__DD_VEL_CMD, msg)
            except IOError:
                print("cmd_vel(): IOError in i2c write.")
            self.linear_velocity = v
            self.angular_velocity = omega

    def set_wheel_radius(self, wheel_radius):
        msg = [byte for byte in struct.pack('<f', wheel_radius)]
        with SMBus(self.i2c_port) as bus:
            try:
                bus.write_i2c_block_data(self.i2c_addr, self.__DD_WHEEL_RADIUS, msg)
            except IOError:
                print("set_wheel_radius(): IOError in i2c write.")
        self.wheel_radius = wheel_radius

    def set_wheel_separation(self, wheel_separation):
        msg = [byte for byte in struct.pack('<f', wheel_separation)]
        with SMBus(self.i2c_port) as bus:
            try:
                bus.write_i2c_block_data(self.i2c_addr, self.__DD_WHEEL_SEPARATION, msg)
            except IOError:
                print("set_wheel_separation(): IOError in i2c write.")
        self.wheel_separation = wheel_separation

    def set_pid_gains(self, motor, kp, ki, kd):
        msg = [motor]
        msg += [byte for data_item in [kp, ki, kd] for byte in struct.pack('<f', data_item)]
        with SMBus(self.i2c_port) as bus:
            try:
                bus.write_i2c_block_data(self.i2c_addr, self.__DD_PID_GAINS, msg)
            except IOError:
                print("set_pid_gains(): IOError in i2c write.")

    def set_encoder_cpr(self, enc, cpr):
        msg = [enc]
        msg += [byte for byte in struct.pack('<i', cpr)]
        with SMBus(self.i2c_port) as bus:
            try:
                bus.write_i2c_block_data(self.i2c_addr, self.__ENC_CPR_REG, msg)
            except IOError:
                print("set_pid_gains(): IOError in i2c write.")

    def get_odom(self):
        """Read odometry data from the drive.

        Returns:
            list: [timestamp (int, 8 bytes), x (float), y (float), theta (float), linear_velocity (float), angular_velocity (float)]
                    or None if read fails.
        """
        with SMBus(self.i2c_port) as bus:
            try:
                data = bus.read_i2c_block_data(self.i2c_addr, self.__DD_ODOM, 28)
                # First 8 bytes: int (timestamp, 8 bytes), next 20 bytes: 5 floats
                timestamp = struct.unpack('<q', bytes(data[0:8]))[0]
                floats = struct.unpack('<fffff', bytes(data[8:28]))
                return [timestamp] + list(floats)
            except IOError:
                print("get_odom(): IOError in i2c read.")
                return None

    def set_pose(self, x, y, theta):
        msg = [byte for data_item in [x, y, theta] for byte in struct.pack('<f', data_item)]
        with SMBus(self.i2c_port) as bus:
            try:
                bus.write_i2c_block_data(self.i2c_addr, self.__DD_POSE, msg)
            except IOError:
                print("set_pose(): IOError in i2c write.")
        self.pose = [x, y, theta]

    def set_no_load_rpm(self, side, no_load_rpm):
        """
        Set the no-load RPM for a DC motor.

        Args:
            side (int): Motor side (e.g., 0 for left, 1 for right)
            no_load_rpm (float): No-load RPM value to set
        """
        msg = [side] + [byte for byte in struct.pack('<f', no_load_rpm)]
        with SMBus(self.i2c_port) as bus:
            try:
                bus.write_i2c_block_data(self.i2c_addr, self.__DCM_NO_LOAD_RPM, msg)
            except IOError:
                print("set_no_load_rpm(): IOError in i2c write.")
