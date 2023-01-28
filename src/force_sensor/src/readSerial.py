#! /usr/bin/env python3

from curses import baudrate
# import rospy
import rclpy
import serial
import sys
import time
import numpy as np

import click
from std_msgs.msg import Float32MultiArray

from threading import Lock, Thread


def read_serial_node(node, port, baudrate):
    """ Read serial port and publish data from it
    Serial output format:
    "[Xf.f,f.f, ...]"
        - Start with '['
        - End with ']\n'
        - First letter X represent measurement type
        - f.f measured results, float number
    Args:
        port (string): serial port name
        baudrate (int): baudrate for serial port
    """

    ser_lock = Lock()

    print(f"About to init connection to serial port: {port} with baudrate: {baudrate}")
    ser = serial.Serial(port, baudrate, timeout=1)

    with ser_lock:
        ser.flushInput()
        ser.flushOutput()
    measurement_type = ""
    pub_voltage = node.create_publisher(Float32MultiArray,
        "/force_torque_sensor_raw/adc", 10)
    msgVoltage = Float32MultiArray()
    measurement_type += "V"


    if not measurement_type:
        print("please specify measurement type")
        return

    print("Read serial node started")

    rate = node.create_rate(100)
    with ser_lock:
        try:
            line = ser.readline().decode('utf8')
        except UnicodeDecodeError:
            print("Error processing the serial messages")
            line = ""
    with ser_lock:
        ser.flush()   # flush serial port to get most recent sensor reading
        ser.flushInput()
        ser.flushOutput()
    if len(line) > 4 and line[0] == '[' and line[-2] == ']':
        data_array = line[1:-2].split('&&')
        for data_string in data_array:
            m_type = data_string[0]
            if (measurement_type.find(m_type) == -1):
                print("measurement type not recognizable")

    while rclpy.ok():
        with ser_lock:
            try:
                line = ser.readline().decode('utf8')
            except UnicodeDecodeError:
                print("Error processing the serial messages")
        with ser_lock:
            ser.flush()   # flush serial port to get most recent sensor reading
        # process serial input and publish messages

        if len(line) > 4 and line[0] == '[' and line[-2] == ']':
            data_string = line[1:-2]
            m_type = data_string[0]
            data = data_string[1:].strip().split(',')
            data_list = list(map(float, data))
            if m_type == 'V':
                msgVoltage.data = [data_list[0]]
                pub_voltage.publish(msgVoltage)
            else:
                print(f"Unknown format: {data_array}")
        else:
            print(f"#Line#: #{line}#")

if __name__ == '__main__':
    rclpy.init(args=sys.argv)
    node = rclpy.create_node('sensor_reading_node')

    if node.has_parameter('/festo/phand/port_f'):
        port = node.declare_parameter('/festo/phand/port_f')
    else:
        node.get_logger().error("Please set port in launch file")
    if node.has_parameter('/festo/phand/baudrate_f'):
        baudrate = node.declare_parameter("/festo/phand/baudrate_f")
    else:
        node.get_logger().error("Please set baudrate in launch file")

    port = "/dev/ttyACM0"
    baudrate = 1000000
    read_serial_node(node, port, baudrate)