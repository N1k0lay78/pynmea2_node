#!/usr/bin/python

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
# sudo apt install ros-iron-geographic-msgs
from geographic_msgs.msg import GeoPoint  # GeoPoint.latitude, GeoPoint.longitude, GeoPoint.altitude
# pip3 install pynmea2
import pynmea2
import serial
import io


class NvmeaReader(Node):
    HZ = 30

    def __init__(self):
        super().__init__('nvmea_reader')
        
        # все поддерживаемые типы есть в pynmea2>types>talker.py (в VS можно перейти с зажатым ctrl и лкм по одному из типов)
        all_types = {
            "dbt": [pynmea2.types.talker.DBT, "/DBT", Float64,  self.dbt_publisher],  # depth
            "gga": [pynmea2.types.talker.GGA, "/GGA", GeoPoint, self.gga_publisher],  # lat lon alt
            "rmc": [pynmea2.types.talker.RMC, "/RMC", Twist,    self.rmc_publisher],  # linear.x = SOG, angular.x = COG
            # TODO добавить другие типы сообщений, которые будут нужны (ещё надо сделать обработчик)
        }

        self.listen_types = {}

        self.declare_parameter('port', rclpy.Parameter.Type.STRING) 
        self.declare_parameter('baudrate', rclpy.Parameter.Type.INTEGER) 
        self.declare_parameter('messages', rclpy.Parameter.Type.STRING_ARRAY)

        # подключаемся к порту
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        ser = serial.Serial(port, baudrate, timeout=5.0)
        self.serial_io = io.TextIOWrapper(io.BufferedRWPair(ser, ser))

        for message in self.get_parameter('messages').value:
            # мы умеем обрабатывать такой тип и он ещё не создан
            if message in all_types and all_types[message][0] not in self.listen_types:
                # all_types = {message_name: [type(message), "/topic_name", TopicType, topic_sender]}
                data = all_types[message]
                self.listen_types[data[0]] = [self.create_publisher(data[2], data[1],  1), data[3]]
            else:
                raise Exception("message type not found or such message is already being processed")
        
        self.updater = self.create_timer(1 / self.HZ, self.update)

    # TODO другие обработчики сообщений сюда

    @staticmethod
    def dbt_publisher(msg: pynmea2.types.talker.DBT, publisher):
        # dbt - глубина в метрах
        depth = Float64()
        depth.data = msg.depth_meters
        publisher.publish(depth)

    @staticmethod
    def gga_publisher(msg: pynmea2.types.talker.GGA, publisher):
        # gga - координаты, ~~качество сигнала~~(перечёркнуто). Для координат надо использовать сообщение GeoPoint
        point = GeoPoint()
        point.latitude = msg.longitude
        point.longitude = msg.latitude
        point.altitude = msg.altitude
        publisher.publish(point)

    @staticmethod
    def rmc_publisher(msg: pynmea2.types.talker.RMC, publisher):
        # rmc - COG, SOG (скорость, курс)
        twist = Twist()
        twist.linear.x = msg.spd_over_grnd
        twist.angular.x = msg.true_course  # в float'ах
        publisher.publish(twist)

    def update(self):
        try:
            line = self.serial_io.readline()
            msg = pynmea2.parse(line)
            if type(msg) in self.listen_types:
                # вызываем обработчик сообщений данного типа
                # да, возможно это выглядит страшно)
                # listen_types = {type(message): [publisher, sender]}
                self.listen_types[type(msg)][1](msg, self.listen_types[type(msg)][0])
        except serial.SerialException as e:
            raise Exception(f'Device error: {e}')
        except pynmea2.ParseError as e:
            self.get_logger().warn(f'Parse error: {e}')


def main():
    rclpy.init()
    nvmea_reader = NvmeaReader()
    rclpy.spin(nvmea_reader)
    nvmea_reader.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
