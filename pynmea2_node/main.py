#!/usr/bin/python

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int64, String
from geometry_msgs.msg import Twist
# sudo apt install ros-iron-geographic-msgs
from geographic_msgs.msg import GeoPoint  # GeoPoint.latitude, GeoPoint.longitude, GeoPoint.altitude
# pip3 install pynmea2
import pynmea2
import serial
import io

KNOTS_TO_M_PER_S = 0.514444 
                       
class NmeaReader(Node):
    def __init__(self):
        super().__init__('nmea_reader')
        
        # все поддерживаемые типы есть в pynmea2>types>talker.py (в VS можно перейти с зажатым ctrl и лкм по одному из типов)
        all_types = {
            "dbt": [pynmea2.types.talker.DBT, [
                ["DBT", Float64,  self.dbt_publisher],                    # depth
                ]], 
            "gga": [pynmea2.types.talker.GGA, [
                ["GGA", GeoPoint, self.gga_publisher],                    # lat lon alt
                ["GGA_quality", Int64, self.gga_quality_publisher],    # quality
                ]],
            "rmc": [pynmea2.types.talker.RMC, [
                ["RMC", Twist,    self.rmc_publisher],  # linear.x = SOG, angular.x = COG
                ]],
            # TODO добавить другие типы сообщений, которые будут нужны (ещё надо сделать обработчик)
        }

        self.listen_types = {}
        self.listen_raw_types = {}

        self.declare_parameter('port', rclpy.Parameter.Type.STRING) 
        self.declare_parameter('baudrate', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('hz', rclpy.Parameter.Type.INTEGER) 
        self.declare_parameter('messages', rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter('raw_messages', rclpy.Parameter.Type.STRING_ARRAY)

        # подключаемся к порту
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        ser = serial.Serial(port, baudrate, timeout=5.0)
        self.serial_io = io.TextIOWrapper(io.BufferedRWPair(ser, ser))

        messages = list(dict.fromkeys(map(lambda x: x.lower(), self.get_parameter('messages').value))) # remove duplicates and make lowercase
        for m in messages:
            if m in all_types.keys():
                msg_type = all_types[m]
                pynmea2_type = msg_type[0]
                if pynmea2_type not in self.listen_types.keys():
                    self.listen_types[pynmea2_type] = []
                processors = self.listen_types[pynmea2_type]
                for t in msg_type[1]:
                    processors.append([self.create_publisher(t[1], t[0],  1), t[2]])
            else:
                raise Exception("message type not found or such message is already being processed")
        
        raw_messages = list(dict.fromkeys(map(lambda x: x.lower(), self.get_parameter('raw_messages').value))) # remove duplicates and make lowercase
        for m in raw_messages:
            if m in all_types:
                msg_type = all_types[m]
                pynmea2_type = msg_type[0]
                self.listen_raw_types[pynmea2_type] = self.create_publisher(String, f"{m}_raw",  1)  # gga_raw, dbt_raw, etc.
            else:
                raise Exception("message type not found or such message is already being processed")

        self.updater = self.create_timer(1 / self.get_parameter('hz').value, self.update)
    


    # TODO другие обработчики сообщений сюда

    @staticmethod
    def dbt_publisher(msg: pynmea2.types.talker.DBT, publisher):
        # dbt - глубина в метрах
        depth = Float64()
        depth.data = msg.depth_meters
        publisher.publish(depth)
    
    @staticmethod
    def gga_quality_publisher(msg: pynmea2.types.talker.GGA, publisher):
        # gga - ~~координаты~~(перечёркнуто), качество сигнала
        gps_qual = Int64()
        gps_qual.data = msg.gps_qual
        publisher.publish(gps_qual)
    
    @staticmethod
    def gga_publisher(msg: pynmea2.types.talker.GGA, publisher):
        # gga - координаты, ~~качество сигнала~~(перечёркнуто). Для координат надо использовать сообщение GeoPoint
        point = GeoPoint()
        point.latitude = msg.latitude
        point.longitude = msg.longitude
        point.altitude = msg.altitude
        publisher.publish(point)

    @staticmethod
    def rmc_publisher(msg: pynmea2.types.talker.RMC, publisher):
        # rmc - SOG, COG (скорость, курс)
        twist = Twist()
        twist.linear.x = msg.spd_over_grnd * KNOTS_TO_M_PER_S
        if msg.true_course is None:
            twist.angular.x = 0.0
        else:
            twist.angular.x = msg.true_course # в float'ах
        publisher.publish(twist)

    def update(self):
        try:
            line = self.serial_io.readline()
            msg = pynmea2.parse(line)
            if type(msg) in self.listen_types:
                # вызываем обработчик сообщений данного типа
                # да, возможно это выглядит страшно)
                # listen_types = {type(message): [ [publisher, sender], [..] ]}
                for publisher, sender in self.listen_types[type(msg)]:
                    sender(msg, publisher)
            elif type(msg) in self.listen_raw_types:
                s = String()
                s.data = line
                self.listen_raw_types[type(msg)].publish(s)
        except pynmea2.ParseError as e:
            self.get_logger().warn(f'Parse error: {e}')


def main():
    rclpy.init()
    nmea_reader = NmeaReader()
    rclpy.spin(nmea_reader)
    nmea_reader.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()