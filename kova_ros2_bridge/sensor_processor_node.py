#!/usr/bin/env python3
"""
Sensor Processor Node - Dedicated node for sensor data processing
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, Imu, NavSatFix
from std_msgs.msg import String
import json
from .sensor_processor import SensorDataProcessor


class SensorProcessorNode(Node):
    """Dedicated sensor data processing node"""
    
    def __init__(self):
        super().__init__('sensor_processor')
        
        # Initialize sensor processor
        self.processor = SensorDataProcessor()
        
        # Create publishers
        self.processed_data_pub = self.create_publisher(String, '/kova/processed_data', 10)
        
        # Create subscribers
        self.camera_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.camera_callback,
            10
        )
        
        self.lidar_sub = self.create_subscription(
            PointCloud2,
            '/lidar/points',
            self.lidar_callback,
            10
        )
        
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )
        
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.gps_callback,
            10
        )
        
        self.get_logger().info('Sensor Processor Node initialized')
    
    def camera_callback(self, msg: Image):
        """Process camera data"""
        try:
            processed_data = self.processor.process_image(msg)
            self.publish_processed_data(processed_data)
        except Exception as e:
            self.get_logger().error(f'Error processing camera data: {str(e)}')
    
    def lidar_callback(self, msg: PointCloud2):
        """Process LiDAR data"""
        try:
            processed_data = self.processor.process_pointcloud(msg)
            self.publish_processed_data(processed_data)
        except Exception as e:
            self.get_logger().error(f'Error processing LiDAR data: {str(e)}')
    
    def imu_callback(self, msg: Imu):
        """Process IMU data"""
        try:
            processed_data = self.processor.process_imu(msg)
            self.publish_processed_data(processed_data)
        except Exception as e:
            self.get_logger().error(f'Error processing IMU data: {str(e)}')
    
    def gps_callback(self, msg: NavSatFix):
        """Process GPS data"""
        try:
            processed_data = self.processor.process_gps(msg)
            self.publish_processed_data(processed_data)
        except Exception as e:
            self.get_logger().error(f'Error processing GPS data: {str(e)}')
    
    def publish_processed_data(self, data: dict):
        """Publish processed sensor data"""
        try:
            msg = String()
            msg.data = json.dumps(data)
            self.processed_data_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing processed data: {str(e)}')


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    try:
        node = SensorProcessorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
