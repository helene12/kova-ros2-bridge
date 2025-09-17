#!/usr/bin/env python3
"""
Kova Bridge - Main ROS2 node for Kova ecosystem integration
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, Imu, NavSatFix
from std_msgs.msg import String
import json
import asyncio
from typing import Dict, Any, Optional
from .sensor_processor import SensorDataProcessor
from .quality_validator import QualityValidator


class KovaBridge(Node):
    """Main Kova Bridge ROS2 node"""
    
    def __init__(self):
        super().__init__('kova_bridge')
        
        # Initialize components
        self.sensor_processor = SensorDataProcessor()
        self.quality_validator = QualityValidator()
        
        # Configuration
        self.declare_parameter('validation_threshold', 0.7)
        self.declare_parameter('auto_claim_rewards', True)
        self.declare_parameter('blockchain_endpoint', 'https://api.mainnet-beta.solana.com')
        
        # Get parameters
        self.validation_threshold = self.get_parameter('validation_threshold').value
        self.auto_claim_rewards = self.get_parameter('auto_claim_rewards').value
        self.blockchain_endpoint = self.get_parameter('blockchain_endpoint').value
        
        # Initialize publishers
        self.status_pub = self.create_publisher(String, '/kova/status', 10)
        self.metrics_pub = self.create_publisher(String, '/kova/metrics', 10)
        
        # Initialize subscribers
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
        
        # Timer for periodic operations
        self.timer = self.create_timer(1.0, self.periodic_callback)
        
        # Statistics
        self.stats = {
            'messages_processed': 0,
            'validation_successes': 0,
            'validation_failures': 0,
            'rewards_claimed': 0.0
        }
        
        self.get_logger().info('Kova Bridge initialized')
    
    def camera_callback(self, msg: Image):
        """Process camera data"""
        try:
            # Process image data
            processed_data = self.sensor_processor.process_image(msg)
            
            # Validate data quality
            validation_result = self.quality_validator.validate(processed_data)
            
            if validation_result['is_valid'] and validation_result['quality_score'] >= self.validation_threshold:
                # Submit to Kova network
                self.submit_to_kova(processed_data, 'camera')
                self.stats['validation_successes'] += 1
            else:
                self.stats['validation_failures'] += 1
                self.get_logger().warn(f'Camera data validation failed: {validation_result["reason"]}')
            
            self.stats['messages_processed'] += 1
            
        except Exception as e:
            self.get_logger().error(f'Error processing camera data: {str(e)}')
    
    def lidar_callback(self, msg: PointCloud2):
        """Process LiDAR data"""
        try:
            # Process point cloud data
            processed_data = self.sensor_processor.process_pointcloud(msg)
            
            # Validate data quality
            validation_result = self.quality_validator.validate(processed_data)
            
            if validation_result['is_valid'] and validation_result['quality_score'] >= self.validation_threshold:
                # Submit to Kova network
                self.submit_to_kova(processed_data, 'lidar')
                self.stats['validation_successes'] += 1
            else:
                self.stats['validation_failures'] += 1
                self.get_logger().warn(f'LiDAR data validation failed: {validation_result["reason"]}')
            
            self.stats['messages_processed'] += 1
            
        except Exception as e:
            self.get_logger().error(f'Error processing LiDAR data: {str(e)}')
    
    def imu_callback(self, msg: Imu):
        """Process IMU data"""
        try:
            # Process IMU data
            processed_data = self.sensor_processor.process_imu(msg)
            
            # Validate data quality
            validation_result = self.quality_validator.validate(processed_data)
            
            if validation_result['is_valid'] and validation_result['quality_score'] >= self.validation_threshold:
                # Submit to Kova network
                self.submit_to_kova(processed_data, 'imu')
                self.stats['validation_successes'] += 1
            else:
                self.stats['validation_failures'] += 1
                self.get_logger().warn(f'IMU data validation failed: {validation_result["reason"]}')
            
            self.stats['messages_processed'] += 1
            
        except Exception as e:
            self.get_logger().error(f'Error processing IMU data: {str(e)}')
    
    def gps_callback(self, msg: NavSatFix):
        """Process GPS data"""
        try:
            # Process GPS data
            processed_data = self.sensor_processor.process_gps(msg)
            
            # Validate data quality
            validation_result = self.quality_validator.validate(processed_data)
            
            if validation_result['is_valid'] and validation_result['quality_score'] >= self.validation_threshold:
                # Submit to Kova network
                self.submit_to_kova(processed_data, 'gps')
                self.stats['validation_successes'] += 1
            else:
                self.stats['validation_failures'] += 1
                self.get_logger().warn(f'GPS data validation failed: {validation_result["reason"]}')
            
            self.stats['messages_processed'] += 1
            
        except Exception as e:
            self.get_logger().error(f'Error processing GPS data: {str(e)}')
    
    def submit_to_kova(self, data: Dict[str, Any], sensor_type: str):
        """Submit validated data to Kova network"""
        try:
            # Create submission payload
            payload = {
                'sensor_type': sensor_type,
                'timestamp': data.get('timestamp'),
                'data': data,
                'quality_score': data.get('quality_score', 0.0),
                'robot_id': self.get_parameter('robot_id').value if self.has_parameter('robot_id') else 'unknown'
            }
            
            # TODO: Implement actual Kova network submission
            self.get_logger().info(f'Submitting {sensor_type} data to Kova network')
            
        except Exception as e:
            self.get_logger().error(f'Error submitting to Kova network: {str(e)}')
    
    def periodic_callback(self):
        """Periodic callback for status updates and reward claiming"""
        try:
            # Publish status
            status_msg = String()
            status_msg.data = json.dumps({
                'node': 'kova_bridge',
                'status': 'running',
                'stats': self.stats
            })
            self.status_pub.publish(status_msg)
            
            # Publish metrics
            metrics_msg = String()
            metrics_msg.data = json.dumps(self.stats)
            self.metrics_pub.publish(metrics_msg)
            
            # Auto-claim rewards if enabled
            if self.auto_claim_rewards:
                self.claim_rewards()
                
        except Exception as e:
            self.get_logger().error(f'Error in periodic callback: {str(e)}')
    
    def claim_rewards(self):
        """Claim rewards from Kova network"""
        try:
            # TODO: Implement actual reward claiming
            self.get_logger().debug('Claiming rewards from Kova network')
            
        except Exception as e:
            self.get_logger().error(f'Error claiming rewards: {str(e)}')


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    try:
        node = KovaBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
