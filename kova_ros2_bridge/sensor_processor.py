#!/usr/bin/env python3
"""
Sensor Data Processor - Processes various sensor data types
"""

import numpy as np
import cv2
from sensor_msgs.msg import Image, PointCloud2, Imu, NavSatFix
import sensor_msgs_py.point_cloud2 as pc2
from typing import Dict, Any
import time


class SensorDataProcessor:
    """Processes sensor data for Kova network submission"""
    
    def __init__(self):
        self.image_enhancement_enabled = True
        self.pointcloud_filtering_enabled = True
        self.imu_calibration_enabled = True
    
    def process_image(self, msg: Image) -> Dict[str, Any]:
        """Process camera image data"""
        try:
            # Convert ROS Image to OpenCV format
            img = self.ros_image_to_cv2(msg)
            
            # Apply image enhancement if enabled
            if self.image_enhancement_enabled:
                img = self.enhance_image(img)
            
            # Calculate quality metrics
            quality_score = self.calculate_image_quality(img)
            
            # Extract features
            features = self.extract_image_features(img)
            
            return {
                'timestamp': time.time(),
                'sensor_type': 'camera',
                'data': {
                    'width': msg.width,
                    'height': msg.height,
                    'encoding': msg.encoding,
                    'image_data': img.tolist() if img is not None else None,
                    'features': features
                },
                'quality_score': quality_score,
                'metadata': {
                    'header': {
                        'stamp': {
                            'sec': msg.header.stamp.sec,
                            'nanosec': msg.header.stamp.nanosec
                        },
                        'frame_id': msg.header.frame_id
                    }
                }
            }
            
        except Exception as e:
            raise Exception(f"Error processing image: {str(e)}")
    
    def process_pointcloud(self, msg: PointCloud2) -> Dict[str, Any]:
        """Process LiDAR point cloud data"""
        try:
            # Convert ROS PointCloud2 to numpy array
            points = self.ros_pointcloud_to_numpy(msg)
            
            # Apply filtering if enabled
            if self.pointcloud_filtering_enabled:
                points = self.filter_pointcloud(points)
            
            # Calculate quality metrics
            quality_score = self.calculate_pointcloud_quality(points)
            
            # Extract features
            features = self.extract_pointcloud_features(points)
            
            return {
                'timestamp': time.time(),
                'sensor_type': 'lidar',
                'data': {
                    'width': msg.width,
                    'height': msg.height,
                    'point_step': msg.point_step,
                    'row_step': msg.row_step,
                    'points': points.tolist() if points is not None else None,
                    'features': features
                },
                'quality_score': quality_score,
                'metadata': {
                    'header': {
                        'stamp': {
                            'sec': msg.header.stamp.sec,
                            'nanosec': msg.header.stamp.nanosec
                        },
                        'frame_id': msg.header.frame_id
                    }
                }
            }
            
        except Exception as e:
            raise Exception(f"Error processing point cloud: {str(e)}")
    
    def process_imu(self, msg: Imu) -> Dict[str, Any]:
        """Process IMU data"""
        try:
            # Extract IMU data
            linear_acceleration = [
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z
            ]
            
            angular_velocity = [
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z
            ]
            
            orientation = [
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w
            ]
            
            # Apply calibration if enabled
            if self.imu_calibration_enabled:
                linear_acceleration = self.calibrate_linear_acceleration(linear_acceleration)
                angular_velocity = self.calibrate_angular_velocity(angular_velocity)
            
            # Calculate quality metrics
            quality_score = self.calculate_imu_quality(linear_acceleration, angular_velocity)
            
            return {
                'timestamp': time.time(),
                'sensor_type': 'imu',
                'data': {
                    'linear_acceleration': linear_acceleration,
                    'angular_velocity': angular_velocity,
                    'orientation': orientation,
                    'covariance': {
                        'linear_acceleration': msg.linear_acceleration_covariance,
                        'angular_velocity': msg.angular_velocity_covariance,
                        'orientation': msg.orientation_covariance
                    }
                },
                'quality_score': quality_score,
                'metadata': {
                    'header': {
                        'stamp': {
                            'sec': msg.header.stamp.sec,
                            'nanosec': msg.header.stamp.nanosec
                        },
                        'frame_id': msg.header.frame_id
                    }
                }
            }
            
        except Exception as e:
            raise Exception(f"Error processing IMU data: {str(e)}")
    
    def process_gps(self, msg: NavSatFix) -> Dict[str, Any]:
        """Process GPS data"""
        try:
            # Extract GPS data
            latitude = msg.latitude
            longitude = msg.longitude
            altitude = msg.altitude
            
            # Calculate quality metrics
            quality_score = self.calculate_gps_quality(latitude, longitude, altitude, msg.status)
            
            return {
                'timestamp': time.time(),
                'sensor_type': 'gps',
                'data': {
                    'latitude': latitude,
                    'longitude': longitude,
                    'altitude': altitude,
                    'status': msg.status.status,
                    'covariance': msg.position_covariance
                },
                'quality_score': quality_score,
                'metadata': {
                    'header': {
                        'stamp': {
                            'sec': msg.header.stamp.sec,
                            'nanosec': msg.header.stamp.nanosec
                        },
                        'frame_id': msg.header.frame_id
                    }
                }
            }
            
        except Exception as e:
            raise Exception(f"Error processing GPS data: {str(e)}")
    
    def ros_image_to_cv2(self, msg: Image) -> np.ndarray:
        """Convert ROS Image message to OpenCV format"""
        try:
            if msg.encoding == 'bgr8':
                return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            elif msg.encoding == 'rgb8':
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
                return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            else:
                # Default handling for other encodings
                return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        except Exception:
            return None
    
    def ros_pointcloud_to_numpy(self, msg: PointCloud2) -> np.ndarray:
        """Convert ROS PointCloud2 message to numpy array"""
        try:
            return np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))
        except Exception:
            return None
    
    def enhance_image(self, img: np.ndarray) -> np.ndarray:
        """Apply image enhancement"""
        if img is None:
            return img
        
        try:
            # Apply histogram equalization
            if len(img.shape) == 3:
                img_yuv = cv2.cvtColor(img, cv2.COLOR_BGR2YUV)
                img_yuv[:,:,0] = cv2.equalizeHist(img_yuv[:,:,0])
                return cv2.cvtColor(img_yuv, cv2.COLOR_YUV2BGR)
            else:
                return cv2.equalizeHist(img)
        except Exception:
            return img
    
    def filter_pointcloud(self, points: np.ndarray) -> np.ndarray:
        """Filter point cloud data"""
        if points is None or len(points) == 0:
            return points
        
        try:
            # Remove points that are too far away
            distances = np.linalg.norm(points, axis=1)
            mask = distances < 50.0  # 50 meter range
            return points[mask]
        except Exception:
            return points
    
    def calibrate_linear_acceleration(self, accel: list) -> list:
        """Apply linear acceleration calibration"""
        # Simple bias correction (in real implementation, use proper calibration)
        bias = [0.1, 0.05, 0.02]  # Example bias values
        return [a - b for a, b in zip(accel, bias)]
    
    def calibrate_angular_velocity(self, gyro: list) -> list:
        """Apply angular velocity calibration"""
        # Simple bias correction (in real implementation, use proper calibration)
        bias = [0.01, 0.01, 0.01]  # Example bias values
        return [g - b for g, b in zip(gyro, bias)]
    
    def calculate_image_quality(self, img: np.ndarray) -> float:
        """Calculate image quality score"""
        if img is None:
            return 0.0
        
        try:
            # Calculate sharpness using Laplacian variance
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if len(img.shape) == 3 else img
            laplacian_var = cv2.Laplacian(gray, cv2.CV_64F).var()
            
            # Normalize to 0-1 range
            quality = min(laplacian_var / 1000.0, 1.0)
            return max(quality, 0.0)
        except Exception:
            return 0.5  # Default quality score
    
    def calculate_pointcloud_quality(self, points: np.ndarray) -> float:
        """Calculate point cloud quality score"""
        if points is None or len(points) == 0:
            return 0.0
        
        try:
            # Calculate density and distribution quality
            num_points = len(points)
            if num_points < 100:
                return 0.3
            
            # Calculate point density
            density_score = min(num_points / 10000.0, 1.0)
            
            # Calculate distribution quality (variance in distances)
            distances = np.linalg.norm(points, axis=1)
            if len(distances) > 0:
                distribution_score = min(np.std(distances) / 10.0, 1.0)
            else:
                distribution_score = 0.0
            
            return (density_score + distribution_score) / 2.0
        except Exception:
            return 0.5  # Default quality score
    
    def calculate_imu_quality(self, accel: list, gyro: list) -> float:
        """Calculate IMU data quality score"""
        try:
            # Check for reasonable acceleration values (not in free fall or extreme acceleration)
            accel_magnitude = np.linalg.norm(accel)
            if 8.0 <= accel_magnitude <= 12.0:  # Earth gravity ± some tolerance
                accel_score = 1.0
            else:
                accel_score = max(0.0, 1.0 - abs(accel_magnitude - 9.81) / 9.81)
            
            # Check for reasonable gyro values (not spinning extremely fast)
            gyro_magnitude = np.linalg.norm(gyro)
            if gyro_magnitude < 10.0:  # Reasonable angular velocity
                gyro_score = 1.0
            else:
                gyro_score = max(0.0, 1.0 - gyro_magnitude / 100.0)
            
            return (accel_score + gyro_score) / 2.0
        except Exception:
            return 0.5  # Default quality score
    
    def calculate_gps_quality(self, lat: float, lon: float, alt: float, status: int) -> float:
        """Calculate GPS data quality score"""
        try:
            # Check if GPS fix is valid
            if status < 0:  # No fix
                return 0.0
            elif status == 0:  # No fix
                return 0.0
            elif status == 1:  # GPS fix
                return 0.7
            elif status == 2:  # DGPS fix
                return 0.9
            else:  # Unknown status
                return 0.5
            
            # Additional quality checks could be added here
            # (e.g., check for reasonable latitude/longitude values)
            
        except Exception:
            return 0.5  # Default quality score
    
    def extract_image_features(self, img: np.ndarray) -> Dict[str, Any]:
        """Extract features from image"""
        if img is None:
            return {}
        
        try:
            features = {}
            
            # Basic image statistics
            if len(img.shape) == 3:
                features['mean_brightness'] = float(np.mean(img))
                features['std_brightness'] = float(np.std(img))
                features['channels'] = img.shape[2]
            else:
                features['mean_brightness'] = float(np.mean(img))
                features['std_brightness'] = float(np.std(img))
                features['channels'] = 1
            
            features['width'] = int(img.shape[1])
            features['height'] = int(img.shape[0])
            
            return features
        except Exception:
            return {}
    
    def extract_pointcloud_features(self, points: np.ndarray) -> Dict[str, Any]:
        """Extract features from point cloud"""
        if points is None or len(points) == 0:
            return {}
        
        try:
            features = {}
            
            # Basic point cloud statistics
            features['num_points'] = int(len(points))
            features['mean_x'] = float(np.mean(points[:, 0]))
            features['mean_y'] = float(np.mean(points[:, 1]))
            features['mean_z'] = float(np.mean(points[:, 2]))
            features['std_x'] = float(np.std(points[:, 0]))
            features['std_y'] = float(np.std(points[:, 1]))
            features['std_z'] = float(np.std(points[:, 2]))
            
            # Calculate bounding box
            features['min_x'] = float(np.min(points[:, 0]))
            features['max_x'] = float(np.max(points[:, 0]))
            features['min_y'] = float(np.min(points[:, 1]))
            features['max_y'] = float(np.max(points[:, 1]))
            features['min_z'] = float(np.min(points[:, 2]))
            features['max_z'] = float(np.max(points[:, 2]))
            
            return features
        except Exception:
            return {}
