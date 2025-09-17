#!/usr/bin/env python3
"""
Quality Validator - Validates sensor data quality for Kova network
"""

import numpy as np
from typing import Dict, Any, List
import time


class QualityValidator:
    """Validates sensor data quality and consistency"""
    
    def __init__(self):
        self.min_quality_threshold = 0.5
        self.enable_anomaly_detection = True
        self.enable_temporal_consistency = True
        
        # Historical data for temporal consistency checks
        self.historical_data = {
            'camera': [],
            'lidar': [],
            'imu': [],
            'gps': []
        }
        self.max_history_size = 100
    
    def validate(self, data: Dict[str, Any]) -> Dict[str, Any]:
        """Validate sensor data quality"""
        try:
            sensor_type = data.get('sensor_type', 'unknown')
            quality_score = data.get('quality_score', 0.0)
            
            # Basic quality check
            if quality_score < self.min_quality_threshold:
                return {
                    'is_valid': False,
                    'quality_score': quality_score,
                    'reason': f'Quality score {quality_score:.2f} below threshold {self.min_quality_threshold}'
                }
            
            # Sensor-specific validation
            if sensor_type == 'camera':
                validation_result = self.validate_camera_data(data)
            elif sensor_type == 'lidar':
                validation_result = self.validate_lidar_data(data)
            elif sensor_type == 'imu':
                validation_result = self.validate_imu_data(data)
            elif sensor_type == 'gps':
                validation_result = self.validate_gps_data(data)
            else:
                validation_result = {
                    'is_valid': True,
                    'quality_score': quality_score,
                    'reason': 'Unknown sensor type, basic validation passed'
                }
            
            # Anomaly detection
            if self.enable_anomaly_detection:
                anomaly_result = self.check_for_anomalies(data, sensor_type)
                if not anomaly_result['is_normal']:
                    validation_result['is_valid'] = False
                    validation_result['reason'] = f"Anomaly detected: {anomaly_result['reason']}"
            
            # Temporal consistency check
            if self.enable_temporal_consistency:
                temporal_result = self.check_temporal_consistency(data, sensor_type)
                if not temporal_result['is_consistent']:
                    validation_result['is_valid'] = False
                    validation_result['reason'] = f"Temporal inconsistency: {temporal_result['reason']}"
            
            # Store data for future temporal checks
            self.store_historical_data(data, sensor_type)
            
            return validation_result
            
        except Exception as e:
            return {
                'is_valid': False,
                'quality_score': 0.0,
                'reason': f'Validation error: {str(e)}'
            }
    
    def validate_camera_data(self, data: Dict[str, Any]) -> Dict[str, Any]:
        """Validate camera data specifically"""
        try:
            sensor_data = data.get('data', {})
            quality_score = data.get('quality_score', 0.0)
            
            # Check image dimensions
            width = sensor_data.get('width', 0)
            height = sensor_data.get('height', 0)
            
            if width < 100 or height < 100:
                return {
                    'is_valid': False,
                    'quality_score': quality_score,
                    'reason': f'Image too small: {width}x{height}'
                }
            
            # Check encoding
            encoding = sensor_data.get('encoding', '')
            if not encoding:
                return {
                    'is_valid': False,
                    'quality_score': quality_score,
                    'reason': 'Missing image encoding'
                }
            
            # Check features
            features = sensor_data.get('features', {})
            if not features:
                return {
                    'is_valid': False,
                    'quality_score': quality_score,
                    'reason': 'Missing image features'
                }
            
            # Check brightness (not too dark or too bright)
            mean_brightness = features.get('mean_brightness', 0)
            if mean_brightness < 10 or mean_brightness > 245:
                return {
                    'is_valid': False,
                    'quality_score': quality_score,
                    'reason': f'Image too dark/bright: brightness={mean_brightness:.1f}'
                }
            
            return {
                'is_valid': True,
                'quality_score': quality_score,
                'reason': 'Camera data validation passed'
            }
            
        except Exception as e:
            return {
                'is_valid': False,
                'quality_score': 0.0,
                'reason': f'Camera validation error: {str(e)}'
            }
    
    def validate_lidar_data(self, data: Dict[str, Any]) -> Dict[str, Any]:
        """Validate LiDAR data specifically"""
        try:
            sensor_data = data.get('data', {})
            quality_score = data.get('quality_score', 0.0)
            
            # Check point cloud data
            points = sensor_data.get('points')
            if points is None or len(points) == 0:
                return {
                    'is_valid': False,
                    'quality_score': quality_score,
                    'reason': 'Empty point cloud'
                }
            
            # Check minimum number of points
            num_points = len(points)
            if num_points < 100:
                return {
                    'is_valid': False,
                    'quality_score': quality_score,
                    'reason': f'Too few points: {num_points}'
                }
            
            # Check features
            features = sensor_data.get('features', {})
            if not features:
                return {
                    'is_valid': False,
                    'quality_score': quality_score,
                    'reason': 'Missing point cloud features'
                }
            
            # Check point distribution
            std_x = features.get('std_x', 0)
            std_y = features.get('std_y', 0)
            std_z = features.get('std_z', 0)
            
            if std_x < 0.1 or std_y < 0.1 or std_z < 0.1:
                return {
                    'is_valid': False,
                    'quality_score': quality_score,
                    'reason': 'Point cloud too concentrated (low variance)'
                }
            
            return {
                'is_valid': True,
                'quality_score': quality_score,
                'reason': 'LiDAR data validation passed'
            }
            
        except Exception as e:
            return {
                'is_valid': False,
                'quality_score': 0.0,
                'reason': f'LiDAR validation error: {str(e)}'
            }
    
    def validate_imu_data(self, data: Dict[str, Any]) -> Dict[str, Any]:
        """Validate IMU data specifically"""
        try:
            sensor_data = data.get('data', {})
            quality_score = data.get('quality_score', 0.0)
            
            # Check acceleration data
            linear_acceleration = sensor_data.get('linear_acceleration', [])
            if len(linear_acceleration) != 3:
                return {
                    'is_valid': False,
                    'quality_score': quality_score,
                    'reason': 'Invalid linear acceleration data'
                }
            
            # Check angular velocity data
            angular_velocity = sensor_data.get('angular_velocity', [])
            if len(angular_velocity) != 3:
                return {
                    'is_valid': False,
                    'quality_score': quality_score,
                    'reason': 'Invalid angular velocity data'
                }
            
            # Check orientation data
            orientation = sensor_data.get('orientation', [])
            if len(orientation) != 4:
                return {
                    'is_valid': False,
                    'quality_score': quality_score,
                    'reason': 'Invalid orientation data'
                }
            
            # Check for reasonable acceleration values
            accel_magnitude = np.linalg.norm(linear_acceleration)
            if accel_magnitude > 50.0:  # Unreasonably high acceleration
                return {
                    'is_valid': False,
                    'quality_score': quality_score,
                    'reason': f'Unreasonable acceleration: {accel_magnitude:.2f} m/s²'
                }
            
            # Check for reasonable angular velocity values
            gyro_magnitude = np.linalg.norm(angular_velocity)
            if gyro_magnitude > 100.0:  # Unreasonably high angular velocity
                return {
                    'is_valid': False,
                    'quality_score': quality_score,
                    'reason': f'Unreasonable angular velocity: {gyro_magnitude:.2f} rad/s'
                }
            
            return {
                'is_valid': True,
                'quality_score': quality_score,
                'reason': 'IMU data validation passed'
            }
            
        except Exception as e:
            return {
                'is_valid': False,
                'quality_score': 0.0,
                'reason': f'IMU validation error: {str(e)}'
            }
    
    def validate_gps_data(self, data: Dict[str, Any]) -> Dict[str, Any]:
        """Validate GPS data specifically"""
        try:
            sensor_data = data.get('data', {})
            quality_score = data.get('quality_score', 0.0)
            
            # Check GPS coordinates
            latitude = sensor_data.get('latitude', 0)
            longitude = sensor_data.get('longitude', 0)
            altitude = sensor_data.get('altitude', 0)
            
            # Check for valid latitude (-90 to 90)
            if not (-90 <= latitude <= 90):
                return {
                    'is_valid': False,
                    'quality_score': quality_score,
                    'reason': f'Invalid latitude: {latitude}'
                }
            
            # Check for valid longitude (-180 to 180)
            if not (-180 <= longitude <= 180):
                return {
                    'is_valid': False,
                    'quality_score': quality_score,
                    'reason': f'Invalid longitude: {longitude}'
                }
            
            # Check for reasonable altitude (-1000 to 50000 meters)
            if not (-1000 <= altitude <= 50000):
                return {
                    'is_valid': False,
                    'quality_score': quality_score,
                    'reason': f'Unreasonable altitude: {altitude}'
                }
            
            # Check GPS status
            status = sensor_data.get('status', -1)
            if status < 0:
                return {
                    'is_valid': False,
                    'quality_score': quality_score,
                    'reason': 'No GPS fix'
                }
            
            return {
                'is_valid': True,
                'quality_score': quality_score,
                'reason': 'GPS data validation passed'
            }
            
        except Exception as e:
            return {
                'is_valid': False,
                'quality_score': 0.0,
                'reason': f'GPS validation error: {str(e)}'
            }
    
    def check_for_anomalies(self, data: Dict[str, Any], sensor_type: str) -> Dict[str, Any]:
        """Check for anomalies in sensor data"""
        try:
            # Simple anomaly detection based on data ranges
            sensor_data = data.get('data', {})
            
            if sensor_type == 'camera':
                features = sensor_data.get('features', {})
                mean_brightness = features.get('mean_brightness', 0)
                
                # Check for extremely dark or bright images
                if mean_brightness < 5 or mean_brightness > 250:
                    return {
                        'is_normal': False,
                        'reason': f'Extreme brightness: {mean_brightness:.1f}'
                    }
            
            elif sensor_type == 'lidar':
                features = sensor_data.get('features', {})
                num_points = features.get('num_points', 0)
                
                # Check for extremely sparse or dense point clouds
                if num_points < 50:
                    return {
                        'is_normal': False,
                        'reason': f'Too few points: {num_points}'
                    }
                elif num_points > 100000:
                    return {
                        'is_normal': False,
                        'reason': f'Too many points: {num_points}'
                    }
            
            elif sensor_type == 'imu':
                linear_acceleration = sensor_data.get('linear_acceleration', [])
                if len(linear_acceleration) == 3:
                    accel_magnitude = np.linalg.norm(linear_acceleration)
                    
                    # Check for extreme acceleration
                    if accel_magnitude > 20.0:
                        return {
                            'is_normal': False,
                            'reason': f'Extreme acceleration: {accel_magnitude:.2f} m/s²'
                        }
            
            elif sensor_type == 'gps':
                latitude = sensor_data.get('latitude', 0)
                longitude = sensor_data.get('longitude', 0)
                
                # Check for coordinates at 0,0 (likely invalid)
                if abs(latitude) < 0.001 and abs(longitude) < 0.001:
                    return {
                        'is_normal': False,
                        'reason': 'GPS coordinates at origin (0,0)'
                    }
            
            return {
                'is_normal': True,
                'reason': 'No anomalies detected'
            }
            
        except Exception as e:
            return {
                'is_normal': False,
                'reason': f'Anomaly detection error: {str(e)}'
            }
    
    def check_temporal_consistency(self, data: Dict[str, Any], sensor_type: str) -> Dict[str, Any]:
        """Check temporal consistency with historical data"""
        try:
            if sensor_type not in self.historical_data:
                return {
                    'is_consistent': True,
                    'reason': 'No historical data for comparison'
                }
            
            history = self.historical_data[sensor_type]
            if len(history) < 2:
                return {
                    'is_consistent': True,
                    'reason': 'Insufficient historical data for comparison'
                }
            
            # Get current data
            current_quality = data.get('quality_score', 0.0)
            current_timestamp = data.get('timestamp', time.time())
            
            # Get recent historical data
            recent_history = history[-5:]  # Last 5 measurements
            
            # Check for sudden quality drops
            avg_historical_quality = np.mean([h.get('quality_score', 0.0) for h in recent_history])
            if current_quality < avg_historical_quality - 0.3:  # 30% drop
                return {
                    'is_consistent': False,
                    'reason': f'Sudden quality drop: {current_quality:.2f} vs {avg_historical_quality:.2f}'
                }
            
            # Check for temporal gaps (missing data)
            if len(recent_history) > 0:
                last_timestamp = recent_history[-1].get('timestamp', 0)
                time_gap = current_timestamp - last_timestamp
                if time_gap > 10.0:  # More than 10 seconds gap
                    return {
                        'is_consistent': False,
                        'reason': f'Large temporal gap: {time_gap:.1f} seconds'
                    }
            
            return {
                'is_consistent': True,
                'reason': 'Temporal consistency check passed'
            }
            
        except Exception as e:
            return {
                'is_consistent': False,
                'reason': f'Temporal consistency error: {str(e)}'
            }
    
    def store_historical_data(self, data: Dict[str, Any], sensor_type: str):
        """Store data for temporal consistency checks"""
        try:
            if sensor_type in self.historical_data:
                self.historical_data[sensor_type].append(data)
                
                # Keep only recent data
                if len(self.historical_data[sensor_type]) > self.max_history_size:
                    self.historical_data[sensor_type] = self.historical_data[sensor_type][-self.max_history_size:]
                    
        except Exception:
            pass  # Ignore errors in historical data storage
