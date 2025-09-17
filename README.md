# Kova ROS2 Bridge

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/python-3.8+-blue.svg)](https://www.python.org/)
[![Build Status](https://github.com/kovasystems/kova-ros2-bridge/workflows/CI/badge.svg)](https://github.com/kovasystems/kova-ros2-bridge/actions)

**ROS2 integration bridge for the Kova ecosystem**

Kova ROS2 Bridge is a comprehensive ROS2 package that enables seamless integration between robotic systems and the Kova decentralized data network. It provides data validation, quality assessment, blockchain integration, and reward claiming for ROS2-based robots.

## Features

- **ROS2 Message Handling**: Support for common sensor message types
- **Data Validation**: Real-time data quality assessment and validation
- **Blockchain Integration**: Seamless integration with Kova blockchain
- **Reward System**: Automatic reward claiming for validated data
- **Multi-Robot Support**: Support for multiple robots in a single network
- **Real-time Streaming**: High-performance real-time data streaming
- **Configuration Management**: Flexible configuration system
- **Monitoring**: Comprehensive monitoring and metrics

## Installation

### Prerequisites

- ROS2 Humble or newer
- Python 3.8+
- Kova Core library

### From Source

```bash
cd your_ros2_workspace/src
git clone https://github.com/kovasystems/kova-ros2-bridge.git
cd kova-ros2-bridge
pip install -e .
colcon build --packages-select kova_ros2_bridge
```

### Dependencies

```bash
# Install Python dependencies
pip install -r requirements.txt

# Install ROS2 dependencies
rosdep install --from-paths src --ignore-src -r -y
```

## Quick Start

### Basic Usage

```bash
# Launch the bridge
ros2 launch kova_ros2_bridge kova_bridge.launch.py

# Launch with custom configuration
ros2 launch kova_ros2_bridge kova_bridge.launch.py config_file:=config/custom_config.yaml
```

### Python API

```python
import rclpy
from rclpy.node import Node
from kova_ros2_bridge import KovaBridge, SensorDataProcessor

class MyRobot(Node):
    def __init__(self):
        super().__init__('my_robot')
        
        # Initialize Kova bridge
        self.kova_bridge = KovaBridge()
        
        # Initialize sensor data processor
        self.sensor_processor = SensorDataProcessor()
        
        # Subscribe to sensor topics
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
    
    def camera_callback(self, msg):
        # Process camera data
        processed_data = self.sensor_processor.process_image(msg)
        
        # Validate and submit to Kova network
        if self.kova_bridge.validate_data(processed_data):
            self.kova_bridge.submit_data(processed_data)
    
    def lidar_callback(self, msg):
        # Process LiDAR data
        processed_data = self.sensor_processor.process_pointcloud(msg)
        
        # Validate and submit to Kova network
        if self.kova_bridge.validate_data(processed_data):
            self.kova_bridge.submit_data(processed_data)
```

## Architecture

### Bridge Components

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   ROS2 Topics   │───▶│  Data Processor │───▶│  Kova Bridge    │
│                 │    │                 │    │                 │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│  Sensor Data    │    │  Validation     │    │  Blockchain     │
│  Collection     │    │  Engine         │    │  Integration    │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

### Data Flow

1. **Data Collection**: Subscribe to ROS2 sensor topics
2. **Data Processing**: Process and enhance sensor data
3. **Validation**: Validate data quality and integrity
4. **Submission**: Submit validated data to Kova network
5. **Reward Claiming**: Automatically claim rewards for valid data

## Configuration

### Bridge Configuration

```yaml
# config/kova_bridge.yaml
kova_bridge:
  # Node configuration
  node_name: "kova_bridge"
  namespace: "/kova"
  
  # Validation settings
  validation:
    min_quality_score: 0.7
    enable_anomaly_detection: true
    enable_temporal_consistency: true
  
  # Blockchain settings
  blockchain:
    solana_rpc_url: "https://api.mainnet-beta.solana.com"
    ipfs_api_url: "http://localhost:5001"
    retry_attempts: 3
    timeout_seconds: 30
  
  # Sensor configuration
  sensors:
    camera:
      enabled: true
      topic: "/camera/image_raw"
      quality_threshold: 0.8
      processing:
        resize: [640, 480]
        enhance: true
        denoise: true
    
    lidar:
      enabled: true
      topic: "/lidar/points"
      quality_threshold: 0.7
      processing:
        filter_outliers: true
        downsample: 0.01
    
    imu:
      enabled: true
      topic: "/imu/data"
      quality_threshold: 0.6
      processing:
        calibrate: true
        filter_noise: true
  
  # Reward settings
  rewards:
    auto_claim: true
    claim_interval: 60.0  # seconds
    min_reward_threshold: 0.001  # SOL
```

### Launch Configuration

```python
# launch/kova_bridge.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    config_file = LaunchConfiguration('config_file')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value='config/kova_bridge.yaml',
            description='Path to configuration file'
        ),
        
        Node(
            package='kova_ros2_bridge',
            executable='kova_bridge_node',
            name='kova_bridge',
            parameters=[config_file],
            output='screen'
        ),
        
        Node(
            package='kova_ros2_bridge',
            executable='sensor_processor_node',
            name='sensor_processor',
            parameters=[config_file],
            output='screen'
        ),
        
        Node(
            package='kova_ros2_bridge',
            executable='reward_claimer_node',
            name='reward_claimer',
            parameters=[config_file],
            output='screen'
        )
    ])
```

## API Reference

### KovaBridge Class

```python
from kova_ros2_bridge import KovaBridge, BridgeConfig

# Create bridge with configuration
config = BridgeConfig(
    node_name="kova_bridge",
    namespace="/kova",
    validation_threshold=0.7,
    blockchain_endpoint="https://api.mainnet-beta.solana.com"
)

bridge = KovaBridge(config)

# Initialize bridge
await bridge.initialize()

# Validate data
is_valid = bridge.validate_data(sensor_data)

# Submit data to network
result = await bridge.submit_data(sensor_data)

# Claim rewards
rewards = await bridge.claim_rewards()
```

### SensorDataProcessor Class

```python
from kova_ros2_bridge import SensorDataProcessor, ProcessingConfig

# Create processor with configuration
config = ProcessingConfig(
    enable_image_enhancement=True,
    enable_pointcloud_filtering=True,
    enable_imu_calibration=True
)

processor = SensorDataProcessor(config)

# Process different sensor types
processed_image = processor.process_image(image_msg)
processed_pointcloud = processor.process_pointcloud(pointcloud_msg)
processed_imu = processor.process_imu(imu_msg)
```

### QualityValidator Class

```python
from kova_ros2_bridge import QualityValidator, ValidationConfig

# Create validator with configuration
config = ValidationConfig(
    min_quality_score=0.7,
    enable_anomaly_detection=True,
    enable_temporal_consistency=True
)

validator = QualityValidator(config)

# Validate sensor data
validation_result = validator.validate(sensor_data)

if validation_result.is_valid:
    print(f"Data quality score: {validation_result.quality_score:.2f}")
else:
    print(f"Data validation failed: {validation_result.error_message}")
```

## Supported Message Types

### Camera Data

```python
from sensor_msgs.msg import Image
from kova_ros2_bridge import ImageProcessor

# Process camera image
processor = ImageProcessor()
processed_image = processor.process(image_msg)

# Get quality metrics
quality_metrics = processor.get_quality_metrics(image_msg)
```

### LiDAR Data

```python
from sensor_msgs.msg import PointCloud2
from kova_ros2_bridge import PointCloudProcessor

# Process point cloud
processor = PointCloudProcessor()
processed_cloud = processor.process(pointcloud_msg)

# Filter outliers
filtered_cloud = processor.filter_outliers(processed_cloud)
```

### IMU Data

```python
from sensor_msgs.msg import Imu
from kova_ros2_bridge import IMUProcessor

# Process IMU data
processor = IMUProcessor()
processed_imu = processor.process(imu_msg)

# Calibrate IMU
calibrated_imu = processor.calibrate(processed_imu)
```

### GPS Data

```python
from sensor_msgs.msg import NavSatFix
from kova_ros2_bridge import GPSProcessor

# Process GPS data
processor = GPSProcessor()
processed_gps = processor.process(gps_msg)

# Convert to local coordinates
local_coords = processor.to_local_coordinates(processed_gps)
```

## Monitoring and Metrics

### Built-in Metrics

The bridge provides comprehensive metrics for monitoring:

- **Data Processing Rate**: Messages processed per second
- **Validation Success Rate**: Percentage of data that passes validation
- **Blockchain Submission Rate**: Successful blockchain submissions per minute
- **Reward Claim Rate**: Rewards claimed per hour
- **Quality Score Distribution**: Histogram of data quality scores
- **Error Rate**: Failed operations per minute

### Prometheus Integration

```python
from kova_ros2_bridge import MetricsCollector

# Enable Prometheus metrics
metrics = MetricsCollector()
metrics.enable_prometheus(port=9090)

# Custom metrics
metrics.add_custom_metric("sensor_data_processed_total", "counter")
metrics.add_custom_metric("validation_duration_seconds", "histogram")
```

### Health Checks

```bash
# Check bridge health
ros2 service call /kova/health_check std_srvs/srv/Empty

# Get bridge status
ros2 topic echo /kova/status

# Get metrics
ros2 topic echo /kova/metrics
```

## Examples

### Basic Robot Integration

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from kova_ros2_bridge import KovaBridge, SensorDataProcessor

class KovaRobot(Node):
    def __init__(self):
        super().__init__('kova_robot')
        
        # Initialize Kova bridge
        self.kova_bridge = KovaBridge()
        self.sensor_processor = SensorDataProcessor()
        
        # Subscribe to sensor topics
        self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 10)
        self.create_subscription(PointCloud2, '/lidar/points', self.lidar_callback, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        
        # Timer for periodic operations
        self.create_timer(1.0, self.periodic_callback)
    
    def camera_callback(self, msg):
        processed = self.sensor_processor.process_image(msg)
        if self.kova_bridge.validate_data(processed):
            self.kova_bridge.submit_data(processed)
    
    def lidar_callback(self, msg):
        processed = self.sensor_processor.process_pointcloud(msg)
        if self.kova_bridge.validate_data(processed):
            self.kova_bridge.submit_data(processed)
    
    def imu_callback(self, msg):
        processed = self.sensor_processor.process_imu(msg)
        if self.kova_bridge.validate_data(processed):
            self.kova_bridge.submit_data(processed)
    
    def periodic_callback(self):
        # Claim rewards periodically
        rewards = self.kova_bridge.claim_rewards()
        if rewards > 0:
            self.get_logger().info(f"Claimed {rewards} SOL in rewards")

def main():
    rclpy.init()
    node = KovaRobot()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Multi-Robot Setup

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from kova_ros2_bridge import KovaBridge, MultiRobotManager

class MultiRobotNode(Node):
    def __init__(self):
        super().__init__('multi_robot_manager')
        
        # Initialize multi-robot manager
        self.robot_manager = MultiRobotManager()
        
        # Add robots
        self.robot_manager.add_robot("robot_1", "/robot_1")
        self.robot_manager.add_robot("robot_2", "/robot_2")
        self.robot_manager.add_robot("robot_3", "/robot_3")
        
        # Start all robots
        self.robot_manager.start_all()
    
    def shutdown(self):
        self.robot_manager.stop_all()

def main():
    rclpy.init()
    node = MultiRobotNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.shutdown()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Testing

### Unit Tests

```bash
# Run unit tests
colcon test --packages-select kova_ros2_bridge

# Run specific test
colcon test --packages-select kova_ros2_bridge --test-result-files
```

### Integration Tests

```bash
# Run integration tests
python -m pytest tests/integration/

# Run with coverage
python -m pytest --cov=kova_ros2_bridge tests/
```

### Simulation Testing

```bash
# Launch simulation
ros2 launch kova_ros2_bridge simulation.launch.py

# Run test scenarios
python tests/simulation/test_scenarios.py
```

## Deployment

### Docker Deployment

```dockerfile
FROM ros:humble

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Install Kova ROS2 Bridge
COPY . /workspace/src/kova_ros2_bridge
WORKDIR /workspace
RUN colcon build --packages-select kova_ros2_bridge

# Set up environment
RUN echo "source /workspace/install/setup.bash" >> ~/.bashrc

CMD ["ros2", "launch", "kova_ros2_bridge", "kova_bridge.launch.py"]
```

### Kubernetes Deployment

```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: kova-ros2-bridge
spec:
  replicas: 1
  selector:
    matchLabels:
      app: kova-ros2-bridge
  template:
    metadata:
      labels:
        app: kova-ros2-bridge
    spec:
      containers:
      - name: kova-ros2-bridge
        image: kovasystems/kova-ros2-bridge:latest
        env:
        - name: ROS_DOMAIN_ID
          value: "0"
        - name: KOVA_CONFIG_FILE
          value: "/config/kova_bridge.yaml"
        volumeMounts:
        - name: config
          mountPath: /config
        resources:
          requests:
            memory: "512Mi"
            cpu: "200m"
          limits:
            memory: "1Gi"
            cpu: "500m"
      volumes:
      - name: config
        configMap:
          name: kova-bridge-config
```

## Troubleshooting

### Common Issues

1. **Connection Issues**: Check ROS2 domain ID and network configuration
2. **Validation Failures**: Adjust quality thresholds in configuration
3. **Blockchain Errors**: Verify blockchain endpoint and credentials
4. **Performance Issues**: Optimize processing parameters and enable caching

### Debug Mode

```bash
# Enable debug logging
ros2 launch kova_ros2_bridge kova_bridge.launch.py log_level:=debug

# Enable verbose output
ros2 launch kova_ros2_bridge kova_bridge.launch.py verbose:=true
```

### Log Analysis

```bash
# View bridge logs
ros2 topic echo /kova/logs

# Check error logs
ros2 topic echo /kova/errors

# Monitor performance metrics
ros2 topic echo /kova/metrics
```

## Contributing

We welcome contributions! Please see our [Contributing Guide](CONTRIBUTING.md) for details.

### Development Setup

1. Fork the repository
2. Clone your fork
3. Create a feature branch
4. Make your changes
5. Add tests
6. Run the test suite
7. Submit a pull request

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Links

- [Website](https://www.kova.systems/)
- [Documentation](https://docs.kova.systems/ros2-bridge/)
- [Discord](https://discord.gg/kova)
- [Twitter](https://twitter.com/KovaSystems)

## Acknowledgments

- The ROS2 community for excellent robotics middleware
- The Kova Systems team for the blockchain integration requirements
- The robotics community for sensor data processing algorithms

---

**Made with ❤️ by the Kova Systems team**