#!/usr/bin/env python3
"""
Reward Claimer Node - Dedicated node for claiming rewards from Kova network
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time


class RewardClaimerNode(Node):
    """Dedicated reward claiming node"""
    
    def __init__(self):
        super().__init__('reward_claimer')
        
        # Configuration
        self.declare_parameter('claim_interval', 60.0)  # seconds
        self.declare_parameter('min_reward_threshold', 0.001)  # SOL
        self.declare_parameter('auto_claim', True)
        
        # Get parameters
        self.claim_interval = self.get_parameter('claim_interval').value
        self.min_reward_threshold = self.get_parameter('min_reward_threshold').value
        self.auto_claim = self.get_parameter('auto_claim').value
        
        # Create publishers
        self.rewards_pub = self.create_publisher(String, '/kova/rewards', 10)
        self.status_pub = self.create_publisher(String, '/kova/reward_status', 10)
        
        # Create subscribers
        self.processed_data_sub = self.create_subscription(
            String,
            '/kova/processed_data',
            self.processed_data_callback,
            10
        )
        
        # Timer for periodic reward claiming
        if self.auto_claim:
            self.timer = self.create_timer(self.claim_interval, self.claim_rewards)
        
        # Statistics
        self.stats = {
            'total_rewards_claimed': 0.0,
            'last_claim_time': 0.0,
            'pending_rewards': 0.0,
            'claim_attempts': 0,
            'successful_claims': 0
        }
        
        self.get_logger().info('Reward Claimer Node initialized')
    
    def processed_data_callback(self, msg: String):
        """Process validated data and accumulate rewards"""
        try:
            data = json.loads(msg.data)
            
            # Calculate potential reward based on data quality
            quality_score = data.get('quality_score', 0.0)
            sensor_type = data.get('sensor_type', 'unknown')
            
            # Calculate reward amount (example calculation)
            base_reward = self.get_base_reward(sensor_type)
            quality_multiplier = quality_score
            reward_amount = base_reward * quality_multiplier
            
            # Add to pending rewards
            self.stats['pending_rewards'] += reward_amount
            
            self.get_logger().debug(f'Accumulated reward: {reward_amount:.6f} SOL for {sensor_type}')
            
        except Exception as e:
            self.get_logger().error(f'Error processing data for rewards: {str(e)}')
    
    def get_base_reward(self, sensor_type: str) -> float:
        """Get base reward amount for sensor type"""
        base_rewards = {
            'camera': 0.001,    # 0.001 SOL per image
            'lidar': 0.002,     # 0.002 SOL per point cloud
            'imu': 0.0005,      # 0.0005 SOL per IMU reading
            'gps': 0.0003       # 0.0003 SOL per GPS reading
        }
        return base_rewards.get(sensor_type, 0.0001)
    
    def claim_rewards(self):
        """Claim accumulated rewards from Kova network"""
        try:
            if self.stats['pending_rewards'] < self.min_reward_threshold:
                self.get_logger().debug(f'Pending rewards {self.stats["pending_rewards"]:.6f} below threshold {self.min_reward_threshold}')
                return
            
            self.stats['claim_attempts'] += 1
            
            # TODO: Implement actual reward claiming with Kova network
            # For now, simulate successful claiming
            claimed_amount = self.stats['pending_rewards']
            
            # Update statistics
            self.stats['total_rewards_claimed'] += claimed_amount
            self.stats['pending_rewards'] = 0.0
            self.stats['last_claim_time'] = time.time()
            self.stats['successful_claims'] += 1
            
            # Publish reward claim result
            reward_msg = String()
            reward_msg.data = json.dumps({
                'claimed_amount': claimed_amount,
                'total_claimed': self.stats['total_rewards_claimed'],
                'timestamp': self.stats['last_claim_time']
            })
            self.rewards_pub.publish(reward_msg)
            
            # Publish status
            status_msg = String()
            status_msg.data = json.dumps({
                'node': 'reward_claimer',
                'status': 'success',
                'stats': self.stats
            })
            self.status_pub.publish(status_msg)
            
            self.get_logger().info(f'Successfully claimed {claimed_amount:.6f} SOL rewards')
            
        except Exception as e:
            self.get_logger().error(f'Error claiming rewards: {str(e)}')
            
            # Publish error status
            status_msg = String()
            status_msg.data = json.dumps({
                'node': 'reward_claimer',
                'status': 'error',
                'error': str(e),
                'stats': self.stats
            })
            self.status_pub.publish(status_msg)
    
    def get_reward_stats(self):
        """Get current reward statistics"""
        return self.stats


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    try:
        node = RewardClaimerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
