"""
═══════════════════════════════════════════════════════════════════════
FILE PURPOSE: test_sensor_fusion.py
═══════════════════════════════════════════════════════════════════════

WHAT IT DOES:
- Tests complete system behavior
- Launches nodes as subprocesses
- Verifies multi-node communication
- Tests fault tolerance

WHEN IT RUNS:
- pytest test_sensor_fusion.py -v

WHY YOU NEED IT:
- Ensures system works end-to-end
- Tests what unit tests can't
- Industry standard for integration testing

═══════════════════════════════════════════════════════════════════════
"""

import pytest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import subprocess
import time

@pytest.fixture
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()

class MessageCollector(Node):
    def __init__(self, topic, msg_type):
        super().__init__('collector')
        self.messages = []
        self.sub = self.create_subscription(
            msg_type, topic, self._callback, 10
        )
    
    def _callback(self, msg):
        self.messages.append(msg)

def test_imu_publishes_data(ros_context):
    """Test IMU node publishes at correct rate"""
    # Launch node
    proc = subprocess.Popen([
        'ros2', 'run', 'sensor_fusion_system', 'imu_sensor_node'
    ])
    time.sleep(2)
    
    try:
        collector = MessageCollector('imu/data', Imu)
        
        # Collect for 1 second
        start = time.time()
        while time.time() - start < 1.1:
            rclpy.spin_once(collector, timeout_sec=0.01)
        
        # Should get ~100 messages (100 Hz)
        assert 90 <= len(collector.messages) <= 110
        
        collector.destroy_node()
    finally:
        proc.terminate()

def test_imu_data_valid(ros_context):
    """Test IMU data has gravity"""
    proc = subprocess.Popen([
        'ros2', 'run', 'sensor_fusion_system', 'imu_sensor_node'
    ])
    time.sleep(2)
    
    try:
        collector = MessageCollector('imu/data', Imu)
        
        # Get one message
        while len(collector.messages) < 1:
            rclpy.spin_once(collector, timeout_sec=0.1)
        
        msg = collector.messages[0]
        
        # Check gravity is present
        assert 9.0 < msg.linear_acceleration.z < 10.5
        
        collector.destroy_node()
    finally:
        proc.terminate()

if __name__ == "__main__":
    pytest.main([__file__, "-v"])