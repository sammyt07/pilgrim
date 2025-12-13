import copy
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

'''
   StormLidarNode
   Author: Sammy Tribble.

   A ROS 2 Python node that injects Gaussian noise to rover LiDAR
   data in normal vs dust / sandstorm environment conditions. Then,
   publishes the noisy LiDAR data to ROS 2 topic /scan_noisy.
'''

class StormLidarNode(Node):
    def __init__(self):
        super().__init__('storm_lidar_node')

        # read rover LiDAR data
        self.sub = self.create_subscription(
            LaserScan,
            'scan',
            self.inject_noise,
            10
        )

        # publish noisy LiDAR data
        self.pub = self.create_publisher(
            LaserScan,
            'scan_noisy',
            10
        )

        self.std_factor = self.declare_parameter('std_factor', 0.02).value
        self.max_noise = self.declare_parameter('max_noise', 0.1).value

        # LiDAR sources of error we can modify in workspace terminals
        self.drop_prob = self.declare_parameter('drop_prob', 0.0).value
        self.spike_prob = self.declare_parameter('spike_prob', 0.0).value
        self.spike_mag = self.declare_parameter('spike_mag', 1.0).value
        self.storm_mag = self.declare_parameter('storm_mag', 0.0).value  # default normal weather

    '''
        Reads rover LiDAR data, injects Gaussian noise,
        and publishes noisy LiDAR data to ROS 2 topic /scan_noisy.
        Input: msg: LaserScan - rover LiDAR beam.
    '''
    def inject_noise(self, msg: LaserScan):
        noisy_msg = copy.deepcopy(msg)  # deep copy of LiDAR beam
        ranges = np.array(msg.ranges, dtype=np.float32)  # rover LiDAR beams
        finite_mask = np.isfinite(ranges)  # valid beams
        storm = self.get_parameter('storm_mag').value

        # add Gaussian noise in O(n)
        noisy_ranges = []  # noisy beams
        for i in range(len(ranges)):
            dist = ranges[i]

            # inject Gaussian noise to rover LiDAR measurement i
            base_std = self.std_factor * dist
            std_i = base_std * (1.0 + storm)
            std_i = min(std_i, self.max_noise)  # clamp noise range
            noise_i = ranges[i] + np.random.normal(0.0, std_i)  # noise LiDAR beam
            
            noisy_ranges.append(noise_i)
        noisy_ranges = np.array(noisy_ranges, dtype=np.float32)

        # --- beams randomly receive "no return" --- #
        drop_eff = min(self.drop_prob * (1.0 + storm), 1.0)
        if drop_eff > 0.0:
            finite_mask = np.isfinite(noisy_ranges)
            drop_mask = np.random.rand(*noisy_ranges.shape) < drop_eff
            final_drop = finite_mask & drop_mask
            noisy_ranges[final_drop] = np.inf
        
        # --- occasional beam spikes --- #
        spike_eff = min(self.spike_prob * (1.0 + storm), 1.0)
        if spike_eff > 0.0 and self.spike_mag > 0.0:
            finite_mask = np.isfinite(noisy_ranges)
            spike_mask = np.random.rand(*noisy_ranges.shape) < spike_eff
            final_spikes = finite_mask & spike_mask
            signs = np.where(np.random.rand(*noisy_ranges.shape) < 0.5, -1.0, 1.0)
            noisy_ranges[final_spikes] += signs[final_spikes] * self.spike_mag
        
        noisy_msg.ranges = noisy_ranges.tolist()
        self.pub.publish(noisy_msg)  # publish noisy LiDAR beam to /scan_noisy

def main():
    rclpy.init()
    node = StormLidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
