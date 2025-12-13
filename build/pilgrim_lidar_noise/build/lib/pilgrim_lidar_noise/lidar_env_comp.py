import numpy as np
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rcl_interfaces.msg import ParameterEvent

'''
   LidarEnvComp
   Author: Sammy Tribble.

   A ROS 2 Python node that subscribes to ROS 2 topics /scan
   and /scan_noisy and compares them in different planetary
   environments. When the rover is driving in normal conditions
   vs weather conditions (e.g., a dust storm or sandstorm).
'''

class LidarEnvComp(Node):
    def __init__(self):
        super().__init__('lidar_env_comp')

        self.storm_detected = False
        self.samples_normal = []  # normal rover LiDAR data
        self.samples_storm  = []  # storm rover LiDAR data

        # read LiDAR rover data in dust / sandstorm
        self.param_storm = self.create_subscription(
            ParameterEvent,
            '/parameter_events',
            self.on_storm_active,
            10
        )

        # read LiDAR rover data
        self.noisy_data = self.create_subscription(
            LaserScan,
            'scan_noisy',
            self.env_noise,
            10
        )

        self.get_logger().info('LidarEnvComp started')  # if '/scan_noisy' is running
    
    '''
        Updates whether a dust / sandstorm is in progress
        and if the ROS 2 noise injection node is active.
    '''
    def on_storm_active(self, event: ParameterEvent):
        prev_status = self.storm_detected
        
        if event.node.lstrip('/') == 'storm_lidar_node':  # is ROS 2 noise injection active?
            for p in event.changed_parameters:
                if p.name == 'storm_mag':
                    self.storm_detected = (p.value.double_value > 0.0)  # is there an active storm?
        
        # log storm status
        if self.storm_detected != prev_status:
            print("storm in-progress!")

    '''
       Compares LiDAR sample data between normal and dust / sandstorm
       hazardous environment conditions.
       Input: msg: LaserScan - rover LiDAR beam.
    '''
    def env_noise(self, msg: LaserScan):
        samples = np.array(msg.ranges, dtype=np.float32)
        finite_mask = np.isfinite(samples)
        samples = samples[finite_mask]

        # is rover in storm vs in normal weather environment?
        if self.storm_detected:
            self.samples_storm.append(samples)
        else:
            self.samples_normal.append(samples)

    '''
       Compares and visualizes probability distributions for noise on
       rover LiDAR in normal vs dust / sandstorm environment conditions.
    '''
    def comp_noise(self, output='lidar_noise_hist.png'):
        # plot rover LiDAR noise distributions
        plt.figure(figsize=(8,5))
        plt.hist(np.concatenate(self.samples_normal), bins=80, alpha=0.6, density=True, label='Normal Weather Conditions')
        plt.hist(np.concatenate(self.samples_storm), bins=80, alpha=0.6, density=True, label='Dust / Sandstorm Conditions')
        plt.legend()
        plt.title("LiDAR Gaussian Noise Distribution: Normal Weather vs Dust / Sandstorm")
        plt.xlabel("Range [m]")
        plt.ylabel("Probability Density")
        plt.tight_layout()
        plt.savefig(output)
        print(f'Rover LiDAR samples saved: {output}')

def main():
    rclpy.init()
    node = LidarEnvComp()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.comp_noise()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
