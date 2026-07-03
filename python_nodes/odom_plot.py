#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import matplotlib.transforms as transforms
import numpy as np
from collections import deque

class OdometryVisualizer(Node):
    def __init__(self):
        super().__init__('odometry_visualizer')
        
        # Create subscribers for three odometry topics
        self.rtabmap_sub = self.create_subscription(
            Odometry,
            'rtabmap/odom',
            self.rtabmap_callback,
            10
        )
        
        self.controller_sub = self.create_subscription(
            Odometry,
            'controller/odometry',
            self.controller_callback,
            10
        )
        
        self.local_sub = self.create_subscription(
            Odometry,
            'odometry/local',
            self.local_callback,
            10
        )
        
        # Storage for odometry data
        self.rtabmap_data = {'x': deque(maxlen=1000), 'y': deque(maxlen=1000), 
                             'cov': deque(maxlen=1)}
        self.controller_data = {'x': deque(maxlen=1000), 'y': deque(maxlen=1000), 
                                'cov': deque(maxlen=1)}
        self.local_data = {'x': deque(maxlen=1000), 'y': deque(maxlen=1000), 
                           'cov': deque(maxlen=1)}
        
        # Set up matplotlib for real-time plotting
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        
        # Create timer for updating plot
        self.timer = self.create_timer(0.1, self.update_plot)
        
        self.get_logger().info('Odometry Visualizer Node Started')
    
    def rtabmap_callback(self, msg):
        self.rtabmap_data['x'].append(msg.pose.pose.position.x)
        self.rtabmap_data['y'].append(msg.pose.pose.position.y)
        # Extract 2D covariance (x, y positions)
        cov_matrix = np.array([[msg.pose.covariance[0], msg.pose.covariance[1]],
                               [msg.pose.covariance[6], msg.pose.covariance[7]]])
        self.rtabmap_data['cov'] = cov_matrix
    
    def controller_callback(self, msg):
        self.controller_data['x'].append(msg.pose.pose.position.x)
        self.controller_data['y'].append(msg.pose.pose.position.y)
        cov_matrix = np.array([[msg.pose.covariance[0], msg.pose.covariance[1]],
                               [msg.pose.covariance[6], msg.pose.covariance[7]]])
        self.controller_data['cov'] = cov_matrix
    
    def local_callback(self, msg):
        self.local_data['x'].append(msg.pose.pose.position.x)
        self.local_data['y'].append(msg.pose.pose.position.y)
        cov_matrix = np.array([[msg.pose.covariance[0], msg.pose.covariance[1]],
                               [msg.pose.covariance[6], msg.pose.covariance[7]]])
        self.local_data['cov'] = cov_matrix
    
    def plot_covariance_ellipse(self, mean, cov, ax, n_std=2.0, **kwargs):
        """
        Plot covariance ellipse for 2D Gaussian distribution
        """
        # Calculate eigenvalues and eigenvectors
        eigenvalues, eigenvectors = np.linalg.eigh(cov)
        
        # Calculate angle of ellipse
        angle = np.degrees(np.arctan2(eigenvectors[1, 0], eigenvectors[0, 0]))
        
        # Width and height are "full" widths, not radius
        width, height = 2 * n_std * np.sqrt(eigenvalues)
        
        ellipse = Ellipse(mean, width, height, angle=angle, **kwargs)
        ax.add_patch(ellipse)
        
        return ellipse
    
    def update_plot(self):
        # self.ax.clear()
        
        # Plot rtabmap odometry
        # if len(self.rtabmap_data['x']) > 0:
        #     self.ax.plot(list(self.rtabmap_data['x']), 
        #                 list(self.rtabmap_data['y']), 
        #                 'b-', label='rtabmap/odom', linewidth=2)
            
        #     # Plot covariance ellipse at current position
        #     if isinstance(self.rtabmap_data['cov'], np.ndarray):
        #         current_pos = [self.rtabmap_data['x'][-1], 
        #                       self.rtabmap_data['y'][-1]]
        #         self.plot_covariance_ellipse(
        #             current_pos, 
        #             self.rtabmap_data['cov'],
        #             self.ax,
        #             n_std=2.0,
        #             facecolor='blue',
        #             alpha=0.3,
        #             edgecolor='blue',
        #             linewidth=2
        #         )
        
        # Plot controller odometry
        if len(self.controller_data['x']) > 0:
            self.ax.plot(list(self.controller_data['x']), 
                        list(self.controller_data['y']), 
                        'r-', label='controller/odometry', linewidth=2)
            
            if isinstance(self.controller_data['cov'], np.ndarray):
                current_pos = [self.controller_data['x'][-1], 
                              self.controller_data['y'][-1]]
                self.plot_covariance_ellipse(
                    current_pos, 
                    self.controller_data['cov'],
                    self.ax,
                    n_std=2.0,
                    facecolor='red',
                    alpha=0.3,
                    edgecolor='red',
                    linewidth=2
                )
        
        # # Plot local odometry
        # if len(self.local_data['x']) > 0:
        #     self.ax.plot(list(self.local_data['x']), 
        #                 list(self.local_data['y']), 
        #                 'g-', label='odometry/local', linewidth=2)
            
        #     if isinstance(self.local_data['cov'], np.ndarray):
        #         current_pos = [self.local_data['x'][-1], 
        #                       self.local_data['y'][-1]]
        #         self.plot_covariance_ellipse(
        #             current_pos, 
        #             self.local_data['cov'],
        #             self.ax,
        #             n_std=2.0,
        #             facecolor='green',
        #             alpha=0.3,
        #             edgecolor='green',
        #             linewidth=2
        #         )
        
        self.ax.set_xlabel('X Position (m)')
        self.ax.set_ylabel('Y Position (m)')
        self.ax.set_title('Multi-Odometry Visualization with Covariance Ellipses')
        self.ax.legend()
        self.ax.grid(True)
        self.ax.set_aspect('equal', adjustable='box')
        
        plt.pause(0.001)

def main(args=None):
    rclpy.init(args=args)
    
    node = OdometryVisualizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        plt.close('all')

if __name__ == '__main__':
    main()