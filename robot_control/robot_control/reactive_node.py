#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import time

class ReactiveFollowGap(Node):
    """ 
    Implement Wall Following on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('reactive_node')
        # Topics & Subs, Pubs
        lidarscan_topic = '/demo/laser/out'
        drive_topic = '/demo/cmd_vel'

        # Pubs and Subs
        self.driver_pub =  self.create_publisher(Twist, drive_topic, 10)
        self.scanner_sub = self.create_subscription(LaserScan, lidarscan_topic, self.lidar_callback, 10)

        # Some tunable variables
        self.max_distance = 5.0
        self.average_window = 10
        self.obs_rad = 1.0
        self.car_rad = 0.5
        self.target_distance = 1.25
        self.speed = 1.0
        self.disparity_th = 0.5

        self.previous_angle = 0

        self.min = 0

    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        proc_ranges = np.array(ranges)

        proc_ranges[proc_ranges==float("-inf")] = 0.0
        proc_ranges[proc_ranges==float("inf")] = self.max_distance

        #Clip high values
        proc_ranges[proc_ranges > self.max_distance] = self.max_distance

        #Moving average
        proc_ranges = np.convolve(proc_ranges, np.ones(self.average_window), mode="valid")/self.average_window
         
        proc_ranges = np.array([proc_ranges[0], proc_ranges[0]] + list(proc_ranges) + [proc_ranges[-1], proc_ranges[-1]])

        

        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        max_width = 0
        start = 0
        end = 0
        in_range = False
        tmp_start = 0
        count = 0
        for i in range(len(free_space_ranges)):
            if free_space_ranges[i] !=0:
                count += 1
            if not in_range:
                if free_space_ranges[i] !=0:
                    in_range = True
                    tmp_start = i
            else:
                if free_space_ranges[i] == 0 and in_range:
                    in_range = False
                    self.obs_rad
                    if i - tmp_start > max_width:
                        start = tmp_start
                        end = i-1
                        max_width = i - tmp_start
        
        if free_space_ranges[-1] != 0:
            if i - tmp_start > max_width:
                    start = tmp_start
                    end = i

        if count == 0:
            print("\nNO POSITIVES")
        elif start == end:
            print(f"\nWhat?{count}")

        return start, end
            

    def obliterate_radius(self, center, rad, ranges, data, val = 0.0):

        left, right = False, False

        i = 1

        from_center = data.angle_increment * abs(math.floor((data.angle_min + data.angle_max)/2) - center)
        while not left or not right:

            if not left:

                if center + i >= len(ranges):
                    left = True

                elif abs(ranges[center + i] * math.sin(data.angle_increment * abs(math.floor((data.angle_min + data.angle_max)/2) - (center + i))) - ranges[center] * math.sin(from_center)) <= rad:
                    ranges[center + i] = val

                else:
                    left = True

            if not right:

                if center - i < 0:
                    right = True

                elif abs(ranges[center - i] * math.sin(data.angle_increment * abs(math.floor((data.angle_min + data.angle_max)/2) - (center - i))) - ranges[center] * math.sin(from_center)) <= rad:
                    ranges[center - i] = val

                else:
                    right = True

            i += 1

        return i
    
    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	    Naive: Choose the furthest point within ranges and go there
        """
        if not (start_i < end_i):
            return -1
            
        maxes = []
        val = 0
        for i in range(start_i, end_i):
            if ranges[i] > val:
                val = ranges[i]
                maxes = []

            if ranges[i] == val:
                maxes.append(i)

        return maxes[math.floor(len(maxes)/2)]
        
        
    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an Twist Message
        """
        start_callback = time.time()
        ranges = data.ranges

        proc_ranges = self.preprocess_lidar(ranges)

        # Find closest point to LiDAR
        minimum = np.argmin(proc_ranges)
        
        self.min = minimum
        # Eliminate all points inside 'bubble' (set them to zero) 
        self.obliterate_radius(minimum, self.obs_rad, proc_ranges, data)

        i = 1
        while i < len(proc_ranges):
            if abs(proc_ranges[i] - proc_ranges[i-1]) > self.disparity_th:
                i += self.obliterate_radius(i, self.car_rad, proc_ranges, data, val = proc_ranges[i] if proc_ranges[i] < proc_ranges[i-1] else proc_ranges[i-1]) + 1
            i += 1
        

        proc_ranges[minimum] = 0.0

        h_left = math.floor(abs(data.angle_min - -1* math.pi/2)/data.angle_increment)
        h_right = math.floor(abs(data.angle_min - math.pi/2)/data.angle_increment)

        # print(f"left:{h_left}, right:{h_right}, length{len(proc_ranges)}, sum:{np.sum(proc_ranges)}")

        proc_ranges[:h_left] = 0.0
        proc_ranges[h_right:] = 0.0

        
        # Find max length gap
        start, end = self.find_max_gap(proc_ranges) 

        # print(proc_ranges[h_left:h_right])

        # Find the best point in the gap
        point = self.find_best_point(start, end, proc_ranges)

        # Publish Drive message
        if point != -1:
            angle = data.angle_min
            increment = data.angle_increment

            twst_msg = Twist()
            if abs(1* (angle + point * increment)) > 0.1:
                # Turn
                twst_msg.angular.x = 0.0
                twst_msg.angular.y = 0.0
                twst_msg.angular.z = 1.0* (angle + point * increment)
            else:
                # Don't turn
                twst_msg.angular.x = 0.0
                twst_msg.angular.y = 0.0
                twst_msg.angular.z = 0.0
                
            # Set linar velocity
            twst_msg.linear.x = 0.75
            twst_msg.linear.y = 0.0
            twst_msg.linear.z = 0.0
            
            # Publish message
            self.driver_pub.publish(twst_msg)



def main(args=None):
    rclpy.init(args=args)
    print("Gap Follow Initialized")
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

