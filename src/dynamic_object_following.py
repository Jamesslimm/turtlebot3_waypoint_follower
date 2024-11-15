import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class ObjectFollower(Node):
    def __init__(self):
        super().__init__('object_follower')
        # Subscriber to scan topic
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        # Publisher to control bot movement
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Define following distance
        self.follow_distance = 0.5  # Ideal distance from object in meters
        self.front_angle_range = 30  # 120 degrees front (half on each side)
        self.angle_origin = 30 # Centre point of angle (angle origin - (angle_range /2)
        self.center_angle_range = 4  # 60 degrees for direct forward movement
        self.too_close_distance = 0.3  # Minimum safe distance to avoid collision
        # PID parameters
        self.kp = 0.2  # Proportional gain
        self.kp_angular = 0.02  # Gain for angular velocity

        # self.ki = 0.01  # Integral gain
        # self.kd = 0.01  # Derivative gain

        # PID variables
        # self.prev_error = 0.0
        # self.integral = 0.0
        # self.last_time = self.get_clock().now()

    def calculate_pid(self, distance):
        """PID controller to calculate linear velocity."""
        # Compute error
        error = distance - self.follow_distance

        # Get current time
        current_time = self.get_clock().now()
        time_delta = (current_time - self.last_time).nanoseconds / 1e9

        if time_delta > 0:
            # Proportional term
            proportional = self.kp * error

            # Integral term
            self.integral += error * time_delta
            integral = self.ki * self.integral

            # Derivative term
            derivative = self.kd * (error - self.prev_error) / time_delta

            # Update previous values
            self.prev_error = error
            self.last_time = current_time

            # Calculate output
            output = proportional + integral + derivative

            # Clamp output between min and max values
            output = max(0.1, min(0.5, output))
            return output
        
        return 0.1

    def calculate_proportional_linear(self, distance):
        """Proportional controller to calculate linear velocity."""
        # Compute error
        error = distance - self.follow_distance

        # Calculate output using proportional control
        output = self.kp * error

        # Clamp output between min and max values
        output = max(0.1, min(0.5, output))
        self.get_logger().info(f"Proportional Controller Output (linear.x): {output:.3f}")
        return output

    def calculate_proportional_angular(self, angle):
        """Proportional controller for angular velocity."""
        # Compute error (center is at 0 degrees)
        error = angle - self.angle_origin

        # Calculate output using proportional control
        output = self.kp_angular * error

        # Clamp output between -0.6 and 0.6
        output = max(-0.6, min(0.6, output))
        
        # Print the proportional control output
        self.get_logger().info(f"Proportional Controller Output (angular.z): {output:.3f}")
        return output
    
    def scan_callback(self, msg):
        # Extract scan ranges within the front 120 degrees
        ranges = msg.ranges
        num_points = len(ranges)

        # Back is at 0 and wraps around, calculate indices
        front_start = -self.front_angle_range
        front_end = self.front_angle_range

        # Wrap-around indexing for back ranges
        front_ranges = ranges[front_start:] + ranges[:front_end]

        # Find the minimum distance in this range (closest object)
        min_distance = min(front_ranges)
        min_index = front_ranges.index(min_distance)
        self.get_logger().info(f"Closest Object Angle ={min_index}")
        self.get_logger().info(f"Closest Object: {min_distance:.2f}")

        # Initialize movement command
        twist = Twist()

        # If object is detected within a certain range
        if min_distance < float('inf'):
            if min_distance > self.follow_distance:
                # Move forward if object is far
                twist.linear.x = self.calculate_proportional_linear(min_distance)
            elif min_distance < self.too_close_distance:
                # Move backward if the object is too close
                twist.linear.x = -0.2
            else:
                # Stop if within the desired follow distance
                twist.linear.x = 0.0

            # Determine turning or forward-only movement based on object's angle
            if (self.front_angle_range - self.center_angle_range) <= min_index <= (self.front_angle_range + self.center_angle_range):
                twist.angular.z = 0.0  # Move straight forward if within central 60 degrees
            # elif min_index < self.front_angle_range:
            #     twist.angular.z = -0.2 # Turn right
            else:
                twist.angular.z = self.calculate_proportional_angular(min_index)  # Turn left
        else:
            # No object detected, stop
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        # Publish command to move the bot
        self.get_logger().info(f"Twist Message Published: linear.x={twist.linear.x:.3f}, angular.z={twist.angular.z:.3f}")
        self.cmd_vel_publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    object_follower = ObjectFollower()
    rclpy.spin(object_follower)
    object_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import LaserScan
# from geometry_msgs.msg import Twist

# class ObjectFollower(Node):
#     def __init__(self):
#         super().__init__('object_follower')
#         # Subscriber to scan topic
#         self.scan_subscriber = self.create_subscription(
#             LaserScan,
#             '/scan',
#             self.scan_callback,
#             10)
#         # Publisher to control bot movement
#         self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
#         # Define following distance
#         self.follow_distance = 0.5  # Ideal distance from object in meters
#         self.front_angle_range = 60  # 120 degrees front (half on each side)
#         self.center_angle_range = 4  # 60 degrees for direct forward movement
#         self.too_close_distance = 0.3  # Minimum safe distance to avoid collision

#     def scan_callback(self, msg):
#         # Extract scan ranges within the front 120 degrees
#         ranges = msg.ranges
#         num_points = len(ranges)

#         # Back is at 0 and wraps around, calculate indices
#         front_start = -self.front_angle_range
#         front_end = self.front_angle_range

#         # Wrap-around indexing for back ranges
#         front_ranges = ranges[front_start:] + ranges[:front_end]

#         # Find the minimum distance in this range (closest object)
#         min_distance = min(front_ranges)
#         min_index = front_ranges.index(min_distance)

#         # Initialize movement command
#         twist = Twist()
        
#         # If object is detected within a certain range
#         if min_distance < float('inf'):
#             if min_distance > self.follow_distance:
#                 # Move forward if object is far
#                 twist.linear.x = 0.2
#             elif min_distance < self.too_close_distance:
#                 # Move backward if the object is too close
#                 twist.linear.x = -0.2
#             else:
#                 # Stop if within the desired follow distance
#                 twist.linear.x = 0.0

#             # Determine turning or forward-only movement based on object's angle
#             if (self.front_angle_range - self.center_angle_range) <= min_index <= (self.front_angle_range + self.center_angle_range):
#                 twist.angular.z = 0.0  # Move straight forward if within central 60 degrees
#             elif min_index < self.front_angle_range:
#                 twist.angular.z = -0.2 # Turn right
#             else:
#                 twist.angular.z = 0.2  # Turn left
#         else:
#             # No object detected, stop
#             twist.linear.x = 0.0
#             twist.angular.z = 0.0

#         # Publish command to move the bot
#         self.cmd_vel_publisher.publish(twist)

# def main(args=None):
#     rclpy.init(args=args)
#     object_follower = ObjectFollower()
#     rclpy.spin(object_follower)
#     object_follower.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
