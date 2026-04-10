# LAB 5 LEARNING 
# ROS2 is currently the most popular middleware for robotics, allowing you to run code on different machines and communicate with each other (lets a bunch of topics and sensors talk to each other)

# code files that run on the ros2 framework are called nodes, for example a node (script) called detect_object.py would be plugged into a node called drive.py
# there is multiple ways nodes can communicate with each other  (publish/subscribe, service, action)
# example of published subscriber is camera.py and detect_object.py
# example of service is survey.py and camera_mover.py, where survey checks the space, send camera mover the next step to move some direction, and then sends back a responce (image), in thsi case camera_mover.py is the "server"
# example of action is way_point_nav.py and controller.py, where way_point_nav sends a waypoint for controller to travel to and controller send back feedback about its progress, when its done it can send a "result" which could be like an image of the new location

# we can use a node parameter so we can adjust the properties of the robot without having to change all the scripts, by making them refer to the parameter file

# a bag file can have can subscribe to multiple topics and record data as it comes in, and you can play it back later

# Ros2 allows for code to run on different machines like windows, linux, mac, etc.

# can also make packages which is a collectio of nodes and it can be written in c++ or python and still work together with the rest of teh code. An example is a package which helps read and get data data from a specific sensor, or maybe even an autopioloet i would guess. 

# Gazebo is just a physics simulator with a visual use interface so we can simulate the drone flight and visualize it on a computer. 
'''
Low-level control  →  Middleware  →  Applications
(PX4, firmware)       (ROS 2)        (your code)
'''

# you first need to get px4 running in the background using the following command: the first part is making the px4 node, and the second part is tell it to use the gazebo x500 drone simutlater  (x500 is the actual drone model)
# make px4_sitl gz_x500_mono_cam 
# ^ run this from the PX4-AUTOPILOT folder

# also need to run Q ground contol (just open the application)
# first time have to build it ^ "chmod +x QGroundControl-x86_64.AppImage"
# then you can run it with "./QGroundControl-x86_64.AppImage"

# run "MicroXRCE bridge "MicroXRCEAGENT udp4 -p 8888" from home directory 

# before running this script you should run "ros2 topic list" to see if it outputs "/fmu/out/vehicle_local_position"
# After that run "ros2 topic echo /fmu/out/vehicle_local_position" and see if it outputs the data from the topic
# if both of these work then it is working as intenteded 

# also chat recommended to do this before running the script to make sure the environment is set up correctly:
#"source /opt/ros/humble/setup.bash"
#"source ~/ROS_2_WS/install/setup.bash"

# now you can finally run this subscriber node by running "python3 task_1_node.py" from the terminal directed to the lab5 folder in ROS_2_WS/src/lab5

'''
Gazebo (simulation world)
        ↓
PX4 (drone autopilot)
        ↓
XRCE bridge (PX4 ↔ ROS 2)
        ↓
ROS 2 (your Python node)
        ↓
QGroundControl (optional GUI)
'''



import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from px4_msgs.msg import VehicleLocalPosition


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
       # by subscribing to the topic, we can receive the data from the topic
        # with px4 some example topics are: /px4_1/fmu/out_vehicle_command, /px4_1/fmu/out_vehicle_local_position, /px4_1/fmu/out_vehicle_global_position
        self.subscription = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(
            f'x: {msg.x:.3f}, y: {msg.y:.3f}, z: {msg.z:.3f}'
        )

    #def listener_callback(self, msg):
    #    self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


