# ros2_turtlesim_control

#### HW1
- Siwakon Rommueang
#### Ros control

- Examine the code

```sh
#!/usr/bin/python3
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from turtlesim.msg import Pose
from std_srvs.srv import Empty
## Use custom msg for /turtle1/scan
from turtlesim_plus_interfaces.msg import ScannerDataArray, ScannerData

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        ## Declared scan distance turtle to pizza.
        self.scan_offset = 1.5
        ## Declared locations of pizza.
        self.way_point : list = list()
        ## Advertise controller cmd vel.
        self.command_publisher = self.create_publisher(Twist,'/turtle_controller/cmd_vel',10)
        ## Subscribed turtle pose.
        self.pose_subscription = self.create_subscription(Pose,'/turtle1/pose',self.pose_callback,10)
        ## Period to update cmd vel.
        self.timer_period = 0.1
        ## Update turtle cmd vel loop.
        self.timer = self.create_timer(self.timer_period,self.timer_callback)
        ## Init Pose msg to get data from /turtle1/pose.
        self.pose = Pose()
        ## Get pizza location.
        self.goal_subscription = self.create_subscription(Point,'/mouse_position',self.goal_callback,10)
        ## Get pizza scanner that return distance turtle to pizza.
        self.scan_subscription = self.create_subscription(ScannerDataArray,'/turtle1/scan',self.scan_callback,10)
        ## Connect to service server that call to eat pizaa.
        self.eat_cli = self.create_client(Empty, '/turtle1/eat')

    def save_way_point_data(self, goal):
        '''
        Add pizaa location to way point.
        '''
        self.way_point.append(goal)

    def delete_way_point_data(self):
        '''
        Delete location that turtle has eat pizza.
        '''
        if self.way_point.__len__() != 0:
            self.way_point.pop(0)

    def let_eat_pizza(self):
        '''
        Call eat service.
        '''
        self.eat_cli.call_async(Empty.Request())
        return None
    
    def timer_callback(self):
        '''
        Publish cmd vel.
        '''
        msg = self.control()
        self.command_publisher.publish(msg)

    def pose_callback(self,msg):
        '''
        Subscribed turtle pose (cuurent).
        '''
        self.pose = msg

    def control(self):
        msg = Twist()
        self.current_position = np.array([self.pose.x,self.pose.y])
        try:
            fp = self.way_point[0]
        except:
            fp = self.current_position
        dp =  fp - self.current_position
        e = np.arctan2(dp[1],dp[0])-self.pose.theta
        K = 3.0
        w = K*np.arctan2(np.sin(e),np.cos(e))
        if np.linalg.norm(dp)>self.scan_offset:
            v = 1.0
        else:
            v = 0.0
            w = 0.0
            self.delete_way_point_data()
        msg.linear.x = v
        msg.angular.z = w
        return msg
    
    def goal_callback(self,msg : Point):
        '''
        Add pizaa location to way point as np array
        '''
        self.save_way_point_data(goal=np.array([msg.x, msg.y]))

    def scan_callback(self,msg : ScannerDataArray):
        '''
        If distance turtle to pizza more than 1.4 and less than 1.6
        '''
        for i, item in enumerate(msg.data):
            pizza_info : ScannerData = item
            self.pizza_distance = pizza_info.distance
            if self.pizza_distance >= 1.4 and self.pizza_distance <= 1.6:
                self.let_eat_pizza()
                print(f"Eat pizza")

def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
```

- Using launch file '/turtle1/cmd_vel' to '/turtle_controller/cmd_vel'

```sh
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim_plus',
            executable='turtlesim_plus_node',
            name='turtlesim_plus',
            output='screen',
            emulate_tty=True,
            remappings=[
                ('/turtle1/cmd_vel','/turtle_controller/cmd_vel'),
            ],
        ),

        Node(
            package='turtlesim_plus',
            executable='controller_py',
            name='controller',
            output='screen',
            emulate_tty=True,
        ),

        Node(
            package='turtlesim_plus',
            executable='pizza_on_click',
            name='spawn_pizza_on_click',
            output='screen',
            emulate_tty=True,
        )
    ])
```
- Run launch file
  ```sh
  ros2 launch turtlessim_plus_bring_up_ros2 turtlessim_plus_bring_up_ros2_launch.py
  ```
- Control algorithm
![Screenshot](control_algorithm.png)
- Video
![Screenshot](home_work.gif)
