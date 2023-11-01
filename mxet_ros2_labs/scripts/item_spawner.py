#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import DeleteEntity, SpawnEntity, GetEntityState
from geometry_msgs.msg import Quaternion, Pose, Point
from std_msgs.msg import String

class ItemSpawner(Node):
    def __init__(self):
        super().__init__('item_spawner')
        self.items = [
            'package://mxet_ros2_labs/urdf/lab8_urdf/red_bottle.urdf',
            'package://mxet_ros2_labs/urdf/lab8_urdf/blue_bottle.urdf',
            'package://mxet_ros2_labs/urdf/lab8_urdf/green_bottle.urdf'
        ]
        self.col = 0

        self.sm = self.create_client(SpawnEntity, 'gazebo/spawn_entity')
        self.dm = self.create_client(DeleteEntity, 'gazebo/delete_entity')
        self.ms = self.create_client(GetEntityState, 'gazebo/get_entity_state')

        self.pub = self.create_publisher(String, 'bottle_color', 10)

    def check_model(self):
        res = self.ms(self.items[self.col], 'world')
        return res.success

    def get_position(self):
        res = self.ms(self.items[self.col], 'world')
        return res.pose.position.z

    def spawn_model(self):
        item = self.items[self.col]
        pose = Pose(Point(x=2.69, y=-1.73, z=1.2), Quaternion())
        request = SpawnEntity.Request()
        request.name = self.items[self.col]
        request.xml = open(item, 'r').read()
        request.initial_pose = pose
        self.sm.call_async(request)

        # Publish the color
        color = String()
        if self.col == 0:
            color.data = 'red'
        elif self.col == 1:
            color.data = 'blue'
        else:
            color.data = 'green'
        self.pub.publish(color)

        self.col = (self.col + 1) % 3

    def delete_model(self):
        request = DeleteEntity.Request()
        request.name = self.items[self.col]
        self.dm.call_async(request)

    def shutdown_hook(self):
        self.delete_model()
        self.get_logger().info("Shutting down")

def main(args=None):
    rclpy.init(args=args)
    print("Waiting for Gazebo services...")
    cs = ItemSpawner()
    rclpy.spin(cs)

if __name__ == "__main__":
    main()