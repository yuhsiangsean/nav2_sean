#!/usr/bin/env python3
"""
PX4 vehicle_odometry -> Nav2 需要的 TF(odom -> base_link) 與 /odom.

- 只取平面資訊（x, y, yaw），高度照抄，roll/pitch 忽略（Nav2 是 2D）
- 座標慣例（Ubuntu 22.04/Humble 版，跟 px4_nav2_bridge 不同）：
    map.x = e, map.y = n, map.z = -d（標準 ENU：東、北、上）
  px4_nav2_bridge 原本是 map.x = n, map.y = -e, map.z = -d（對齊 OptiTrack
  rig 擺放方向，X 前、Y 左），這裡改成標準 ENU，跟 PX4 EKF2 的指北方向
  （magnetic north，見 hover_test 專案關於 heading/EKF2_EV_CTRL 的討論）
  對齊，不是對齊 mocap rig 擺放方向。
- 朝向處理原封不動沿用 px4_nav2_bridge：orientation 暫時固定成 identity
  （yaw=0），原因跟當初一樣未解決（Motive rigid body 本地軸定義問題，
  yaw 不可信）。等之後討論確定怎麼處理 heading 再調整，這裡先不動。
- 時間戳用 ROS 系統時間（全部節點都用 use_sim_time:=false）
"""
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster


class Px4OdomTf(Node):
    def __init__(self):
        super().__init__('px4_odom_tf')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.declare_parameter('topic_odometry', '/fmu/out/vehicle_odometry')
        topic = self.get_parameter('topic_odometry').value
        self.create_subscription(VehicleOdometry, topic, self.cb, qos)
        self.pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf = TransformBroadcaster(self)

    def cb(self, m: VehicleOdometry):
        n, e, d = m.position

        # orientation 暫時固定成 identity（yaw=0）：沿用 px4_nav2_bridge 未解決
        # 的妥協（Motive rigid body 本地軸定義問題，yaw 不可信）。cmd_vel_bridge.py
        # 那邊也要維持同樣的假設（機體座標=世界座標），兩邊才會一致。先不動，
        # 等之後討論確定怎麼處理 heading 再一起改。
        cz, sz = 1.0, 0.0

        # 機體座標=世界座標（跟上面 orientation 固定 identity 的假設一致），
        # 不需要再用 yaw 旋轉一次。
        v0, v1 = m.velocity[0], m.velocity[1]
        bx, by = v1, v0
        wz = -m.angular_velocity[2]

        stamp = self.get_clock().now().to_msg()

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = float(e)
        t.transform.translation.y = float(n)
        t.transform.translation.z = float(-d)
        t.transform.rotation.z = sz
        t.transform.rotation.w = cz
        self.tf.sendTransform(t)

        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id = 'odom'
        o.child_frame_id = 'base_link'
        o.pose.pose.position.x = float(e)
        o.pose.pose.position.y = float(n)
        o.pose.pose.position.z = float(-d)
        o.pose.pose.orientation.z = sz
        o.pose.pose.orientation.w = cz
        o.twist.twist.linear.x = float(bx)
        o.twist.twist.linear.y = float(by)
        o.twist.twist.angular.z = float(wz)
        self.pub.publish(o)


def main():
    rclpy.init()
    node = Px4OdomTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
