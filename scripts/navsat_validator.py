#!/usr/bin/env python3
# lc29h_gps_rtk/scripts/navsat_validator.py
# 1. build
#  $ colcon build --symlink-install --parallel-workers 1 --packages-select lc29h_gps_rtk
#  $ . install/setup.bash
#
# 2. execute
#  $ ros2 launch lc29h_gps_rtk lc29h_gps_rtk.launch.py
#
#  $ ros2 run lc29h_gps_rtk navsat_validator.py
#

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped  # RViz2表示用
# QoS設定用のライブラリをインポート
from rclpy.qos import qos_profile_sensor_data

class NavSatValidator(Node):
    def __init__(self):
        super().__init__('navsat_validator')
        
        self.subscription = self.create_subscription(
            NavSatFix, '/fix', self.listener_callback, qos_profile_sensor_data)
        
        # RViz2が直接描画できるPose型のPublisherを追加
        self.pose_pub = self.create_publisher(PoseStamped, '/gps_pose', 10)
        
        # 基準となる初期位置（最初の1点目を0,0にする）
        self.init_lat = None
        self.init_lon = None
        
        # 地球の半径 (メートル)
        self.EARTH_RADIUS = 6378137.0

        # 許容する最大位置誤差 (メートル)
        self.max_allowed_error = 0.05 

    def listener_callback(self, msg: NavSatFix):
        # 1. 測位ステータスの検証
        status = msg.status.status
        if status == msg.status.STATUS_NO_FIX:
            self.get_logger().error('GNSS status: NO FIX (データが無効です)')
            return
        elif status == msg.status.STATUS_GBAS_FIX:
            status_str = 'GBAS/RTK FIX (高精度)'
        else:
            status_str = 'Standard FIX (単独測位/SBAS)'

        # 2. 誤差（共分散）の検証
        lon_variance = msg.position_covariance[0]
        lat_variance = msg.position_covariance[4]

        # 受信機によっては共分散を計算せずすべて0.0で出力する仕様のものがあります
        if lon_variance == 0.0 and lat_variance == 0.0:
            self.get_logger().info(
                f'[{status_str}] データ受信成功. (注意: 受信機が誤差情報を出力していません)'
            )
            return

        horizontal_error = math.sqrt(lon_variance + lat_variance)

        # 【追加】共分散の大きさから、内部でFixかFloatかを推測して文字分ける
        # 誤差が5cm(0.05m)未満ならFix、それ以上ならFloatとみなす
        if horizontal_error < 0.04:
            status_str = 'RTK-FIX (センチメートル完全固定)'
        else:
            status_str = 'RTK-FLOAT (位置が漂流中・不完全)'

        # 標準偏差（メートル単位の推定誤差）を計算
        horizontal_error = math.sqrt(lon_variance + lat_variance)

        # 3. 総合判定の出力
        if horizontal_error > self.max_allowed_error:
            # 許容値（5cm）を超えているので警告を出す
            #self.get_logger().warn(
            print(
                f'[{status_str}] 警告: 誤差が大きすぎます! '
                f'現在誤差: {horizontal_error:.3f}m (許容: {self.max_allowed_error}m)'
            )
        else:
            #self.get_logger().info(
            print(
                f'[{status_str}] データ正常. '
                f'推定位置誤差: {horizontal_error:.3f}m'
            )

        # 最初の1点目を原点として記憶
        if self.init_lat is None:
            self.init_lat = msg.latitude
            self.init_lon = msg.longitude
            #self.get_logger().info(f'基準位置を設定しました: Lat={self.init_lat}, Lon={self.init_lon}')
            print(f'基準位置を設定しました: Lat={self.init_lat}, Lon={self.init_lon}')
            return

        # 緯度・経度の差分をメートルに変換 (簡易平面投影)
        rad_lat = math.radians(self.init_lat)
        y = (msg.latitude - self.init_lat) * (math.pi / 180.0) * self.EARTH_RADIUS
        x = (msg.longitude - self.init_lon) * (math.pi / 180.0) * self.EARTH_RADIUS * math.cos(rad_lat)

        # RViz2用のメッセージを作成
        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = 'map'  # 固定座標系をmapに指定
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0  # 回転は正面固定
        
        self.pose_pub.publish(pose)
        #self.get_logger().info(f'アンテナ位置(cm) X: {x*100:.1f}cm, Y: {y*100:.1f}cm')
        print(f'アンテナ位置(cm) X: {x*100:.1f}cm, Y: {y*100:.1f}cm')

def main(args=None):
    rclpy.init(args=args)
    node = NavSatValidator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

