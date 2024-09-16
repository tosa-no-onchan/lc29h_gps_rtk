# lc29h_gps_rtk.launch.py
# 1. build
#  $ colcon build --symlink-install --parallel-workers 1 --packages-select lc29h_gps_rtk
#  $ . install/setup.bash
#
# 2. execute
#  $ ros2 launch lc29h_gps_rtk lc29h_gps_rtk.launch.py
#
# 3. check topic
#  $ ros2 topic info /fix --verbose
#
# free mount point
# Name:	geortk.jpName:	geortk.jp
# Address: 160.16.132.128
#
# https://geortk.jp/mountpoint -> NG user/passwd が必要か?
# jf6dir 	Miyaki-Saga 	RTCM 3.3 	1005(1),1077(1),1087(1),1097(1),1127(1),1230(1) 	JPN 	33.33 	130.45 
# kawano3band 	Fukuoka-miyama 	RTCM 3.3 	1005(1),1008(1),1075(1),1085(1),1095(1),1115(1),1125(1),1230(1) 	JPN 	33.10 	130.46 
#
# https://rtk.silentsystem.jp/  -> OK
# 1)大分県豊後大野市三重町	大分県農林水産研究指導センター	32.98991237	131.5946694	201.9756011	160.16.134.72	2101	RTCM3	Ntrip	公開  -> OK
# 2)香川県三豊市	香川高専詫間キャンパス	34.23432896	133.6367415	62.25	rtk2go.com(3.143.243.81)	2101	u-blox	Ntrip	公開	○  -> NG
# RTK2go.com」の仕様変更により、ID部分に利用者のメールアドレス要入力(2022.11)
# 3) 大分市	大分工業高等専門学校	33.23271329	131.6515685	120.8720128	160.16.134.72	2101  oita-kosen   -> OK
#import launch
#import launch_ros.actions

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([

        # for lc29h
        DeclareLaunchArgument('device', default_value='/dev/ttyUSB0', description=''),
        DeclareLaunchArgument('frame_id', default_value='gps_link', description=''),
        DeclareLaunchArgument('rate', default_value='5', description=''),          #  1/2/5/10
        DeclareLaunchArgument('topicName', default_value='fix', description=''),
        DeclareLaunchArgument('gga_num', default_value='17', description='supress GGA satellite num in RTK float'),   # 16-33:recomend , 0: no check
        DeclareLaunchArgument('filter', default_value='0.0', description='filtered max speed'),  # robot max speed + x [m/s]
        #DeclareLaunchArgument('filter', default_value='0.6', description='filtered max speed'),  # robot max speed + x [m/s]
        DeclareLaunchArgument('count', default_value='10', description='filtered max count'),

        # for ntrip_Client
        #DeclareLaunchArgument('ip', default_value='183.178.46.135', description='server ip'), 
        #DeclareLaunchArgument('ip', default_value='160.16.134.72', description='server ip'), 
        #DeclareLaunchArgument('ip', default_value='3.143.243.81', description='server ip'), 
        DeclareLaunchArgument('ip', default_value='160.16.134.72', description='server ip'), 
        DeclareLaunchArgument('port', default_value='2101', description='port'),
        #DeclareLaunchArgument('user', default_value='IBS', description='user id'),
        DeclareLaunchArgument('user', default_value='', description='user id'),
        #DeclareLaunchArgument('passwd', default_value='IBS', description='passworrd'),
        DeclareLaunchArgument('passwd', default_value='', description='passworrd'),
        #DeclareLaunchArgument('mountpoint', default_value='T430_32', description='mountpoint'),
        #DeclareLaunchArgument('mountpoint', default_value='oita-mie', description='mountpoint'),
        DeclareLaunchArgument('mountpoint', default_value='oita-kosen', description='mountpoint'),
        #DeclareLaunchArgument('latitude', default_value='50.09', description=''),
        DeclareLaunchArgument('latitude', default_value='33.39', description=''),
        #DeclareLaunchArgument('longitude', default_value='8.66', description=''),
        DeclareLaunchArgument('longitude', default_value='133.25', description=''),

        Node(
            package='lc29h_gps_rtk',
            executable='lc29h_gps_rtk_node',
            output="screen",
            #emulate_tty=True,
            parameters=[
                        {"device": LaunchConfiguration('device') ,
                         "frame_id": LaunchConfiguration('frame_id'),
                         "rate": LaunchConfiguration('rate'),
                         "topicName": LaunchConfiguration('topicName'),
                         "gga_num": LaunchConfiguration('gga_num'),
                         "filter": LaunchConfiguration('filter'),
                         "count": LaunchConfiguration('count'),
                         # for ntrip_client
                         "ip": LaunchConfiguration('ip'),
                         "port": LaunchConfiguration('port'),
                         "user": LaunchConfiguration('user'),
                         "passwd": LaunchConfiguration('passwd'),
                         "mountpoint": LaunchConfiguration('mountpoint'),
                         "latitude": LaunchConfiguration('latitude'),
                         "longitude": LaunchConfiguration('longitude'),
                         }
            ]
        )
    ])

'''
#https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-CPP.html
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="cpp_parameters",
            executable="minimal_param_node",
            name="custom_minimal_param_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"my_parameter": "earth"}
            ]
        )
    ])
'''