### lc29h_gps_rtk  
  
  ROS2 対応 lc29h GPS RTK publish node.  
  gps : [lc29hea](https://ja.aliexpress.com/item/1005006280471184.html?spm=a2g0o.productlist.main.21.56c1ZyHTZyHT25&algo_pvid=838ac7d5-7be4-4faf-bac1-391f2191bded&algo_exp_id=838ac7d5-7be4-4faf-bac1-391f2191bded-10&pdp_npi=4%40dis%21JPY%218520%218094%21%21%21400.00%21380.00%21%402101ef8717087634246034361e6f4e%2112000036592226693%21sea%21JP%210%21AB&curPageLogUid=F4yjv3rNSTbA&utparam-url=scene%3Asearch%7Cquery_from%3A)  
  
  ubuntu 22.04  
  PC and OrangePi 5 (Armbian and ubuntu 22.04)  
  ros2:humble  
  
  参照: [ROS2 LC29H-EA GPS RTK を作る。](http://www.netosa.com/blog/2024/04/ros2-lc29h-gps-rtk.html)  

#### 1. down load and build.  

    $ cd ~/colcon_ws/src  
    $ git clone https://github.com/tosa-no-onchan/lc29h_gps_rtk.git  
    $ cd ..  
    $ colcon build --symlink-install [--parallel-workers 1] --packages-select lc29h_gps_rtk  
    $ . install/setup.bash  
#### 2. update launch param  

    $ cd src/lc29h_gps_rtk/launch  
    $ mv lc29h_gps_rtk.launch.py-sample lc29h_gps_rtk.launch.py  
  
    edit launh.py  

````  
        # for ntrip_Client
        DeclareLaunchArgument('ip', default_value='183.178.46.135', description=''), 
        DeclareLaunchArgument('port', default_value='2101', description=''),
        DeclareLaunchArgument('user', default_value='IBS', description=''),
        DeclareLaunchArgument('passwd', default_value='IBS', description=''),
        DeclareLaunchArgument('mountpoint', default_value='T430_32', description=''),
        DeclareLaunchArgument('latitude', default_value='50.09', description=''),
        DeclareLaunchArgument('longitude', default_value='8.66', description=''),
````  

#### 3. run on SBC(Armibian orangePi 5)  

    $ sudo chmod 777 /dev/ttyUSB0  
    $ ros2 launch lc29h_gps_rtk lc29h_gps_rtk.launch.py  
    
    f.RTCM[nn] が出たら、RTK float OK  
    F.RTCM[nn] が出たら、 RTK Fix OK です。  

#### 4. check on Remote PC  

    $ sudo ufw disable  
    $ ros2 topic list  
    /fix  
    /parameter_events  
    /rosout  
    
    $ ros2 topic echo /fix  
    $ ros2 topic info /fix --verbose  

#### 5. setting  

  At GGA[6] (status) is 5:RTK float, You can let node to wait publish until GGA[7] (Numbers of satellite) value became more than gga_num value.  
  While acutual GGA[7] value is less than gga_num value, Node will not publish sensor_msgs::msg::NavSatFix at all.  

```
        DeclareLaunchArgument('gga_num', default_value='0', description='GGA satellite num in RTK float'),   # 17-30:recomend , 0: no check
```

#### 6. How to.  

  You can watch node's output "gga_num:17 - 21 over" in a terminal. Since then wait for a while about from 5 to 10 minutes over.  
  And then, although GGA[6] is 5:RTK float, LC29H-EA RTK will become to semi fix state   
  and start to put out the nearly correct values such as 100cm level. I think.   
  Please wait for it with patience.  
  You had better to wait RTK Fix.
  
     gga_num:nn  
     nn: 17 - 21 over   

  You can check with the following command ,and check whether if latitude and longitude value be fix.       
  
  $ ros2 topic echo /fix  

  Once LC29H-EA RTK becomes to semi fix state, you can move GPS freely.  


