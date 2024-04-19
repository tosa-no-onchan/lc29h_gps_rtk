### lc29h_gps_rtk  
  
  ROS2 対応 lc29h GPS RTK publish node.  
  gps : [lc29hea](https://ja.aliexpress.com/item/1005006280471184.html?spm=a2g0o.productlist.main.21.56c1ZyHTZyHT25&algo_pvid=838ac7d5-7be4-4faf-bac1-391f2191bded&algo_exp_id=838ac7d5-7be4-4faf-bac1-391f2191bded-10&pdp_npi=4%40dis%21JPY%218520%218094%21%21%21400.00%21380.00%21%402101ef8717087634246034361e6f4e%2112000036592226693%21sea%21JP%210%21AB&curPageLogUid=F4yjv3rNSTbA&utparam-url=scene%3Asearch%7Cquery_from%3A)  
  
  ubuntu 22.04  
  PC and OrangePi 5 (Armbian and ubuntu 22.04)  
  ros2:humble  
  
  参照: [SparkFun ZED-F9P NTRIPClient を試してみる。](http://www.netosa.com/blog/2024/04/sparkfun-zed-f9p-ntripclient.html)  

#### 1. down load and build.  

    $ cd ~/colcon_ws/src  
    $ git clone https://github.com/tosa-no-onchan/lc29h_gps_rtk.git  
    $ cd ..  
    $ colcon build --symlink-install [--parallel-workers 1] --packages-select lc29h_gps_rtk  
    $ . install/setup.bash  

#### 2. run on SBC(Armibian orangePi 5)  

    $ sudo chmod 777 /dev/ttyUSB0  
    $ ros2 launch lc29h_gps_rtk lc29h_gps_rtk.launch.py  

#### 3. check on Remote PC  

    $ sudo ufw disable  
    $ ros2 topic list  
    /fix  
    /parameter_events  
    /rosout  

    $ ros2 topic echo /fix  
    $ ros2 topic info /fix --verbose  


