/*-----------------------------------
- lc29h_gps
-   node.cpp
-
- みちびきのデータ形式は、下記参照
- https://www.petitmonte.com/robot/howto_gysfdmaxb.html
-
------------------------------------*/

#include "lc29h_gps_rtk/node.hpp"

using namespace lc29h_gps;

using namespace boost::placeholders;

//#define USE_BUNKAI

#ifdef USE_BUNKAI
// NMEAの緯度経度を「度分秒」(DMS)の文字列に変換する
std::string NMEA2DMS(float val) {
  int d = val / 100;
  int m = ((val / 100.0) - d) * 100.0;
  float s = ((((val / 100.0) - d) * 100.0) - m) * 60;
  return std::to_string(d) + "度" + std::to_string(m) + "分" + std::to_string(s, 1) + "秒";
}
 
// (未使用)NMEAの緯度経度を「度分」(DM)の文字列に変換する
std::string NMEA2DM(float val) {
  int d = val / 100;
  float m = ((val / 100.0) - d) * 100.0;
  return std::to_string(d) + "度" + std::to_string(m, 4) + "分";
}
 
// NMEAの緯度経度を「度」(DD)の文字列に変換する
std::string NMEA2DD(float val) {
  int d = val / 100;
  int m = (((val / 100.0) - d) * 100.0) / 60;
  float s = (((((val / 100.0) - d) * 100.0) - m) * 60) / (60 * 60);
  return std::to_string(d + m + s, 6);
}
 
// UTC時刻から日本の標準時刻に変換する(GMT+9:00)
std::string UTC2GMT900(std::string str) {
  int hh = (str.substring(0,2).toInt()) + 9;
  if(hh > 24) hh = hh - 24;
 
  return std::to_string(hh,DEC) + ":" + str.substring(2,4) + ":" + str.substring(4,6);  
}
#endif

//------------------------
// lc29h ROS2 Node
//------------------------   
//GysfdmaxbNode::GysfdmaxbNode(rclcpp::NodeOptions const & options)
//: rclcpp::Node{"gysfdmaxb_gps", options}
//{
//  //init();
//}

void Lc29hNode::init(std::shared_ptr<rclcpp::Node> node) {

  node_=node;
  gps.init(node_);

  first_f_=true;

  // Params must be set before initializing IO
  getRosParams();

  // set up Ros Publish part
  //fix_ =std::make_shared<sensor_msgs::msg::NavSatFix>();
  ////fix_.status.status  = sensor_msgs::NavSatStatus::STATUS_FIX;
  //fix_->status.status  = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  //fix_->status.service = 0;
  //fix_->header.frame_id = frame_id_;

  //fix_publisher_ = nh->advertise<sensor_msgs::NavSatFix>(fix_topic_, 10);
  // https://yhrscoding.hatenablog.com/entry/2021/09/26/104445
  rclcpp::QoS gps_qos(10);
  //gps_qos.reliable();   // 2024.4.11 もともとは、こちら
  gps_qos.best_effort();  // こちらはどうか? by nishi 2024.4.12
  gps_qos.durability_volatile();
  fix_publisher_ = node_->create_publisher<sensor_msgs::msg::NavSatFix>(fix_topic_, gps_qos);

  initializeIo();

  // subscribe GPS raw data
  //gps.subscribe_gps(GysfdmaxbNode::subscribe_gps);


  //gps.fncStr_=publish_void_str;
  //gps.fncStr_=boost::bind(&GysfdmaxbNode::publish_void_str,_1);
  //gps.subscribe_str(boost::bind(&GysfdmaxbNode::publish_void_str,_1));


  //gps.subscribe_char(boost::bind(&GysfdmaxbNode::publish_void_char,this,_1));
  //rtk_fix_f_=false;
  time_prev_ = std::chrono::system_clock::now();
  speed_over_cnt_=0;

  // subscribe GPS nmea data and Publish
  gps.subscribe_str(boost::bind(&Lc29hNode::publish_nmea_str,this,_1));

}

void Lc29hNode::initializeIo(){
  gps.setConfigOnStartup(config_on_startup_flag_);

  gps.initializeSerial(device_, baudrate_, uart_in_, uart_out_,rate_);

  gps.initializeNtripClient(ip_,port_,user_,passwd_,mountpoint_,latitude_,longitude_);

}

void Lc29hNode::getRosParams() {
  node_->declare_parameter<std::string>("device",std::string("/dev/ttyUSB0"));
  node_->declare_parameter<std::string>("frame_id",std::string("gps_link"));
  node_->declare_parameter<std::string>("topicName",std::string("fix"));

  set_usb_ = false;

  // Measurement rate params
  node_->declare_parameter<int>("rate",5); // [Hz]
  node_->declare_parameter<int>("gga_num",0);
  // gps filter add by nishi 2024.4.16
  node_->declare_parameter<double>("filter",0.0);
  node_->declare_parameter<int>("count",0);


  node_->get_parameter<std::string>("device",device_);
  node_->get_parameter<std::string>("frame_id",frame_id_);
  node_->get_parameter<std::string>("topicName",fix_topic_);
  node_->get_parameter<int>("rate",rate_);
  node_->get_parameter<int>("gga_num",gga_num_);
  node_->get_parameter<double>("filter",filter_);  // add by nishi 2024.4.16
  node_->get_parameter<int>("count",count_);  // add by nishi 2024.4.20

  // for ntrip_client
  node_->declare_parameter<std::string>("ip",std::string("183.178.46.135"));
  node_->declare_parameter<int>("port",2101);
  node_->declare_parameter<std::string>("user",std::string("IBS"));
  node_->declare_parameter<std::string>("passwd",std::string("IBS"));
  node_->declare_parameter<std::string>("mountpoint",std::string("T430_32"));
  node_->declare_parameter<double>("latitude",50.09);
  node_->declare_parameter<double>("longitude",8.66);

  node_->get_parameter<std::string>("ip",ip_);
  node_->get_parameter<int>("port",port_);
  node_->get_parameter<std::string>("user",user_);
  node_->get_parameter<std::string>("passwd",passwd_);
  node_->get_parameter<std::string>("mountpoint",mountpoint_);
  node_->get_parameter<double>("latitude",latitude_);
  node_->get_parameter<double>("longitude",longitude_);

  //if (enable_ppp_)
  //  ROS_WARN("Warning: PPP is enabled - this is an expert setting.");


  // measurement period [ms]
  //meas_rate = 1000 / rate_;

  // activate/deactivate any config
  //nh->param("config_on_startup", config_on_startup_flag_, true);

  // raw data stream logging 
  //rawDataStreamPa_.getRosParams();
}

//------------------------------------
// subscribe GPS nmea data and Publish
//------------------------------------
void Lc29hNode::publish_nmea_str(std::string& data) {
  //static ros::Publisher publisher = nh->advertise<nmea_msgs::Sentence>(topic,
  //                                                          kROSQueueSize);
  //nmea_msgs::Sentence m;
  //m.header.stamp = ros::Time::now();
  //m.header.frame_id = frame_id;
  //m.sentence = sentence;
  //publisher.publish(m);
  
  //std::cout << ">>> called publish_nmea_str() \n" << data << std::endl;
  //std::cout <<  data << std::endl;

  if(data != ""){
    int i, index = 0, len = data.length();
    std::string str = "";
  
    // StringListの生成(簡易)
    //std::string list[30];
    int lng=50;
    std::string list[lng];
    for (i = 0; i < lng; i++) {
      list[i] = "";
    }


    // 「,」を区切り文字として文字列を配列にする
    for (i = 0; i < len; i++) {
      if (data[i] == ',') {
        list[index++] = str;
        if(index >= lng)
          break;
        str = "";
        continue;
      }
      str += data[i];
    }

    // https://ales-corp.co.jp/technical-information-nmea/    
    // $GPGGAセンテンスのみ読み込む
    if (list[0] == "$GNGGA" && index < 30) {
      //std::cout << " get $GNGGA:" << std::endl;
      //std::cout << " list[6]:" << list[6] << std::endl;
      int gga_num;
      try{
        gps.gga_status_=std::stoi(list[6]);
        gga_num = std::stoi(list[7]);
      }
      //catch(std::invalid_argument &e){
      catch (...){
        gps.gga_status_=0;
        gga_num = 0;
      }

      // ステータス
      //if(list[6] != "0"){      
      if(gps.gga_status_ != 0){      
        // ntrip server 用 gga を更新する。
        if(gps.set_ggaf_==false){
          // gps.set_gga() を実行する。
          gps.set_gga(data);
          gps.set_ggaf_=true;
        }

        // RTK fix まだです。
        if(gps.gga_status_ != 4 && gps.gga_status_ != 5){
          return;
        }

        if(gps.gga_status_ == 0){
          //std::cout << " $GNGGA Fix complete!! status:" << list[6] << std::endl;
          //gps.rtk_fix_f_=true;
          // test もう一度だけ、 gps.set_gga() を実行する。by nishi 2024.4.19
          //gps.set_ggaf_=false;
        }

        bool pub_f=true;
        // set up Ros Publish part
        fix_ =std::make_shared<sensor_msgs::msg::NavSatFix>();
        //fix_.status.status  = sensor_msgs::NavSatStatus::STATUS_FIX;
        fix_->status.status  = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
        fix_->status.service = 0;

        fix_->header.frame_id = frame_id_;

        #ifdef USE_BUNKAI

        // 現在時刻
        //Serial.print(UTC2GMT900(list[1]));
        std::cout << UTC2GMT900(list[1]);
        
        // 緯度:latitude
        std::cout <<" 緯度:";

        std::cout << NMEA2DMS(std::stof(list[2]));
        std::cout << "(";
        std::cout << NMEA2DD(std::stof(list[2]));
        std::cout <<")";
 
        // 経度:longitude
        std::cout <<" 経度:";
        std::cout << NMEA2DMS(std::stof(list[4]));
        std::cout << "(";
        std::cout << NMEA2DD(std::stof(list[4]));
        std::cout <<")";
 
        // 海抜:altitude
        std::cout <<" 海抜:";
        std::cout << list[9]; 
        //list[10].toLowerCase();
        std::cout <<list[10] << std::endl;
        #else
        //std::cout <<  data << std::endl;

        #endif

        fix_->header.stamp = node_->now();
        try{
          fix_->latitude  = std::stof(list[2])/100.0;  // 緯度
          fix_->longitude = std::stof(list[4])/100.0;  // 経度
          fix_->altitude  = std::stof(list[9]);  // 高度、海抜
        }
        catch (...){
          // std::stof error
          return;
        }
        // RTK float です。
        if(gps.gga_status_ == 5){
          // RTK float のときに、衛星数のチェックをする指定です。
          if(gga_num_ > 0){
            //std::cout << " gga_num:" << gga_num <<std::endl;
            if(gga_num < gga_num_){
              std::cout << " < gga_num:" << gga_num <<std::endl;
              pub_f=false;
            }
            else{
              std::cout << " gga_num:" << gga_num <<std::endl;
            }
          }
          // RTK float のときに、filter の指定があります。 add by nihsi 2024.4.16
          else if(filter_ > 0.0 && count_ > 0){

            // https://www.rk-k.com/archives/6973
            auto time_now = std::chrono::system_clock::now();
            //std::chrono::duration<double, std::milli> time_off_mil = time_now_ - time_prev_;
            std::chrono::duration<double> time_off_sec = time_now - time_prev_;

            //std::cout << time_c_off_mil.count() << "ms" << std::endl;
            //std::cout << time_off_sec.count() << "s" << std::endl;

            double time_offs=time_off_sec.count();

            //std::cout <<" time_offs:"<<time_offs <<std::endl;

            if(first_f_==true){
              first_f_=false;
              latitude_prev_=fix_->latitude;
              longitude_prev_=fix_->longitude;
              altitude_prev_=fix_->altitude;
            }
            // 緯度(北緯 latitude)、経度(東経 longitude)の距離
            // https://www.wingfield.gr.jp/archives/9721
            // 自分の緯度から、上記ページの	経度1度長 (m)	緯度1度長 (m)を調べて変更して下さい。
            // 緯度	経度1度長 (m)	緯度1度長 (m)
            // 33	93452.8629	110904.4688	1557.5477	1848.4078	0.0000107006	0.0000090168
            // 移動距離を計算
            double off_latitude=(fix_->latitude - latitude_prev_) * 110904.4688;    // 距離[m]
            //double off_latitude=(fix_->latitude - latitude_prev_);    // 角度
            double off_longitude=(fix_->longitude - longitude_prev_) * 93452.8629;  // 距離[m]
            //double off_longitude=(fix_->longitude - longitude_prev_);  // 角度
            double off_altitude=fix_->altitude - altitude_prev_;

            //std::cout <<" off_latitude:"<< off_latitude <<std::endl;
            //std::cout <<" off_longitude:"<< off_longitude <<std::endl;

            //https://cpprefjp.github.io/reference/cmath/hypot.html
            //double speed = std::hypot(off_latitude, off_longitude, off_altitude)/time_offs;
            //double speed = std::hypot(off_latitude, off_longitude)/time_offs;  // 移動速度  [m/s]
            double speed = std::sqrt(off_latitude*off_latitude+off_longitude*off_longitude)/time_offs;  // 移動速度  [m/s]

            //std::cout <<" offs:"<< offs <<std::endl;
            // フィルターより移動速度が大きい
            // ロボットの移動時に、GPSの横飛びが生じると誤判定になります。
            if(speed > filter_){
              pub_f=false;
              speed_over_cnt_++;
              std::cout <<" over speed:"<< speed <<" speed_over_cnt_:" << speed_over_cnt_ << " gga_num:" << gga_num <<std::endl;
              if(speed_over_cnt_>= count_){
                //std::cout <<" over speed:"<< speed <<" speed_over_cnt_:" << speed_over_cnt_ <<std::endl;
                pub_f=true;
                speed_over_cnt_=0;
              }
              else{
                fix_->latitude = latitude_prev_;
                fix_->longitude = longitude_prev_;
                //fix_->altitude = altitude_prev_;
              }
            }
            else{
                speed_over_cnt_=0;
                std::cout <<" speed:"<< speed <<" gga_num:" << gga_num <<std::endl;
            }
            time_prev_=time_now;
            latitude_prev_ = fix_->latitude;
            longitude_prev_ = fix_->longitude;
            altitude_prev_ = fix_->altitude;
          }
          // test by nishi 2024.4.22
          else{
              std::cout << " gga_num:" << gga_num <<std::endl;
          }
        }

        //float64[9] position_covariance
        //# If the covariance of the fix is known, fill it in completely. If the
        //# GPS receiver provides the variance of each measurement, put them
        //# along the diagonal. If only Dilution of Precision is available,
        //# estimate an approximate covariance from that.

        //uint8 COVARIANCE_TYPE_UNKNOWN = 0
        //uint8 COVARIANCE_TYPE_APPROXIMATED = 1
        //uint8 COVARIANCE_TYPE_DIAGONAL_KNOWN = 2
        //uint8 COVARIANCE_TYPE_KNOWN = 3
        //uint8 position_covariance_type

        fix_->position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

        //fix_->position_covariance[0] = position_error_model_.drift.X()*position_error_model_.drift.X() + position_error_model_.gaussian_noise.X()*position_error_model_.gaussian_noise.X();
        //fix_->position_covariance[4] = position_error_model_.drift.Y()*position_error_model_.drift.Y() + position_error_model_.gaussian_noise.Y()*position_error_model_.gaussian_noise.Y();
        //fix_->position_covariance[8] = position_error_model_.drift.Z()*position_error_model_.drift.Z() + position_error_model_.gaussian_noise.Z()*position_error_model_.gaussian_noise.Z();

        if(pub_f==true)
          fix_publisher_->publish(*fix_);

        //std::cout << " publish()" << std::endl;

      }
      else{
        //Serial.print("測位できませんでした。");
      }
      
      //Serial.println("");
    }
  }


}

int main(int argc, char** argv) {

  using namespace std::chrono_literals;
  rclcpp::WallRate loop(1);

  //ros::init(argc, argv, "gysfdmaxb_gps");
  //nh.reset(new ros::NodeHandle("~"));

  rclcpp::init(argc, argv);

  //ros::Subscriber subRtcm = nh->subscribe("/rtcm", 10, rtcmCallback);
  //nh->param("debug", ublox_gps::debug, 1);
  //if(ublox_gps::debug) {
  //  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
  //                                     ros::console::levels::Debug))
  //   ros::console::notifyLoggerLevelsChanged();
  //
  //}
  #ifdef ROS1_USE
  Lc29hNode node;
  ros::spinOnce();
  ros::Rate rate(1);   //  1[Hz]

  //int i=0;

  //while(1){
  //  gps.read_line();
  //  ros::spinOnce();
  //  i++;
  //
  //  if(i > 50)
  //    break;
  //  rate.sleep();
  //}

  ros::spin();
  #endif

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("lc29h_gps",rclcpp::NodeOptions{});

  //rclcpp::spin(std::make_shared<Lc29h_gps::Lc29hbNode>(rclcpp::NodeOptions{}));

  Lc29hNode lc29h;
  lc29h.init(node);

  while(rclcpp::ok()){
    loop.sleep();
  }


  rclcpp::shutdown();

  return 0;
}

