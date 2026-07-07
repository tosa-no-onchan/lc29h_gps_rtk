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
#if defined(USE_BUNKAI)
// NMEAの緯度経度を「度分秒」(DMS)の文字列に変換する
std::string NMEA2DMS(float val) {
  int d = val / 100;
  int m = ((val / 100.0) - d) * 100.0;
  float s = ((((val / 100.0) - d) * 100.0) - m) * 60;
  //return std::to_string(d) + "度" + std::to_string(m) + "分" + std::to_string(s, 1) + "秒";
  return std::to_string(d) + ":" + std::to_string(m) + ":" + std::to_string(s);
}
 
// (未使用)NMEAの緯度経度を「度分」(DM)の文字列に変換する
std::string NMEA2DM(float val) {
  int d = val / 100;
  float m = ((val / 100.0) - d) * 100.0;
  //return std::to_string(d) + "度" + std::to_string(m, 4) + "分";
  return std::to_string(d) + ":" + std::to_string(m) + ":";
}
 
// NMEAの緯度経度を「度」(DD)の文字列に変換する
std::string NMEA2DD(float val) {
  int d = val / 100;
  int m = (((val / 100.0) - d) * 100.0) / 60;
  float s = (((((val / 100.0) - d) * 100.0) - m) * 60) / (60 * 60);
  //return std::to_string(d + m + s, 6);
  return std::to_string(d + m + s);
}
 
// UTC時刻から日本の標準時刻に変換する(GMT+9:00)
std::string UTC2GMT900(std::string str) {
  //int hh = (str.substring(0,2).toInt()) + 9;
  int hh = (std::stoi(str.substr(0,2))) + 9;
  if(hh > 24) hh = hh - 24;
 
  //return std::to_string(hh,DEC) + ":" + str.substring(2,4) + ":" + str.substring(4,6);  
  return std::to_string(hh) + ":" + str.substr(2,4) + ":" + str.substr(4,6);  
}
#endif

//------------------------
// lc29h ROS2 Node
//------------------------
void Lc29hNode::init(std::shared_ptr<rclcpp::Node> node) {
  node_=node;
  gps.init(node_);

  first_f_=true;
  gnrmc_fix_=false;   // add by nishi 2026.7.4

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
  node_->declare_parameter<int>("rate",1); // [Hz]
  // add by nishi 2025.8.12
  node_->declare_parameter<bool>("use_dgns",true);
  node_->declare_parameter<int>("gga_num",0);
  // gps filter add by nishi 2024.4.16
  node_->declare_parameter<double>("filter",0.0);
  node_->declare_parameter<int>("count",0);

  node_->get_parameter<std::string>("device",device_);
  node_->get_parameter<std::string>("frame_id",frame_id_);
  node_->get_parameter<std::string>("topicName",fix_topic_);
  node_->get_parameter<int>("rate",rate_);
  node_->get_parameter<bool>("use_dgns",use_dgns_);
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

    // $GNRMC をチェックします add by nishi 2026.7.4
    # if defined(USE_GNPRMC)
      if( list[0] =="$GNRMC"){
        // Fix しています
        if(list[12] =="F"){
          gnrmc_fix_=true;
        }
        else{
          gnrmc_fix_=false;
        }
      }
    #endif

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

      # if defined(USE_GNPRMC)
        // $GNRMC が、 Fix しています!!
        if(gps.gga_status_==5 && gnrmc_fix_==true){
          gps.gga_status_=4;
          std::cout << "Force set GGA.status:" << gps.gga_status_ << std::endl;
        }
      #endif

      // GGA ステータス
      // gps.gga_status_: 0=測位不能、1=単独測位、2=DGPS、4=RTK fix、5=RTK float、6=デッドレコニング
      std::cout << "GGA.status:" << gps.gga_status_ << std::endl;

      //if(list[6] != "0"){      
      if(gps.gga_status_ != 0){

        //#define DEBUG_CHECK_NMEA
        double lati,longi;
        double altitude,latitude,longitude;
        try{
          //fix_->latitude  = std::stod(list[2])/100.0;  // 緯度
          //fix_->longitude = std::stod(list[4])/100.0;  // 経度
          //fix_->altitude  = std::stod(list[9]);  // 高度、海抜
          altitude  = std::stod(list[9]);  // 高度、海抜
          // chnaged by nishi 2025.8.9
          lati  = std::stod(list[2]);  // 緯度
          longi = std::stod(list[4]);  // 経度

          #if defined(DEBUG_CHECK_NMEA)
            std::cout <<"NMEA lat:"<<list[2] <<" longi:"<< list[4] <<std::endl;
            std::cout <<"lati:"<< std::fixed << std::setprecision(8) << lati << " longi:"<< std::fixed << std::setprecision(8) << longi << std::endl;
          #endif
          double lati_m = std::fmod(lati,100.0)/60.0;
          double longi_m = std::fmod(longi,100.0)/60.0;

          //fix_->latitude = std::floor(lati/100.0) + lati_m;   // 緯度
          //fix_->longitude = std::floor(longi/100.0) + longi_m;  // 経度
          latitude = std::floor(lati/100.0) + lati_m;   // 緯度
          longitude = std::floor(longi/100.0) + longi_m;  // 経度

          #if defined(DEBUG_CHECK_NMEA)
            std::cout <<"latitude:"<< std::fixed << std::setprecision(10) << latitude << " longitude:"<< longitude << std::endl;
          #endif
        }
        catch (...){
          // std::stof error
          return;
        }

        // ntrip server 用 gga を更新する。
        if(gps.set_ggaf_==false){
          // gps.set_gga() を実行する。
          gps.set_gga(data);
          gps.set_ggaf_=true;
        }

        bool pub_f=true;
        // set up Ros Publish part
        fix_ =std::make_shared<sensor_msgs::msg::NavSatFix>();
        //fix_->status.status  = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
        // update 2026.6.29 by nishi
        // fix_->status.status を実際に即した値にします。
        // NMEA $GNGGA のステータス(list[6])に応じてROS 2のステータスを厳格に打ち分ける
        // gps.gga_status_ : 1=単独, 2=DGPS, 4=RTK-Fix, 5=RTK-Float, 0=未測位(屋内など)
        if (gps.gga_status_ == 4) {
            // 【屋外・超高精度】RTK-Fix状態
            fix_->status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX; // 2
        } 
        else if (gps.gga_status_ == 5) {
            // 【屋外・環境による】RTK-Float状態
            // もし「Floatも完全に弾きたい（Fixのみ信頼する）」場合は、ここも STATUS_NO_FIX にしてください。
            // 今回は「Floatも一応通す（あとはUKFのマハラノビス距離に任せる）」形にしています。
            fix_->status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX; // 2
        }
        else if (gps.gga_status_ == 1 || gps.gga_status_ == 2) {
            // 通常の単独測位 / DGPS
            fix_->status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX; // 0
        } 
        else {
            // 【屋内など】測位不可、または異常値
            // これを設定すると navsat_transform_node や UKF は自動的にこのデータを無視します。
            fix_->status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX; // -1
        }

        fix_->altitude  = altitude;
        fix_->latitude = latitude;   // 緯度
        fix_->longitude = longitude;  // 経度

        fix_->status.service = 0;

        fix_->header.frame_id = frame_id_;

        #if defined(USE_BUNKAI)
          // 現在時刻
          //Serial.print(UTC2GMT900(list[1]));
          std::cout << UTC2GMT900(list[1]);
          
          // 緯度:latitude
          std::cout <<" 緯度 ";
          std::cout << list[2] << " ";

          std::cout << NMEA2DMS(std::stof(list[2]));
          std::cout << "(";
          std::cout << NMEA2DD(std::stof(list[2]));
          std::cout <<")";
  
          // 経度:longitude
          std::cout <<" 経度 ";
          std::cout << list[4] << " ";

          std::cout << NMEA2DMS(std::stof(list[4]));
          std::cout << "(";
          std::cout << NMEA2DD(std::stof(list[4]));
          std::cout <<")";
  
          // 海抜:altitude
          std::cout <<" 海抜 ";
          std::cout << list[9]; 
          //list[10].toLowerCase();
          std::cout <<list[10] << std::endl;
        #else
          //std::cout <<  data << std::endl;
        #endif

        fix_->header.stamp = node_->now();

        if(gps.gga_status_ != 4){
          // RTK float のときに、衛星数のチェックをする指定です。
          if(gga_num_ > 0){
            //std::cout << " gga_num:" << gga_num <<std::endl;
            if(gga_num < gga_num_){
              std::cout << " < gga_num:" << gga_num <<std::endl;
              //pub_f=false;
            }
            else{
              std::cout << " gga_num:" << gga_num <<std::endl;
            }
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

        //fix_->position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

        //fix_->position_covariance[0] = position_error_model_.drift.X()*position_error_model_.drift.X() + position_error_model_.gaussian_noise.X()*position_error_model_.gaussian_noise.X();
        //fix_->position_covariance[4] = position_error_model_.drift.Y()*position_error_model_.drift.Y() + position_error_model_.gaussian_noise.Y()*position_error_model_.gaussian_noise.Y();
        //fix_->position_covariance[8] = position_error_model_.drift.Z()*position_error_model_.drift.Z() + position_error_model_.gaussian_noise.Z()*position_error_model_.gaussian_noise.Z();

        // --- [ここから追加・修正] HDOPから共分散（誤差）を疑似計算する処理 ---
        // $GNGGA[8]
        double hdop = 1.0; // デフォルト値
        try {
          if (!list[8].empty()) {
            hdop = std::stod(list[8]); // カンマ8番目のHDOPを取得
          }
        }
        catch (...) {
          hdop = 1.0; // パース失敗時は安全のため1.0とする
        }

        // 測位状態に応じたベース精度（メートル）の設定
        double base_accuracy = 1.0; 
        if (gps.gga_status_ == 4) {
          base_accuracy = 0.02; // RTK-Fixならベース精度は約2cm
        } 
        else if (gps.gga_status_ == 5) {
          base_accuracy = 0.20; // RTK-Floatならベース精度は約20cm
        } 
        else {
          base_accuracy = 2.0;  // 単独測位などの場合は約2.0m
        }

        // 水平方向の標準偏差（誤差メートル）を算出
        double horizontal_std_dev = hdop * base_accuracy;
        // 高度方向は一般的に水平の1.5〜2倍程度大きくなるため
        double vertical_std_dev = horizontal_std_dev * 2.0; 

        // 分散（標準偏差の2乗）をROS 2メッセージにセット
        fix_->position_covariance[0] = horizontal_std_dev * horizontal_std_dev; // 経度(東)の分散
        fix_->position_covariance[4] = horizontal_std_dev * horizontal_std_dev; // 緯度(北)の分散
        fix_->position_covariance[8] = vertical_std_dev * vertical_std_dev;     // 高度(上)の分散

        // 対角成分（0, 4, 8番目）をセットしたことを示すタイプを指定
        fix_->position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
        // --- [ここまで追加・修正] ---

        if(pub_f==true)
          fix_publisher_->publish(*fix_);

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

