#include "lc29h_gps_rtk/gps.hpp"
#include <boost/version.hpp>


namespace lc29h_gps {
//using namespace lc29h_gps;

using namespace boost::placeholders;

//! Sleep time [ms] after setting the baudrate
constexpr static int kSetBaudrateSleepMs = 500;

const boost::posix_time::time_duration Gps::default_timeout_ =
    boost::posix_time::milliseconds(
        static_cast<int>(Gps::kDefaultAckTimeout * 1000));


//Gps::Gps() : configured_(false), config_on_startup_flag_(true) {
Gps::Gps(){
 //subscribeAcks();
}

//#define USE_READ_THREAD

Gps::~Gps() {
    ok = false;
    #if defined(USE_READ_THREAD)
      read_thread.join();
    #endif

    ntrip_client_.Stop();

    serial_->close();
   //close(); 
}

#if defined(USE_WORKER)
void Gps::setWorker(const boost::shared_ptr<Worker>& worker) {
  if (worker_) return;
  worker_ = worker;
  worker_->setCallback(boost::bind(&CallbackHandlers::readCallback,
                                   &callbacks_, _1, _2));
  configured_ = static_cast<bool>(worker);
}
#endif

// https://tips.hecomi.com/entry/20120728/1343504831
// https://blog.myon.info/entry/2015/04/19/boost-asio-serial/
void Gps::initializeSerial(std::string port, unsigned int baudrate,
                           uint16_t uart_in, uint16_t uart_out, int rate) {
  port_ = port;
  rate_ = rate;

  // measurement period [ms]
  meas_rate_ = 1000 / rate_;

  boost::shared_ptr<boost::asio::io_service> io_service(
      new boost::asio::io_service);

  boost::shared_ptr<boost::asio::serial_port> serial(
      new boost::asio::serial_port(*io_service));

  // open serial port
  try {
    serial->open(port);
  } 
  catch (std::runtime_error& e) {
    throw std::runtime_error("Lc29h: Could not open serial port :"
                             + port + " " + e.what());
  }

  //ROS_INFO("Gysfdmaxb: Opened serial port %s", port.c_str());
  RCLCPP_INFO(node_->get_logger(),"Lc29h: Opened serial port %s", port.c_str());
    
  if(BOOST_VERSION < 106600)
  {
    // NOTE(Kartik): Set serial port to "raw" mode. This is done in Boost but
    // until v1.66.0 there was a bug which didn't enable the relevant code,
    // fixed by commit: https://github.com/boostorg/asio/commit/619cea4356
    int fd = serial->native_handle();
    termios tio;
    tcgetattr(fd, &tio);
    cfmakeraw(&tio);
    tcsetattr(fd, TCSANOW, &tio);
  }

  #define SET_CONFIG
  #if defined(SET_CONFIG)
    // Set the I/O worker
    //if (worker_) return;
    //setWorker(boost::shared_ptr<Worker>(
    //    new AsyncWorker<boost::asio::serial_port>(serial, io_service)));

    configured_ = false;

    // Set the baudrate
    boost::asio::serial_port_base::baud_rate current_baudrate;
    serial->get_option(current_baudrate);
    // Incrementally increase the baudrate to the desired value
    for (int i = 0; i < sizeof(kBaudrates)/sizeof(kBaudrates[0]); i++) {
      if (current_baudrate.value() == baudrate)
        break;
      // Don't step down, unless the desired baudrate is lower
      if(current_baudrate.value() > kBaudrates[i] && baudrate > kBaudrates[i])
        continue;
      serial->set_option(
          boost::asio::serial_port_base::baud_rate(kBaudrates[i]));
      boost::this_thread::sleep(
          boost::posix_time::milliseconds(kSetBaudrateSleepMs));
      serial->get_option(current_baudrate);
      //ROS_DEBUG("Gysfdmaxb: Set ASIO baudrate to %u", current_baudrate.value());
      //ROS_INFO("Lc29h: Set ASIO baudrate to %u", current_baudrate.value());
      RCLCPP_INFO(node_->get_logger(),"Lc29h: Set ASIO baudrate to %u", current_baudrate.value());
    }
    //if (config_on_startup_flag_) {
    //  configured_ = configUart1(baudrate, uart_in, uart_out);
    //  if(!configured_ || current_baudrate.value() != baudrate) {
    //    throw std::runtime_error("Could not configure serial baud rate");
    //  }
    //} 
    //else {
    //  configured_ = true;
    //}
  #endif

  io_service_ =  io_service;
  serial_=serial;

  #define SET_OPTION
  #if defined(SET_OPTION)
    // https://blog.myon.info/entry/2015/04/19/boost-asio-serial/

    //ROS_INFO("Gysfdmaxb: set option");
    RCLCPP_INFO(node_->get_logger(),"Lc29h: set option");
    //std::cout << "Gysfdmaxb: set option" << std::endl;

    // テキトウに1秒待つ
    std::this_thread::sleep_for(std::chrono::seconds(1));

    //#define CHANGE_S_RATE
    #if defined(CHANGE_S_RATE)
      // set Baud Rate 115200 for gysfdamx
      std::string s_boaud ="$PMTK251,115200*1F\r\n";
      
      if(boost::asio::write(*serial_, boost::asio::buffer(s_boaud)) != s_boaud.length()){
        //ROS_ERROR("Gysfdmax: set Baud Rate error");
        RCLCPP_ERROR(node_->get_logger(),"Lc29h: set Baud Rate error");
        //std::cout << "Gysfdmax: set Baud Rate error" << std::endl;
        return;
      }

      //ROS_INFO("Gysfdmaxb: set Baud Rate OK");
      RCLCPP_INFO(node_->get_logger(),"Lc29h: set Baud Rate OK");
      //std::cout << "Gysfdmaxb: set Baud Rate OK" << std::endl;

      // テキトウに5秒待つ
      std::this_thread::sleep_for(std::chrono::seconds(5));

      // set Baud Rate 115200 for boost serial
      serial->set_option(
          boost::asio::serial_port_base::baud_rate(_baudrate));
      boost::this_thread::sleep(
          boost::posix_time::milliseconds(kSetBaudrateSleepMs));

      boost::asio::serial_port_base::baud_rate current_baudrate;
      serial->get_option(current_baudrate);

      //ROS_INFO("Gysfdmaxb: Get ASIO baudrate to %u", current_baudrate.value());
      RCLCPP_INFO(node_->get_logger(),"Lc29h: Get ASIO baudrate to %u", current_baudrate.value());
      //std::cout <<"Gysfdmaxb: Get ASIO baudrate to" << current_baudrate.value() << std::endl;
    #endif

    #if defined(SET_SAMPLE_DATA)
      // set sample data
      //std::string s_sample_form = "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n";
      std::string s_sample_form = "$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n";
      if(boost::asio::write(*serial_, boost::asio::buffer(s_sample_form)) != s_sample_form.length()){
        //ROS_ERROR("Gysfdmaxb: set sampling data error");
        RCLCPP_ERROR(node_->get_logger(),"Lc29h: set sampling data error");
        //std::cout << "Gysfdmaxb: set sampling data error" << std::endl;
        return;
      }
      //ROS_INFO("Gysfdmaxb: set sampling data OK");
      RCLCPP_INFO(node_->get_logger(),"Lc29h: set sampling data OK");
      //std::cout << "Gysfdmaxb: set sampling data OK" << std::endl;

      boost::this_thread::sleep(
            boost::posix_time::milliseconds(kSetBaudrateSleepMs));
    #endif

    //#define RESTORE_DEFAULT
    #if defined(RESTORE_DEFAULT)
      std::string s_restore_def="$PQTMRESTOREPAR*13";
      if(boost::asio::write(*serial_, boost::asio::buffer(s_restore_def)) != s_restore_def.length()){
        //RCLCPP_ERROR(node_->get_logger(),"Lc29h: set output rate error");
        std::cout << "Lc29h: restore default error" << std::endl;
        return;
      }

    #endif


    //#define SET_OUTPUT_RATE
    #if defined(SET_OUTPUT_RATE)
      // set samplling rate 2[Hz]
      //std::string s_sample_rate = "$PMTK220,500*2B\r\n";
      // set samplling rate 4[Hz]
      //std::string s_sample_rate = "$PMTK220,250*29\r\n";
      // set samplling rate 5[Hz]
      //std::string s_sample_rate = "$PMTK220,200*2C\r\n";
      // set samplling rate 6[Hz]
      //std::string s_sample_rate = "$PMTK220,166*2F\r\n";

      //RCLCPP_INFO(node_->get_logger(),"Lc29h: set outpuy rate %d",rate_);
      std::cout << "Lc29h: set output rate:" << rate_ << std::endl;

      std::string s_sample_rate = "$PQTMCFGFIXRATE,W,"+std::to_string(meas_rate_);
      checksum(s_sample_rate);
      std::cout << s_sample_rate << std::endl;

      if(boost::asio::write(*serial_, boost::asio::buffer(s_sample_rate)) != s_sample_rate.length()){
        //RCLCPP_ERROR(node_->get_logger(),"Lc29h: set output rate error");
        std::cout << "Lc29h: set output rate error" << std::endl;
        return;
      }

      //#define TEST_XCX
      #if defined(TEST_XCX)

      boost::this_thread::sleep(
            boost::posix_time::milliseconds(kSetBaudrateSleepMs));

      std::string s_save_parm1="$PAIR513*3D";
      if(boost::asio::write(*serial_, boost::asio::buffer(s_save_parm1)) != s_save_parm1.length()){
        //RCLCPP_ERROR(node_->get_logger(),"Lc29h: set output rate error");
        std::cout << "Lc29h: save_parm1 error" << std::endl;
        return;
      }

      boost::this_thread::sleep(
          boost::posix_time::milliseconds(kSetBaudrateSleepMs));

      std::string s_save_parm2="$PQTMSAVEPAR*5A";
      if(boost::asio::write(*serial_, boost::asio::buffer(s_save_parm2)) != s_save_parm2.length()){
        //RCLCPP_ERROR(node_->get_logger(),"Lc29h: set output rate error");
        std::cout << "Lc29h: save_parm2 error" << std::endl;
        return;
      }
      #endif
      boost::this_thread::sleep(
            boost::posix_time::milliseconds(kSetBaudrateSleepMs));

    #endif
    // テキトウに2秒待つ
    std::this_thread::sleep_for(std::chrono::seconds(2));

    //RCLCPP_INFO(node_->get_logger(),"Lc29h: set option end");
    std::cout << "Lc29h: set option end" << std::endl;

  #endif


  #if defined(USE_READ_THREAD)
    /* and turn on the streamer */
    ok = true;
    read_thread = boost::thread(boost::bind(&Gps::read_gps, this));
  #else
    // exec async_read_some();
    io_service_->post(boost::bind(&Gps::async_read_some, this));
    background_thread_.reset(new boost::thread(
        boost::bind(&boost::asio::io_service::run, io_service_)));
  #endif

}

void Gps::resetSerial(std::string port) {
  boost::shared_ptr<boost::asio::io_service> io_service(
      new boost::asio::io_service);
  boost::shared_ptr<boost::asio::serial_port> serial(
      new boost::asio::serial_port(*io_service));

  // open serial port
  try {
    serial->open(port);
  } 
  catch (std::runtime_error& e) {
    throw std::runtime_error("Lc29h: Could not open serial port :"
                             + port + " " + e.what());
  }

  //ROS_INFO("Gysfdmaxb: Reset serial port %s", port.c_str());
  RCLCPP_INFO(node_->get_logger(),"Lc29h: Reset serial port %s", port.c_str());

  #if defined(USE_WORKER_X)
    // Set the I/O worker
    if (worker_) return;
    setWorker(boost::shared_ptr<Worker>(
        new AsyncWorker<boost::asio::serial_port>(serial, io_service)));
    configured_ = false;

    // Poll UART PRT Config
    std::vector<uint8_t> payload;
    payload.push_back(CfgPRT::PORT_ID_UART1);
    if (!poll(CfgPRT::CLASS_ID, CfgPRT::MESSAGE_ID, payload)) {
      ROS_ERROR("Resetting Serial Port: Could not poll UART1 CfgPRT");
      return;
    }
    CfgPRT prt;
    if(!read(prt, default_timeout_)) {
      ROS_ERROR("Resetting Serial Port: Could not read polled UART1 CfgPRT %s",
                  "message");
      return;
    }

    // Set the baudrate
    serial->set_option(boost::asio::serial_port_base::baud_rate(prt.baudRate));
    configured_ = true;
  #endif
}

/*
*
* thread read_gps
*/
void Gps::read_gps(){
  // 受信データ
  boost::asio::streambuf response_buf;

  while(ok){

    // serial から response_buf に '\n' まで読み込む
    boost::asio::read_until(*serial_, response_buf, '\n');

    // https://faithandbrave.hateblo.jp/entry/20110324/1300950590
    // const 属性を外す
    // https://www.paveway.info/entry/2019/12/12/cpp_constcast

    const char *data_c =  boost::asio::buffer_cast<const char*>(response_buf.data());    // const char *
    //auto data_c =  boost::asio::buffer_cast<const char*>(response_buf.data());    // const char *

    //char *data_p =  const_cast<char*>(boost::asio::buffer_cast<const char*>(response_buf.data()));    // char *
    //std::string &data =  const_cast<std::string&>(boost::asio::buffer_cast<const char*>(response_buf.data()));    // char *
    //printf("%x",data);
    //std::string data = std::string(data_p);


    std::string data = std::string(data_c);

    if(call_back_f==true){
      fncStr_(data);
    }
  }
}

void Gps::async_read_some()
{
	if (serial_.get() == NULL || !serial_->is_open()) return;
	serial_->async_read_some( 
		boost::asio::buffer(read_buf_raw_, SERIAL_PORT_READ_BUF_SIZE),
		boost::bind(
			&Gps::on_receive,this,
      //_1,_2));
			boost::asio::placeholders::error, 
			boost::asio::placeholders::bytes_transferred));
}

void Gps::on_receive(const boost::system::error_code& ec, size_t bytes_transferred)
{
	boost::mutex::scoped_lock look(mutex_);

	if (serial_.get() == NULL || !serial_->is_open()) return;
	if (ec) {
		async_read_some();
		return;
	}

	for (unsigned int i = 0; i < bytes_transferred; ++i) {
		char c = read_buf_raw_[i];
		if (c == end_of_line_char_) {
			this->on_receive_all(nmea_str);
			nmea_str.clear();
		}
		else {
			nmea_str += c;
		}
	}
  //async_read_some();
  io_service_->post(boost::bind(&Gps::async_read_some, this));
}

void Gps::on_receive_all(std::string &data)
{
	//std::cout << data << std::endl;
  if(call_back_f==true){
    fncStr_(data);
  }
}

void Gps::checksum(std::string &data){
  //std::cout <<"data.length()=" << data.length() << std::endl;
  unsigned char arr[data.length() + 1]; 
	strcpy((char *)arr, data.c_str());
  unsigned char checksum=0;

  int l=data.length();
  for(int i=1;i<l;i++){
    checksum ^=arr[i];
  }
  //std::cout << "checksum" << std::hex << (unsigned int)checksum << std::endl;
  // https://www.techiedelight.com/ja/convert-an-integer-to-hex-string-in-cpp/
  data += "*"+((boost::format("%X") % (uint16_t)checksum).str())+"\r\n";
}

/*
* NtripClient
* callback_ntrip(const char *buffer, int size)
* 
*/
void Gps::callback_ntrip(const char *buffer, int size){
  if(gga_status_ == 4)
    std::cout << "F.RTCM["<< size << "]" << std::endl;
  else if(gga_status_ == 5)
    std::cout << "f.RTCM["<< size << "]" << std::endl;
  else
    std::cout << "RTCM["<< size << "]" << std::endl;

  if(boost::asio::write(*serial_, boost::asio::buffer(buffer,size)) != (std::size_t)size){
    //ROS_ERROR("Gysfdmaxb: set sampling rate error");
    std::cout << "Gysfdmaxb: send rtcm error" << std::endl;
  }
  rtcm_cnt_++;
  if(rtcm_cnt_ >= 60){
    rtcm_cnt_=0;
    //if(gga_status_ == 0){
      // gps 側に、 "$GNGGA" の取り込みを促す。
      set_ggaf_=false;
    //}
  }
}
/*
* NtripClient
* initializeNtripClient()
* 
*/
void Gps::initializeNtripClient(std::string ip,int port,std::string user,std::string passwd,std::string mountpoint,double latitude,double longitude){

  //double latitude;
  //double longitude;

  #if defined(DEFAULT_SETTING)
    std::string ip = "127.0.0.1";
    int port = 8090;
    std::string user = "test01";
    std::string passwd = "123456";
    std::string mountpoint = "RTCM32";
    latitude = 22.57311;
    longitude = 113.94905;
  #else
    //std::string ip = "183.178.46.135";
    //int port = 2101;
    //std::string user = "IBS";
    //std::string passwd = "IBS";
    //std::string mountpoint = "T430_32";
    //latitude = 50.09;
    //longitude = 8.66;
  #endif

  ntrip_client_.Init(ip, port, user, passwd, mountpoint);

  // callback ルーチン
  // ntrip server からの受信 RTCM データ
  // そのまま、lc29h へ送ります。
  #if defined(USE_ORG)
    ntrip_client_.OnReceived([] (const char *buffer, int size) {
      //printf("Recv[%d]: ", size);
      std::cout << "Recv["<< size << "]" << std::endl;
      //for (int i = 0; i < size; ++i) {
      //for (int i = 0; i < 10; ++i) {
      //  printf("%02X ", static_cast<uint8_t>(buffer[i]));
      //}
      //printf("\n");
    });
  #endif
  ntrip_client_.OnReceived(boost::bind(&Gps::callback_ntrip,this,_1,_2));

  // lc29h は、 "GNGGA" を下記でセットする。
  // std::string gga;
  // if (libntrip::GetGGAFrameData(22.57311, 113.94905, 10.0, &gga) == 0) {
  //   printf("GGA buffer: %s\n", gga.c_str());
  //   ntrip_client_.set_gga_buffer(gga);
  // }

  //  void set_location(double latitude, double longitude)
  ntrip_client_.set_location(latitude,longitude);
  //ntrip_client_.set_report_interval(1);
  // changed by nishi 2024.9.14
  ntrip_client_.set_report_interval(5);
  ntrip_client_.Run();
  std::this_thread::sleep_for(std::chrono::seconds(1));  // Maybe take longer?

  //int cnt=10;
  //while (ntrip_client_.service_is_running()) {
  //  std::this_thread::sleep_for(std::chrono::seconds(1));
  //  cnt--;
  //  if(cnt <=0)
  //    break;
  //}
  //ntrip_client_.Stop();

}

}