
/*--------------------------------------------------
- gps.h
-
- https://dormolin.livedoor.blog/archives/52018720.html
- https://ez-net.jp/article/6A/su9HMTVF/wKanc4EjbZFS/
- https://gist.github.com/yoggy/3323808
- https://rowan.hatenadiary.org/entry/20090825/p1
-
----------------------------------------------------*/

#ifndef LC29H_GPS_GPS_HPP
#define LC29H_GPS_GPS_HPP


// Boost
#include <boost/asio.hpp>
#include <boost/asio/placeholders.hpp>

//#include <boost/asio/ip/tcp.hpp>
//#include <boost/asio/ip/udp.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/atomic.hpp>

#include <boost/thread.hpp>

#include <boost/function.hpp>
//#include <boost/bind.hpp>
#include <boost/bind/bind.hpp>

// ROS includes
//#include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>


#include <boost/format.hpp>

#include <boost/array.hpp>

// Other u-blox packages
//#include <ublox/serialization/ublox_msgs.h>
// u-blox gps
//#include <gysfdmaxb_gps/async_worker.h>
//#include <gysfdmaxb_gps/callback.h>


#include <stdint.h>

#include <string>
#include <vector>

#include "ntrip/ntrip_client.h"
#include "ntrip/ntrip_util.h"


namespace lc29h_gps {

typedef void (*fSub)(const char *);
typedef void (*fNmea)(const std::string&);


typedef  std::function<void(char *)> FncChar;
typedef  boost::function<void(std::string&)> FncStr;


//! Possible baudrates for u-blox devices
constexpr static unsigned int kBaudrates[] = { //4800,
                                               //9600,
                                               19200,
                                               38400,
                                               57600,
                                               115200,
                                               230400,
                                               460800 };
/**
 * @brief Handles communication with and configuration of the u-blox device
 */
class Gps {
 public:
  //! Default timeout for ACK messages in seconds
  constexpr static double kDefaultAckTimeout = 1.0;
  //! Size of write buffer for output messages
  constexpr static int kWriterSize = 2056;

  Gps();
  virtual ~Gps();

  void init(std::shared_ptr<rclcpp::Node> node){
    node_=node;
    // add by nishi 2024.4.21
    gga_status_=0;
  }

  /**
   * @brief Set the internal flag for enabling or disabling the initial configurations.
   * @param config_on_startup boolean flag
   */
  void setConfigOnStartup(const bool config_on_startup) { config_on_startup_flag_ = config_on_startup; }

  /**
   * @brief Initialize the Serial I/O port.
   * @param port the device port address
   * @param baudrate the desired baud rate of the port
   * @param uart_in the UART In protocol, see CfgPRT for options
   * @param uart_out the UART Out protocol, see CfgPRT for options
   */
  void initializeSerial(std::string port, unsigned int baudrate,
                        uint16_t uart_in, uint16_t uart_out, int rate=1);



  /**
   * @brief Reset the Serial I/O port after u-blox reset.
   * @param port the device port address
   * @param baudrate the desired baud rate of the port
   * @param uart_in the UART In protocol, see CfgPRT for options
   * @param uart_out the UART Out protocol, see CfgPRT for options
   */
  void resetSerial(std::string port);

  /**
   * @brief Send rtcm correction message.
  */
  //bool sendRtcm(const std::vector<uint8_t> &message);

  /**
   * @brief Closes the I/O port, and initiates save on shutdown procedure
   * if enabled.
   */
  void close();

  /**
   * @brief Reset I/O communications.
   * @param wait Time to wait before restarting communications
   */
  void reset(const boost::posix_time::time_duration& wait);

  void subscribe_gps(const fSub& fsub){
    fsub_=fsub;
    call_back_f=true;
  }

  void subscribe_namea(const fNmea& fnamea){
    fnamea_=fnamea;
    call_back_f=true;
  }

  void subscribe_char(FncChar fncChar){
    fncChar_=fncChar;
    call_back_f=true;
  }

  void subscribe_str(FncStr fncStr){
    fncStr_=fncStr;
    call_back_f=true;
  }

  void read_gps();

  //void on_receive();
  void async_read_some();

  void on_receive(const boost::system::error_code& ec, size_t bytes_transferred);
  void on_receive_all(std::string &data);

  void checksum(std::string &data);

  void set_gga(std::string &gga){
    std::cout << "called set_gga()" << std::endl;
    gga_=gga;
    ntrip_client_.set_gga_buffer(gga_);
  }


  void initializeNtripClient(std::string ip,int port,std::string user,std::string passwd,std::string mountpoint,double latitude,double longitude);
  void callback_ntrip(const char *buffer, int size);


  FncStr fncStr_;

  FncChar fncChar_;

  libntrip::NtripClient ntrip_client_;

  bool set_ggaf_=false;
  //std::string rtk_fix_f_;

  int gga_status_;  // GGA[6] Status

 private:
    bool ok;
    bool call_back_f=false;
    fSub fsub_;
    fNmea fnamea_;

    std::string gga_;

    std::shared_ptr<rclcpp::Node> node_;

    //std::function<void(char *)> fnc;

    int rtcm_cnt_=0;
    //unsigned int _baudrate=9600;
    //unsigned int _baudrate=115200;
    unsigned int _baudrate=460800;
    int rate_=1;
    int meas_rate_;
    char nmea_data[512];
    std::string nmea_str;

    char end_of_line_char_=0x0a;

    // 受信データ
    #define SERIAL_PORT_READ_BUF_SIZE 256
    boost::array<unsigned char, SERIAL_PORT_READ_BUF_SIZE> read_buf_raw_;

    boost::shared_ptr<boost::thread> background_thread_; //!< thread for the I/O

    //std::string read_buf_str_;

    typedef boost::mutex Mutex;
    Mutex mutex_; //!< Lock for the input buffer


  //! Types for ACK/NACK messages, WAIT is used when waiting for an ACK
  enum AckType {
    NACK, //! Not acknowledged
    ACK, //! Acknowledge
    WAIT //! Waiting for ACK
  };

  //! Stores ACK/NACK messages
  struct Ack {
    AckType type; //!< The ACK type
    uint8_t class_id; //!< The class ID of the ACK
    uint8_t msg_id; //!< The message ID of the ACK
  };


  //! Whether or not the I/O port has been configured
  bool configured_;
  //! Whether or not to save Flash BBR on shutdown
  bool save_on_shutdown_;
  //!< Whether or not initial configuration to the hardware is done
  bool config_on_startup_flag_;


  //! The default timeout for ACK messages
  static const boost::posix_time::time_duration default_timeout_;
  //! Stores last received ACK accessed by multiple threads
  //mutable boost::atomic<Ack> ack_;

  std::string host_, port_;
  boost::shared_ptr<boost::asio::io_service> io_service_;
  boost::shared_ptr<boost::asio::serial_port> serial_;

  boost::thread read_thread;

};

} // nemespace gysfdmaxb_gps

#endif

