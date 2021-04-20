#include <iostream>
#include <sstream>
#include <thread>
#include <string>
#include <vector>
#include <mutex>
#include <serial/serial.h>

#include <ros/ros.h>
#include <std_msgs/String.h>


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "nmea_ros_driver");
  ros::NodeHandle nh_, private_nh_("~");
  
  int32_t baudrate_;
  serial::Serial com_;
  std::string nmea_com_;
  ros::Publisher nmea_msg_pub_;

  private_nh_.param<int>("baudrate", baudrate_, 115200);
  private_nh_.param<std::string>("nmea_com", nmea_com_, "/dev/ttyUSB0");

  nmea_msg_pub_ = nh_.advertise<std_msgs::String>("nmea_msg", 10);

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  try
  {
    com_.setPort(nmea_com_);
    com_.setBaudrate(baudrate_);
    com_.setBytesize(serial::eightbits);
    com_.setParity(serial::parity_none);
    com_.setStopbits(serial::stopbits_one);
    com_.setFlowcontrol(serial::flowcontrol_none);
    serial::Timeout serial_timeout = serial::Timeout::simpleTimeout(1000);
    com_.setTimeout(serial_timeout);
    com_.open();
    //com_.setRTS(false);
    //com_.setDTR(false);
  }
  catch (serial::IOException &e)
  {
    ROS_ERROR_STREAM("Unable to open serial port:" << nmea_com_);
    return -1;
  }

  ROS_INFO_STREAM("Open serial port:" << nmea_com_ << " successful!!");

  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  while (ros::ok())
  {
    ros::spinOnce();
    
    std_msgs::String msg;
    size_t len = com_.readline(msg.data);

    if (len > 0)
    {
      nmea_msg_pub_.publish(msg);
    }
    else
    {
      ROS_WARN_STREAM("read gps port time out!");
    }
  }

  ROS_INFO("All finish");

  return 0;
}

