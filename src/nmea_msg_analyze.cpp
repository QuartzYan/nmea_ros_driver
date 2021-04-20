#include <iostream>
#include <sstream>
#include <thread>
#include <string>
#include <vector>
#include <mutex>
#include <boost/crc.hpp>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/NavSatFix.h>

class NmeaMsgAnalyzeNode
{
public:
  NmeaMsgAnalyzeNode(ros::NodeHandle nh, ros::NodeHandle private_nh);
  ~NmeaMsgAnalyzeNode(){};

  template <class Type>
  Type stringToNum(const std::string& str)
  {
    std::istringstream iss(str);
    Type num;
    iss >> num;
    return num;
  }

private:
  ros::NodeHandle nh_, private_nh_;
  std::string frame_id_;
  std::string nmea_topic_name_;
  ros::Publisher fixed_msg_pub_;
  ros::Publisher gps_yaw_pub_;
  ros::Subscriber nmea_msg_sub_;

  void NmeaMsgCallBack(const std_msgs::String::ConstPtr &msg);
  bool CheckNmeaMsgSum(const std::string msg);
  bool CheckCRC32Sum(const std::string msg);

  uint32_t Hex2Int(const char *str, const uint8_t len)
  {
    int val = 0;
    for (uint8_t i = 0; i < len; i++)
    {
      val = val * 16;
      //char a = str[i];
      if (str[i] >= 'A' && str[i] <= 'F')
        val += str[i] - 'A' + 10;
      else if (str[i] >= 'a' && str[i] <= 'f')
        val += str[i] - 'a' + 10;
      else
        val += str[i] - '0';
    }
    return val;
  }
};

NmeaMsgAnalyzeNode::NmeaMsgAnalyzeNode(ros::NodeHandle nh, ros::NodeHandle private_nh)
    : nh_(nh), private_nh_(private_nh)
{
  //get ros param
  private_nh_.param<std::string>("frame_id", frame_id_, "gps_link");
  private_nh_.param<std::string>("nmea_topic_name", nmea_topic_name_, "nmea_msg");
  //init Publisher and Subscriber
  fixed_msg_pub_ = nh_.advertise<sensor_msgs::NavSatFix>("/gps/fixed", 5);
  gps_yaw_pub_ = nh_.advertise<std_msgs::Float64>("/gps/yaw", 5);
  nmea_msg_sub_ = nh_.subscribe(nmea_topic_name_, 1, &NmeaMsgAnalyzeNode::NmeaMsgCallBack, this);
  //sleep
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
}

bool NmeaMsgAnalyzeNode::CheckNmeaMsgSum(const std::string msg)
{
  int sum = 0;
  int sum_msg = 0;
  for (size_t i = 1; i < msg.length(); i++)
  {
    if(i == 1)
    {
      sum = msg[i];
      continue;
    }
    else if (msg[i] != '*')
    {
      sum ^= msg[i];
      continue;
    }
    else
    {
      sum_msg = Hex2Int(&msg[i+1], 2);
      break;
    }
  }

  //ROS_INFO_STREAM("sum:" << sum << "\t" << "sum_msg:" << sum_msg);

  if (sum == sum_msg)
    return true;
  else
    return false;
}

bool NmeaMsgAnalyzeNode::CheckCRC32Sum(const std::string msg)
{
  uint32_t length = 0;
  uint32_t msg_crc = 0x00;
  for (size_t i = 1; i < msg.length(); i++)
  {
    if (msg[i] == '*')
    {
      msg_crc = Hex2Int(&msg[i+1], 8);
      length = i - 1;
      break;
    }
  }
 
  boost::crc_32_type crc32;
  crc32.process_bytes(msg.substr(1).data(), length);
  
  //std::string std = "HEADINGA,COM1,0,60.0,UNKNOWN,1408,113980.800,00000000,0000,1114;COLD_START,NONE,0.000000000,0.000000000,0.000000000,0.000000000,180.000000000,90.000000000,\"AAAA\",0,0,0,0,0,0,0,0";
  //crc32.process_bytes(std.data(), std.length());
  //msg_crc = 0xfb829747;

  //ROS_INFO_STREAM("crc:" << crc32.checksum() << "\t" << "crc_msg:" << msg_crc);
  
  if (msg_crc == crc32.checksum())
    return true;
  else
    return true; 
}

void NmeaMsgAnalyzeNode::NmeaMsgCallBack(const std_msgs::String::ConstPtr &msg)
{
  //$GPGGA,040354.80,0000.0000000,N,00000.0000000,E,0,00,0.0,-6378154.1620,M,17.162,M,,*72
  std::vector<std::string> list;
  if (msg->data[0] == '$')  //NMEA-0183 message type 
  {
    if(CheckNmeaMsgSum(msg->data))
    {
      //ROS_INFO_STREAM("msg:" << msg->data);
      std::vector<std::string> msg_list;
      std::string str;
      for (size_t i = 1; i < msg->data.length(); i++)
      {
        if (msg->data[i] == ',')
        {
          msg_list.push_back(str);
          str.clear();
          continue;
        }
        else if (msg->data[i] == '*')
        {
          msg_list.push_back(str);
          str.clear();
          break;
        }
        else
        {
          str += msg->data[i];
          continue;
        }      
      }

      //ROS_INFO_STREAM("msg:" << msg_list.size());

      if (msg_list[0] == "GPGGA")
      {
        sensor_msgs::NavSatFix nav_msg;
        nav_msg.header.stamp = ros::Time::now();
        nav_msg.header.frame_id = frame_id_;

        nav_msg.status.status = stringToNum<int>(msg_list[6]);
        nav_msg.status.service = nav_msg.status.SERVICE_GPS;
        
        if (msg_list[3] == "N")
        {
          nav_msg.latitude = stringToNum<int>(msg_list[2].substr(0, 2)) + stringToNum<double>(msg_list[2].substr(2));
        }
        else
        {
          nav_msg.latitude = -1.0 * (stringToNum<int>(msg_list[2].substr(0, 2)) + stringToNum<double>(msg_list[2].substr(2)));
        }

        if (msg_list[5] == "E")
        {
          nav_msg.longitude = stringToNum<int>(msg_list[4].substr(0, 3)) + stringToNum<double>(msg_list[4].substr(3));
        }
        else
        {
          nav_msg.longitude = -1.0 * (stringToNum<int>(msg_list[4].substr(0, 3)) + stringToNum<double>(msg_list[4].substr(3)));
        }
        
        nav_msg.altitude = stringToNum<double>(msg_list[9]);

        double hdop = stringToNum<double>(msg_list[8]);

        double default_epe_quality = 0.0;

        switch (nav_msg.status.status)
        {
        case -1:
          default_epe_quality = 1000000.0;
          nav_msg.position_covariance_type = nav_msg.COVARIANCE_TYPE_UNKNOWN;
          break;
        case 0:
          default_epe_quality = 1000000.0;
          nav_msg.position_covariance_type = nav_msg.COVARIANCE_TYPE_UNKNOWN;
          break;
        case 1:
          default_epe_quality = 4.0;
          nav_msg.position_covariance_type = nav_msg.COVARIANCE_TYPE_APPROXIMATED;
          break;
        case 2:
          default_epe_quality = 0.1;
          nav_msg.position_covariance_type = nav_msg.COVARIANCE_TYPE_APPROXIMATED;
          break;
        case 4:
          default_epe_quality = 0.02;
          nav_msg.position_covariance_type = nav_msg.COVARIANCE_TYPE_APPROXIMATED;
          break;
        case 5:
          default_epe_quality = 4.0;
          nav_msg.position_covariance_type = nav_msg.COVARIANCE_TYPE_APPROXIMATED;
          break;
        default:
          default_epe_quality = 1000000.0;
          nav_msg.position_covariance_type = nav_msg.COVARIANCE_TYPE_UNKNOWN;
          break;
        }

        nav_msg.position_covariance[0] = pow((hdop * default_epe_quality), 2);
        nav_msg.position_covariance[4] = pow((hdop * default_epe_quality), 2);
        nav_msg.position_covariance[8] = pow((4.0 * hdop * default_epe_quality), 2);

        fixed_msg_pub_.publish(nav_msg);       
      }
      
    }
    else
    { 
      ROS_WARN_STREAM("nmea msg check sum error!! msg:" << msg->data);
    }
  }
  else if (msg->data[0] == '#')  //custom message type
  {
    if(CheckCRC32Sum(msg->data))
    {
      std::vector<std::string> msg_list;
      std::string str;
      for (size_t i = 1; i < msg->data.length(); i++)
      {
        if (msg->data[i] == ',' || msg->data[i] == ';')
        {
          msg_list.push_back(str);
          str.clear();
          continue;
        }
        else if (msg->data[i] == '*')
        {
          msg_list.push_back(str);
          str.clear();
          break;
        }
        else
        {
          str += msg->data[i];
          continue;
        }      
      }

      //ROS_INFO_STREAM("msg:" << msg_list.size());
      
      //Baseline length 13
      //ROS_INFO_STREAM("Baseline length:" << stringToNum<double>(msg_list[13]));
      //yaw
      //ROS_INFO_STREAM("yaw:" << stringToNum<double>(msg_list[14]));
      std_msgs::Float64 yaw;
      yaw.data = stringToNum<double>(msg_list[14]);
      gps_yaw_pub_.publish(yaw);
      //pitch
      //ROS_INFO_STREAM("pitch:" << stringToNum<double>(msg_list[15]));
  
    }
    else
    { 
      ROS_WARN_STREAM("nmea msg check sum error!! msg:" << msg->data);
    }
  }
  else  //unknown message type
  {
    ROS_WARN_STREAM("unknown message type!! msg:" << msg->data);
  }
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "nmea_msg_analyze");
  ros::NodeHandle nh, private_nh("~");

  NmeaMsgAnalyzeNode nd(nh, private_nh);

  ros::spin();

  ROS_INFO("All finish");

  return 0;
}
