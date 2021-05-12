#include <iostream>
#include <sstream>
#include <thread>
#include <string>
#include <vector>
#include <boost/crc.hpp>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Point.h>
#include "nmea_ros_driver/GPSRPY.h"
#include "nmea_ros_driver/GPSPoint.h"
#include "nmea_ros_driver/XXGGA.h"
#include "nmea_ros_driver/XXGSA.h"
#include "nmea_ros_driver/XXGSV.h"
#include "nmea_ros_driver/XXRMC.h"
#include "nmea_ros_driver/XXVTG.h"
#include "nmea_ros_driver/XXGLL.h"
#include "nmea_ros_driver/NTR.h"
#include "nmea_ros_driver/PJK.h"
#include "nmea_ros_driver/TRA.h"
#include "nmea_ros_driver/HEADINGA.h"
#include "nmea_ros_driver/BESTPOSA.h"

class NmeaMsgAnalyzeNode
{
public:
  NmeaMsgAnalyzeNode(ros::NodeHandle nh, ros::NodeHandle private_nh);
  ~NmeaMsgAnalyzeNode(){};

private:
  ros::NodeHandle nh_, private_nh_;
  bool enable_custom_;
  bool enable_gga_, enable_gsa_, enable_gsv_, enable_rmc_, enable_vtg_, enable_gll_;
  ros::Publisher gga_pub_, gsa_pub_, gsv_pub_, rmc_pub_, vtg_pub_, gll_pub_;
  ros::Publisher ntr_pub_, pjk_pub_, tra_pub_, headinga_pub_, bestposa_pub_;
  std::string frame_id_;
  std::string nmea_topic_name_;
  ros::Publisher fixed_msg_pub_;
  ros::Publisher gps_point_pub_;
  ros::Publisher gps_rpy_pub_;
  ros::Subscriber nmea_msg_sub_;

  const std::map<std::string, uint8_t> nmea_msg_type_ = {
      {"GGA", 1},
      {"GSA", 2},
      {"GSV", 3},
      {"RMC", 4},
      {"VTG", 5},
      {"GLL", 6},
  };
  //nmea message
  bool procGGA(const std::vector<std::string> msg_list);
  bool procGSA(const std::vector<std::string> msg_list);
  bool procGSV(const std::vector<std::string> msg_list);
  bool procRMC(const std::vector<std::string> msg_list);
  bool procVTG(const std::vector<std::string> msg_list);
  bool procGLL(const std::vector<std::string> msg_list);
  //custom message
  bool procNTR(const std::vector<std::string> msg_list);
  bool procPJK(const std::vector<std::string> msg_list);
  bool procTRA(const std::vector<std::string> msg_list);
  bool procHEADINGA(const std::vector<std::string> msg_list);
  bool procBESTPOSA(const std::vector<std::string> msg_list);

  void NmeaMsgCallBack(const std_msgs::String::ConstPtr &msg);

  std::vector<std::string> SplitNmeaMsg(const std::string msg);
  std::vector<std::string> SplitCustomMsg(const std::string msg);

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
  template <class Type>
  Type stringToNum(const std::string &str)
  {
    std::istringstream iss(str);
    Type num;
    iss >> num;
    return num;
  }
};

NmeaMsgAnalyzeNode::NmeaMsgAnalyzeNode(ros::NodeHandle nh, ros::NodeHandle private_nh)
    : nh_(nh), private_nh_(private_nh)
{
  //get ros param
  private_nh_.param<std::string>("frame_id", frame_id_, "gps_link");
  private_nh_.param<std::string>("nmea_topic_name", nmea_topic_name_, "nmea_msg");
  private_nh_.param<bool>("enable_custom", enable_custom_, false);
  private_nh_.param<bool>("enable_gga", enable_gga_, false);
  private_nh_.param<bool>("enable_gsa", enable_gsa_, false);
  private_nh_.param<bool>("enable_gsv", enable_gsv_, false);
  private_nh_.param<bool>("enable_rmc", enable_rmc_, false);
  private_nh_.param<bool>("enable_vtg", enable_vtg_, false);
  private_nh_.param<bool>("enable_gll", enable_gll_, false);

  //init Publisher and Subscriber
  if (enable_gga_)
  {
   gga_pub_ = nh_.advertise<nmea_ros_driver::XXGGA>("/gps/nmea/XXGGA", 5);
  }
  if (enable_gsa_)
  {
   gsa_pub_ = nh_.advertise<nmea_ros_driver::XXGSA>("/gps/nmea/XXGSA", 5);
  }
  if (enable_gsv_)
  {
   gsv_pub_ = nh_.advertise<nmea_ros_driver::XXGSV>("/gps/nmea/XXGSV", 5);
  }
  if (enable_rmc_)
  {
   rmc_pub_ = nh_.advertise<nmea_ros_driver::XXRMC>("/gps/nmea/XXRMC", 5);
  }
  if (enable_vtg_)
  {
   vtg_pub_ = nh_.advertise<nmea_ros_driver::XXVTG>("/gps/nmea/XXVTG", 5);
  }
  if (enable_gll_)
  {
   gll_pub_ = nh_.advertise<nmea_ros_driver::XXGLL>("/gps/nmea/XXGLL", 5);
  }
  if (enable_custom_)
  {
    ntr_pub_ = nh_.advertise<nmea_ros_driver::NTR>("/gps/custom/NTR", 5); 
    pjk_pub_ = nh_.advertise<nmea_ros_driver::PJK>("/gps/custom/PJK", 5); 
    tra_pub_ = nh_.advertise<nmea_ros_driver::TRA>("/gps/custom/TRA", 5); 
    headinga_pub_ = nh_.advertise<nmea_ros_driver::HEADINGA>("/gps/custom/HEADINGA", 5); 
    bestposa_pub_ = nh_.advertise<nmea_ros_driver::BESTPOSA>("/gps/custom/BESTPOSA", 5); 
  }
  
  fixed_msg_pub_ = nh_.advertise<sensor_msgs::NavSatFix>("/gps/fixed", 5);
  gps_rpy_pub_ = nh_.advertise<nmea_ros_driver::GPSRPY>("/gps/rpy", 5);
  gps_point_pub_ = nh_.advertise<nmea_ros_driver::GPSPoint>("/gps/point", 5);
  nmea_msg_sub_ = nh_.subscribe(nmea_topic_name_, 5, &NmeaMsgAnalyzeNode::NmeaMsgCallBack, this);
  //sleep
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
}

bool NmeaMsgAnalyzeNode::CheckNmeaMsgSum(const std::string msg)
{
  int sum = 0;
  int sum_msg = 0;
  for (size_t i = 1; i < msg.length(); i++)
  {
    if (i == 1)
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
      sum_msg = Hex2Int(&msg[i + 1], 2);
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
      msg_crc = Hex2Int(&msg[i + 1], 8);
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
std::vector<std::string> NmeaMsgAnalyzeNode::SplitNmeaMsg(const std::string msg)
{
  std::string str;
  std::vector<std::string> msg_list;
  for (size_t i = 1; i < msg.length(); i++)
  {
    if (msg[i] == ',')
    {
      msg_list.push_back(str);
      str.clear();
      continue;
    }
    else if (msg[i] == '*')
    {
      msg_list.push_back(str);
      str.clear();
      break;
    }
    else
    {
      str += msg[i];
      continue;
    }
  }

  return msg_list;
}
std::vector<std::string> NmeaMsgAnalyzeNode::SplitCustomMsg(const std::string msg)
{
  std::string str;
  std::vector<std::string> msg_list;
  for (size_t i = 1; i < msg.length(); i++)
  {
    if (msg[i] == ',' || msg[i] == ';')
    {
      msg_list.push_back(str);
      str.clear();
      continue;
    }
    else if (msg[i] == '*')
    {
      msg_list.push_back(str);
      str.clear();
      break;
    }
    else
    {
      str += msg[i];
      continue;
    }
  }
  return msg_list;
}
void NmeaMsgAnalyzeNode::NmeaMsgCallBack(const std_msgs::String::ConstPtr &msg)
{
  if (msg->data[0] == '$') //NMEA-0183 message type
  {
    if (CheckNmeaMsgSum(msg->data))
    {
      std::vector<std::string> msg_list = SplitNmeaMsg(msg->data);
      //ROS_INFO_STREAM(msg_list[0]);
      auto it = nmea_msg_type_.find(msg_list[0].substr(2));
      if (it != nmea_msg_type_.end()) //nmea message type
      {
        switch (it->second)
        {
        case 1: //GGA
          procGGA(msg_list);
          break;
        case 2: //GSA
          procGSA(msg_list);
          break;
        case 3: //GSV
          procGSV(msg_list);
          break;
        case 4: //RMC
          procRMC(msg_list);
          break;
        case 5: //VTG
          procVTG(msg_list);
          break;
        case 6: //GLL
          procGLL(msg_list);
          break;
        default:
          break;
        }
      }
      else  //custom message type
      {
        if (msg_list[0].substr(0, 2) == "NTR")
        {
          procNTR(msg_list);
        }
        else if (msg_list[0].substr(0, 2) == "TRA")
        {
          procTRA(msg_list);
        }
        else if (msg_list[0] == "PTNL")
        {
          procPJK(msg_list);
        }
        else
        {
          ROS_WARN_STREAM("unknown message type!! msg:" << msg->data);
        }
      }
    }
    else
    {
      ROS_WARN_STREAM("nmea msg check sum error!! msg:" << msg->data);
    }
  }
  else if (msg->data[0] == '#') //custom message type
  {
    if (CheckCRC32Sum(msg->data))
    {
      std::vector<std::string> msg_list = SplitCustomMsg(msg->data);
      if (msg_list[0] == "HEADINGA")
      {
        procHEADINGA(msg_list);
      }
      else if (msg_list[0] == "BESTPOSA")
      {
        procBESTPOSA(msg_list);
      }
      else
      {
        ROS_WARN_STREAM("unknown message type!! msg:" << msg->data);
      }
    }
    else
    {
      ROS_WARN_STREAM("custom msg check sum error!! msg:" << msg->data);
    }
  }
  else //unknown message type
  {
    ROS_WARN_STREAM("unknown message type!! msg:" << msg->data);
  }
}
bool NmeaMsgAnalyzeNode::procGGA(const std::vector<std::string> msg_list)
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

  nmea_ros_driver::XXGGA gga;
  //TODO

  if (enable_gga_)
  {
    gga_pub_.publish(gga);
  }
}
bool NmeaMsgAnalyzeNode::procGSA(const std::vector<std::string> msg_list)
{
  nmea_ros_driver::XXGSA gsa;
  //TODO

  if (enable_gsa_)
  {
    gsa_pub_.publish(gsa);
  }
}
bool NmeaMsgAnalyzeNode::procGSV(const std::vector<std::string> msg_list)
{
  nmea_ros_driver::XXGSV gsv;
  //TODO

  if (enable_gsv_)
  {
    gsv_pub_.publish(gsv);
  }
}
bool NmeaMsgAnalyzeNode::procRMC(const std::vector<std::string> msg_list)
{
  nmea_ros_driver::XXRMC rmc;
  //TODO

  if (enable_rmc_)
  {
    rmc_pub_.publish(rmc);
  }
}
bool NmeaMsgAnalyzeNode::procVTG(const std::vector<std::string> msg_list)
{
  nmea_ros_driver::XXVTG vtg;
  //TODO

  if (enable_vtg_)
  {
    vtg_pub_.publish(vtg);
  }
}
bool NmeaMsgAnalyzeNode::procGLL(const std::vector<std::string> msg_list)
{
  nmea_ros_driver::XXGLL gll;
  //TODO

  if (enable_gll_)
  {
    gll_pub_.publish(gll);
  }
}
bool NmeaMsgAnalyzeNode::procNTR(const std::vector<std::string> msg_list)
{
  nmea_ros_driver::NTR ntr;
  //TODO

  if (enable_custom_)
  {
    ntr_pub_.publish(ntr);
  }
}
bool NmeaMsgAnalyzeNode::procPJK(const std::vector<std::string> msg_list)
{
  double x = std::stod(msg_list[4]);
  double y = std::stod(msg_list[6]);
  double z = std::stod(msg_list[11].substr(3));

  nmea_ros_driver::GPSPoint msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = frame_id_;
  msg.x = x;
  msg.y = y;
  msg.z = z;

  gps_point_pub_.publish(msg);
  //std::cout << "x:" << msg_list[4].substr(1) << "\t" << "y:" << msg_list[6].substr(1) << std::endl;
  // ROS_INFO_STREAM("X:" << x << "\t" << "Y:" << y);
  //ROS_INFO("%lf %lf", x, y);

  nmea_ros_driver::PJK pjk;
  //TODO

  if (enable_custom_)
  {
    pjk_pub_.publish(pjk);
  }
}
bool NmeaMsgAnalyzeNode::procTRA(const std::vector<std::string> msg_list)
{
  nmea_ros_driver::TRA tra;
  //TODO

  if (enable_custom_)
  {
    tra_pub_.publish(tra);
  }
}
bool NmeaMsgAnalyzeNode::procHEADINGA(const std::vector<std::string> msg_list)
{
  nmea_ros_driver::GPSRPY msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = frame_id_;
  msg.r = 0.0;
  msg.p = stringToNum<double>(msg_list[14]);
  msg.y = stringToNum<double>(msg_list[13]);
  gps_rpy_pub_.publish(msg);

  nmea_ros_driver::HEADINGA headinga;
  //TODO

  if (enable_custom_)
  {
    headinga_pub_.publish(headinga);
  }
}
bool NmeaMsgAnalyzeNode::procBESTPOSA(const std::vector<std::string> msg_list)
{
  nmea_ros_driver::BESTPOSA bestposa;
  //TODO

  if (enable_custom_)
  {
    bestposa_pub_.publish(bestposa);
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
