#include <thread>
#include <string>
#include <vector>
#include <sstream>
#include <iostream>
#include <boost/crc.hpp>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <boost/assign.hpp>
#include <ros/ros.h>

#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "nmea_ros_driver/GPSPoint.h"
#include "nmea_ros_driver/GPSRPY.h"
#include <sensor_msgs/NavSatFix.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::NavSatFix, nmea_ros_driver::GPSPoint, nmea_ros_driver::GPSRPY> nmeaSyncPolicy;

class NmeaToOdomNode
{
public:
  NmeaToOdomNode(ros::NodeHandle nh, ros::NodeHandle private_nh);
  ~NmeaToOdomNode(){};

private:
  ros::NodeHandle nh_, private_nh_;
  bool pub_odom_tf_;
  std::string cmd_topic_name_;
  std::string base_frame_id_;
  std::string nmea_frame_id_;
  std::string odom_frame_id_;
  ros::Publisher odom_pub_;

  tf2_ros::TransformBroadcaster odom_broadcaster_;

  boost::assign_detail::generic_list<double> pose_covariance_;
  boost::assign_detail::generic_list<double> twist_covariance_;

  message_filters::Subscriber<nmea_ros_driver::GPSPoint> *nmea_point_sub_;
  message_filters::Subscriber<nmea_ros_driver::GPSRPY> *nmea_rpy_sub_;
  message_filters::Subscriber<sensor_msgs::NavSatFix> *nmea_sub_;
  message_filters::Synchronizer<nmeaSyncPolicy> *emna_sync_;

  void NmeaCallBack(const sensor_msgs::NavSatFix::ConstPtr &gps, 
                    const nmea_ros_driver::GPSPoint::ConstPtr &point, 
                    const nmea_ros_driver::GPSRPY::ConstPtr &rpy);
};

NmeaToOdomNode::NmeaToOdomNode(ros::NodeHandle nh, ros::NodeHandle private_nh)
    : nh_(nh), private_nh_(private_nh)
{

  private_nh_.param<bool>("pub_odom_tf", pub_odom_tf_, true);
  private_nh_.param<std::string>("base_frame_id", base_frame_id_, "base_link");
  private_nh_.param<std::string>("odom_frame_id", odom_frame_id_, "odom");
  private_nh_.param<std::string>("nmea_frame_id", nmea_frame_id_, "main_gps_link");

  std::vector<double> pose_cov_diag(6, 1e-6);
  std::vector<double> twist_cov_diag(6, 1e-6);

  pose_covariance_ = boost::assign::list_of(pose_cov_diag[0])(0)(0)(0)(0)(0)
                                            (0)(pose_cov_diag[1])(0)(0)(0)(0)
                                            (0)(0)(pose_cov_diag[2])(0)(0)(0)
                                            (0)(0)(0)(pose_cov_diag[3])(0)(0)
                                            (0)(0)(0)(0)(pose_cov_diag[4])(0)
                                            (0)(0)(0)(0)(0)(pose_cov_diag[5]);
  twist_covariance_ = boost::assign::list_of(twist_cov_diag[0])(0)(0)(0)(0)(0)
                                            (0)(twist_cov_diag[1])(0)(0)(0)(0)
                                            (0)(0)(twist_cov_diag[2])(0)(0)(0)
                                            (0)(0)(0)(twist_cov_diag[3])(0)(0)
                                            (0)(0)(0)(0)(twist_cov_diag[4])(0)
                                            (0)(0)(0)(0)(0)(twist_cov_diag[5]);

  nmea_point_sub_ = new message_filters::Subscriber<nmea_ros_driver::GPSPoint>(nh_, "/gps/point", 5);
  nmea_rpy_sub_ = new message_filters::Subscriber<nmea_ros_driver::GPSRPY>(nh_, "/gps/rpy", 5);
  nmea_sub_ = new message_filters::Subscriber<sensor_msgs::NavSatFix>(nh_, "/gps/fixed", 5);
  emna_sync_ = new message_filters::Synchronizer<nmeaSyncPolicy>(nmeaSyncPolicy(5), *nmea_sub_, *nmea_point_sub_, *nmea_rpy_sub_);
  emna_sync_->registerCallback(boost::bind(&NmeaToOdomNode::NmeaCallBack, this, _1, _2, _3));

  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/gps/odom", 5);

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

void NmeaToOdomNode::NmeaCallBack(const sensor_msgs::NavSatFix::ConstPtr &gps, 
                                  const nmea_ros_driver::GPSPoint::ConstPtr &point, 
                                  const nmea_ros_driver::GPSRPY::ConstPtr &rpy)
{
  static bool first_msg_flage = true;
  static double init_x(0.0), init_y(0.0), init_z(0.0), init_th(0.0);
  static tf::StampedTransform nmea_rt;

  if (first_msg_flage)
  {
    if (gps->status.status != 4)
    {
      ROS_WARN_STREAM("GPS positioning accuracy LOW! GPS STATUS:" << int(gps->status.status));
      return;
    }

    init_x = point->x;
    init_y = point->y;
    init_z = point->z;
    init_th = rpy->y;
    tf::TransformListener tf_;
    if(tf_.waitForTransform(base_frame_id_, nmea_frame_id_, ros::Time(0), ros::Duration(3)))
    {
      tf_.lookupTransform(base_frame_id_, nmea_frame_id_, ros::Time(0), nmea_rt);
      ROS_INFO_STREAM("Get " << base_frame_id_ << " form " << nmea_frame_id_ << " successful!");
      first_msg_flage = false;
    }
    else
    {
      ROS_ERROR_STREAM("Get " << base_frame_id_ << " form " << nmea_frame_id_ << " time out!!");
    }
  }

  static ros::Time now_time;
  static ros::Time last_time;

  now_time = ros::Time::now();

  if (last_time.isZero())
  {
    last_time = now_time;
    return;
  }

  double dt = (now_time - last_time).toSec();

  if (dt == 0)
    return;
  last_time = now_time;

  if (gps->status.status != 4)
  {
   ROS_WARN_STREAM("GPS positioning accuracy LOW! GPS STATUS:" << int(gps->status.status));
  }

  double gps_x = (point->x - init_x);
  double gps_y = -1.0 * (point->y - init_y);
  double gps_z = (point->z - init_z);
  double gps_th = 0.0;
  if (rpy->y > 0 && rpy->y <= 180.0)
  {
    gps_th = (rpy->y) * -3.1415926 / 180.0;
  }
  else
  {
    gps_th = (360.0 - rpy->y) * 3.1415926 / 180.0;
  }
  
  double x = gps_x ;//- nmea_rt.getOrigin().getX();
  double y = gps_y ;//- nmea_rt.getOrigin().getY();
  double z = gps_z ;//- nmea_rt.getOrigin().getZ();
  double th = gps_th;
  
  //first, we'll publish the transform over tf
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = now_time;
  odom_trans.header.frame_id = odom_frame_id_;
  odom_trans.child_frame_id = base_frame_id_;

  odom_trans.transform.translation.x = x;
  odom_trans.transform.translation.y = y;
  odom_trans.transform.translation.z = 0.0;
  tf2::Quaternion odom_quat;
  odom_quat.setRPY(0, 0, th);
  odom_trans.transform.rotation.x = odom_quat.x();
  odom_trans.transform.rotation.y = odom_quat.y();
  odom_trans.transform.rotation.z = odom_quat.z();
  odom_trans.transform.rotation.w = odom_quat.w();

  //send the transform
  if (pub_odom_tf_)
  {
    odom_broadcaster_.sendTransform(odom_trans);
  }

  nav_msgs::Odometry odom;
  odom.header.stamp = now_time;
  odom.header.frame_id = odom_frame_id_;
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation.x = odom_quat.x();
  odom.pose.pose.orientation.y = odom_quat.y();
  odom.pose.pose.orientation.z = odom_quat.z();
  odom.pose.pose.orientation.w = odom_quat.w();
  odom.pose.covariance = pose_covariance_;
  //set the velocity
  odom.child_frame_id = base_frame_id_;
  odom.twist.twist.linear.x = 0.0;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = 0.0;
  odom.twist.covariance = twist_covariance_;

  odom_pub_.publish(odom);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "nmea_to_odom");
  ros::NodeHandle nh, private_nh("~");

  NmeaToOdomNode nd(nh, private_nh);

  ros::spin();

  ROS_INFO("All finish");

  return 0;
}