#include <ros/ros.h>
#include "mpc/Polynome.h"
#include "planner_algorithm/poly_traj_utils.hpp"
#include "planner_algorithm/back_end_optimizer.h"

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "visualization_msgs/Marker.h"
using namespace Eigen;

ros::Publisher despoint_vis_pub;
ros::Publisher despoint_pub;
ros::Publisher odom_out_pub;

double dt = 0.1;
double t_cur;

bool has_traj = false;
bool has_odom = false;
bool publish_odom_only = true; // if true, ignore control, directly write odom along trajectory
std::string odom_frame_id = "world";
bool seed_odom_on_start = true;
double seed_x = 0.0, seed_y = 0.0, seed_z = 1.0;

Trajectory trajectory;
minco::MinJerkOpt jerkOpt_;
double traj_duration;
ros::Time start_time;
int traj_id;


//odometry on real time
Eigen::Vector3d  odometry_pos;
Eigen::Vector3d  odometry_vel;
double           odometry_yaw;

void tempRenderAPoint(Vector3d pt, Vector3d color)
{
    visualization_msgs::Marker sphere;
    sphere.header.frame_id      = "world";
    sphere.header.stamp         = ros::Time::now();
    sphere.type                 = visualization_msgs::Marker::SPHERE;
    sphere.action               = visualization_msgs::Marker::ADD;
    sphere.id                   = 1;
    sphere.pose.orientation.w   = 1.0;
    sphere.color.r              = color(0);
    sphere.color.g              = color(1);
    sphere.color.b              = color(2);
    sphere.color.a              = 0.8;
    sphere.scale.x              = 0.2;
    sphere.scale.y              = 0.2;
    sphere.scale.z              = 0.2;
    sphere.pose.position.x      = pt(0);
    sphere.pose.position.y      = pt(1);
    sphere.pose.position.z      = pt(2);
    despoint_vis_pub.publish(sphere);
}

void rcvOdomCallBack(nav_msgs::OdometryPtr msg)
{
  if(has_odom == false){ cout <<"[TRAJ_SERVER] has odometry "<<endl; }
  has_odom = true;
  odometry_pos[0] = msg->pose.pose.position.x;
  odometry_pos[1] = msg->pose.pose.position.y;
  odometry_pos[2] = msg->pose.pose.position.z;

  
  odometry_vel[0] = msg->twist.twist.linear.x;
  odometry_vel[1] = msg->twist.twist.linear.y;
  odometry_vel[2] = msg->twist.twist.linear.z;

  Eigen::Quaterniond q( msg->pose.pose.orientation.w,
			                  msg->pose.pose.orientation.x,
		                  	msg->pose.pose.orientation.y,
		                  	msg->pose.pose.orientation.z );
  Eigen::Matrix3d R(q);
  odometry_yaw = atan2(R.col(0)[1],R.col(0)[0]);
  
}

void polynomialTrajCallback(mpc::PolynomeConstPtr msg)
{
  // parse pos traj
  MatrixXd posP(3, msg -> pos_pts.size() - 2);
  VectorXd T(msg -> t_pts.size());
  MatrixXd initS, tailS;

  for (int i = 1; i < msg -> pos_pts.size() - 1 ;i++)
  {
    posP(0, i-1) = msg->pos_pts[i].x;
    posP(1, i-1) = msg->pos_pts[i].y;
    posP(2, i-1) = msg->pos_pts[i].z;
  }
  for (int i=0; i<msg->t_pts.size();i++)
  {
    T(i) = msg->t_pts[i];
  }
  
  initS.setZero(3, 3);
  tailS.setZero(3, 3);
  initS.col(0) = Vector3d(msg->pos_pts[0].x, msg->pos_pts[0].y, msg->pos_pts[0].z);
  initS.col(1) = Vector3d(msg->init_v.x, msg->init_v.y, msg->init_v.z);
  initS.col(2) = Vector3d(msg->init_a.x, msg->init_a.y, msg->init_a.z);
  tailS.col(0) = Vector3d(msg->pos_pts.back().x, msg->pos_pts.back().y, msg->pos_pts.back().z);
  tailS.col(1) = Vector3d::Zero();
  tailS.col(2) = Vector3d::Zero();
  jerkOpt_.reset(initS, msg->pos_pts.size()-1);
  jerkOpt_.generate(posP, tailS, T);

  trajectory    = jerkOpt_.getTraj();
  traj_duration = trajectory.getTotalDuration();

  start_time  = msg -> start_time;

  if(has_traj == false){ cout <<"[TRAJ_SERVER] has trajectory "<<endl; }
  has_traj = true;
}

double err_yaw( double des_yaw, double odom_yaw)
{
  if(des_yaw - odom_yaw >= pi)
    return (des_yaw - odom_yaw) - 2 * pi;
  else if(des_yaw - odom_yaw <= -pi)
    return 2 * pi + (des_yaw - odom_yaw);
  else
    return (des_yaw - odom_yaw); 
}

void cmdCallback(const ros::TimerEvent &e)
{
    if (!has_odom && seed_odom_on_start)
    {
      nav_msgs::Odometry odom;
      odom.header.stamp = ros::Time::now();
      odom.header.frame_id = odom_frame_id;
      odom.pose.pose.position.x = seed_x;
      odom.pose.pose.position.y = seed_y;
      odom.pose.pose.position.z = seed_z;
      odom.pose.pose.orientation.w = 1.0;
      odom_out_pub.publish(odom);
    }

    if ((!has_traj) || (!has_odom))
      return;
    
    ros::Time time_now  = ros::Time::now();
    t_cur               = (time_now - start_time).toSec();

    if (t_cur < traj_duration && t_cur >= 0.0)
    {
      Vector3d des_pos   = trajectory.getPos(t_cur);
      Vector3d des_vel   = trajectory.getVel(t_cur);
      Vector3d des_acc   = trajectory.getAcc(t_cur);
      Vector2d des_velxy   = Vector2d(des_vel(0), des_vel(1));
      tempRenderAPoint(des_pos, Vector3d(0.1,0.2,0.9) );
    
      double des_yaw = atan2(des_vel(1), des_vel(0));
      double des_yawdot = 0.0;
      if (des_velxy.squaredNorm() > 1e-6)
        des_yawdot  =  (des_acc(1) * des_vel(0) - des_acc(0) * des_vel(1)) / (des_velxy.squaredNorm());

      if (publish_odom_only)
      {
        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = odom_frame_id;
        odom.pose.pose.position.x = des_pos(0);
        odom.pose.pose.position.y = des_pos(1);
        odom.pose.pose.position.z = des_pos(2);
        Eigen::AngleAxisd yaw_aa(des_yaw, Eigen::Vector3d::UnitZ());
        Eigen::Quaterniond q(yaw_aa);
        odom.pose.pose.orientation.w = q.w();
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.twist.twist.linear.x = des_vel(0);
        odom.twist.twist.linear.y = des_vel(1);
        odom.twist.twist.linear.z = des_vel(2);
        odom.twist.twist.angular.z = des_yawdot;
        odom_out_pub.publish(odom);
      }
    }

    else if(t_cur >= traj_duration)
    {
      if (publish_odom_only)
      {
        Vector3d des_pos   = trajectory.getPos(traj_duration);
        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = odom_frame_id;
        odom.pose.pose.position.x = des_pos(0);
        odom.pose.pose.position.y = des_pos(1);
        odom.pose.pose.position.z = des_pos(2);
        Eigen::Quaterniond q(Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()));
        odom.pose.pose.orientation.w = q.w();
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom_out_pub.publish(odom);
      }
    }

}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_server");
  ros::NodeHandle nh("~");
  nh.param("seed_odom_on_start", seed_odom_on_start, true);
  nh.param("seed_x", seed_x, 0.0);
  nh.param("seed_y", seed_y, 0.0);
  nh.param("seed_z", seed_z, 1.0);

  ros::Subscriber traj_sub      = nh.subscribe("trajectory_topic", 10, polynomialTrajCallback);
  ros::Subscriber odom_sub      = nh.subscribe("odom", 1, rcvOdomCallBack );

  despoint_pub      = nh.advertise<geometry_msgs::PoseStamped>("despoint", 20); 
  despoint_vis_pub  = nh.advertise<visualization_msgs::Marker>("point/vis", 20); 
  odom_out_pub   = nh.advertise<nav_msgs::Odometry>("odom_out", 20); 

  ros::Timer cmd_timer = nh.createTimer(ros::Duration(0.01), cmdCallback);

  ros::Duration(1.0).sleep();
  ROS_WARN("[Traj server]: ready.");
  ros::spin();

  return 0;
}