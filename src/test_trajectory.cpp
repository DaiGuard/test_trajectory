#include <ros/ros.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <tm_kinematics/tm_kin.h>

#include <cmath>

#include "test_trajectory/jog_control.h"



int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_trajectory");
  ros::NodeHandle nh;

  JogControl jog(nh);

  // ros::Publisher cmd_pub
  //   = nh.advertise<trajectory_msgs::JointTrajectory>(
  //       "/arm_controller/command", 10);
  
  // ros::Rate rate(10);

  // trajectory_msgs::JointTrajectory traj;
  // trajectory_msgs::JointTrajectoryPoint point;

  // // trajectory setting
  // traj.joint_names.clear();
  // traj.joint_names.push_back("elbow_1_joint");
  // traj.joint_names.push_back("shoulder_1_joint");
  // traj.joint_names.push_back("shoulder_2_joint");
  // traj.joint_names.push_back("wrist_1_joint");
  // traj.joint_names.push_back("wrist_2_joint");
  // traj.joint_names.push_back("wrist_3_joint");

  // // 1 point setting
  // point.positions.resize(6, 0.0);
  // point.velocities.resize(6, 0.0);
  // point.accelerations.resize(6, 0.0);
  // point.effort.resize(6, 0.0);
  // point.time_from_start = ros::Duration(0.1);

  // traj.points.clear();
  // traj.points.push_back(point);

  // float diff = M_PI / 10.0;

  // while(ros::ok())
  // {    
  //   if(point.positions[0] > M_PI/2.0)
  //   {
  //     diff = -M_PI / 10.0;
  //   }
  //   else if(point.positions[0] < -M_PI/2.0)
  //   {
  //     diff = M_PI / 10.0;
  //   }

  //   point.positions[0] += diff;

  //   traj.points[0] = point;

  //   cmd_pub.publish(traj); 

  //   printf("\r(");
  //   for(int i=0; i<6; i++)   
  //   {
  //     printf("%f, ", traj.points[0].positions[i]);
  //   }
  //   printf(")");
  //   fflush(stdout);

  //   // loop spin
  //   rate.sleep();
  //   ros::spinOnce();
  // }

  ros::spin();

  return EXIT_SUCCESS;
}