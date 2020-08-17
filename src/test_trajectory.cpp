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
  
  ros::Rate rate(10);

  double diff = 0.1;

  while(ros::ok())
  {
    tf::Transform tf_now = jog.getNowPose();

    tf::Vector3 vec = tf_now.getOrigin();

    vec.setY(vec.getY() + diff);

    if(vec.getY() > 0.5)
      diff = - 0.1;
    else if(vec.getY() < -0.5)
      diff = 0.1;

    tf_now.setOrigin(vec);

    jog.moveTrajectory(tf_now);

    rate.sleep();
    ros::spinOnce();
  }  

  return EXIT_SUCCESS;
}