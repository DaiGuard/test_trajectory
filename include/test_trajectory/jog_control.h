#ifndef __JOG_CONTROL_H__
#define __JOG_CONTROL_H__


#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <tf/transform_broadcaster.h>

#include <map>


class JogControl
{
  public:
    JogControl(ros::NodeHandle& nh);
    ~JogControl();

    void moveTrajectory(const tf::Transform& goal);

    sensor_msgs::JointState getNowJointState(){ return _joint_state; }
    tf::Transform getNowPose(){ return _pose; }    

  private:
    // joint name dictionary map
    std::map<std::string, int> _joint_map;
    sensor_msgs::JointState _joint_state;    
    tf::Transform _pose;

    ros::Subscriber _js_sub;
    ros::Publisher _cmd_pub;

    void cbJointState(const sensor_msgs::JointStateConstPtr& js);    
};

#endif