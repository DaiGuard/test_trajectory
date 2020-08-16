#ifndef __JOG_CONTROL_H__
#define __JOG_CONTROL_H__


#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>

#include <map>


class JogControl
{
  public:
    JogControl(ros::NodeHandle& nh);
    ~JogControl();

  private:
    // joint name dictionary map
    std::map<std::string, int> _joint_map;

    ros::Subscriber _js_sub;

    void cbJointState(const sensor_msgs::JointStateConstPtr& js);    
};

#endif