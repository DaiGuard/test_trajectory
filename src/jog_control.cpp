#include "test_trajectory/jog_control.h"

#include <tm_kinematics/tm_kin.h>

#include <vector>
#include <cmath>

JogControl::JogControl(ros::NodeHandle &nh)
{
  // joint map initialize
  _joint_map["elbow_1_joint"] = 2;
  _joint_map["shoulder_1_joint"] = 0;
  _joint_map["shoulder_2_joint"] = 1;
  _joint_map["wrist_1_joint"] = 3;
  _joint_map["wrist_2_joint"] = 4;
  _joint_map["wrist_3_joint"] = 5;

  // set subscriber for /joint_states
  _js_sub = nh.subscribe("/joint_states", 10, &JogControl::cbJointState, this);
}


JogControl::~JogControl()
{

}


void JogControl::cbJointState(const sensor_msgs::JointStateConstPtr& js)
{
  double q[6];
  double T[16];
  int axis_id;
  double tx, ty, tz, qx, qy, qz, qw;

  size_t joint_num = js->name.size();
  
  for(int i=0; i<joint_num; i++)
  {
    axis_id = _joint_map.at(js->name[i]);
    q[axis_id] = js->position[i];
  }  
  
  tm_kinematics::forward(q, T);

  std::vector<double> elem;

  elem.resize(4);
  elem[0] =  T[0] - T[5] - T[10] + 1.0;
  elem[1] = -T[0] + T[5] - T[10] + 1.0;
  elem[2] = -T[0] - T[5] + T[10] + 1.0;
  elem[3] =  T[0] + T[5] + T[10] + 1.0;
  
  double max_val = 
    *std::max_element(elem.begin(), elem.end());

  if(max_val < 0.0)
    return;
}


