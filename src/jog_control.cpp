#include "test_trajectory/jog_control.h"

#include <tm_kinematics/tm_kin.h>

#include <vector>
#include <cmath>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>


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

  // set publisher for /arm_controller/command
  _cmd_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/arm_controller/command", 10);

  // init
  _joint_state.name.resize(6);
  _joint_state.position.resize(6);
  _joint_state.velocity.resize(6);
  _joint_state.effort.resize(6);
}


JogControl::~JogControl()
{

}


void JogControl::moveTrajectory(const tf::Transform& goal)
{
  double M[16] = {1.0, 0.0, 0.0, 0.0,
                  0.0, 1.0, 0.0, 0.0,
                  0.0, 0.0, 1.0, 0.0,
                  0.0, 0.0, 0.0, 1.0};
  std::vector<double> q;
  q.resize(6*8, 0.0);

  tf::Matrix3x3 R = goal.getBasis();
  tf::Vector3 T = goal.getOrigin();
  
  M[0] = R[0][0]; M[1] = R[0][1]; M[2]  = R[0][2];  M[3]  = T[0];
  M[4] = R[1][0]; M[5] = R[1][1]; M[6]  = R[1][2];  M[7]  = T[1];
  M[8] = R[2][0]; M[9] = R[2][1]; M[10] = R[2][2];  M[11] = T[2];

  // printf("---TARGET---\n");
  // for(int i=0; i<4; i++)
  // {
  //   for(int j=0; j<4; j++)
  //   {
  //     printf("%lf, ", M[j+4*i]);
  //   }
  //   printf("\n");
  // }

  tm_kinematics::inverse(M, &q[0], 0.1);


  double min_diff = -1.0;
  sensor_msgs::JointState joint_target;

  for(int i=0; i<8; i++)
  {
    sensor_msgs::JointState joint_temp;

    joint_temp.velocity.resize(6, 0.0);
    joint_temp.effort.resize(6, 0.0);

    for(const auto& [ key, value ] : _joint_map)
    {
      joint_temp.name.push_back(key);
      joint_temp.position.push_back(q[i*6+value]);
    }

    double diff = 0.0;
    for(int i=0; i<6; i++)
    {
      diff += (joint_temp.position[i] - _joint_state.position[i]) * (joint_temp.position[i] - _joint_state.position[i]);
    }

    if(min_diff < 0 || diff < min_diff)
    {
      joint_target = joint_temp;
      min_diff = diff;
    }
  }

  trajectory_msgs::JointTrajectoryPoint point_target;
  trajectory_msgs::JointTrajectory traj_target;

  point_target.positions = joint_target.position;
  point_target.velocities.resize(6, 0.0);
  point_target.accelerations.resize(6, 0.0);
  point_target.effort.resize(6,  0.0);
  point_target.time_from_start = ros::Duration(0.1);
  
  traj_target.joint_names = joint_target.name;
  traj_target.points.push_back(point_target);

  _cmd_pub.publish(traj_target);
}


void JogControl::cbJointState(const sensor_msgs::JointStateConstPtr& js)
{
  double q[6];
  double M[16];
  int axis_id;
  double tx, ty, tz, qx, qy, qz, qw;  

  // now joint position copy
  _joint_state = *js;

  size_t joint_num = js->name.size();
  
  for(int i=0; i<joint_num; i++)
  {
    axis_id = _joint_map.at(js->name[i]);
    q[axis_id] = js->position[i];
  }  
  
  tm_kinematics::forward(q, M);


  tf::Matrix3x3 mat;
  tf::Quaternion qua;

  mat.setValue(M[0], M[1], M[2], M[4], M[5], M[6], M[8], M[9], M[10]);
  mat.getRotation(qua);

  _pose.setOrigin(tf::Vector3(M[3], M[7], M[11]));
  _pose.setRotation(qua);
}


