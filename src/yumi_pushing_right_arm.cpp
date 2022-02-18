#include <ros/ros.h>
#include <kdl_wrapper/kdl_wrapper.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <sstream>
#include <string>
#include <vector>
#include <time.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <signal.h>
#include <boost/lexical_cast.hpp>

using namespace std;
#define JOINT_VELOCITY_LIMIT 0.05

// global variables
vector<double> push_cmd;
double push_cmd_time = -1;
volatile sig_atomic_t flag = 0;
KDL::JntArray right_arm_joint_positions;
vector<double> right_arm_joint_velocity;

// function declarations
vector <double> str2vector(string str, int n_data);
double limit_joint_velcmd(double cmd, int joint);
void print_joint_values();

void terminate(int sig){
  flag = 1;
}

void joint_state_callback(const sensor_msgs::JointState & msg)
{
    int right_arm_indecis[7] = {1,3,13,5,7,9,11};
    for(int i = 0;i < 7; i++)
    {
        right_arm_joint_positions(i) = msg.position[right_arm_indecis[i]];
        right_arm_joint_velocity[i] = msg.velocity[right_arm_indecis[i]];
    }
}

void update_traj_callback(const std_msgs::String & msg)
{
  push_cmd = str2vector(msg.data, 3);
  push_cmd_time = ros::Time::now().toSec();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "yumi_pushing_node");

  right_arm_joint_positions.resize(7);
  right_arm_joint_velocity.resize(7);
  push_cmd.resize(3);
  std_msgs::Float64 cmd;
  KDLWrapper right_arm_kdl_wrapper;
  KDL::Twist right_arm_cart_velocity;
  KDL::JntArray right_arm_joint_velcmd(7);
  string command_topic;
  KDL::Frame right_tool_tip_frame;

  signal(SIGINT, terminate);
  srand (time(NULL));
  std::cout << std::fixed << std::setprecision(3);
  ros::NodeHandle joint_node;
  ros::Subscriber joint_subscriber = joint_node.subscribe("/yumi/joint_states", 1, joint_state_callback);
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  ros::NodeHandle param_node;
  vector<ros::NodeHandle> r_velocity_command_node(7);
  vector<ros::Publisher> r_velocity_command_pub(7);
  int urdf_order[7] = {1,2,7,3,4,5,6};
  for(int i = 0; i < 7; i++)
  {
    command_topic = "yumi/joint_vel_controller_" + to_string(urdf_order[i]) + "_r/command";
    r_velocity_command_pub[i] = r_velocity_command_node[i].advertise<std_msgs::Float64>(command_topic.c_str(), 10);
  }
  ros::NodeHandle cmd_node;
  ros::Subscriber cmd_subscriber = cmd_node.subscribe("/socket_string_msg", 10000, update_traj_callback);

  spinner.start();
  cout << "yumi_pushing_node started" << endl;

  // ros parameters
  double cmd_timeout = 0.5;
  param_node.getParam("/command_timeout", cmd_timeout);
  vector<double> r_init_joint_position;
  param_node.getParam("/initial_joint_position/right_arm", r_init_joint_position);

  if(!right_arm_kdl_wrapper.init("yumi_body", "yumi_link_7_r"))
      ROS_ERROR("Error initiliazing right_arm_kdl_wrapper");
  right_arm_kdl_wrapper.ik_solver_vel->setLambda(0.3);

  usleep(1000000);
  print_joint_values();

  char user_resp;
  cout << "Press any key to continue (ctrl+c to kill)!" << endl;
  cin >> user_resp;
  cmd.data = 0;
  for (int i = 0; i < 7; i++)
    r_velocity_command_pub[i].publish(cmd);
  if (flag)
    return 0;

  right_arm_kdl_wrapper.fk_solver_pos->JntToCart(right_arm_joint_positions, right_tool_tip_frame, -1);
  cout << "x: " <<right_tool_tip_frame.p(0) << ", y:" << right_tool_tip_frame.p(1) << ", z:" << right_tool_tip_frame.p(2) << endl;
  double ee_height_setpoint = 0.187; //= right_tool_tip_frame.p(2);

  cout << "end effector height: " << ee_height_setpoint << endl;

  while(ros::ok())
  {
    // move the arms to the initial position
    bool all_fine = false;
    all_fine = false;
    while(all_fine == false)
    {
      right_arm_kdl_wrapper.fk_solver_pos->JntToCart(right_arm_joint_positions, right_tool_tip_frame, -1);
      if (right_tool_tip_frame.p(2) < (ee_height_setpoint-0.05))
      {
        while (right_tool_tip_frame.p(2) < (ee_height_setpoint+0.005))
        {
	        right_arm_cart_velocity.vel = KDL::Vector(0.0, 0.0, 0.01);
          right_arm_cart_velocity.rot = KDL::Vector(0.0, 0.0, 0.0);
          right_arm_kdl_wrapper.ik_solver_vel->CartToJnt(right_arm_joint_positions, right_arm_cart_velocity, right_arm_joint_velcmd);
          for(int i = 0; i < 7; i++)
          {
            cmd.data = right_arm_joint_velcmd(i);
            r_velocity_command_pub[i].publish(cmd);
          }
          usleep(50000); //wait 50 msec
          right_arm_kdl_wrapper.fk_solver_pos->JntToCart(right_arm_joint_positions, right_tool_tip_frame, -1);
        }
      }
      all_fine = true;
      for (int i = 0; i < 7; i++)
      {
        cmd.data = 0.3*(r_init_joint_position[i]-right_arm_joint_positions(i));
        cmd.data = limit_joint_velcmd(cmd.data, i);
        r_velocity_command_pub[i].publish(cmd);
        if(abs(r_init_joint_position[i]-right_arm_joint_positions(i))>0.02)
          all_fine = false;
      }
    }
    cout << "joints in intial postiion!" << endl;
    usleep(2000000);
    right_arm_kdl_wrapper.fk_solver_pos->JntToCart(right_arm_joint_positions, right_tool_tip_frame, -1);
    double ee_x_setpoint = right_tool_tip_frame.p(0) + 0.0;
    double ee_y_setpoint = right_tool_tip_frame.p(1) + 0.0;
    all_fine = false;
    while (all_fine == false)
    {
      double vx = 1.0*(ee_x_setpoint - right_tool_tip_frame.p(0));
      double vy = 1.0*(ee_y_setpoint - right_tool_tip_frame.p(1));
      double vz = 1.0*(ee_height_setpoint - right_tool_tip_frame.p(2));
      right_arm_cart_velocity.vel = KDL::Vector(vx, vy, vz);
      right_arm_cart_velocity.rot = KDL::Vector(0.0, 0.0, 0.0);
      right_arm_kdl_wrapper.ik_solver_vel->CartToJnt(right_arm_joint_positions, right_arm_cart_velocity, right_arm_joint_velcmd);
      for(int i = 0; i < 7; i++)
      {
        cmd.data = right_arm_joint_velcmd(i);
        r_velocity_command_pub[i].publish(cmd);
      }
      usleep(50000); //wait 50 msec
      right_arm_kdl_wrapper.fk_solver_pos->JntToCart(right_arm_joint_positions, right_tool_tip_frame, -1);
      if (abs(vx) < 0.001 && abs(vy) < 0.001 && abs(vz) < 0.001)
        all_fine = true;
    }
    cout << "cart control done!" << endl;
    cmd.data = 0;
    for (int i = 0; i < 7; i++)
      r_velocity_command_pub[i].publish(cmd);
    if (flag)
      return 0;

    // wait until commands are received
    while(push_cmd_time < 0)
    {
      usleep(1000);
      if (flag)
        return 0;
    }

    // follow the given commands
    while(ros::ok())
    {
      double time_since_last_cmd = ros::Time::now().toSec() - push_cmd_time;
      if (time_since_last_cmd > cmd_timeout)
      {
        cout << "Commands stopped - restarting!" << endl;
        cmd.data = 0;
        for(int i = 0; i < 7; i++)
          r_velocity_command_pub[i].publish(cmd);
        push_cmd_time = -1;
        break;
      }
      if (flag)
      {
        cmd.data = 0;
        for(int i = 0; i < 7; i++)
          r_velocity_command_pub[i].publish(cmd);
        return 0;
      }
      right_arm_kdl_wrapper.fk_solver_pos->JntToCart(right_arm_joint_positions, right_tool_tip_frame, -1);
      double ee_height_error = ee_height_setpoint - right_tool_tip_frame.p(2);
      cout << ee_height_setpoint << endl;
      right_arm_cart_velocity.vel = KDL::Vector(push_cmd[0], push_cmd[1], ee_height_error*1.0);
      right_arm_cart_velocity.rot = KDL::Vector(0.0, 0.0, push_cmd[2]);
      right_arm_kdl_wrapper.ik_solver_vel->CartToJnt(right_arm_joint_positions, right_arm_cart_velocity, right_arm_joint_velcmd);
      for(int i = 0; i < 7; i++)
      {
        cmd.data = right_arm_joint_velcmd(i);
        r_velocity_command_pub[i].publish(cmd);
      }
      usleep(50000); //wait 50 msec
    }
  }
  cout << "node terminated successfully" << endl;
  return 0;
}

vector <double> str2vector(string str, int n_data)
{
  vector<double> data;
  for (int j = 0; j < n_data; j++)
  {
    std::size_t p = str.find(" ");
    string value = str.substr(0, p);
    str = str.substr(p + 1);
    data.push_back(boost::lexical_cast<double>(value));
  }
  return data;
}

double limit_joint_velcmd(double cmd, int joint)
{
  double limited_cmd = cmd;
  double joint_vel = right_arm_joint_velocity[joint];
  if((cmd - joint_vel) > JOINT_VELOCITY_LIMIT)
    limited_cmd = JOINT_VELOCITY_LIMIT;
  else if((cmd - joint_vel) < (-JOINT_VELOCITY_LIMIT))
    limited_cmd = -JOINT_VELOCITY_LIMIT;
  return limited_cmd;
}

void print_joint_values()
{
  for(int i = 0; i < 7; i++)
  {
    if (right_arm_joint_positions(i) >= 0)
      std::cout << " ";
    std::cout << right_arm_joint_positions(i) << ", ";
  }
  cout << endl;
}
