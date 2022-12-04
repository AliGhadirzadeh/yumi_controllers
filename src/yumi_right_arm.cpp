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
#define DIM_ACTION 7
#define NUM_JOINTS 7
#define EE_Z_LOW_THRESHOLD 0.05

// global variables
vector<double> action;
double action_time = -1;
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
  action = str2vector(msg.data, DIM_ACTION);
  action_time = ros::Time::now().toSec();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "yumi_right_arm_node");

  right_arm_joint_positions.resize(NUM_JOINTS);
  right_arm_joint_velocity.resize(NUM_JOINTS);
  action.resize(DIM_ACTION);
  std_msgs::Float64 cmd;
  KDLWrapper right_arm_kdl_wrapper;
  KDL::Twist right_arm_cart_velocity;
  KDL::JntArray right_arm_joint_velcmd(NUM_JOINTS);
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
  vector<ros::NodeHandle> r_velocity_command_node(NUM_JOINTS);
  vector<ros::Publisher> r_velocity_command_pub(NUM_JOINTS);
  int urdf_order[NUM_JOINTS] = {1,2,7,3,4,5,6};
  for(int i = 0; i < NUM_JOINTS; i++)
  {
    command_topic = "yumi/joint_vel_controller_" + to_string(urdf_order[i]) + "_r/command";
    r_velocity_command_pub[i] = r_velocity_command_node[i].advertise<std_msgs::Float64>(command_topic.c_str(), 10);
  }
  ros::NodeHandle cmd_node;
  ros::Subscriber cmd_subscriber = cmd_node.subscribe("/socket_string_msg", 10000, update_traj_callback);

  spinner.start();
  cout << "yumi_right_arm_node started" << endl;

  // ros parameters
  double cmd_timeout = 5; // 5 secs until reset
  double action_duration = 0.3 // 300 msecs to execut the velocity action
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
  cout << "x: " << right_tool_tip_frame.p(0) << ", y:" << right_tool_tip_frame.p(1) << ", z:" << right_tool_tip_frame.p(2) << endl;

  // move the arms to the initial position
  while(ros::ok())
  {
    bool all_fine = false;
    all_fine = false;
    while(all_fine == false)
    {
      right_arm_kdl_wrapper.fk_solver_pos->JntToCart(right_arm_joint_positions, right_tool_tip_frame, -1);
      if (right_tool_tip_frame.p(2) < (EE_Z_LOW_THRESHOLD+0.04))
      {
        while (right_tool_tip_frame.p(2) < (EE_Z_LOW_THRESHOLD+0.08))
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
    usleep(1000000);
    // randomly moving the arm
    right_arm_kdl_wrapper.fk_solver_pos->JntToCart(right_arm_joint_positions, right_tool_tip_frame, -1);
    double ee_x_setpoint = right_tool_tip_frame.p(0) + ((rand()/(RAND_MAX+1.0))-0.5)/10;
    double ee_y_setpoint = right_tool_tip_frame.p(1) + ((rand()/(RAND_MAX+1.0))-0.5)/10;
    all_fine = false;
    while (all_fine == false)
    {
      double vx = 1.0*(ee_x_setpoint - right_tool_tip_frame.p(0));
      double vy = 1.0*(ee_y_setpoint - right_tool_tip_frame.p(1));
      double vz = 1.0*(EE_Z_LOW_THRESHOLD + 0.05 - right_tool_tip_frame.p(2));
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
    while(action_time < 0)
    {
      usleep(1000);
      if (flag)
        return 0;
    }

    // follow the given commands
    while(ros::ok())
    {
      double time_since_last_cmd = ros::Time::now().toSec() - action_time;
      if (time_since_last_cmd > cmd_timeout)
      {
        cout << "Commands stopped - restarting!" << endl;
        cmd.data = 0;
        for(int i = 0; i < NUM_JOINTS; i++)
          r_velocity_command_pub[i].publish(cmd);
        action_time = -1;
        break;
      }
      if (flag)
      {
        cmd.data = 0;
        for(int i = 0; i < NUM_JOINTS; i++)
          r_velocity_command_pub[i].publish(cmd);
        return 0;
      }
      if (time_since_last_cmd > action_duration)
      {
        cmd.data = 0;
        for(int i = 0; i < NUM_JOINTS; i++)
          r_velocity_command_pub[i].publish(cmd);
      }
      else
      {
        right_arm_kdl_wrapper.fk_solver_pos->JntToCart(right_arm_joint_positions, right_tool_tip_frame, -1);
        right_arm_cart_velocity.vel = KDL::Vector(action[0], action[1], action[2]);
        right_arm_cart_velocity.rot = KDL::Vector(action[3], action[4], action[5]);
        right_arm_kdl_wrapper.ik_solver_vel->CartToJnt(right_arm_joint_positions, right_arm_cart_velocity, right_arm_joint_velcmd);
        for(int i = 0; i < NUM_JOINTS; i++)
        {
          cmd.data = right_arm_joint_velcmd(i);
          r_velocity_command_pub[i].publish(cmd);
        }
      }
      usleep(10000); //wait 10 msec
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
