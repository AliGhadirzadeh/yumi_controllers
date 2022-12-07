#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <sstream>
#include <string>
#include <vector>
#include <time.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <signal.h>
#include <boost/lexical_cast.hpp>
#include <kdl_wrapper/kdl_wrapper.h>

using namespace std;
#define JOINT_VELOCITY_LIMIT 0.05

// global variables
volatile sig_atomic_t flag = 0;
KDL::JntArray right_arm_joint_positions;
vector<double> right_arm_joint_velocity;
KDL::JntArray left_arm_joint_positions;
vector<double> left_arm_joint_velocity;

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
    int left_arm_indecis[7] = {0,2,12,4,6,8,10};
    for(int i = 0;i < 7; i++)
    {
        right_arm_joint_positions(i) = msg.position[right_arm_indecis[i]];
        left_arm_joint_positions(i) = msg.position[left_arm_indecis[i]];
        right_arm_joint_velocity[i] = msg.velocity[right_arm_indecis[i]];
        left_arm_joint_velocity[i] = msg.velocity[left_arm_indecis[i]];
    }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "yumi_pushing_node");

  right_arm_joint_positions.resize(7);
  right_arm_joint_velocity.resize(7);
  left_arm_joint_positions.resize(7);
  left_arm_joint_velocity.resize(7);
  string command_topic;
  std_msgs::Float64 cmd;

  signal(SIGINT, terminate);
  srand (time(NULL));
  std::cout << std::fixed << std::setprecision(3);
  ros::NodeHandle joint_node;
  ros::Subscriber joint_subscriber = joint_node.subscribe("/yumi/joint_states", 1, joint_state_callback);
  ros::AsyncSpinner spinner(1);
  ros::NodeHandle param_node;
  vector<ros::NodeHandle> r_velocity_command_node(7);
  vector<ros::Publisher> r_velocity_command_pub(7);
  vector<ros::NodeHandle> l_velocity_command_node(7);
  vector<ros::Publisher> l_velocity_command_pub(7);
  int urdf_order[7] = {1,2,7,3,4,5,6};
  for(int i = 0; i < 7; i++)
  {
    command_topic = "yumi/joint_vel_controller_" + to_string(urdf_order[i]) + "_r/command";
    r_velocity_command_pub[i] = r_velocity_command_node[i].advertise<std_msgs::Float64>(command_topic.c_str(), 10);
    command_topic = "yumi/joint_vel_controller_" + to_string(urdf_order[i]) + "_l/command";
    l_velocity_command_pub[i] = l_velocity_command_node[i].advertise<std_msgs::Float64>(command_topic.c_str(), 10);
  }

  spinner.start();
  cout << "goto_init_jpos_node started" << endl;
  cout << "The node moves both of the arms to the initial position defined by the launch file" << endl;
  cout << "Do not forget to run the launch file! " << endl;
  // ros parameters
  vector<double> r_init_joint_position, l_init_joint_position;
  param_node.getParam("/initial_joint_position/right_arm", r_init_joint_position);
  param_node.getParam("/initial_joint_position/left_arm", l_init_joint_position);

  usleep(1000000);
  print_joint_values();
  char user_resp;
  cout << "Press any key to continue (ctrl+c to kill)!" << endl;
  cin >> user_resp;
  cmd.data = 0;
  for (int i = 0; i < 7; i++)
  {
    r_velocity_command_pub[i].publish(cmd);
    l_velocity_command_pub[i].publish(cmd);
  }
  if (flag)
    return 0;

  while(ros::ok())
  {
    // move the arms to the initial position
    bool all_fine = true;
    for (int i = 0; i < 7; i++)
    {
      cmd.data = 0.3*(r_init_joint_position[i]-right_arm_joint_positions(i));
      cmd.data = limit_joint_velcmd(cmd.data, i);
      r_velocity_command_pub[i].publish(cmd);
      cmd.data = 0.3*(l_init_joint_position[i]-left_arm_joint_positions(i));
      cmd.data = limit_joint_velcmd(cmd.data, i);
      l_velocity_command_pub[i].publish(cmd);
      if(abs(r_init_joint_position[i]-right_arm_joint_positions(i))>0.02)
        all_fine = false;
      if(abs(l_init_joint_position[i]-left_arm_joint_positions(i))>0.02)
        all_fine = false;
    }
    if (all_fine)
      break;
    usleep(50000);
  }
  cout << "joints in intial postiion!" << endl;
  cmd.data = 0;
  for (int i = 0; i < 7; i++)
  {
    r_velocity_command_pub[i].publish(cmd);
    l_velocity_command_pub[i].publish(cmd);
  }
  usleep(1000000);
  cout << "node terminated successfully" << endl;
  return 0;
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
  cout << "right arm:" << endl;
  for(int i = 0; i < 7; i++)
  {
    if (right_arm_joint_positions(i) >= 0)
      std::cout << " ";
    std::cout << right_arm_joint_positions(i) << ", ";
  }
  cout << endl;
  cout << "left arm:" << endl;
  for(int i = 0; i < 7; i++)
  {
    if (left_arm_joint_positions(i) >= 0)
      std::cout << " ";
    std::cout << left_arm_joint_positions(i) << ", ";
  }
  cout << endl;
}
