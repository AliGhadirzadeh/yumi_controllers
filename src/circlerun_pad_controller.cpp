#include <ros/ros.h>
// KDL stuff
#include <kdl_wrapper/kdl_wrapper.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

#include "yumi_controllers/circlerun_pad_controller.h"
#include <sstream>
#include <vector>
#include <string>
using namespace std;

KDL::JntArray right_arm_joint_positions;
KDL::JntArray left_arm_joint_positions;
vector<double> right_arm_joint_velocity;
vector<double> left_arm_joint_velocity;
std::vector<double> ball;
double ballmsg_timestamp = -1;

// function declarations
void print_joint_values();
double limit_joint_velcmd(double cmd, int joint, int arm, bool & limited);

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
void ball_x_callback(const std_msgs::Float64 & msg)
{
  ball[0] = msg.data;
  ballmsg_timestamp = ros::Time::now().toSec();
}
void ball_y_callback(const std_msgs::Float64 & msg)
{
  ball[1] = msg.data;
}
void ball_acc_x_callback(const std_msgs::Float64 & msg)
{
  ball[2] = msg.data;
}
void ball_acc_y_callback(const std_msgs::Float64 & msg)
{
  ball[3] = msg.data;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "circlerun_pad_controller"); // init the ROS node
  right_arm_joint_positions.resize(7);
  right_arm_joint_velocity.resize(7);
  left_arm_joint_positions.resize(7);
  left_arm_joint_velocity.resize(7);
  ball.resize(4);
  ros::NodeHandle joint_node;
  ros::Subscriber joint_subscriber = joint_node.subscribe("/yumi/joint_states", 1, joint_state_callback);
  ros::NodeHandle ball_x_node;
  ros::Subscriber ball_x_sub = ball_x_node.subscribe("/circlerun_ball_x", 1, ball_x_callback);
  ros::NodeHandle ball_y_node;
  ros::Subscriber ball_y_sub = ball_y_node.subscribe("/circlerun_ball_y", 1, ball_y_callback);
  ros::NodeHandle ball_acc_x_node;
  ros::Subscriber ball_acc_x_sub = ball_acc_x_node.subscribe("/circlerun_ball_acc_x", 1, ball_acc_x_callback);
  ros::NodeHandle ball_acc_y_node;
  ros::Subscriber ball_acc_y_sub = ball_acc_y_node.subscribe("/circlerun_ball_acc_y", 1, ball_acc_y_callback);
  string command_topic;

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

  ros::AsyncSpinner spinner(1);
  spinner.start();

  srand( time(NULL) );

  sleep(5);
  print_joint_values();

  double run_duration = 120;
  if (argc > 1)
    run_duration = atof(argv[1]);


  std_msgs::Float64 cmd;
  bool ball_msg_ok = false;

  KDLWrapper right_arm_kdl_wrapper;
  KDLWrapper left_arm_kdl_wrapper;

  if(!right_arm_kdl_wrapper.init("yumi_body", "yumi_link_7_r"))
      ROS_ERROR("Error initiliazing right_arm_kdl_wrapper");
  if(!left_arm_kdl_wrapper.init("yumi_body", "yumi_link_7_l"))
      ROS_ERROR("Error initiliazing left_arm_kdl_wrapper");

  right_arm_kdl_wrapper.ik_solver_vel->setLambda(0.3);
  left_arm_kdl_wrapper.ik_solver_vel->setLambda(0.3);

  KDL::JntArray right_arm_joint_velcmd(7);
  KDL::JntArray left_arm_joint_velcmd(7);
  KDL::Frame right_tool_tip_frame, left_tool_tip_frame;

  char user_resp;

  cout << "Move the arms to initi pose\n?";
  cout << "\t(y to yes, 0 to terminate, or any other character to continue)" << endl;
  cin >> user_resp;
  if(user_resp == 'y')
  {
    bool all_fine = false;
    bool limited;
    while(all_fine == false)
    {
      all_fine = true;
      for (int i = 0; i < 7; i++)
      {
        cmd.data = 0.3*(right_arm_presets[0][i]-right_arm_joint_positions(i));
        cmd.data = limit_joint_velcmd(cmd.data, i, RIGHT_ARM, limited);
        r_velocity_command_pub[i].publish(cmd);
        if(abs(right_arm_presets[0][i]-right_arm_joint_positions(i))>0.02)
          all_fine = false;
        cmd.data = 0.3*(left_arm_presets[0][i]-left_arm_joint_positions(i));
        cmd.data = limit_joint_velcmd(cmd.data, i, LEFT_ARM, limited);
        l_velocity_command_pub[i].publish(cmd);
        if (abs(left_arm_presets[0][i]-left_arm_joint_positions(i))>0.02)
          all_fine = false;
      }
      usleep(50000);
    }
    for (int i = 0; i < 7; i++)
    {
      cmd.data = 0.0;
      l_velocity_command_pub[i].publish(cmd);
      r_velocity_command_pub[i].publish(cmd);
    }
    print_joint_values();
    cout << "Press any key to continue or 0 to terminate" << endl;
    cin >> user_resp;
    if(user_resp == '0')
      return 0;
  }
  else if(user_resp == '0')
    return 0;

  KDL::Twist right_arm_cart_velocity;
  KDL::Twist left_arm_cart_velocity;
  right_arm_cart_velocity.vel = KDL::Vector(0.0, 0.0, 0.0);
  right_arm_cart_velocity.rot = KDL::Vector(0.0, 0.0, 0.0);
  left_arm_cart_velocity.vel = KDL::Vector(0.0, 0.0, 0.0);
  left_arm_cart_velocity.rot = KDL::Vector(0.0, 0.0, 0.0);

  // timing
  double current_time, start_time, time_to_next_motor_cmd, time_to_next_cmd, time_to_next_print, dt;
  start_time = ros::Time::now().toSec();
  time_to_next_motor_cmd = start_time;
  time_to_next_cmd = start_time;
  time_to_next_print = start_time;
  current_time = start_time;

  while (ros::ok()) {
    dt = ros::Time::now().toSec() - current_time;
    current_time += dt;
    if (current_time > (start_time + run_duration))
      break;
    if (ballmsg_timestamp < 0 || (current_time - ballmsg_timestamp) > 0.2)
      ball_msg_ok = false;
    else
      ball_msg_ok = true;
    if (current_time > time_to_next_cmd)
    {
      if (ball[0] == 0 && ball[1] == 0)   // this is to avoid errors for atan2
      {
        ball[0] += 0.01;
        ball[1] += 0.01;
      }
      double gain_pos = 0.1;
      double gain_vel = 0.5;
      double rad = 2.0 / 3.0;
      double theta = atan2(ball[1], ball[0]);
      theta += pi / 12;
      double ball_x = rad * cos(theta);
      double ball_y = rad * sin(theta);
      double cmd_x = gain_pos * (ball_x - ball[0]);
      double cmd_y = gain_pos * (ball_y - ball[1]);
      double right_arm_vel = gain_vel * (cmd_x-ball[2]);
      double left_arm_vel = -gain_vel * (cmd_x-ball[2]);
      right_arm_vel += -gain_vel* (cmd_y - ball[3]);
      left_arm_vel += -gain_vel* (cmd_y - ball[3]);

      if(right_arm_vel > 0.02)
        right_arm_vel = 0.02;
      if(right_arm_vel < -0.02)
        right_arm_vel = -0.02;
      if(left_arm_vel > 0.02)
        left_arm_vel = 0.02;
      if(left_arm_vel < -0.02)
        left_arm_vel = -0.02;

      right_arm_cart_velocity.vel(2) = right_arm_vel;
      left_arm_cart_velocity.vel(2) = left_arm_vel;
/*
      cout << "x:    \t" << ball[0] << endl;
      cout << "y:    \t" << ball[1] << endl;
      cout << "theta:\t" << theta << endl;
      cout << "x_tar:\t" << ball_x << endl;
      cout << "y_tar:\t" << ball_y << endl;
      cout << "x_cmd:\t" << cmd_x << endl;
      cout << "y_cmd:\t" << cmd_y << endl;
      cout << "rvel: \t" << right_arm_vel << endl;
      cout << "lvel: \t" << left_arm_vel << endl;
      cout << "\n\n" << endl;*/

      time_to_next_cmd += 0.01;
      right_arm_kdl_wrapper.fk_solver_pos->JntToCart(right_arm_joint_positions, right_tool_tip_frame, -1);
      left_arm_kdl_wrapper.fk_solver_pos->JntToCart(left_arm_joint_positions, left_tool_tip_frame, -1);
    }

    if (current_time > time_to_next_motor_cmd)
    {
      time_to_next_motor_cmd += 0.005;
      right_arm_kdl_wrapper.ik_solver_vel->CartToJnt(right_arm_joint_positions, right_arm_cart_velocity, right_arm_joint_velcmd);
      left_arm_kdl_wrapper.ik_solver_vel->CartToJnt(left_arm_joint_positions, left_arm_cart_velocity, left_arm_joint_velcmd);
      if (ball_msg_ok)
      {
        for(int i = 0; i < 7; i++)
        {
          cmd.data = right_arm_joint_velcmd(i);
          r_velocity_command_pub[i].publish(cmd);
          cmd.data = left_arm_joint_velcmd(i);
          l_velocity_command_pub[i].publish(cmd);
        }
      }
      else
      {
        for(int i = 0; i < 7; i++)
        {
          cmd.data = 0;
          r_velocity_command_pub[i].publish(cmd);
          l_velocity_command_pub[i].publish(cmd);
        }
      }
    }
    if(current_time > time_to_next_print)
    {
      cout << right_tool_tip_frame.p(0) << "\t\t" << right_tool_tip_frame.p(1) << "\t\t" << right_tool_tip_frame.p(2) << "\t\t\t\t";
      cout << left_tool_tip_frame.p(0) << "\t\t" << left_tool_tip_frame.p(1) << "\t\t" << left_tool_tip_frame.p(2) << endl;
      if(ball_msg_ok == false)
        cout << "ball messages are not received!" << endl;
      time_to_next_print += 2;
    }
  }
  cmd.data = 0;
  for(int i = 0; i < 7; i++)
  {
    r_velocity_command_pub[i].publish(cmd);
    l_velocity_command_pub[i].publish(cmd);
  }
  std::cout << "I am done!" << std::endl;
  return 0;
}


void print_joint_values()
{
  std::cout << std::setprecision(3);
  std::cout << "RIGHT ARM JOINTS: ";
  for(int i = 0; i < 7; i++)
  {
    if (right_arm_joint_positions(i) >= 0)
      std::cout << " ";
    std::cout << right_arm_joint_positions(i) << ", ";
  }
  std::cout << std::endl;
  std::cout << "LEFT ARM JOINTS:  ";
  for(int i = 0; i < 7; i++)
  {
    if (left_arm_joint_positions(i) >= 0)
      std::cout << " ";
    std::cout << left_arm_joint_positions(i) << ", ";
  }
  std::cout << std::endl;
}

double limit_joint_velcmd(double cmd, int joint, int arm, bool & limited)
{
  double limited_cmd = cmd;
  limited = 0;
  double joint_vel = 0;
  if(arm == RIGHT_ARM)
    joint_vel = right_arm_joint_velocity[joint];
  else if(arm == LEFT_ARM)
    joint_vel = left_arm_joint_velocity[joint];
  else
  {
    cout << "Unknown arm passed!" << endl;
    return 0;
  }
  if((cmd - joint_vel) > joint_velcmd_limit[joint])
  {
    limited = 1;
    limited_cmd = joint_velcmd_limit[joint];
  }
  else if((cmd - joint_vel) < (-joint_velcmd_limit[joint]))
  {
    limited = 1;
    limited_cmd = -joint_velcmd_limit[joint];
  }
  return limited_cmd;
}
