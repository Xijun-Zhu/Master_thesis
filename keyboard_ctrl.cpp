#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "GeographicLib/Geoid.hpp"
#include <signal.h>
#include <stdio.h>
#ifndef _WIN32
# include <termios.h>
# include <unistd.h>
#else
# include <windows.h>
#endif


#define MOVE_UNIT_NUM   1     // Move 1 unit(s) every time move key pressed, 1 unit = 10 cm
#define DESCEND_SPPED   1     // 2 unit(s) per second, 1 unit = 10 cm
#define DESCEND_HEIGHT  40     // Descend 15 unit(s) , 1 unit = 10 cm

#define KEYCODE_RIGHT 0x43
#define KEYCODE_LEFT 0x44
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
#define KEYCODE_B 0x62
#define KEYCODE_C 0x63
#define KEYCODE_D 0x64
#define KEYCODE_E 0x65
#define KEYCODE_F 0x66
#define KEYCODE_G 0x67
#define KEYCODE_Q 0x71
#define KEYCODE_R 0x72
#define KEYCODE_T 0x74
#define KEYCODE_V 0x76

#define MANUAL_MODE       0
#define AUTONOMOUS_MODE   1

bool flight_mode = MANUAL_MODE;   // 0: mannual   1: autonomous

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geographic_msgs::GeoPoseStamped goal_position;
void goal_position_cb(const geographic_msgs::GeoPoseStamped::ConstPtr& pose_){
    goal_position = *pose_;
}

class KeyboardReader
{
public:
  KeyboardReader()
#ifndef _WIN32
    : kfd(0)
#endif
  {
#ifndef _WIN32
    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    struct termios raw;
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
#endif
  }
  void readOne(char * c)
  {
#ifndef _WIN32
    int rc = read(kfd, c, 1);
    if (rc < 0)
    {
      throw std::runtime_error("read failed");
    }
#else
    for(;;)
    {
      HANDLE handle = GetStdHandle(STD_INPUT_HANDLE);
      INPUT_RECORD buffer;
      DWORD events;
      PeekConsoleInput(handle, &buffer, 1, &events);
      if(events > 0)
      {
        ReadConsoleInput(handle, &buffer, 1, &events);
        if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_LEFT)
        {
          *c = KEYCODE_LEFT;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_UP)
        {
          *c = KEYCODE_UP;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_RIGHT)
        {
          *c = KEYCODE_RIGHT;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_DOWN)
        {
          *c = KEYCODE_DOWN;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x52) // R key
        {
          *c = KEYCODE_R;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x46) // F key
        {
          *c = KEYCODE_F;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x51) // Q key
        {
          *c = KEYCODE_Q;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x43) // C key
        {
          *c = KEYCODE_C;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x54) // T key
        {
          *c = KEYCODE_T;
          return;
        }
      }
    }
#endif
  }
  void shutdown()
  {
#ifndef _WIN32
    tcsetattr(kfd, TCSANOW, &cooked);
#endif
  }
private:
#ifndef _WIN32
  int kfd;
  struct termios cooked;
#endif
};

KeyboardReader input;

class TeleopTurtle
{
public:
  TeleopTurtle();
  void keyLoop();

private:

  
  ros::NodeHandle nh;
  ros::Subscriber state_sub;
  ros::Publisher local_pos_pub;
  ros::ServiceClient arming_client;
  ros::ServiceClient set_mode_client;
  ros::Publisher goal_pos_pub;
  ros::Subscriber goal_pos_sub;
};

TeleopTurtle::TeleopTurtle():
{
  state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
  arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
  set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");    
  goal_pos_pub = nh.advertise < geographic_msgs::GeoPoseStamped > 
            ("mavros/setpoint_position/global", 10);
  goal_pos_sub = nh.subscribe<geographic_msgs::GeoPoseStamped> 
            ("mavros/setpoint_position/global", 10, goal_position_cb);
}

void quit(int sig)
{
  (void)sig;
  input.shutdown();
  ros::shutdown();
  exit(0);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "KeyboardCtrl");
  TeleopTurtle teleop_turtle;

  signal(SIGINT,quit);

  teleop_turtle.keyLoop();
  quit(0);
  
  return(0);
}


void TeleopTurtle::keyLoop()
{
  char c;
  bool dirty=false;
  if(flight_mode == 0) puts("Manual mode!");
  else puts("Autonomous mode!");

  puts("Reading from keyboard"); 
  puts("---------------------------");
  puts("Use arrow keys to move the drone.");
  puts("'t' to fly to two meters high. 'c' to change mode. 'q' to quit.");

  ros::Rate rate(10);
  ros::Rate rate_atn_mode(DESCEND_SPPED);

  // wait for FCU connection
  while(ros::ok() && !current_state.connected){
      ros::spinOnce();
      rate.sleep();
  }

  for (int i=0; i<20; ++i) {
    ros::spinOnce();
    rate.sleep();
  }

  for(;;)
  {
    // get the next event from the keyboard  
    try
    {
      input.readOne(&c);
    }
    catch (const std::runtime_error &)
    {
      perror("read():");
      return;
    }

    linear_=angular_=0;
    ROS_DEBUG("value: 0x%02X\n", c);
  
    switch(c)
    {
      case KEYCODE_LEFT:
        ROS_INFO("WEST");
        goal_position.header.stamp = ros::Time::now();
        goal_position.pose.position.longitude -= 0.00000162 * MOVE_UNIT_NUM;
        dirty = true;
        break;
      case KEYCODE_RIGHT:
        ROS_INFO("EAST");
        goal_position.header.stamp = ros::Time::now();
        goal_position.pose.position.longitude += 0.00000162 * MOVE_UNIT_NUM;
        dirty = true;
        break;
      case KEYCODE_UP:       
        ROS_INFO("NORTH");
        goal_position.header.stamp = ros::Time::now();
        goal_position.pose.position.latitude += 0.0000009 * MOVE_UNIT_NUM;
        dirty = true;
        break;
      case KEYCODE_DOWN:
        ROS_INFO("SOUTH");
        goal_position.header.stamp = ros::Time::now();
        goal_position.pose.position.latitude -= 0.0000009 * MOVE_UNIT_NUM;
        dirty = true;
        break;
      case KEYCODE_R:
        ROS_INFO("RISE");
        goal_position.header.stamp = ros::Time::now();
        goal_position.pose.position.altitude += 0.1 * MOVE_UNIT_NUM;
        dirty = true;
        break;
      case KEYCODE_F:
        ROS_INFO("FALL");
        goal_position.header.stamp = ros::Time::now();
        goal_position.pose.position.altitude -= 0.1 * MOVE_UNIT_NUM;
        dirty = true;
        break;
      case KEYCODE_C:
        if(flight_mode == AUTONOMOUS_MODE){
          flight_mode = MANUAL_MODE;
          puts("Mode switch key pressed! Now change to manual mode!");
        }
        else {
          flight_mode = AUTONOMOUS_MODE;
          puts("Mode switch key pressed! Now change to autonomous mode!");
        }
        break;
      case KEYCODE_T:
        goal_position.header.stamp = ros::Time::now();
        goal_position.pose.position.altitude += DESCEND_HEIGHT * 0.1;
        goal_pos_pub.publish(goal_position);
        ROS_INFO("Fly to two meter above the ground.");
        break;
      case KEYCODE_Q:
        ROS_DEBUG("quit");
        return;
    }
   
    if(dirty ==true)
    {
      if(flight_mode == MANUAL_MODE)
        goal_pos_pub.publish(goal_position);
      else{
        ROS_INFO("Autonomous flight started...");
        goal_pos_pub.publish(goal_position);
        // Waiting until drone move 10 cm horizontally (5s)
        for(int i = 0; i < (DESCEND_SPPED * 5); i++)
        {
          ros::spinOnce();
          rate_atn_mode.sleep();
        }

        // Start Descending
        ROS_INFO("Start Descending...");

        for(int i = 0; i < DESCEND_HEIGHT; i++) 
        {
          goal_position.header.stamp = ros::Time::now();
          goal_position.pose.position.altitude -= 0.1;
          goal_pos_pub.publish(goal_position);
          ROS_INFO("Goal altitude: %.2f",goal_position.pose.position.altitude);
          rate_atn_mode.sleep();
          // ros::spinOnce();
        }

        ROS_INFO("Autonomous flight finished...");
      }
      dirty=false;
    }
  }
  return;
}


