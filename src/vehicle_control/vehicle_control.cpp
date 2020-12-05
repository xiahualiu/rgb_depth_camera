#include<ros/ros.h>
#include<unistd.h>
#include<map>
#include<geometry_msgs/Twist.h>
#include<termios.h>
#include<stdio.h>
#include<ros/console.h>

std::map<char, std::vector<float>> moveBindings
{
	{'w', {1,0}},
	{'a', {0,-1}},
	{'s', {-1,0}},
	{'d', {0,1}}
};

float speed(0.5);
float turn(1.0);

int getch(void)
{
  int ch;
  struct termios oldt;
  struct termios newt;

  // Store old settings, and copy to new settings
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;

  // Make required changes and apply the settings
  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_iflag |= IGNBRK;
  newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
  newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
  newt.c_cc[VMIN] = 1;
  newt.c_cc[VTIME] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &newt);

  // Get the current character
  ch = getchar();

  // Reapply old settings
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

  return ch;
}

int main(int argc, char** argv)
{
  char key='0';
  int x=0;
  int z=0;
  int th=0;
  // Init ROS node
  ros::init(argc, argv, "vehicle_control");
  ros::NodeHandle nh;
    
  // Init cmd_vel publisher
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/vehicle/cmd_vel", 1);
  ros::Rate loop_rate(50);
  // Create Twist message
  geometry_msgs::Twist twist;
  while(ros::ok)
    {
    // Get the pressed key
    key = getch();
    // If the key corresponds to a key in moveBindings
    if (moveBindings.find(key) != moveBindings.end())
    {
      // Grab the direction data
      x = moveBindings[key][0];
      th = moveBindings[key][1];
      fprintf(stdout,"\rCurrent: speed %f\tturn %f | Last command: %c   ", speed, turn, key);
    }

    // Otherwise, set the robot to stop
    else
    {
      x = 0;
      th = 0;
      // If ctrl-C (^C) was pressed, terminate the program
      if (key == '\x03')
      {
        break;
      }
    }

    // Update the Twist message
    twist.linear.x = x * speed;
    twist.linear.y = 0;
    twist.linear.z = 0;

    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = th * turn;

    // Publish it and resolve any remaining callbacks
    pub.publish(twist);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
