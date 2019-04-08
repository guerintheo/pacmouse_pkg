#include "ros/ros.h"
#include "std_msgs/String.h"
#include "pacmouse_pkg/Drive.h"
#include <iostream>

#include <pigpio.h>
#include <unistd.h>
#include <signal.h>

#include "rotary_encoder.hpp"
#include "rotary_encoder.cpp"

#define ENCODER_A_1 23
#define ENCODER_B_1 24

#define ENCODER_A_2 22
#define ENCODER_B_2 27

#define MOTOR_MODE_PIN 25
#define L_MOT_GPIO 12
#define R_MOT_GPIO 13
#define L_MOT_DIR 7
#define R_MOT_DIR 16

#define MOTOR_PWM_FREQUENCY 100
 
//TODO: Figure out a way to import these parameters rather than recompiling source when you want to change them. 
float LOOP_RATE = 10;

int pos_1 = 0;
int pos_2 = 0;

re_decoder *decoder1 = NULL;
re_decoder *decoder2 = NULL;

void motor_command_callback(const pacmouse_pkg::Drive::ConstPtr& msg)
{
  ROS_INFO("Setting PWM value to: [%f, %f]", msg->L, msg->R);
  // TODO: Add support for negative values by setting the direction pin to 1 if backwards is desired.

        // # set the directions of the motors. 0 is forward, 1 is backward
  gpioWrite(L_MOT_DIR, (int)(msg->L < 0));
  gpioWrite(R_MOT_DIR, (int)(msg->R < 0));
  gpioPWM(L_MOT_GPIO, abs((int)(msg->L * 255)));
  gpioPWM(R_MOT_GPIO, abs((int)(msg->R * 255)));

}

void callback_1(int way)
{
  pos_1 += way;
}

void callback_2(int way)
{
  pos_2 += way;
}

//These could overflow after 11 days of continous full speed running... 
int last_pos_1 = 0;
int last_pos_2 = 0;
int current_pos_1 = 0;
int current_pos_2 = 0;
float vel_1 = 0.0;
float vel_2 = 0.0;
// Calculates the instantaneous velocity given an update rate in hertz. (In number of ticks per second)
float calc_velocity_1(float hertz)
{
  last_pos_1 = current_pos_1;
  current_pos_1 = pos_1;
  vel_1 = (current_pos_1 - last_pos_1) * hertz;

  return vel_1;
}

float calc_velocity_2(float hertz)
{
  last_pos_2 = current_pos_2;
  current_pos_2 = pos_2;
  vel_2 = (current_pos_2 - last_pos_2) * hertz;
  
  return vel_2;
}

void publish_encoders(ros::Publisher encoder_publisher) {
  std_msgs::String msg;

  vel_1 = calc_velocity_1(LOOP_RATE);
  vel_2 = calc_velocity_2(LOOP_RATE);

  std::stringstream ss;
  ss << "pos1: " << pos_1 << " vel1: " << vel_1 << " pos2: " << pos_2 << " vel1: " << vel_2;
  // TODO: Define this message structure
  msg.data = ss.str();
  ROS_INFO("%s", msg.data.c_str());

  encoder_publisher.publish(msg);
}

// cleanup pigpio and callbacks 
void shutdown(int s){
  decoder1->re_cancel();
  decoder2->re_cancel();
  gpioTerminate();

  printf("Caught signal %d\n",s);
  ros::shutdown();

}

int main(int argc, char **argv)
{
  /**
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "motor_test");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

// SHUTDOWN CODE
  // struct sigaction sigIntHandler;

  // sigIntHandler.sa_handler = shutdown;
  // sigemptyset(&sigIntHandler.sa_mask);
  // sigIntHandler.sa_flags = 0;

  // sigaction(SIGINT, &sigIntHandler, NULL);

// TODO: It seems that neither of these SIGINT handlers works. 

  signal(SIGINT, shutdown);

  ros::Subscriber sub = n.subscribe("/pacmouse/motor/cmd", 1000, motor_command_callback);

  ros::Publisher encoder_publisher = n.advertise<std_msgs::String>("/pacmouse/encoders", 1000);

  if (gpioInitialise() < 0) {
    ROS_INFO("Could not initialize GPIOs. I'm borked.");
    return 1;
  }

  // set all the motor pins as output
  gpioSetMode(L_MOT_GPIO, PI_OUTPUT);
  gpioSetMode(R_MOT_GPIO, PI_OUTPUT);
  gpioSetMode(L_MOT_DIR, PI_OUTPUT);
  gpioSetMode(R_MOT_DIR, PI_OUTPUT);

  // and set their motor frequency
  gpioSetPWMfrequency(L_MOT_GPIO, MOTOR_PWM_FREQUENCY);
  gpioSetPWMfrequency(R_MOT_GPIO, MOTOR_PWM_FREQUENCY);

  gpioSetMode(MOTOR_MODE_PIN, PI_OUTPUT);
  gpioWrite(MOTOR_MODE_PIN, 1);

  decoder1 = new re_decoder(ENCODER_A_1, ENCODER_B_1, callback_1);
  decoder2 = new re_decoder(ENCODER_A_2, ENCODER_B_2, callback_2);

  ros::Rate loop_rate(LOOP_RATE);

  while (ros::ok())
  {

    publish_encoders(encoder_publisher);

    ros::spinOnce();

    loop_rate.sleep();

  }
  return 0;
}
