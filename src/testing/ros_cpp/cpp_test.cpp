#include "ros/ros.h"
#include "pacmouse_pkg/Drive.h"
#include <iostream>

#include <pigpio.h>
#include <unistd.h>
#include <signal.h>

#include "rotary_encoder.hpp"
#include "rotary_encoder.cpp"


// Load these parameters from the ROS Parameter Server rather than recompiling
// source when you want to change them. To load the parameters from a YAML file,
// call `rosparam load src/params.yaml /pacmouse/params` from pacmouse_pkg/, and
// re-run this script.
float LOOP_RATE;

int BUTTON_1;
int BUTTON_2;
int BUTTON_3; 
int BUTTON_4;

int ENCODER_R_A;
int ENCODER_R_B;
int ENCODER_L_A;
int ENCODER_L_B;

int MOTOR_MODE_PIN;
int L_MOT_GPIO;
int R_MOT_GPIO;
int R_MOT_DIR;

float MOTOR_PWM_FREQUENCY;
float TICKS_PER_RADIAN;

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

void load_params(ros::NodeHandle n) {
    n.getParam("/pacmouse/params/button_1", BUTTON_1);
    n.getParam("/pacmouse/params/button_2", BUTTON_2);
    n.getParam("/pacmouse/params/button_3", BUTTON_3);
    n.getParam("/pacmouse/params/button_4", BUTTON_4);
    n.getParam("/pacmouse/params/encoder_freq", LOOP_RATE);
    n.getParam("/pacmouse/params/enc_r_a", ENCODER_R_A);
    n.getParam("/pacmouse/params/enc_r_b", ENCODER_R_B);
    n.getParam("/pacmouse/params/enc_l_a", ENCODER_L_A);
    n.getParam("/pacmouse/params/enc_l_b", ENCODER_L_B);
    n.getParam("/pacmouse/params/motor_mode_pin", MOTOR_MODE_PIN);
    n.getParam("/pacmouse/params/ml_pwm", L_MOT_GPIO);
    n.getParam("/pacmouse/params/mr_pwm", R_MOT_GPIO);
    n.getParam("/pacmouse/params/ml_dir", L_MOT_DIR);
    n.getParam("/pacmouse/params/mr_dir", R_MOT_DIR);
    n.getParam("/pacmouse/params/motor_pwm_freq", MOTOR_PWM_FREQUENCY);
    n.getParam("/pacmouse/params/ticks_per_radian", TICKS_PER_RADIAN);
}

void callback_1(int way)
{
  pos_1 += way;
}

void callback_2(int way)
{
  pos_2 -= way;
}

//These could overflow after 11 days of continous full speed running... 
int last_pos_1 = 0;
int last_pos_2 = 0;
int current_pos_1 = 0;
int current_pos_2 = 0;

// Calculates the instantaneous velocity given an update rate in hertz. (In number of ticks per second)
float calc_velocity_1(float hertz)
{
  last_pos_1 = current_pos_1;
  current_pos_1 = pos_1;
  return (current_pos_1 - last_pos_1) * hertz / TICKS_PER_RADIAN;

}

float calc_velocity_2(float hertz)
{
  last_pos_2 = current_pos_2;
  current_pos_2 = pos_2;
  return (current_pos_2 - last_pos_2) * hertz / TICKS_PER_RADIAN;
  
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
  // Load the initial parameters from file
  load_params(n);

// SHUTDOWN CODE
  // struct sigaction sigIntHandler;

  // sigIntHandler.sa_handler = shutdown;
  // sigemptyset(&sigIntHandler.sa_mask);
  // sigIntHandler.sa_flags = 0;

  // sigaction(SIGINT, &sigIntHandler, NULL);

// TODO: It seems that neither of these SIGINT handlers works. 

  signal(SIGINT, shutdown);

  ros::Subscriber sub = n.subscribe("/pacmouse/motor/cmd", 1000, motor_command_callback);

  ros::Publisher position_publisher = n.advertise<pacmouse_pkg::Drive>("/pacmouse/encoders/position", 1);
  ros::Publisher velocity_publisher = n.advertise<pacmouse_pkg::Drive>("/pacmouse/encoders/velocity", 1);

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

  decoder1 = new re_decoder(ENCODER_R_A, ENCODER_R_B, callback_1);
  decoder2 = new re_decoder(ENCODER_L_A, ENCODER_L_B, callback_2);

  ros::Rate loop_rate(LOOP_RATE);

  pacmouse_pkg::Drive pos_msg;
  pacmouse_pkg::Drive vel_msg;

  while (ros::ok())
  {

    pos_msg.R = pos_1 / TICKS_PER_RADIAN;
    pos_msg.L = pos_2 / TICKS_PER_RADIAN;
    
    vel_msg.R = calc_velocity_1(LOOP_RATE);
    vel_msg.L = calc_velocity_2(LOOP_RATE);

    position_publisher.publish(pos_msg);
    velocity_publisher.publish(vel_msg);

    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}
