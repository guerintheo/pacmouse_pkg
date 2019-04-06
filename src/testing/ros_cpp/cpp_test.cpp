/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// COMPILE WITH: 
// g++ -o rot_enc_cpp test_rotary_encoder.cpp rotary_encoder.cpp -lpigpio -lrt
// g++ -o cpp_test cpp_test.cpp rotary_encoder.cpp -lpigpio -lrt


// %Tag(FULLTEXT)%
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>

#include <pigpio.h>
#include <unistd.h>
#include <signal.h>

#include "rotary_encoder.hpp"

#define ENCODER_A 23
#define ENCODER_B 24

#define MOT_GPIO 12
#define PWM_DUTY 255

int pos_1 = 0;
int pos_2 = 0;

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
// %Tag(CALLBACK)%
void motor_command_callback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("Setting PWM value to: [%s]", msg->data.c_str());
  gpioPWM(MOT_GPIO, PWM_DUTY);

}
// %EndTag(CALLBACK)%

void callback_1(int way)
{

   pos_1 += way;

   // std::cout << "pos_1=" << pos_1 << std::endl;
}

void callback_2(int way)
{

   pos_2 += way;

   // std::cout << "pos_1=" << pos_1 << std::endl;
}

void shutdown(int s){
  // cleanup pigpio and callbacks
  dec.re_cancel();
  gpioTerminate();

  printf("Caught signal %d\n",s);
  exit(1); 

}

void publish_encoders(ros::Publisher encoder_publisher) {
// %Tag(FILL_MESSAGE)%
  std_msgs::String msg;

  std::stringstream ss;
  ss << "pos1: " << pos_1 << " pos2: " << pos_2;
  msg.data = ss.str();
// %EndTag(FILL_MESSAGE)%

// %Tag(ROSCONSOLE)%
  ROS_INFO("%s", msg.data.c_str());
// %EndTag(ROSCONSOLE)%

  /**
   * The publish() function is how you send messages. The parameter
   * is the message object. The type of this object must agree with the type
   * given as a template parameter to the advertise<>() call, as was done
   * in the constructor above.
   */
// %Tag(PUBLISH)%
  chatter_pub.publish(msg);
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
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

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */

// SETUP SHUTDOWN CODE
  struct sigaction sigIntHandler;

  sigIntHandler.sa_handler = shutdown;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;

  sigaction(SIGINT, &sigIntHandler, NULL);

// %Tag(SUBSCRIBER)%
  ros::Subscriber sub = n.subscribe("/pacmouse/motor/cmd", 1000, motor_command_callback);
// %EndTag(SUBSCRIBER)%

// %Tag(PUBLISHER)%
  ros::Publisher encoder_publisher = n.advertise<std_msgs::String>("/pacmouse/encoders", 1000);
// %EndTag(PUBLISHER)%

  if (gpioInitialise() < 0) return 1;

  re_decoder dec(ENCODER_A_1, ENCODER_B_1, callback_1);
  re_decoder dec(ENCODER_A_2, ENCODER_B_2, callback_2);

  // red_ecoder dec = red_decoder()

  



  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
// %Tag(SPIN)%
  // ros::spin();
// %EndTag(SPIN)%


// %Tag(LOOP_RATE)%
  ros::Rate loop_rate(10);
// %EndTag(LOOP_RATE)%

  while (ros::ok())
  {

    publish_encoders(encoder_publisher);
// %EndTag(PUBLISH)%

// %Tag(SPINONCE)%
    ros::spinOnce();
// %EndTag(SPINONCE)%

// %Tag(RATE_SLEEP)%
    loop_rate.sleep();
// %EndTag(RATE_SLEEP)%


  return 0;
}
// %EndTag(FULLTEXT)%