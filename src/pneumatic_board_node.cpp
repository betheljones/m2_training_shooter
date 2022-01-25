#include "ros/ros.h"
#include "std_msgs/Bool.h"

#include <sstream>
#include <string>
#include <iostream>
#include <vector>
#include <boost/bind.hpp>
#include <gpiod.h>
struct gpiod_chip *chip;
const char *chipname = "gpiochip0";
struct gpiod_line *line[8];

void callback(const std_msgs::Bool::ConstPtr& msg, int i){
  ROS_INFO("setting output %d to %d", i, msg -> data);
  gpiod_line_set_value(line[i],  msg -> data);
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  chip = gpiod_chip_open_by_name(chipname);
  line[0] =  gpiod_chip_get_line(chip, 25);
  gpiod_line_request_output(line[0], "pneumetic_board", 0);
  line[1] =  gpiod_chip_get_line(chip, 18);
  gpiod_line_request_output(line[1], "pneumetic_board", 0);
  line[2] =  gpiod_chip_get_line(chip, 24);
  gpiod_line_request_output(line[2], "pneumetic_board", 0);
  line[3] =  gpiod_chip_get_line(chip, 22);
  gpiod_line_request_output(line[3], "pneumetic_board", 0);
  line[4] =  gpiod_chip_get_line(chip, 23);
  gpiod_line_request_output(line[4], "pneumetic_board", 0);
  line[5] =  gpiod_chip_get_line(chip, 27);
  gpiod_line_request_output(line[5], "pneumetic_board", 0);
  line[6] =  gpiod_chip_get_line(chip, 26);
  gpiod_line_request_output(line[6], "pneumetic_board", 0);
  line[7] =  gpiod_chip_get_line(chip, 17);
  gpiod_line_request_output(line[7], "pneumetic_board", 0);

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
  ros::init(argc, argv, "m2_training_shooter_node");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  std::vector<ros::Subscriber> subscriber;
  for(int i=0; i<8; i++){
    std::stringstream topic_name;
    topic_name << "output/" << i;
    ros::Subscriber sub = n.subscribe<std_msgs::Bool>(topic_name.str(), 10, boost::bind(callback, _1, i));
    subscriber.push_back(sub);
    ROS_INFO("creating topic %s", topic_name.str().c_str());
  }

  ros::spin();


  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */

  return 0;
}