#include <ros/ros.h>
#include "core_msgs/line_segments.h"
#include "core_msgs/yolomsg.h"
#include <iostream>
#include <vector>
#include <string>
#include <math.h>
#include <std_msgs/Float64.h>
#include <chrono>
using namespace std;

ros::Publisher pub_right_front_wheel;
ros::Publisher pub_left_front_wheel;
ros::Publisher pub_left_rear_wheel;
ros::Publisher pub_right_rear_wheel;

std_msgs::Float64 right_front;
std_msgs::Float64 left_front;
std_msgs::Float64 left_rear;
std_msgs::Float64 right_rear;

int correct = 0;

bool isenabled = true;
bool stop = false;
bool start_zone = true;
double end_secs;
double begin_secs;
bool stop_done = 0;
bool aligned = false;

void decision_center(const core_msgs::line_segments::ConstPtr& msg) {
    if (!isenabled) return;
    if (stop) return;
    int count; //this is the number of actual data points received
    int array_size = msg->size;
    if (array_size == 0)
        return;

    count = array_size;
    for (int i =0;i < array_size;i++)
      {
          if (isnan(msg->com_x[i]))
          {
              count=i;
              cout << "count = " << count << "    " << msg->com_x[0] << endl;
              break;
          }
      }

     if (count>0){
          stop_done=0;
          double error = msg->com_x[0];
          right_front.data=50;
          left_front.data=50;
          left_rear.data=50;
          right_rear.data=50;

          if (abs(error)<55){
              correct = 0;
          }

          if (abs(error)>80 || correct==1){
              correct =1;
              if (error>0){
                  left_front.data = 50; //50
                  right_front.data = -25;//-48
                  left_rear.data = 20; //20
                  right_rear.data = 0; //-20
              }
              else{
                  right_front.data = 50;
                  left_front.data = -25;
                  right_rear.data = 20;//chaznge
                  left_rear.data = 0;
              }
            }


  /*
          else if (abs(error)>80 || correct == 2){
              correct =2;
              if (error>0){
                  right_front.data = -20;
                  left_rear.data=20;
                  right_rear.data = -20;
              }
              else{
                  left_front.data = -20;
                  right_rear.data = 20;
                  left_rear.data = -20;
              }
          }
  */

          pub_left_front_wheel.publish(left_front);
          pub_right_front_wheel.publish(right_front);
          pub_left_rear_wheel.publish(left_rear);
          pub_right_rear_wheel.publish(right_rear);

         //cout << left_front.data << "    " << right_front.data << "    " << right_rear.data
        //    << "    " << left_rear.data << endl;
         //cout <<"a " << array_size << endl;
            }
     }

void emergency_stop(const core_msgs::yolomsg::ConstPtr& msg) {
    if (!isenabled) return;
    int detected_num = msg->num;
    if (detected_num>1){
      stop = true;
      start_zone =false;
      begin_secs = ros::Time::now().toSec();
      end_secs = 0;
      if (stop_done==0){
        while(end_secs-begin_secs<0.165){
          right_front.data = -11;
          left_front.data = -11;
          right_rear.data = -11;
          left_rear.data = -11;
          pub_left_front_wheel.publish(left_front);
          pub_right_front_wheel.publish(right_front);
          pub_left_rear_wheel.publish(left_rear);
          pub_right_rear_wheel.publish(right_rear);
          end_secs = ros::Time::now().toSec();
          //cout << left_front.data << "    " << right_front.data << "    " << right_rear.data
          //    << "    " << left_rear.data << endl;
          }
          stop_done=1;
      }
      while(end_secs-begin_secs < 0.5){
        right_front.data = 0.1;
        left_front.data = 0.1;
        right_rear.data = 0.1;
        left_rear.data = 0.1;
        pub_left_front_wheel.publish(left_front);
        pub_right_front_wheel.publish(right_front);
        pub_left_rear_wheel.publish(left_rear);
        pub_right_rear_wheel.publish(right_rear);
        //cout << left_front.data << "    " << right_front.data << "    " << right_rear.data
        //    << "    " << left_rear.data << endl;
        end_secs = ros::Time::now().toSec();

        }
      }
    if (detected_num<2){
    stop = false;
    }
  }

int main (int argc, char **argv)
{
    ros::init (argc, argv, "line_detect_actuation_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/segments", 10, decision_center);
    ros::Subscriber sub2 = nh.subscribe("/detected_objects", 10, emergency_stop);
    pub_right_front_wheel = nh.advertise<std_msgs::Float64>("model20/right_front_wheel_velocity_controller/command", 10);
    pub_left_front_wheel = nh.advertise<std_msgs::Float64>("model20/left_front_wheel_velocity_controller/command", 10);
    pub_right_rear_wheel = nh.advertise<std_msgs::Float64>("model20/right_rear_wheel_velocity_controller/command", 10);
    pub_left_rear_wheel = nh.advertise<std_msgs::Float64>("model20/left_rear_wheel_velocity_controller/command", 10);

    ros::Rate loop_rate(20);

    while (ros::ok()) {
        ros::spinOnce();
        bool entrance_finished;
        if (isenabled && nh.getParam("/entrance_finished", entrance_finished)) {
            if (entrance_finished) {
                isenabled = false;
                cout << "end" <<endl;
                if (!aligned)
                {
                  double t1 = ros::Time::now().toSec();
                  double t2 =0;
                  while(t2-t1 < 0.8){
                    right_front.data = 30;
                    left_front.data = 30;
                    right_rear.data = 30;
                    left_rear.data = 30;
                    pub_left_front_wheel.publish(left_front);
                    pub_right_front_wheel.publish(right_front);
                    pub_left_rear_wheel.publish(left_rear);
                    pub_right_rear_wheel.publish(right_rear);
                    cout << left_front.data << "    " << right_front.data << "    " << right_rear.data
                        << "    " << left_rear.data << endl;
                    t2 = ros::Time::now().toSec();
                  }
                t1 = ros::Time::now().toSec();
                t2 =0;
                while(t2-t1 < 0.35){
                  right_front.data = 100;
                  left_front.data = -20;
                  right_rear.data = 70;
                  left_rear.data = -20;
                  pub_left_front_wheel.publish(left_front);
                  pub_right_front_wheel.publish(right_front);
                  pub_left_rear_wheel.publish(left_rear);
                  pub_right_rear_wheel.publish(right_rear);
                  cout << left_front.data << "    " << right_front.data << "    " << right_rear.data
                      << "    " << left_rear.data << endl;
                  t2 = ros::Time::now().toSec();
                }

                t1 = ros::Time::now().toSec();
                t2 =0;
                while(t2-t1 < 0.13){
                  right_front.data = 12;
                  left_front.data = 12;
                  right_rear.data = 12;
                  left_rear.data = 12;
                  pub_left_front_wheel.publish(left_front);
                  pub_right_front_wheel.publish(right_front);
                  pub_left_rear_wheel.publish(left_rear);
                  pub_right_rear_wheel.publish(right_rear);
                  cout << left_front.data << "    " << right_front.data << "    " << right_rear.data
                      << "    " << left_rear.data << endl;
                  t2 = ros::Time::now().toSec();
                }

                t1 = ros::Time::now().toSec();
                t2 =0;
                while(t2-t1 < 0.1){
                  right_front.data = -11;
                  left_front.data = -11;
                  right_rear.data = -11;
                  left_rear.data = -11;
                  pub_left_front_wheel.publish(left_front);
                  pub_right_front_wheel.publish(right_front);
                  pub_left_rear_wheel.publish(left_rear);
                  pub_right_rear_wheel.publish(right_rear);
                  cout << left_front.data << "    " << right_front.data << "    " << right_rear.data
                      << "    " << left_rear.data << endl;
                  t2 = ros::Time::now().toSec();
                }

                right_front.data = 0;
                left_front.data = 0;
                right_rear.data = 0;
                left_rear.data = 0;
                pub_left_front_wheel.publish(left_front);
                pub_right_front_wheel.publish(right_front);
                pub_left_rear_wheel.publish(left_rear);
                pub_right_rear_wheel.publish(right_rear);
                //cout << left_front.data << "    " << right_front.data << "    " << right_rear.data
                //    << "    " << left_rear.data << endl;
                aligned = true;
                }
            }
        }


        loop_rate.sleep();
    }
    return 0;
}
