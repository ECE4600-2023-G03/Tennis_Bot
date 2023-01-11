#include "rclcpp/rclcpp.hpp"
#include "detections/msg/detection.hpp"
#include "detections/msg/detection_set.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <cmath>
#include <chrono>
#include <functional>
#include <memory>
#include <string>


#define CONTROL_LOOP_EXEC_FREQ      100               // How many times a second the control loop runs
#define CONTROL_LOOP_EXEC_PERIOD    10             
#define TIMEOUT_PERIOD              1500            

#define PIXEL_COUNT_X               4032              // Verify this
#define PIXEL_COUNT_Y               4032              // Verify this

#define CENTER_FRAME                PIXEL_COUNT_X/2

#define LINEAR_SPEED                110                // Don't know what a good speed is
#define SPIN_SPEED                  105               // Constant Speed of the robot while it's spinning

// PID 
#define P_WEIGHT      0.3
#define I_WEIGHT      0.2


using std::placeholders::_1;
using namespace std::chrono_literals;

enum{
  LEFT = 0,
  TOP,
  RIGHT,
  BOTTOM
};


class ControlLoopPub : public rclcpp::Node
{
public:
  ControlLoopPub()
  : Node("Control_Loop_Publisher")
  {

    subscription_ = this->create_subscription<detections::msg::Detection>(   
      "Closest_Detection", 10, std::bind(&ControlLoopPub::new_detection_callback, this, _1));

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("twist", 10);                // Publish messages to ROS

    turn = true;
    valid_detection = false;

    control_timer_ = this->create_wall_timer(
        10ms, std::bind(&ControlLoopPub::control_callback, this));   // Timer for executing the loop at a fixed rate

  }

private:

  // Callback for when we receive a DetectionSet msg
  void new_detection_callback(const detections::msg::Detection & msg)  
  {
    closest_detect = msg;    
    timeout_cnt = 0;
    valid_detection = true;            // Detection is valid
  }


  // Callback for the control loop of the robot
  void control_callback()   
  {
    float avg_xbounds = 0;        // location of the ball on the x coordinate of our frame, avg between left and right bounds
    double p_error;
    double total_error = 0;

    if(timeout_cnt > 100){
      valid_detection = false;
      turn = true;
      timeout_cnt = 0;
    }
    if(turn_cnt > 100){
      turn = true;
      turn_cnt = 0;
    }

    if(valid_detection == true){

        avg_xbounds = (closest_detect.bounds[LEFT]*PIXEL_COUNT_X + closest_detect.bounds[RIGHT]*PIXEL_COUNT_X)/2;
    
        RCLCPP_INFO_STREAM(this->get_logger(), "Left bound: " << closest_detect.bounds[LEFT] << " Right bound: " << closest_detect.bounds[RIGHT]);     // CHANGE

        p_error = CENTER_FRAME - avg_xbounds;     // error is the difference between the center of frame and the ball
        
        RCLCPP_INFO_STREAM(this->get_logger(), "Pixels to center frame: " << p_error);     // CHANGE

        //replace the current value in ring buffer from the running average (we are calculating the average of i_error_arr in O(1))
        i_error = (i_error*CONTROL_LOOP_EXEC_FREQ - i_error_arr[i_err_ind++] + p_error) / CONTROL_LOOP_EXEC_FREQ;

        // Ring buffer logic
        if(i_err_ind > CONTROL_LOOP_EXEC_FREQ-1){
            i_err_ind = 0;
        }

        // Are we still facing the ball
        if(p_error < 500 && p_error > -500){
          turn_cnt = 0;
        }

        //successfuly turned towards ball
        if(p_error < 300 && p_error > -300){
          turn = false;        // Reset the turn timer if we are on track
          turn_cnt = 0;
        }

        // If we need to turn (1s )
        if(turn == true){
          total_error = abs(P_WEIGHT*p_error + I_WEIGHT*i_error);   // Combine the P and I errors
          if(total_error > 2000)
            total_error = 2000;

          if(p_error > 1){
            twist.angular.z = -total_error/4032*127 + 128;                 // PID control of angular speed
          }
          if(p_error < -1){
            twist.angular.z = total_error/4032*127 + 128;      // PID control of angular speed
          }
          
          twist.linear.x = 0;      // Set the linear speed of the robot to a constant rate
        }
          
        else{
          i_error = 0;
          // Reset Ring buffer
          for(int i = 0; i < CONTROL_LOOP_EXEC_FREQ; i++)
              i_error_arr[i] = 0;
          twist.angular.z = 0;                // PID control of angular speed
          twist.linear.x = LINEAR_SPEED;      // Set the linear speed of the robot to a constant rate
        }

    }
    else{
        twist.angular.z = SPIN_SPEED;       // Spin at a constant rate looking for balls
        twist.linear.x = 0;                 // Stop
        i_error = 0;
        // Reset Ring buffer
        for(int i = 0; i < CONTROL_LOOP_EXEC_FREQ; i++)
            i_error_arr[i] = 0;
    }
  
    RCLCPP_INFO_STREAM(this->get_logger(), "Twist: Angular- " << twist.angular.z << " Linear- " << twist.linear.x << " Turning: " << turn << " Turn cnt: " << turn_cnt << " Valid: " << valid_detection << " Total Error: " << total_error);     // CHANGE

    publisher_->publish(twist);

    timeout_cnt += 1;
    turn_cnt += 1;

  }

  int i_err_ind = 0;
  double i_error_arr[CONTROL_LOOP_EXEC_FREQ] = {0};        // location of the ball on the x coordinate of our frame, avg between left and right bounds
  double i_error = 0;        // location of the ball on the x coordinate of our frame, avg between left and right bounds
  int turn_cnt = 0;
  int timeout_cnt = 0;

  detections::msg::Detection closest_detect = detections::msg::Detection();
  int valid_detection;                          // Whether the most recent detection is still valid
  int turn;                                  // Whether or not to turn the bot

  geometry_msgs::msg::Twist twist;

  rclcpp::TimerBase::SharedPtr control_timer_;

  rclcpp::Subscription<detections::msg::Detection>::SharedPtr subscription_;  
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;  
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlLoopPub>());
  rclcpp::shutdown();
  return 0;
}