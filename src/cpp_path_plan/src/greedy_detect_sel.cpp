#include "rclcpp/rclcpp.hpp"
#include "detections/msg/detection.hpp"
#include "detections/msg/detection_set.hpp"

#include <memory>
#include <math.h>

using std::placeholders::_1;


#define TENNIS_BALL_INDX  32

enum{
  LEFT = 0,
  TOP,
  RIGHT,
  BOTTOM
};




class GreedyDetectionPub : public rclcpp::Node
{
public:
  GreedyDetectionPub()
  : Node("Greedy_Detection_Publisher")
  {
    subscription_ = this->create_subscription<detections::msg::DetectionSet>(   
      "detections", 10, std::bind(&GreedyDetectionPub::topic_callback, this, _1));

    publisher_ = this->create_publisher<detections::msg::Detection>("Closest_Detection", 10);
    
  }

private:

  // Scores a ball on how likely the ball is the closest ball in frame
  float detect_score(const detections::msg::Detection & detect) const{
    float ret = 0;

    // Calculate the area (bigger area in camera = closer)
    ret += abs(detect.bounds[RIGHT] - detect.bounds[LEFT]) * abs(detect.bounds[BOTTOM] - detect.bounds[TOP]);

    ret += detect.bounds[BOTTOM];      // Balls that are closer to the bottom of frame get higher score

    ret += detect.conf;      // Add this as a tie breaker so we go towards balls we're more condident in

    return ret;
  }

  void copy_bounds(const float *bounds, float *copy_bounds) const{

    for(int z = 0; z < 4; z++){
      copy_bounds[z] = bounds[z];
      //RCLCPP_INFO_STREAM(this->get_logger(), "Bounds: " << z << " " << closest_detect.bounds[z]);     // CHANGE
    }

  }

  // Callback for when we receive a DetectionSet msg
  void topic_callback(const detections::msg::DetectionSet & msg) const  
  {
    auto closest_detect = detections::msg::Detection();
    auto detect = detections::msg::Detection();

    float closest_ball_score = 0;
    int ret_ind = 0;
    int first_ball_not_found = true;

    RCLCPP_INFO_STREAM(this->get_logger(), "Size: '" << msg.num_detections);     // CHANGE

    if(msg.num_detections > 0){
      
      // Iterate through the list until we find a tennis ball
      int starting_ind = 0;
      while(first_ball_not_found && starting_ind < msg.num_detections){
        if(msg.detections[starting_ind].index == TENNIS_BALL_INDX)
          first_ball_not_found = false;
        else
          starting_ind++;
      }

      if(starting_ind < msg.num_detections){

        closest_detect = msg.detections[starting_ind];
        copy_bounds(&msg.detections[starting_ind].bounds[starting_ind], &closest_detect.bounds[starting_ind]);

        closest_ball_score = detect_score(closest_detect);

        for(int i = starting_ind + 1; i < msg.num_detections; i++){
          float score_cmp = 0;
          
          if(msg.detections[starting_ind].index == TENNIS_BALL_INDX){

            detect = msg.detections[i];
            copy_bounds(&msg.detections[i].bounds[0], &detect.bounds[0]);
          
            score_cmp = detect_score(detect);

            if(score_cmp > closest_ball_score){
              copy_bounds(&detect.bounds[0], &closest_detect.bounds[0]);
              closest_ball_score = score_cmp;
              closest_detect = detect;
              ret_ind = i;
            }

          }

        }

        RCLCPP_INFO_STREAM(this->get_logger(), "Detection #: '" << ret_ind << ", is the closest detection");     // CHANGE

        publisher_->publish(closest_detect);

      }// If starting_ind < num_detections

    }
    
  }
  rclcpp::Subscription<detections::msg::DetectionSet>::SharedPtr subscription_;  // CHANGE
  rclcpp::Publisher<detections::msg::Detection>::SharedPtr publisher_;  // CHANGE
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GreedyDetectionPub>());
  rclcpp::shutdown();
  return 0;
}