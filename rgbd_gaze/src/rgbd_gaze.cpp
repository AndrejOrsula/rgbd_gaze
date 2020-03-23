/// Gaze estimator based on 3D model of the eye

////////////////////
/// DEPENDENCIES ///
////////////////////

// ROS 2
#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

// ROS 2 interfaces
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rgbd_gaze_msgs/msg/pupil_centres_stamped.hpp>

//////////////////
/// NAMESPACES ///
//////////////////

using namespace std::placeholders;

/////////////////
/// CONSTANTS ///
/////////////////

/// The name of this node
const std::string NODE_NAME = "rgbd_gaze";
/// Size of the qeueu size used by the synchronizer in its policy
const uint8_t SYNCHRONIZER_QUEUE_SIZE = 10;

/////////////
/// TYPES ///
/////////////

/// Policy of the synchronizer
typedef message_filters::sync_policies::ExactTime<geometry_msgs::msg::PoseStamped,
                                                  rgbd_gaze_msgs::msg::PupilCentresStamped>
    synchronizer_policy;

////////////////////////
/// HELPER FUNCTIONS ///
////////////////////////

//////////////////
/// NODE CLASS ///
//////////////////

/// Class representation of this node
class RgbdGaze : public rclcpp::Node
{
public:
  /// Constructor
  RgbdGaze();

private:
  /// Subscriber to the head pose
  message_filters::Subscriber<geometry_msgs::msg::PoseStamped> sub_head_pose_;
  /// Subscriber to pupil centres
  message_filters::Subscriber<rgbd_gaze_msgs::msg::PupilCentresStamped> sub_pupil_centres_;

  /// Synchronizer of the subscribers
  message_filters::Synchronizer<synchronizer_policy> synchronizer_;

  /// Callback called each time a message is received on all topics
  void synchronized_callback(const geometry_msgs::msg::PoseStamped::SharedPtr head_pose,
                             const rgbd_gaze_msgs::msg::PupilCentresStamped::SharedPtr pupil_centres);
};

RgbdGaze::RgbdGaze() : Node(NODE_NAME),
                       sub_head_pose_(this, "head_pose"),
                       sub_pupil_centres_(this, "pupil_centres"),
                       synchronizer_(synchronizer_policy(SYNCHRONIZER_QUEUE_SIZE), sub_head_pose_, sub_pupil_centres_)
{
  // Synchronize the subscriptions under a single callback
  synchronizer_.registerCallback(&RgbdGaze::synchronized_callback, this);

  // Parameters of the element
  this->declare_parameter<bool>("visualise", true);

  RCLCPP_INFO(this->get_logger(), "Node initialised");
}

void RgbdGaze::synchronized_callback(const geometry_msgs::msg::PoseStamped::SharedPtr head_pose,
                                     const rgbd_gaze_msgs::msg::PupilCentresStamped::SharedPtr pupil_centres)
{
  RCLCPP_DEBUG(this->get_logger(), "Received synchronized messages for processing");
}

////////////
/// MAIN ///
////////////

/// Main function that initiates an object of `RgbdGaze` class as the core of this node.
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RgbdGaze>());
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
