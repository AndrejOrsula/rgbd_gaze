/// Estimator of the pupil centre position in 3D, using color and depth images

////////////////////
/// DEPENDENCIES ///
////////////////////

// ROS 2
#include <rclcpp/rclcpp.hpp>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <cv_bridge/cv_bridge.h>

// ROS 2 interfaces
#include <sensor_msgs/msg/image.hpp>
#include <eye_region_msgs/msg/eye_regions_of_interest_stamped.hpp>

// OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//////////////////
/// NAMESPACES ///
//////////////////

using namespace std::placeholders;

/////////////////
/// CONSTANTS ///
/////////////////

/// The name of this node
const std::string NODE_NAME = "pupil_centre";
/// Size of the qeueu size used by the synchronizer in its policy
const uint8_t SYNCHRONIZER_QUEUE_SIZE = 10;
/// Extra infration of the eye region in horizontal direction, in pixels
const uint8_t ROI_TO_RECT_PADDING_HORZIZONTAL = 5;
/// Extra infration of the eye region in vertical direction, in pixels
const uint8_t ROI_TO_RECT_PADDING_VERTICAL = 5;

/////////////
/// TYPES ///
/////////////

/// Policy of the synchronizer
typedef message_filters::sync_policies::ExactTime<sensor_msgs::msg::Image,
                                                  sensor_msgs::msg::Image,
                                                  eye_region_msgs::msg::EyeRegionsOfInterestStamped>
    synchronizer_policy;

////////////////////////
/// HELPER FUNCTIONS ///
////////////////////////

inline cv::Rect roi_to_rect(const sensor_msgs::msg::RegionOfInterest &roi, const uint8_t padding_horizontal = 0, const uint8_t padding_vertical = 0)
{
  return cv::Rect(roi.x_offset - padding_horizontal, roi.y_offset - padding_vertical, roi.width + 2 * padding_horizontal, roi.height + 2 * padding_vertical);
}

//////////////////
/// NODE CLASS ///
//////////////////

/// Class representation of this node
class PupilCentre : public rclcpp::Node
{
public:
  /// Constructor
  PupilCentre();

private:
  /// Subscriber to color frames
  image_transport::SubscriberFilter sub_color_;
  /// Subscriber to registered (aligned) depth frames
  image_transport::SubscriberFilter sub_depth_;
  /// Subscriber to the eye regions
  message_filters::Subscriber<eye_region_msgs::msg::EyeRegionsOfInterestStamped> sub_eyes_;

  /// Synchronizer of the subscribers
  message_filters::Synchronizer<synchronizer_policy> synchronizer_;

  /// Callback called each time a message is received on all topics
  void synchronized_callback(const sensor_msgs::msg::Image::SharedPtr msg_img_color,
                             const sensor_msgs::msg::Image::SharedPtr msg_img_depth,
                             const eye_region_msgs::msg::EyeRegionsOfInterestStamped::SharedPtr msg_eyes);
};

PupilCentre::PupilCentre() : Node(NODE_NAME),
                             sub_color_(this, "camera/color/image_raw", "raw"),
                             sub_depth_(this, "camera/aligned_depth_to_color/image_raw", "raw"),
                             sub_eyes_(this, "eye_regions"),
                             synchronizer_(synchronizer_policy(SYNCHRONIZER_QUEUE_SIZE), sub_color_, sub_depth_, sub_eyes_)
{
  // Synchronize the subscriptions under a single callback
  synchronizer_.registerCallback(&PupilCentre::synchronized_callback, this);

  // Parameters of the element
  this->declare_parameter<bool>("visualise", true);

  RCLCPP_INFO(this->get_logger(), "Node initialised");
}

void PupilCentre::synchronized_callback(const sensor_msgs::msg::Image::SharedPtr msg_img_color,
                                        const sensor_msgs::msg::Image::SharedPtr msg_img_depth,
                                        const eye_region_msgs::msg::EyeRegionsOfInterestStamped::SharedPtr msg_eyes)
{
  RCLCPP_DEBUG(this->get_logger(), "Received synchronized messages for processing");

  // Use cv_bridge to convert sensor_msgs::msg::Image to cv::Mat
  cv_bridge::CvImagePtr img_color, img_depth;
  try
  {
    img_color = cv_bridge::toCvCopy(msg_img_color, sensor_msgs::image_encodings::BGR8);
    img_depth = cv_bridge::toCvCopy(msg_img_depth, sensor_msgs::image_encodings::TYPE_16UC1);
  }
  catch (const cv_bridge::Exception exception)
  {
    RCLCPP_ERROR(this->get_logger(), "Invalid frame. Exception from cv_bridge: %s", exception.what());
    return;
  }

  // Convert eye ROIs to rectangles
  cv::Rect roi_eye_left = roi_to_rect(msg_eyes->eyes.eye_left, ROI_TO_RECT_PADDING_HORZIZONTAL, ROI_TO_RECT_PADDING_VERTICAL);
  cv::Rect roi_eye_right = roi_to_rect(msg_eyes->eyes.eye_right, ROI_TO_RECT_PADDING_HORZIZONTAL, ROI_TO_RECT_PADDING_VERTICAL);

  // Get the eye region images as cv::Mat, while being bounded to the ROI
  cv::Mat img_color_eye_left = img_color->image(roi_eye_left);
  cv::Mat img_color_eye_right = img_color->image(roi_eye_right);

  // Visualise if desired
  if (this->get_parameter("visualise").get_value<bool>())
  {
    cv::namedWindow("img_color_eye_left", cv::WINDOW_KEEPRATIO);
    cv::namedWindow("img_color_eye_right", cv::WINDOW_KEEPRATIO);
    cv::imshow("img_color_eye_left", img_color_eye_left);
    cv::imshow("img_color_eye_right", img_color_eye_right);
    cv::waitKey(1);
  }

  // TODO: Implement pupil centre estimation
}

////////////
/// MAIN ///
////////////

/// Main function that initiates an object of `PupilCentre` class as the core of this node.
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PupilCentre>());
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
