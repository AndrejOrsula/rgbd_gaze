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
#include <sensor_msgs/msg/camera_info.hpp>
#include <eye_region_msgs/msg/eye_regions_of_interest_stamped.hpp>
#include <rgbd_gaze_msgs/msg/pupil_centres_stamped.hpp>

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

/// Index of the left eye
const uint8_t EYE_LEFT = 0;
/// Index of the right eye
const uint8_t EYE_RIGHT = 1;

/////////////
/// TYPES ///
/////////////

/// Policy of the synchronizer
typedef message_filters::sync_policies::ExactTime<sensor_msgs::msg::Image,
                                                  sensor_msgs::msg::Image,
                                                  sensor_msgs::msg::CameraInfo,
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
  /// Subscriber to the camera info
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> sub_camera_info_;
  /// Subscriber to the eye regions
  message_filters::Subscriber<eye_region_msgs::msg::EyeRegionsOfInterestStamped> sub_eyes_;
  /// Synchronizer of the subscribers
  message_filters::Synchronizer<synchronizer_policy> synchronizer_;

  /// Publisher of the pupil centres
  rclcpp::Publisher<rgbd_gaze_msgs::msg::PupilCentresStamped>::SharedPtr pub_pupil_centres_;

  /// Callback called each time a message is received on all topics
  void synchronized_callback(const sensor_msgs::msg::Image::SharedPtr msg_img_color,
                             const sensor_msgs::msg::Image::SharedPtr msg_img_depth,
                             const sensor_msgs::msg::CameraInfo::SharedPtr msg_camera_info,
                             const eye_region_msgs::msg::EyeRegionsOfInterestStamped::SharedPtr msg_eyes);
  /// Extract 3D position of a pixel from depth map
  bool depth_px_to_xyz(
      const cv::Mat &img_depth_rect,
      cv::Vec3d &output,
      const std::array<double, 9> &intrinsics,
      const uint16_t x,
      const uint16_t y,
      const float depth_scale = 0.001);
};

PupilCentre::PupilCentre() : Node(NODE_NAME),
                             sub_color_(this, "camera/color/image_raw", "raw"),
                             sub_depth_(this, "camera/aligned_depth_to_color/image_raw", "raw"),
                             sub_camera_info_(this, "camera/aligned_depth_to_color/camera_info"),
                             sub_eyes_(this, "eye_regions"),
                             synchronizer_(synchronizer_policy(SYNCHRONIZER_QUEUE_SIZE), sub_color_, sub_depth_, sub_camera_info_, sub_eyes_)
{
  // Synchronize the subscriptions under a single callback
  synchronizer_.registerCallback(&PupilCentre::synchronized_callback, this);

  // Register publisher of the pupil centres
  rclcpp::QoS qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
  pub_pupil_centres_ = this->create_publisher<rgbd_gaze_msgs::msg::PupilCentresStamped>("pupil_centres", qos);

  // Parameters of the element
  this->declare_parameter<bool>("visualise", true);

  RCLCPP_INFO(this->get_logger(), "Node initialised");
}

void PupilCentre::synchronized_callback(const sensor_msgs::msg::Image::SharedPtr msg_img_color,
                                        const sensor_msgs::msg::Image::SharedPtr msg_img_depth,
                                        const sensor_msgs::msg::CameraInfo::SharedPtr msg_camera_info,
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

  rgbd_gaze_msgs::msg::PupilCentresStamped pupil_centres;
  pupil_centres.header = msg_img_color->header;

  cv::Mat img_color_eyes[2];
  for (uint8_t eye = 0; eye < 2; eye++)
  {
    // Convert eye ROIs to rectangles
    cv::Rect roi_eyes = roi_to_rect(msg_eyes->eyes.rois[eye], ROI_TO_RECT_PADDING_HORZIZONTAL, ROI_TO_RECT_PADDING_VERTICAL);

    // Get the eye region images as cv::Mat, while being bounded to the ROI
    img_color_eyes[eye] = img_color->image(roi_eyes);

    // TODO: Implement pupil centre estimation

    cv::Point2d pupil_centre; // <--- output

    cv::Vec3d pupil;
    auto ret = depth_px_to_xyz(img_depth->image, pupil, msg_camera_info->k, msg_eyes->eyes.rois[eye].x_offset + pupil_centre.x, msg_eyes->eyes.rois[eye].y_offset + pupil_centre.y);

    // Place pupil centre into the message
    pupil_centres.pupils.centres[eye].x = pupil[0];
    pupil_centres.pupils.centres[eye].y = pupil[1];
    pupil_centres.pupils.centres[eye].z = pupil[2];

    // Publish the pupil centres
    pub_pupil_centres_->publish(pupil_centres);

    // Visualise if desired
    if (this->get_parameter("visualise").get_value<bool>())
    {
      std::string eye_side;
      if (eye == EYE_LEFT)
      {
        eye_side = "L";
      }
      else
      {
        eye_side = "R";
      }
      cv::namedWindow(eye_side + "_ROI", cv::WINDOW_KEEPRATIO);
      cv::imshow(eye_side + "_ROI", img_color_eyes[EYE_LEFT]);

      cv::waitKey(10);
    }
  }
}

bool PupilCentre::depth_px_to_xyz(
    const cv::Mat &img_depth_rect,
    cv::Vec3d &output,
    const std::array<double, 9> &intrinsics,
    const uint16_t x,
    const uint16_t y,
    const float depth_scale)
{
  if (x >= img_depth_rect.cols || y >= img_depth_rect.rows)
  {
    std::cerr << "depth_px_to_xyz() - Pixel out of bounds\n";
    return false;
  }

  uint16_t z = img_depth_rect.at<uint16_t>(x, y);

  if (z == 0)
  {
    return false;
  }
  else
  {
    output[2] = z * depth_scale;
    output[0] = output[2] * ((x - intrinsics[2]) / intrinsics[0]);
    output[1] = output[2] * ((y - intrinsics[5]) / intrinsics[4]);
    return true;
  }
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
