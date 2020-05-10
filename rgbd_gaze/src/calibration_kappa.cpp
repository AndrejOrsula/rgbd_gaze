/// Personal calibration of kappa angle for use with RGB-D gaze estimation

////////////////////
/// DEPENDENCIES ///
////////////////////

// ROS 2
#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>

// ROS 2 interfaces
#include <geometry_msgs/msg/point_stamped.hpp>
#include <gaze_msgs/msg/gaze_binocular_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// Eigen
#include <Eigen/Geometry>

// PCL
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

/////////////////
/// CONSTANTS ///
/////////////////

/// The name of this node
const std::string NODE_NAME = "rgbd_gaze_calibration_kappa";
/// Size of the queue size used by the synchronizer in its policy
const uint8_t SYNCHRONIZER_QUEUE_SIZE = 10;

/// Index of the left eye
const uint8_t EYE_LEFT = 0;
/// Index of the right eye
const uint8_t EYE_RIGHT = 1;

const float VISUAL_OPTICAL_AXIS_LENGTH = 5.0;
const float VISUAL_OPTICAL_AXIS_WIDTH = 0.001;
const float VISUAL_OPTICAL_AXIS_COLOR[] = {1.0, 0, 0, 1};

const float VISUAL_REAL_GAZE_WIDTH = 0.0015;
const float VISUAL_REAL_GAZE_COLOR[] = {0, 1.0, 0, 1};

/////////////
/// TYPES ///
/////////////

/// Policy of the synchronizer
typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::msg::PoseStamped,
                                                        gaze_msgs::msg::GazeBinocularStamped,
                                                        geometry_msgs::msg::PointStamped>
    synchronizer_policy;

////////////////////////
/// HELPER FUNCTIONS ///
////////////////////////

namespace Eigen
{
  void fromMsg(const geometry_msgs::msg::Vector3 &input, Eigen::Vector3d &output)
  {
    output.x() = input.x;
    output.y() = input.y;
    output.z() = input.z;
  }
} // namespace Eigen

namespace tf2
{
  Eigen::Vector3d transform(Eigen::Vector3d &input, const Eigen::Isometry3d &transformation)
  {
    return transformation * input;
  }

  Eigen::ParametrizedLine<double, 3> transform(Eigen::ParametrizedLine<double, 3> &parametrized_line, const Eigen::Isometry3d &transformation)
  {
    return Eigen::ParametrizedLine<double, 3>(transformation * parametrized_line.origin(), transformation.rotation() * parametrized_line.direction());
  }
} // namespace tf2

double compute_angle(double x1, double y1, double x2, double y2)
{
  double norm1 = std::sqrt(x1 * x1 + y1 * y1);
  double norm2 = std::sqrt(x2 * x2 + y2 * y2);
  x1 /= norm1;
  y1 /= norm1;
  x2 /= norm2;
  y2 /= norm2;

  return std::atan2(x1 * y2 - y1 * x2, x1 * x2 + y1 * y2);
  // return std::acos((x1 * y1 + x2 * y2) / (std::sqrt(x1 * x1 + x2 * x2) * std::sqrt(y1 * y1 + y2 * y2)));
}

//////////////////
/// NODE CLASS ///
//////////////////

/// Class representation of this node
class RgbdGazeCalibrationKappa : public rclcpp::Node
{
public:
  /// Constructor
  RgbdGazeCalibrationKappa();

private:
  /// Subscriber to the optical axis
  message_filters::Subscriber<geometry_msgs::msg::PoseStamped> sub_head_pose_;
  /// Subscriber to the optical axis
  message_filters::Subscriber<gaze_msgs::msg::GazeBinocularStamped>
      sub_optical_axes_;
  /// Subscriber to the head pose
  message_filters::Subscriber<geometry_msgs::msg::PointStamped> sub_scene_point_;
  /// Synchronizer of the subscribers
  message_filters::Synchronizer<synchronizer_policy> synchronizer_;

  /// Publisher of visualisation markers
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;

  /// Buffer for tf2 transforms
  tf2_ros::Buffer tf2_buffer_;
  /// Listener of tf2 transforms
  tf2_ros::TransformListener tf2_listener_;

  // /// Vector of observed kappa angle pairs <alpha, beta>
  // std::vector<std::pair<double, double>> kappa_observations_[2];

  /// Kappa angle observations represented as point cloud (used for outlier removal)
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_kappa_observations_[2];

  uint64_t epoch_;

  /// Callback called each time a message is received on all topics
  void synchronized_callback(
      const geometry_msgs::msg::PoseStamped::SharedPtr msg_head_pose,
      const gaze_msgs::msg::GazeBinocularStamped::SharedPtr msg_optical_axes,
      const geometry_msgs::msg::PointStamped::SharedPtr msg_scene_point);
};

RgbdGazeCalibrationKappa::RgbdGazeCalibrationKappa() : Node(NODE_NAME),
                                                       sub_head_pose_(this, "head_pose"),
                                                       sub_optical_axes_(this, "optical_axes"),
                                                       sub_scene_point_(this, "scene_point"),
                                                       synchronizer_(synchronizer_policy(SYNCHRONIZER_QUEUE_SIZE), sub_head_pose_, sub_optical_axes_, sub_scene_point_),
                                                       tf2_buffer_(this->get_clock()),
                                                       tf2_listener_(tf2_buffer_),
                                                       epoch_(0)
{
  // Synchronize the subscriptions under a single callback
  synchronizer_.registerCallback(&RgbdGazeCalibrationKappa::synchronized_callback, this);

  // Parameters of the element
  this->declare_parameter<bool>("publish_markers", true);

  // Register publisher of the visualisation markers
  if (this->get_parameter("publish_markers").get_value<bool>())
  {
    rclcpp::QoS qos_markers = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualisation_markers", qos_markers);
  }

  // Initialise point cloud
  for (uint8_t eye = 0; eye < 2; eye++)
  {
    pc_kappa_observations_[eye].reset(new pcl::PointCloud<pcl::PointXYZ>);
    pc_kappa_observations_[eye]->is_dense = true;
    pc_kappa_observations_[eye]->height = 1;
    pc_kappa_observations_[eye]->width = 0;
  }

  RCLCPP_INFO(this->get_logger(), "Node initialised");
}

void RgbdGazeCalibrationKappa::synchronized_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg_head_pose,
                                                     const gaze_msgs::msg::GazeBinocularStamped::SharedPtr msg_optical_axes,
                                                     const geometry_msgs::msg::PointStamped::SharedPtr msg_scene_point)
{
  RCLCPP_DEBUG(this->get_logger(), "Received synchronized messages for processing");
  epoch_++;

  // Convert head pose to Eigen
  Eigen::Isometry3d head_pose;
  tf2::fromMsg(msg_head_pose->pose, head_pose);

  // Convert scene point to Eigen
  Eigen::Vector3d scene_point;
  Eigen::fromMsg(msg_scene_point->point, scene_point);

  // Create a parametrized lines from optical axes
  Eigen::ParametrizedLine<double, 3> optical_axes[2];
  for (uint8_t eye = 0; eye < 2; eye++)
  {
    Eigen::Vector3d origin, direction;
    Eigen::fromMsg(msg_optical_axes->gaze[eye].origin, origin);
    Eigen::fromMsg(msg_optical_axes->gaze[eye].direction, direction);
    optical_axes[eye] = Eigen::ParametrizedLine<double, 3>(origin, direction);
  }

  // Convert scene point into coordinate system of the head pose
  if (msg_scene_point->header.frame_id != msg_head_pose->header.frame_id)
  {
    // Attemp to find transformation between the coordinate frames
    geometry_msgs::msg::TransformStamped transform_stamped;
    try
    {
      transform_stamped = tf2_buffer_.lookupTransform(msg_scene_point->header.frame_id, msg_head_pose->header.frame_id, rclcpp::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
      return;
    }

    // Transform scene point into coordinate system of optical axes
    scene_point = tf2::transform(scene_point, tf2::transformToEigen(transform_stamped.transform));
  }

  // Convert optical axes into coordinate system of the head pose, if needed
  if (msg_optical_axes->header.frame_id != msg_head_pose->header.frame_id)
  {
    // Attemp to find transformation between the coordinate frames
    geometry_msgs::msg::TransformStamped transform_stamped;
    try
    {
      transform_stamped = tf2_buffer_.lookupTransform(msg_optical_axes->header.frame_id, msg_head_pose->header.frame_id, rclcpp::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
      return;
    }

    // Transform optical axes into coordinate system of optical axes
    for (uint8_t eye = 0; eye < 2; eye++)
    {
      optical_axes[eye] = tf2::transform(optical_axes[eye], tf2::transformToEigen(transform_stamped.transform));
    }
  }

  // Compute kappa
  Eigen::ParametrizedLine<double, 3> real_gaze[2];
  for (uint8_t eye = 0; eye < 2; eye++)
  {
    // Construct the real gaze based on the scene point
    real_gaze[eye] = Eigen::ParametrizedLine<double, 3>(optical_axes[eye].origin(), (scene_point - optical_axes[eye].origin()).normalized());

    // Transform optical axes and real gaze of both eyes to head coordinate frame (kappa is fixed with this frame)
    Eigen::Vector3d optical_axes_direction_wrt_head = head_pose.inverse().rotation() * optical_axes[eye].direction();
    Eigen::Vector3d real_gaze_direction_wrt_head = head_pose.inverse().rotation() * real_gaze[eye].direction();

    // Compute alpha (around x)
    double alpha = compute_angle(optical_axes_direction_wrt_head.y(), optical_axes_direction_wrt_head.z(),
                                 real_gaze_direction_wrt_head.y(), real_gaze_direction_wrt_head.z());

    // Compute beta (around y)
    double beta = compute_angle(optical_axes_direction_wrt_head.z(), optical_axes_direction_wrt_head.x(),
                                real_gaze_direction_wrt_head.z(), real_gaze_direction_wrt_head.x());

    // Combine kappa angles and store
    // kappa_observations_[eye].push_back(std::pair(alpha, beta));
    pc_kappa_observations_[eye]->push_back(pcl::PointXYZ(alpha, beta, 0));
  }

  // Find the mean kappa for each eye
  struct
  {
    double alpha;
    double beta;
  } kappa_calibrated[2];
  for (uint8_t eye = 0; eye < 2; eye++)
  {
    // Remove outliers
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statistical_outlier_removal;
    statistical_outlier_removal.setInputCloud(pc_kappa_observations_[eye]);
    statistical_outlier_removal.setMeanK(3);
    statistical_outlier_removal.setStddevMulThresh(1);
    std::vector<int> inliers;
    statistical_outlier_removal.filter(inliers);

    // Find the mean of the inlier kappa observations
    kappa_calibrated[eye].alpha = 0.0;
    kappa_calibrated[eye].beta = 0.0;
    for (auto &inlier : inliers)
    {
      kappa_calibrated[eye].alpha += pc_kappa_observations_[eye]->points[inlier].x;
      kappa_calibrated[eye].beta += pc_kappa_observations_[eye]->points[inlier].y;
    }
    kappa_calibrated[eye].alpha /= inliers.size();
    kappa_calibrated[eye].beta /= inliers.size();
  }

  // Print results
  {
    std::string output_string;
    output_string = "Results of epoch #" + std::to_string(epoch_);
    for (uint8_t eye = 0; eye < 2; eye++)
    {
      std::string eye_side;
      if (eye == EYE_LEFT)
      {
        eye_side = "left";
      }
      else
      {
        eye_side = "right";
      }

      output_string += "\n" + eye_side +
                       ":\n\tkappa:" +
                       "\n\t\talpha: " + std::to_string(kappa_calibrated[eye].alpha) +
                       "\n\t\tbeta: " + std::to_string(kappa_calibrated[eye].beta);
    }
    RCLCPP_INFO(this->get_logger(), output_string);
  }

  // Publish eyeball as a visualisation marker, if desired
  if (this->get_parameter("publish_markers").get_value<bool>())
  {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker default_marker;
    default_marker.header = msg_head_pose->header;
    default_marker.action = visualization_msgs::msg::Marker::ADD;
    for (uint8_t eye = 0; eye < 2; eye++)
    {
      std::string eye_side;
      if (eye == EYE_LEFT)
      {
        eye_side = "_left";
      }
      else
      {
        eye_side = "_right";
      }
      default_marker.ns = std::string(this->get_namespace()) + "kappa_calibration" + eye_side;

      // Optical axis
      {
        visualization_msgs::msg::Marker optical_axis_marker = default_marker;
        optical_axis_marker.id = 0;
        optical_axis_marker.type = visualization_msgs::msg::Marker::ARROW;
        geometry_msgs::msg::Point start, end;
        start = Eigen::toMsg(optical_axes[eye].origin());
        end = Eigen::toMsg(optical_axes[eye].pointAt(VISUAL_OPTICAL_AXIS_LENGTH));
        optical_axis_marker.points.push_back(start);
        optical_axis_marker.points.push_back(end);
        optical_axis_marker.scale.x = VISUAL_OPTICAL_AXIS_WIDTH;
        optical_axis_marker.scale.y =
            optical_axis_marker.scale.z = 0;
        optical_axis_marker.color.r = VISUAL_OPTICAL_AXIS_COLOR[0];
        optical_axis_marker.color.g = VISUAL_OPTICAL_AXIS_COLOR[1];
        optical_axis_marker.color.b = VISUAL_OPTICAL_AXIS_COLOR[2];
        optical_axis_marker.color.a = VISUAL_OPTICAL_AXIS_COLOR[3];
        markers.markers.push_back(optical_axis_marker);
      }

      // Real gaze
      {
        visualization_msgs::msg::Marker real_gaze_marker = default_marker;
        real_gaze_marker.id = 1;
        real_gaze_marker.type = visualization_msgs::msg::Marker::ARROW;
        geometry_msgs::msg::Point start, end;
        start = Eigen::toMsg(real_gaze[eye].origin());
        end = Eigen::toMsg(scene_point);
        real_gaze_marker.points.push_back(start);
        real_gaze_marker.points.push_back(end);
        real_gaze_marker.scale.x =
            real_gaze_marker.scale.y = VISUAL_REAL_GAZE_WIDTH;
        real_gaze_marker.scale.z = 0.0;
        real_gaze_marker.color.r = VISUAL_REAL_GAZE_COLOR[0];
        real_gaze_marker.color.g = VISUAL_REAL_GAZE_COLOR[1];
        real_gaze_marker.color.b = VISUAL_REAL_GAZE_COLOR[2];
        real_gaze_marker.color.a = VISUAL_REAL_GAZE_COLOR[3];
        markers.markers.push_back(real_gaze_marker);
      }
    }
    pub_markers_->publish(markers);
  }
}

////////////
/// MAIN ///
////////////

/// Main function that initiates an object of `RgbdGazeCalibrationKappa` class as the core of this node.
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RgbdGazeCalibrationKappa>());
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
