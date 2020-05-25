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
#include <std_srvs/srv/set_bool.hpp>

// Eigen
#include <Eigen/Geometry>

// PCL
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

// File IO
#include <fstream>

/////////////////
/// CONSTANTS ///
/////////////////

/// The name of this node
const std::string NODE_NAME = "rgbd_gaze_accuracy";
/// Size of the queue size used by the synchronizer in its policy
const uint8_t SYNCHRONIZER_QUEUE_SIZE = 50;

/// Index of the left eye
const uint8_t EYE_LEFT = 0;
/// Index of the right eye
const uint8_t EYE_RIGHT = 1;

const float VISUAL_VISUAL_AXIS_LENGTH = 5.0;
const float VISUAL_VISUAL_AXIS_WIDTH = 0.001;
const float VISUAL_VISUAL_AXIS_COLOR[] = {1.0, 0, 0, 1};

const float VISUAL_REAL_GAZE_WIDTH = 0.0015;
const float VISUAL_REAL_GAZE_COLOR[] = {0, 1.0, 0, 1};

// If false, use radians for angular measurements
const bool DEG = true;

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
}

//////////////////
/// NODE CLASS ///
//////////////////

/// Class representation of this node
class RgbdGazeAccuracy : public rclcpp::Node
{
public:
  /// Constructor
  RgbdGazeAccuracy();
  /// Destructor
  ~RgbdGazeAccuracy();

private:
  /// Subscriber to the head pose
  message_filters::Subscriber<geometry_msgs::msg::PoseStamped> sub_head_pose_;
  /// Subscriber to the visual axis
  message_filters::Subscriber<gaze_msgs::msg::GazeBinocularStamped> sub_visual_axes_;
  /// Subscriber to the head pose
  message_filters::Subscriber<geometry_msgs::msg::PointStamped> sub_scene_point_;
  /// Synchronizer of the subscribers
  message_filters::Synchronizer<synchronizer_policy> synchronizer_;

  /// Service for enabling data collection
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_data_collection_service_;
  /// Flag that determines whether to collect data on the next synchronized callback
  bool enable_data_collection_;
  /// File stream for collecting data
  std::ofstream data_file_;

  /// Publisher of visualisation markers
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;

  /// Buffer for tf2 transforms
  tf2_ros::Buffer tf2_buffer_;
  /// Listener of tf2 transforms
  tf2_ros::TransformListener tf2_listener_;

  uint64_t epoch_;

  void handle_enable_data_collection_service(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      const std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  /// Callback called each time a message is received on all topics
  void synchronized_callback(
      const geometry_msgs::msg::PoseStamped::SharedPtr msg_head_pose,
      const gaze_msgs::msg::GazeBinocularStamped::SharedPtr msg_visual_axes,
      const geometry_msgs::msg::PointStamped::SharedPtr msg_scene_point);
};

RgbdGazeAccuracy::RgbdGazeAccuracy() : Node(NODE_NAME),
                                       sub_head_pose_(this, "head_pose"),
                                       sub_visual_axes_(this, "visual_axes"),
                                       sub_scene_point_(this, "scene_point"),
                                       synchronizer_(synchronizer_policy(SYNCHRONIZER_QUEUE_SIZE), sub_head_pose_, sub_visual_axes_, sub_scene_point_),
                                       tf2_buffer_(this->get_clock()),
                                       tf2_listener_(tf2_buffer_),
                                       enable_data_collection_(true),
                                       epoch_(0)
{
  // Synchronize the subscriptions under a single callback
  synchronizer_.registerCallback(&RgbdGazeAccuracy::synchronized_callback, this);

  // Parameters of the element
  this->declare_parameter<bool>("publish_markers", true);

  // Register publisher of the visualisation markers
  if (this->get_parameter("publish_markers").get_value<bool>())
  {
    rclcpp::QoS qos_markers = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualisation_markers", qos_markers);
  }

  // Register a service that begins data collection
  enable_data_collection_service_ = this->create_service<std_srvs::srv::SetBool>("enable_data_collection",
                                                                                 std::bind(&RgbdGazeAccuracy::handle_enable_data_collection_service, this,
                                                                                           std::placeholders::_1,
                                                                                           std::placeholders::_2,
                                                                                           std::placeholders::_3));

  // Open file for writing the data
  data_file_.open("rgbd_gaze_data.csv", std::ios::out | std::ios::app);

  // Pring headers into the file
  data_file_ << "epoch,"
             << "L_visual_axis_x,L_visual_axis_y,L_visual_axis_z,L_real_gaze_x,L_real_gaze_y,L_real_gaze_z,L_alpha,L_beta,L_kappa,"
             << "R_visual_axis_x,R_visual_axis_y,R_visual_axis_z,R_real_gaze_x,R_real_gaze_y,R_real_gaze_z,R_alpha,R_beta,R_kappa,"
             << "head_rotation_roll,head_rotation_pitch,head_rotation_yaw,head_to_camera_distance,head_to_point_distance\n";

  RCLCPP_INFO(this->get_logger(), "Node initialised");
}

RgbdGazeAccuracy::~RgbdGazeAccuracy()
{
  data_file_.flush();
  data_file_.close();
}

void RgbdGazeAccuracy::handle_enable_data_collection_service(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    const std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  (void)request_header;
  enable_data_collection_ = request->data;
  response->success = true;
}

void RgbdGazeAccuracy::synchronized_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg_head_pose,
                                             const gaze_msgs::msg::GazeBinocularStamped::SharedPtr msg_visual_axes,
                                             const geometry_msgs::msg::PointStamped::SharedPtr msg_scene_point)
{
  RCLCPP_DEBUG(this->get_logger(), "Received synchronized messages for processing");

  if (!enable_data_collection_)
  {
    return;
  }

  epoch_++;

  // Convert head pose to Eigen
  Eigen::Isometry3d head_pose;
  tf2::fromMsg(msg_head_pose->pose, head_pose);

  // Convert scene point to Eigen
  Eigen::Vector3d scene_point;
  Eigen::fromMsg(msg_scene_point->point, scene_point);

  // Create a parametrized lines from visual axes
  Eigen::ParametrizedLine<double, 3> visual_axes[2];
  for (uint8_t eye = 0; eye < 2; eye++)
  {
    Eigen::Vector3d origin, direction;
    Eigen::fromMsg(msg_visual_axes->gaze[eye].origin, origin);
    Eigen::fromMsg(msg_visual_axes->gaze[eye].direction, direction);
    visual_axes[eye] = Eigen::ParametrizedLine<double, 3>(origin, direction);
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

    // Transform scene point into coordinate system of visual axes
    scene_point = tf2::transform(scene_point, tf2::transformToEigen(transform_stamped.transform));
  }

  // Convert visual axes into coordinate system of the head pose, if needed
  if (msg_visual_axes->header.frame_id != msg_head_pose->header.frame_id)
  {
    // Attemp to find transformation between the coordinate frames
    geometry_msgs::msg::TransformStamped transform_stamped;
    try
    {
      transform_stamped = tf2_buffer_.lookupTransform(msg_visual_axes->header.frame_id, msg_head_pose->header.frame_id, rclcpp::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
      return;
    }

    // Transform visual axes into coordinate system of visual axes
    for (uint8_t eye = 0; eye < 2; eye++)
    {
      visual_axes[eye] = tf2::transform(visual_axes[eye], tf2::transformToEigen(transform_stamped.transform));
    }
  }

  // Compute kappa
  double alpha[2];
  double beta[2];
  double kappa[2];
  Eigen::Vector3d visual_axes_direction_wrt_head[2];
  Eigen::Vector3d real_gaze_direction_wrt_head[2];
  Eigen::ParametrizedLine<double, 3> real_gaze[2];
  for (uint8_t eye = 0; eye < 2; eye++)
  {
    // Construct the real gaze based on the scene point
    real_gaze[eye] = Eigen::ParametrizedLine<double, 3>(visual_axes[eye].origin(), (scene_point - visual_axes[eye].origin()).normalized());

    // Transform visual axes and real gaze of both eyes to head coordinate frame (kappa is fixed with this frame)
    visual_axes_direction_wrt_head[eye] = head_pose.inverse().rotation() * visual_axes[eye].direction();
    real_gaze_direction_wrt_head[eye] = head_pose.inverse().rotation() * real_gaze[eye].direction();

    // Compute alpha (around x)
    alpha[eye] = compute_angle(visual_axes_direction_wrt_head[eye].y(), visual_axes_direction_wrt_head[eye].z(),
                               real_gaze_direction_wrt_head[eye].y(), real_gaze_direction_wrt_head[eye].z());

    // Compute beta (around y)
    beta[eye] = compute_angle(visual_axes_direction_wrt_head[eye].z(), visual_axes_direction_wrt_head[eye].x(),
                              real_gaze_direction_wrt_head[eye].z(), real_gaze_direction_wrt_head[eye].x());

    kappa[eye] = std::sqrt(alpha[eye] * alpha[eye] + beta[eye] * beta[eye]);
  }

  // Write data in
  {
    auto head_rot_euler = head_pose.rotation().eulerAngles(0, 1, 2);

    if (DEG)
    {
      for (uint8_t eye = 0; eye < 2; eye++)
      {
        alpha[eye] *= 180 / M_PI;
        beta[eye] *= 180 / M_PI;
        kappa[eye] *= 180 / M_PI;
      }
      head_rot_euler *= 180 / M_PI;
    }

    data_file_ << std::to_string(epoch_) << ","
               << std::to_string(visual_axes_direction_wrt_head[EYE_LEFT].x()) << "," << std::to_string(visual_axes_direction_wrt_head[EYE_LEFT].y()) << "," << std::to_string(visual_axes_direction_wrt_head[EYE_LEFT].z()) << "," << std::to_string(real_gaze_direction_wrt_head[EYE_LEFT].x()) << "," << std::to_string(real_gaze_direction_wrt_head[EYE_LEFT].y()) << "," << std::to_string(real_gaze_direction_wrt_head[EYE_LEFT].z()) << "," << std::to_string(alpha[EYE_LEFT]) << "," << std::to_string(beta[EYE_LEFT]) << "," << std::to_string(kappa[EYE_LEFT]) << ","
               << std::to_string(visual_axes_direction_wrt_head[EYE_RIGHT].x()) << "," << std::to_string(visual_axes_direction_wrt_head[EYE_RIGHT].y()) << "," << std::to_string(visual_axes_direction_wrt_head[EYE_RIGHT].z()) << "," << std::to_string(real_gaze_direction_wrt_head[EYE_RIGHT].x()) << "," << std::to_string(real_gaze_direction_wrt_head[EYE_RIGHT].y()) << "," << std::to_string(real_gaze_direction_wrt_head[EYE_RIGHT].z()) << "," << std::to_string(alpha[EYE_RIGHT]) << "," << std::to_string(beta[EYE_RIGHT]) << "," << std::to_string(kappa[EYE_RIGHT]) << ","
               << std::to_string(head_rot_euler.x()) << "," << std::to_string(head_rot_euler.y()) << "," << std::to_string(head_rot_euler.z()) << "," << std::to_string(head_pose.translation().norm()) << "," << std::to_string((head_pose.translation() - scene_point).norm()) << "\n";
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

      // Visual axis
      {
        visualization_msgs::msg::Marker visual_axis_marker = default_marker;
        visual_axis_marker.id = 0;
        visual_axis_marker.type = visualization_msgs::msg::Marker::ARROW;
        geometry_msgs::msg::Point start, end;
        start = Eigen::toMsg(visual_axes[eye].origin());
        end = Eigen::toMsg(visual_axes[eye].pointAt(VISUAL_VISUAL_AXIS_LENGTH));
        visual_axis_marker.points.push_back(start);
        visual_axis_marker.points.push_back(end);
        visual_axis_marker.scale.x = VISUAL_VISUAL_AXIS_WIDTH;
        visual_axis_marker.scale.y =
            visual_axis_marker.scale.z = 0;
        visual_axis_marker.color.r = VISUAL_VISUAL_AXIS_COLOR[0];
        visual_axis_marker.color.g = VISUAL_VISUAL_AXIS_COLOR[1];
        visual_axis_marker.color.b = VISUAL_VISUAL_AXIS_COLOR[2];
        visual_axis_marker.color.a = VISUAL_VISUAL_AXIS_COLOR[3];
        markers.markers.push_back(visual_axis_marker);
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

/// Main function that initiates an object of `RgbdGazeAccuracy` class as the core of this node.
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RgbdGazeAccuracy>());
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
