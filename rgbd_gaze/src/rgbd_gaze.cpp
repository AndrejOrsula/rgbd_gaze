/// Gaze estimator based on 3D model of the eye

////////////////////
/// DEPENDENCIES ///
////////////////////

// ROS 2
#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// ROS 2 interfaces
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <rgbd_gaze_msgs/msg/pupil_centres_stamped.hpp>

// Eigen
#include <Eigen/Geometry>

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

/// Index of the left eye
const uint8_t EYE_LEFT = 0;
/// Index of the right eye
const uint8_t EYE_RIGHT = 1;

const float VISUAL_PUPIL_CENTRE_SCALE = 0.0075;
const float VISUAL_PUPIL_CENTRE_COLOR[] = {0, 0, 0.25, 1};

const float VISUAL_EYEBALL_CENTRE_SCALE = 0.01;
const float VISUAL_EYEBALL_CENTRE_COLOR[] = {0, 0, 0.5, 1};

const float VISUAL_CORNEA_CENTRE_SCALE = 0.005;
const float VISUAL_CORNEA_CENTRE_COLOR[] = {0, 0, 0.75, 1};

const float VISUAL_OPTICAL_AXIS_LENGTH = 2.0;
const float VISUAL_OPTICAL_AXIS_WIDTH = 0.0025;
const float VISUAL_OPTICAL_AXIS_COLOR[] = {1.0, 0, 0, 1};

const float VISUAL_VISUAL_AXIS_LENGTH = 2.0;
const float VISUAL_VISUAL_AXIS_WIDTH = 0.0025;
const float VISUAL_VISUAL_AXIS_COLOR[] = {0, 1.0, 0, 1};

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

void transform_to_camera_frame(Eigen::ParametrizedLine<double, 3> &parametrized_line, const Eigen::Isometry3d &head_pose)
{
  parametrized_line.origin() = head_pose * parametrized_line.origin();
  parametrized_line.direction() = (head_pose * parametrized_line.direction()).normalized();
}

////////////////////
/// HELPER CLASS ///
////////////////////

class Eye3dModel
{
public:
  /// Position of the eyeball centre wrt. head
  Eigen::Vector3d eyeball_centre_;
  /// Horizontal angular deviation between optical and visual axis, rotated around centre of corneal curvature - in radians
  double alpha_;
  /// Vertical angular deviation between optical and visual axis, rotated around centre of corneal curvature - in radians
  double beta_;
  /// Distance between eyebal centre and the entre of the corneal curvature
  double distance_eyeball_cornea_;

  Eigen::ParametrizedLine<double, 3> determine_optical_axis(const Eigen::Isometry3d &head_pose, const Eigen::Vector3d &pupil_centre);
  Eigen::ParametrizedLine<double, 3> determine_visual_axis(const Eigen::ParametrizedLine<double, 3> &optical_axis);
};

Eigen::ParametrizedLine<double, 3> Eye3dModel::determine_optical_axis(const Eigen::Isometry3d &head_pose, const Eigen::Vector3d &pupil_centre)
{
  // Get pupil centre (wrt. head)
  Eigen::Vector3d pupil_centre_wrt_head = head_pose.inverse() * pupil_centre;

  // Determine optical axis unit vector (wrt. head)
  Eigen::Vector3d optical_axis_unit_vect = (pupil_centre_wrt_head - eyeball_centre_).normalized();

  // Represent optical axis as parametrized line (wrt. head)
  return Eigen::ParametrizedLine<double, 3>(eyeball_centre_, optical_axis_unit_vect);
}

Eigen::ParametrizedLine<double, 3> Eye3dModel::determine_visual_axis(const Eigen::ParametrizedLine<double, 3> &optical_axis)
{
  // Determine visual axis unit vector by the use of kappa (wrt. head)
  Eigen::Quaternion kappa = Eigen::AngleAxisd(beta_, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(alpha_, Eigen::Vector3d::UnitX());

  // Rotate
  Eigen::Vector3d visual_axis_unit_vect = kappa * optical_axis.direction();

  // Represent visual axis as a parametrized line (wrt. head)
  Eigen::ParametrizedLine<double, 3> visual_axis(optical_axis.pointAt(distance_eyeball_cornea_), visual_axis_unit_vect);
}

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

  /// Eye model parameters based on specific user calibration
  Eye3dModel eye_models_[2];

  /// Publisher of the pupil centres
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;

  /// Broadcaster of the TF frames
  tf2_ros::TransformBroadcaster tf2_broadcaster_;

  /// Callback called each time a message is received on all topics
  void synchronized_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg_head_pose,
                             const rgbd_gaze_msgs::msg::PupilCentresStamped::SharedPtr msg_pupil_centres);
};

RgbdGaze::RgbdGaze() : Node(NODE_NAME),
                       sub_head_pose_(this, "head_pose"),
                       sub_pupil_centres_(this, "pupil_centres"),
                       synchronizer_(synchronizer_policy(SYNCHRONIZER_QUEUE_SIZE), sub_head_pose_, sub_pupil_centres_),
                       tf2_broadcaster_(this)
{
  // Synchronize the subscriptions under a single callback
  synchronizer_.registerCallback(&RgbdGaze::synchronized_callback, this);

  // Parameters of the element
  this->declare_parameter<bool>("visualise", true);
  this->declare_parameter<bool>("broadcast_tf", false);
  this->declare_parameter<bool>("publish_markers", true);

  // Register publisher of the pupil centres
  rclcpp::QoS qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
  pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("rgbd_gaze_markers", qos);

  // User specific parameters obtained via calibration
  this->declare_parameter<std::string>("user.id", "");
  // Left eye
  eye_models_[EYE_LEFT].eyeball_centre_[0] = this->declare_parameter<double>("user.eyes.left.eyeball_centre_.x", 0.0);
  eye_models_[EYE_LEFT].eyeball_centre_[1] = this->declare_parameter<double>("user.eyes.left.eyeball_centre_.y", 0.0);
  eye_models_[EYE_LEFT].eyeball_centre_[2] = this->declare_parameter<double>("user.eyes.left.eyeball_centre_.z", 0.0);
  eye_models_[EYE_LEFT].alpha_ = this->declare_parameter<double>("user.eyes.left.kappa.alpha", 0.0);
  eye_models_[EYE_LEFT].beta_ = this->declare_parameter<double>("user.eyes.left.kappa.beta", 0.0);
  eye_models_[EYE_LEFT].distance_eyeball_cornea_ = this->declare_parameter<double>("user.eyes.left.distance_eyeball_cornea", 0.0);
  // Right eye
  eye_models_[EYE_RIGHT].eyeball_centre_[0] = this->declare_parameter<double>("user.eyes.right.eyeball_centre_.x", 0.0);
  eye_models_[EYE_RIGHT].eyeball_centre_[1] = this->declare_parameter<double>("user.eyes.right.eyeball_centre_.y", 0.0);
  eye_models_[EYE_RIGHT].eyeball_centre_[2] = this->declare_parameter<double>("user.eyes.right.eyeball_centre_.z", 0.0);
  eye_models_[EYE_RIGHT].alpha_ = this->declare_parameter<double>("user.eyes.right.kappa.alpha", 0.0);
  eye_models_[EYE_RIGHT].beta_ = this->declare_parameter<double>("user.eyes.right.kappa.beta", 0.0);
  eye_models_[EYE_RIGHT].distance_eyeball_cornea_ = this->declare_parameter<double>("user.eyes.right.distance_eyeball_cornea", 0.0);

  RCLCPP_INFO(this->get_logger(), "Loaded parameters for user %s", this->get_parameter("user.id").get_value<std::string>().c_str());

  RCLCPP_INFO(this->get_logger(), "Node initialised");
}

void RgbdGaze::synchronized_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg_head_pose,
                                     const rgbd_gaze_msgs::msg::PupilCentresStamped::SharedPtr msg_pupil_centres)
{
  RCLCPP_INFO(this->get_logger(), "Received synchronized messages for processing");

  // Convert head pose ROS2 to Eigen (wrt. camera)
  Eigen::Isometry3d head_pose;
  tf2::fromMsg(msg_head_pose->pose, head_pose);

  // Convert pupil centre ROS2 to Eigen (wrt. camera)
  Eigen::Vector3d pupil_centres[2];
  tf2::fromMsg(msg_pupil_centres->pupils.centres[0], pupil_centres[EYE_LEFT]);
  tf2::fromMsg(msg_pupil_centres->pupils.centres[1], pupil_centres[EYE_RIGHT]);

  Eigen::ParametrizedLine<double, 3> optical_axis[2], visual_axis[2];
  for (uint8_t eye = 0; eye < 2; eye++)
  {
    // Get optical and visual axis (wrt. head pose)
    optical_axis[eye] = eye_models_[eye].determine_optical_axis(head_pose, pupil_centres[eye]);
    visual_axis[eye] = eye_models_[eye].determine_visual_axis(optical_axis[eye]);

    // Transform optical and visual axis into camera coordinate system
    transform_to_camera_frame(optical_axis[eye], head_pose);
    transform_to_camera_frame(visual_axis[eye], head_pose);
  }

  if (this->get_parameter("broadcast_tf").get_value<bool>())
  {
    // Quat for flipping the eye gaze
    tf2::Quaternion flip_quat;
    flip_quat.setRPY(0, M_PI, M_PI);

    geometry_msgs::msg::TransformStamped transform;
    transform.header = msg_pupil_centres->header;
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

      // Eyeball centre - optical axis
      transform.child_frame_id = "eyeball_centre_optical" + eye_side;
      transform.transform.translation.x = optical_axis[eye].origin()[0];
      transform.transform.translation.y = optical_axis[eye].origin()[1];
      transform.transform.translation.z = optical_axis[eye].origin()[2];
      tf2::Quaternion optical_axis_quat;
      optical_axis_quat.setValue(optical_axis[eye].direction()[0], optical_axis[eye].direction()[1], optical_axis[eye].direction()[2]);
      optical_axis_quat *= flip_quat;
      transform.transform.rotation = tf2::toMsg(optical_axis_quat);
      tf2_broadcaster_.sendTransform(transform);

      // Cornea centre - visual axis
      transform.child_frame_id = "cornea_centre_visual" + eye_side;
      transform.transform.translation.x = visual_axis[eye].origin()[0];
      transform.transform.translation.y = visual_axis[eye].origin()[1];
      transform.transform.translation.z = visual_axis[eye].origin()[2];
      tf2::Quaternion visual_axis_quat;
      visual_axis_quat.setValue(visual_axis[eye].direction()[0], visual_axis[eye].direction()[1], visual_axis[eye].direction()[2]);
      visual_axis_quat *= flip_quat;
      transform.transform.rotation = tf2::toMsg(visual_axis_quat);
      tf2_broadcaster_.sendTransform(transform);
    }
  }

  if (this->get_parameter("publish_markers").get_value<bool>())
  {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker default_marker;
    default_marker.header = msg_pupil_centres->header;
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
      default_marker.ns = std::string(this->get_namespace()) + "gaze" + eye_side;

      // Pupil centre
      {
        visualization_msgs::msg::Marker pupil_centre_marker = default_marker;
        pupil_centre_marker.id = 0;
        pupil_centre_marker.type = visualization_msgs::msg::Marker::SPHERE;
        pupil_centre_marker.pose.position = Eigen::toMsg(pupil_centres[eye]);
        pupil_centre_marker.scale.x = VISUAL_PUPIL_CENTRE_SCALE;
        pupil_centre_marker.scale.y = VISUAL_PUPIL_CENTRE_SCALE;
        pupil_centre_marker.scale.z = VISUAL_PUPIL_CENTRE_SCALE;
        pupil_centre_marker.color.r = VISUAL_PUPIL_CENTRE_COLOR[0];
        pupil_centre_marker.color.g = VISUAL_PUPIL_CENTRE_COLOR[1];
        pupil_centre_marker.color.b = VISUAL_PUPIL_CENTRE_COLOR[2];
        pupil_centre_marker.color.a = VISUAL_PUPIL_CENTRE_COLOR[3];
        markers.markers.push_back(pupil_centre_marker);
      }

      // Eyeball centre
      {
        visualization_msgs::msg::Marker eyeball_centre_marker = default_marker;
        eyeball_centre_marker.id = 1;
        eyeball_centre_marker.type = visualization_msgs::msg::Marker::SPHERE;
        eyeball_centre_marker.pose.position = Eigen::toMsg(optical_axis[eye].origin());
        eyeball_centre_marker.scale.x = VISUAL_EYEBALL_CENTRE_SCALE;
        eyeball_centre_marker.scale.y = VISUAL_EYEBALL_CENTRE_SCALE;
        eyeball_centre_marker.scale.z = VISUAL_EYEBALL_CENTRE_SCALE;
        eyeball_centre_marker.color.r = VISUAL_EYEBALL_CENTRE_COLOR[0];
        eyeball_centre_marker.color.g = VISUAL_EYEBALL_CENTRE_COLOR[1];
        eyeball_centre_marker.color.b = VISUAL_EYEBALL_CENTRE_COLOR[2];
        eyeball_centre_marker.color.a = VISUAL_EYEBALL_CENTRE_COLOR[3];
        markers.markers.push_back(eyeball_centre_marker);
      }

      // Cornea centre
      {
        visualization_msgs::msg::Marker cornea_centre_marker = default_marker;
        cornea_centre_marker.id = 2;
        cornea_centre_marker.type = visualization_msgs::msg::Marker::SPHERE;
        cornea_centre_marker.pose.position = Eigen::toMsg(visual_axis[eye].origin());
        cornea_centre_marker.scale.x = VISUAL_CORNEA_CENTRE_SCALE;
        cornea_centre_marker.scale.y = VISUAL_CORNEA_CENTRE_SCALE;
        cornea_centre_marker.scale.z = VISUAL_CORNEA_CENTRE_SCALE;
        cornea_centre_marker.color.r = VISUAL_CORNEA_CENTRE_COLOR[0];
        cornea_centre_marker.color.g = VISUAL_CORNEA_CENTRE_COLOR[1];
        cornea_centre_marker.color.b = VISUAL_CORNEA_CENTRE_COLOR[2];
        cornea_centre_marker.color.a = VISUAL_CORNEA_CENTRE_COLOR[3];
        markers.markers.push_back(cornea_centre_marker);
      }

      // Optical axis
      {
        visualization_msgs::msg::Marker optical_axis_marker = default_marker;
        optical_axis_marker.id = 3;
        optical_axis_marker.type = visualization_msgs::msg::Marker::ARROW;
        geometry_msgs::msg::Point start, end;
        start = Eigen::toMsg(optical_axis[eye].origin());
        end = Eigen::toMsg(optical_axis[eye].pointAt(VISUAL_OPTICAL_AXIS_LENGTH));
        optical_axis_marker.points.push_back(start);
        optical_axis_marker.points.push_back(end);
        optical_axis_marker.scale.x = VISUAL_OPTICAL_AXIS_WIDTH;
        optical_axis_marker.scale.y = 0;
        optical_axis_marker.scale.z = 0;
        optical_axis_marker.color.r = VISUAL_OPTICAL_AXIS_COLOR[0];
        optical_axis_marker.color.g = VISUAL_OPTICAL_AXIS_COLOR[1];
        optical_axis_marker.color.b = VISUAL_OPTICAL_AXIS_COLOR[2];
        optical_axis_marker.color.a = VISUAL_OPTICAL_AXIS_COLOR[3];
        markers.markers.push_back(optical_axis_marker);
      }

      // Optical axis
      {
        visualization_msgs::msg::Marker visual_axis_marker = default_marker;
        visual_axis_marker.id = 4;
        visual_axis_marker.type = visualization_msgs::msg::Marker::ARROW;
        geometry_msgs::msg::Point start, end;
        start = Eigen::toMsg(visual_axis[eye].origin());
        end = Eigen::toMsg(visual_axis[eye].pointAt(VISUAL_VISUAL_AXIS_LENGTH));
        visual_axis_marker.points.push_back(start);
        visual_axis_marker.points.push_back(end);
        visual_axis_marker.scale.x = VISUAL_VISUAL_AXIS_WIDTH;
        visual_axis_marker.scale.y = 0;
        visual_axis_marker.scale.z = 0;
        visual_axis_marker.color.r = VISUAL_VISUAL_AXIS_COLOR[0];
        visual_axis_marker.color.g = VISUAL_VISUAL_AXIS_COLOR[1];
        visual_axis_marker.color.b = VISUAL_VISUAL_AXIS_COLOR[2];
        visual_axis_marker.color.a = VISUAL_VISUAL_AXIS_COLOR[3];
        markers.markers.push_back(visual_axis_marker);
      }

      pub_markers_->publish(markers);
    }
  }
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
