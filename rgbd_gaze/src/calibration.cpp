/// Personal calibration for use with RGB-D gaze estimation

////////////////////
/// DEPENDENCIES ///
////////////////////

// ROS 2
#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <tf2_eigen/tf2_eigen.h>
#include <pcl_conversions/pcl_conversions.h>

// ROS 2 interfaces
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <eyelid_contour_msgs/msg/eyelid_contours_stamped.hpp>

// PCL
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/lmeds.h>
#include <pcl/sample_consensus/msac.h>
#include <pcl/sample_consensus/rransac.h>
#include <pcl/sample_consensus/rmsac.h>
#include <pcl/sample_consensus/mlesac.h>
#include <pcl/sample_consensus/prosac.h>
#include <pcl/sample_consensus/sac_model_sphere.h>

//////////////////
/// NAMESPACES ///
//////////////////

using namespace std::placeholders;
using namespace std::chrono_literals;

/////////////////
/// CONSTANTS ///
/////////////////

/// The name of this node
const std::string NODE_NAME = "rgbd_gaze_calibration";
/// Size of the qeueu size used by the synchronizer in its policy
const uint8_t SYNCHRONIZER_QUEUE_SIZE = 10;

/// Index of the left eye
const uint8_t EYE_LEFT = 0;
/// Index of the right eye
const uint8_t EYE_RIGHT = 1;

/// Determines how often to fit the observed eyelid point into the eyeball
const uint8_t FIT_MODEL_EVERY_X_EPOCHS = 5;

const float VISUAL_PUPIL_CENTRE_SCALE = 0.005;
const float VISUAL_PUPIL_CENTRE_COLOR[] = {0.25, 0.25, 1.0, 1};
const float VISUAL_EYEBALL_COLOR[] = {0, 0.75, 0, 1};

/////////////
/// TYPES ///
/////////////

/// Policy of the synchronizer
typedef message_filters::sync_policies::ExactTime<geometry_msgs::msg::PoseStamped,
                                                  eyelid_contour_msgs::msg::EyelidContoursStamped>
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
  /// Subscriber to eyelid contours
  message_filters::Subscriber<eyelid_contour_msgs::msg::EyelidContoursStamped> sub_eyelid_contours_;
  /// Synchronizer of the subscribers
  message_filters::Synchronizer<synchronizer_policy> synchronizer_;

  /// Publisher of visualisation markers
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  /// Publisher of point cloud containing cummulative contours of eyelids
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pc_;

  /// Cummulative point cloud of the observer eyelid contours
  pcl::PointCloud<pcl::PointXYZ>::Ptr eyelids_cummulative_[2];

  /// Number of times that the synchronised epoch was called
  uint64_t epoch_;

  /// Callback called each time a message is received on all topics
  void synchronized_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg_head_pose,
                             const eyelid_contour_msgs::msg::EyelidContoursStamped::SharedPtr msg_eyelid_contours);
};

RgbdGaze::RgbdGaze() : Node(NODE_NAME),
                       sub_head_pose_(this, "head_pose"),
                       sub_eyelid_contours_(this, "eyelid_contours"),
                       synchronizer_(synchronizer_policy(SYNCHRONIZER_QUEUE_SIZE), sub_head_pose_, sub_eyelid_contours_),
                       epoch_(0)
{
  // Synchronize the subscriptions under a single callback
  synchronizer_.registerCallback(&RgbdGaze::synchronized_callback, this);

  // Register publisher of the visualisation markers
  rclcpp::QoS qos_markers = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
  pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("rgbd_gaze_calibration_markers", qos_markers);

  // Register publisher of the eyelid contours point cloud
  rclcpp::QoS qos_pc = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
  pub_pc_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("eyelid_contour_pc", qos_pc);

  // Parameters of the element
  this->declare_parameter<double>("eyeball_radius.min", 0.0105);
  this->declare_parameter<double>("eyeball_radius.max", 0.0135);
  this->declare_parameter<double>("sample_consensus.distance_threshold", 0.0025);
  this->declare_parameter<double>("sample_consensus.probability", 0.99);
  this->declare_parameter<int>("sample_consensus.max_iterations", 1000);
  this->declare_parameter<bool>("publish_markers", true);
  this->declare_parameter<bool>("publish_eyelid_pc", true);

  // Initialise point clouds
  for (uint8_t eye = 0; eye < 2; eye++)
  {
    eyelids_cummulative_[eye].reset(new pcl::PointCloud<pcl::PointXYZ>);
    eyelids_cummulative_[eye]->is_dense = true;
    eyelids_cummulative_[eye]->height = 1;
    eyelids_cummulative_[eye]->width = 0;
  }

  RCLCPP_INFO(this->get_logger(), "Node initialised");
}

void RgbdGaze::synchronized_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg_head_pose,
                                     const eyelid_contour_msgs::msg::EyelidContoursStamped::SharedPtr msg_eyelid_contours)
{
  RCLCPP_DEBUG(this->get_logger(), "Received synchronized messages for processing");

  // Convert head pose ROS2 to Eigen (wrt. camera)
  Eigen::Affine3d head_pose;
  tf2::fromMsg(msg_head_pose->pose, head_pose);

  // Iterate over both eyes and add observer eyelid contours into the cummulative point clouds
  for (uint8_t eye = 0; eye < 2; eye++)
  {
    for (auto &&point_msg : msg_eyelid_contours->eyes.eyelids[eye].contour)
    {

      // Convert eyelid point ROS2 to Eigen (wrt. camera)
      Eigen::Vector3d point_wrt_camera;
      Eigen::fromMsg(point_msg, point_wrt_camera);

      // Transform pupil centre to head coordinates to gain head pose invariance
      Eigen::Vector3d point = head_pose.inverse() * point_wrt_camera;

      // Add the point to the cummulative point cloud
      eyelids_cummulative_[eye]->push_back(pcl::PointXYZ(point[0], point[1], point[2]));
    }
  }

  // Publish all observations of the eyelid contours as a point cloud, if desired
  if (this->get_parameter("publish_eyelid_pc").get_value<bool>())
  {
    // Transform point cloud back into camera frame and convert it into msg
    sensor_msgs::msg::PointCloud2 single_eyelid_pcs[2];
    for (uint8_t eye = 0; eye < 2; eye++)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::transformPointCloud(*eyelids_cummulative_[eye], *transformed_cloud, head_pose);
      pcl::toROSMsg(*transformed_cloud, single_eyelid_pcs[eye]);
    }

    // Concatenate the point clouds
    sensor_msgs::msg::PointCloud2 both_eyes_pc;
    pcl::concatenatePointCloud(single_eyelid_pcs[EYE_LEFT], single_eyelid_pcs[EYE_RIGHT], both_eyes_pc);

    // Publish
    both_eyes_pc.header = msg_eyelid_contours->header;
    pub_pc_->publish(both_eyes_pc);
  }

  // Increment epoch and continue only if necessary
  if (FIT_MODEL_EVERY_X_EPOCHS > 0 && ++epoch_ % FIT_MODEL_EVERY_X_EPOCHS != 0)
  {
    return;
  }

  // Create output for the fitted eyeball (w.r.t head)
  Eigen::Vector3d eyeball_centres[2];
  float eyeball_radii[2];

  // Fit eyeball to the point for both eyes
  for (uint8_t eye = 0; eye < 2; eye++)
  {

    // Create sample consensus model for the eyeball, approximated by a sphere and use cummulative eyelid point clouds as input
    pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr
        sphere_sac_model(new pcl::SampleConsensusModelSphere<pcl::PointXYZ>(eyelids_cummulative_[eye]));

    // Set limits on the eyeball size, defaults to human average
    sphere_sac_model->setRadiusLimits(this->get_parameter("eyeball_radius.min").get_value<double>(),
                                      this->get_parameter("eyeball_radius.max").get_value<double>());

    // Create output coefficients of the sphere
    Eigen::VectorXf sphere_coefficients;

    // Fit the model
    pcl::LeastMedianSquares<pcl::PointXYZ> sac(sphere_sac_model);
    sac.setDistanceThreshold(this->get_parameter("sample_consensus.distance_threshold").get_value<double>());
    sac.setProbability(this->get_parameter("sample_consensus.probability").get_value<double>());
    sac.setMaxIterations(this->get_parameter("sample_consensus.max_iterations").get_value<int>());
    sac.computeModel();
    sac.getModelCoefficients(sphere_coefficients);

    eyeball_centres[eye] = sphere_coefficients.head<3>().cast<double>();
    eyeball_radii[eye] = sphere_coefficients[3];
  }

  // Print results
  RCLCPP_INFO(this->get_logger(), "-- Results of epoch #%d --", epoch_);
  for (uint8_t eye = 0; eye < 2; eye++)
  {
    std::string eye_side;
    if (eye == EYE_LEFT)
    {
      eye_side = "Left";
    }
    else
    {
      eye_side = "Right";
    }
    RCLCPP_INFO(this->get_logger(),
                "\n" + eye_side + ":\n\teyeball_centre:\n\t\tx: %f\n\t\ty: %f\n\t\tz: %f\n\teyeball_radius: %f",
                eyeball_centres[eye][0], eyeball_centres[eye][1], eyeball_centres[eye][2], eyeball_radii[eye]);
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
      default_marker.ns = std::string(this->get_namespace()) + "eyeball" + eye_side;

      // Eyeball centre
      {
        // Transform eyeball back to the camera frame
        Eigen::Vector3d eyeball_centre = head_pose * eyeball_centres[eye];

        visualization_msgs::msg::Marker eyeball_centre_marker = default_marker;
        eyeball_centre_marker.id = 0;
        eyeball_centre_marker.type = visualization_msgs::msg::Marker::SPHERE;
        eyeball_centre_marker.pose.position.x = eyeball_centre[0];
        eyeball_centre_marker.pose.position.y = eyeball_centre[1];
        eyeball_centre_marker.pose.position.z = eyeball_centre[2];
        eyeball_centre_marker.scale.x =
            eyeball_centre_marker.scale.y =
                eyeball_centre_marker.scale.z = 2 * eyeball_radii[eye];
        eyeball_centre_marker.color.r = VISUAL_EYEBALL_COLOR[0];
        eyeball_centre_marker.color.g = VISUAL_EYEBALL_COLOR[1];
        eyeball_centre_marker.color.b = VISUAL_EYEBALL_COLOR[2];
        eyeball_centre_marker.color.a = VISUAL_EYEBALL_COLOR[3];
        markers.markers.push_back(eyeball_centre_marker);
      }
    }

    pub_markers_->publish(markers);
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
