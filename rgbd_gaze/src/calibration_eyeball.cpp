/// Personal calibration of eyeball for use with RGB-D gaze estimation

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
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/sample_consensus/lmeds.h>
#include <pcl/sample_consensus/sac_model_sphere.h>

/////////////////
/// CONSTANTS ///
/////////////////

/// The name of this node
const std::string NODE_NAME = "rgbd_gaze_calibration_eyeball";
/// Size of the queue size used by the synchronizer in its policy
const uint8_t SYNCHRONIZER_QUEUE_SIZE = 50;

/// Index of the left eye
const uint8_t EYE_LEFT = 0;
/// Index of the right eye
const uint8_t EYE_RIGHT = 1;

/// If enabled, model is fit every epoch. Otherwise, model is fit every `FIT_MODEL_FREQUENCY^n` epochs (rounded up).
const bool FIT_MODEL_EVERY_EPOCH = false;
const float FIT_MODEL_FREQUENCY = 1.2;

/// Number of eyelid landmarks for each eye. Index 0 and EYELID_LANDMARKS_COUNT/2.0 must correspond to eyelid corners.
const uint8_t EYELID_LANDMARKS_COUNT = 12;

const float VISUAL_EYEBALL_COLOR[] = {0, 0, 0.5, 0.75};

/////////////
/// TYPES ///
/////////////

/// Policy of the synchronizer
typedef message_filters::sync_policies::ExactTime<geometry_msgs::msg::PoseStamped,
                                                  eyelid_contour_msgs::msg::EyelidContoursStamped>
    synchronizer_policy;

//////////////////
/// NODE CLASS ///
//////////////////

/// Class representation of this node
class RgbdGazeCalibrationEyeball : public rclcpp::Node
{
public:
  /// Constructor
  RgbdGazeCalibrationEyeball();

private:
  /// Subscriber to the head pose
  message_filters::Subscriber<geometry_msgs::msg::PoseStamped> sub_head_pose_;
  /// Subscriber to eyelid contours
  message_filters::Subscriber<eyelid_contour_msgs::msg::EyelidContoursStamped> sub_eyelid_contours_;
  /// Synchronizer of the subscribers
  message_filters::Synchronizer<synchronizer_policy> synchronizer_;

  /// Publisher of visualisation markers
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  /// Publisher of point cloud containing cumulative contours of eyelids
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pc_;

  /// Cumulative point clouds of the observer eyelid contours
  pcl::PointCloud<pcl::PointXYZ>::Ptr eyelids_cumulative_[2];

  /// Number of times that the synchronised epoch was called
  uint64_t epoch_;
  uint16_t every_x_epochs_;

  /// Callback called each time a message is received on all topics
  void synchronized_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg_head_pose,
                             const eyelid_contour_msgs::msg::EyelidContoursStamped::SharedPtr msg_eyelid_contours);
};

RgbdGazeCalibrationEyeball::RgbdGazeCalibrationEyeball() : Node(NODE_NAME),
                                                           sub_head_pose_(this, "head_pose"),
                                                           sub_eyelid_contours_(this, "eyelid_contours"),
                                                           synchronizer_(synchronizer_policy(SYNCHRONIZER_QUEUE_SIZE), sub_head_pose_, sub_eyelid_contours_),
                                                           epoch_(0),
                                                           every_x_epochs_(1)
{
  // Synchronize the subscriptions under a single callback
  synchronizer_.registerCallback(&RgbdGazeCalibrationEyeball::synchronized_callback, this);

  // Parameters of the element
  this->declare_parameter<bool>("eyeball_radius.auto", true);
  this->declare_parameter<double>("eyeball_radius.auto_tolerance", 0.001);
  this->declare_parameter<double>("eyeball_radius.min", 0.0105);
  this->declare_parameter<double>("eyeball_radius.max", 0.0135);
  this->declare_parameter<bool>("sample_consensus.use_landmark_centroids", false);
  this->declare_parameter<double>("sample_consensus.distance_threshold", 0.0025);
  this->declare_parameter<double>("sample_consensus.probability", 0.99);
  this->declare_parameter<int>("sample_consensus.max_iterations", 1000);
  this->declare_parameter<bool>("publish_markers", true);
  this->declare_parameter<bool>("publish_eyelid_pc", true);

  // Register publisher of the visualisation markers
  if (this->get_parameter("publish_markers").get_value<bool>())
  {
    rclcpp::QoS qos_markers = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualisation_markers", qos_markers);
  }

  // Register publisher of the eyelid contours point cloud
  if (this->get_parameter("publish_eyelid_pc").get_value<bool>())
  {
    rclcpp::QoS qos_pc = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    pub_pc_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("eyelid_contours_cumulative", qos_pc);
  }

  // Initialise point clouds
  for (uint8_t eye = 0; eye < 2; eye++)
  {
    eyelids_cumulative_[eye].reset(new pcl::PointCloud<pcl::PointXYZ>);
    eyelids_cumulative_[eye]->is_dense = true;
    eyelids_cumulative_[eye]->height = 1;
    eyelids_cumulative_[eye]->width = 0;
  }

  RCLCPP_INFO(this->get_logger(), "Node initialised");
}

void RgbdGazeCalibrationEyeball::synchronized_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg_head_pose,
                                                       const eyelid_contour_msgs::msg::EyelidContoursStamped::SharedPtr msg_eyelid_contours)
{
  RCLCPP_DEBUG(this->get_logger(), "Received synchronized messages for processing");
  epoch_++;

  // Convert head pose ROS2 to Eigen (wrt. camera)
  Eigen::Affine3d head_pose;
  tf2::fromMsg(msg_head_pose->pose, head_pose);

  // Iterate over both eyes and add observer eyelid contours into the cumulative point clouds
  for (uint8_t eye = 0; eye < 2; eye++)
  {
    for (auto &&point_msg : msg_eyelid_contours->eyes.eyelids[eye].contour)
    {

      // Convert eyelid point ROS2 to Eigen (wrt. camera)
      Eigen::Vector3d point_wrt_camera;
      Eigen::fromMsg(point_msg, point_wrt_camera);

      // Transform pupil centre to head coordinates to gain head pose invariance
      Eigen::Vector3d point = head_pose.inverse() * point_wrt_camera;

      // Add the point to the cumulative point cloud
      eyelids_cumulative_[eye]->push_back(pcl::PointXYZ(point[0], point[1], point[2]));
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
      pcl::transformPointCloud(*eyelids_cumulative_[eye], *transformed_cloud, head_pose);
      pcl::toROSMsg(*transformed_cloud, single_eyelid_pcs[eye]);
    }

    // Concatenate the point clouds
    sensor_msgs::msg::PointCloud2 both_eyes_pc;
    pcl::concatenatePointCloud(single_eyelid_pcs[EYE_LEFT], single_eyelid_pcs[EYE_RIGHT], both_eyes_pc);

    // Publish
    both_eyes_pc.header = msg_eyelid_contours->header;
    pub_pc_->publish(both_eyes_pc);
  }

  if (!FIT_MODEL_EVERY_EPOCH)
  {
    // Increment epoch and continue only if necessary
    if (epoch_ % every_x_epochs_ != 0)
    {
      return;
    }
    every_x_epochs_ = std::ceil(every_x_epochs_ * FIT_MODEL_FREQUENCY);
  }

  // Create output for the fitted eyeball (w.r.t head)
  Eigen::Vector3d eyeball_centres[2];
  float eyeball_radii[2];

  // Fit eyeball to the point for both eyes
  for (uint8_t eye = 0; eye < 2; eye++)
  {
    // Create input point cloud for sample consensus algorithm
    pcl::PointCloud<pcl::PointXYZ>::Ptr sac_input;
    sac_input.reset(new pcl::PointCloud<pcl::PointXYZ>);
    sac_input->is_dense = true;
    sac_input->height = 1;
    sac_input->width = 0;

    // Create a point cloud for each feature on the eyelid and fill them (for outlier removal)
    pcl::PointCloud<pcl::PointXYZ>::Ptr eyelid_landmarks[EYELID_LANDMARKS_COUNT];
    for (uint8_t i = 0; i < EYELID_LANDMARKS_COUNT; i++)
    {
      eyelid_landmarks[i].reset(new pcl::PointCloud<pcl::PointXYZ>);
      eyelid_landmarks[i]->is_dense = true;
      eyelid_landmarks[i]->height = 1;
      eyelid_landmarks[i]->width = eyelids_cumulative_[eye]->size() / EYELID_LANDMARKS_COUNT;
    }
    for (uint16_t i = 0; i < eyelids_cumulative_[eye]->size(); ++i)
    {
      eyelid_landmarks[i % EYELID_LANDMARKS_COUNT]->points.push_back(eyelids_cumulative_[eye]->at(i));
    }

    bool use_landmark_centroids = this->get_parameter("sample_consensus.use_landmark_centroids").get_value<bool>();
    bool use_auto_radius = this->get_parameter("eyeball_radius.auto").get_value<bool>();

    // Remove outliers from each landmark
    Eigen::Vector3f landmark_centroids[EYELID_LANDMARKS_COUNT];
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statistical_outlier_removal;
    statistical_outlier_removal.setMeanK(3);
    statistical_outlier_removal.setStddevMulThresh(1);
    for (uint8_t i = 0; i < EYELID_LANDMARKS_COUNT; i++)
    {
      statistical_outlier_removal.setInputCloud(eyelid_landmarks[i]);
      std::vector<int> inliers;
      statistical_outlier_removal.filter(inliers);

      // Find centroids, if needed
      if (use_landmark_centroids || use_auto_radius)
      {
        landmark_centroids[i] = Eigen::Vector3f(0.0, 0.0, 0.0);
        for (auto &inlier : inliers)
        {
          pcl::PointXYZ point = eyelid_landmarks[i]->at(inlier);
          landmark_centroids[i] += Eigen::Vector3f(point.x, point.y, point.z);
        }
        landmark_centroids[i] /= inliers.size();
      }

      // Setup point cloud for sample consensus algorithm
      if (use_landmark_centroids)
      {
        // Use centroids
        sac_input->push_back(pcl::PointXYZ(landmark_centroids[i].x(), landmark_centroids[i].y(), landmark_centroids[i].z()));
      }
      else
      {
        // Else use all inliers
        for (auto &inlier : inliers)
        {
          sac_input->push_back(eyelid_landmarks[i]->at(inlier));
        }
      }
    }

    // Create sample consensus model for the eyeball, approximated by a sphere and use cumulative eyelid point clouds as input
    pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr sphere_sac_model(new pcl::SampleConsensusModelSphere<pcl::PointXYZ>(sac_input));

    if (use_auto_radius)
    {
      // Compute estimate of eyeball radius from eyelid corners
      double eyeball_radius_estimate = (landmark_centroids[0].head<2>() - landmark_centroids[6].head<2>()).norm() / 2.0;

      // Set limits on the eyeball size based on the estimate
      double tolerance = this->get_parameter("eyeball_radius.auto_tolerance").get_value<double>();
      sphere_sac_model->setRadiusLimits(eyeball_radius_estimate - tolerance,
                                        eyeball_radius_estimate + tolerance);
    }
    else
    {
      // Set limits on the eyeball size, defaults to human average
      sphere_sac_model->setRadiusLimits(this->get_parameter("eyeball_radius.min").get_value<double>(),
                                        this->get_parameter("eyeball_radius.max").get_value<double>());
    }

    // Create output coefficients of the sphere
    Eigen::VectorXf sphere_coefficients;

    // Fit the model
    pcl::LeastMedianSquares<pcl::PointXYZ> sac(sphere_sac_model);
    sac.setDistanceThreshold(this->get_parameter("sample_consensus.distance_threshold").get_value<double>());
    sac.setProbability(this->get_parameter("sample_consensus.probability").get_value<double>());
    sac.setMaxIterations(this->get_parameter("sample_consensus.max_iterations").get_value<int>());
    sac.computeModel();
    sac.getModelCoefficients(sphere_coefficients);

    // Make sure the correct number of coefficients was returned
    if (sphere_coefficients.size() != 4)
    {
      return;
    }

    eyeball_centres[eye] = sphere_coefficients.head<3>().cast<double>();
    eyeball_radii[eye] = sphere_coefficients[3];
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

      output_string += "\n" + eye_side + ":\n\teyeball_centre:\n\t\tx: " + std::to_string(eyeball_centres[eye][0]) +
                       "\n\t\ty: " + std::to_string(eyeball_centres[eye][1]) +
                       "\n\t\tz: " + std::to_string(eyeball_centres[eye][2]) +
                       "\n\teyeball_radius: " + std::to_string(eyeball_radii[eye]);
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

/// Main function that initiates an object of `RgbdGazeCalibrationEyeball` class as the core of this node.
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RgbdGazeCalibrationEyeball>());
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
