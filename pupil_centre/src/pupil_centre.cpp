/// Estimator of pupil centre position in 3D

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
#include <gaze_msgs/msg/pupil_centres_stamped.hpp>

// OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/////////////////
/// CONSTANTS ///
/////////////////

/// The name of this node
const std::string NODE_NAME = "pupil_centre";
/// Size of the queue size used by the synchronizer in its policy
const uint8_t SYNCHRONIZER_QUEUE_SIZE = 50;

/// Index of the left eye
const uint8_t EYE_LEFT = 0;
/// Index of the right eye
const uint8_t EYE_RIGHT = 1;

/// Weights used to convert BGR image into single channel
const double BGR_TO_MONO_WEIGHTS[] = {0.0, 0.0, 1.0}; // Must add up to 1.0

/// Inflation of the eye region in horizontal direction
const uint8_t ROI_PADDING_HORZIZONTAL = 3;
/// Inflation of the eye region in vertical direction
const uint8_t ROI_PADDING_VERTICAL = 5;

/// Value of `sigmaColor` for bilateral filter that is applied to each eye image before processing
const double BILATERAL_FILTER_SIGMA_COLOR = 35.0;
/// Value of `sigmaSpace` for bilateral filter that is applied to each eye image before processing
const double BILATERAL_FILTER_SIGMA_SPACE = 2.5;

/// Determines whether to use Scharr instead of Sobel for gradient computation
const bool GRADIENTS_SCHARR_INSTEAD_SOBEL = true;
/// Determines whether to apply weights based on inverted intensity of a pixel that is considered to be pupil centre; assumption is that iris appears darker than the rest of the image
const bool APPLY_INTENSITY_WEIGHTS = true;
/// Determines whether to consider only gradients that have their magnitude at least `MINIMUM_GRADIENT_SIGNIFICANCE` percent of the maximum detected gradient magnitude while computing the objective function
const bool CONSIDER_ONLY_SIGNIFICANT_GRADIENTS = true;
/// Factor between 0.0 and 1.0 that determines minimum allowed magnitude of gradients that are included during computation of the objective function, valid only if `CONSIDER_ONLY_SIGNIFICANT_GRADIENTS` is set to true
const double MINIMUM_GRADIENT_SIGNIFICANCE = 0.25;
/// Determines whether to enable postprocessing of the objective funtion that can resolve problems with local minima close to image borders.
const bool POSTPROCESS_OBJECTIVE_FUNCTION = true;
/// Treshold used during postprocessing that determines size of blobs that are removed, if connected to image boundary.
const uint8_t POSTPROCESS_THRESHOLD = 200;
/// Determines the neighbourhood size around detected 2D pupil centre that is considered to estimation of its 3D position
const uint8_t PUPIL_3D_NEIGHBOURHOOD_SIZE = 3;

/// Delay used in the context of cv::waitKey, applicable only if visualisation is enabled
const int CV_WAITKEY_DELAY = 10;

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

namespace sensor_msgs::roi
{
/// Convert ROI msg to CV rectangle
inline cv::Rect_<int16_t> to_rect(const sensor_msgs::msg::RegionOfInterest &roi, const cv::Size_<int16_t> &max_size, const int8_t padding_horizontal = 0, const int8_t padding_vertical = 0)
{
  return cv::Rect_<int16_t>((signed)(roi.x_offset - padding_horizontal),
                            (signed)(roi.y_offset - padding_vertical),
                            (signed)(roi.width + 2 * padding_horizontal),
                            (signed)(roi.height + 2 * padding_vertical)) &
         cv::Rect_<int16_t>(0, 0, max_size.width, max_size.height);
}
} // namespace sensor_msgs::roi

namespace cv
{
/// Convert CV point to msg
inline geometry_msgs::msg::Point to_msg(const cv::Point3_<double> &point)
{
  geometry_msgs::msg::Point msg;
  msg.x = point.x;
  msg.y = point.y;
  msg.z = point.z;
  return msg;
}

/// Determines whether contour is touching border of the image.
inline bool is_contour_touching_border(const std::vector<cv::Point> &contour, const cv::Size &image_size)
{
  cv::Rect contour_bounding_rectangle = cv::boundingRect(contour);
  return contour_bounding_rectangle.x <= 0 ||
         contour_bounding_rectangle.y <= 0 ||
         contour_bounding_rectangle.br().x >= image_size.width ||
         contour_bounding_rectangle.br().y >= image_size.height;
}

/// Show image, normalize and colourize if it contains floating point data
inline void img_show(const std::string &window, const cv::Mat &img)
{
  cv::Mat img_to_show;
  if (img.type() == CV_32F || img.type() == CV_64F)
  {
    double min_val, max_val;
    cv::minMaxLoc(img, &min_val, &max_val, NULL, NULL);
    if (min_val != max_val)
    {
      img.convertTo(img_to_show, CV_8UC1, 255.0 / (max_val - min_val), -255.0 * min_val / (max_val - min_val));
      cv::applyColorMap(img_to_show, img_to_show, COLORMAP_JET);
    }
  }
  else
  {
    img_to_show = img;
  }
  cv::namedWindow(window, cv::WINDOW_KEEPRATIO);
  cv::imshow(window, img_to_show);
}

/// Shown image with pupil centre drawn as a dot
inline void img_show_pupil(const std::string &window, const cv::Mat &img, cv::Point pupil_centre, const uint16_t offset_x = 0, const uint16_t offset_y = 0)
{
  cv::Mat_<cv::Vec<uint8_t, 3>> img_to_show;
  if (img.type() == CV_8UC1)
  {
    cv::cvtColor(img, img_to_show, cv::COLOR_GRAY2BGR);
  }
  else if (img.type() == CV_8UC3)
  {
    img_to_show = img;
  }

  pupil_centre.x += offset_x;
  pupil_centre.y += offset_y;

  cv::circle(img_to_show, pupil_centre, 0, cv::Scalar(255, 0, 0));
  cv::img_show(window, img_to_show);
}
} // namespace cv

/// Extract neighbourhood around a point from an image
inline cv::Mat extract_neighbourhood(const cv::Mat &img, const cv::Point &point, const uint8_t neighbourhood = 3)
{
  uint8_t neighbourhood_half = neighbourhood / 2;
  cv::Rect_<int16_t> roi = cv::Rect_<int16_t>((signed)(point.x - neighbourhood_half),
                                              (signed)(point.y - neighbourhood_half),
                                              neighbourhood,
                                              neighbourhood) &
                           cv::Rect_<int16_t>(0, 0, img.cols, img.rows);
  return img(roi);
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
  rclcpp::Publisher<gaze_msgs::msg::PupilCentresStamped>::SharedPtr pub_pupil_centres_;

  /// Callback called each time a message is received on all topics
  void synchronized_callback(const sensor_msgs::msg::Image::SharedPtr msg_img_color,
                             const sensor_msgs::msg::Image::SharedPtr msg_img_depth,
                             const sensor_msgs::msg::CameraInfo::SharedPtr msg_camera_info,
                             const eye_region_msgs::msg::EyeRegionsOfInterestStamped::SharedPtr msg_eyes);

  /// Localisation of the 3D pupil centre
  cv::Point3_<double> localise_pupil(const cv::Mat_<uint8_t> &img_mono,
                                     const cv::Mat_<uint16_t> &img_depth,
                                     const std::array<double, 9> &camera_matrix,
                                     const cv::Rect_<uint16_t> &eye_roi,
                                     const uint8_t eye_index = EYE_LEFT);

  /// Computation of the objective function based on gradients
  cv::Mat_<double> compute_objective_function(const cv::Mat_<uint8_t> &img_eye_mono, const uint8_t eye_index = EYE_LEFT);

  /// Create a 3D cloud point at a specific pixel from depth map
  bool create_cloud_point(
      const cv::Mat_<uint16_t> &img_depth,
      cv::Point3_<double> &output,
      const std::array<double, 9> &camera_matrix,
      const cv::Point_<int16_t> &pixel,
      const double depth_scale = 0.001);
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
  pub_pupil_centres_ = this->create_publisher<gaze_msgs::msg::PupilCentresStamped>("pupil_centres", qos);

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

  // Create output msg
  gaze_msgs::msg::PupilCentresStamped pupil_centres;
  pupil_centres.header = msg_img_color->header;

  // Convert color and depth images to CV
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

  // Convert color to mono
  cv::Mat_<uint8_t> img_mono;
  if (BGR_TO_MONO_WEIGHTS[2] == 1.0)
  {
    // Pure red
    cv::extractChannel(img_color->image, img_mono, 2);
  }
  else if (BGR_TO_MONO_WEIGHTS[1] == 1.0)
  {
    // Pure green
    cv::extractChannel(img_color->image, img_mono, 1);
  }
  else if (BGR_TO_MONO_WEIGHTS[0] == 1.0)
  {
    // Pure blue
    cv::extractChannel(img_color->image, img_mono, 0);
  }
  else
  {
    // Combination of channels
    cv::transform(img_color->image, img_mono, cv::Vec<double, 3>(BGR_TO_MONO_WEIGHTS[0], BGR_TO_MONO_WEIGHTS[1], BGR_TO_MONO_WEIGHTS[2]));
  }

  // Process both eyes
  for (uint8_t eye_index = 0; eye_index < 2; eye_index++)
  {
    // Convert eye ROI to CV
    cv::Rect eye_roi = sensor_msgs::roi::to_rect(msg_eyes->eyes.rois[eye_index], img_mono.size(), ROI_PADDING_HORZIZONTAL, ROI_PADDING_VERTICAL);

    // Determine 3D location of the pupil
    cv::Point3_<double> pupil_centre = localise_pupil(img_mono, img_depth->image, msg_camera_info->k, eye_roi, eye_index);

    // Convert to msg
    pupil_centres.pupils.centres[eye_index] = cv::to_msg(pupil_centre);
  }

  // Wait to render images, if visualisation is enabled
  if (this->get_parameter("visualise").get_value<bool>())
  {
    cv::waitKey(CV_WAITKEY_DELAY);
  }

  // Publish the pupil centres
  pub_pupil_centres_->publish(pupil_centres);
}

cv::Point3_<double> PupilCentre::localise_pupil(const cv::Mat_<uint8_t> &img_mono,
                                                const cv::Mat_<uint16_t> &img_depth,
                                                const std::array<double, 9> &camera_matrix,
                                                const cv::Rect_<uint16_t> &eye_roi,
                                                const uint8_t eye_index)
{
  // Get the eye image
  cv::Mat_<uint8_t> img_eye_mono = img_mono(eye_roi);

  // Compute objective function
  cv::Mat_<double> objective_function = compute_objective_function(img_eye_mono, eye_index);

  // Find pupil centre as maximum argument of the objective function
  cv::Point pupil_centre;
  cv::minMaxLoc(objective_function, NULL, NULL, NULL, &pupil_centre);

  // Extract a small neighbourhood around the detected pupil and normalize it to serve as weights
  cv::Mat_<double> objective_function_pupil_neighbourhood = extract_neighbourhood(objective_function, pupil_centre, PUPIL_3D_NEIGHBOURHOOD_SIZE);
  objective_function_pupil_neighbourhood /= cv::sum(objective_function_pupil_neighbourhood)[0];

  // Create output
  cv::Point3_<double> pupil_centre_3d = cv::Vec<double, 3>(0.0, 0.0, 0.0);
  // Keep track of residual weights that failed to be applied
  double residuals = 0.0;
  // Iterate over the neighbourhood and compute 3D position for each pixel
  for (uint16_t r = 0; r < objective_function_pupil_neighbourhood.rows; ++r)
  {
    const double *objective_function_pupil_neighbourhood_row_ptr = objective_function_pupil_neighbourhood.ptr<double>(r);

    for (uint16_t c = 0; c < objective_function_pupil_neighbourhood.cols; ++c)
    {
      cv::Point3_<double> sample;
      cv::Point_<int16_t> pixel = cv::Point_<int16_t>(eye_roi.x + pupil_centre.x - PUPIL_3D_NEIGHBOURHOOD_SIZE / 2 + c,
                                                      eye_roi.y + pupil_centre.y - PUPIL_3D_NEIGHBOURHOOD_SIZE / 2 + r);
      if (create_cloud_point(img_depth, sample, camera_matrix, pixel))
      {
        pupil_centre_3d += objective_function_pupil_neighbourhood_row_ptr[c] * sample;
      }
      else
      {
        residuals += objective_function_pupil_neighbourhood_row_ptr[c];
      }
    }
  }
  if (pupil_centre_3d.x == 0.0 && pupil_centre_3d.y == 0.0 && pupil_centre_3d.z == 0.0)
  {
    RCLCPP_ERROR(this->get_logger(), "Cannot obtain any depth information for the detected pupil centre");
    pupil_centre_3d = cv::Vec<double, 3>(0.0, 0.0, 0.0);
  }
  else
  {
    // Normalize with residuals, if some pixels were invalid
    pupil_centre_3d *= 1.0 / (1.0 - residuals);
  }

  // Visualise if enabled
  if (this->get_parameter("visualise").get_value<bool>())
  {
    std::string eye_side;
    if (eye_index == EYE_LEFT)
    {
      eye_side = "L_";
    }
    else
    {
      eye_side = "R_";
    }

    cv::img_show_pupil(eye_side + "pupil_centre", img_eye_mono, pupil_centre);
  }

  return pupil_centre_3d;
}

cv::Mat_<double> PupilCentre::compute_objective_function(const cv::Mat_<uint8_t> &img_eye_mono,
                                                         const uint8_t eye_index)
{
  // Create output image
  cv::Mat_<double> objective_function = cv::Mat_<double>::zeros(img_eye_mono.size());

  // Equalize
  cv::Mat img_eye_equalized;
  cv::equalizeHist(img_eye_mono, img_eye_equalized);

  // Filter
  cv::Mat1b img_eye_filtered;
  cv::bilateralFilter(img_eye_equalized, img_eye_filtered, -1, BILATERAL_FILTER_SIGMA_COLOR, BILATERAL_FILTER_SIGMA_SPACE);

  // Compute gradients
  cv::Mat_<double> gradients_x, gradients_y;
  if (GRADIENTS_SCHARR_INSTEAD_SOBEL)
  {
    cv::Scharr(img_eye_filtered, gradients_x, CV_64F, 1, 0, 1.0, 0.0);
    cv::Scharr(img_eye_filtered, gradients_y, CV_64F, 0, 1, 1.0, 0.0);
  }
  else
  {
    cv::Sobel(img_eye_filtered, gradients_x, CV_64F, 1, 0, 1.0, 0.0);
    cv::Sobel(img_eye_filtered, gradients_y, CV_64F, 0, 1, 1.0, 0.0);
  }

  // Compute gradient magnitudes
  cv::Mat_<double> gradient_magnitudes;
  cv::magnitude(gradients_x, gradients_y, gradient_magnitudes);

  // Determine minimum magnitude that pixel gradients must have in order to be considered in computations, if enabled
  double min_allowed_magnitude;
  if (CONSIDER_ONLY_SIGNIFICANT_GRADIENTS)
  {
    double max_gradient_magnitude;
    cv::minMaxLoc(gradient_magnitudes, NULL, &max_gradient_magnitude, NULL, NULL);
    min_allowed_magnitude = MINIMUM_GRADIENT_SIGNIFICANCE * max_gradient_magnitude;
  }

// Parallelize computations, if OpenMP is available
#pragma omp parallel for
  // Compute objective for all possible centres
  for (uint16_t r = 0; r < img_eye_mono.rows; ++r)
  {
    const uint8_t *img_eye_mono_row_ptr = img_eye_mono.ptr<uint8_t>(r);
    double *objective_function_row_ptr = objective_function.ptr<double>(r);

    for (uint16_t c = 0; c < img_eye_mono.cols; ++c)
    {
      const cv::Vec<int16_t, 2> possible_centre = cv::Vec<int16_t, 2>(c, r);

      // For each possible centre, consider gradients of all other pixels
      for (uint16_t g_r = 0; g_r < img_eye_mono.rows; ++g_r)
      {
        const double *gradient_magnitudes_row_ptr = gradient_magnitudes.ptr<double>(g_r);
        const double *gradients_x_row_ptr = gradients_x.ptr<double>(g_r);
        const double *gradients_y_row_ptr = gradients_y.ptr<double>(g_r);

        for (uint16_t g_c = 0; g_c < img_eye_mono.cols; ++g_c)
        {
          const cv::Vec<int16_t, 2> gradient_pixel = cv::Vec<int16_t, 2>(g_c, g_r);

          // Skip the same pixel
          if (possible_centre == gradient_pixel)
          {
            continue;
          }

          // Make sure the gradient has satisfactory magnitude, if enabled
          if (CONSIDER_ONLY_SIGNIFICANT_GRADIENTS)
          {
            if (gradient_magnitudes_row_ptr[g_c] < min_allowed_magnitude)
            {
              continue;
            }
          }

          // Get normalized gradient vector
          cv::Vec<double, 2> gradient = cv::Vec<double, 2>(gradients_x_row_ptr[g_c] / gradient_magnitudes_row_ptr[g_c], gradients_y_row_ptr[g_c] / gradient_magnitudes_row_ptr[g_c]);

          // Compute and normalize displacement vector from the possible centre to the compared gradient pixel
          cv::Vec<double, 2> displacement = gradient_pixel - possible_centre;
          displacement = cv::normalize(displacement);

          // Compute objective with respect to the gradient as dot product between displacement vector and gradient vector of the pixel
          double objective_wrt_gradient = displacement.dot(gradient);
          // Allow only positive values
          objective_wrt_gradient = std::max<double>(objective_wrt_gradient, 0.0);
          // Square the objective
          objective_wrt_gradient *= objective_wrt_gradient;

          // Sum objective with respect to all gradients
          objective_function_row_ptr[c] += objective_wrt_gradient;
        }
      }

      // Apply weight according to the inverted intensity of pixel considered to be pupil centre, if enabled
      if (APPLY_INTENSITY_WEIGHTS)
      {
        objective_function_row_ptr[c] *= std::sqrt(255 - img_eye_mono_row_ptr[c]);
      }
    }
  }

  // Remove local maxima of the objective function that are connected to image boundaries, if enabled
  if (POSTPROCESS_OBJECTIVE_FUNCTION)
  {
    // Convert the objective function to normalized 8-bit image
    cv::Mat_<uint8_t> objective_function_8bit;
    double min_val, max_val;
    cv::minMaxLoc(objective_function, &min_val, &max_val, NULL, NULL);
    if (min_val != max_val)
    {
      objective_function.convertTo(objective_function_8bit, CV_8UC1, 255.0 / (max_val - min_val), -255.0 * min_val / (max_val - min_val));
    }

    // Apply a threshold and detect contours
    cv::Mat_<uint8_t> objective_function_binary;
    cv::threshold(objective_function_8bit, objective_function_binary, POSTPROCESS_THRESHOLD, 255, CV_THRESH_BINARY);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(objective_function_binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    // Image that contains mask of the boundary regions that need to be removed
    cv::Mat_<uint8_t> boundary_contours = cv::Mat::zeros(objective_function.size(), CV_8UC1);
    // Flag that makes sure at least one viabla countour is present in the image
    bool found_viable_contour = false;
    if (contours.size() > 1)
    {
      for (size_t i = 0; i < contours.size(); i++)
      {
        if (cv::is_contour_touching_border(contours[i], objective_function.size()))
        {
          cv::drawContours(boundary_contours, contours, i, cv::Scalar(255), cv::FILLED);
        }
        else
        {
          found_viable_contour = true;
        }
      }

      // Remove all boundary contours from the objective function, if at least one viable contour was found
      if (found_viable_contour)
      {
        for (uint16_t r = 0; r < objective_function.rows; ++r)
        {
          const uint8_t *boundary_contours_row_ptr = boundary_contours.ptr<uint8_t>(r);
          double *objective_function_row_ptr = objective_function.ptr<double>(r);
          for (uint16_t c = 0; c < objective_function.cols; ++c)
          {
            if (boundary_contours_row_ptr[c])
            {
              objective_function_row_ptr[c] = 0.0;
            }
          }
        }
      }
    }
  }

  // Visualise if enabled
  if (this->get_parameter("visualise").get_value<bool>())
  {
    std::string eye_side;
    if (eye_index == EYE_LEFT)
    {
      eye_side = "L_";
    }
    else
    {
      eye_side = "R_";
    }

    // cv::img_show(eye_side + "img_eye_mono", img_eye_mono);
    // cv::img_show(eye_side + "img_eye_equalized", img_eye_equalized);
    // cv::img_show(eye_side + "img_eye_filtered", img_eye_filtered);
    // cv::img_show(eye_side + "gradient_magnitudes", gradient_magnitudes);
    cv::img_show(eye_side + "objective_function", objective_function);
  }

  return objective_function;
}

bool PupilCentre::create_cloud_point(
    const cv::Mat_<uint16_t> &img_depth,
    cv::Point3_<double> &output,
    const std::array<double, 9> &camera_matrix,
    const cv::Point_<int16_t> &pixel,
    const double depth_scale)
{
  if (pixel.x >= img_depth.cols || pixel.y >= img_depth.rows)
  {
    RCLCPP_ERROR(this->get_logger(), "create_cloud_point() - Pixel out of bounds");
    return false;
  }

  uint16_t z = img_depth.at<uint16_t>(pixel);

  if (z == 0)
  {
    return false;
  }
  else
  {
    output.z = z * depth_scale;
    output.x = output.z * ((pixel.x - camera_matrix[2]) / camera_matrix[0]);
    output.y = output.z * ((pixel.y - camera_matrix[5]) / camera_matrix[4]);
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
