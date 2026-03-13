#include "sonar_3d/Sonar3D.hh"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <gz/common/Console.hh>
#include <gz/common/Profiler.hh>
#include <gz/rendering/Camera.hh>
#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/Scene.hh>

namespace gz
{
namespace sensors
{
class Sonar3D::Implementation
{
public:
  // ROS 2 setup
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;

  // Rendering
  rendering::GpuRaysPtr gpuRays;
  gz::common::ConnectionPtr connection;
};
Sonar3D::Sonar3D() : RenderingSensor(), dataPtr(std::make_unique<Implementation>()) {}

Sonar3D::~Sonar3D() {}

void Sonar3D::Load(const sdf::Sensor & _sdf)
{
  RenderingSensor::Load(_sdf);

  // 1. Initialize ROS 2 Node
  if (!rclcpp::ok())
  {
    rclcpp::init(0, nullptr);
  }
  this->dataPtr->ros_node_ = rclcpp::Node::make_shared(this->Name() + "_ros");
  this->dataPtr->pc_pub_ =
    this->dataPtr->ros_node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      this->Topic() + "/pointcloud", 10);

  // 2. Load 3D Parameters from SDF (with defaults)
  this->dataPtr->h_beams = 256;
  this->dataPtr->v_beams = 128;  // 3D Expansion
  this->dataPtr->h_fov = 1.57;   // 90 degrees
  this->dataPtr->v_fov = 1.0;    // ~57 degrees
  this->dataPtr->max_distance = 50.0;
  this->dataPtr->n_freq = 512;
  this->dataPtr->ray_skips = 1;

  // 3. Find and configure the internal GpuRays sensor
  this->dataPtr->gpuRays = std::dynamic_pointer_cast<rendering::GpuRays>(this->Sensor());
  if (this->dataPtr->gpuRays)
  {
    this->dataPtr->connection = this->dataPtr->gpuRays->ConnectNewGpuRaysFrame(
      std::bind(
        &Implementation::OnNewFrame, this->dataPtr.get(), std::placeholders::_1,
        std::placeholders::_2, std::placeholders::_3, std::placeholders::_4,
        std::placeholders::_5));
  }
  else
  {
    gzerr << "Sonar3D requires a GpuRays sensor attached!" << std::endl;
  }
}
}  // namespace sensors
}  // namespace gz

// Register the plugin with Gazebo
// GZ_ADD_PLUGIN(Sonar3D, gz::sensors::Sensor, Sonar3D)
// GZ_ADD_PLUGIN_ALIAS(Sonar3D, "gz::sensors::Sonar3D")

// //////////////////////////////////////////////////
// class Sonar3D::Implementation
// {
// public:
//   // ROS 2 setup
//   rclcpp::Node::SharedPtr ros_node_;
//   rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;

//   // Rendering
//   rendering::GpuRaysPtr gpuRays;
//   gz::common::ConnectionPtr connection;

//   // 3D Sonar Parameters
//   int h_beams, v_beams, ray_skips, n_freq;
//   double h_fov, v_fov, max_distance, sound_speed, sonar_freq, bandwidth, source_level;

//   // Buffers
//   std::mutex mutex;
//   cv::Mat depth_image, normal_image, reflectivity_image;
//   sensor_msgs::msg::PointCloud2 pc_msg;
//   bool new_data_available = false;

//   // Callbacks and Processors
//   void OnNewFrame(const float *_data, unsigned int _width, unsigned int _height,
//                   unsigned int _channels, const std::string &_format);
//   void ComputeSonar3D();
//   void FillPointCloud3DMsg(const NpsGazeboSonar::CArray3D& acoustic_volume);
// };

// //////////////////////////////////////////////////
// Sonar3D::Sonar3D() : RenderingSensor(), dataPtr(std::make_unique<Implementation>())
// {
// }

// //////////////////////////////////////////////////
// Sonar3D::~Sonar3D()
// {
// }

// //////////////////////////////////////////////////
// void Sonar3D::Load(const sdf::Sensor &_sdf)
// {
//   RenderingSensor::Load(_sdf);

//   // 1. Initialize ROS 2 Node
//   if (!rclcpp::ok()) {
//     rclcpp::init(0, nullptr);
//   }
//   this->dataPtr->ros_node_ = rclcpp::Node::make_shared(this->Name() + "_ros");
//   this->dataPtr->pc_pub_ =
//   this->dataPtr->ros_node_->create_publisher<sensor_msgs::msg::PointCloud2>(
//     this->Topic() + "/pointcloud", 10);

//   // 2. Load 3D Parameters from SDF (with defaults)
//   this->dataPtr->h_beams = 256;
//   this->dataPtr->v_beams = 128; // 3D Expansion
//   this->dataPtr->h_fov = 1.57;  // 90 degrees
//   this->dataPtr->v_fov = 1.0;   // ~57 degrees
//   this->dataPtr->max_distance = 50.0;
//   this->dataPtr->n_freq = 512;
//   this->dataPtr->ray_skips = 1;

//   // 3. Find and configure the internal GpuRays sensor
//   this->dataPtr->gpuRays = std::dynamic_pointer_cast<rendering::GpuRays>(this->Sensor());
//   if (this->dataPtr->gpuRays) {
//     this->dataPtr->connection = this->dataPtr->gpuRays->ConnectNewGpuRaysFrame(
//       std::bind(&Implementation::OnNewFrame, this->dataPtr.get(),
//                 std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
//                 std::placeholders::_4, std::placeholders::_5));
//   } else {
//     gzerr << "Sonar3D requires a GpuRays sensor attached!" << std::endl;
//   }
// }

// //////////////////////////////////////////////////
// bool Sonar3D::Update(const std::chrono::steady_clock::duration &_now)
// {
//   GZ_PROFILE("Sonar3D::Update");
//   return RenderingSensor::Update(_now);
// }

// //////////////////////////////////////////////////
// void Sonar3D::PostUpdate(const UpdateInfo &_info)
// {
//   std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
//   if (this->dataPtr->new_data_available) {
//     // Set timestamp and publish the 3D Point Cloud
//     auto now = this->dataPtr->ros_node_->now();
//     this->dataPtr->pc_msg.header.stamp.sec = now.seconds();
//     this->dataPtr->pc_msg.header.stamp.nanosec = now.nanoseconds();
//     this->dataPtr->pc_pub_->publish(this->dataPtr->pc_msg);
//     this->dataPtr->new_data_available = false;
//   }
// }

// //////////////////////////////////////////////////
// void Sonar3D::Implementation::OnNewFrame(const float *_data, unsigned int _width,
//                                          unsigned int _height, unsigned int _channels,
//                                          const std::string &_format)
// {
//   std::lock_guard<std::mutex> lock(this->mutex);

//   // 1. Copy raw depth data to OpenCV Mat
//   this->depth_image = cv::Mat(_height, _width, CV_32FC1);
//   memcpy(this->depth_image.data, _data, _width * _height * sizeof(float));

//   // Note: For a real implementation, you would calculate normals here using cv::Sobel
//   // this->normal_image = ComputeNormalImage(this->depth_image);
//   this->normal_image = cv::Mat::zeros(_height, _width, CV_32FC3);
//   this->reflectivity_image = cv::Mat::ones(_height, _width, CV_32FC1); // Default reflectivity

//   // 2. Run CUDA Physics
//   this->ComputeSonar3D();
// }

// //////////////////////////////////////////////////
// void Sonar3D::Implementation::ComputeSonar3D()
// {
//   // Allocate dummy pointers for things not fully set up in this skeleton
//   float *ray_elevations = new float[this->v_beams];
//   float **beam_corrector = nullptr; // Ignore culling for basic 3D

//   // Call the 3D CUDA Wrapper
//   NpsGazeboSonar::CArray3D acoustic_volume = NpsGazeboSonar::sonar_calculation_3d_wrapper(
//     this->depth_image, this->normal_image,
//     1.0, 1.0, this->h_fov, this->v_fov,
//     0.01, 0.01, 0.01, ray_elevations, 0.01,
//     1500.0, this->max_distance, 200.0, // sound speed, max dist, source level
//     this->h_beams, this->v_beams, // 3D dimensions
//     this->depth_image.cols, this->depth_image.rows, this->ray_skips,
//     900000.0, 10000.0, this->n_freq, // freq, bandwidth, bins
//     this->reflectivity_image, 0.05, nullptr, beam_corrector, 1.0, false, false
//   );

//   // Process the volume into a point cloud
//   this->FillPointCloud3DMsg(acoustic_volume);

//   delete[] ray_elevations;
// }

// //////////////////////////////////////////////////
// void Sonar3D::Implementation::FillPointCloud3DMsg(const NpsGazeboSonar::CArray3D&
// acoustic_volume)
// {
//   pcl::PointCloud<pcl::PointXYZI> pc;
//   double range_resolution = (1500.0 / (2.0 * 10000.0)); // c / (2 * bandwidth)
//   double intensity_threshold = 0.5; // Tunable parameter

//   // Iterate through the 3D volume (Vertical Beams -> Horizontal Beams -> Frequency/Range Bins)
//   for (int v = 0; v < this->v_beams; v++) {
//     double elevation = ((double)v / this->v_beams - 0.5) * this->v_fov;

//     for (int h = 0; h < this->h_beams; h++) {
//       double azimuth = ((double)h / this->h_beams - 0.5) * this->h_fov;

//       for (int r = 0; r < this->n_freq; r++) {
//         // Get magnitude of the complex return
//         double intensity = std::abs(acoustic_volume[v][h][r]);

//         // If intensity represents a solid hit
//         if (intensity > intensity_threshold) {
//           double range = r * range_resolution;

//           // Spherical to Cartesian Conversion (Forward is X in ROS standard)
//           pcl::PointXYZI pt;
//           pt.x = range * cos(azimuth) * cos(elevation);
//           pt.y = range * sin(azimuth) * cos(elevation);
//           pt.z = range * sin(elevation);
//           pt.intensity = intensity;

//           pc.push_back(pt);
//         }
//       }
//     }
//   }

//   // Convert PCL back to ROS 2 message
//   pcl::toROSMsg(pc, this->pc_msg);
//   this->pc_msg.header.frame_id = "sonar_3d_link";
//   this->new_data_available = true;
// }
