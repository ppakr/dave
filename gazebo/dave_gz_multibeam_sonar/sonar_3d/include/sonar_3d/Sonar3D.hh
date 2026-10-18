#ifndef SONAR_3D_HH_
#define SONAR_3D_HH_

#include <chrono>
#include <complex>
#include <memory>
#include <valarray>

#include <gz/msgs/pointcloud_packed.pb.h>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector2.hh>
#include <gz/math/Vector3.hh>
#include <gz/sensors/EnvironmentalData.hh>
#include <gz/sensors/RenderingSensor.hh>
#include <gz/transport/Node.hh>

#include <marine_acoustic_msgs/msg/projected_sonar_image.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "sonar_calculation_cuda.cuh"

namespace gz
{
typedef std::complex<float> Complex;
typedef std::valarray<Complex> CArray;
typedef std::valarray<CArray> CArray2D;

typedef std::valarray<float> Array;
typedef std::valarray<Array> Array2D;

namespace sensors
{

class Sonar3D : public RenderingSensor
{
public:
  /// \brief Constructor
  Sonar3D();

  /// \brief Destructor
  virtual ~Sonar3D();

  /// \brief Load the sensor from SDF parameters
  virtual void Load(const sdf::Sensor & _sdf) override;

  /// \brief Update the sensor (triggers rendering)
  // virtual bool Update(const std::chrono::steady_clock::duration & _now) override;

  /// \brief PostUpdate (publishes data to ROS 2)
  // virtual void PostUpdate(const UpdateInfo & _info) override;

private:
  /// \brief Private implementation class
  class Implementation;
  std::unique_ptr<Implementation> dataPtr;
};

}  // namespace sensors
}  // namespace gz
#endif