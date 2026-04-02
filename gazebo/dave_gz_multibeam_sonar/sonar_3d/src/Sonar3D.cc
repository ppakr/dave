#include <marine_acoustic_msgs/msg/projected_sonar_image.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/header.hpp>

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

constexpr int NUM_SONARS = 64;

// Pitch angle for sensor i, matching generate_3d_sonar.py:
//   pitch_i = radians(-20.0 + 1.10 + i * 0.60)
// Sensor 0 is most downward (-18.9 deg), sensor 63 is most upward (+18.9 deg).
static float sonar_pitch_rad(int i)
{
  constexpr double deg2rad = M_PI / 180.0;
  return static_cast<float>((-20.0 + 1.10 + i * 0.60) * deg2rad);
}

struct Point3f
{
  float x, y, z;
};

class SonarAggregator : public rclcpp::Node
{
public:
  SonarAggregator() : Node("sonar_aggregator"), received_count_(0)
  {
    received_.fill(false);

    pc_pub_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("/sensor/sonar_3d/pointcloud", 10);

    for (int i = 0; i < NUM_SONARS; ++i)
    {
      std::string topic =
        "/sensor/sonar_3d/beam_" + std::to_string(i) + "/sonar_image_raw_" + std::to_string(i);

      auto sub = this->create_subscription<marine_acoustic_msgs::msg::ProjectedSonarImage>(
        topic, 10, [this, i](const marine_acoustic_msgs::msg::ProjectedSonarImage::SharedPtr msg)
        { this->sonar_callback(msg, i); });
      subscriptions_.push_back(sub);

      RCLCPP_INFO(this->get_logger(), "Subscribed to: %s", topic.c_str());
    }

    RCLCPP_INFO(
      this->get_logger(),
      "SonarAggregator node started. Publishing to: /sensor/sonar_3d/pointcloud");
  }

private:
  // Returns the byte size of one element for a given dtype code.
  // dtype codes match marine_acoustic_msgs convention (same as Python numpy dtype_map).
  static size_t elem_size_for_dtype(uint32_t dtype)
  {
    switch (dtype)
    {
      case 0:
      case 1:
        return 1;  // uint8, int8
      case 2:
      case 3:
        return 2;  // uint16, int16
      case 4:
      case 5:
        return 4;  // uint32, int32
      case 6:
      case 7:
        return 8;  // uint64, int64
      case 8:
        return 4;  // float32
      case 9:
        return 8;  // float64
      default:
        return 0;
    }
  }

  // Extracts the numeric value of one element from raw byte data.
  static double extract_value(
    const std::vector<uint8_t> & data, size_t offset, uint32_t dtype, bool is_bigendian)
  {
    if (offset >= data.size())
    {
      return 0.0;
    }
    switch (dtype)
    {
      case 0:
        return static_cast<double>(data[offset]);
      case 1:
        return static_cast<double>(static_cast<int8_t>(data[offset]));
      case 2:
      {
        uint16_t v;
        std::memcpy(&v, &data[offset], 2);
        if (is_bigendian)
        {
          v = __builtin_bswap16(v);
        }
        return static_cast<double>(v);
      }
      case 3:
      {
        int16_t v;
        std::memcpy(&v, &data[offset], 2);
        if (is_bigendian)
        {
          v = static_cast<int16_t>(__builtin_bswap16(static_cast<uint16_t>(v)));
        }
        return static_cast<double>(v);
      }
      case 4:
      {
        uint32_t v;
        std::memcpy(&v, &data[offset], 4);
        if (is_bigendian)
        {
          v = __builtin_bswap32(v);
        }
        return static_cast<double>(v);
      }
      case 5:
      {
        int32_t v;
        std::memcpy(&v, &data[offset], 4);
        if (is_bigendian)
        {
          v = static_cast<int32_t>(__builtin_bswap32(static_cast<uint32_t>(v)));
        }
        return static_cast<double>(v);
      }
      case 6:
      {
        uint64_t v;
        std::memcpy(&v, &data[offset], 8);
        if (is_bigendian)
        {
          v = __builtin_bswap64(v);
        }
        return static_cast<double>(v);
      }
      case 7:
      {
        int64_t v;
        std::memcpy(&v, &data[offset], 8);
        if (is_bigendian)
        {
          v = static_cast<int64_t>(__builtin_bswap64(static_cast<uint64_t>(v)));
        }
        return static_cast<double>(v);
      }
      case 8:
      {
        float v;
        std::memcpy(&v, &data[offset], 4);
        if (is_bigendian)
        {
          uint32_t tmp;
          std::memcpy(&tmp, &v, 4);
          tmp = __builtin_bswap32(tmp);
          std::memcpy(&v, &tmp, 4);
        }
        return static_cast<double>(v);
      }
      case 9:
      {
        double v;
        std::memcpy(&v, &data[offset], 8);
        if (is_bigendian)
        {
          uint64_t tmp;
          std::memcpy(&tmp, &v, 8);
          tmp = __builtin_bswap64(tmp);
          std::memcpy(&v, &tmp, 8);
        }
        return v;
      }
      default:
        return 0.0;
    }
  }

  static sensor_msgs::msg::PointCloud2 create_point_cloud(
    const std_msgs::msg::Header & header, const std::vector<Point3f> & points)
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = header;
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(points.size());
    cloud.is_dense = false;
    cloud.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

    for (const auto & pt : points)
    {
      *iter_x = pt.x;
      *iter_y = pt.y;
      *iter_z = pt.z;
      ++iter_x;
      ++iter_y;
      ++iter_z;
    }
    return cloud;
  }

  void sonar_callback(
    const marine_acoustic_msgs::msg::ProjectedSonarImage::SharedPtr msg, int sonar_idx)
  {
    const auto & img = msg->image;
    uint32_t beam_count = img.beam_count;
    auto range_count = static_cast<uint32_t>(msg->ranges.size());

    if (beam_count == 0 || range_count == 0)
    {
      return;
    }

    size_t esz = elem_size_for_dtype(img.dtype);
    if (esz == 0)
    {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000, "Sonar %d: unknown dtype %u", sonar_idx,
        img.dtype);
      return;
    }

    size_t expected_bytes = static_cast<size_t>(range_count) * beam_count * esz;
    if (img.data.size() < expected_bytes)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Sonar %d: data size (%zu) < expected (%zu), skipping", sonar_idx, img.data.size(),
        expected_bytes);
      return;
    }

    // Data layout: rows=ranges, cols=beams (row-major).
    // Find the range index of maximum intensity for each beam (column).
    std::vector<uint32_t> max_indices(beam_count, 0);
    std::vector<float> max_values(beam_count, 0.0f);
    float threshold{60.0f};  // set threshold to filter out low intensity values, this can be tuned
                             // based on the expected intensity range
    for (uint32_t beam = 0; beam < beam_count; ++beam)
    {
      double max_val = -1.0;
      uint32_t max_idx = 0;
      for (uint32_t range = 0; range < range_count; ++range)
      {
        size_t offset = (static_cast<size_t>(range) * beam_count + beam) * esz;
        double val = extract_value(img.data, offset, img.dtype, img.is_bigendian);
        if (val > max_val)
        {
          max_val = val;
          max_idx = range;
        }
      }
      // std::cout << "Sonar " << sonar_idx << ", Beam " << beam << ": max intensity at range index
      // "
      //           << max_idx << " with value " << max_val << std::endl;
      max_indices[beam] = max_idx;
      max_values[beam] = static_cast<float>(max_val);
    }

    // Project each beam's max-intensity sample into 3-D space.
    // beam_directions from the plugin are in the sensor's LOCAL frame (z=0 always).
    // Rotate into the common frame by applying the sensor's pitch about the Y-axis:
    //   x' = cos(pitch) * dir.x
    //   y' = dir.y
    //   z' = sin(pitch) * dir.x
    const float pitch = sonar_pitch_rad(sonar_idx);
    const float cos_p = std::cos(pitch);
    const float sin_p = std::sin(pitch);

    std::vector<Point3f> points;
    points.reserve(beam_count);
    for (uint32_t beam = 0; beam < beam_count; ++beam)
    {
      if (max_values[beam] < threshold)
      {
        continue;  // skip low intensity beams
      }
      float r = msg->ranges[max_indices[beam]];
      const auto & dir = msg->beam_directions[beam];
      const float lx = static_cast<float>(dir.x);
      const float ly = static_cast<float>(dir.y);
      points.push_back({r * cos_p * lx, r * ly, r * sin_p * lx});
    }

    // if (points.empty())
    // {
    //   return;
    // }

    // Store this sonar's points and track receipt.
    if (!received_[sonar_idx])
    {
      received_[sonar_idx] = true;
      sonar_points_[sonar_idx] = std::move(points);
      latest_header_ = msg->header;
      ++received_count_;
    }

    // Once all sonars have reported, combine and publish.
    if (received_count_ == NUM_SONARS)
    {
      std::vector<Point3f> combined;
      for (auto & pts : sonar_points_)
      {
        combined.insert(combined.end(), pts.begin(), pts.end());
        pts.clear();
      }
      // Points have been rotated into the model's base_link frame.
      latest_header_.frame_id = "3d_sonar/base_link";
      pc_pub_->publish(create_point_cloud(latest_header_, combined));
      received_.fill(false);
      received_count_ = 0;
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;
  std::vector<rclcpp::Subscription<marine_acoustic_msgs::msg::ProjectedSonarImage>::SharedPtr>
    subscriptions_;

  std::array<std::vector<Point3f>, NUM_SONARS> sonar_points_;
  std::array<bool, NUM_SONARS> received_;
  size_t received_count_;
  std_msgs::msg::Header latest_header_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SonarAggregator>());
  rclcpp::shutdown();
  return 0;
}