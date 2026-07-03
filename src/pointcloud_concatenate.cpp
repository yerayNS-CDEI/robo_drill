#include "robo_drill/pointcloud_concatenate.hpp"

#include <tf2/exceptions.h>
// Constructor
PointcloudConcatenate::PointcloudConcatenate() : Node("pointcloud_concatenate")
{
  // Initialise variables / parameters to class variables
  handleParams();

  // Initialization tf2 listener
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Subscriptions run in a reentrant group so they keep draining concurrently
  // with (and with each other while) the update timer is busy in TF.
  sub_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Initialise publishers and subscribers
  // Use SensorDataQoS for input subscriptions (BEST_EFFORT for sensor compatibility)
  auto sub_qos = rclcpp::SensorDataQoS().keep_last(1);
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = sub_cb_group_;
  sub_cloud_in1_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(cloud_in1_topic_, sub_qos, std::bind(&PointcloudConcatenate::subCallbackCloudIn1, this, std::placeholders::_1), sub_options);
  sub_cloud_in2_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(cloud_in2_topic_, sub_qos, std::bind(&PointcloudConcatenate::subCallbackCloudIn2, this, std::placeholders::_1), sub_options);
  sub_cloud_in3_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(cloud_in3_topic_, sub_qos, std::bind(&PointcloudConcatenate::subCallbackCloudIn3, this, std::placeholders::_1), sub_options);
  sub_cloud_in4_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(cloud_in4_topic_, sub_qos, std::bind(&PointcloudConcatenate::subCallbackCloudIn4, this, std::placeholders::_1), sub_options);
  // Use RELIABLE QoS for output (RViz2 compatibility)
  auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
  pub_cloud_out_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(cloud_out_topic_, pub_qos);

  // Drive update() from a wall timer in its own callback group.
  const double hz = param_hz_ > 0.0 ? param_hz_ : 10.0;
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / hz)),
    std::bind(&PointcloudConcatenate::update, this),
    timer_cb_group_);
}

// Destructor
PointcloudConcatenate::~PointcloudConcatenate()
{
  // Free up allocated memory
  RCLCPP_INFO(this->get_logger(), "Destructing PointcloudConcatenate...");
  // delete pointer_name;
}

void PointcloudConcatenate::subCallbackCloudIn1(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(cloud_mutex_);
  cloud_in1_ptr_ = msg;
  cloud_in1_arrival_ = this->now();
}

void PointcloudConcatenate::subCallbackCloudIn2(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(cloud_mutex_);
  cloud_in2_ptr_ = msg;
  cloud_in2_arrival_ = this->now();
}

void PointcloudConcatenate::subCallbackCloudIn3(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(cloud_mutex_);
  cloud_in3_ptr_ = msg;
  cloud_in3_arrival_ = this->now();
}

void PointcloudConcatenate::subCallbackCloudIn4(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(cloud_mutex_);
  cloud_in4_ptr_ = msg;
  cloud_in4_arrival_ = this->now();
}

void PointcloudConcatenate::handleParams()
{
  this->declare_parameter("target_frame", "base_link");
  this->declare_parameter("clouds", 2);
  this->declare_parameter("hz", 10.0);
  this->declare_parameter("cloud_in1_topic", "/cloud_in1");
  this->declare_parameter("cloud_in2_topic", "/cloud_in2");
  this->declare_parameter("cloud_in3_topic", "/cloud_in3");
  this->declare_parameter("cloud_in4_topic", "/cloud_in4");
  this->declare_parameter("cloud_out_topic", "/cloud_out");
  this->declare_parameter("transform_timeout_sec", 0.05);
  this->declare_parameter("fallback_to_latest_transform", true);
  this->declare_parameter("max_cloud_age", 0.5);

  this->get_parameter("target_frame", param_frame_target_);
  this->get_parameter("clouds", param_clouds_);
  this->get_parameter("hz", param_hz_);
  this->get_parameter("cloud_in1_topic", cloud_in1_topic_);
  this->get_parameter("cloud_in2_topic", cloud_in2_topic_);
  this->get_parameter("cloud_in3_topic", cloud_in3_topic_);
  this->get_parameter("cloud_in4_topic", cloud_in4_topic_);
  this->get_parameter("cloud_out_topic", cloud_out_topic_);
  this->get_parameter("transform_timeout_sec", transform_timeout_sec_);
  this->get_parameter("fallback_to_latest_transform", fallback_to_latest_transform_);
  this->get_parameter("max_cloud_age", max_cloud_age_);

  RCLCPP_INFO(this->get_logger(), "Parameters loaded.");
}

double PointcloudConcatenate::getHz()
{
  return param_hz_;
}

void PointcloudConcatenate::normalizeToXYZI(
  const sensor_msgs::msg::PointCloud2 & cloud_in,
  sensor_msgs::msg::PointCloud2 & cloud_out)
{
  const size_t n = static_cast<size_t>(cloud_in.width) * cloud_in.height;

  // Build a fresh XYZI cloud (point_step = 16). Preserve header/endianness and
  // collapse to an unorganized cloud (height = 1) since the concatenator emits
  // unorganized clouds anyway.
  cloud_out = sensor_msgs::msg::PointCloud2();
  cloud_out.header = cloud_in.header;
  cloud_out.height = 1;
  cloud_out.width = static_cast<uint32_t>(n);
  cloud_out.is_bigendian = cloud_in.is_bigendian;
  cloud_out.is_dense = cloud_in.is_dense;

  sensor_msgs::PointCloud2Modifier mod(cloud_out);
  mod.setPointCloud2Fields(
    4,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
  mod.resize(n);

  if (n == 0)
  {
    return;
  }

  // Both ouster_ros::Point and sick_scan_xd publish intensity as FLOAT32, so a
  // float iterator reads either. Fill 0 if a source omits intensity entirely.
  bool has_intensity = false;
  for (const auto & f : cloud_in.fields)
  {
    if (f.name == "intensity")
    {
      has_intensity = true;
      break;
    }
  }

  sensor_msgs::PointCloud2ConstIterator<float> in_x(cloud_in, "x");
  sensor_msgs::PointCloud2ConstIterator<float> in_y(cloud_in, "y");
  sensor_msgs::PointCloud2ConstIterator<float> in_z(cloud_in, "z");
  sensor_msgs::PointCloud2Iterator<float> out_x(cloud_out, "x");
  sensor_msgs::PointCloud2Iterator<float> out_y(cloud_out, "y");
  sensor_msgs::PointCloud2Iterator<float> out_z(cloud_out, "z");
  sensor_msgs::PointCloud2Iterator<float> out_i(cloud_out, "intensity");

  if (has_intensity)
  {
    sensor_msgs::PointCloud2ConstIterator<float> in_i(cloud_in, "intensity");
    for (size_t k = 0; k < n;
         ++k, ++in_x, ++in_y, ++in_z, ++in_i, ++out_x, ++out_y, ++out_z, ++out_i)
    {
      *out_x = *in_x;
      *out_y = *in_y;
      *out_z = *in_z;
      *out_i = *in_i;
    }
  }
  else
  {
    for (size_t k = 0; k < n;
         ++k, ++in_x, ++in_y, ++in_z, ++out_x, ++out_y, ++out_z, ++out_i)
    {
      *out_x = *in_x;
      *out_y = *in_y;
      *out_z = *in_z;
      *out_i = 0.0f;
    }
  }
}

bool PointcloudConcatenate::transformCloudToTarget(
  const sensor_msgs::msg::PointCloud2 & cloud_in,
  sensor_msgs::msg::PointCloud2 & cloud_out,
  const char * cloud_name)
{
  if (cloud_in.header.frame_id.empty())
  {
    RCLCPP_WARN(this->get_logger(), "Skipping %s cloud because frame_id is empty.", cloud_name);
    return false;
  }

  // Normalize to XYZI up front so every source — whatever the driver layout —
  // ends up byte-compatible for concatenation, on both the pass-through and
  // transform paths.
  sensor_msgs::msg::PointCloud2 normalized;
  normalizeToXYZI(cloud_in, normalized);

  if (normalized.header.frame_id == param_frame_target_)
  {
    cloud_out = normalized;
    cloud_out.header.frame_id = param_frame_target_;
    return true;
  }

  const auto cloud_stamp = rclcpp::Time(normalized.header.stamp);
  const auto timeout = rclcpp::Duration::from_seconds(transform_timeout_sec_);
  std::string tf_error;

  if (tf_buffer_->canTransform(
      param_frame_target_, normalized.header.frame_id, cloud_stamp, timeout, &tf_error))
  {
    try
    {
      const auto transform = tf_buffer_->lookupTransform(
        param_frame_target_, normalized.header.frame_id, cloud_stamp, timeout);
      pcl_ros::transformPointCloud(param_frame_target_, transform, normalized, cloud_out);
      cloud_out.header.frame_id = param_frame_target_;
      cloud_out.header.stamp = normalized.header.stamp;
      return true;
    }
    catch (const tf2::TransformException & e)
    {
      tf_error = e.what();
    }
  }

  if (!fallback_to_latest_transform_)
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Transforming %s cloud failed (%s -> %s): %s",
      cloud_name, normalized.header.frame_id.c_str(), param_frame_target_.c_str(),
      tf_error.empty() ? "transform unavailable" : tf_error.c_str());
    return false;
  }

  try
  {
    const auto latest_transform = tf_buffer_->lookupTransform(
      param_frame_target_,
      normalized.header.frame_id,
      rclcpp::Time(0, 0, this->get_clock()->get_clock_type()),
      timeout);
    pcl_ros::transformPointCloud(param_frame_target_, latest_transform, normalized, cloud_out);
    cloud_out.header.frame_id = param_frame_target_;
    cloud_out.header.stamp = normalized.header.stamp;

    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Using latest TF for %s cloud (%s -> %s); exact transform at %.3f was unavailable.",
      cloud_name, normalized.header.frame_id.c_str(), param_frame_target_.c_str(),
      cloud_stamp.seconds());
    return true;
  }
  catch (const tf2::TransformException & e)
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Transforming %s cloud failed (%s -> %s): %s",
      cloud_name, normalized.header.frame_id.c_str(), param_frame_target_.c_str(), e.what());
    return false;
  }
}

void PointcloudConcatenate::update()
{
  // Is run periodically and handles calling the different methods
  if (pub_cloud_out_->get_subscription_count() == 0)
  {
    return;
  }

  // Snapshot the latest buffered cloud + arrival time from every source under
  // the lock, then release it before the (slow) transform/concat work.
  struct Source
  {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud;
    rclcpp::Time arrival;
    const char * name;
  };
  std::array<Source, 4> sources;
  {
    std::lock_guard<std::mutex> lk(cloud_mutex_);
    sources[0] = {cloud_in1_ptr_, cloud_in1_arrival_, "1"};
    sources[1] = {cloud_in2_ptr_, cloud_in2_arrival_, "2"};
    sources[2] = {cloud_in3_ptr_, cloud_in3_arrival_, "3"};
    sources[3] = {cloud_in4_ptr_, cloud_in4_arrival_, "4"};
  }

  const rclcpp::Time now = this->now();

  // Clear the output pointcloud
  cloud_out_ = sensor_msgs::msg::PointCloud2();
  bool has_data = false;

  for (int i = 0; i < param_clouds_ && i < static_cast<int>(sources.size()); ++i)
  {
    const Source & src = sources[i];
    if (!src.cloud)
    {
      continue;  // never received
    }

    // Staleness guard: skip a source whose newest cloud is older than
    // max_cloud_age_, so a dead/hung sensor drops out instead of freezing a
    // stale cloud into every frame. Normal jitter stays well under this.
    if (max_cloud_age_ > 0.0 && (now - src.arrival).seconds() > max_cloud_age_)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Cloud %s is stale (%.2fs old > %.2fs); excluding from merge.",
        src.name, (now - src.arrival).seconds(), max_cloud_age_);
      continue;
    }

    // Skip if empty (but keep going — other clouds might have data)
    if (src.cloud->width == 0 || src.cloud->data.empty())
    {
      continue;
    }

    sensor_msgs::msg::PointCloud2 cloud_transformed;
    if (!transformCloudToTarget(*src.cloud, cloud_transformed, src.name))
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Transforming cloud %s failed!", src.name);
      continue;
    }

    if (has_data)
    {
      sensor_msgs::msg::PointCloud2 temp_out;
      concatenateFlexible(cloud_out_, cloud_transformed, temp_out);
      cloud_out_ = temp_out;
    }
    else
    {
      cloud_out_ = cloud_transformed;
      has_data = true;
    }
  }

  // Publish the concatenated pointcloud
  if (has_data)
  {
    publishPointcloud(cloud_out_);
  }
}

void PointcloudConcatenate::concatenateFlexible(const sensor_msgs::msg::PointCloud2 &cloud_a, const sensor_msgs::msg::PointCloud2 &cloud_b, sensor_msgs::msg::PointCloud2 &cloud_out)
{
  // Handle empty clouds
  if (cloud_a.width == 0 || cloud_a.data.empty())
  {
    cloud_out = cloud_b;
    return;
  }
  if (cloud_b.width == 0 || cloud_b.data.empty())
  {
    cloud_out = cloud_a;
    return;
  }
  
  // Check point_step compatibility
  if (cloud_a.point_step != cloud_b.point_step)
  {
    RCLCPP_WARN(this->get_logger(), "Cannot concatenate clouds with different point_step (%d vs %d)", 
                cloud_a.point_step, cloud_b.point_step);
    cloud_out = cloud_a;
    return;
  }
  
  // Calculate total points
  size_t points_a = cloud_a.height * cloud_a.width;
  size_t points_b = cloud_b.height * cloud_b.width;
  size_t total_points = points_a + points_b;
  
  // Start with cloud_a structure
  cloud_out.header = cloud_a.header;
  cloud_out.height = 1;  // Make unorganized cloud
  cloud_out.width = total_points;
  cloud_out.fields = cloud_a.fields;
  cloud_out.is_bigendian = cloud_a.is_bigendian;
  cloud_out.point_step = cloud_a.point_step;
  cloud_out.row_step = total_points * cloud_out.point_step;
  cloud_out.is_dense = cloud_a.is_dense && cloud_b.is_dense;
  
  // Allocate data
  size_t total_data_size = cloud_a.data.size() + cloud_b.data.size();
  cloud_out.data.resize(total_data_size);
  
  // Copy cloud_a data
  std::memcpy(cloud_out.data.data(), cloud_a.data.data(), cloud_a.data.size());
  
  // Copy cloud_b data after cloud_a
  std::memcpy(cloud_out.data.data() + cloud_a.data.size(), cloud_b.data.data(), cloud_b.data.size());
}

void PointcloudConcatenate::publishPointcloud(sensor_msgs::msg::PointCloud2 &cloud)
{
  // Publishes the combined pointcloud

  // Update the timestamp
  cloud.header.stamp = this->now();
  cloud.header.frame_id = param_frame_target_;
  // Publish
  pub_cloud_out_->publish(cloud);
}
