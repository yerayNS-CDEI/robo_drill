// Self-filter node: drops points falling inside the robot's collision geometry,
// using MoveIt's point_containment_filter::ShapeMask. Robot model is loaded
// from the `robot_description` parameter; joint poses are read from TF
// (published by robot_state_publisher), so move_group is not required.

#include <cstring>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Geometry>
#include <geometric_shapes/shapes.h>
#include <moveit/point_containment_filter/shape_mask.h>
#include <moveit/robot_model/robot_model.h>
#include <rclcpp/rclcpp.hpp>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace robo_drill
{

using point_containment_filter::ShapeHandle;
using point_containment_filter::ShapeMask;

class RobotBodyFilterNode : public rclcpp::Node
{
public:
  explicit RobotBodyFilterNode(const rclcpp::NodeOptions & options)
  : Node("robot_body_filter", options)
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/combined_cloud");
    output_topic_ = declare_parameter<std::string>("output_topic", "/combined_cloud_filtered");
    padding_ = declare_parameter<double>("padding", 0.05);
    scale_ = declare_parameter<double>("scale", 1.0);
    tf_timeout_sec_ = declare_parameter<double>("tf_timeout", 0.1);
    // ShapeMask treats max_sensor_dist as a literal upper bound, not a
    // "disabled" sentinel — points beyond it are CLIPped and never tested for
    // containment. Default to 100 m so we cover any realistic lidar range.
    max_sensor_dist_ = declare_parameter<double>("max_sensor_dist", 100.0);
    keep_organized_ = declare_parameter<bool>("keep_organized", false);
    // rclcpp Humble can't infer parameter type from an empty default vector;
    // declare dynamic-typed so a YAML `exclude_links: []` is accepted.
    {
      rcl_interfaces::msg::ParameterDescriptor d;
      d.dynamic_typing = true;
      exclude_links_ = declare_parameter(
        "exclude_links", std::vector<std::string>{}, d);
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  // Must be called after the node is wrapped in a shared_ptr (i.e. after
  // make_shared returns), because RobotModelLoader stores a shared_ptr<Node>.
  void init(const std::shared_ptr<rclcpp::Node> & /*self*/)
  {
    // Read URDF from the `robot_description` parameter. The launch builds it
    // via xacro and selects the right top-level file for the mode (base vs
    // full). We bypass moveit's RobotModelLoader because in Humble it waits
    // 10 s for an SRDF topic we don't publish; we only need link collision
    // geometry, so feed an empty SRDF straight into RobotModel.
    declare_parameter("robot_description", rclcpp::ParameterType::PARAMETER_STRING);
    std::string urdf_str = get_parameter("robot_description").as_string();
    if (urdf_str.empty()) {
      RCLCPP_FATAL(get_logger(),
        "Parameter 'robot_description' is empty — pass the URDF via the launch.");
      throw std::runtime_error("robot_description empty");
    }

    auto urdf = urdf::parseURDF(urdf_str);
    if (!urdf) {
      RCLCPP_FATAL(get_logger(), "Failed to parse URDF from 'robot_description'.");
      throw std::runtime_error("urdf parse failed");
    }
    auto srdf = std::make_shared<srdf::Model>();  // empty SRDF is fine for self-filtering
    auto model = std::make_shared<moveit::core::RobotModel>(urdf, srdf);
    if (!model) {
      RCLCPP_FATAL(get_logger(), "Failed to build moveit RobotModel.");
      throw std::runtime_error("robot model build failed");
    }

    shape_mask_ = std::make_unique<ShapeMask>(
      [this](ShapeHandle h, Eigen::Isometry3d & out) { return getShapeTransform(h, out); });

    registerCollisionShapes(*model);

    // Reliable QoS matches rtabmap_util/point_cloud_aggregator (publisher) and
    // rtabmap_slam (subscriber) defaults; best_effort silently fails to peer
    // with reliable endpoints.
    //
    // Both input and output use KeepLast(1). A deeper output history causes DDS
    // NACK/retransmit cycles when icp_odometry (a slow reliable reader, ~100ms
    // per ICP call) falls behind, which adds protocol overhead that starves
    // other subscribers on the same topic.
    rclcpp::QoS sub_qos(rclcpp::KeepLast(1));
    sub_qos.reliable();
    rclcpp::QoS pub_qos(rclcpp::KeepLast(1));
    pub_qos.reliable();
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, sub_qos,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { cloudCallback(msg); });
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, pub_qos);

    RCLCPP_INFO(get_logger(),
      "robot_body_filter: %zu shapes from %zu links; %s -> %s (padding=%.3f, scale=%.3f).",
      shape_lookup_.size(), links_with_shapes_,
      input_topic_.c_str(), output_topic_.c_str(), padding_, scale_);
  }

private:
  struct ShapeInfo
  {
    std::string link_name;
    Eigen::Isometry3d collision_origin;  // shape pose in link frame
  };

  void registerCollisionShapes(const moveit::core::RobotModel & model)
  {
    std::unordered_map<std::string, bool> excluded;
    for (const auto & name : exclude_links_) {
      excluded[name] = true;
    }

    for (const moveit::core::LinkModel * link : model.getLinkModelsWithCollisionGeometry()) {
      if (excluded.count(link->getName())) {
        RCLCPP_INFO(get_logger(), "Excluding link from filter: %s", link->getName().c_str());
        continue;
      }
      const auto & shapes = link->getShapes();
      const auto & origins = link->getCollisionOriginTransforms();
      if (shapes.size() != origins.size()) {
        RCLCPP_WARN(get_logger(),
          "Link %s: shapes/origins size mismatch (%zu vs %zu); skipping.",
          link->getName().c_str(), shapes.size(), origins.size());
        continue;
      }
      for (size_t i = 0; i < shapes.size(); ++i) {
        ShapeHandle h = shape_mask_->addShape(shapes[i], scale_, padding_);
        shape_lookup_[h] = ShapeInfo{link->getName(), origins[i]};
        RCLCPP_INFO(get_logger(),
          "  shape %u: link='%s' type=%d",
          h, link->getName().c_str(), static_cast<int>(shapes[i]->type));
      }
      ++links_with_shapes_;
    }
  }

  // Called by ShapeMask once per registered shape during maskContainment.
  bool getShapeTransform(ShapeHandle h, Eigen::Isometry3d & out)
  {
    auto it = transform_cache_.find(h);
    if (it == transform_cache_.end()) {
      return false;
    }
    out = it->second;
    return true;
  }

  // Pre-compute (cloud_frame <- link * collision_origin) for every shape, once
  // per cloud, so the masking hot path is TF-lookup-free.
  //
  // We look up the LATEST transform (TimePointZero), not the cloud's stamp: the
  // upstream concatenator restamps /combined_cloud with its own wall-now (not
  // true capture time), so an exact-stamp lookup is no more correct than latest
  // — and exact-stamp lookups kept extrapolating past the newest arm TF, which
  // both blocked for the full tf_timeout and (with the old whole-cloud drop)
  // collapsed the output rate. Latest is instant and never extrapolates.
  //
  // A link whose TF is unavailable is SKIPPED for this cloud (its shape is just
  // omitted) instead of dropping the whole cloud — leaking a few self-points
  // from one link beats publishing nothing.
  void updateTransformCache(const std::string & cloud_frame)
  {
    transform_cache_.clear();
    const tf2::Duration timeout = tf2::durationFromSec(tf_timeout_sec_);

    // Group lookups by link to avoid redundant tf2 calls for multi-shape links,
    // and remember per-link failures so we attempt each link at most once.
    std::unordered_map<std::string, Eigen::Isometry3d> link_tf_cache;
    std::unordered_map<std::string, bool> link_failed;

    for (const auto & kv : shape_lookup_) {
      const ShapeHandle handle = kv.first;
      const ShapeInfo & info = kv.second;

      if (link_failed.count(info.link_name)) {
        continue;
      }
      auto cached = link_tf_cache.find(info.link_name);
      Eigen::Isometry3d cloud_T_link;
      if (cached != link_tf_cache.end()) {
        cloud_T_link = cached->second;
      } else {
        try {
          auto tf_msg = tf_buffer_->lookupTransform(
            cloud_frame, info.link_name, tf2::TimePointZero, timeout);
          cloud_T_link = tf2::transformToEigen(tf_msg);
          link_tf_cache.emplace(info.link_name, cloud_T_link);
        } catch (const tf2::TransformException & ex) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "TF lookup %s <- %s failed; skipping link this cloud: %s",
            cloud_frame.c_str(), info.link_name.c_str(), ex.what());
          link_failed[info.link_name] = true;
          continue;
        }
      }
      transform_cache_[handle] = cloud_T_link * info.collision_origin;
    }
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    updateTransformCache(msg->header.frame_id);

    // Sensor origin is the cloud frame's origin expressed in the cloud frame.
    const Eigen::Vector3d sensor_origin = Eigen::Vector3d::Zero();
    std::vector<int> mask;
    {
      std::lock_guard<std::mutex> lk(mask_mutex_);
      shape_mask_->maskContainment(*msg, sensor_origin, 0.0, max_sensor_dist_, mask);
    }

    publishFiltered(*msg, mask);
  }

  void publishFiltered(const sensor_msgs::msg::PointCloud2 & in, const std::vector<int> & mask)
  {
    auto out = std::make_unique<sensor_msgs::msg::PointCloud2>();
    out->header = in.header;
    out->fields = in.fields;
    out->point_step = in.point_step;
    out->is_bigendian = in.is_bigendian;

    const size_t n = static_cast<size_t>(in.width) * in.height;

    if (keep_organized_) {
      out->height = in.height;
      out->width = in.width;
      out->row_step = in.row_step;
      out->is_dense = false;
      out->data = in.data;
      sensor_msgs::PointCloud2Iterator<float> ix(*out, "x");
      sensor_msgs::PointCloud2Iterator<float> iy(*out, "y");
      sensor_msgs::PointCloud2Iterator<float> iz(*out, "z");
      const float nan = std::numeric_limits<float>::quiet_NaN();
      for (size_t i = 0; i < n; ++i, ++ix, ++iy, ++iz) {
        if (mask[i] == ShapeMask::INSIDE) {
          *ix = nan; *iy = nan; *iz = nan;
        }
      }
    } else {
      // Compact: drop INSIDE points AND any point with a non-finite float
      // field. Downstream PCL kdtrees assert on NaN when is_dense is true,
      // and some custom point representations check fields beyond x/y/z
      // (e.g. intensity), so we validate every float field.
      std::vector<size_t> float_offsets;
      float_offsets.reserve(in.fields.size());
      for (const auto & f : in.fields) {
        if (f.datatype == sensor_msgs::msg::PointField::FLOAT32) {
          float_offsets.push_back(f.offset);
        }
      }

      out->height = 1;
      out->is_dense = true;
      out->data.resize(in.data.size());
      const uint8_t * src = in.data.data();
      uint8_t * dst = out->data.data();
      size_t kept = 0;
      for (size_t i = 0; i < n; ++i) {
        if (mask[i] == ShapeMask::INSIDE) {
          continue;
        }
        const uint8_t * pt = src + i * in.point_step;
        bool valid = true;
        for (size_t off : float_offsets) {
          float v;
          std::memcpy(&v, pt + off, sizeof(float));
          if (!std::isfinite(v)) { valid = false; break; }
        }
        if (!valid) {
          continue;
        }
        std::memcpy(dst + kept * in.point_step, pt, in.point_step);
        ++kept;
      }
      out->data.resize(kept * in.point_step);
      out->width = static_cast<uint32_t>(kept);
      out->row_step = static_cast<uint32_t>(kept * in.point_step);

      // Skip publishing empty clouds — they make PCL kdtree consumers warn
      // about "empty input cloud" without contributing to mapping anyway.
      if (kept == 0) {
        return;
      }
    }

    pub_->publish(std::move(out));
  }

  std::string input_topic_;
  std::string output_topic_;
  double padding_{0.05};
  double scale_{1.0};
  double tf_timeout_sec_{0.1};
  double max_sensor_dist_{0.0};
  bool keep_organized_{false};
  std::vector<std::string> exclude_links_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<ShapeMask> shape_mask_;

  std::unordered_map<ShapeHandle, ShapeInfo> shape_lookup_;
  std::unordered_map<ShapeHandle, Eigen::Isometry3d> transform_cache_;
  size_t links_with_shapes_{0};
  std::mutex mask_mutex_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

}  // namespace robo_drill

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<robo_drill::RobotBodyFilterNode>(options);
  node->init(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
