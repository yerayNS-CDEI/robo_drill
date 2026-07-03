#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <Eigen/Dense>
#include <rclcpp/qos.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <mutex>

class LidarFovFilterNode : public rclcpp::Node
{
public:
  LidarFovFilterNode() : Node("lidar_fov_filter_node"),
                         tf_buffer_(this->get_clock()),
                         tf_listener_(tf_buffer_)
  {
    // -------- Params (topics) --------
    this->declare_parameter<std::string>("cloud_topic",        "/dome/points");
    this->declare_parameter<std::string>("image_topic",        "/camera/camera/color/image_raw");
    this->declare_parameter<std::string>("camera_info_topic",  "/camera/camera/color/camera_info");
    this->declare_parameter<std::string>("depth_topic",        "/camera/camera/depth/image_rect_raw");
    this->declare_parameter<std::string>("output_topic",       "/dome/fov_points");

    cloud_topic_       = this->get_parameter("cloud_topic").as_string();
    image_topic_       = this->get_parameter("image_topic").as_string();
    camera_info_topic_ = this->get_parameter("camera_info_topic").as_string();
    depth_topic_       = this->get_parameter("depth_topic").as_string();
    output_topic_      = this->get_parameter("output_topic").as_string();

    // -------- Params (tuning) --------
    this->declare_parameter("depth_tol_base_m",  depth_tol_base_m_);
    this->declare_parameter("depth_tol_scale",   depth_tol_scale_);
    this->declare_parameter("grad_thresh_m",     grad_thresh_m_);
    this->declare_parameter("border_guard_rows", border_guard_rows_);
    this->declare_parameter("window_radius",     window_radius_);
    this->declare_parameter("max_temporal_diff_ms", max_temporal_diff_ms_);
    this->declare_parameter("min_depth_range_m", min_depth_range_m_);
    this->declare_parameter("max_depth_range_m", max_depth_range_m_);

    depth_tol_base_m_  = this->get_parameter("depth_tol_base_m").as_double();
    depth_tol_scale_   = this->get_parameter("depth_tol_scale").as_double();
    grad_thresh_m_     = this->get_parameter("grad_thresh_m").as_double();
    border_guard_rows_ = this->get_parameter("border_guard_rows").as_int();
    window_radius_     = this->get_parameter("window_radius").as_int();
    max_temporal_diff_ms_ = this->get_parameter("max_temporal_diff_ms").as_int();
    min_depth_range_m_ = this->get_parameter("min_depth_range_m").as_double();
    max_depth_range_m_ = this->get_parameter("max_depth_range_m").as_double();

// -------- QoS --------
    auto qos_cloud = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
    auto qos_img   = rclcpp::SensorDataQoS();
    auto qos_info  = rclcpp::SensorDataQoS();

    // -------- Subs/Pub --------
    cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_, qos_info,
        std::bind(&LidarFovFilterNode::cameraInfoCallback, this, std::placeholders::_1));

    img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        image_topic_, qos_img,
        std::bind(&LidarFovFilterNode::imageCallback, this, std::placeholders::_1));

    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        depth_topic_, qos_img,
        std::bind(&LidarFovFilterNode::depthCallback, this, std::placeholders::_1));

    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic_, qos_cloud,
        std::bind(&LidarFovFilterNode::cloudCallback, this, std::placeholders::_1));

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, qos_cloud);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string lidar_topic_, camera_info_topic_, lidar_frame_, camera_frame_, output_topic_, cloud_topic_, depth_topic_, image_topic_;

  // Camera intrinsics + frame
  double fx_{0}, fy_{0}, cx_{0}, cy_{0};
  int img_width_{0}, img_height_{0};
  bool cam_info_ready_{false};
  // Latest RGB image (rectified) and guard
  cv::Mat latest_bgr_;
  rclcpp::Time latest_img_stamp_;
  std::mutex img_mtx_;

  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    // Expecting rectified image paired with this CameraInfo
    fx_ = msg->k[0];
    fy_ = msg->k[4];
    cx_ = msg->k[2];
    cy_ = msg->k[5];
    img_width_ = msg->width;
    img_height_ = msg->height;

    camera_frame_ = msg->header.frame_id; // e.g., camera_color_optical_frame
    cam_info_ready_ = true;

    if (camera_frame_.find("optical") == std::string::npos)
    {
      RCLCPP_WARN(get_logger(), "CameraInfo frame '%s' is not an *_optical_frame. Projection axes may be wrong.",
                  camera_frame_.c_str());
    }
  }

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try
    {
      cv::Mat bgr = cv_bridge::toCvShare(msg, "bgr8")->image;
      std::lock_guard<std::mutex> lk(img_mtx_);
      bgr.copyTo(latest_bgr_);
      latest_img_stamp_ = msg->header.stamp;
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_WARN(get_logger(), "cv_bridge(color) error: %s", e.what());
    }
  }

  void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try {
      cv::Mat d = cv_bridge::toCvShare(msg, msg->encoding)->image;
      cv::Mat d32;
      if (msg->encoding == "32FC1") {
        d32 = d;
      } else if (msg->encoding == "16UC1") {
        d.convertTo(d32, CV_32F, 0.001); // mm -> m
      } else {
        return; // unsupported encoding
      }

      // validity check - DISABLED erosion to allow more valid pixels
      // Erosion was too aggressive and removing most valid depth data
      cv::Mat valid = (d32 > 0) & (d32 == d32); // finite & >0
      // cv::erode(valid, valid, cv::getStructuringElement(cv::MORPH_RECT, {3,3}));  // DISABLED

      std::lock_guard<std::mutex> lk(depth_mtx_);
      d32.copyTo(latest_depth32_);
      valid.copyTo(latest_depth_valid_);
      latest_depth_stamp_ = msg->header.stamp;
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_WARN(get_logger(), "cv_bridge(depth) error: %s", e.what());
    }
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (!cam_info_ready_) return;

    // Grab latest color + depth snapshots with temporal validation
    cv::Mat bgr, depth32, depthValid;
    rclcpp::Time img_stamp, depth_stamp;
    {
      std::lock_guard<std::mutex> lk(img_mtx_);
      if (latest_bgr_.empty())
      {
        // Publish empty cloud so topic ticks (debug visibility)
        sensor_msgs::msg::PointCloud2 empty;
        empty.header = msg->header;
        empty.height = 1;
        empty.width = 0;
        empty.is_dense = false;
        pub_->publish(empty);
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No RGB image yet.");
        return;
      }
      bgr = latest_bgr_;
      img_stamp = latest_img_stamp_;
    }
    {
      std::lock_guard<std::mutex> lk2(depth_mtx_);
      if (latest_depth32_.empty() || latest_depth_valid_.empty()) {
        publishEmpty(msg->header, "No depth yet");
        return;
      }
      depth32 = latest_depth32_;
      depthValid = latest_depth_valid_;
      depth_stamp = latest_depth_stamp_;
    }
    
    // Temporal synchronization check
    const double img_diff_ms = std::abs((rclcpp::Time(msg->header.stamp) - img_stamp).seconds() * 1000.0);
    const double depth_diff_ms = std::abs((rclcpp::Time(msg->header.stamp) - depth_stamp).seconds() * 1000.0);
    if (img_diff_ms > max_temporal_diff_ms_ || depth_diff_ms > max_temporal_diff_ms_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, 
        "Temporal mismatch: RGB=%.1fms, Depth=%.1fms (max=%.1fms)", 
        img_diff_ms, depth_diff_ms, static_cast<double>(max_temporal_diff_ms_));
      publishEmpty(msg->header, "Temporal sync failed");
      return;
    }
    
    const int img_w = bgr.cols, img_h = bgr.rows;
    if (img_w <= 0 || img_h <= 0)
    {
      sensor_msgs::msg::PointCloud2 empty;
      empty.header = msg->header;
      empty.height = 1;
      empty.width = 0;
      empty.is_dense = false;
      pub_->publish(empty);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "RGB image has invalid size.");
      return;
    }

    // Transform lidar->camera at the cloud timestamp
    geometry_msgs::msg::TransformStamped tf;
    try
    {
      tf = tf_buffer_.lookupTransform(
          camera_frame_,        // target (camera optical)
          msg->header.frame_id, // source (actual lidar frame)
          msg->header.stamp,
          rclcpp::Duration::from_seconds(0.05));
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "TF lookup (%s->%s) failed: %s",
                           msg->header.frame_id.c_str(), camera_frame_.c_str(), ex.what());
      // Publish empty for visibility
      sensor_msgs::msg::PointCloud2 empty;
      empty.header = msg->header;
      empty.height = 1;
      empty.width = 0;
      empty.is_dense = false;
      pub_->publish(empty);
      return;
    }

    // Build 4x4 transform
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    const auto &tr = tf.transform.translation;
    const auto &q = tf.transform.rotation;
    Eigen::Quaterniond Q(q.w, q.x, q.y, q.z);
    T.block<3, 3>(0, 0) = Q.toRotationMatrix();
    T.block<3, 1>(0, 3) = Eigen::Vector3d(tr.x, tr.y, tr.z);

    // Prepare colored output cloud in lidar frame: x,y,z,rgba
    sensor_msgs::msg::PointCloud2 out;
    out.header = msg->header;
    out.height = 1;
    out.is_bigendian = false;
    out.is_dense = false;

    using PF = sensor_msgs::msg::PointField;
    out.fields.resize(4);
    out.fields[0] = PF();
    out.fields[0].name = "x";
    out.fields[0].offset = 0;
    out.fields[0].datatype = PF::FLOAT32;
    out.fields[0].count = 1;
    out.fields[1] = PF();
    out.fields[1].name = "y";
    out.fields[1].offset = 4;
    out.fields[1].datatype = PF::FLOAT32;
    out.fields[1].count = 1;
    out.fields[2] = PF();
    out.fields[2].name = "z";
    out.fields[2].offset = 8;
    out.fields[2].datatype = PF::FLOAT32;
    out.fields[2].count = 1;
    out.fields[3] = PF();
    out.fields[3].name = "rgba";
    out.fields[3].offset = 12;
    out.fields[3].datatype = PF::UINT32;
    out.fields[3].count = 1;
    out.point_step = 16;

    std::vector<uint8_t> buf;
    buf.reserve(static_cast<size_t>(msg->width) * out.point_step);

    // Find offsets reliably
    auto findOffset = [&](const std::string &name) -> int
    {
      for (const auto &f : msg->fields)
        if (f.name == name)
          return f.offset;
      return -1;
    };
    const int off_x = findOffset("x");
    const int off_y = findOffset("y");
    const int off_z = findOffset("z");
    if (off_x < 0 || off_y < 0 || off_z < 0)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Input cloud missing x/y/z fields.");
      // Publish empty for visibility
      out.width = 0;
      out.row_step = 0;
      out.data.clear();
      pub_->publish(out);
      return;
    }

    const uint8_t *data = msg->data.data();
    const size_t N = static_cast<size_t>(msg->width) * msg->height;
    const size_t in_step = msg->point_step;

    // stats - expanded for detailed diagnosis
    size_t kept=0, nan_drop=0, behind_drop=0, oob_drop=0;
    size_t depth_invalid=0, depth_mismatch=0, edge_reject=0, window_fail=0;

    auto push_xyz_rgba = [&](float Xs, float Ys, float Zs, uint32_t rgba)
    {
      const uint8_t *px = reinterpret_cast<uint8_t *>(&Xs);
      const uint8_t *py = reinterpret_cast<uint8_t *>(&Ys);
      const uint8_t *pz = reinterpret_cast<uint8_t *>(&Zs);
      const uint8_t *pr = reinterpret_cast<uint8_t *>(&rgba);
      buf.insert(buf.end(), px, px + 4);
      buf.insert(buf.end(), py, py + 4);
      buf.insert(buf.end(), pz, pz + 4);
      buf.insert(buf.end(), pr, pr + 4);
    };

    for (size_t i = 0; i < N; ++i)
    {
      const uint8_t *base = data + i * in_step;
      const float xs = *reinterpret_cast<const float *>(base + off_x);
      const float ys = *reinterpret_cast<const float *>(base + off_y);
      const float zs = *reinterpret_cast<const float *>(base + off_z);
      if (!std::isfinite(xs) || !std::isfinite(ys) || !std::isfinite(zs))
      {
        ++nan_drop;
        continue;
      }

      Eigen::Vector4d pL(xs, ys, zs, 1.0);
      Eigen::Vector4d pC = T * pL;
      const double X=pC.x(), Y=pC.y(), Z=pC.z();
      if (Z <= 0.0) { ++behind_drop; continue; }

      // CRITICAL: Only process points within camera's valid depth range
      // Camera depth sensors (RealSense/OAK-D) typically valid 0.3-10m
      // LiDAR sees 0-30m+, causing depth mismatch for distant points
      if (Z < min_depth_range_m_ || Z > max_depth_range_m_) { ++oob_drop; continue; }

      // project
      const double u = fx_*X/Z + cx_;
      const double v = fy_*Y/Z + cy_;

      // ---- VISIBILITY + EDGE/NOISE REJECTION + WINDOWED PICK ----
      const int uu = static_cast<int>(u + 0.5); // Round to nearest pixel
      const int vv = static_cast<int>(v + 0.5);
      if (uu < 0 || uu >= img_w || vv < 0 || vv >= img_h) { ++oob_drop; continue; }
      if (vv > img_h - border_guard_rows_) { ++oob_drop; continue; }
      if (!depthValid.at<uint8_t>(vv, uu)) { ++depth_invalid; continue; }

      const float z_img = depth32.at<float>(vv, uu);
      if (!std::isfinite(z_img) || z_img <= 0.0f) { ++depth_invalid; continue; }

      // Stricter depth gradient check - reject edges
      const int uu_next = std::min(uu+1, img_w-1);
      const int vv_next = std::min(vv+1, img_h-1);
      const int uu_prev = std::max(uu-1, 0);
      const int vv_prev = std::max(vv-1, 0);
      
      const float z_right = depth32.at<float>(vv, uu_next);
      const float z_down  = depth32.at<float>(vv_next, uu);
      const float z_left  = depth32.at<float>(vv, uu_prev);
      const float z_up    = depth32.at<float>(vv_prev, uu);
      
      // Check all four neighbors for depth discontinuities
      // RELAXED: Only reject if ANY neighbor shows extreme discontinuity (was rejecting if ANY exceeded threshold)
      int bad_neighbors = 0;
      if (std::isfinite(z_right) && std::fabs(z_right - z_img) > grad_thresh_m_) bad_neighbors++;
      if (std::isfinite(z_down)  && std::fabs(z_down  - z_img) > grad_thresh_m_) bad_neighbors++;
      if (std::isfinite(z_left)  && std::fabs(z_left  - z_img) > grad_thresh_m_) bad_neighbors++;
      if (std::isfinite(z_up)    && std::fabs(z_up    - z_img) > grad_thresh_m_) bad_neighbors++;
      // Only reject if 3 or 4 neighbors are bad (was: any neighbor bad)
      if (bad_neighbors >= 3) { ++edge_reject; continue; }

      // Depth matching tolerance - both base tolerance and scaled by distance
      const float tau_uv = std::max(static_cast<float>(depth_tol_base_m_),
                                    static_cast<float>(depth_tol_scale_) * static_cast<float>(Z));
      const float dz_uv  = std::fabs(static_cast<float>(Z) - z_img);
      
      // RELAXED: Allow depth mismatch within tolerance
      // Removed the strict "LiDAR must be closer than camera" check which was too aggressive
      if (dz_uv > tau_uv) { ++depth_mismatch; continue; }

      // Small window: choose best pixel by |Z - depth|, with stricter validation
      int best_u = -1, best_v = -1;
      float best_d = std::numeric_limits<float>::infinity();
      const int R = std::max(0, window_radius_); // Allow R=0 for exact pixel only

      for (int dv=-R; dv<=R; ++dv) {
        const int y = vv + dv; if (y < 0 || y >= img_h) continue;
        for (int du=-R; du<=R; ++du) {
          const int x = uu + du; if (x < 0 || x >= img_w) continue;
          if (!depthValid.at<uint8_t>(y, x)) continue;
          const float z2 = depth32.at<float>(y, x);
          if (!std::isfinite(z2) || z2 <= 0.0f) continue;
          
          // Use same tolerance for window search - RELAXED
          const float tau2 = std::max(static_cast<float>(depth_tol_base_m_),
                                      static_cast<float>(depth_tol_scale_) * static_cast<float>(Z));
          const float dz2 = std::fabs(static_cast<float>(Z) - z2);
          
          // RELAXED: Only check depth difference, removed strict ordering check
          if (dz2 > tau2) continue;
          
          if (dz2 < best_d) { best_d = dz2; best_u = x; best_v = y; }
        }
      }
      if (best_u < 0) { ++window_fail; continue; }

      // Use exact pixel color (no median) to avoid bleeding from adjacent surfaces
      // Or use very tight 3x3 median only if depth is consistent
      const cv::Vec3b cpx = bgr.at<cv::Vec3b>(best_v, best_u);

      const uint32_t rgba = (255u<<24) |
                            (uint32_t(cpx[2])<<16) |
                            (uint32_t(cpx[1])<<8)  |
                            uint32_t(cpx[0]);
      push_xyz_rgba(xs, ys, zs, rgba);
      ++kept;
    }

    out.width = static_cast<uint32_t>(buf.size() / out.point_step);
    out.row_step = out.width * out.point_step;
    out.data = std::move(buf);

    // Always publish so you can see rate in `ros2 topic hz`
    pub_->publish(out);

    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "Colorized: kept=%zu | REJECTED: nan=%zu behind=%zu oob=%zu depth_invalid=%zu depth_mismatch=%zu edge=%zu window_fail=%zu | img=%dx%d tau_base=%.2f scale=%.2f grad=%.2f win=%d",
      kept, nan_drop, behind_drop, oob_drop, depth_invalid, depth_mismatch, edge_reject, window_fail, 
      img_w, img_h, depth_tol_base_m_, depth_tol_scale_, grad_thresh_m_, window_radius_);
  }

  void publishEmpty(const std_msgs::msg::Header &h, const char* why)
  {
    sensor_msgs::msg::PointCloud2 empty;
    empty.header = h; empty.height=1; empty.width=0; empty.is_dense=false;
    pub_->publish(empty);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No output: %s", why);
  }



  // Latest depth (32F meters) + eroded validity mask
  cv::Mat latest_depth32_;
  cv::Mat latest_depth_valid_;
  rclcpp::Time latest_depth_stamp_;
  std::mutex depth_mtx_;

  // Tuning - More conservative defaults to prevent color bleeding
  double depth_tol_base_m_  = 10.05; // 5cm base tolerance (was 6m - way too large!)
  double depth_tol_scale_   = 0.01; // +1% of Z with distance (was 2%)
  double grad_thresh_m_     = 0.10; // reject depth edges above 10 cm step (was 30cm)
  int    border_guard_rows_ = 4;    // skip bottom rows (floor artifacts)
  int    window_radius_     = 0;    // R=0 for exact pixel match (was 1 - 3x3 window)
  int    max_temporal_diff_ms_ = 50; // Max time difference for RGB/Depth sync (ms)
  double min_depth_range_m_ = 0.3;  // Min distance to color (camera depth min range)
  double max_depth_range_m_ = 6.0;  // Max distance to color (camera depth max range)
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarFovFilterNode>());
  rclcpp::shutdown();
  return 0;
}
