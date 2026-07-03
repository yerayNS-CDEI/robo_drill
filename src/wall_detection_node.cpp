// Copyright 2026 robo_drill
//
// Tier-A wall detection from the merged LiDAR cloud.
//
// Walls are vertical planes, so in a gravity-aligned horizontal projection they
// collapse to dense straight LINES. This node turns the hard 3D multi-plane
// problem into the well-studied 2D line-extraction problem, which is far more
// stable than greedy 3D RANSAC in cluttered indoor scenes:
//
//   1. Transform /combined_cloud_filtered into a gravity-aligned frame.
//   2. Estimate the floor height and keep only a vertical band above it
//      (drops floor, ceiling, and most low furniture).
//   3. Accumulate the band into a 2D bird's-eye grid. A vertical surface stacks
//      many returns into one XY cell AND spans a large vertical extent there;
//      clutter does neither, so cells with enough points AND enough vertical
//      extent are wall-like.
//   4. Extract line segments from those cells (OpenCV HoughLinesP).
//   5. For each segment, gather its 3D support, measure length / height extent /
//      confidence, and publish walls as markers (RViz) and a PoseArray whose
//      pose = wall midpoint with orientation along the wall NORMAL — directly
//      consumable by the arm scanning planner for standoff trajectories.
//
// Tier B (precise per-wall plane fit) and Tier C (temporal persistence in the
// map frame + YOLO fusion) build on this output; see the proposal.

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/imgproc.hpp>

#include <Eigen/Dense>

#include "robo_drill/msg/wall.hpp"
#include "robo_drill/msg/wall_array.hpp"

namespace robo_drill
{

using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;

// A detected wall in the gravity-aligned target frame. Endpoints lie on the
// floor-projected wall line; the band spans [z_min, z_max].
struct Wall
{
  Eigen::Vector2f p1;       // segment endpoint A (x,y), target frame
  Eigen::Vector2f p2;       // segment endpoint B (x,y), target frame
  Eigen::Vector2f normal;   // unit normal in XY (points toward +support side)
  float z_min{0.0f};
  float z_max{0.0f};
  float length{0.0f};
  int support{0};
  float confidence{0.0f};

  // Tier-A line-support points (3D, target frame) -> input to the Tier-B fit.
  CloudT::Ptr inliers;

  // Tier-B: precise constrained vertical-plane fit. plane_d completes the plane
  // (normal . p + plane_d = 0). refined=false means the fit was skipped/failed
  // and the Tier-A line geometry is reported as-is.
  bool refined{false};
  float plane_d{0.0f};
};

struct RhtBinKey
{
  int theta{0};
  int phi{0};
  int rho{0};

  bool operator==(const RhtBinKey & other) const
  {
    return theta == other.theta && phi == other.phi && rho == other.rho;
  }
};

struct RhtBinKeyHash
{
  size_t operator()(const RhtBinKey & key) const
  {
    size_t h = std::hash<int>{}(key.theta);
    h ^= std::hash<int>{}(key.phi) + 0x9e3779b9u + (h << 6) + (h >> 2);
    h ^= std::hash<int>{}(key.rho) + 0x9e3779b9u + (h << 6) + (h >> 2);
    return h;
  }
};

struct RhtBinValue
{
  int votes{0};
  Eigen::Vector3f normal_sum{Eigen::Vector3f::Zero()};
  float rho_sum{0.0f};
};

class WallDetectionNode : public rclcpp::Node
{
public:
  WallDetectionNode()
  : Node("wall_detection_node")
  {
    // ---- topics / frames ----
    input_topic_ = declare_parameter<std::string>("input_topic", "/combined_cloud_filtered");
    // Must be a FIXED, gravity-aligned frame (odom/map): accumulation assumes
    // static walls stay put across the window. base_link would smear while the
    // robot drives.
    target_frame_ = declare_parameter<std::string>("target_frame", "odom");

    // Temporal accumulation: each merged cloud carries only ~800 band points —
    // far too sparse for stable single-frame line fitting (endpoints and cell
    // occupancy jump every frame). Accumulating a short window in the fixed
    // frame densifies the band and makes static walls persistent.
    accumulation_window_ = declare_parameter<double>("accumulation_window", 0.7);

    // Patience for the exact-stamp odom lookup before falling back to latest.
    tf_timeout_ = declare_parameter<double>("tf_timeout", 0.10);
    // Max age gap (s) between the cloud stamp and the LATEST transform when the
    // exact-stamp lookup fails. The latest transform carries the robot's CURRENT
    // pose; applying it to a cloud captured a moment earlier rotates the whole
    // cloud to the wrong place during a spin. Only accept the fallback when it is
    // this close in time (error ~ angular_vel * gap); otherwise drop the frame.
    max_tf_stale_ = declare_parameter<double>("max_tf_stale", 0.05);

    // ---- detector front-end ----
    detector_mode_ = declare_parameter<std::string>("detector_mode", "projection_hough");
    if (detector_mode_ != "projection_hough" && detector_mode_ != "rht_3d")
    {
      RCLCPP_WARN(get_logger(),
        "Unknown detector_mode='%s'; falling back to projection_hough.",
        detector_mode_.c_str());
      detector_mode_ = "projection_hough";
    }

    // ---- preprocessing ----
    voxel_leaf_ = declare_parameter<double>("voxel_leaf", 0.05);
    floor_percentile_ = declare_parameter<double>("floor_percentile", 0.05);
    band_min_height_ = declare_parameter<double>("band_min_height", 0.30);
    band_max_height_ = declare_parameter<double>("band_max_height", 2.00);
    // Ceiling estimate (symmetric to the floor): a HIGH z-percentile, robust to
    // a few stray points above the ceiling. Used to anchor wall acceptance to
    // the room ceiling instead of a fixed height (see scoreWall).
    ceiling_percentile_ = declare_parameter<double>("ceiling_percentile", 0.95);
    // A support point within this of the detected ceiling counts as "reaching"
    // it. Tolerates ceiling sparsity / a slightly low ceiling estimate.
    ceiling_gap_ = declare_parameter<double>("ceiling_gap", 0.30);

    // ---- 2D grid + verticality test ----
    grid_resolution_ = declare_parameter<double>("grid_resolution", 0.05);
    min_points_per_cell_ = declare_parameter<int>("min_points_per_cell", 3);
    min_cell_vertical_extent_ = declare_parameter<double>("min_cell_vertical_extent", 0.5);
    // Slice-occupancy verticality: bin each grid column into slices of this
    // height and require the occupied slices to be DENSE within their span (a
    // real wall fills its column; a doorway lintel fills only the top; a stray
    // baseboard+lintel pair spans a big extent but leaves a hollow middle).
    // vertical_fill_ratio = occupied_slices / span_slices, so it tolerates a
    // wall occluded from the bottom (mid->top still dense) yet rejects the
    // hollow lintel-over-opening column that used to read as a full wall.
    vertical_slice_height_ = declare_parameter<double>("vertical_slice_height", 0.20);
    vertical_fill_ratio_ = declare_parameter<double>("vertical_fill_ratio", 0.6);

    // ---- line extraction (Hough) + acceptance ----
    min_wall_length_ = declare_parameter<double>("min_wall_length", 1.0);
    // Furniture rejection: the wall top must reach this high above the floor. A
    // real wall runs to the ceiling; counters/drawers/most cabinets do not.
    min_wall_height_ = declare_parameter<double>("min_wall_height", 1.5);
    max_wall_gap_ = declare_parameter<double>("max_wall_gap", 0.30);
    hough_threshold_ = declare_parameter<int>("hough_threshold", 20);
    line_inlier_dist_ = declare_parameter<double>("line_inlier_dist", 0.10);
    min_support_points_ = declare_parameter<int>("min_support_points", 50);
    // Furniture rejection by HIGH-RETURN COUNT: a real wall returns many points
    // up near min_wall_height; a table/counter has at most a few stray returns
    // that high, which a single 98th-percentile z_max can't tell from a real
    // wall top. Require at least this many support points to actually reach the
    // top band. 0 disables (falls back to the z_max-only check).
    min_high_support_ = declare_parameter<int>("min_high_support", 10);
    // Column rejection: the ceiling-reaching support must span at least this far
    // ALONG the wall. A column concentrates full-height returns in a ~0.5 m
    // footprint; a real wall spreads them along its length. 0 disables.
    min_ceiling_support_length_ = declare_parameter<double>("min_ceiling_support_length", 1.0);
    // MASS/BLOB rejection. The detector gathers support within line_inlier_dist of
    // its fitted line, so it happily finds a thin slice through a wide solid mass
    // (cluttered corner, filled area, stacked material to the ceiling) and calls
    // it a wall — the ceiling/column tests don't catch it because the mass IS tall
    // and long. But a real wall is a THIN plane with open space on at least one
    // side, whereas a mass has tall returns on BOTH sides. So look at the
    // perpendicular shells [blob_shell_inner, blob_shell_outer] beyond the wall on
    // each side and count ceiling-reaching returns there; if BOTH sides are
    // populated (>= blob_side_ratio of the wall's own high support, and at least
    // blob_min_side points), the candidate sits inside a mass -> reject.
    // blob_side_ratio 0 disables. Inner is set beyond a plausible wall thickness so
    // the wall's own returns aren't counted; outer bounds the neighbourhood probed.
    blob_shell_inner_ = declare_parameter<double>("blob_shell_inner", 0.25);
    blob_shell_outer_ = declare_parameter<double>("blob_shell_outer", 0.80);
    blob_side_ratio_ = declare_parameter<double>("blob_side_ratio", 0.5);
    blob_min_side_ = declare_parameter<int>("blob_min_side", 5);
    merge_angle_deg_ = declare_parameter<double>("merge_angle_deg", 8.0);
    merge_dist_ = declare_parameter<double>("merge_dist", 0.15);

    // ---- alternative front-end: 3D randomized Hough plane proposals ----
    rht_max_iterations_ = declare_parameter<int>("rht_max_iterations", 12000);
    rht_max_rounds_ = declare_parameter<int>("rht_max_rounds", 6);
    rht_vote_threshold_ = declare_parameter<int>("rht_vote_threshold", 8);
    rht_top_bins_per_round_ = declare_parameter<int>("rht_top_bins_per_round", 20);
    rht_rho_bin_size_ = declare_parameter<double>("rht_rho_bin_size", 0.08);
    rht_theta_bin_size_deg_ = declare_parameter<double>("rht_theta_bin_size_deg", 4.0);
    rht_phi_bin_size_deg_ = declare_parameter<double>("rht_phi_bin_size_deg", 3.0);
    rht_max_wall_phi_deg_ = declare_parameter<double>("rht_max_wall_phi_deg", 15.0);
    rht_min_pairwise_dist_ = declare_parameter<double>("rht_min_pairwise_dist", 0.20);
    rht_max_pairwise_dist_ = declare_parameter<double>("rht_max_pairwise_dist", 4.00);
    rht_min_triangle_area_ = declare_parameter<double>("rht_min_triangle_area", 0.02);
    rht_plane_inlier_dist_ = declare_parameter<double>("rht_plane_inlier_dist", 0.08);
    rht_min_candidate_inliers_ = declare_parameter<int>("rht_min_candidate_inliers", 80);
    rht_min_segment_points_ = declare_parameter<int>("rht_min_segment_points", 20);
    rht_segment_gap_ = declare_parameter<double>("rht_segment_gap", 0.35);
    rht_random_seed_ = declare_parameter<int>("rht_random_seed", 1337);

    // ---- Tier B: constrained vertical-plane fit ----
    plane_dist_thresh_ = declare_parameter<double>("plane_dist_thresh", 0.04);
    plane_eps_angle_deg_ = declare_parameter<double>("plane_eps_angle_deg", 15.0);
    ransac_max_iter_ = declare_parameter<int>("ransac_max_iter", 200);
    min_plane_inliers_ = declare_parameter<int>("min_plane_inliers", 80);

    publish_debug_cloud_ = declare_parameter<bool>("publish_debug_cloud", true);

    // ---- motion gate ----
    // While the robot turns, the cloud lags the pose and the accumulated band
    // smears across orientations, so walls land at the wrong coordinates. Pause
    // detection whenever the robot is turning faster than max_angular_vel, and
    // for motion_settle_time afterwards (covers the cloud latency + accumulation
    // window still holding turn-era clouds). Mirrors the aggregator's gate but
    // stops the smeared band from ever being built. Uses the same odom as the
    // aggregator (twist.angular.z).
    motion_gate_ = declare_parameter<bool>("motion_gate", true);
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/rtabmap/odom");
    max_angular_vel_ = declare_parameter<double>("max_angular_vel", 0.3);
    motion_settle_time_ = declare_parameter<double>("motion_settle_time", 0.5);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    auto sub_qos = rclcpp::SensorDataQoS().keep_last(1);
    sub_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, sub_qos,
      std::bind(&WallDetectionNode::cloudCallback, this, std::placeholders::_1));
    if (motion_gate_)
    {
      sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, rclcpp::SensorDataQoS(),
        std::bind(&WallDetectionNode::odomCallback, this, std::placeholders::_1));
    }

    auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/markers", pub_qos);
    pub_poses_ = create_publisher<geometry_msgs::msg::PoseArray>("~/poses", pub_qos);
    pub_walls_ = create_publisher<robo_drill::msg::WallArray>("~/walls", pub_qos);
    if (publish_debug_cloud_)
    {
      pub_debug_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/candidate_cloud", pub_qos);
    }

    RCLCPP_INFO(get_logger(), "wall_detection_node up: %s -> walls in '%s' (mode=%s)",
                input_topic_.c_str(), target_frame_.c_str(), detector_mode_.c_str());
  }

private:
  // ------------------------------------------------------------------
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    have_odom_ = true;
    if (std::abs(msg->twist.twist.angular.z) > max_angular_vel_)
    {
      last_rotating_ = now();
    }
  }

  // True once the robot has been below max_angular_vel for motion_settle_time.
  // Fails open when the gate is off or no odom has arrived, so a missing odom
  // topic can't silently stall detection.
  bool motionSettled() const
  {
    if (!motion_gate_ || !have_odom_)
    {
      return true;
    }
    return (now() - last_rotating_).seconds() >= motion_settle_time_;
  }

  // ------------------------------------------------------------------
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (pub_markers_->get_subscription_count() == 0 &&
        pub_poses_->get_subscription_count() == 0 &&
        pub_walls_->get_subscription_count() == 0 &&
        (!pub_debug_ || pub_debug_->get_subscription_count() == 0))
    {
      return;  // nobody listening
    }

    // Motion gate: while turning (and for the settle window after), skip building
    // the band entirely and drop the accumulation buffer so no turn-era cloud
    // smears into the next detection. The aggregator holds the existing map.
    if (!motionSettled())
    {
      buffer_.clear();
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
        "detection paused: robot turning (> %.2f rad/s) or settling", max_angular_vel_);
      return;
    }
    if (msg->width == 0 || msg->data.empty())
    {
      return;
    }

    // ---- 1. transform into the gravity-aligned target frame ----
    Eigen::Isometry3d T;
    if (msg->header.frame_id == target_frame_)
    {
      T.setIdentity();
    }
    else
    {
      geometry_msgs::msg::TransformStamped tf;
      try
      {
        tf = tf_buffer_->lookupTransform(
          target_frame_, msg->header.frame_id, msg->header.stamp,
          rclcpp::Duration::from_seconds(tf_timeout_));
      }
      catch (const tf2::TransformException & e)
      {
        // Exact-stamp lookup failed (odom TF lags the cloud, or the cloud stamp
        // ran ahead of odom). Fall back to the LATEST transform, but ONLY if it
        // is within max_tf_stale of the cloud stamp: the latest transform is the
        // robot's CURRENT pose, and applying it to a cloud captured earlier
        // rotates every point to the wrong place while turning (the wrong-wall
        // symptom during a spin). If the latest transform is too stale, drop the
        // frame — a missed frame is recoverable; a mis-placed wall is not.
        try
        {
          tf = tf_buffer_->lookupTransform(
            target_frame_, msg->header.frame_id, tf2::TimePointZero);
        }
        catch (const tf2::TransformException & e2)
        {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "TF %s -> %s failed: %s", msg->header.frame_id.c_str(),
            target_frame_.c_str(), e2.what());
          return;
        }
        const double gap =
          std::abs((rclcpp::Time(msg->header.stamp) - rclcpp::Time(tf.header.stamp)).seconds());
        if (gap > max_tf_stale_)
        {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "dropping cloud: no TF at stamp, latest is %.0f ms stale (> %.0f ms cap)",
            gap * 1e3, max_tf_stale_ * 1e3);
          return;
        }
      }
      T = tf2::transformToEigen(tf);
    }

    CloudT::Ptr raw(new CloudT);
    pcl::fromROSMsg(*msg, *raw);
    CloudT::Ptr cloud(new CloudT);
    pcl::transformPointCloud(*raw, *cloud, T.matrix().cast<float>());
    const Eigen::Vector2f viewpoint_xy(
      static_cast<float>(T.translation().x()),
      static_cast<float>(T.translation().y()));
    const bool viewpoint_valid = (msg->header.frame_id != target_frame_);

    // ---- 2. voxel downsample ----
    if (voxel_leaf_ > 0.0)
    {
      pcl::VoxelGrid<PointT> vg;
      vg.setInputCloud(cloud);
      vg.setLeafSize(voxel_leaf_, voxel_leaf_, voxel_leaf_);
      CloudT::Ptr ds(new CloudT);
      vg.filter(*ds);
      cloud = ds;
    }
    if (cloud->empty())
    {
      return;
    }

    // ---- 2b. temporal accumulation in the fixed target frame ----
    // Buffer the voxelized cloud, drop frames older than the window, and detect
    // on the union. Static walls accumulate to a dense, stable column per cell;
    // transient/sparse returns don't.
    const rclcpp::Time stamp(msg->header.stamp);
    buffer_.emplace_back(stamp, cloud);
    while (!buffer_.empty() &&
           (stamp - buffer_.front().first).seconds() > accumulation_window_)
    {
      buffer_.pop_front();
    }
    CloudT::Ptr accum(new CloudT);
    for (const auto & e : buffer_)
    {
      *accum += *e.second;
    }
    cloud = accum;

    // ---- 3. estimate floor + ceiling height, keep a vertical band above floor ----
    const float floor_z = floorHeight(*cloud);
    const float ceiling_z = ceilingHeight(*cloud);
    const float z_lo = floor_z + static_cast<float>(band_min_height_);
    const float z_hi = floor_z + static_cast<float>(band_max_height_);

    CloudT::Ptr band(new CloudT);
    band->reserve(cloud->size());
    for (const auto & p : cloud->points)
    {
      if (p.z >= z_lo && p.z <= z_hi)
      {
        band->push_back(p);
      }
    }
    if (band->size() < static_cast<size_t>(min_support_points_))
    {
      publishWalls({}, msg->header.stamp);
      if (pub_debug_)
      {
        publishDebug(band, msg->header.stamp);
      }
      return;
    }

    // ---- 4. extract wall lines, 5. measure + publish ----
    std::vector<Wall> walls;
    if (detector_mode_ == "rht_3d")
    {
      walls = extractWallsRht3D(
        *band, floor_z, z_lo, z_hi, ceiling_z, viewpoint_xy, viewpoint_valid);
    }
    else
    {
      walls = extractWallsProjection(*band, floor_z, z_lo, z_hi, ceiling_z);
    }
    publishWalls(walls, msg->header.stamp);
    if (pub_debug_)
    {
      publishDebug(band, msg->header.stamp);
    }

    size_t refined = 0;
    for (const auto & w : walls)
    {
      refined += w.refined ? 1 : 0;
    }
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
      "%s floor=%.2f ceiling=%.2f band=[%.2f,%.2f] band_pts=%zu walls=%zu (refined=%zu)",
      detector_mode_.c_str(), floor_z, ceiling_z, z_lo, z_hi, band->size(), walls.size(),
      refined);
  }

  // ------------------------------------------------------------------
  // Robust floor height: a low percentile of z (not the strict min, which a
  // single stray return below the floor would corrupt).
  float floorHeight(const CloudT & cloud) const
  {
    std::vector<float> zs;
    zs.reserve(cloud.size());
    for (const auto & p : cloud.points)
    {
      zs.push_back(p.z);
    }
    const size_t k = std::min(zs.size() - 1,
      static_cast<size_t>(floor_percentile_ * zs.size()));
    std::nth_element(zs.begin(), zs.begin() + k, zs.end());
    return zs[k];
  }

  // ------------------------------------------------------------------
  // Robust ceiling height: a HIGH percentile of z (symmetric to floorHeight),
  // so a few stray returns above the ceiling don't corrupt it. Walls reach this;
  // furniture (tables/drawers/cabinets/shelves) tops out well below it.
  float ceilingHeight(const CloudT & cloud) const
  {
    std::vector<float> zs;
    zs.reserve(cloud.size());
    for (const auto & p : cloud.points)
    {
      zs.push_back(p.z);
    }
    const size_t k = std::min(zs.size() - 1,
      static_cast<size_t>(ceiling_percentile_ * zs.size()));
    std::nth_element(zs.begin(), zs.begin() + k, zs.end());
    return zs[k];
  }

  // ------------------------------------------------------------------
  std::vector<Wall> extractWallsProjection(const CloudT & band, float floor_z,
                                           float z_lo, float z_hi,
                                           float ceiling_z) const
  {
    // Grid bounds in the target frame.
    float min_x = std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float max_x = -std::numeric_limits<float>::max();
    float max_y = -std::numeric_limits<float>::max();
    for (const auto & p : band.points)
    {
      min_x = std::min(min_x, p.x);
      min_y = std::min(min_y, p.y);
      max_x = std::max(max_x, p.x);
      max_y = std::max(max_y, p.y);
    }
    const float res = static_cast<float>(grid_resolution_);
    const int W = static_cast<int>((max_x - min_x) / res) + 1;
    const int H = static_cast<int>((max_y - min_y) / res) + 1;
    if (W <= 1 || H <= 1)
    {
      return {};
    }

    // Per-cell point count + a vertical slice-occupancy bitmask over the band.
    // The mask records WHICH height slices a column has returns in, so we can
    // tell a column densely filled top-to-bottom (a wall) from one filled only
    // at the top (a doorway lintel) or only in the middle (furniture) — the old
    // zmax-zmin extent could not, and so confirmed lintels as full walls.
    const float slice_h = std::max(static_cast<float>(vertical_slice_height_), res);
    int num_slices = static_cast<int>((z_hi - z_lo) / slice_h) + 1;
    num_slices = std::max(1, std::min(num_slices, 32));  // cap to the bitmask width
    std::vector<uint16_t> count(static_cast<size_t>(W) * H, 0);
    std::vector<uint32_t> slices(static_cast<size_t>(W) * H, 0u);
    auto idx = [&](int ix, int iy) { return static_cast<size_t>(iy) * W + ix; };

    for (const auto & p : band.points)
    {
      const int ix = static_cast<int>((p.x - min_x) / res);
      const int iy = static_cast<int>((p.y - min_y) / res);
      if (ix < 0 || ix >= W || iy < 0 || iy >= H)
      {
        continue;
      }
      const size_t c = idx(ix, iy);
      if (count[c] < std::numeric_limits<uint16_t>::max())
      {
        ++count[c];
      }
      int s = static_cast<int>((p.z - z_lo) / slice_h);
      s = std::max(0, std::min(s, num_slices - 1));
      slices[c] |= (1u << s);
    }

    // Rasterize wall-like cells: enough points AND a tall, DENSE vertical column.
    // "Tall"  = occupied slices span >= min_cell_vertical_extent.
    // "Dense" = occupied/span >= vertical_fill_ratio, so a hollow column (lintel
    //           on top + baseboard at the floor, open in between) is rejected
    //           while a wall occluded from the bottom (dense from mid up) passes.
    const int min_span_slices =
      std::max(1, static_cast<int>(std::ceil(min_cell_vertical_extent_ / slice_h)));
    cv::Mat occ(H, W, CV_8UC1, cv::Scalar(0));
    for (int iy = 0; iy < H; ++iy)
    {
      for (int ix = 0; ix < W; ++ix)
      {
        const size_t c = idx(ix, iy);
        if (count[c] < min_points_per_cell_)
        {
          continue;
        }
        const uint32_t m = slices[c];
        if (m == 0u)
        {
          continue;
        }
        const int lo = __builtin_ctz(m);                 // lowest occupied slice
        const int hi = 31 - __builtin_clz(m);            // highest occupied slice
        const int span = hi - lo + 1;
        const int occ_slices = __builtin_popcount(m);
        if (span >= min_span_slices &&
            occ_slices >= static_cast<int>(std::ceil(vertical_fill_ratio_ * span)))
        {
          occ.at<uint8_t>(iy, ix) = 255;
        }
      }
    }

    // OpenCV HoughLinesP works in pixel (=cell) units.
    const double min_len_px = min_wall_length_ / res;
    const double max_gap_px = max_wall_gap_ / res;
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(occ, lines, /*rho=*/1.0, /*theta=*/CV_PI / 180.0,
                    hough_threshold_, min_len_px, max_gap_px);

    // Pixel endpoints -> metric, then score each candidate by its 3D support.
    std::vector<Wall> walls;
    for (const auto & l : lines)
    {
      Wall w;
      w.p1 = Eigen::Vector2f(min_x + (l[0] + 0.5f) * res, min_y + (l[1] + 0.5f) * res);
      w.p2 = Eigen::Vector2f(min_x + (l[2] + 0.5f) * res, min_y + (l[3] + 0.5f) * res);
      if (scoreWall(band, w, floor_z, ceiling_z, z_hi))
      {
        walls.push_back(w);
      }
    }
    std::vector<Wall> merged = mergeWalls(walls);
    for (auto & w : merged)
    {
      refineWall(w);  // Tier B: precise constrained vertical-plane fit
    }
    return merged;
  }

  // ------------------------------------------------------------------
  bool sampleRhtPlane(const CloudT & band, const std::vector<int> & active_indices,
                      std::mt19937 & rng, Eigen::Vector3f & normal, float & rho) const
  {
    if (active_indices.size() < 3)
    {
      return false;
    }

    std::uniform_int_distribution<size_t> pick(0, active_indices.size() - 1);
    for (int attempt = 0; attempt < 12; ++attempt)
    {
      const size_t i0 = pick(rng);
      const size_t i1 = pick(rng);
      const size_t i2 = pick(rng);
      if (i0 == i1 || i0 == i2 || i1 == i2)
      {
        continue;
      }

      const auto & p1 = band.points[active_indices[i0]];
      const auto & p2 = band.points[active_indices[i1]];
      const auto & p3 = band.points[active_indices[i2]];
      const Eigen::Vector3f e1(p1.x, p1.y, p1.z);
      const Eigen::Vector3f e2(p2.x, p2.y, p2.z);
      const Eigen::Vector3f e3(p3.x, p3.y, p3.z);

      auto pair_ok = [&](const Eigen::Vector3f & a, const Eigen::Vector3f & b) {
        const float d = (a - b).norm();
        if (d < static_cast<float>(rht_min_pairwise_dist_))
        {
          return false;
        }
        if (rht_max_pairwise_dist_ > rht_min_pairwise_dist_ &&
            d > static_cast<float>(rht_max_pairwise_dist_))
        {
          return false;
        }
        return true;
      };
      if (!pair_ok(e1, e2) || !pair_ok(e2, e3) || !pair_ok(e1, e3))
      {
        continue;
      }

      const Eigen::Vector3f n_raw = (e3 - e2).cross(e1 - e2);
      const float triangle_area = 0.5f * n_raw.norm();
      if (triangle_area < static_cast<float>(rht_min_triangle_area_))
      {
        continue;
      }

      normal = n_raw.normalized();
      rho = normal.dot(e1);
      if (rho < 0.0f)
      {
        normal = -normal;
        rho = -rho;
      }
      return true;
    }
    return false;
  }

  bool planeToRhtKey(const Eigen::Vector3f & normal, float rho, RhtBinKey & key) const
  {
    const float theta_step = std::max(
      1e-3f, static_cast<float>(rht_theta_bin_size_deg_ * M_PI / 180.0));
    const float phi_step = std::max(
      1e-3f, static_cast<float>(rht_phi_bin_size_deg_ * M_PI / 180.0));
    const float max_phi = static_cast<float>(rht_max_wall_phi_deg_ * M_PI / 180.0);
    const float theta = std::atan2(normal.y(), normal.x());
    const float phi = std::atan2(normal.z(), std::hypot(normal.x(), normal.y()));
    if (std::abs(phi) > max_phi)
    {
      return false;
    }

    const int theta_bins = std::max(1, static_cast<int>(std::ceil((2.0 * M_PI) / theta_step)));
    const int phi_bins = std::max(1, static_cast<int>(std::ceil((2.0 * max_phi) / phi_step)));
    key.theta = static_cast<int>(std::floor((theta + static_cast<float>(M_PI)) / theta_step));
    key.phi = static_cast<int>(std::floor((phi + max_phi) / phi_step));
    key.rho = static_cast<int>(std::floor(rho / std::max(1e-3, rht_rho_bin_size_)));
    key.theta = std::max(0, std::min(key.theta, theta_bins - 1));
    key.phi = std::max(0, std::min(key.phi, phi_bins - 1));
    return true;
  }

  std::vector<std::vector<int>> splitPlaneRuns(const CloudT & band,
                                               const std::vector<int> & plane_indices,
                                               const Eigen::Vector2f & normal) const
  {
    std::vector<std::vector<int>> runs;
    if (plane_indices.empty())
    {
      return runs;
    }

    const Eigen::Vector2f dir(-normal.y(), normal.x());
    float d = 0.0f;
    for (const int idx : plane_indices)
    {
      d += -normal.dot(Eigen::Vector2f(band.points[idx].x, band.points[idx].y));
    }
    d /= static_cast<float>(plane_indices.size());
    const Eigen::Vector2f q0 = -d * normal;

    std::vector<std::pair<float, int>> ordered;
    ordered.reserve(plane_indices.size());
    for (const int idx : plane_indices)
    {
      const Eigen::Vector2f q(band.points[idx].x, band.points[idx].y);
      ordered.emplace_back((q - q0).dot(dir), idx);
    }
    std::sort(ordered.begin(), ordered.end(),
      [](const auto & a, const auto & b) { return a.first < b.first; });

    const float gap_thresh = std::max(
      static_cast<float>(rht_segment_gap_),
      2.0f * static_cast<float>(line_inlier_dist_));
    std::vector<int> current;
    current.reserve(ordered.size());
    current.push_back(ordered.front().second);
    for (size_t i = 1; i < ordered.size(); ++i)
    {
      if ((ordered[i].first - ordered[i - 1].first) > gap_thresh)
      {
        runs.push_back(current);
        current.clear();
      }
      current.push_back(ordered[i].second);
    }
    if (!current.empty())
    {
      runs.push_back(current);
    }
    return runs;
  }

  bool buildWallFromPlaneRun(const CloudT & band, const std::vector<int> & run,
                             const Eigen::Vector2f & normal, Wall & w) const
  {
    if (run.size() < 2)
    {
      return false;
    }

    const Eigen::Vector2f dir(-normal.y(), normal.x());
    float d = 0.0f;
    for (const int idx : run)
    {
      d += -normal.dot(Eigen::Vector2f(band.points[idx].x, band.points[idx].y));
    }
    d /= static_cast<float>(run.size());
    const Eigen::Vector2f q0 = -d * normal;

    std::vector<float> ts;
    ts.reserve(run.size());
    for (const int idx : run)
    {
      const Eigen::Vector2f q(band.points[idx].x, band.points[idx].y);
      ts.push_back((q - q0).dot(dir));
    }

    const size_t klo = static_cast<size_t>(0.02 * (ts.size() - 1));
    const size_t khi = static_cast<size_t>(0.98 * (ts.size() - 1));
    std::nth_element(ts.begin(), ts.begin() + klo, ts.end());
    const float tlo = ts[klo];
    std::nth_element(ts.begin(), ts.begin() + khi, ts.end());
    const float thi = ts[khi];
    if ((thi - tlo) < static_cast<float>(min_wall_length_))
    {
      return false;
    }

    w.p1 = q0 + dir * tlo;
    w.p2 = q0 + dir * thi;
    return true;
  }

  void orientWallTowardViewpoint(Wall & w, const Eigen::Vector2f & stable_normal,
                                 const Eigen::Vector2f & viewpoint,
                                 bool viewpoint_valid) const
  {
    Eigen::Vector2f desired = stable_normal;
    if (viewpoint_valid)
    {
      const Eigen::Vector2f mid = 0.5f * (w.p1 + w.p2);
      if ((viewpoint - mid).dot(desired) < 0.0f)
      {
        desired = -desired;
      }
    }
    w.normal = desired;
  }

  std::vector<int> collectSupportIndices(const CloudT & band,
                                         const std::vector<bool> & active,
                                         const Wall & w) const
  {
    std::vector<int> support;
    const Eigen::Vector2f d = w.p2 - w.p1;
    const float len = d.norm();
    if (len < 1e-3f)
    {
      return support;
    }

    const Eigen::Vector2f dir = d / len;
    const Eigen::Vector2f nrm(-dir.y(), dir.x());
    const float tol = static_cast<float>(line_inlier_dist_);
    for (size_t i = 0; i < band.size(); ++i)
    {
      if (!active[i])
      {
        continue;
      }
      const Eigen::Vector2f q(band.points[i].x, band.points[i].y);
      const Eigen::Vector2f rel = q - w.p1;
      const float t = rel.dot(dir);
      if (t < -tol || t > len + tol)
      {
        continue;
      }
      if (std::abs(rel.dot(nrm)) <= tol)
      {
        support.push_back(static_cast<int>(i));
      }
    }
    return support;
  }

  std::vector<Wall> extractWallsRht3D(const CloudT & band, float floor_z,
                                      float z_lo, float z_hi,
                                      float ceiling_z,
                                      const Eigen::Vector2f & viewpoint,
                                      bool viewpoint_valid) const
  {
    (void)z_lo;
    std::vector<Wall> walls;
    std::vector<bool> active(band.size(), true);
    size_t active_count = band.size();
    std::mt19937 rng(static_cast<uint32_t>(rht_random_seed_));

    int rounds = 0;
    int evaluated_bins = 0;
    // Per-stage rejection tally (for tuning the precision/recall trade-off): how
    // many candidate planes/runs each gate discarded, so a missing wall can be
    // traced to the exact stage that dropped it instead of guessing.
    int voted_bins = 0;      // bins that cleared rht_vote_threshold
    int rej_inliers = 0;     // < rht_min_candidate_inliers within the plane band
    int rej_short_run = 0;   // run shorter than rht_min_segment_points
    int rej_build = 0;       // buildWallFromPlaneRun failed (e.g. < min_wall_length)
    int score_rej[6] = {0, 0, 0, 0, 0, 0};  // scoreWall reasons: [1]len [2]support [3]ceiling [4]blob [5]column
    int rej_support = 0;     // no active support points left to claim
    int accepted_runs = 0;
    while (active_count >= static_cast<size_t>(min_support_points_) &&
           rounds < rht_max_rounds_)
    {
      std::vector<int> active_indices;
      active_indices.reserve(active_count);
      for (size_t i = 0; i < active.size(); ++i)
      {
        if (active[i])
        {
          active_indices.push_back(static_cast<int>(i));
        }
      }
      if (active_indices.size() < 3)
      {
        break;
      }

      std::unordered_map<RhtBinKey, RhtBinValue, RhtBinKeyHash> bins;
      bins.reserve(static_cast<size_t>(rht_max_iterations_ / 2));
      for (int iter = 0; iter < rht_max_iterations_; ++iter)
      {
        Eigen::Vector3f normal;
        float rho = 0.0f;
        if (!sampleRhtPlane(band, active_indices, rng, normal, rho))
        {
          continue;
        }

        RhtBinKey key;
        if (!planeToRhtKey(normal, rho, key))
        {
          continue;
        }
        auto & bin = bins[key];
        bin.votes += 1;
        bin.normal_sum += normal;
        bin.rho_sum += rho;
      }
      if (bins.empty())
      {
        break;
      }

      std::vector<std::pair<RhtBinKey, RhtBinValue>> candidates;
      candidates.reserve(bins.size());
      for (const auto & entry : bins)
      {
        if (entry.second.votes >= rht_vote_threshold_)
        {
          candidates.push_back(entry);
        }
      }
      if (candidates.empty())
      {
        // No bin cleared the vote threshold this round. Do NOT fall back to
        // evaluating every bin: that bypasses rht_vote_threshold entirely and
        // lets the weakest, chance-aligned proposals through (a false-positive
        // source). End the rounds instead.
        break;
      }

      voted_bins += static_cast<int>(candidates.size());
      std::sort(candidates.begin(), candidates.end(),
        [](const auto & a, const auto & b) { return a.second.votes > b.second.votes; });
      if (static_cast<int>(candidates.size()) > rht_top_bins_per_round_)
      {
        candidates.resize(static_cast<size_t>(rht_top_bins_per_round_));
      }

      bool accepted_in_round = false;
      evaluated_bins += static_cast<int>(candidates.size());
      for (const auto & [key, value] : candidates)
      {
        (void)key;
        if (active_count < static_cast<size_t>(min_support_points_))
        {
          break;
        }

        Eigen::Vector3f normal = value.normal_sum;
        if (normal.norm() < 1e-3f)
        {
          continue;
        }
        normal.normalize();
        float rho = value.rho_sum / static_cast<float>(std::max(1, value.votes));
        if (rho < 0.0f)
        {
          normal = -normal;
          rho = -rho;
        }

        std::vector<int> plane_indices;
        plane_indices.reserve(active_indices.size() / 2);
        for (const int idx : active_indices)
        {
          if (!active[static_cast<size_t>(idx)])
          {
            continue;
          }
          const auto & p = band.points[idx];
          const Eigen::Vector3f q(p.x, p.y, p.z);
          if (std::abs(normal.dot(q) - rho) <= static_cast<float>(rht_plane_inlier_dist_))
          {
            plane_indices.push_back(idx);
          }
        }
        if (plane_indices.size() < static_cast<size_t>(rht_min_candidate_inliers_))
        {
          ++rej_inliers;
          continue;
        }

        Eigen::Vector2f normal_xy(normal.x(), normal.y());
        const float nn = normal_xy.norm();
        if (nn < 1e-3f)
        {
          continue;
        }
        normal_xy /= nn;

        const auto runs = splitPlaneRuns(band, plane_indices, normal_xy);
        for (const auto & run : runs)
        {
          if (run.size() < static_cast<size_t>(rht_min_segment_points_))
          {
            ++rej_short_run;
            continue;
          }

          Wall proposal;
          if (!buildWallFromPlaneRun(band, run, normal_xy, proposal))
          {
            ++rej_build;
            continue;
          }
          int reason = 0;
          if (!scoreWall(band, proposal, floor_z, ceiling_z, z_hi, &reason))
          {
            if (reason >= 1 && reason <= 5) { ++score_rej[reason]; }
            continue;
          }
          orientWallTowardViewpoint(proposal, normal_xy, viewpoint, viewpoint_valid);

          const std::vector<int> support = collectSupportIndices(band, active, proposal);
          if (support.empty())
          {
            ++rej_support;
            continue;
          }
          for (const int idx : support)
          {
            if (active[static_cast<size_t>(idx)])
            {
              active[static_cast<size_t>(idx)] = false;
              --active_count;
            }
          }
          walls.push_back(proposal);
          ++accepted_runs;
          accepted_in_round = true;
        }
      }

      if (!accepted_in_round)
      {
        break;
      }
      ++rounds;
    }

    std::vector<Wall> merged = mergeWalls(walls);
    for (auto & w : merged)
    {
      refineWall(w);
    }

    // Per-stage funnel (throttled INFO so it is visible during tuning without
    // enabling debug logging). Read left to right: of the bins that cleared the
    // vote threshold and were evaluated, how many candidates each gate dropped,
    // and how many runs survived to become walls. A wall you expected but don't
    // see is dying at whichever bucket is large: rej_inliers -> too sparse for
    // rht_min_candidate_inliers / rht_plane_inlier_dist; score.ceiling -> not
    // reaching the ceiling (min_high_support / band_max_height); score.column ->
    // full-height support too localized (min_ceiling_support_length); etc.
    RCLCPP_INFO_THROTTLE(get_logger(), rht_log_clock_, 2000,
      "rht_3d funnel: rounds=%d voted_bins=%d evaluated=%d | rej_inliers=%d "
      "rej_short_run=%d rej_build=%d | score_rej len=%d support=%d ceiling=%d "
      "blob=%d column=%d | rej_support=%d accepted_runs=%d -> walls=%zu (active_left=%zu)",
      rounds, voted_bins, evaluated_bins, rej_inliers, rej_short_run, rej_build,
      score_rej[1], score_rej[2], score_rej[3], score_rej[4], score_rej[5],
      rej_support, accepted_runs, merged.size(), active_count);
    return merged;
  }

  // ------------------------------------------------------------------
  // Tier B: fit a vertical plane to the wall's pooled support via RANSAC
  // constrained PARALLEL to the gravity axis (so the plane stays vertical and
  // can't tilt onto floor/ceiling clutter). On success, refines the normal,
  // plane offset, along-wall endpoints and height extent to the actual inlier
  // geometry. On failure the Tier-A line geometry is kept (refined=false).
  void refineWall(Wall & w) const
  {
    if (!w.inliers || w.inliers->size() < static_cast<size_t>(min_plane_inliers_))
    {
      return;
    }

    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(ransac_max_iter_);
    seg.setDistanceThreshold(plane_dist_thresh_);
    seg.setAxis(Eigen::Vector3f::UnitZ());  // plane parallel to gravity = vertical
    seg.setEpsAngle(plane_eps_angle_deg_ * M_PI / 180.0);
    seg.setInputCloud(w.inliers);

    pcl::PointIndices::Ptr idx(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
    seg.segment(*idx, *coeff);
    if (idx->indices.size() < static_cast<size_t>(min_plane_inliers_) ||
        coeff->values.size() < 4)
    {
      return;  // keep Tier-A geometry
    }

    // Force the normal exactly horizontal (drop the tiny z left by eps_angle).
    Eigen::Vector2f n(coeff->values[0], coeff->values[1]);
    const float nn = n.norm();
    if (nn < 1e-3f)
    {
      return;
    }
    n /= nn;
    // Keep the Tier-A orientation (toward the support/robot side).
    if (n.dot(w.normal) < 0.0f)
    {
      n = -n;
    }

    // Plane inlier centroid + recomputed offset for the horizontal normal.
    Eigen::Vector2f c(0.0f, 0.0f);
    float zmin = std::numeric_limits<float>::max();
    float zmax = -std::numeric_limits<float>::max();
    for (int i : idx->indices)
    {
      const auto & p = w.inliers->points[i];
      c += Eigen::Vector2f(p.x, p.y);
      zmin = std::min(zmin, p.z);
      zmax = std::max(zmax, p.z);
    }
    c /= static_cast<float>(idx->indices.size());

    // Along-wall extent: project plane inliers onto the in-plane horizontal dir.
    const Eigen::Vector2f dir(-n.y(), n.x());
    float tmin = std::numeric_limits<float>::max();
    float tmax = -std::numeric_limits<float>::max();
    for (int i : idx->indices)
    {
      const auto & p = w.inliers->points[i];
      const float t = (Eigen::Vector2f(p.x, p.y) - c).dot(dir);
      tmin = std::min(tmin, t);
      tmax = std::max(tmax, t);
    }

    w.normal = n;
    w.plane_d = -(n.dot(c));   // n . x + d = 0 through the centroid
    w.p1 = c + dir * tmin;
    w.p2 = c + dir * tmax;
    w.length = tmax - tmin;
    w.z_min = zmin;
    w.z_max = zmax;
    w.support = static_cast<int>(idx->indices.size());
    w.confidence = std::min(1.0f, static_cast<float>(w.support) / (w.length * 100.0f));
    w.refined = true;
  }

  // ------------------------------------------------------------------
  // Gather the band points within line_inlier_dist of the (finite) segment;
  // fill length / normal / z extent / support / confidence. Returns false if
  // the candidate is too short or under-supported.
  // reject_reason (optional, for rht_3d tuning instrumentation): on a false
  // return it is set to which gate rejected the candidate -- 1=length,
  // 2=support, 3=ceiling-reach, 4=blob/mass, 5=column. 0 = accepted.
  bool scoreWall(const CloudT & band, Wall & w, float floor_z, float ceiling_z,
                 float z_hi, int * reject_reason = nullptr) const
  {
    if (reject_reason) { *reject_reason = 0; }
    const Eigen::Vector2f d = w.p2 - w.p1;
    const float len = d.norm();
    if (len < static_cast<float>(min_wall_length_))
    {
      if (reject_reason) { *reject_reason = 1; }
      return false;
    }
    const Eigen::Vector2f dir = d / len;
    const Eigen::Vector2f nrm(-dir.y(), dir.x());

    // Ceiling-reach threshold (used by the furniture gate below AND by the
    // both-sides mass test), computed up-front so the band scan can tally *tall*
    // returns beside the wall in a single pass. Anchored to the LOWER of the true
    // ceiling and the band cap so a ceiling above the band can still be reached;
    // falls back to a fixed height when no clean ceiling was found.
    const float effective_top = std::min(ceiling_z, z_hi);
    const bool ceiling_valid =
      (effective_top - floor_z) >= static_cast<float>(min_wall_height_);
    const float top_thresh = ceiling_valid
      ? (effective_top - static_cast<float>(ceiling_gap_))
      : (floor_z + static_cast<float>(min_wall_height_));

    int support = 0;
    float signed_sum = 0.0f;
    const float tol = static_cast<float>(line_inlier_dist_);
    const float shell_in = static_cast<float>(blob_shell_inner_);
    const float shell_out = static_cast<float>(blob_shell_outer_);
    int shell_plus_tall = 0, shell_minus_tall = 0;  // ceiling-reaching returns beside the wall
    w.inliers.reset(new CloudT);
    w.inliers->reserve(band.size() / 4);
    std::vector<float> zs;
    zs.reserve(band.size() / 4);

    for (const auto & p : band.points)
    {
      const Eigen::Vector2f q(p.x, p.y);
      const Eigen::Vector2f rel = q - w.p1;
      const float t = rel.dot(dir);
      if (t < -tol || t > len + tol)
      {
        continue;  // outside the segment span
      }
      const float perp = rel.dot(nrm);
      const float ap = std::abs(perp);
      if (ap <= tol)
      {
        ++support;
        signed_sum += perp;
        zs.push_back(p.z);
        w.inliers->push_back(p);  // kept for the Tier-B plane fit
      }
      else if (blob_side_ratio_ > 0.0 && ap > shell_in && ap <= shell_out &&
               p.z >= top_thresh)
      {
        // A ceiling-reaching return in the perpendicular shell beside the wall;
        // a real wall has ~none of these, a solid mass has them on both sides.
        (perp > 0.0f ? shell_plus_tall : shell_minus_tall) += 1;
      }
    }
    if (support < min_support_points_)
    {
      if (reject_reason) { *reject_reason = 2; }
      return false;
    }

    w.length = len;
    // Orient the normal toward the side the support sits on (typically the
    // robot side), so the scanning planner can offset a standoff pose outward.
    w.normal = (signed_sum >= 0.0f) ? nrm : -nrm;
    // Robust height: 2nd/98th percentile of support z, not the raw min/max. A
    // single stray return (a lintel point clipped in near a doorway edge, or a
    // floor speckle) would otherwise stretch the rendered quad floor-to-ceiling
    // across open air.
    {
      const size_t klo = static_cast<size_t>(0.02 * (zs.size() - 1));
      const size_t khi = static_cast<size_t>(0.98 * (zs.size() - 1));
      std::nth_element(zs.begin(), zs.begin() + klo, zs.end());
      const float zmin = zs[klo];
      std::nth_element(zs.begin(), zs.begin() + khi, zs.end());
      const float zmax = zs[khi];
      w.z_min = zmin;
      w.z_max = zmax;
    }
    // Furniture rejection by CEILING REACH (ceiling-anchored, occlusion-safe).
    // A wall is a vertical plane that runs to the ceiling; tables, drawer rows,
    // cabinets, cupboards and shelves all top out BELOW it. So instead of a
    // fixed height, require the support to actually reach the detected room
    // ceiling. This is robust to ceiling height (a 1.9 m cupboard under a 2.7 m
    // ceiling fails; a fixed 1.8 m gate would pass it) and, by requiring a COUNT
    // of points near the ceiling rather than the percentile z_max alone, robust
    // to a few stray tall returns (clutter behind, accumulation smear) that
    // would otherwise inflate z_max past the gate. Gating the TOP (not the full
    // floor->ceiling span) stays occlusion-safe: a wall hidden low by furniture
    // still reaches the ceiling, whereas a short box never does.
    //
    // The band is clipped at z_hi (= floor + band_max_height), so if the ceiling
    // sits ABOVE the band cap no support point can ever reach it and every wall
    // would be rejected. Anchor to what we can actually observe: the LOWER of
    // the true ceiling and the band cap. (Raise band_max_height above the real
    // ceiling so the anchor uses the true ceiling rather than the band cap.)
    //
    // Fallback: if no clean ceiling was found (open/sloped space, ceiling out of
    // the lidar's view), ceiling-floor is too small to trust, so revert to the
    // fixed min_wall_height top-reach test. (top_thresh was computed above.)
    int high_support = 0;
    for (const float z : zs)
    {
      if (z >= top_thresh)
      {
        ++high_support;
      }
    }
    if (min_high_support_ > 0)
    {
      if (high_support < min_high_support_)
      {
        if (reject_reason) { *reject_reason = 3; }
        return false;  // furniture: doesn't reach the ceiling with real support
      }
    }
    else if (w.z_max < top_thresh)
    {
      if (reject_reason) { *reject_reason = 3; }
      return false;
    }

    // MASS/BLOB rejection (the fix for lines fit through a solid, ceiling-height
    // mass — cluttered corner, filled area, stacked material). A real wall is a
    // thin plane with open space on at least one side; a mass has ceiling-reaching
    // returns on BOTH sides. If both perpendicular shells beside the wall are as
    // populated as the wall's OWN tall support, the "wall" is really a slice
    // through a mass -> reject. Occlusion-safe: a genuine wall keeps one side open.
    if (blob_side_ratio_ > 0.0 && high_support > 0)
    {
      const int need = std::max(blob_min_side_,
        static_cast<int>(std::ceil(blob_side_ratio_ * high_support)));
      if (shell_plus_tall >= need && shell_minus_tall >= need)
      {
        if (reject_reason) { *reject_reason = 4; }
        return false;  // embedded in a wide mass, not an isolated wall
      }
    }

    // COLUMN rejection. The ceiling test above only counts HOW MANY support
    // points reach the ceiling, not WHERE along the wall they are — so a column
    // (a dense, full-height return concentrated in a ~0.5 m footprint) satisfies
    // it for the whole segment, and a collinear grid line or a low drawer row
    // can stretch that segment metres past the actual column. A real wall has
    // its ceiling-reaching support DISTRIBUTED along its length. So require the
    // along-wall extent of the ceiling-reaching support (robust 5-95 percentile,
    // to ignore a couple of strays) to span at least min_ceiling_support_length;
    // if the full-height support is localized, it's a column, not a wall.
    if (min_ceiling_support_length_ > 0.0)
    {
      std::vector<float> hi_t;
      hi_t.reserve(w.inliers->size());
      for (const auto & p : w.inliers->points)
      {
        if (p.z >= top_thresh)
        {
          hi_t.push_back((Eigen::Vector2f(p.x, p.y) - w.p1).dot(dir));
        }
      }
      if (hi_t.size() < 2)
      {
        if (reject_reason) { *reject_reason = 5; }
        return false;
      }
      const size_t klo = static_cast<size_t>(0.05 * (hi_t.size() - 1));
      const size_t khi = static_cast<size_t>(0.95 * (hi_t.size() - 1));
      std::nth_element(hi_t.begin(), hi_t.begin() + klo, hi_t.end());
      const float tlo = hi_t[klo];
      std::nth_element(hi_t.begin(), hi_t.begin() + khi, hi_t.end());
      const float thi = hi_t[khi];
      if ((thi - tlo) < static_cast<float>(min_ceiling_support_length_))
      {
        if (reject_reason) { *reject_reason = 5; }
        return false;  // full-height support localized -> column, not a wall
      }
    }

    w.support = support;
    // Confidence: support density along the wall (points per meter of length),
    // saturated to [0,1]. A solid wall returns far more than 100 pts/m.
    w.confidence = std::min(1.0f, static_cast<float>(support) / (len * 100.0f));
    return true;
  }

  // ------------------------------------------------------------------
  // HoughLinesP can fragment one wall into several near-collinear segments.
  // Greedily merge segments with similar orientation that are close in the
  // perpendicular direction, keeping the widest extent.
  std::vector<Wall> mergeWalls(std::vector<Wall> in) const
  {
    const float ang_tol = static_cast<float>(merge_angle_deg_) * static_cast<float>(M_PI) / 180.0f;
    std::vector<Wall> out;
    std::vector<bool> used(in.size(), false);

    for (size_t i = 0; i < in.size(); ++i)
    {
      if (used[i])
      {
        continue;
      }
      Wall acc = in[i];
      used[i] = true;
      Eigen::Vector2f dir = (acc.p2 - acc.p1).normalized();

      for (size_t j = i + 1; j < in.size(); ++j)
      {
        if (used[j])
        {
          continue;
        }
        const Eigen::Vector2f dj = (in[j].p2 - in[j].p1).normalized();
        const float cosang = std::abs(dir.dot(dj));
        if (cosang < std::cos(ang_tol))
        {
          continue;  // different orientation
        }
        const Eigen::Vector2f nrm(-dir.y(), dir.x());
        const float perp = std::abs((in[j].p1 - acc.p1).dot(nrm));
        if (perp > static_cast<float>(merge_dist_))
        {
          continue;  // parallel but offset -> a different wall
        }
        // ALONG-WALL gap gate. Collinear + close perpendicularly is not enough:
        // two separate walls flanking an open passage (doorway/corridor) are both
        // of those, and merging them bridges a wall straight across the opening.
        // Only merge fragments that actually overlap or sit within max_wall_gap
        // of each other along the wall; a wider gap means distinct walls, kept
        // separate (mirrors the doorway-splitting the projection path relies on).
        {
          const float ta1 = acc.length;  // acc spans [0, length] along dir from acc.p1
          const float tb0 = (in[j].p1 - acc.p1).dot(dir);
          const float tb1 = (in[j].p2 - acc.p1).dot(dir);
          const float blo = std::min(tb0, tb1), bhi = std::max(tb0, tb1);
          const float overlap = std::min(ta1, bhi) - std::max(0.0f, blo);
          if (overlap < -static_cast<float>(max_wall_gap_))
          {
            continue;  // collinear but separated by an open passage -> keep split
          }
        }
        // Merge: take the extreme endpoints along dir.
        std::array<Eigen::Vector2f, 4> pts{acc.p1, acc.p2, in[j].p1, in[j].p2};
        float tmin = std::numeric_limits<float>::max();
        float tmax = -std::numeric_limits<float>::max();
        Eigen::Vector2f a = acc.p1, b = acc.p2;
        for (const auto & pt : pts)
        {
          const float t = (pt - acc.p1).dot(dir);
          if (t < tmin) { tmin = t; a = pt; }
          if (t > tmax) { tmax = t; b = pt; }
        }
        acc.p1 = a;
        acc.p2 = b;
        acc.length = (b - a).norm();
        acc.support += in[j].support;
        acc.z_min = std::min(acc.z_min, in[j].z_min);
        acc.z_max = std::max(acc.z_max, in[j].z_max);
        acc.confidence = std::min(1.0f, static_cast<float>(acc.support) / (acc.length * 100.0f));
        if (acc.inliers && in[j].inliers)
        {
          *acc.inliers += *in[j].inliers;  // pool support for the plane fit
        }
        used[j] = true;
      }
      out.push_back(acc);
    }
    return out;
  }

  // ------------------------------------------------------------------
  void publishWalls(const std::vector<Wall> & walls, const builtin_interfaces::msg::Time & stamp)
  {
    visualization_msgs::msg::MarkerArray ma;

    // DELETEALL clears stale walls from the previous frame.
    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = target_frame_;
    clear.header.stamp = stamp;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    ma.markers.push_back(clear);

    geometry_msgs::msg::PoseArray poses;
    poses.header.frame_id = target_frame_;
    poses.header.stamp = stamp;

    robo_drill::msg::WallArray wall_array;
    wall_array.header.frame_id = target_frame_;
    wall_array.header.stamp = stamp;

    int id = 0;
    for (const auto & w : walls)
    {
      // Structured output for the scanning planner.
      robo_drill::msg::Wall wm;
      wm.start.x = w.p1.x();   wm.start.y = w.p1.y();   wm.start.z = w.z_min;
      wm.end.x = w.p2.x();     wm.end.y = w.p2.y();     wm.end.z = w.z_min;
      wm.normal.x = w.normal.x();  wm.normal.y = w.normal.y();  wm.normal.z = 0.0;
      wm.d = w.plane_d;
      wm.length = w.length;
      wm.height = w.z_max - w.z_min;
      wm.z_min = w.z_min;
      wm.z_max = w.z_max;
      wm.confidence = w.confidence;
      wm.inliers = w.support;
      wm.refined = w.refined;
      wall_array.walls.push_back(wm);

      // Wall face as a vertical quad (LINE_LIST outline). Green = Tier-B plane
      // fit succeeded (metric plane valid); cyan = Tier-A line geometry only.
      visualization_msgs::msg::Marker face;
      face.header.frame_id = target_frame_;
      face.header.stamp = stamp;
      face.ns = "wall_face";
      face.id = id;
      face.type = visualization_msgs::msg::Marker::LINE_LIST;
      face.action = visualization_msgs::msg::Marker::ADD;
      face.scale.x = 0.03;
      face.color.a = 0.9f;
      face.color.r = w.refined ? 0.1f : 0.1f;
      face.color.g = w.refined ? 1.0f : 0.8f;
      face.color.b = w.refined ? 0.3f : 1.0f;
      face.pose.orientation.w = 1.0;
      auto corner = [&](const Eigen::Vector2f & xy, float z) {
        geometry_msgs::msg::Point pt;
        pt.x = xy.x();
        pt.y = xy.y();
        pt.z = z;
        return pt;
      };
      const auto bl = corner(w.p1, w.z_min);
      const auto br = corner(w.p2, w.z_min);
      const auto tl = corner(w.p1, w.z_max);
      const auto tr = corner(w.p2, w.z_max);
      for (const auto & seg : {std::pair{bl, br}, std::pair{tl, tr},
                               std::pair{bl, tl}, std::pair{br, tr}})
      {
        face.points.push_back(seg.first);
        face.points.push_back(seg.second);
      }
      ma.markers.push_back(face);

      // Midpoint pose, +X along the wall normal -> standoff direction.
      const Eigen::Vector2f mid = 0.5f * (w.p1 + w.p2);
      const float z_mid = 0.5f * (w.z_min + w.z_max);
      const float yaw = std::atan2(w.normal.y(), w.normal.x());

      geometry_msgs::msg::Pose pose;
      pose.position.x = mid.x();
      pose.position.y = mid.y();
      pose.position.z = z_mid;
      pose.orientation.z = std::sin(yaw / 2.0f);
      pose.orientation.w = std::cos(yaw / 2.0f);
      poses.poses.push_back(pose);

      // Normal arrow.
      visualization_msgs::msg::Marker arrow;
      arrow.header.frame_id = target_frame_;
      arrow.header.stamp = stamp;
      arrow.ns = "wall_normal";
      arrow.id = id;
      arrow.type = visualization_msgs::msg::Marker::ARROW;
      arrow.action = visualization_msgs::msg::Marker::ADD;
      arrow.scale.x = 0.5;   // length
      arrow.scale.y = 0.05;
      arrow.scale.z = 0.05;
      arrow.color.a = 0.9f;
      arrow.color.r = 1.0f;
      arrow.color.g = 0.5f;
      arrow.color.b = 0.0f;
      arrow.pose = pose;
      ma.markers.push_back(arrow);

      ++id;
    }

    pub_markers_->publish(ma);
    pub_poses_->publish(poses);
    pub_walls_->publish(wall_array);
  }

  void publishDebug(const CloudT::Ptr & band, const builtin_interfaces::msg::Time & stamp)
  {
    sensor_msgs::msg::PointCloud2 out;
    pcl::toROSMsg(*band, out);
    out.header.frame_id = target_frame_;
    out.header.stamp = stamp;
    pub_debug_->publish(out);
  }

  // ---- params ----
  std::string input_topic_, target_frame_;
  std::string detector_mode_;
  double accumulation_window_;
  double tf_timeout_;
  double max_tf_stale_;
  // ---- motion gate ----
  bool motion_gate_;
  std::string odom_topic_;
  double max_angular_vel_;
  double motion_settle_time_;
  double voxel_leaf_, floor_percentile_, band_min_height_, band_max_height_;
  double ceiling_percentile_, ceiling_gap_;
  double grid_resolution_, min_cell_vertical_extent_;
  double vertical_slice_height_, vertical_fill_ratio_;
  int min_points_per_cell_;
  double min_wall_length_, min_wall_height_, max_wall_gap_, line_inlier_dist_, merge_angle_deg_, merge_dist_;
  double min_ceiling_support_length_;
  double blob_shell_inner_, blob_shell_outer_, blob_side_ratio_;
  int blob_min_side_;
  int hough_threshold_, min_support_points_, min_high_support_;
  int rht_max_iterations_, rht_max_rounds_, rht_vote_threshold_, rht_top_bins_per_round_;
  int rht_min_candidate_inliers_, rht_min_segment_points_, rht_random_seed_;
  double rht_rho_bin_size_, rht_theta_bin_size_deg_, rht_phi_bin_size_deg_;
  double rht_max_wall_phi_deg_, rht_min_pairwise_dist_, rht_max_pairwise_dist_;
  double rht_min_triangle_area_, rht_plane_inlier_dist_, rht_segment_gap_;
  double plane_dist_thresh_, plane_eps_angle_deg_;
  int ransac_max_iter_, min_plane_inliers_;
  bool publish_debug_cloud_;

  // Rolling window of recent voxelized clouds in the (fixed) target frame.
  std::deque<std::pair<rclcpp::Time, CloudT::Ptr>> buffer_;

  // Motion-gate state: last time the robot was turning too fast (system clock).
  rclcpp::Time last_rotating_{0, 0, RCL_ROS_TIME};
  bool have_odom_{false};

  // ---- ROS ----
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_poses_;
  rclcpp::Publisher<robo_drill::msg::WallArray>::SharedPtr pub_walls_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_debug_;

  // Own clock for throttling the rht_3d funnel log from the const extractor
  // (Node::get_clock() yields a const clock in a const method, which the
  // throttle macro can't advance). mutable so it works from a const method.
  mutable rclcpp::Clock rht_log_clock_{RCL_STEADY_TIME};
};

}  // namespace robo_drill

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robo_drill::WallDetectionNode>());
  rclcpp::shutdown();
  return 0;
}
