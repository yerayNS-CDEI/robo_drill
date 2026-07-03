// Copyright 2026 robo_drill
//
// Tier-C persistent wall aggregator.
//
// NOT a SLAM / occupancy mapper and NOT a replacement for the RTAB-Map node in
// rtabmap.launch.py. RTAB-Map owns localization (the map<-odom TF) and the
// occupancy/3D map; this node *consumes* that localization and adds only a thin
// semantic layer on top: it remembers WHERE THE WALL PLANES ARE (normal, offset,
// extent, height, openings) in RTAB-Map's `map` frame — geometry the occupancy
// map doesn't provide and the arm scanning planner needs.
//
// The Tier-A/B detector (wall_detection_node) is an *instantaneous* detector: it
// reports the walls currently in view, in the smooth `odom` frame. This node
// turns that stream into a persistent set of wall objects:
//
//   * Each incoming WallArray is transformed odom -> map at its stamp, so walls
//     are anchored in the fixed `map` frame and survive the robot driving on.
//     Because detection stays in smooth odom and only the *result* is dropped
//     into map, a map<-odom loop-closure jump relocates the live detections
//     without disturbing already-mapped walls.
//   * Each detection is associated to an existing persistent wall (same plane:
//     near-parallel normal, small perpendicular offset, overlapping extent) and
//     fused (inlier-weighted plane update + union of extent), or seeded as a new
//     provisional wall. A wall is "confirmed" once seen >= min_observations
//     times, which rejects transient false detections.
//   * Optionally (use_openings, gated on a valid YOLO hand-eye calibration) the
//     detected 3D door/window centers are associated to their wall and reported
//     in Wall.openings, so the scan planner knows where the holes are.
//
// Output: robo_drill/WallArray on ~/persistent_walls (map frame, confirmed) + markers.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <opencv2/imgproc.hpp>

#include <Eigen/Dense>

#include "robo_drill/msg/wall.hpp"
#include "robo_drill/msg/wall_array.hpp"

namespace robo_drill
{

// A persistent wall in the map frame.
struct PWall
{
  Eigen::Vector2f normal;   // unit horizontal normal
  float d{0.0f};            // normal . x + d = 0
  Eigen::Vector2f p1, p2;   // endpoints (at z_min)
  float z_min{0.0f};
  float z_max{0.0f};
  int observations{0};
  long total_inliers{0};
  float confidence{0.0f};
  bool refined{true};       // lidar-confirmed height (else grid-only layout)
  rclcpp::Time last_seen{0, 0, RCL_ROS_TIME};
  int swept_unseen{0};      // in-range cycles with no fresh detection (decay)
  int id{0};
  std::vector<Eigen::Vector2f> openings;  // door/window centers (xy, map)
};

// A single detection already transformed into the map frame.
struct DWall
{
  Eigen::Vector2f normal;
  float d{0.0f};
  Eigen::Vector2f p1, p2;
  float z_min{0.0f};
  float z_max{0.0f};
  int inliers{0};
};

// A wall line extracted from RTAB-Map's optimized occupancy grid (drift-free,
// loop-closure-corrected XY layout). Height is supplied later from the lidar.
struct GridWall
{
  Eigen::Vector2f p1, p2;
  Eigen::Vector2f normal;   // perpendicular to the segment (sign fixed at output)
  float d{0.0f};
  int id{0};
  rclcpp::Time last_seen;
  // Update cycles the robot spent within lidar range of this wall. Once high
  // enough with no lidar confirmation, the wall is retracted as a false positive.
  int swept{0};
};

class WallAggregatorNode : public rclcpp::Node
{
public:
  WallAggregatorNode()
  : Node("wall_aggregator_node")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/wall_detection_node/walls");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");

    assoc_angle_deg_ = declare_parameter<double>("assoc_angle_deg", 12.0);
    assoc_dist_ = declare_parameter<double>("assoc_dist", 0.25);
    assoc_overlap_gap_ = declare_parameter<double>("assoc_overlap_gap", 1.0);
    min_observations_ = declare_parameter<int>("min_observations", 3);
    provisional_timeout_ = declare_parameter<double>("provisional_timeout", 5.0);

    use_openings_ = declare_parameter<bool>("use_openings", false);
    openings_topic_ = declare_parameter<std::string>(
      "openings_topic", "/yolo_detection_node/detected_openings");
    opening_assoc_dist_ = declare_parameter<double>("opening_assoc_dist", 0.35);

    // Grid anchoring: take the drift-free wall LAYOUT from RTAB-Map's optimized
    // occupancy grid, and use the lidar walls only to confirm verticality and
    // supply height. The grid is globally consistent (loop-closure-corrected),
    // so output walls no longer wobble with instantaneous ICP/odom drift.
    use_grid_ = declare_parameter<bool>("use_grid", false);
    grid_topic_ = declare_parameter<std::string>("grid_topic", "/map");
    grid_occupied_thresh_ = declare_parameter<int>("grid_occupied_thresh", 50);
    grid_min_wall_length_ = declare_parameter<double>("grid_min_wall_length", 1.0);
    grid_max_wall_gap_ = declare_parameter<double>("grid_max_wall_gap", 0.4);
    grid_hough_threshold_ = declare_parameter<int>("grid_hough_threshold", 15);
    grid_match_dist_ = declare_parameter<double>("grid_match_dist", 0.40);
    grid_match_angle_deg_ = declare_parameter<double>("grid_match_angle_deg", 15.0);
    grid_prune_timeout_ = declare_parameter<double>("grid_prune_timeout", 30.0);
    // Structural validation of each raw Hough line before it becomes a wall.
    // HoughLinesP happily draws a line through any collinear scatter (furniture
    // clusters, speckle, diagonal clutter); requiring the cells UNDER the line
    // to be genuinely occupied rejects those. fill_ratio = occupied / sampled
    // cells along the segment.
    grid_min_fill_ratio_ = declare_parameter<double>("grid_min_fill_ratio", 0.6);
    // BLOB rejection. fill_ratio alone can't tell a thin wall from a line drawn
    // THROUGH a solid occupied mass (furniture cluster, mapping smear): both are
    // fully occupied ALONG the line. A real wall is thin perpendicular to its
    // length — free/unknown space on its room side — whereas a blob is occupied on
    // BOTH sides. So reject a segment whose cells are solid to both sides out to
    // grid_max_wall_thickness for more than grid_max_interior_ratio of its length.
    // grid_max_wall_thickness = how far off the line we probe (a bit beyond a
    // plausible wall's grid thickness); 0 disables the test.
    grid_max_wall_thickness_ = declare_parameter<double>("grid_max_wall_thickness", 0.30);
    grid_max_interior_ratio_ = declare_parameter<double>("grid_max_interior_ratio", 0.4);
    // Crossing-line suppression: two NON-parallel output walls (normals differing
    // by more than cross_min_angle_deg) that intersect at a point interior to BOTH
    // (>= cross_endpoint_margin from every endpoint) can't both be real walls — a
    // real corner meets at endpoints, not through the middle. Keep the stronger,
    // drop the other. Fixes one wall detected as two crossing lines and furniture-
    // face lines through a cluttered region. cross_endpoint_margin 0 disables.
    cross_min_angle_deg_ = declare_parameter<double>("cross_min_angle_deg", 15.0);
    cross_endpoint_margin_ = declare_parameter<double>("cross_endpoint_margin", 0.5);
    // Opening carving: a run of KNOWN-FREE cells this long along the line is a
    // real gap (doorway/passage the robot saw through), so the line is split
    // there instead of bridging a solid wall across the opening.
    grid_opening_min_run_ = declare_parameter<double>("grid_opening_min_run", 0.6);
    // Max along-wall gap that still gets bridged into one continuous wall.
    // Larger -> doorways/notches are bridged (fewer, longer walls); smaller ->
    // walls split at gaps (more, shorter walls). Tune to taste.
    grid_merge_gap_ = declare_parameter<double>("grid_merge_gap", 1.2);
    // If false, emit the full grid layout immediately (walls not yet swept by
    // the lidar are grid-only: refined=false, default height). If true, withhold
    // a grid wall until the lidar confirms it as a tall wall.
    require_lidar_confirmation_ = declare_parameter<bool>("require_lidar_confirmation", false);
    default_z_min_ = declare_parameter<double>("default_z_min", 0.0);
    default_z_max_ = declare_parameter<double>("default_z_max", 2.5);

    // Grey-wall retraction: drop a provisional grid wall once the robot has been
    // within lidar_confirm_range on >= reject_after_sweeps cycles with no lidar
    // confirmation. 0 disables retraction (grey walls persist as before).
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    lidar_confirm_range_ = declare_parameter<double>("lidar_confirm_range", 4.0);
    reject_after_sweeps_ = declare_parameter<int>("reject_after_sweeps", 20);
    // Sweep-gated demotion of CONFIRMED walls: if the robot is in range of a
    // confirmed wall but no fresh detection refreshes it within support_timeout
    // for this many in-range cycles, prune it (stale/false confirmation). 0 keeps
    // the old "confirmed walls persist forever" behaviour.
    decay_after_sweeps_ = declare_parameter<int>("decay_after_sweeps", 20);
    support_timeout_ = declare_parameter<double>("support_timeout", 3.0);

    // Snap confirmed grid walls perpendicular onto the lidar plane (corrects the
    // ~0.2 m grid-vs-surface offset). snap_alpha is the EMA weight on the latest
    // observation (lower = smoother/slower).
    snap_to_lidar_plane_ = declare_parameter<bool>("snap_to_lidar_plane", true);
    snap_alpha_ = declare_parameter<double>("snap_alpha", 0.3);
    // Max perpendicular shift the snap may apply, so a bad association can't fling
    // a wall. Decoupled from grid_match_dist (the *association* gate) so a wall can
    // associate tightly yet still be pulled further onto the lidar plane. Was
    // hard-wired to grid_match_dist; 0 falls back to that for compatibility.
    snap_max_shift_ = declare_parameter<double>("snap_max_shift", 0.0);

    // Fuse-weight ceiling: total_inliers accumulates forever, so after many
    // observations the persistent plane freezes (wP >> wW) and can no longer be
    // pulled toward fresh detections — a wall first fused at a slightly wrong
    // offset stays there, and the snap then targets that stale plane. Clamping the
    // accumulated weight used in the plane update keeps a confirmed wall's plane
    // responsive to new observations. 0 disables the clamp (freeze as before).
    max_fuse_inliers_ = declare_parameter<int>("max_fuse_inliers", 300);

    // Localization-settling gate: pause wall ingestion/confirmation while the
    // robot rotates (cloud lags the map) and for a settle window afterwards.
    confirm_gate_ = declare_parameter<bool>("confirm_gate", true);
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/rtabmap/odom");
    confirm_max_angular_vel_ = declare_parameter<double>("confirm_max_angular_vel", 0.3);
    confirm_settle_time_ = declare_parameter<double>("confirm_settle_time", 0.5);
    // Yaw-covariance ceiling from the odom message; 0 disables the covariance
    // gate (tune to your icp_odometry covariance scale using the odom log line).
    confirm_max_cov_ = declare_parameter<double>("confirm_max_cov", 0.0);

    // ---- persistence to file ----
    // Save the confirmed persistent walls to a file as the robot drives. The
    // aggregator already dedups (each wall has a stable id and its coordinates
    // are updated in place on every fresh association), so we snapshot the whole
    // confirmed set keyed by id: earlier walls are kept, a re-observed wall
    // overwrites its own entry (coordinates updated, not duplicated), and a
    // newly confirmed wall is appended. The full file is rewritten atomically
    // only when the set actually changed, so a stationary robot doesn't churn
    // the disk. On startup an existing file is loaded back in (see below), so
    // walls persist across runs instead of being rewritten from scratch.
    save_walls_to_file_ = declare_parameter<bool>("save_walls_to_file", true);
    // File name (or absolute path). A relative value is resolved into the
    // package's rgb_detections/ folder, alongside the YOLO detection CSVs, so
    // the walls map lives with the rest of the perception output.
    wall_file_path_ = declare_parameter<std::string>(
      "wall_file_path", "detected_walls.yaml");
    // Coordinates are quantized to this (m) when deciding whether the set
    // "changed" enough to rewrite, so sub-mm jitter on a re-fused wall doesn't
    // trigger a write every frame.
    wall_file_epsilon_ = declare_parameter<double>("wall_file_epsilon", 0.01);
    if (save_walls_to_file_)
    {
      wall_file_path_ = resolveWallFilePath(wall_file_path_);
      // Load an existing map so walls survive a restart. The loaded walls seed
      // walls_ as already-confirmed; every incoming detection then runs through
      // fuse() against them — matching one updates it in place, a new one is
      // appended — which is exactly the requested "update if exists, else add".
      loadWallsFromFile();
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();
    sub_walls_ = create_subscription<robo_drill::msg::WallArray>(
      input_topic_, qos,
      std::bind(&WallAggregatorNode::wallsCallback, this, std::placeholders::_1));
    if (use_openings_)
    {
      sub_openings_ = create_subscription<geometry_msgs::msg::PoseArray>(
        openings_topic_, qos,
        std::bind(&WallAggregatorNode::openingsCallback, this, std::placeholders::_1));
    }
    if (use_grid_)
    {
      // RTAB-Map publishes /map latched (transient_local); the subscriber MUST
      // match that durability to receive the last map on (re)connect — a
      // volatile subscriber misses the latched grid and only sees a fresh one if
      // RTAB-Map republishes (it won't while the robot is still), leaving
      // have_grid_ false and falling back to drifting lidar-only walls.
      auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
      sub_grid_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        grid_topic_, map_qos,
        std::bind(&WallAggregatorNode::gridCallback, this, std::placeholders::_1));
    }

    if (confirm_gate_)
    {
      auto odom_qos = rclcpp::SensorDataQoS();  // odom is best-effort/sensor QoS
      sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, odom_qos,
        std::bind(&WallAggregatorNode::odomCallback, this, std::placeholders::_1));
    }

    pub_map_ = create_publisher<robo_drill::msg::WallArray>("~/persistent_walls", qos);
    pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/markers", qos);

    RCLCPP_INFO(get_logger(), "wall_aggregator_node up: %s -> persistent walls in '%s'%s%s%s%s",
                input_topic_.c_str(), map_frame_.c_str(),
                use_grid_ ? " (grid-anchored)" : " (lidar-only)",
                use_openings_ ? " +openings" : "",
                save_walls_to_file_ ? " -> file: " : "",
                save_walls_to_file_ ? wall_file_path_.c_str() : "");
  }

private:
  // ------------------------------------------------------------------
  void openingsCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    latest_openings_ = *msg;
    have_openings_ = true;
  }

  // Track localization stability from the odometry (twist + pose covariance).
  // Any moment the robot turns too fast OR the pose covariance is too high marks
  // localization "unsettled"; we then keep it unsettled for confirm_settle_time
  // afterwards to cover the map<-odom re-alignment lag once motion stops.
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    have_odom_ = true;
    const double wz = std::abs(msg->twist.twist.angular.z);
    const double yaw_cov = msg->pose.covariance[35];  // rot-Z variance
    const bool fast = wz > confirm_max_angular_vel_;
    const bool uncertain = confirm_max_cov_ > 0.0 && yaw_cov > confirm_max_cov_;
    if (fast || uncertain)
    {
      last_unsettled_ = now();
    }
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
      "odom |wz|=%.2f rad/s yaw_cov=%.4f settled=%d", wz, yaw_cov,
      static_cast<int>(localizationSettled()));
  }

  bool localizationSettled()
  {
    if (!confirm_gate_)
    {
      return true;
    }
    if (!have_odom_)
    {
      return true;  // no odom signal -> fail open (don't stall the pipeline)
    }
    return (now() - last_unsettled_).seconds() >= confirm_settle_time_;
  }

  void wallsCallback(const robo_drill::msg::WallArray::SharedPtr msg)
  {
    if (msg->walls.empty())
    {
      // Still age out provisional walls and republish the stable map.
      pruneProvisional(now());
      publish(msg->header.stamp);
      return;
    }

    // Localization-settling gate. While the robot rotates, the map<-odom
    // correction lags and the cloud is transiently misaligned with the map;
    // detections ingested then land in the wrong place and get confirmed as
    // ghost walls "in the middle of nowhere". So only INGEST/CONFIRM detections
    // when localization is settled: the robot is turning slower than
    // confirm_max_angular_vel, that has held for confirm_settle_time (covers the
    // re-alignment lag AFTER a turn), and the odom covariance is below
    // confirm_max_cov. Existing walls still publish; we just don't seed/confirm.
    if (!localizationSettled())
    {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
        "wall ingestion paused: localization unsettled (rotating / re-aligning / high cov)");
      pruneProvisional(now());
      publish(msg->header.stamp);
      return;
    }

    // odom -> map at the detection stamp (fall back to latest on extrapolation).
    Eigen::Isometry3d T;
    if (msg->header.frame_id == map_frame_)
    {
      T.setIdentity();
    }
    else if (!lookup(map_frame_, msg->header.frame_id, msg->header.stamp, T))
    {
      return;
    }
    const Eigen::Matrix3d R = T.linear();

    for (const auto & wm : msg->walls)
    {
      DWall w;
      const Eigen::Vector3d s = T * Eigen::Vector3d(wm.start.x, wm.start.y, wm.start.z);
      const Eigen::Vector3d e = T * Eigen::Vector3d(wm.end.x, wm.end.y, wm.end.z);
      Eigen::Vector3d n = R * Eigen::Vector3d(wm.normal.x, wm.normal.y, wm.normal.z);
      n.z() = 0.0;
      if (n.norm() < 1e-3)
      {
        continue;
      }
      n.normalize();
      w.normal = Eigen::Vector2f(n.x(), n.y());
      w.p1 = Eigen::Vector2f(s.x(), s.y());
      w.p2 = Eigen::Vector2f(e.x(), e.y());
      w.d = -w.normal.dot(w.p1);
      // map<-odom is gravity-aligned, so height is preserved.
      w.z_min = static_cast<float>(s.z());
      w.z_max = w.z_min + wm.height;
      w.inliers = std::max(1, wm.inliers);
      fuse(w);
    }

    pruneProvisional(now());
    if (use_openings_ && have_openings_)
    {
      associateOpenings();
    }
    publish(msg->header.stamp);
  }

  // ------------------------------------------------------------------
  // Associate detection w to an existing persistent wall and fuse, else seed.
  void fuse(const DWall & w)
  {
    const float ang_tol = std::cos(assoc_angle_deg_ * M_PI / 180.0);
    const Eigen::Vector2f wdir(-w.normal.y(), w.normal.x());
    const float wt0 = std::min(project(w.p1, w.p1, wdir), project(w.p2, w.p1, wdir));
    const float wt1 = std::max(project(w.p1, w.p1, wdir), project(w.p2, w.p1, wdir));

    int best = -1;
    float best_dist = std::numeric_limits<float>::max();
    for (size_t i = 0; i < walls_.size(); ++i)
    {
      PWall & p = walls_[i];
      const float dot = p.normal.dot(w.normal);
      if (std::abs(dot) < ang_tol)
      {
        continue;  // not parallel
      }
      const float s = (dot >= 0.0f) ? 1.0f : -1.0f;   // align orientation
      const float perp = std::abs(p.d - s * w.d);
      if (perp > static_cast<float>(assoc_dist_))
      {
        continue;  // parallel but a different plane
      }
      // Overlap along p's direction (negative overlap = gap).
      const Eigen::Vector2f pdir(-p.normal.y(), p.normal.x());
      const float pt0 = std::min(project(p.p1, p.p1, pdir), project(p.p2, p.p1, pdir));
      const float pt1 = std::max(project(p.p1, p.p1, pdir), project(p.p2, p.p1, pdir));
      const float a0 = project(w.p1, p.p1, pdir);
      const float a1 = project(w.p2, p.p1, pdir);
      const float wlo = std::min(a0, a1), whi = std::max(a0, a1);
      const float overlap = std::min(pt1, whi) - std::max(pt0, wlo);
      if (overlap < -static_cast<float>(assoc_overlap_gap_))
      {
        continue;  // collinear but too far apart -> a different wall
      }
      if (perp < best_dist)
      {
        best_dist = perp;
        best = static_cast<int>(i);
      }
    }
    (void)wt0;
    (void)wt1;

    if (best < 0)
    {
      seed(w);
      return;
    }

    // ---- fuse into walls_[best] ----
    PWall & p = walls_[best];
    const float s = (p.normal.dot(w.normal) >= 0.0f) ? 1.0f : -1.0f;
    const Eigen::Vector2f wn = s * w.normal;
    const float wd = s * w.d;

    // Clamp the accumulated weight so a long-lived wall's plane stays responsive
    // to fresh detections instead of freezing at its first (possibly misaligned)
    // offset. 0 = no clamp.
    long pw = std::max(1L, p.total_inliers);
    if (max_fuse_inliers_ > 0)
    {
      pw = std::min(pw, static_cast<long>(max_fuse_inliers_));
    }
    const float wP = static_cast<float>(pw);
    const float wW = static_cast<float>(w.inliers);
    Eigen::Vector2f nf = (wP * p.normal + wW * wn).normalized();
    float df = (wP * p.d + wW * wd) / (wP + wW);

    // Union extent along the fused direction, projected onto the fused plane.
    const Eigen::Vector2f dir(-nf.y(), nf.x());
    std::array<Eigen::Vector2f, 4> pts{p.p1, p.p2, w.p1, w.p2};
    float tmin = std::numeric_limits<float>::max();
    float tmax = -std::numeric_limits<float>::max();
    Eigen::Vector2f a = p.p1, b = p.p2;
    for (const auto & q : pts)
    {
      const float t = (q - p.p1).dot(dir);
      if (t < tmin) { tmin = t; a = q; }
      if (t > tmax) { tmax = t; b = q; }
    }
    auto onPlane = [&](const Eigen::Vector2f & q) {
      return q - nf * (nf.dot(q) + df);
    };
    p.normal = nf;
    p.d = df;
    p.p1 = onPlane(a);
    p.p2 = onPlane(b);
    p.z_min = std::min(p.z_min, w.z_min);
    p.z_max = std::max(p.z_max, w.z_max);
    p.total_inliers += w.inliers;
    p.observations += 1;
    p.last_seen = now();
    p.confidence = std::min(1.0f,
      static_cast<float>(p.observations) / static_cast<float>(min_observations_));
  }

  void seed(const DWall & w)
  {
    PWall p;
    p.normal = w.normal;
    p.d = w.d;
    p.p1 = w.p1;
    p.p2 = w.p2;
    p.z_min = w.z_min;
    p.z_max = w.z_max;
    p.observations = 1;
    p.total_inliers = w.inliers;
    p.last_seen = now();
    p.id = next_id_++;
    p.confidence = std::min(1.0f, 1.0f / static_cast<float>(min_observations_));
    walls_.push_back(p);
  }

  // Drop walls still unconfirmed that haven't been seen for a while. Confirmed
  // walls persist (the building doesn't move).
  void pruneProvisional(const rclcpp::Time & t)
  {
    walls_.erase(std::remove_if(walls_.begin(), walls_.end(),
      [&](const PWall & p) {
        return p.observations < min_observations_ &&
               (t - p.last_seen).seconds() > provisional_timeout_;
      }), walls_.end());
  }

  // Attach each opening center to the nearest confirmed wall it lies on.
  void associateOpenings()
  {
    for (auto & p : walls_)
    {
      p.openings.clear();
    }
    if (latest_openings_.header.frame_id != map_frame_)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Openings are in '%s' not '%s'; skipping (transform not implemented).",
        latest_openings_.header.frame_id.c_str(), map_frame_.c_str());
      return;
    }
    for (const auto & pose : latest_openings_.poses)
    {
      const Eigen::Vector2f c(pose.position.x, pose.position.y);
      int best = -1;
      float best_perp = static_cast<float>(opening_assoc_dist_);
      for (size_t i = 0; i < walls_.size(); ++i)
      {
        const PWall & p = walls_[i];
        if (p.observations < min_observations_)
        {
          continue;
        }
        const float perp = std::abs(p.normal.dot(c) + p.d);
        if (perp > best_perp)
        {
          continue;
        }
        const Eigen::Vector2f dir(-p.normal.y(), p.normal.x());
        const float t = (c - p.p1).dot(dir);
        const float len = (p.p2 - p.p1).dot(dir);
        if (t < std::min(0.0f, len) || t > std::max(0.0f, len))
        {
          continue;  // off the ends of the wall
        }
        best_perp = perp;
        best = static_cast<int>(i);
      }
      if (best >= 0)
      {
        walls_[best].openings.push_back(c);
      }
    }
  }

  // ------------------------------------------------------------------
  // Extract drift-free wall lines from the optimized occupancy grid and merge
  // them into the persistent grid_walls_ set (stable ids).
  void gridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr grid)
  {
    const int W = grid->info.width, H = grid->info.height;
    if (W <= 1 || H <= 1)
    {
      return;
    }
    const float res = grid->info.resolution;  // origin offsets are applied per-segment in validateAndSplit

    cv::Mat occ(H, W, CV_8UC1, cv::Scalar(0));
    for (int r = 0; r < H; ++r)
    {
      for (int c = 0; c < W; ++c)
      {
        const int8_t v = grid->data[static_cast<size_t>(r) * W + c];
        if (v >= grid_occupied_thresh_)   // -1 unknown / 0 free are skipped
        {
          occ.at<uint8_t>(r, c) = 255;
        }
      }
    }

    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(occ, lines, 1.0, CV_PI / 180.0, grid_hough_threshold_,
                    grid_min_wall_length_ / res, grid_max_wall_gap_ / res);

    // Validate + carve each raw Hough line against the cells it actually covers,
    // emitting only the well-supported, opening-free sub-segments (metric).
    std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f>> segs;
    const int opening_run_px =
      std::max(1, static_cast<int>(std::lround(grid_opening_min_run_ / res)));
    const int min_len_px =
      std::max(1, static_cast<int>(std::lround(grid_min_wall_length_ / res)));
    for (const auto & l : lines)
    {
      validateAndSplit(*grid, l, opening_run_px, min_len_px, segs);
    }

    // Merge collinear Hough fragments into full-length walls. The grid splits a
    // wall at every doorway/notch; bridging same-line segments whose gap is
    // < grid_merge_gap rejoins them into one continuous wall. Then re-derive the
    // grid wall set fresh (the optimized grid is the source of truth), reusing
    // ids from the previous set so markers don't churn.
    std::vector<GridWall> merged = mergeGridSegments(segs);

    // Re-apply the blob test to the MERGED extent. validateAndSplit only vetoes
    // raw Hough pieces, but mergeGridSegments then bridges collinear edge-pieces
    // into one line that can span the interior of a solid mass (a line fit along a
    // cluttered corner's diagonal). Re-checking the full merged extent drops that
    // line, which per-piece validation lets slip through.
    if (grid_max_wall_thickness_ > 0.0)
    {
      const float mres = grid->info.resolution;
      const float mox = grid->info.origin.position.x;
      const float moy = grid->info.origin.position.y;
      const int off_px = std::max(2, static_cast<int>(std::ceil(grid_max_wall_thickness_ / mres)));
      std::vector<GridWall> kept;
      kept.reserve(merged.size());
      for (const auto & m : merged)
      {
        const int mx0 = static_cast<int>((m.p1.x() - mox) / mres);
        const int my0 = static_cast<int>((m.p1.y() - moy) / mres);
        const int mx1 = static_cast<int>((m.p2.x() - mox) / mres);
        const int my1 = static_cast<int>((m.p2.y() - moy) / mres);
        if (interiorRatio(*grid, mx0, my0, mx1, my1, off_px) <= grid_max_interior_ratio_)
        {
          kept.push_back(m);
        }
      }
      merged.swap(kept);
    }

    const rclcpp::Time t = now();
    for (auto & m : merged)
    {
      int prev = -1;
      for (size_t i = 0; i < grid_walls_.size(); ++i)
      {
        if (planeMatch(grid_walls_[i].normal, grid_walls_[i].d,
                       grid_walls_[i].p1, grid_walls_[i].p2,
                       m.normal, m.d, m.p1, m.p2,
                       grid_match_angle_deg_, grid_match_dist_, grid_merge_gap_))
        {
          prev = static_cast<int>(i);
          break;
        }
      }
      m.id = (prev >= 0) ? grid_walls_[prev].id : next_grid_id_++;
      m.swept = (prev >= 0) ? grid_walls_[prev].swept : 0;  // carry sweep history
      m.last_seen = t;
    }
    grid_walls_ = std::move(merged);
    have_grid_ = true;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
      "grid %dx%d res=%.3f -> %zu grid walls (raw Hough lines=%zu)",
      W, H, res, grid_walls_.size(), lines.size());
  }

  // Fraction of samples along a pixel segment whose perpendicular neighbourhood
  // is solidly occupied on BOTH sides -> the line runs through the INTERIOR of a
  // solid occupied mass (furniture cluster / clutter / mapping smear) rather than
  // along a wall edge. A real wall is thin: it has free/unknown space on its room
  // side, so at most one side is solid and its interior ratio is ~0. off_px is how
  // far off the line each side is probed (a bit beyond a plausible wall thickness).
  float interiorRatio(const nav_msgs::msg::OccupancyGrid & grid,
                      int x0, int y0, int x1, int y1, int off_px) const
  {
    const int W = grid.info.width, H = grid.info.height;
    const float dx = static_cast<float>(x1 - x0), dy = static_cast<float>(y1 - y0);
    const float len = std::hypot(dx, dy);
    if (len < 1.0f)
    {
      return 0.0f;
    }
    const float ux = dx / len, uy = dy / len;   // along
    const float nx = -uy, ny = ux;              // perpendicular (unit)
    const int steps = static_cast<int>(len);
    auto occAt = [&](int cx, int cy) {
      if (cx < 0 || cx >= W || cy < 0 || cy >= H)
      {
        return false;  // off-map counts as not-occupied (an edge, wall-like)
      }
      return grid.data[static_cast<size_t>(cy) * W + cx] >= grid_occupied_thresh_;
    };
    const int need = std::max(1, static_cast<int>(std::ceil(0.7f * off_px)));
    int interior = 0, samples = 0;
    for (int i = 0; i <= steps; ++i)
    {
      const float cx = x0 + ux * i, cy = y0 + uy * i;
      int solid_sides = 0;
      for (int side = -1; side <= 1; side += 2)
      {
        int occ = 0;
        for (int k = 1; k <= off_px; ++k)
        {
          if (occAt(static_cast<int>(std::lround(cx + side * nx * k)),
                    static_cast<int>(std::lround(cy + side * ny * k))))
          {
            ++occ;
          }
        }
        if (occ >= need)
        {
          ++solid_sides;
        }
      }
      ++samples;
      if (solid_sides == 2)
      {
        ++interior;
      }
    }
    return samples > 0 ? static_cast<float>(interior) / static_cast<float>(samples) : 0.0f;
  }

  // Walk the cells under one raw Hough segment, split it at runs of KNOWN-FREE
  // cells (real openings the robot saw through), and keep each resulting piece
  // only if it is long enough AND its cells are genuinely occupied
  // (fill_ratio >= grid_min_fill_ratio). This is what separates a wall from a
  // line HoughLinesP hallucinated through collinear furniture/speckle. Kept
  // pieces are appended to `segs` as metric (map-frame) endpoint pairs.
  void validateAndSplit(const nav_msgs::msg::OccupancyGrid & grid,
                        const cv::Vec4i & l, int opening_run_px, int min_len_px,
                        std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f>> & segs) const
  {
    const int W = grid.info.width, H = grid.info.height;
    const float res = grid.info.resolution;
    const float ox = grid.info.origin.position.x;
    const float oy = grid.info.origin.position.y;
    auto toMap = [&](int col, int row) {
      return Eigen::Vector2f(ox + (col + 0.5f) * res, oy + (row + 0.5f) * res);
    };

    const int x0 = l[0], y0 = l[1], x1 = l[2], y1 = l[3];
    const int steps = std::max(std::abs(x1 - x0), std::abs(y1 - y0));
    if (steps < 1)
    {
      return;
    }

    // Emit the piece [start_step, end_step] if it clears the length + fill gates.
    int seg_start = -1, occupied_in_seg = 0, sampled_in_seg = 0;
    int sx = x0, sy = y0;  // pixel coords of the current piece's start
    auto flush = [&](int s_start, int s_end, int ssx, int ssy, int ex, int ey) {
      if (s_start < 0 || s_end < s_start)
      {
        return;
      }
      if ((s_end - s_start) < min_len_px || sampled_in_seg <= 0)
      {
        return;
      }
      if (occupied_in_seg < static_cast<int>(std::ceil(grid_min_fill_ratio_ * sampled_in_seg)))
      {
        return;  // collinear scatter, not a wall
      }
      // Blob rejection: a line drawn through the interior of a solid occupied
      // mass is occupied to BOTH sides; a real wall has open space on its room
      // side. Drop the piece if it's "interior" for too much of its length.
      if (grid_max_wall_thickness_ > 0.0)
      {
        const int off_px = std::max(2, static_cast<int>(std::ceil(grid_max_wall_thickness_ / res)));
        if (interiorRatio(grid, ssx, ssy, ex, ey, off_px) > grid_max_interior_ratio_)
        {
          return;  // line runs through a solid blob, not along a wall
        }
      }
      segs.emplace_back(toMap(ssx, ssy), toMap(ex, ey));
    };

    int free_run = 0;
    int last_occ_step = -1, last_occ_x = x0, last_occ_y = y0;
    for (int i = 0; i <= steps; ++i)
    {
      const float a = static_cast<float>(i) / steps;
      const int cx = static_cast<int>(std::lround(x0 + a * (x1 - x0)));
      const int cy = static_cast<int>(std::lround(y0 + a * (y1 - y0)));
      if (cx < 0 || cx >= W || cy < 0 || cy >= H)
      {
        continue;
      }
      const int8_t v = grid.data[static_cast<size_t>(cy) * W + cx];
      const bool occupied = v >= grid_occupied_thresh_;
      const bool known_free = v >= 0 && v < grid_occupied_thresh_;

      if (seg_start < 0)
      {
        seg_start = i;  sx = cx;  sy = cy;  occupied_in_seg = 0;  sampled_in_seg = 0;
        free_run = 0;  last_occ_step = -1;
      }
      ++sampled_in_seg;
      if (occupied)
      {
        ++occupied_in_seg;
        free_run = 0;
        last_occ_step = i;  last_occ_x = cx;  last_occ_y = cy;
      }
      else if (known_free)
      {
        ++free_run;
        if (free_run >= opening_run_px)
        {
          // Close the piece at the last occupied cell before the opening.
          flush(seg_start, last_occ_step, sx, sy, last_occ_x, last_occ_y);
          seg_start = -1;  // a new piece starts after the gap
        }
      }
      // unknown cells neither support nor break the wall.
    }
    flush(seg_start, last_occ_step, sx, sy, last_occ_x, last_occ_y);
  }

  // Greedily merge collinear, gap-close segments into full walls.
  std::vector<GridWall> mergeGridSegments(
    const std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f>> & segs) const
  {
    struct Seg { Eigen::Vector2f a, b, dir, nrm; float d; };
    std::vector<Seg> in;
    for (const auto & s : segs)
    {
      const Eigen::Vector2f v = s.second - s.first;
      const float len = v.norm();
      if (len < static_cast<float>(grid_min_wall_length_))
      {
        continue;
      }
      Seg q;
      q.a = s.first;  q.b = s.second;  q.dir = v / len;
      q.nrm = Eigen::Vector2f(-q.dir.y(), q.dir.x());
      q.d = -q.nrm.dot(q.a);
      in.push_back(q);
    }

    const float ang = std::cos(grid_match_angle_deg_ * M_PI / 180.0);
    std::vector<GridWall> out;
    std::vector<bool> used(in.size(), false);
    for (size_t i = 0; i < in.size(); ++i)
    {
      if (used[i])
      {
        continue;
      }
      used[i] = true;
      Eigen::Vector2f a = in[i].a, b = in[i].b;
      const Eigen::Vector2f nrm = in[i].nrm, dir = in[i].dir;
      const float d = in[i].d;
      bool grew = true;
      while (grew)
      {
        grew = false;
        for (size_t j = 0; j < in.size(); ++j)
        {
          if (used[j] || std::abs(nrm.dot(in[j].nrm)) < ang)
          {
            continue;
          }
          const float s = (nrm.dot(in[j].nrm) >= 0.0f) ? 1.0f : -1.0f;
          if (std::abs(d - s * in[j].d) > static_cast<float>(grid_match_dist_))
          {
            continue;  // not the same line
          }
          const float t1 = std::max((a - a).dot(dir), (b - a).dot(dir));
          const float u0 = std::min((in[j].a - a).dot(dir), (in[j].b - a).dot(dir));
          const float u1 = std::max((in[j].a - a).dot(dir), (in[j].b - a).dot(dir));
          const float overlap = std::min(t1, std::max(u0, u1)) - std::max(0.0f, u0);
          if (overlap < -static_cast<float>(grid_merge_gap_))
          {
            continue;  // gap along the line too large -> keep separate
          }
          std::array<Eigen::Vector2f, 4> pts{a, b, in[j].a, in[j].b};
          float tmin = std::numeric_limits<float>::max();
          float tmax = -std::numeric_limits<float>::max();
          Eigen::Vector2f na = a, nb = b;
          for (const auto & p : pts)
          {
            const float t = (p - a).dot(dir);
            if (t < tmin) { tmin = t; na = p; }
            if (t > tmax) { tmax = t; nb = p; }
          }
          a = na;  b = nb;  used[j] = true;  grew = true;
        }
      }
      GridWall g;
      g.p1 = a;  g.p2 = b;  g.normal = nrm;  g.d = -nrm.dot(a);
      out.push_back(g);
    }
    return out;
  }

  // Shortest distance from point q to segment [a,b] in 2D.
  static float distPointSeg(const Eigen::Vector2f & q,
                            const Eigen::Vector2f & a, const Eigen::Vector2f & b)
  {
    const Eigen::Vector2f ab = b - a;
    const float len2 = ab.squaredNorm();
    const float t = (len2 > 1e-9f)
      ? std::clamp((q - a).dot(ab) / len2, 0.0f, 1.0f) : 0.0f;
    return (q - (a + t * ab)).norm();
  }

  // Count an "in-range" sweep for every grid wall the robot is currently close
  // enough to observe. Walls that accumulate sweeps without ever being confirmed
  // by the lidar are later retracted in gridAnchoredWalls() as false positives.
  // ALSO decays stale CONFIRMED lidar walls (see below) so a once-confirmed
  // false positive doesn't stay pink forever.
  void updateSweep()
  {
    if ((reject_after_sweeps_ <= 0 || grid_walls_.empty()) &&
        (decay_after_sweeps_ <= 0 || walls_.empty()))
    {
      return;
    }
    Eigen::Isometry3d T;
    if (!lookup(map_frame_, base_frame_, builtin_interfaces::msg::Time(), T))
    {
      return;  // no robot pose yet; don't penalise any wall
    }
    const Eigen::Vector2f robot(T.translation().x(), T.translation().y());
    const float range = static_cast<float>(lidar_confirm_range_);
    if (reject_after_sweeps_ > 0)
    {
      for (auto & g : grid_walls_)
      {
        if (distPointSeg(robot, g.p1, g.p2) <= range &&
            g.swept < std::numeric_limits<int>::max())
        {
          ++g.swept;
        }
      }
    }

    // Sweep-gated demotion of CONFIRMED walls. The "building doesn't move" so
    // confirmed walls normally persist forever -- but a transient object or a
    // furniture row that got confirmed once would then stay pink even after the
    // lidar stops seeing a wall there. So: if the robot is within range of a
    // confirmed lidar wall (the 360 deg merged cloud SHOULD see it) yet that
    // wall hasn't been refreshed by a fresh detection within support_timeout for
    // decay_after_sweeps in-range cycles, it's a stale/false confirmation ->
    // prune it. Pruning drops the grid wall's obs to 0, so it reverts to grey and
    // the grey-retraction above removes it. Occlusion-safe: a wall the robot
    // isn't near, or one still being seen (last_seen fresh), is never touched.
    if (decay_after_sweeps_ > 0)
    {
      const rclcpp::Time t = now();
      walls_.erase(std::remove_if(walls_.begin(), walls_.end(),
        [&](PWall & p) {
          const bool in_range = distPointSeg(robot, p.p1, p.p2) <= range;
          const bool fresh = (t - p.last_seen).seconds() < support_timeout_;
          if (fresh)
          {
            p.swept_unseen = 0;
          }
          else if (in_range)
          {
            ++p.swept_unseen;
          }
          return p.swept_unseen >= decay_after_sweeps_;
        }), walls_.end());
    }
  }

  // Build the output walls from the drift-free grid layout, enriched with lidar
  // height where the lidar has swept the wall. Furniture is already filtered by
  // grid_min_wall_length, so we emit the FULL grid layout immediately: a wall
  // with tall lidar support is `refined` (real height, high confidence); one
  // not yet swept is grid-only (default height, lower confidence). With
  // require_lidar_confirmation=true the unswept walls are withheld instead.
  std::vector<PWall> gridAnchoredWalls()
  {
    std::vector<PWall> out;
    for (const auto & g : grid_walls_)
    {
      float zmin = std::numeric_limits<float>::max();
      float zmax = -std::numeric_limits<float>::max();
      int obs = 0;
      long inl = 0;
      float conf = 0.0f;
      Eigen::Vector2f side(0.0f, 0.0f);
      // Inlier-weighted lidar plane offset, expressed for g.normal (so the grid
      // line can be snapped perpendicular onto the surface the lidar actually
      // sees). d for normal n satisfies n.x + d = 0; aligning each lidar wall's
      // normal to g.normal (sign s) makes its offset s*p.d in g.normal terms.
      float sum_w = 0.0f, sum_w_d = 0.0f;
      for (const auto & p : walls_)
      {
        if (!planeMatch(g.normal, g.d, g.p1, g.p2, p.normal, p.d, p.p1, p.p2,
                        grid_match_angle_deg_, grid_match_dist_, assoc_overlap_gap_))
        {
          continue;
        }
        zmin = std::min(zmin, p.z_min);
        zmax = std::max(zmax, p.z_max);
        obs += p.observations;
        inl += p.total_inliers;
        conf = std::max(conf, p.confidence);
        side += p.normal * static_cast<float>(p.observations);  // robot-facing side
        const float s = (g.normal.dot(p.normal) >= 0.0f) ? 1.0f : -1.0f;
        const float wgt = static_cast<float>(std::max(1L, p.total_inliers));
        sum_w += wgt;
        sum_w_d += wgt * s * p.d;
      }
      const bool confirmed = obs >= min_observations_;
      if (!confirmed && require_lidar_confirmation_)
      {
        continue;
      }
      // Retract a provisional wall the robot has swept past enough times with no
      // lidar support at all: the grid had an obstacle there but the lidar found
      // no real (tall) wall -> it was furniture/speckle. Walls with any lidar
      // support (obs > 0) are kept (still maturing toward confirmation).
      if (!confirmed && obs == 0 && reject_after_sweeps_ > 0 &&
          g.swept >= reject_after_sweeps_)
      {
        continue;
      }
      PWall w;
      w.p1 = g.p1;
      w.p2 = g.p2;
      Eigen::Vector2f n = g.normal;
      if (confirmed && n.dot(side) < 0.0f)
      {
        n = -n;  // orient toward the side the lidar saw it from
      }
      w.normal = n;
      w.d = -n.dot(g.p1);

      // SNAP confirmed walls perpendicular onto the lidar plane. The grid line
      // can sit up to ~0.2 m off the true surface (cell quantization, thick
      // occupied bands, ray-tracing); the lidar plane is metric. We keep the
      // grid's orientation + extent (drift-free layout) and only shift the line
      // along its normal to where the lidar sees the wall. This is the degenerate
      // 1-DOF case of point-to-plane ICP for a single planar surface, solved in
      // closed form. Global drift is ~0, so this corrects local grid error only.
      if (confirmed && snap_to_lidar_plane_ && sum_w > 0.0f)
      {
        // Desired offset for the (possibly flipped) output normal n.
        const float d_target = (n.dot(g.normal) >= 0.0f) ? (sum_w_d / sum_w)
                                                          : -(sum_w_d / sum_w);
        const float d_grid = -n.dot(g.p1);
        float delta = d_grid - d_target;            // shift along n onto the plane
        const float cap = (snap_max_shift_ > 0.0)
          ? static_cast<float>(snap_max_shift_)
          : static_cast<float>(grid_match_dist_);
        delta = std::max(-cap, std::min(cap, delta));   // a bad assoc can't fling it
        // Temporal EMA per stable wall id to avoid per-frame jitter.
        auto it = wall_snap_.find(g.id);
        const float a = static_cast<float>(snap_alpha_);
        const float sm = (it == wall_snap_.end()) ? delta : (a * delta + (1.0f - a) * it->second);
        wall_snap_[g.id] = sm;
        w.p1 += n * sm;
        w.p2 += n * sm;
        w.d = -n.dot(w.p1);
      }

      w.refined = confirmed;
      if (confirmed)
      {
        w.z_min = zmin;
        w.z_max = zmax;
        w.observations = obs;
        w.total_inliers = inl;
        w.confidence = std::min(1.0f, conf);
      }
      else
      {
        w.z_min = static_cast<float>(default_z_min_);   // grid-only: layout known,
        w.z_max = static_cast<float>(default_z_max_);   // height not yet measured
        w.observations = 0;
        w.total_inliers = 0;
        w.confidence = 0.5f;
      }
      w.id = g.id;
      out.push_back(w);
    }
    return out;
  }

  // True if two finite segments lie on the same plane: near-parallel normals,
  // small perpendicular offset, and overlap (or gap < gap_tol).
  static bool planeMatch(
    const Eigen::Vector2f & n1, float d1, const Eigen::Vector2f & a1, const Eigen::Vector2f & b1,
    const Eigen::Vector2f & n2, float d2, const Eigen::Vector2f & a2, const Eigen::Vector2f & b2,
    double angle_deg, double dist_tol, double gap_tol)
  {
    const float dot = n1.dot(n2);
    if (std::abs(dot) < std::cos(angle_deg * M_PI / 180.0))
    {
      return false;
    }
    (void)d2;
    // Perpendicular offset measured LOCALLY as the closest approach of segment 2
    // to segment 1's line (n1.x + d1 = 0). The previous test compared the plane
    // offsets |d1 - s*d2|, which are referenced to the WORLD ORIGIN: for two
    // near-parallel walls far from the origin, a tiny angular difference made
    // |d1 - s*d2| balloon well past dist_tol (e.g. 0.94 m for walls really only
    // ~0.6 m apart), so genuine parallel duplicates were never merged. Using the
    // endpoint-to-line distance is origin-independent and correct.
    const float e_a = std::abs(n1.dot(a2) + d1);
    const float e_b = std::abs(n1.dot(b2) + d1);
    if (std::min(e_a, e_b) > static_cast<float>(dist_tol))
    {
      return false;
    }
    const Eigen::Vector2f dir(-n1.y(), n1.x());
    const float t0 = std::min((a1 - a1).dot(dir), (b1 - a1).dot(dir));
    const float t1 = std::max((a1 - a1).dot(dir), (b1 - a1).dot(dir));
    const float u0 = std::min((a2 - a1).dot(dir), (b2 - a1).dot(dir));
    const float u1 = std::max((a2 - a1).dot(dir), (b2 - a1).dot(dir));
    const float overlap = std::min(t1, u1) - std::max(t0, u0);
    return overlap >= -static_cast<float>(gap_tol);
  }

  // Final de-duplication: collapse output walls that are really the same plane
  // (near-parallel, small perpendicular offset, overlapping) into one. This
  // catches duplicates the upstream Hough/grid merge left behind — e.g. a thick
  // wall that produced an edge line on each face, or two collinear fragments at
  // a slight angle. Keeps the union extent and the strongest wall's stats/id.
  std::vector<PWall> dedupWalls(const std::vector<PWall> & in) const
  {
    std::vector<PWall> out;
    std::vector<bool> used(in.size(), false);
    for (size_t i = 0; i < in.size(); ++i)
    {
      if (used[i])
      {
        continue;
      }
      PWall acc = in[i];
      used[i] = true;
      bool grew = true;
      while (grew)
      {
        grew = false;
        for (size_t j = 0; j < in.size(); ++j)
        {
          if (used[j])
          {
            continue;
          }
          const PWall & b = in[j];
          // Require genuine overlap (gap_tol 0) so two real walls separated by a
          // doorway on the same line are NOT fused — only true duplicates are.
          if (!planeMatch(acc.normal, acc.d, acc.p1, acc.p2,
                          b.normal, b.d, b.p1, b.p2,
                          grid_match_angle_deg_, grid_match_dist_, 0.0))
          {
            continue;
          }
          // Union the extent along acc's direction, projected onto acc's plane.
          const Eigen::Vector2f dir(-acc.normal.y(), acc.normal.x());
          const std::array<Eigen::Vector2f, 4> pts{acc.p1, acc.p2, b.p1, b.p2};
          float tmin = std::numeric_limits<float>::max();
          float tmax = -std::numeric_limits<float>::max();
          Eigen::Vector2f A = acc.p1, B = acc.p2;
          for (const auto & q : pts)
          {
            const float t = (q - acc.p1).dot(dir);
            if (t < tmin) { tmin = t; A = q; }
            if (t > tmax) { tmax = t; B = q; }
          }
          auto onPlane = [&](const Eigen::Vector2f & q) {
            return q - acc.normal * (acc.normal.dot(q) + acc.d);
          };
          // Keep the id/orientation of the stronger wall so markers stay stable.
          if (b.total_inliers > acc.total_inliers)
          {
            acc.id = b.id;
            if (b.normal.dot(acc.normal) < 0.0f) { acc.normal = -acc.normal; acc.d = -acc.d; }
          }
          acc.p1 = onPlane(A);
          acc.p2 = onPlane(B);
          acc.z_min = std::min(acc.z_min, b.z_min);
          acc.z_max = std::max(acc.z_max, b.z_max);
          acc.observations = std::max(acc.observations, b.observations);
          acc.total_inliers = std::max(acc.total_inliers, b.total_inliers);
          acc.confidence = std::max(acc.confidence, b.confidence);
          acc.refined = acc.refined || b.refined;
          acc.openings.insert(acc.openings.end(), b.openings.begin(), b.openings.end());
          used[j] = true;
          grew = true;
        }
      }
      out.push_back(acc);
    }
    return out;
  }

  // "Strength" ordering for keeping the better of two conflicting walls: prefer
  // lidar-confirmed (refined), then more observations, then more inliers, then
  // longer. Returns true if a is at least as strong as b.
  static bool atLeastAsStrong(const PWall & a, const PWall & b)
  {
    if (a.refined != b.refined) return a.refined;
    if (a.observations != b.observations) return a.observations > b.observations;
    if (a.total_inliers != b.total_inliers) return a.total_inliers > b.total_inliers;
    return (a.p2 - a.p1).squaredNorm() >= (b.p2 - b.p1).squaredNorm();
  }

  // Drop spurious CROSSING wall fits. Real building walls meet at their ENDPOINTS
  // (L/T corners); they do not cross through each other's MIDDLE. A cluttered
  // region (a tall mass, bookshelves against a wall) instead yields several
  // differently-oriented lines through the SAME area (e.g. one real wall detected
  // as two crossing segments, plus a furniture-face line). So if two non-parallel
  // walls intersect at a point that is interior to BOTH (>= cross_endpoint_margin
  // from every endpoint), they can't both be walls — keep the stronger, drop the
  // other. Near-parallel pairs are left to dedupWalls; true corners are preserved
  // because their intersection sits at/near an endpoint.
  std::vector<PWall> suppressCrossings(const std::vector<PWall> & in) const
  {
    if (cross_endpoint_margin_ <= 0.0)
    {
      return in;
    }
    const float min_ang = std::cos(cross_min_angle_deg_ * M_PI / 180.0);
    const float margin = static_cast<float>(cross_endpoint_margin_);
    std::vector<bool> drop(in.size(), false);
    for (size_t i = 0; i < in.size(); ++i)
    {
      if (drop[i]) continue;
      for (size_t j = i + 1; j < in.size(); ++j)
      {
        if (drop[j]) continue;
        // Only consider clearly NON-parallel pairs (parallel dups -> dedupWalls).
        if (std::abs(in[i].normal.dot(in[j].normal)) > min_ang) continue;
        const Eigen::Vector2f p = in[i].p1, r = in[i].p2 - in[i].p1;
        const Eigen::Vector2f q = in[j].p1, s = in[j].p2 - in[j].p1;
        const float rxs = r.x() * s.y() - r.y() * s.x();
        if (std::abs(rxs) < 1e-6f) continue;  // parallel/degenerate
        const Eigen::Vector2f qp = q - p;
        const float t = (qp.x() * s.y() - qp.y() * s.x()) / rxs;  // param on i
        const float u = (qp.x() * r.y() - qp.y() * r.x()) / rxs;  // param on j
        const float li = r.norm(), lj = s.norm();
        // Intersection must be interior to BOTH segments (margin from each end).
        if (t * li < margin || (1.0f - t) * li < margin) continue;
        if (u * lj < margin || (1.0f - u) * lj < margin) continue;
        // They cross through the middle of both -> drop the weaker.
        if (atLeastAsStrong(in[i], in[j])) drop[j] = true;
        else { drop[i] = true; break; }
      }
    }
    std::vector<PWall> out;
    out.reserve(in.size());
    for (size_t i = 0; i < in.size(); ++i)
    {
      if (!drop[i]) out.push_back(in[i]);
    }
    return out;
  }

  // ------------------------------------------------------------------
  // ------------------------------------------------------------------
  // Resolve the wall-map path. An absolute path is used verbatim; a relative
  // one is placed in the package's rgb_detections/ folder (same convention as
  // the YOLO node's csv_output_dir), and the parent directory is created.
  std::string resolveWallFilePath(const std::string & req) const
  {
    namespace fs = std::filesystem;
    fs::path p;
    if (fs::path(req).is_absolute())
    {
      p = req;
    }
    else
    {
      fs::path base;
      try
      {
        base = fs::path(ament_index_cpp::get_package_share_directory("robo_drill"))
               / "rgb_detections";
      }
      catch (const std::exception & e)
      {
        RCLCPP_WARN(get_logger(),
          "wall_aggregator: could not resolve robo_drill share dir (%s); "
          "using ./rgb_detections", e.what());
        base = fs::path("rgb_detections");
      }
      p = base / req;
    }
    std::error_code ec;
    fs::create_directories(p.parent_path(), ec);
    if (ec)
    {
      RCLCPP_WARN(get_logger(), "wall_aggregator: could not create '%s' (%s)",
                  p.parent_path().c_str(), ec.message().c_str());
    }
    return p.string();
  }

  // ------------------------------------------------------------------
  // Load a previously saved wall map (if present) so walls persist across runs.
  // Loaded walls seed walls_ as already-confirmed; fuse() then associates every
  // incoming detection against them (update-in-place on a match, append if new).
  void loadWallsFromFile()
  {
    std::ifstream in(wall_file_path_);
    if (!in)
    {
      RCLCPP_INFO(get_logger(),
        "wall_aggregator: no existing wall file at '%s'; starting fresh",
        wall_file_path_.c_str());
      return;
    }

    auto trim = [](std::string s) {
      const auto b = s.find_first_not_of(" \t\r\n");
      const auto e = s.find_last_not_of(" \t\r\n");
      return (b == std::string::npos) ? std::string() : s.substr(b, e - b + 1);
    };
    auto parse_pair = [](const std::string & v, float & a, float & b) {
      const auto lb = v.find('[');
      const auto rb = v.find(']');
      if (lb == std::string::npos || rb == std::string::npos || rb <= lb)
      {
        return false;
      }
      const std::string inner = v.substr(lb + 1, rb - lb - 1);
      const auto comma = inner.find(',');
      if (comma == std::string::npos)
      {
        return false;
      }
      try
      {
        a = std::stof(inner.substr(0, comma));
        b = std::stof(inner.substr(comma + 1));
      }
      catch (...) { return false; }
      return true;
    };

    std::vector<PWall> loaded;
    PWall cur;
    bool have_cur = false;
    bool have_p1 = false, have_p2 = false;
    int max_id = -1;
    const rclcpp::Time now = this->now();

    auto flush = [&]() {
      if (have_cur && have_p1 && have_p2)
      {
        // Ensure the loaded wall counts as confirmed so it is published and can
        // be matched immediately, without waiting to be re-observed.
        cur.observations = std::max(cur.observations, min_observations_);
        cur.last_seen = now;
        // Recover the normal from the endpoints if the file lacked a usable one.
        if (cur.normal.norm() < 1e-3f)
        {
          const Eigen::Vector2f d = cur.p2 - cur.p1;
          if (d.norm() > 1e-3f)
          {
            const Eigen::Vector2f dir = d.normalized();
            cur.normal = Eigen::Vector2f(-dir.y(), dir.x());
            cur.d = -cur.normal.dot(cur.p1);
          }
        }
        max_id = std::max(max_id, cur.id);
        loaded.push_back(cur);
      }
    };

    std::string line;
    while (std::getline(in, line))
    {
      const std::string t = trim(line);
      if (t.empty() || t[0] == '#' || t == "walls:")
      {
        continue;
      }
      // A new list entry begins with "- id:".
      std::string body = t;
      if (body.rfind("- ", 0) == 0)
      {
        flush();
        cur = PWall{};
        have_cur = true;
        have_p1 = have_p2 = false;
        body = trim(body.substr(2));
      }
      const auto colon = body.find(':');
      if (colon == std::string::npos)
      {
        continue;
      }
      const std::string key = trim(body.substr(0, colon));
      const std::string val = trim(body.substr(colon + 1));
      try
      {
        if (key == "id") { cur.id = std::stoi(val); }
        else if (key == "d") { cur.d = std::stof(val); }
        else if (key == "z_min") { cur.z_min = std::stof(val); }
        else if (key == "z_max") { cur.z_max = std::stof(val); }
        else if (key == "observations") { cur.observations = std::stoi(val); }
        else if (key == "confidence") { cur.confidence = std::stof(val); }
        else if (key == "normal") { float a, b; if (parse_pair(val, a, b)) { cur.normal = {a, b}; } }
        else if (key == "p1") { float a, b; if (parse_pair(val, a, b)) { cur.p1 = {a, b}; have_p1 = true; } }
        else if (key == "p2") { float a, b; if (parse_pair(val, a, b)) { cur.p2 = {a, b}; have_p2 = true; } }
      }
      catch (...) { /* skip malformed field */ }
    }
    flush();

    walls_ = std::move(loaded);
    next_id_ = max_id + 1;
    RCLCPP_INFO(get_logger(), "wall_aggregator: loaded %zu walls from '%s'",
                walls_.size(), wall_file_path_.c_str());
  }

  // ------------------------------------------------------------------
  // Persist the confirmed persistent walls to a file. This is a full snapshot
  // keyed by the aggregator's stable wall id, so it inherently satisfies the
  // three requirements: (1) walls seen earlier stay in the file (they remain in
  // walls_/finals until retracted), (2) a re-observed wall does NOT create a
  // duplicate — it occupies the same id and simply overwrites its own line, and
  // (3) its coordinates are the freshly fused ones. Written atomically (temp +
  // rename) and only when the set changed, so a partial file is never observed
  // and a stationary robot doesn't rewrite every frame.
  void saveWallsToFile(const std::vector<PWall> & finals)
  {
    // Change signature: ids + coordinates quantized to wall_file_epsilon. If it
    // matches the last write, nothing meaningful moved, so skip the write.
    const double q = (wall_file_epsilon_ > 0.0) ? wall_file_epsilon_ : 0.01;
    auto quant = [q](float v) { return static_cast<long>(std::llround(v / q)); };
    size_t sig = 1469598103934665603ULL;  // FNV-1a offset basis
    auto mix = [&sig](long v) {
      sig ^= static_cast<size_t>(v);
      sig *= 1099511628211ULL;  // FNV-1a prime
    };
    // Order-independent so a reshuffle of finals (same walls) isn't a "change":
    // sort a per-wall hash. finals is small, so this is cheap.
    std::vector<size_t> wall_sigs;
    wall_sigs.reserve(finals.size());
    for (const auto & p : finals)
    {
      size_t ws = 1469598103934665603ULL;
      auto wmix = [&ws](long v) { ws ^= static_cast<size_t>(v); ws *= 1099511628211ULL; };
      wmix(p.id);
      wmix(quant(p.p1.x())); wmix(quant(p.p1.y()));
      wmix(quant(p.p2.x())); wmix(quant(p.p2.y()));
      wmix(quant(p.z_min));  wmix(quant(p.z_max));
      wall_sigs.push_back(ws);
    }
    std::sort(wall_sigs.begin(), wall_sigs.end());
    for (const size_t ws : wall_sigs) { mix(static_cast<long>(ws)); }
    if (sig == last_saved_signature_)
    {
      return;  // nothing changed since the last write
    }

    std::ostringstream ss;
    ss << std::fixed << std::setprecision(4);
    ss << "# Persistent walls detected by robo_drill wall_aggregator_node.\n";
    ss << "# frame: " << map_frame_ << "\n";
    ss << "# Regenerated in place; walls are keyed by id (no duplicates).\n";
    ss << "walls:\n";
    for (const auto & p : finals)
    {
      ss << "  - id: " << p.id << "\n";
      ss << "    normal: [" << p.normal.x() << ", " << p.normal.y() << "]\n";
      ss << "    d: " << p.d << "\n";
      ss << "    p1: [" << p.p1.x() << ", " << p.p1.y() << "]\n";
      ss << "    p2: [" << p.p2.x() << ", " << p.p2.y() << "]\n";
      ss << "    z_min: " << p.z_min << "\n";
      ss << "    z_max: " << p.z_max << "\n";
      ss << "    observations: " << p.observations << "\n";
      ss << "    confidence: " << p.confidence << "\n";
    }

    // Atomic write: fully write a sibling temp file, then rename over the target
    // so a reader never sees a half-written file (and a crash mid-write can't
    // corrupt the last good map).
    const std::string tmp = wall_file_path_ + ".tmp";
    {
      std::ofstream out(tmp, std::ios::trunc);
      if (!out)
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "wall_aggregator: cannot open '%s' for writing", tmp.c_str());
        return;
      }
      out << ss.str();
      out.flush();
      if (!out)
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "wall_aggregator: write failed for '%s'", tmp.c_str());
        return;
      }
    }
    if (std::rename(tmp.c_str(), wall_file_path_.c_str()) != 0)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "wall_aggregator: rename '%s' -> '%s' failed", tmp.c_str(),
        wall_file_path_.c_str());
      std::remove(tmp.c_str());
      return;
    }
    last_saved_signature_ = sig;
    RCLCPP_INFO(get_logger(), "wall_aggregator: saved %zu walls to '%s'",
                finals.size(), wall_file_path_.c_str());
  }

  void publish(const builtin_interfaces::msg::Time & stamp)
  {
    if (use_grid_)
    {
      updateSweep();  // age the robot's lidar coverage of each grid wall
    }

    robo_drill::msg::WallArray out;
    out.header.frame_id = map_frame_;
    out.header.stamp = stamp;

    // No DELETEALL here: confirmed walls have stable ids and never vanish, so we
    // re-ADD them in place each cycle. Re-clearing every frame is what makes a
    // marker layer flicker — the persistent layer must not do that. Instead we
    // remember which ids we drew and DELETE only the ones that disappeared (a
    // wall retracted as a false positive), so RViz doesn't keep a ghost marker.
    visualization_msgs::msg::MarkerArray ma;

    // Once, on the first publish, clear any markers left in RViz by a PREVIOUS
    // run of this node: a restart resets our id counter, so old ids are never
    // reused or DELETEd and would linger as ghost/duplicate walls. A one-shot
    // DELETEALL (only on the first message, so no per-frame flicker) wipes them;
    // the current walls are ADDed after it in the same array.
    if (!markers_cleared_)
    {
      visualization_msgs::msg::Marker clear;
      clear.header.frame_id = map_frame_;
      clear.header.stamp = stamp;
      clear.action = visualization_msgs::msg::Marker::DELETEALL;
      ma.markers.push_back(clear);
      markers_cleared_ = true;
    }

    // Grid-anchored output (drift-free XY from the optimized grid) when enabled
    // and a grid has arrived; otherwise the lidar-only confirmed walls.
    std::vector<PWall> finals;
    if (use_grid_ && have_grid_)
    {
      finals = gridAnchoredWalls();
    }
    else
    {
      for (const auto & p : walls_)
      {
        if (p.observations >= min_observations_)
        {
          finals.push_back(p);
        }
      }
    }

    // Collapse any leftover duplicate walls (same plane drawn twice) into one.
    finals = dedupWalls(finals);
    // Drop spurious crossing fits (one real wall detected as two crossing lines;
    // furniture-face lines through a cluttered region). Real corners are kept.
    finals = suppressCrossings(finals);

    // Persist the confirmed set to file (dedup'd + coordinates updated in place).
    if (save_walls_to_file_)
    {
      saveWallsToFile(finals);
    }

    // Visibility into which path is live: if you configured use_grid but this
    // logs "lidar-only", the grid never arrived (QoS/topic) and walls will drift
    // with odom. "grid-anchored" means walls take the drift-free grid geometry.
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
      "%s: %zu walls out (lidar walls_=%zu, have_grid=%s)",
      (use_grid_ && have_grid_) ? "grid-anchored" : "lidar-only",
      finals.size(), walls_.size(), have_grid_ ? "yes" : "no");

    for (const auto & p : finals)
    {
      robo_drill::msg::Wall wm;
      wm.start.x = p.p1.x();  wm.start.y = p.p1.y();  wm.start.z = p.z_min;
      wm.end.x = p.p2.x();    wm.end.y = p.p2.y();    wm.end.z = p.z_min;
      wm.normal.x = p.normal.x();  wm.normal.y = p.normal.y();  wm.normal.z = 0.0;
      wm.d = p.d;
      wm.length = (p.p2 - p.p1).norm();
      wm.height = p.z_max - p.z_min;
      wm.z_min = p.z_min;
      wm.z_max = p.z_max;
      wm.confidence = p.confidence;
      wm.inliers = static_cast<int>(std::min<long>(p.total_inliers, std::numeric_limits<int>::max()));
      wm.refined = p.refined;
      for (const auto & o : p.openings)
      {
        geometry_msgs::msg::Point pt;
        pt.x = o.x();  pt.y = o.y();  pt.z = 0.5f * (p.z_min + p.z_max);
        wm.openings.push_back(pt);
      }
      out.walls.push_back(wm);
      addMarkers(ma, p, stamp);
    }

    // DELETE markers for walls drawn last cycle but absent now (retracted), plus
    // labels for walls no longer labelled (retracted OR flipped pink->grey).
    std::vector<int> current_ids, current_label_ids;
    current_ids.reserve(finals.size());
    for (const auto & p : finals)
    {
      current_ids.push_back(p.id);
      if (p.refined)
      {
        current_label_ids.push_back(p.id);
      }
    }
    auto del_marker = [&](const std::string & ns, int id) {
      visualization_msgs::msg::Marker del;
      del.header.frame_id = map_frame_;
      del.header.stamp = stamp;
      del.ns = ns;
      del.id = id;
      del.action = visualization_msgs::msg::Marker::DELETE;
      ma.markers.push_back(del);
    };
    for (int id : published_ids_)
    {
      if (std::find(current_ids.begin(), current_ids.end(), id) == current_ids.end())
      {
        del_marker("wall_map", id);
      }
    }
    for (int id : published_label_ids_)
    {
      if (std::find(current_label_ids.begin(), current_label_ids.end(), id) == current_label_ids.end())
      {
        del_marker("wall_id", id);
      }
    }
    published_ids_ = std::move(current_ids);
    published_label_ids_ = std::move(current_label_ids);

    pub_map_->publish(out);
    pub_markers_->publish(ma);
  }

  void addMarkers(visualization_msgs::msg::MarkerArray & ma, const PWall & p,
                  const builtin_interfaces::msg::Time & stamp)
  {
    visualization_msgs::msg::Marker face;
    face.header.frame_id = map_frame_;
    face.header.stamp = stamp;
    face.ns = "wall_map";
    face.id = p.id;
    face.type = visualization_msgs::msg::Marker::LINE_LIST;
    face.action = visualization_msgs::msg::Marker::ADD;
    face.scale.x = 0.04;
    face.color.a = 0.9f;
    // purple = lidar-confirmed (real height); grey = grid-only layout (height
    // not yet measured by the lidar).
    face.color.r = p.refined ? 0.7f : 0.6f;
    face.color.g = p.refined ? 0.1f : 0.6f;
    face.color.b = p.refined ? 0.9f : 0.6f;
    face.pose.orientation.w = 1.0;
    auto corner = [&](const Eigen::Vector2f & xy, float z) {
      geometry_msgs::msg::Point pt;
      pt.x = xy.x();  pt.y = xy.y();  pt.z = z;
      return pt;
    };
    const auto bl = corner(p.p1, p.z_min), br = corner(p.p2, p.z_min);
    const auto tl = corner(p.p1, p.z_max), tr = corner(p.p2, p.z_max);
    for (const auto & seg : {std::pair{bl, br}, std::pair{tl, tr},
                             std::pair{bl, tl}, std::pair{br, tr}})
    {
      face.points.push_back(seg.first);
      face.points.push_back(seg.second);
    }
    ma.markers.push_back(face);

    // Wall number, floating at the centre of the rectangle. PINK (lidar-
    // confirmed) walls only, for now. The text is the wall's stable id, so it
    // stays put across cycles and matches the id used when querying a wall.
    if (p.refined)
    {
      visualization_msgs::msg::Marker txt;
      txt.header.frame_id = map_frame_;
      txt.header.stamp = stamp;
      txt.ns = "wall_id";
      txt.id = p.id;
      txt.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      txt.action = visualization_msgs::msg::Marker::ADD;
      txt.pose.position.x = 0.5 * (p.p1.x() + p.p2.x());
      txt.pose.position.y = 0.5 * (p.p1.y() + p.p2.y());
      txt.pose.position.z = 0.5f * (p.z_min + p.z_max);
      txt.pose.orientation.w = 1.0;
      txt.scale.z = 0.5;   // text height (m)
      txt.color.a = 1.0f;
      txt.color.r = 1.0f;
      txt.color.g = 1.0f;
      txt.color.b = 1.0f;
      txt.text = std::to_string(p.id);
      ma.markers.push_back(txt);
    }

    for (size_t k = 0; k < p.openings.size(); ++k)
    {
      visualization_msgs::msg::Marker o;
      o.header.frame_id = map_frame_;
      o.header.stamp = stamp;
      o.ns = "wall_opening";
      o.id = p.id * 100 + static_cast<int>(k);
      o.type = visualization_msgs::msg::Marker::SPHERE;
      o.action = visualization_msgs::msg::Marker::ADD;
      o.pose.position.x = p.openings[k].x();
      o.pose.position.y = p.openings[k].y();
      o.pose.position.z = 0.5f * (p.z_min + p.z_max);
      o.pose.orientation.w = 1.0;
      o.scale.x = o.scale.y = o.scale.z = 0.3;
      o.color.a = 0.9f;
      o.color.r = 1.0f;  o.color.g = 0.3f;  o.color.b = 0.0f;
      ma.markers.push_back(o);
    }
  }

  // ------------------------------------------------------------------
  static float project(const Eigen::Vector2f & q, const Eigen::Vector2f & origin,
                       const Eigen::Vector2f & dir)
  {
    return (q - origin).dot(dir);
  }

  bool lookup(const std::string & target, const std::string & source,
              const builtin_interfaces::msg::Time & stamp, Eigen::Isometry3d & out)
  {
    try
    {
      out = tf2::transformToEigen(tf_buffer_->lookupTransform(
        target, source, stamp, rclcpp::Duration::from_seconds(0.1)));
      return true;
    }
    catch (const tf2::TransformException &)
    {
      try
      {
        out = tf2::transformToEigen(tf_buffer_->lookupTransform(
          target, source, tf2::TimePointZero));
        return true;
      }
      catch (const tf2::TransformException & e)
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "TF %s -> %s failed: %s", source.c_str(), target.c_str(), e.what());
        return false;
      }
    }
  }

  // ---- params ----
  std::string input_topic_, map_frame_, openings_topic_, grid_topic_;
  double assoc_angle_deg_, assoc_dist_, assoc_overlap_gap_, provisional_timeout_;
  double opening_assoc_dist_;
  int min_observations_;
  bool use_openings_{false};
  bool use_grid_{false};
  int grid_occupied_thresh_, grid_hough_threshold_;
  double grid_min_wall_length_, grid_max_wall_gap_;
  double grid_match_dist_, grid_match_angle_deg_, grid_prune_timeout_, grid_merge_gap_;
  double grid_min_fill_ratio_, grid_opening_min_run_;
  double grid_max_wall_thickness_, grid_max_interior_ratio_;
  double cross_min_angle_deg_, cross_endpoint_margin_;
  bool require_lidar_confirmation_{false};
  double default_z_min_, default_z_max_;
  std::string base_frame_;
  double lidar_confirm_range_;
  int reject_after_sweeps_;
  int decay_after_sweeps_{20};
  double support_timeout_{3.0};
  bool snap_to_lidar_plane_{true};
  double snap_alpha_{0.3};
  double snap_max_shift_{0.0};
  int max_fuse_inliers_{300};
  bool confirm_gate_{true};
  std::string odom_topic_;
  double confirm_max_angular_vel_{0.3};
  double confirm_settle_time_{0.5};
  double confirm_max_cov_{0.0};
  bool have_odom_{false};
  rclcpp::Time last_unsettled_{0, 0, RCL_ROS_TIME};
  std::map<int, float> wall_snap_;  // EMA perpendicular snap per grid wall id
  bool save_walls_to_file_{true};
  std::string wall_file_path_;
  double wall_file_epsilon_{0.01};
  size_t last_saved_signature_{0};  // change-detect, so we only rewrite on change

  // ---- state ----
  std::vector<PWall> walls_;
  int next_id_{0};
  geometry_msgs::msg::PoseArray latest_openings_;
  bool have_openings_{false};
  std::vector<GridWall> grid_walls_;
  int next_grid_id_{0};
  bool have_grid_{false};
  std::vector<int> published_ids_;        // wall_map marker ids drawn last cycle
  std::vector<int> published_label_ids_;  // wall_id (number) marker ids drawn last cycle
  bool markers_cleared_{false};     // sent the one-shot startup DELETEALL yet?

  // ---- ROS ----
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<robo_drill::msg::WallArray>::SharedPtr sub_walls_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_openings_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_grid_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Publisher<robo_drill::msg::WallArray>::SharedPtr pub_map_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
};

}  // namespace robo_drill

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robo_drill::WallAggregatorNode>());
  rclcpp::shutdown();
  return 0;
}