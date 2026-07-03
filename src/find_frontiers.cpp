//////////////////////////////////////////////////////////////////////////////////////
//// APLICANDO "RAYTRACING" A LOS RAYOS DEL LIDAR
//////////////////////////////////////////////////////////////////////////////////////

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <deque>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "robo_drill/msg/frontiers.hpp"
#include <boost/pending/disjoint_sets.hpp>
#include <numeric>
#include <algorithm>

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/time.h"

#include <unordered_map>
#include <cmath>  

class FindFrontiers : public rclcpp::Node
{
private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_frontiers_pub_, map_free_pub_, map_filtered_pub_, map_working_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
    rclcpp::Publisher<robo_drill::msg::Frontiers>::SharedPtr frontiers_pub_;

    visualization_msgs::msg::MarkerArray markers_;
    int min_frontier_size_;
    double cell_size_;

    tf2_ros::Buffer tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    std::string map_frame_id_, robot_frame_id_;
    double blind_radius_;
    bool group_rings_;
    double ring_bin_m_;
    int free_dilate_iters_;

    // Gap-bridging parameters
    int  bridge_max_gap_cells_;     // e.g. 2 -> fill up to 2 unknown cells between free cells
    bool bridge_rows_;              // run horizontal pass
    bool bridge_cols_;              // run vertical pass
    int  bridge_every_n_frames_;    // run every N frames (1 = every frame)
    int  frame_count_;              // internal counter
    int  bridge_obst_max_gap_cells_;   // e.g. 2 -> fill up to 2 unknown cells between free cells
    bool bridge_obst_rows_;
    bool bridge_obst_cols_;

    // Hole filling (column interiors)
    bool fill_holes_enable_;
    int  fill_holes_max_area_cells_;   // umbral de área en celdas (p.ej. 300)
    int  fill_holes_pad_cells_;        // padding opcional (0..2) sólo sobre -1
    int  fill_holes_every_n_frames_;   // ejecutar cada N mapas (1 = cada mapa)
    int  fill_holes_counter_;          // contador interno

    // Split long frontiers and sharp turns
    bool   split_long_frontiers_;
    double max_frontier_segment_m_;
    bool   split_on_sharp_turns_;
    double turn_split_deg_;                 // umbral de giro (p.ej. 45°)
    int    min_segment_cells_after_turn_;   // tamaño mínimo tras cortar por codo
    int    turn_window_k_;                  // window half-size in cells to compute heading change

    // PRUNE after split (only when few segments remain)
    bool   prune_after_split_;
    int    prune_few_threshold_;
    int    prune_bfs_limit_;

public:
    FindFrontiers();

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid &msg);
    std::vector<int> twoPassLabeling(const nav_msgs::msg::OccupancyGrid &map_frontiers, std::map<int, int> &labels_sizes, bool diagonal) const;
    bool isFrontier(const int &cell, const nav_msgs::msg::OccupancyGrid &map) const;
    void publishMarkers(const robo_drill::msg::Frontiers &frontiers_msg);

    // UTILS
    int point2cell(const geometry_msgs::msg::Point &point, const nav_msgs::msg::OccupancyGrid &map_) const;
    geometry_msgs::msg::Point cell2point(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;

    std::vector<int> getStraightPoints(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;
    std::vector<int> getAdjacentPoints(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;

    int rightCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;
    int uprightCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;
    int upCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;
    int upleftCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;
    int leftCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;
    int downleftCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;
    int downCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;
    int downrightCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;
    int floor0(const float &value) const;
    void dilateFreeOnWorking(nav_msgs::msg::OccupancyGrid &grid, int iters) const;
    void carveRayFree(nav_msgs::msg::OccupancyGrid &g, int x0, int y0, int x1, int y1) const;
    void carveFreeWithRays(nav_msgs::msg::OccupancyGrid &g, double rx, double ry, double r) const;
    void bridgeSmallUnknownGaps(nav_msgs::msg::OccupancyGrid &g, int max_gap, bool do_rows, bool do_cols) const;
    void bridgeSmallFreeOrUnknownGaps(nav_msgs::msg::OccupancyGrid &g, int max_gap, bool do_rows, bool do_cols) const;
    void bridgeSmallUnknownGapsGeneric(nav_msgs::msg::OccupancyGrid &g,
                                   int max_gap, bool do_rows, bool do_cols,
                                   int endpoint_value, int fill_value) const;
    void fillSmallEnclosedUnknownHoles(nav_msgs::msg::OccupancyGrid &g, int max_area_cells, int pad_cells) const;
};

/**
 * @brief Constructor for the FindFrontiers node.
 * 
 * Initializes the ROS2 node with various parameters for frontier detection and processing.
 * Sets up TF2 buffer and listener for coordinate transformations between frames.
 * 
 * Parameters loaded:
 * - min_frontier_size: Minimum number of cells required to consider a frontier valid (default: 30)
 * - map_frame_id: Frame ID for the map coordinate system (default: "map")
 * - robot_frame_id: Frame ID for the robot base (default: "base_link")
 * - blind_radius: Radius in meters around the robot where frontiers are ignored, accounting
 *                 for the minimum range of the LiDAR sensor. Frontiers within this distance
 *                 are filtered out to avoid detection artifacts in the sensor's blind zone (default: 3.0m)
 * - group_rings: Enable/disable grouping of frontiers into radial rings (default: true)
 * - ring_bin_m: Size of radial bins for ring grouping (default: 1.5 * resolution)
 * - free_dilate_iters: Number of dilation iterations for free space (default: 1)
 * 
 * Gap-bridging parameters:
 * - bridge_max_gap_cells: Maximum gap size in cells to bridge in frontier regions (default: 5)
 * - bridge_rows/bridge_cols: Enable bridging in row/column directions (default: true)
 * - bridge_every_n_frames: Apply bridging every N frames (default: 1)
 * - bridge_obst_max_gap_cells: Maximum gap in obstacle regions to bridge (default: 10, 0=disabled)
 * - bridge_obst_rows/bridge_obst_cols: Enable obstacle bridging in row/column directions
 * 
 * Hole-filling parameters:
 * - fill_holes_enable: Enable filling of interior holes in columns (default: false)
 * - fill_holes_max_area_cells: Maximum area in cells for holes to fill (default: 300)
 * - fill_holes_pad_cells: Padding around holes (default: 0)
 * - fill_holes_every_n_frames: Apply hole filling every N frames (default: 1)
 * 
 * Frontier splitting parameters:
 * - split_long_frontiers: Enable splitting of long frontier segments (default: true)
 * - max_frontier_segment_m: Maximum segment length in meters before splitting (default: 3.5m)
 * - split_on_sharp_turns: Enable splitting at sharp directional changes (default: false)
 * - turn_split_deg: Angle threshold in degrees for turn detection (default: 45°)
 * - min_segment_cells_after_turn: Minimum cells after turn split (default: max(10, min_frontier_size_))
 * - turn_window_k: Window size for turn angle calculation (default: 3)
 * 
 * Post-split pruning parameters:
 * - prune_after_split: Enable pruning of small frontiers after splitting (default: true)
 * - prune_few_threshold: Only prune when fewer than N segments remain (default: 5)
 * - prune_bfs_limit: Maximum BFS nodes for reachability checks (default: 2000)
 * 
 * Creates publishers for:
 * - /frontiers: Custom frontier message
 * - map_frontiers, map_free, map_filtered, map_working: Occupancy grid visualizations
 * - frontiers_markers: Visualization markers
 * 
 * Subscribes to:
 * - /map: Occupancy grid input
 */
FindFrontiers::FindFrontiers() : Node("find_frontiers"), cell_size_(-1),
                                 tf_buffer_(this->get_clock()),
                                 tf_listener_(std::make_unique<tf2_ros::TransformListener>(tf_buffer_))
{
    // Declare & load parameter with default values
    get_parameter_or("min_frontier_size", min_frontier_size_, 30); // default 5 cells
    get_parameter_or("map_frame_id",   map_frame_id_,   std::string("map"));
    get_parameter_or("robot_frame_id", robot_frame_id_, std::string("base_link"));   // base_link    / sick_6  /  os_lidar
    get_parameter_or("blind_radius",   blind_radius_,   3.00); // en metros: AJUSTA a tu LiDAR (range mínimo proyectado)
    get_parameter_or("group_rings", group_rings_, true);
    get_parameter_or("ring_bin_m",  ring_bin_m_,  1.5 * this->declare_parameter("resolution_fallback", 0.05));
    get_parameter_or("free_dilate_iters", free_dilate_iters_, 1);

    // Gap-bridging
    get_parameter_or("bridge_max_gap_cells",  bridge_max_gap_cells_, 1); //! ????
    get_parameter_or("bridge_rows",           bridge_rows_,          true);
    get_parameter_or("bridge_cols",           bridge_cols_,          true);
    get_parameter_or("bridge_every_n_frames", bridge_every_n_frames_, 1);
    frame_count_ = 0;
    get_parameter_or("bridge_obst_max_gap_cells", bridge_obst_max_gap_cells_, 10);  //! 0 = desactivado
    get_parameter_or("bridge_obst_rows",          bridge_obst_rows_,          true);
    get_parameter_or("bridge_obst_cols",          bridge_obst_cols_,          true);

    // Hole filling (column interiors)
    get_parameter_or("fill_holes_enable",            fill_holes_enable_,            false);
    get_parameter_or("fill_holes_max_area_cells",    fill_holes_max_area_cells_,    300);
    get_parameter_or("fill_holes_pad_cells",         fill_holes_pad_cells_,         0);
    get_parameter_or("fill_holes_every_n_frames",    fill_holes_every_n_frames_,    1);
    fill_holes_counter_ = 0;

    // Split long frontiers and sharp turns
    get_parameter_or("split_long_frontiers",   split_long_frontiers_,   true);
    get_parameter_or("max_frontier_segment_m", max_frontier_segment_m_, 3.5); // p.ej. 2 m
    get_parameter_or("split_on_sharp_turns",          split_on_sharp_turns_,          false);
    get_parameter_or("turn_split_deg",                turn_split_deg_,                45.0);
    get_parameter_or("min_segment_cells_after_turn",  min_segment_cells_after_turn_,  std::max(10, min_frontier_size_));
    get_parameter_or("turn_window_k",                 turn_window_k_,                 3);

    // PRUNE after split (only when few segments remain)
    get_parameter_or("prune_after_split",    prune_after_split_,    true);
    get_parameter_or("prune_few_threshold",  prune_few_threshold_,  5);
    get_parameter_or("prune_bfs_limit",      prune_bfs_limit_,      2000); // cota de nodos en el BFS

    // publisher and subscribers
    frontiers_pub_ = this->create_publisher<robo_drill::msg::Frontiers>("/frontiers", 1);
    map_frontiers_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map_frontiers", 1);
    map_free_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map_free", 1);
    map_filtered_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map_filtered", 1);
    map_working_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map_working", 1);
    markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("frontiers_markers", 1);

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 1, std::bind(&FindFrontiers::mapCallback, this, std::placeholders::_1));
}

void FindFrontiers::mapCallback(const nav_msgs::msg::OccupancyGrid &msg)
{
    RCLCPP_DEBUG(this->get_logger(), "map received!");

    nav_msgs::msg::OccupancyGrid map_occupancy = msg;
    nav_msgs::msg::OccupancyGrid map_frontiers = msg;
    nav_msgs::msg::OccupancyGrid map_free = msg;
    nav_msgs::msg::OccupancyGrid map_working = map_occupancy;

    // init cell_size
    if (cell_size_ < 0)
        cell_size_ = msg.info.resolution;

    if (ring_bin_m_ <= 0.0) 
        ring_bin_m_ = 1.5 * msg.info.resolution;

    // reset maps
    map_frontiers.data.assign(map_frontiers.data.size(), 0);
    map_free.data.assign(map_free.data.size(), 0);

    // FILTER UNCONNECTED FREE CELLS
    // create map free (assign each cell if it is free)
    for (unsigned int i = 0; i < map_free.data.size(); ++i)
    {
        if (map_occupancy.data[i] == 0)
            map_free.data[i] = 100;
    }
    // Label free (connected cells)
    std::map<int, int> labels_free_sizes;
    std::vector<int> labels_free = twoPassLabeling(map_free, labels_free_sizes, false);
    // Classify as unknown all free groups except for the biggest one
    int remaining_label = -1;
    if (!labels_free_sizes.empty()) {
        remaining_label = std::max_element(labels_free_sizes.begin(),
                                        labels_free_sizes.end(),
                                        [](const std::pair<int, int> &p1, const std::pair<int, int> &p2)
                                        { return p1.second < p2.second; })
                            ->first;
    }
    if (remaining_label != -1) {
        for (unsigned int i = 0; i < map_occupancy.data.size(); ++i)
        {
            if (map_occupancy.data[i] == 0 && labels_free[i] != remaining_label)
            {
                map_occupancy.data[i] = -1;
                map_free.data[i] = 0;
            }
        }
    }
    // publish map_free
    map_free_pub_->publish(map_free);
    RCLCPP_DEBUG(this->get_logger(), "map free published!");

    // publish map_free
    map_filtered_pub_->publish(map_occupancy);
    RCLCPP_DEBUG(this->get_logger(), "map filtered published!");

    bool have_pose = false;
    double rx = 0.0, ry = 0.0;
    geometry_msgs::msg::TransformStamped tf;

    try {
        tf = tf_buffer_.lookupTransform(map_frame_id_, robot_frame_id_,
                                        tf2::TimePointZero, tf2::durationFromSec(1.20));
        rx = tf.transform.translation.x;
        ry = tf.transform.translation.y;
        have_pose = true;

        // Ray carving: libera solo hasta impacto con obstáculo
        this->carveFreeWithRays(map_working, rx, ry, blind_radius_);

    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "TF lookup failed for ray carving: %s", ex.what());
    }

    if (free_dilate_iters_ > 0) 
        dilateFreeOnWorking(map_working, free_dilate_iters_);

    if (bridge_max_gap_cells_ > 0 &&
        (frame_count_++ % std::max(1, bridge_every_n_frames_) == 0)) {
        // Puentes entre libres
        bridgeSmallUnknownGaps(map_working, bridge_max_gap_cells_, bridge_rows_, bridge_cols_);
    }

    if (bridge_obst_max_gap_cells_ > 0 &&
        (frame_count_++ % std::max(1, bridge_every_n_frames_) == 0)) {
            // Puentes entre obstáculos (rellena -1 con 100 si están entre 100..100)
            // bridgeSmallUnknownGapsGeneric(map_working,
            //                             bridge_obst_max_gap_cells_,
            //                             bridge_obst_rows_, bridge_obst_cols_,
            //                             /*endpoint_value=*/100, /*fill_value=*/100);
            bridgeSmallFreeOrUnknownGaps(map_working, bridge_max_gap_cells_, bridge_rows_, bridge_cols_);
    }

    if (fill_holes_enable_ && (++fill_holes_counter_ % std::max(1, fill_holes_every_n_frames_)) == 0) {
        fillSmallEnclosedUnknownHoles(map_working, fill_holes_max_area_cells_, fill_holes_pad_cells_);
    }

    const int W = (int)map_working.info.width;
    const int H = (int)map_working.info.height;
    auto inb = [&](int x,int y){ return x>=0 && x<W && y>=0 && y<H; };
    auto at  = [&](int x,int y){ return y*W + x; };

    // --- Reachable free desde el robot sobre map_working ---
    std::vector<uint8_t> reachable(map_working.data.size(), 0);

    // robot cell
    int rx_cell = -1, ry_cell = -1;
    if (have_pose) {
        rx_cell = (int)std::floor((rx - map_working.info.origin.position.x) / map_working.info.resolution);
        ry_cell = (int)std::floor((ry - map_working.info.origin.position.y) / map_working.info.resolution);
    }

    // busca una celda libre cercana si la del robot no es 0 (pequeño radio 1..2)
    int start_id = -1;
    for (int rad=0; rad<=2 && start_id==-1; ++rad){
        for (int dy=-rad; dy<=rad; ++dy){
            for (int dx=-rad; dx<=rad; ++dx){
                int x=rx_cell+dx, y=ry_cell+dy;
                if (!inb(x,y)) continue;
                int id=at(x,y);
                if (map_working.data[id]==0){ start_id=id; break; }
            }
            if (start_id!=-1) break;
        }
    }
    if (start_id != -1) {
        std::deque<int> q; 
        q.push_back(start_id); 
        reachable[start_id] = 1;

        const int dx4[4] = {1,-1,0,0}, dy4[4] = {0,0,1,-1};
        while (!q.empty()) {
            int id = q.front(); q.pop_front();
            int x = id % W, y = id / W;
            for (int k = 0; k < 4; ++k) {
                int nx = x + dx4[k], ny = y + dy4[k]; 
                if (!inb(nx,ny)) continue;
                int nid = at(nx,ny);
                if (map_working.data[nid] == 0 && !reachable[nid]) {
                    reachable[nid] = 1; 
                    q.push_back(nid);
                }
            }
        }
    }

    // --- Etiquetado de UNKNOWN y clasificación abierto/cerrado ---
    nav_msgs::msg::OccupancyGrid map_unknown = map_working;
    for (auto &v : map_unknown.data) v = (v == -1) ? 100 : 0;

    std::map<int,int> unk_sizes;
    std::vector<int> unk_labels = twoPassLabeling(map_unknown, unk_sizes, /*diagonal=*/false);
    std::unordered_map<int, bool> unknown_is_open;
    unknown_is_open.reserve(unk_sizes.size());

    // Conjunto de componentes UNKNOWN "abiertos"
    for (int i = 0; i < (int)map_working.data.size(); ++i) {
        if (map_working.data[i] != -1) continue;
        int lbl = unk_labels[i]; if (lbl == 0) continue;

        int x = i % W, y = i / W;
        // toca borde -> abierto
        if (x==0 || y==0 || x==W-1 || y==H-1) { unknown_is_open[lbl] = true; continue; }

        // si ya marcado abierto, sigue
        if (unknown_is_open[lbl]) continue;

        // ¿contorno con libre conectado al cluster libre principal?
        bool boundary_has_reachable_free = false;
        const int dx4[4] = {1,-1,0,0}, dy4[4] = {0,0,1,-1};
        for (int k = 0; k < 4 && !boundary_has_reachable_free; ++k) {
            int nx = x + dx4[k], ny = y + dy4[k]; 
            if (!inb(nx,ny)) continue;
            int nid = at(nx,ny);
            if (map_working.data[nid] == 0 && reachable[nid]) {
                boundary_has_reachable_free = true;
            }
        }
        if (boundary_has_reachable_free) unknown_is_open[lbl] = true;

    }

    // // FIND FRONTIERS
    // // create map frontiers (assign each cell if it is frontier)
    // for (unsigned int i = 0; i < map_frontiers.data.size(); ++i)
    // {
    //     if (isFrontier(i, map_working))
    //         map_frontiers.data[i] = 100;
    // }

    // FIND FRONTIERS (solo si toca UNKNOWN "abierto")
    for (unsigned int i = 0; i < map_frontiers.data.size(); ++i)
    {
        if (!isFrontier(i, map_working)) continue;

        // Verifica que al menos un vecino UNKNOWN pertenezca a un comp. abierto
        int W_ = (int)map_frontiers.info.width;
        int x = i % W_, y = i / W_;
        bool touches_open_unknown = false;
        // 4-conexión hacia UNKNOWN (suficiente y más estricta)
        const int dx4[4] = {1,-1,0,0}, dy4[4] = {0,0,1,-1};
        for (int k=0;k<4 && !touches_open_unknown;++k){
            int nx = x + dx4[k], ny = y + dy4[k];
            if (nx<0 || ny<0 || nx>=W_ || ny>=(int)map_frontiers.info.height) continue;
            int nid = ny*W_ + nx;
            if (map_working.data[nid] == -1) {
                int ul = unk_labels[nid];
                if (ul != 0 && unknown_is_open.count(ul) && unknown_is_open[ul]) {
                    touches_open_unknown = true;
                }
            }
        }
        if (touches_open_unknown) map_frontiers.data[i] = 100;
    }

    // publish map_working
    map_working_pub_->publish(map_working);
    RCLCPP_DEBUG(this->get_logger(), "map working published!");

    // publish map_frontiers
    map_frontiers_pub_->publish(map_frontiers);
    RCLCPP_DEBUG(this->get_logger(), "map frontiers published!");

    // Label frontiers (connected cells)
    std::map<int, int> labels_sizes;
    std::vector<int> labels = twoPassLabeling(map_frontiers, labels_sizes, true);

    // --- DEBUG: tamaños de fronteras (todas vs. aceptadas) ---
    auto print_sizes = [&](const std::vector<int>& sizes, const char* tag){
        if (sizes.empty()) {
            RCLCPP_DEBUG(this->get_logger(), "[frontiers] %s: n=0", tag);
            return;
        }
        std::vector<int> s = sizes; std::sort(s.begin(), s.end());
        long long sum = std::accumulate(s.begin(), s.end(), 0LL);
        double mean = double(sum) / double(s.size());
        int mn = s.front(), mx = s.back();
        int p50 = s[s.size()/2];
        int p90 = s[ (size_t)std::floor(0.9 * (s.size()-1)) ];

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "[frontiers] %s: n=%zu, min=%d, p50=%d, p90=%d, max=%d, mean=%.1f (min_frontier_size=%d)",
            tag, s.size(), mn, p50, p90, mx, mean, min_frontier_size_);
    };

    int f=0,o=0,u=0;
    for (auto v: map_working.data) { if(v==0)++f; else if(v==100)++o; else ++u; }
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
    "[working] free=%d occ=%d unk=%d", f,o,u);

    int open_cnt=0, closed_cnt=0;
    for (auto &kv: unknown_is_open) (kv.second?open_cnt:closed_cnt)++;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
        "[unknown] open=%d closed=%d", open_cnt, closed_cnt);

    std::vector<int> all_sizes;    all_sizes.reserve(labels_sizes.size());
    std::vector<int> kept_sizes;   kept_sizes.reserve(labels_sizes.size());
    for (const auto& kv : labels_sizes) {
        all_sizes.push_back(kv.second);
        if (kv.second >= min_frontier_size_) kept_sizes.push_back(kv.second);
    }

    // Estadísticas (throttled cada 2s)
    print_sizes(all_sizes,  "all");
    print_sizes(kept_sizes, "kept");

    // Create and fill frontiers message
    std::vector<int> frontiers_labels;
    robo_drill::msg::Frontiers frontiers_msg;
    frontiers_msg.header = msg.header;
    frontiers_msg.frontiers.clear();
    for (unsigned int i = 0; i < labels.size(); ++i)
    {
        if (labels[i] != 0) // labeled cell
        {
            // if too small continue
            if (labels_sizes[labels[i]] < min_frontier_size_)
                continue;

            // search existing frontier
            bool new_label = true;
            for (unsigned int j = 0; j < frontiers_msg.frontiers.size(); j++)
            {
                // found
                if (frontiers_labels[j] == labels[i])
                {
                    frontiers_msg.frontiers[j].cells.push_back(i);
                    frontiers_msg.frontiers[j].cells_points.push_back(cell2point(i, map_frontiers));
                    new_label = false;
                    break;
                }
            }
            // not found: create new frontier
            if (new_label)
            {
                robo_drill::msg::Frontier new_frontier;
                new_frontier.size = labels_sizes[labels[i]];
                new_frontier.cells.push_back(i);
                new_frontier.cells_points.push_back(cell2point(i, map_frontiers));
                frontiers_msg.frontiers.push_back(new_frontier);
                frontiers_labels.push_back(labels[i]);
            }
        }
    }

    // --- Split de fronteras por "codos" (giros bruscos) y, opcionalmente, por longitud ---
    {
        std::vector<robo_drill::msg::Frontier> new_frontiers;
        new_frontiers.reserve(frontiers_msg.frontiers.size());

        // target segment length in cells if also splitting by length
        const int seg_len_cells = (split_long_frontiers_ && max_frontier_segment_m_ > 0.0)
            ? std::max(min_frontier_size_, (int)std::round(max_frontier_segment_m_ / map_frontiers.info.resolution))
            : std::numeric_limits<int>::max();

        const double EPS = 1e-9;
        const double turn_thresh_rad = turn_split_deg_ * M_PI / 180.0;

        const int min_cells_threshold =
            split_on_sharp_turns_
                ? std::max(min_segment_cells_after_turn_, min_frontier_size_)
                : min_frontier_size_;

        auto angle_dir = [&](const geometry_msgs::msg::Point& a,
                             const geometry_msgs::msg::Point& b) -> double {
            return std::atan2(b.y - a.y, b.x - a.x);
        };
        auto angle_wrap_diff = [&](double a, double b) -> double {
            double d = a - b;
            while (d >  M_PI) d -= 2.0 * M_PI;
            while (d < -M_PI) d += 2.0 * M_PI;
            return std::fabs(d);
        };

        for (unsigned int i = 0; i < frontiers_msg.frontiers.size(); ++i) {
            int label = (i < (unsigned)frontiers_labels.size()) ? frontiers_labels[i] : 0;

            // ordenar la cadena de celdas de esta frontera (igual que antes)
            std::deque<int> ordered_cells;
            if (!frontiers_msg.frontiers[i].cells.empty())
                ordered_cells.push_back(frontiers_msg.frontiers[i].cells.front());

            while (ordered_cells.size() < frontiers_msg.frontiers[i].size) {
                auto initial_size = ordered_cells.size();

                std::vector<int> frontAdjacentPoints = getAdjacentPoints(ordered_cells.front(), map_frontiers);
                for (unsigned int k = 0; k < frontAdjacentPoints.size(); k++) {
                    int cand = frontAdjacentPoints[k];
                    if (cand != -1 && labels[cand] == label &&
                        std::find(ordered_cells.begin(), ordered_cells.end(), cand) == ordered_cells.end()) {
                        ordered_cells.push_front(cand);
                        break;
                    }
                }

                std::vector<int> backAdjacentPoints = getAdjacentPoints(ordered_cells.back(), map_frontiers);
                for (unsigned int k = 0; k < backAdjacentPoints.size(); k++) {
                    int cand = backAdjacentPoints[k];
                    if (cand != -1 && labels[cand] == label &&
                        std::find(ordered_cells.begin(), ordered_cells.end(), cand) == ordered_cells.end()) {
                        ordered_cells.push_back(cand);
                        break;
                    }
                }

                if (initial_size == ordered_cells.size() && ordered_cells.size() < frontiers_msg.frontiers[i].size)
                    break;
            }

            if (ordered_cells.empty()) continue;

            // vector plano de celdas y puntos
            std::vector<int> chain(ordered_cells.begin(), ordered_cells.end());
            std::vector<geometry_msgs::msg::Point> pts; pts.reserve(chain.size());
            for (int cell : chain) pts.push_back(cell2point(cell, map_frontiers));

            // cortes por codos con ventana (k - turn_window_k_ , k + turn_window_k_)
            std::vector<size_t> cuts; cuts.reserve(chain.size()+2);
            cuts.push_back(0);
            if (split_on_sharp_turns_) {
                const int n = std::max(1, turn_window_k_);
                if (pts.size() >= (size_t)(2*n + 1)) {
                    for (size_t k = (size_t)n; k + (size_t)n < pts.size(); ++k) {
                        const geometry_msgs::msg::Point& A = pts[k - n];
                        const geometry_msgs::msg::Point& B = pts[k];
                        const geometry_msgs::msg::Point& C = pts[k + n];

                        double len1 = std::hypot(B.x - A.x, B.y - A.y);
                        double len2 = std::hypot(C.x - B.x, C.y - B.y);
                        if (len1 < EPS || len2 < EPS) continue;

                        double ang1 = angle_dir(A, B);
                        double ang2 = angle_dir(B, C);
                        double dtheta = angle_wrap_diff(ang2, ang1); // heading change between long segments

                        if (dtheta >= turn_thresh_rad) {
                            cuts.push_back(k); // corta en el vértice del codo
                        }
                    }
                }
            }
            cuts.push_back(chain.size());

            // construir segmentos entre cortes (y opcionalmente también dividir por longitud)
            for (size_t ci = 0; ci + 1 < cuts.size(); ++ci) {
                size_t s = cuts[ci];
                size_t e = cuts[ci+1];

                for (size_t start = s; start < e; ) {
                    size_t end = std::min(e, start + (size_t)seg_len_cells);
                    int seg_cells = (int)(end - start);

                    if (seg_cells >= min_cells_threshold) {
                        robo_drill::msg::Frontier seg;
                        seg.size = seg_cells;
                        seg.cells.reserve(seg_cells);
                        seg.cells_points.reserve(seg_cells);
                        for (size_t k = start; k < end; ++k) {
                            seg.cells.push_back(chain[k]);
                            seg.cells_points.push_back(pts[k]);
                        }
                        // centro del segmento
                        seg.center_cell  = seg.cells[seg.size / 2];
                        seg.center_point = seg.cells_points[seg.size / 2];
                        new_frontiers.push_back(std::move(seg));
                    }
                    if (seg_len_cells == std::numeric_limits<int>::max()) break; // no length split
                    start = end;
                }
            }
        }

        frontiers_msg.frontiers.swap(new_frontiers);
    }

    // --- PRUNE: eliminar segmentos pegados a huecos cerrados (tras split) ---
    if (prune_after_split_ && (int)frontiers_msg.frontiers.size() <= prune_few_threshold_) {

        auto is_hollow_unknown = [&](int sx, int sy)->bool {
            // BFS sobre UNKNOWN empezando en (sx,sy), comprobando si su contorno
            // tiene libres (0) o solo obstáculos (100). Si solo obstáculos → hueco cerrado.
            std::deque<int> q;
            std::vector<uint8_t> vis(W*H, 0);

            int sid = at(sx,sy);
            if (!inb(sx,sy) || map_working.data[sid] != -1) return false;

            q.push_back(sid); vis[sid]=1;
            bool boundary_has_free=false, boundary_has_obst=false;
            int explored=0;

            const int dx4[4]={1,-1,0,0}, dy4[4]={0,0,1,-1};
            while(!q.empty()){
                int id=q.front(); q.pop_front();
                if (++explored > prune_bfs_limit_) break; // cota dura de seguridad

                int x = id % W, y = id / W;
                for (int k=0;k<4;++k){
                    int xx=x+dx4[k], yy=y+dy4[k];
                    if (!inb(xx,yy)) continue;
                    int nid=at(xx,yy);
                    int v = map_working.data[nid];
                    if (v == -1 && !vis[nid]) { vis[nid]=1; q.push_back(nid); }
                    else if (v == 0)   { boundary_has_free = true; }
                    else if (v == 100) { boundary_has_obst = true; }
                }
                if (boundary_has_free) return false; // no es hueco “cerrado”
            }
            return (boundary_has_obst && !boundary_has_free);
        };

        auto touches_hollow = [&](const robo_drill::msg::Frontier& F)->bool {
            // Chequea vecinos 4-conexión hacia UNKNOWN alrededor de c/u de las celdas del segmento
            const int dx4[4]={1,-1,0,0}, dy4[4]={0,0,1,-1};
            for (int c : F.cells){
                int cx = c % W, cy = c / W;
                for (int k=0;k<4;++k){
                    int nx=cx+dx4[k], ny=cy+dy4[k];
                    if (!inb(nx,ny)) continue;
                    if (map_working.data[at(nx,ny)] == -1 && is_hollow_unknown(nx,ny))
                        return true;
                }
            }
            return false;
        };

        std::vector<robo_drill::msg::Frontier> kept; kept.reserve(frontiers_msg.frontiers.size());
        for (const auto& seg : frontiers_msg.frontiers) {
            if (!touches_hollow(seg)) kept.push_back(seg);
        }
        frontiers_msg.frontiers.swap(kept);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "[prune] after-split kept=%zu", frontiers_msg.frontiers.size());
    }

    // Publish
    frontiers_pub_->publish(frontiers_msg);
    RCLCPP_DEBUG(this->get_logger(), "frontiers published!");
    publishMarkers(frontiers_msg);
    RCLCPP_DEBUG(this->get_logger(), "marker array published!");
}

// Check if a cell is frontier (free close to unknown)
bool FindFrontiers::isFrontier(const int &cell, const nav_msgs::msg::OccupancyGrid &map) const
{
    // if (map.data[cell] == 0) // check if it is free
    // {
    //     // /// NEIGBHOURS IN 4 DIRECTIONS
    //     // auto straightPoints = getStraightPoints(cell, map);
    //     // for (unsigned int i = 0; i < straightPoints.size(); ++i)
    //     //     if (straightPoints[i] != -1 && map.data[straightPoints[i]] == -1) // check if any neigbor is unknown
    //     //         return true;

    //     // /// NEIGBHOURS IN 8 DIRECTIONS
    //     // auto neigh = getAdjacentPoints(cell, map);
    //     // for (int idx : neigh)
    //     //     if (idx != -1 && map.data[idx] == -1) // any neighbor unknown?
    //     //         return true;
    // }
    // // If it is obstacle or unknown, it can not be frontier
    // return false;


    /// NO CORNER-CUTTING
    if (map.data[cell] != 0) return false; // solo libres pueden ser frontera

    const int W = (int)map.info.width;
    const int H = (int)map.info.height;
    auto at = [&](int x,int y){ return y*W + x; };

    int cx = cell % W, cy = cell / W;

    // 4-neighborhood: si hay UNKNOWN ortogonal, cuenta siempre
    const int dx4[4] = { 1,-1, 0, 0 };
    const int dy4[4] = { 0, 0, 1,-1 };
    for (int k=0;k<4;++k){
        int nx = cx + dx4[k], ny = cy + dy4[k];
        if (nx<0||nx>=W||ny<0||ny>=H) continue;
        int nid = at(nx,ny);
        if (map.data[nid] == -1) return true;
    }

    // 8-neighborhood diagonales: solo si NO hay "esquina bloqueada" por obstáculos
    const int dx8[4] = { 1, 1,-1,-1 };
    const int dy8[4] = { 1,-1, 1,-1 };
    for (int k=0;k<4;++k){
        int nx = cx + dx8[k], ny = cy + dy8[k];
        if (nx<0||nx>=W||ny<0||ny>=H) continue;
        int nid = at(nx,ny);
        if (map.data[nid] != -1) continue;

        // celdas ortogonales intermedias
        int ax = cx;       int ay = ny;     // (cx, ny)
        int bx = nx;       int by = cy;     // (nx, cy)

        bool corner_blocked =
            (ax>=0 && ax<W && ay>=0 && ay<H && (map.data[at(ax,ay)] == 100 || map.data[at(ax,ay)] == -1)) ||
            (bx>=0 && bx<W && by>=0 && by<H && (map.data[at(bx,by)] == 100 || map.data[at(bx,by)] == -1));

        if (!corner_blocked) return true; // diagonal válida (no atraviesa esquina de obstáculo)
    }

    return false;
}

// Two pass labeling to label frontiers [http://en.wikipedia.org/wiki/Connected-component_labeling]
std::vector<int> FindFrontiers::twoPassLabeling(const nav_msgs::msg::OccupancyGrid &map_labelr, std::map<int, int> &labels_sizes, bool diagonal) const
{
    labels_sizes.clear();
    std::vector<int> labels(map_labelr.data.size(), 0);
    // labels.assign(map_labelr.data.begin(), map_labelr.data.end());

    std::vector<int> neigh_labels;

    // Capacidad = nº de celdas + 1 (etiquetas empiezan en 1)
    const size_t MAX_LABELS = map_labelr.data.size() + 1;
    std::vector<int> rank(MAX_LABELS, 0);
    std::vector<int> parent(MAX_LABELS, 0);
    boost::disjoint_sets<int*, int*> dj_set(rank.data(), parent.data());
    int current_label_ = 1;

    // 1ST PASS: Assign temporary labels to frontiers and establish relationships
    for (unsigned int i = 0; i < map_labelr.data.size(); i++)
    {
        if (map_labelr.data[i] != 0)
        {
            neigh_labels.clear();
            // Find )8 or 4) connectivity neighbours already labeled
            if (diagonal && upleftCell(i, map_labelr) != -1 && labels[upleftCell(i, map_labelr)] != 0)
                neigh_labels.push_back(labels[upleftCell(i, map_labelr)]);
            if (upCell(i, map_labelr) != -1 && labels[upCell(i, map_labelr)] != 0)
                neigh_labels.push_back(labels[upCell(i, map_labelr)]);
            if (diagonal && uprightCell(i, map_labelr) != -1 && labels[uprightCell(i, map_labelr)] != 0)
                neigh_labels.push_back(labels[uprightCell(i, map_labelr)]);
            if (leftCell(i, map_labelr) != -1 && labels[leftCell(i, map_labelr)] != 0)
                neigh_labels.push_back(labels[leftCell(i, map_labelr)]);

            if (neigh_labels.empty()) // case: No neighbours
            {
                dj_set.make_set(current_label_); //   create new set of labels
                labels[i] = current_label_;      //   update cell's label
                current_label_++;                //   update label
            }
            else // case: With neighbours
            {
                labels[i] = *std::min_element(neigh_labels.begin(), neigh_labels.end()); //   choose minimum label of the neighbours
                for (unsigned int j = 0; j < neigh_labels.size(); ++j)                   //   update neighbours sets
                    dj_set.union_set(labels[i], neigh_labels[j]);                        //   unite sets minimum label with the others
            }
        }
    }

    // 2ND PASS: Assign final label
    // compress sets for efficiency
    for (unsigned int i = 0; i < map_labelr.data.size(); i++)
        if (labels[i] != 0)
        {
            // relabel each element with the lowest equivalent label
            labels[i] = dj_set.find_set(labels[i]);
            // increment the size of the label
            if (labels_sizes.count(labels[i]) == 0)
                labels_sizes[labels[i]] = 1;
            else
                labels_sizes[labels[i]]++;
        }

    return labels;
}

void FindFrontiers::publishMarkers(const robo_drill::msg::Frontiers &frontiers_msg)
{
    // resize
    markers_.markers.clear();
    markers_.markers.resize(frontiers_msg.frontiers.size() * 3 + 1); // each frontier: cells, center_point and text

    // deleteall
    markers_.markers[0].action = visualization_msgs::msg::Marker::DELETEALL;

    // omplir
    for (unsigned int i = 0; i < frontiers_msg.frontiers.size(); i++)
    {
        std_msgs::msg::ColorRGBA c;
        double x = (double)i / (double)frontiers_msg.frontiers.size();
        c.a = 1.0;
        c.r = x < 0.33 ? 1 - 0.33 * x : (x < 0.66 ? 0 : 3 * (x - 0.66));
        c.g = x < 0.33 ? 3 * x : (x < 0.66 ? 1 - 0.33 * (x - 0.33) : 0);
        c.b = x < 0.33 ? 0 : (x < 0.66 ? 3 * (x - 0.33) : 1 - 0.33 * (x - 0.66));

        // RCLCPP_INFO(this->get_logger(), "Colors: i = %u\nx = %f\nr = %f\ng = %f\nb = %f", i, x, c.r, c.g, c.b);

        // point center
        visualization_msgs::msg::Marker marker_center;
        marker_center.header.frame_id = frontiers_msg.header.frame_id;
        marker_center.header.stamp = this->now();
        marker_center.lifetime = rclcpp::Duration(0, 0);
        marker_center.type = visualization_msgs::msg::Marker::CYLINDER;
        marker_center.scale.x = marker_center.scale.y = 0.1;
        marker_center.scale.z = 0.5;
        marker_center.color = c;
        marker_center.ns = "centers";
        marker_center.id = 3 * i;
        marker_center.pose.position.x = frontiers_msg.frontiers[i].center_point.x;
        marker_center.pose.position.y = frontiers_msg.frontiers[i].center_point.y;
        marker_center.pose.position.z = marker_center.scale.z / 2;
        marker_center.pose.orientation.x = marker_center.pose.orientation.y = marker_center.pose.orientation.z = 0;
        marker_center.pose.orientation.w = 1;
        markers_.markers[3 * i + 1] = marker_center;

        // cells
        visualization_msgs::msg::Marker marker_points;
        marker_points.header.frame_id = frontiers_msg.header.frame_id;
        marker_points.header.stamp = this->now();
        marker_points.lifetime = rclcpp::Duration(0, 0);
        marker_points.type = visualization_msgs::msg::Marker::POINTS;
        marker_points.pose.orientation.x = marker_points.pose.orientation.y = marker_points.pose.orientation.z = 0;
        marker_points.pose.orientation.w = 1;
        marker_points.points = frontiers_msg.frontiers[i].cells_points;
        marker_points.scale.x = marker_points.scale.y = marker_points.scale.z = cell_size_;
        marker_points.colors = std::vector<std_msgs::msg::ColorRGBA>(marker_points.points.size(), c);
        marker_points.ns = "cells";
        marker_points.id = 3 * i + 1;
        markers_.markers[3 * i + 2] = marker_points;

        // text
        visualization_msgs::msg::Marker marker_id;
        marker_id.header.frame_id = frontiers_msg.header.frame_id;
        marker_id.header.stamp = this->now();
        marker_id.lifetime = rclcpp::Duration(0, 0);
        marker_id.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        marker_id.scale.x = marker_id.scale.y = 1;
        marker_id.scale.z = 0.5;
        marker_id.text = std::to_string(i);
        marker_id.color = c;
        marker_id.pose = marker_center.pose;
        marker_id.pose.position.z = marker_center.scale.z + 0.2;
        marker_id.ns = "id";
        marker_id.id = 3 * i + 2;
        markers_.markers[3 * i + 3] = marker_id;
    }
    // publish
    markers_pub_->publish(markers_);
}

// UTILS ////////////////////////////////////////////////////////////////////////////////////////////////////
int FindFrontiers::point2cell(const geometry_msgs::msg::Point &point, const nav_msgs::msg::OccupancyGrid &map_) const
{
    if (point.x <= map_.info.origin.position.x || point.x >= map_.info.width * map_.info.resolution + map_.info.origin.position.x ||
        point.y <= map_.info.origin.position.y || point.y >= map_.info.height * map_.info.resolution + map_.info.origin.position.y)
    {
        return -1;
    }

    int x_cell = floor0((point.x - map_.info.origin.position.x) / map_.info.resolution);
    int y_cell = floor0((point.y - map_.info.origin.position.y) / map_.info.resolution);
    int cell = x_cell + (y_cell)*map_.info.width;
    return cell;
}

geometry_msgs::msg::Point FindFrontiers::cell2point(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
{
    geometry_msgs::msg::Point point;
    point.x = (cell % map_.info.width) * map_.info.resolution + map_.info.origin.position.x + map_.info.resolution / 2;
    point.y = floor(cell / map_.info.width) * map_.info.resolution + map_.info.origin.position.y + map_.info.resolution / 2;
    return point;
}

std::vector<int> FindFrontiers::getStraightPoints(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
{
    return std::vector<int>({leftCell(cell, map_),
                             upCell(cell, map_),
                             rightCell(cell, map_),
                             downCell(cell, map_)});
}
std::vector<int> FindFrontiers::getAdjacentPoints(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
{
    return std::vector<int>({leftCell(cell, map_),
                             upCell(cell, map_),
                             rightCell(cell, map_),
                             downCell(cell, map_),
                             upleftCell(cell, map_),
                             uprightCell(cell, map_),
                             downrightCell(cell, map_),
                             downleftCell(cell, map_)});
}
int FindFrontiers::rightCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
{
    // x < width-1
    const int W = static_cast<int>(map_.info.width);
    if ((cell % W) != W - 1) 
        return cell + 1;
    return -1;
}

int FindFrontiers::uprightCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
{
    // x < width-1 && y > 0
    const int W = static_cast<int>(map_.info.width);
    if (((cell % W) != W - 1) && (cell >= W))
        return cell - W + 1;
    return -1;
}

int FindFrontiers::upCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
{
    if (cell >= (int)map_.info.width)
        return cell - map_.info.width;
    return -1;
}

int FindFrontiers::upleftCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
{
    // x > 0 && y > 0
    if ((cell >= (int)map_.info.width) && ((cell % (int)map_.info.width) != 0))
        return cell - map_.info.width - 1;
    return -1;
}

int FindFrontiers::leftCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
{
    // x > 0
    if ((cell % map_.info.width) != 0)
        return cell - 1;
    return -1;
}

int FindFrontiers::downleftCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
{
    // x > 0 && y < height-1
    if (((cell % map_.info.width) != 0) && ((cell / map_.info.width) < (map_.info.height - 1)))
        return cell + map_.info.width - 1;
    return -1;
}

int FindFrontiers::downCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
{
    if ((cell / map_.info.width) < (map_.info.height - 1))
        return cell + map_.info.width;
    return -1;
}

int FindFrontiers::downrightCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
{
    // x < width-1 && y < height-1
    const int W = static_cast<int>(map_.info.width), H = static_cast<int>(map_.info.height);
    if (((cell / W) < (H - 1)) && ((cell % W) != W - 1))
        return cell + W + 1;
    return -1;
}

int FindFrontiers::floor0(const float &value) const
{
    if (value < 0.0)
        return ceil(value);
    else
        return floor(value);
}

void FindFrontiers::dilateFreeOnWorking(nav_msgs::msg::OccupancyGrid &g, int iters) const
{
    const int W = g.info.width, H = g.info.height;
    auto at = [&](int x, int y){ return y*W + x; };

    std::vector<int8_t> src = g.data, dst(W*H);
    for (int it = 0; it < iters; ++it) {
        for (int y = 0; y < H; ++y) {
            for (int x = 0; x < W; ++x) {
                int idx = at(x,y);
                if (src[idx] == 100) { dst[idx] = 100; continue; } // nunca pisar obstáculos
                if (src[idx] == 0)   { dst[idx] = 0;   continue; } // ya libre

                // src[idx] == -1 (desconocido): se vuelve libre si hay libre en vecindad 8
                bool near_free = false;
                for (int dy = -1; dy <= 1 && !near_free; ++dy) {
                    int yy = y + dy; if (yy < 0 || yy >= H) continue;
                    for (int dx = -1; dx <= 1; ++dx) {
                        int xx = x + dx; if (xx < 0 || xx >= W) continue;
                        if (src[at(xx,yy)] == 0) { near_free = true; break; }
                    }
                }
                dst[idx] = near_free ? 0 : -1;
            }
        }
        src.swap(dst);
    }
    g.data.assign(src.begin(), src.end());
}

void FindFrontiers::carveRayFree(nav_msgs::msg::OccupancyGrid &g, int x0, int y0, int x1, int y1) const
{
    const int W = static_cast<int>(g.info.width), H = static_cast<int>(g.info.height);
    auto in_bounds = [&](int x,int y){ return (x>=0 && x<W && y>=0 && y<H); };
    auto at       = [&](int x,int y){ return y*W + x; };

    int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;

    int x = x0, y = y0;
    while (true) {
        if (!in_bounds(x,y)) break;
        int id = at(x,y);
        if (g.data[id] == 100) break;          // obstáculo => corta
        if (g.data[id] != 100) g.data[id] = 0; // marca libre
        if (x == x1 && y == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x += sx; }
        if (e2 <= dx) { err += dx; y += sy; }
    }
}

void FindFrontiers::carveFreeWithRays(nav_msgs::msg::OccupancyGrid &g, double rx, double ry, double r) const
{
    const double res = g.info.resolution;
    const int W = static_cast<int>(g.info.width), H = static_cast<int>(g.info.height);
    auto in_bounds = [&](int x,int y){ return (x>=0 && x<W && y>=0 && y<H); };

    if (r  <= 0.0001)
        return;

    int x0 = static_cast<int>(std::floor((rx - g.info.origin.position.x) / res));
    int y0 = static_cast<int>(std::floor((ry - g.info.origin.position.y) / res));
    if (!in_bounds(x0,y0)) return;

    // ~1 celda por arco; mínimo 32 rayos
    int samples = std::max(32, static_cast<int>(std::ceil((2.0 * M_PI * r) / res)));
    const double dtheta = (2.0 * M_PI) / samples;

    for (int i = 0; i < samples; ++i) {
        double theta = i * dtheta;
        double ex = rx + r * std::cos(theta);
        double ey = ry + r * std::sin(theta);
        int x1 = static_cast<int>(std::floor((ex - g.info.origin.position.x) / res));
        int y1 = static_cast<int>(std::floor((ey - g.info.origin.position.y) / res));
        if (!in_bounds(x1,y1)) continue;
        carveRayFree(g, x0, y0, x1, y1);
    }
}

void FindFrontiers::bridgeSmallUnknownGaps(nav_msgs::msg::OccupancyGrid &g,
                                           int max_gap, bool do_rows, bool do_cols) const
{
    // Fill short runs of UNKNOWN (-1) sandwiched by FREE (0), without touching obstacles (100).
    if (max_gap <= 0) return;

    const int W = static_cast<int>(g.info.width);
    const int H = static_cast<int>(g.info.height);
    auto at = [&](int x,int y){ return y*W + x; };

    // --- Horizontal pass (rows) ---
    if (do_rows) {
        for (int y = 0; y < H; ++y) {
            int x = 0;
            while (x < W) {
                // advance to a FREE cell
                while (x < W && g.data[at(x,y)] != 0) ++x;
                if (x >= W) break;

                // count UNKNOWN gap right after this FREE cell
                int k = x + 1;
                while (k < W && g.data[at(k,y)] == -1) ++k;
                int gap = k - (x + 1);

                // if gap in (0..max_gap] and next is FREE, fill UNKNOWN with FREE
                if (gap > 0 && gap <= max_gap && k < W && g.data[at(k,y)] == 0) {
                    for (int t = x + 1; t < k; ++t) {
                        if (g.data[at(t,y)] != 100) g.data[at(t,y)] = 0; // never overwrite obstacles
                    }
                    x = k; // continue after the right FREE endpoint
                } else {
                    // no valid bridge; skip ahead
                    x = std::max(k, x + 1);
                }
            }
        }
    }

    // --- Vertical pass (columns) ---
    if (do_cols) {
        for (int x = 0; x < W; ++x) {
            int y = 0;
            while (y < H) {
                while (y < H && g.data[at(x,y)] != 0) ++y;
                if (y >= H) break;

                int k = y + 1;
                while (k < H && g.data[at(x,k)] == -1) ++k;
                int gap = k - (y + 1);

                if (gap > 0 && gap <= max_gap && k < H && g.data[at(x,k)] == 0) {
                    for (int t = y + 1; t < k; ++t) {
                        if (g.data[at(x,t)] != 100) g.data[at(x,t)] = 0;
                    }
                    y = k;
                } else {
                    y = std::max(k, y + 1);
                }
            }
        }
    }

    // // Versión original (UNKNOWN entre libres -> se rellena con FREE)
    // bridgeSmallUnknownGapsGeneric(g, max_gap, do_rows, do_cols, /*endpoint_value=*/0, /*fill_value=*/0);
}

void FindFrontiers::bridgeSmallFreeOrUnknownGaps(nav_msgs::msg::OccupancyGrid &g,
                                                 int max_gap, bool do_rows, bool do_cols) const
{
    if (max_gap <= 0) return;

    const int W = static_cast<int>(g.info.width);
    const int H = static_cast<int>(g.info.height);
    auto at = [&](int x,int y){ return y*W + x; };

    // --- Horizontal pass (rows) ---
    if (do_rows) {
        for (int y = 0; y < H; ++y) {
            int x = 0;
            while (x < W) {
                // avanzar hasta OBSTACLE
                while (x < W && g.data[at(x,y)] != 100) ++x;
                if (x >= W) break;

                int k = x + 1;
                // contar FREE o UNKNOWN después del obstáculo
                while (k < W && (g.data[at(k,y)] == 0 || g.data[at(k,y)] == -1)) ++k;
                int gap = k - (x + 1);

                // si gap <= max_gap y termina en OBSTACLE
                if (gap > 0 && gap <= max_gap && k < W && g.data[at(k,y)] == 100) {
                    for (int t = x + 1; t < k; ++t) {
                        g.data[at(t,y)] = 100; // rellenar como obstáculo
                    }
                    x = k;
                } else {
                    x = std::max(k, x + 1);
                }
            }
        }
    }

    // --- Vertical pass (columns) ---
    if (do_cols) {
        for (int x = 0; x < W; ++x) {
            int y = 0;
            while (y < H) {
                while (y < H && g.data[at(x,y)] != 100) ++y;
                if (y >= H) break;

                int k = y + 1;
                while (k < H && (g.data[at(x,k)] == 0 || g.data[at(x,k)] == -1)) ++k;
                int gap = k - (y + 1);

                if (gap > 0 && gap <= max_gap && k < H && g.data[at(x,k)] == 100) {
                    for (int t = y + 1; t < k; ++t) {
                        g.data[at(x,t)] = 100;
                    }
                    y = k;
                } else {
                    y = std::max(k, y + 1);
                }
            }
        }
    }
}

void FindFrontiers::bridgeSmallUnknownGapsGeneric(nav_msgs::msg::OccupancyGrid &g,
                                                  int max_gap, bool do_rows, bool do_cols,
                                                  int endpoint_value, int fill_value) const
{
    // Rellena runs de -1 (UNKNOWN) entre dos celdas endpoint_value (0 o 100)
    if (max_gap <= 0) return;

    const int W = static_cast<int>(g.info.width);
    const int H = static_cast<int>(g.info.height);
    auto at = [&](int x,int y){ return y*W + x; };

    auto try_bridge_row = [&](int y){
        int x = 0;
        while (x < W) {
            // avanza hasta encontrar el primer endpoint
            while (x < W && g.data[at(x,y)] != endpoint_value) ++x;
            if (x >= W) return;

            // cuenta UNKNOWN seguidos
            int k = x + 1;
            while (k < W && g.data[at(k,y)] == -1) ++k;
            int gap = k - (x + 1);

            // puente si el siguiente es endpoint y el gap cabe en max_gap
            if (gap > 0 && gap <= max_gap && k < W && g.data[at(k,y)] == endpoint_value) {
                for (int t = x + 1; t < k; ++t) {
                    // nunca pises el “otro” endpoint (ej. no pises libres si estás rellenando obstáculos)
                    if (fill_value == 0 && g.data[at(t,y)] == 100) continue; // no pisar obstáculos cuando rellenamos libres
                    if (fill_value == 100) {
                        g.data[at(t,y)] = 100; // al unificar obstáculos, pisar TODO (libre o desconocido)
                    } else {
                        g.data[at(t,y)] = 0;
                    }
                }
                x = k;  // continúa tras el endpoint derecho
            } else {
                x = std::max(k, x + 1);
            }
        }
    };

    auto try_bridge_col = [&](int x){
        int y = 0;
        while (y < H) {
            while (y < H && g.data[at(x,y)] != endpoint_value) ++y;
            if (y >= H) return;

            int k = y + 1;
            while (k < H && g.data[at(x,k)] == -1) ++k;
            int gap = k - (y + 1);

            if (gap > 0 && gap <= max_gap && k < H && g.data[at(x,k)] == endpoint_value) {
                for (int t = y + 1; t < k; ++t) {
                    if (fill_value == 0 && g.data[at(x,t)] == 100) continue; // no pisar obstáculos cuando rellenamos libres
                    if (fill_value == 100) {
                        g.data[at(x,t)] = 100; // al unificar obstáculos, pisar TODO (libre o desconocido)
                    } else {
                        g.data[at(x,t)] = 0;
                    }
                }
                y = k;
            } else {
                y = std::max(k, y + 1);
            }
        }
    };

    if (do_rows) for (int y = 0; y < H; ++y) try_bridge_row(y);
    if (do_cols) for (int x = 0; x < W; ++x) try_bridge_col(x);
}


void FindFrontiers::fillSmallEnclosedUnknownHoles(nav_msgs::msg::OccupancyGrid &g,
                                                  int max_area_cells, int pad_cells) const
{
    if (max_area_cells <= 0) return;

    const int W = static_cast<int>(g.info.width);
    const int H = static_cast<int>(g.info.height);
    const int N = W * H;

    auto at  = [&](int x,int y){ return y*W + x; };
    auto inb = [&](int x,int y){ return (x>=0 && x<W && y>=0 && y<H); };

    std::vector<uint8_t> visited(N, 0);
    const int dx4[4] = { 1,-1, 0, 0 };
    const int dy4[4] = { 0, 0, 1,-1 };
    const int dx8[8] = { 1,-1, 0, 0, 1, 1,-1,-1 };
    const int dy8[8] = { 0, 0, 1,-1, 1,-1, 1,-1 };

    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            const int start = at(x,y);
            if (visited[start] || g.data[start] != -1) continue;

            // BFS del componente UNKNOWN (-1)
            std::deque<int> q;
            std::vector<int> comp;
            bool touches_border      = false;
            bool boundary_has_free   = false;   // <-- NUEVO
            bool boundary_has_obst   = false;   // <-- NUEVO

            visited[start] = 1;
            q.push_back(start);
            comp.push_back(start);

            while (!q.empty()) {
                int id = q.front(); q.pop_front();
                int cx = id % W, cy = id / W;

                if (cx == 0 || cy == 0 || cx == W-1 || cy == H-1) touches_border = true;

                for (int k = 0; k < 4; ++k) {
                    int nx = cx + dx4[k], ny = cy + dy4[k];
                    if (!inb(nx,ny)) continue;
                    int nid = at(nx,ny);

                    if (g.data[nid] == -1) {
                        if (!visited[nid]) {
                            visited[nid] = 1;
                            q.push_back(nid);
                            comp.push_back(nid);
                        }
                    } else {
                        // Clasificamos el contorno del componente
                        if (g.data[nid] == 0)   boundary_has_free = true;
                        if (g.data[nid] == 100) boundary_has_obst = true;
                    }
                }
            }

            // *** REGLA: sólo rellenar como obstáculo si:
            //  - no toca el borde del mapa
            //  - su contorno NO tiene libres y SÍ tiene obstáculos
            //  - y el área es pequeña
            if (!touches_border &&
                boundary_has_obst && !boundary_has_free &&
                static_cast<int>(comp.size()) <= max_area_cells)
            {
                for (int id : comp) {
                    g.data[id] = 100;
                }

                // padding opcional sólo sobre -1 adyacente
                for (int p = 0; p < pad_cells; ++p) {
                    std::vector<int> to_fill;
                    to_fill.reserve(comp.size() * 2);
                    for (int id : comp) {
                        int cx = id % W, cy = id / W;
                        for (int k = 0; k < 8; ++k) {
                            int nx = cx + dx8[k], ny = cy + dy8[k];
                            if (!inb(nx,ny)) continue;
                            int nid = at(nx,ny);
                            if (g.data[nid] == -1) {
                                g.data[nid] = 100;
                                to_fill.push_back(nid);
                            }
                        }
                    }
                    comp.insert(comp.end(), to_fill.begin(), to_fill.end());
                    if (to_fill.empty()) break;
                }
            }
            // Si el contorno tiene libres (isla en zona libre), NO lo tocamos aquí:
            // lo puede resolver tu "bridge" (si es pequeño) o lo dejamos como -1.
        }
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FindFrontiers>());
    rclcpp::shutdown();
    return 0;
}












//////////////////////////////////////////////////////////////////////////////////////
//// ORIGINAL
//////////////////////////////////////////////////////////////////////////////////////

// #include <chrono>
// #include <functional>
// #include <memory>
// #include <string>
// #include <deque>

// #include "rclcpp/rclcpp.hpp"
// #include "visualization_msgs/msg/marker_array.hpp"
// #include "nav_msgs/msg/occupancy_grid.hpp"
// #include "std_msgs/msg/color_rgba.hpp"
// #include "robo_drill_interfaces/msg/frontiers.hpp"
// #include <boost/pending/disjoint_sets.hpp>
// #include <numeric>
// #include <algorithm>

// class FindFrontiers : public rclcpp::Node
// {
// private:
//     rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
//     rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_frontiers_pub_, map_free_pub_, map_filtered_pub_;
//     rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
//     rclcpp::Publisher<robo_drill_interfaces::msg::Frontiers>::SharedPtr frontiers_pub_;

//     visualization_msgs::msg::MarkerArray markers_;
//     int min_frontier_size_;
//     double cell_size_;

// public:
//     FindFrontiers();

// private:
//     void mapCallback(const nav_msgs::msg::OccupancyGrid &msg);
//     std::vector<int> twoPassLabeling(const nav_msgs::msg::OccupancyGrid &map_frontiers, std::map<int, int> &labels_sizes, bool diagonal) const;
//     bool isFrontier(const int &cell, const nav_msgs::msg::OccupancyGrid &map) const;
//     void publishMarkers(const robo_drill_interfaces::msg::Frontiers &frontiers_msg);

//     // UTILS
//     int point2cell(const geometry_msgs::msg::Point &point, const nav_msgs::msg::OccupancyGrid &map_) const;
//     geometry_msgs::msg::Point cell2point(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;

//     std::vector<int> getStraightPoints(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;
//     std::vector<int> getAdjacentPoints(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;

//     int rightCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;
//     int uprightCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;
//     int upCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;
//     int upleftCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;
//     int leftCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;
//     int downleftCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;
//     int downCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;
//     int downrightCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const;
//     int floor0(const float &value) const;
// };

// FindFrontiers::FindFrontiers() : Node("find_frontiers"), cell_size_(-1)
// {
//     // Declare & load parameter with default values
//     get_parameter_or("min_frontier_size", min_frontier_size_, 5); // default 5 cells

//     // publisher and subscribers
//     frontiers_pub_ = this->create_publisher<robo_drill_interfaces::msg::Frontiers>("/frontiers", 1);
//     map_frontiers_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map_frontiers", 1);
//     map_free_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map_free", 1);
//     map_filtered_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map_filtered", 1);
//     markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("frontiers_markers", 1);

//     map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
//         "/map", 1, std::bind(&FindFrontiers::mapCallback, this, std::placeholders::_1));
// }

// void FindFrontiers::mapCallback(const nav_msgs::msg::OccupancyGrid &msg)
// {
//     RCLCPP_DEBUG(this->get_logger(), "map received!");

//     nav_msgs::msg::OccupancyGrid map_occupancy = msg;
//     nav_msgs::msg::OccupancyGrid map_frontiers = msg;
//     nav_msgs::msg::OccupancyGrid map_free = msg;

//     // init cell_size
//     if (cell_size_ < 0)
//         cell_size_ = msg.info.resolution;

//     // reset maps
//     map_frontiers.data.assign(map_frontiers.data.size(), 0);
//     map_free.data.assign(map_free.data.size(), 0);

//     // FILTER UNCONNECTED FREE CELLS
//     // create map free (assign each cell if it is free)
//     for (unsigned int i = 0; i < map_free.data.size(); ++i)
//     {
//         if (map_occupancy.data[i] == 0)
//             map_free.data[i] = 100;
//     }
//     // Label free (connected cells)
//     std::map<int, int> labels_free_sizes;
//     std::vector<int> labels_free = twoPassLabeling(map_free, labels_free_sizes, false);
//     // Classify as unknown all free groups except for the biggest one
//     int remaining_label = std::max_element(labels_free_sizes.begin(),
//                                            labels_free_sizes.end(),
//                                            [](const std::pair<int, int> &p1, const std::pair<int, int> &p2)
//                                            { return p1.second < p2.second; })
//                               ->first;
//     for (unsigned int i = 0; i < map_occupancy.data.size(); ++i)
//     {
//         if (map_occupancy.data[i] == 0 and labels_free[i] != remaining_label)
//         {
//             map_occupancy.data[i] = -1;
//             map_free.data[i] = 0;
//         }
//     }
//     // publish map_free
//     map_free_pub_->publish(map_free);
//     RCLCPP_DEBUG(this->get_logger(), "map free published!");

//     // publish map_free
//     map_filtered_pub_->publish(map_occupancy);
//     RCLCPP_DEBUG(this->get_logger(), "map filtered published!");

//     // FIND FRONTIERS
//     // create map frontiers (assign each cell if it is frontier)
//     for (unsigned int i = 0; i < map_frontiers.data.size(); ++i)
//     {
//         if (isFrontier(i, map_occupancy))
//             map_frontiers.data[i] = 100;
//     }

//     // publish map_frontiers
//     map_frontiers_pub_->publish(map_frontiers);
//     RCLCPP_DEBUG(this->get_logger(), "map frontiers published!");

//     // Label frontiers (connected cells)
//     std::map<int, int> labels_sizes;
//     std::vector<int> labels = twoPassLabeling(map_frontiers, labels_sizes, true);

//     // Create and fill frontiers message
//     std::vector<int> frontiers_labels;
//     robo_drill_interfaces::msg::Frontiers frontiers_msg;
//     frontiers_msg.header = msg.header;
//     frontiers_msg.frontiers.clear();
//     for (unsigned int i = 0; i < labels.size(); ++i)
//     {
//         if (labels[i] != 0) // labeled cell
//         {
//             // if too small continue
//             if (labels_sizes[labels[i]] < min_frontier_size_)
//                 continue;

//             // search existing frontier
//             bool new_label = true;
//             for (unsigned int j = 0; j < frontiers_msg.frontiers.size(); j++)
//             {
//                 // found
//                 if (frontiers_labels[j] == labels[i])
//                 {
//                     frontiers_msg.frontiers[j].cells.push_back(i);
//                     frontiers_msg.frontiers[j].cells_points.push_back(cell2point(i, map_frontiers));
//                     new_label = false;
//                     break;
//                 }
//             }
//             // not found: create new frontier
//             if (new_label)
//             {
//                 robo_drill_interfaces::msg::Frontier new_frontier;
//                 new_frontier.size = labels_sizes[labels[i]];
//                 new_frontier.cells.push_back(i);
//                 new_frontier.cells_points.push_back(cell2point(i, map_frontiers));
//                 frontiers_msg.frontiers.push_back(new_frontier);
//                 frontiers_labels.push_back(labels[i]);
//             }
//         }
//     }

//     // Compute center cell
//     for (unsigned int i = 0; i < frontiers_msg.frontiers.size(); ++i)
//     {
//         int label = frontiers_labels[i];

//         // order the frontier cells
//         std::deque<int> ordered_cells(0);
//         ordered_cells.push_back(frontiers_msg.frontiers[i].cells.front());
//         while (ordered_cells.size() < frontiers_msg.frontiers[i].size)
//         {
//             auto initial_size = ordered_cells.size();

//             // connect cells to first cell
//             std::vector<int> frontAdjacentPoints = getAdjacentPoints(ordered_cells.front(), map_frontiers);
//             for (unsigned int k = 0; k < frontAdjacentPoints.size(); k++)
//                 if (frontAdjacentPoints[k] != -1 && labels[frontAdjacentPoints[k]] == label && std::find(ordered_cells.begin(), ordered_cells.end(), frontAdjacentPoints[k]) == ordered_cells.end())
//                 {
//                     ordered_cells.push_front(frontAdjacentPoints[k]);
//                     break;
//                 }

//             // connect cells to last cell
//             std::vector<int> backAdjacentPoints = getAdjacentPoints(ordered_cells.back(), map_frontiers);
//             for (unsigned int k = 0; k < backAdjacentPoints.size(); k++)
//                 if (backAdjacentPoints[k] != -1 && labels[backAdjacentPoints[k]] == label && std::find(ordered_cells.begin(), ordered_cells.end(), backAdjacentPoints[k]) == ordered_cells.end())
//                 {
//                     ordered_cells.push_back(backAdjacentPoints[k]);
//                     break;
//                 }

//             if (initial_size == ordered_cells.size() && ordered_cells.size() < frontiers_msg.frontiers[i].size)
//                 break;
//         }
//         // center cell
//         frontiers_msg.frontiers[i].center_cell = ordered_cells[ordered_cells.size() / 2];
//         frontiers_msg.frontiers[i].center_point = cell2point(frontiers_msg.frontiers[i].center_cell, map_frontiers);
//     }

//     // Publish
//     frontiers_pub_->publish(frontiers_msg);
//     RCLCPP_DEBUG(this->get_logger(), "frontiers published!");
//     publishMarkers(frontiers_msg);
//     RCLCPP_DEBUG(this->get_logger(), "marker array published!");
// }

// // Check if a cell is frontier (free close to unknown)
// bool FindFrontiers::isFrontier(const int &cell, const nav_msgs::msg::OccupancyGrid &map) const
// {
//     if (map.data[cell] == 0) // check if it is free
//     {
//         auto straightPoints = getStraightPoints(cell, map);
//         for (unsigned int i = 0; i < straightPoints.size(); ++i)
//             if (straightPoints[i] != -1 && map.data[straightPoints[i]] == -1) // check if any neigbor is unknown
//                 return true;
//     }
//     // If it is obstacle or unknown, it can not be frontier
//     return false;
// }

// // Two pass labeling to label frontiers [http://en.wikipedia.org/wiki/Connected-component_labeling]
// std::vector<int> FindFrontiers::twoPassLabeling(const nav_msgs::msg::OccupancyGrid &map_labelr, std::map<int, int> &labels_sizes, bool diagonal) const
// {
//     labels_sizes.clear();
//     std::vector<int> labels(map_labelr.data.size());
//     labels.assign(map_labelr.data.begin(), map_labelr.data.end());

//     std::vector<int> neigh_labels;
//     std::vector<int> rank(1000);
//     std::vector<int> parent(1000);
//     boost::disjoint_sets<int *, int *> dj_set(&rank[0], &parent[0]);
//     int current_label_ = 1;

//     // 1ST PASS: Assign temporary labels to frontiers and establish relationships
//     for (unsigned int i = 0; i < map_labelr.data.size(); i++)
//     {
//         if (map_labelr.data[i] != 0)
//         {
//             neigh_labels.clear();
//             // Find )8 or 4) connectivity neighbours already labeled
//             if (diagonal && upleftCell(i, map_labelr) != -1 && labels[upleftCell(i, map_labelr)] != 0)
//                 neigh_labels.push_back(labels[upleftCell(i, map_labelr)]);
//             if (upCell(i, map_labelr) != -1 && labels[upCell(i, map_labelr)] != 0)
//                 neigh_labels.push_back(labels[upCell(i, map_labelr)]);
//             if (diagonal && uprightCell(i, map_labelr) != -1 && labels[uprightCell(i, map_labelr)] != 0)
//                 neigh_labels.push_back(labels[uprightCell(i, map_labelr)]);
//             if (leftCell(i, map_labelr) != -1 && labels[leftCell(i, map_labelr)] != 0)
//                 neigh_labels.push_back(labels[leftCell(i, map_labelr)]);

//             if (neigh_labels.empty()) // case: No neighbours
//             {
//                 dj_set.make_set(current_label_); //   create new set of labels
//                 labels[i] = current_label_;      //   update cell's label
//                 current_label_++;                //   update label
//             }
//             else // case: With neighbours
//             {
//                 labels[i] = *std::min_element(neigh_labels.begin(), neigh_labels.end()); //   choose minimum label of the neighbours
//                 for (unsigned int j = 0; j < neigh_labels.size(); ++j)                   //   update neighbours sets
//                     dj_set.union_set(labels[i], neigh_labels[j]);                        //   unite sets minimum label with the others
//             }
//         }
//     }

//     // 2ND PASS: Assign final label
//     dj_set.compress_sets(labels.begin(), labels.end());
//     // compress sets for efficiency
//     for (unsigned int i = 0; i < map_labelr.data.size(); i++)
//         if (labels[i] != 0)
//         {
//             // relabel each element with the lowest equivalent label
//             labels[i] = dj_set.find_set(labels[i]);
//             // increment the size of the label
//             if (labels_sizes.count(labels[i]) == 0)
//                 labels_sizes[labels[i]] = 1;
//             else
//                 labels_sizes[labels[i]]++;
//         }

//     return labels;
// }

// void FindFrontiers::publishMarkers(const robo_drill_interfaces::msg::Frontiers &frontiers_msg)
// {
//     // resize
//     markers_.markers.resize(frontiers_msg.frontiers.size() * 3 + 1); // each frontier: cells, center_point and text

//     // deleteall
//     markers_.markers[0].action = visualization_msgs::msg::Marker::DELETEALL;

//     // omplir
//     for (unsigned int i = 0; i < frontiers_msg.frontiers.size(); i++)
//     {
//         std_msgs::msg::ColorRGBA c;
//         double x = (double)i / (double)frontiers_msg.frontiers.size();
//         c.a = 1.0;
//         c.r = x < 0.33 ? 1 - 0.33 * x : (x < 0.66 ? 0 : 3 * (x - 0.66));
//         c.g = x < 0.33 ? 3 * x : (x < 0.66 ? 1 - 0.33 * (x - 0.33) : 0);
//         c.b = x < 0.33 ? 0 : (x < 0.66 ? 3 * (x - 0.33) : 1 - 0.33 * (x - 0.66));

//         // RCLCPP_INFO(this->get_logger(), "Colors: i = %u\nx = %f\nr = %f\ng = %f\nb = %f", i, x, c.r, c.g, c.b);

//         // point center
//         visualization_msgs::msg::Marker marker_center;
//         marker_center.header.frame_id = frontiers_msg.header.frame_id;
//         marker_center.header.stamp = this->now();
//         marker_center.lifetime = rclcpp::Duration(0, 0);
//         marker_center.type = visualization_msgs::msg::Marker::CYLINDER;
//         marker_center.scale.x = marker_center.scale.y = 0.1;
//         marker_center.scale.z = 0.5;
//         marker_center.color = c;
//         marker_center.ns = "centers";
//         marker_center.id = 3 * i;
//         marker_center.pose.position.x = frontiers_msg.frontiers[i].center_point.x;
//         marker_center.pose.position.y = frontiers_msg.frontiers[i].center_point.y;
//         marker_center.pose.position.z = marker_center.scale.z / 2;
//         marker_center.pose.orientation.x = marker_center.pose.orientation.y = marker_center.pose.orientation.z = 0;
//         marker_center.pose.orientation.w = 1;
//         markers_.markers[3 * i + 1] = marker_center;

//         // cells
//         visualization_msgs::msg::Marker marker_points;
//         marker_points.header.frame_id = frontiers_msg.header.frame_id;
//         marker_points.header.stamp = this->now();
//         marker_points.lifetime = rclcpp::Duration(0, 0);
//         marker_points.type = visualization_msgs::msg::Marker::POINTS;
//         marker_points.pose.orientation.x = marker_points.pose.orientation.y = marker_points.pose.orientation.z = 0;
//         marker_points.pose.orientation.w = 1;
//         marker_points.points = frontiers_msg.frontiers[i].cells_points;
//         marker_points.scale.x = marker_points.scale.y = marker_points.scale.z = cell_size_;
//         marker_points.colors = std::vector<std_msgs::msg::ColorRGBA>(marker_points.points.size(), c);
//         marker_points.ns = "cells";
//         marker_points.id = 3 * i + 1;
//         markers_.markers[3 * i + 2] = marker_points;

//         // text
//         visualization_msgs::msg::Marker marker_id;
//         marker_id.header.frame_id = frontiers_msg.header.frame_id;
//         marker_id.header.stamp = this->now();
//         marker_id.lifetime = rclcpp::Duration(0, 0);
//         marker_id.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
//         marker_id.scale.x = marker_id.scale.y = 1;
//         marker_id.scale.z = 0.5;
//         marker_id.text = std::to_string(i);
//         marker_id.color = c;
//         marker_id.pose = marker_center.pose;
//         marker_id.pose.position.z = marker_center.scale.z + 0.2;
//         marker_id.ns = "id";
//         marker_id.id = 3 * i + 2;
//         markers_.markers[3 * i + 3] = marker_id;
//     }
//     // publish
//     markers_pub_->publish(markers_);
// }

// // UTILS ////////////////////////////////////////////////////////////////////////////////////////////////////
// int FindFrontiers::point2cell(const geometry_msgs::msg::Point &point, const nav_msgs::msg::OccupancyGrid &map_) const
// {
//     if (point.x <= map_.info.origin.position.x || point.x >= map_.info.width * map_.info.resolution + map_.info.origin.position.x ||
//         point.y <= map_.info.origin.position.y || point.y >= map_.info.height * map_.info.resolution + map_.info.origin.position.y)
//     {
//         return -1;
//     }

//     int x_cell = floor0((point.x - map_.info.origin.position.x) / map_.info.resolution);
//     int y_cell = floor0((point.y - map_.info.origin.position.y) / map_.info.resolution);
//     int cell = x_cell + (y_cell)*map_.info.width;
//     return cell;
// }

// geometry_msgs::msg::Point FindFrontiers::cell2point(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
// {
//     geometry_msgs::msg::Point point;
//     point.x = (cell % map_.info.width) * map_.info.resolution + map_.info.origin.position.x + map_.info.resolution / 2;
//     point.y = floor(cell / map_.info.width) * map_.info.resolution + map_.info.origin.position.y + map_.info.resolution / 2;
//     return point;
// }

// std::vector<int> FindFrontiers::getStraightPoints(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
// {
//     return std::vector<int>({leftCell(cell, map_),
//                              upCell(cell, map_),
//                              rightCell(cell, map_),
//                              downCell(cell, map_)});
// }
// std::vector<int> FindFrontiers::getAdjacentPoints(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
// {
//     return std::vector<int>({leftCell(cell, map_),
//                              upCell(cell, map_),
//                              rightCell(cell, map_),
//                              downCell(cell, map_),
//                              upleftCell(cell, map_),
//                              uprightCell(cell, map_),
//                              downrightCell(cell, map_),
//                              downleftCell(cell, map_)});
// }
// int FindFrontiers::rightCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
// {
//     // only go left if no index error and if current cell is not already on the left boundary
//     if ((cell % map_.info.width != 0))
//         return cell + 1;

//     return -1;
// }
// int FindFrontiers::uprightCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
// {
//     if ((cell % map_.info.width != 0) && (cell >= (int)map_.info.width))
//         return cell - map_.info.width + 1;

//     return -1;
// }
// int FindFrontiers::upCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
// {
//     if (cell >= (int)map_.info.width)
//         return cell - map_.info.width;

//     return -1;
// }
// int FindFrontiers::upleftCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
// {
//     if ((cell >= (int)map_.info.width) && ((cell + 1) % (int)map_.info.width != 0))
//         return cell - map_.info.width - 1;

//     return -1;
// }
// int FindFrontiers::leftCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
// {
//     if ((cell + 1) % map_.info.width != 0)
//         return cell - 1;

//     return -1;
// }
// int FindFrontiers::downleftCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
// {
//     if (((cell + 1) % map_.info.width != 0) && ((cell / map_.info.width) < (map_.info.height - 1)))
//         return cell + map_.info.width - 1;

//     return -1;
// }
// int FindFrontiers::downCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
// {
//     if ((cell / map_.info.width) < (map_.info.height - 1))
//         return cell + map_.info.width;

//     return -1;
// }
// int FindFrontiers::downrightCell(const int &cell, const nav_msgs::msg::OccupancyGrid &map_) const
// {
//     if (((cell / map_.info.width) < (map_.info.height - 1)) && (cell % map_.info.width != 0))
//         return cell + map_.info.width + 1;

//     return -1;
// }
// int FindFrontiers::floor0(const float &value) const
// {
//     if (value < 0.0)
//         return ceil(value);
//     else
//         return floor(value);
// }

// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<FindFrontiers>());
//     rclcpp::shutdown();
//     return 0;
// }
