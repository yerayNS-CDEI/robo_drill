#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream>
#include <stdlib.h>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "robo_drill/msg/frontiers.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/back_up.hpp"

#include "tf2/utils.h"                             // getYaw
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" // toMsg
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

class ExplorationBase : public rclcpp::Node
{
public:
    using NavToPose = nav2_msgs::action::NavigateToPose;
    using ComputePath = nav2_msgs::action::ComputePathToPose;
    using BackUp = nav2_msgs::action::BackUp;
    using GoalHandleNavToPose = rclcpp_action::ClientGoalHandle<NavToPose>;
    using GoalHandleComputePath = rclcpp_action::ClientGoalHandle<ComputePath>;
    using GoalHandleBackUp = rclcpp_action::ClientGoalHandle<BackUp>;

protected:
    // ------------------ ATTRIBUTES ------------------
    std::string algorithm_variant_; // variant of the algorithm (in case you need it)
    std::string results_file_;      // name of the results file (csv)
    std::string world_;             // name of the world

    int robot_status_;                    // 0: moving, 1: goal reached, 2: failed to reach goal
    geometry_msgs::msg::Pose robot_pose_; // current robot pose
    geometry_msgs::msg::Pose goal_;       // last goal sent
    rclcpp::Time goal_start_;             // used to compute goal_time
    double goal_time_;                    // seconds since last goal was sent
    double goal_distance_;                // remaining distance to reach the last goal
    rclcpp_action::GoalUUID goal_id_;     // last goal id

    // Subscribers / publishers / actions / timers
    rclcpp::CallbackGroup::SharedPtr callback_group_1_, callback_group_2_;
    rclcpp::Subscription<robo_drill::msg::Frontiers>::SharedPtr frontiers_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_invalid_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_marker_pub_;
    visualization_msgs::msg::Marker g_invalid_;
    visualization_msgs::msg::Marker goal_marker_;
    nav_msgs::msg::OccupancyGrid map_;
    robo_drill::msg::Frontiers frontiers_msg_;
    rclcpp_action::Client<NavToPose>::SharedPtr nav_to_pose_client_;
    rclcpp_action::Client<ComputePath>::SharedPtr compute_path_client_;
    rclcpp_action::Client<BackUp>::SharedPtr backup_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    // tf
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    // GLOBAL STATS
    int cells_explored_;             // explored cells (free or obstacle)
    int num_goals_sent_;             // total goals sent
    int num_goals_ko_;               // total goals cancelled
    double distance_traveled_;       // total absolute distance traveled during exploration (m)
    double angle_traveled_;          // total absolute angle turned during exploration (rad)
    rclcpp::Time begin_exploration_; // used to compute the total exploration time

    // Flags
    bool exploration_ended_;   // flag to finish exploration
    bool exploration_started_; // flag to start exploration
    bool got_map_;             // flag to ack the map reception
    bool return_to_origin_ = false;
    bool goal_to_origin_sent_ = false;

    // ------------------ METHODS ------------------
public:
    ExplorationBase(const std::string &_name);
    ~ExplorationBase();
    void loop();

protected:
    // CALLBACKS (inputs)
    void mapCallback(const nav_msgs::msg::OccupancyGrid &msg);
    void frontiersCallback(const robo_drill::msg::Frontiers &msg);

    // EXPLORATION ALGORITHM MAIN FUNCTIONS
    // Functions to be implemented in derived class
    virtual bool replan() = 0;
    virtual geometry_msgs::msg::Pose decideGoal() = 0;
    // Base class functions
    virtual geometry_msgs::msg::Pose decideGoalBase();
    void finish();

    // AUXILIARY
    geometry_msgs::msg::Pose generateRandomPose(float radius, geometry_msgs::msg::Pose reference_pose);
    bool isValidGoal(const geometry_msgs::msg::Pose &goal, double &path_distance);
    bool isInMap(const geometry_msgs::msg::Point &position) const;
    bool isFree(const geometry_msgs::msg::Point &position) const;
    bool hasLineOfSight(const geometry_msgs::msg::Point &from, const geometry_msgs::msg::Point &to) const;
    geometry_msgs::msg::Point getMapCenter() const;
    int floor0(const float &value) const;

    // NAVIGATION
    // send goal
    bool sendGoal(const geometry_msgs::msg::Pose &goal_pose);
    void goalResponseCallback(const GoalHandleNavToPose::SharedPtr &goal_handle);
    void goalFeedbackCallback(const GoalHandleNavToPose::SharedPtr &goal_handle,
                              const std::shared_ptr<const NavToPose::Feedback> feedback);
    void goalResultCallback(const GoalHandleNavToPose::WrappedResult &result);
    bool executeBackup(double distance);
    // compute path
    double computePathLength(const geometry_msgs::msg::Pose &goal_pose);

    // TF
    bool updateRobotPose();
};

ExplorationBase::ExplorationBase(const std::string &_name)
    : Node(_name, rclcpp::NodeOptions()
                      .allow_undeclared_parameters(true)
                      .automatically_declare_parameters_from_overrides(true)),
      robot_status_(1),
      cells_explored_(0),
      num_goals_sent_(0),
      num_goals_ko_(0),
      distance_traveled_(0),
      angle_traveled_(0),
      begin_exploration_(this->now()),
      exploration_ended_(false),
      exploration_started_(false),
      got_map_(false)
{
    // Callback groups
    callback_group_1_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_2_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // subscribers
    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_1_;
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map_working", 1, std::bind(&ExplorationBase::mapCallback, this, std::placeholders::_1), options);
    frontiers_sub_ = this->create_subscription<robo_drill::msg::Frontiers>(
        "/frontiers", 1, std::bind(&ExplorationBase::frontiersCallback, this, std::placeholders::_1), options);

    // publishers
    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_sent", 1);
    goal_invalid_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("goal_invalid", 1);
    goal_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("goal_marker", 1);
    
    g_invalid_.header.frame_id = "map";
    g_invalid_.header.stamp = this->now();
    g_invalid_.type = visualization_msgs::msg::Marker::ARROW;
    g_invalid_.action = visualization_msgs::msg::Marker::ADD;
    g_invalid_.id = 1;
    g_invalid_.scale.x = 1;
    g_invalid_.scale.y = 0.05;
    g_invalid_.scale.z = 0.05;
    g_invalid_.color.a = 1.0;
    g_invalid_.color.r = 1.0;
    g_invalid_.color.g = 0.0;
    g_invalid_.color.b = 0.0;
    g_invalid_.lifetime = rclcpp::Duration(3, 0); // visualized for 3s
    
    // Goal marker (green arrow showing current exploration goal)
    goal_marker_.header.frame_id = "map";
    goal_marker_.type = visualization_msgs::msg::Marker::ARROW;
    goal_marker_.action = visualization_msgs::msg::Marker::ADD;
    goal_marker_.id = 0;
    goal_marker_.scale.x = 1.5;   // Arrow length
    goal_marker_.scale.y = 0.15;  // Arrow width
    goal_marker_.scale.z = 0.15;  // Arrow height
    goal_marker_.color.a = 1.0;   // Fully opaque
    goal_marker_.color.r = 0.0;
    goal_marker_.color.g = 1.0;   // Green
    goal_marker_.color.b = 0.0;

    // Get parameters from ROS
    int node_period;
    get_parameter_or("results_file", results_file_, std::string("~/exploration_results.csv"));
    get_parameter_or("world", world_, std::string("undefined"));
    get_parameter_or("algorithm_variant", algorithm_variant_, std::string());
    get_parameter_or("node_period_ms", node_period, 500);

    // substitute '~' by home path
    auto found = results_file_.find('~');
    if (found != std::string::npos)
    {
        if (getenv("HOME") == NULL)
        {
            RCLCPP_ERROR(this->get_logger(), "ExplorationBase: Parameter 'results_file_' has '~' but 'HOME' environment variable is not defined");
            return;
        }
        auto home_dir = std::string(getenv("HOME"));
        results_file_.replace(found, 1, home_dir + "/");
    }

    // check that file can be opened
    std::ofstream outfile(results_file_, std::ofstream::out | std::ofstream::app); // append instead of overwrite
    if (not outfile.is_open())
    {
        RCLCPP_ERROR(this->get_logger(), "ExplorationBase: couldn't open/write the results file %s\n", results_file_.c_str());
        return;
    }

    // action of nav2
    nav_to_pose_client_ = rclcpp_action::create_client<NavToPose>(this, "/navigate_to_pose", callback_group_1_);
    compute_path_client_ = rclcpp_action::create_client<ComputePath>(this, "/compute_path_to_pose", callback_group_2_);
    backup_client_ = rclcpp_action::create_client<BackUp>(this, "/backup", callback_group_2_);

    // tf listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // timer to periodically call loop()
    timer_ = this->create_wall_timer(std::chrono::milliseconds(node_period), std::bind(&ExplorationBase::loop, this), callback_group_1_);

    // seed for random
    srand((unsigned)time(NULL));
}

ExplorationBase::~ExplorationBase()
{
    // print file if ended by user or error
    if (not exploration_ended_)
        finish();
}

void ExplorationBase::mapCallback(const nav_msgs::msg::OccupancyGrid &msg)
{
    RCLCPP_DEBUG(this->get_logger(), "ExplorationBase::mapCallback");

    // store map
    map_ = msg;

    // compute explored cells
    cells_explored_ = 0;
    for (unsigned int i = 0; i < map_.data.size(); ++i)
        if (map_.data[i] != -1)
            cells_explored_++;

    // flag
    got_map_ = true;
}

void ExplorationBase::frontiersCallback(const robo_drill::msg::Frontiers &msg)
{
    RCLCPP_DEBUG(this->get_logger(), "ExplorationBase::frontiersCallback");

    frontiers_msg_ = msg;

    // Until first forntiers message received, exploration doesn't start
    exploration_started_ = true;

    // If there is not frontiers, exploration ended!
    if (frontiers_msg_.frontiers.empty())
        exploration_ended_ = true;
}

geometry_msgs::msg::Pose ExplorationBase::decideGoalBase()
{
    RCLCPP_DEBUG(this->get_logger(), "ExplorationBase::decideGoalBase");

    // secure call
    geometry_msgs::msg::Pose g = robot_pose_;
    if (not exploration_ended_)
        g = decideGoal();

    // If the (for any reason) the decided goal is not a valid goal, generate a random goal in the surroundings
    // Iteratively increase the radius until we obtain a valid goal
    double radius = 0.1;
    double path_length;
    bool first_time = true;
    while (not isValidGoal(g, path_length))
    {
        if (first_time)
        {
            RCLCPP_WARN(this->get_logger(),
                        "!!! Goal provided by decideGoal() is not valid [%f, %f, %f].\nGenerating random goal in the surroundings...",
                        g.position.x,
                        g.position.y,
                        tf2::getYaw(g.orientation));

            // publish the invalid goal to visualize
            g_invalid_.header.stamp = this->now();
            g_invalid_.id++;
            g_invalid_.pose = g;
            goal_invalid_pub_->publish(g_invalid_);

            first_time = false;
        }

        // generate a new random pose
        g = generateRandomPose(radius, g);

        // Increase radius 5cm (limit the radius size to map size)
        radius += 0.05;

        // exit if ctrl+c or no more frontiers
        if (not rclcpp::ok() or exploration_ended_)
            break;
    }
    return g;
}

void ExplorationBase::loop()
{
    RCLCPP_DEBUG(this->get_logger(), "ExplorationBase::loop(): goal_time: %f, goal_distance=%f", goal_time_, goal_distance_);

    // NOT STARTED
    if (not exploration_started_ || not got_map_)
        return;

    if (updateRobotPose())
    {
        if (replan())
        {
            sendGoal(decideGoalBase());
        }
    }
    else
        RCLCPP_WARN(this->get_logger(), "ExplorationBase::loop(): Couldn't get robot position!");

    // END
    if (exploration_ended_ && !goal_to_origin_sent_)
    {
        finish();
        goal_to_origin_sent_ = true;
    }
}

void ExplorationBase::finish()
{
    double t = (this->now() - begin_exploration_).seconds();
    int t_min = int(t) / 60;
    int t_sec = int(t) % 60;

    // print stats
    printf("//////////////////////////////////////////////////////\n");
    if (exploration_ended_)
        printf("//////////////// EXPLORATION FINISHED ////////////////\n");
    else
        printf("// EXPLORATION ENDED BEFORE EXPLORING THE WHOLE MAP //\n");
    printf("//////////////////////////////////////////////////////\n");
    printf("\tAlgorithm name: %s\n", this->get_name());
    printf("\tAlgorithm variant: %s\n", algorithm_variant_.c_str());
    printf("\tWorld: %s\n", world_.c_str());
    printf("\tSent %i goals (aborted %i)\n", num_goals_sent_, num_goals_ko_);
    printf("\tDistance traveled %.2f m\n", distance_traveled_);
    printf("\tAngle turned %.2f m\n", angle_traveled_);
    printf("\tDuration %2.2i:%2.2i min\n", t_min, t_sec);
    printf("\tExplored %.2f m^2 (%d cells)\n\n",
           cells_explored_ * map_.info.resolution * map_.info.resolution,
           cells_explored_);

    // store stats in CSV
    printf("Storing results in file: %s\n", results_file_.c_str());
    std::ofstream outfile(results_file_,
                          std::ofstream::out | std::ofstream::app); // append instead of overwrite
    if (outfile.is_open())
    {
        // initialize file with header
        if (outfile.tellp() == 0)
        {
            outfile << "#DATE_TIME, "
                    << "FINISHED, "
                    << "ALGORITHM NAME, "
                    << "VARIANT, "
                    << "WORLD, "
                    << "GOALS SENT, "
                    << "GOALS ABORTED, "
                    << "DISTANCE TRAVELED (m), "
                    << "ANGLE TURNED (rad), "
                    << "DURATION (s), "
                    << "CELLS EXPLORED\n";
        }

        // current date and time on the current system
        time_t now = time(0);
        char *date_time = ctime(&now); // convert now to string form
        if (date_time[strlen(date_time) - 1] == '\n')
            date_time[strlen(date_time) - 1] = '\0'; // remove line break at the end

        outfile << date_time << ", "          //
                << exploration_ended_ << ", " //
                << this->get_name() << ", "   //
                << algorithm_variant_ << ", " //
                << world_ << ", "             //
                << num_goals_sent_ << ", "    //
                << num_goals_ko_ << ", "      //
                << distance_traveled_ << ", " //
                << angle_traveled_ << ", "    //
                << t << ", "                  //
                << cells_explored_ << "\n";   //

        outfile.close();
    }
    else
    {
        printf("ExplorationBase::finish(): couldn't open/write the results file %s\n", results_file_.c_str());
    }

    // ---- Goal back to origin ----
    geometry_msgs::msg::Pose origin_pose;
    origin_pose.position.x = 0.0;
    origin_pose.position.y = 0.0;
    origin_pose.position.z = 0.0;
    origin_pose.orientation.w = 1.0;  // yaw = 0

    RCLCPP_INFO(this->get_logger(), "Exploration finished -> navigating back to origin (0,0)");
    return_to_origin_ = true;
    sendGoal(origin_pose);

    // rclcpp::shutdown();
}

///// AUXILIARY FUNCTIONS //////////////////////////////////////////////////////////////////////
geometry_msgs::msg::Pose ExplorationBase::generateRandomPose(float radius, geometry_msgs::msg::Pose reference_pose)
{
    RCLCPP_DEBUG(this->get_logger(), "ExplorationBase::generateRandomPose");

    if (radius <= 0)
    {
        RCLCPP_WARN(this->get_logger(), "ExplorationBase::generateRandomPose(): radius must be > 0. Changed to 1");
        radius = 1;
    }

    // Check if reference_pose inside map
    if (not isInMap(reference_pose.position))
    {
        RCLCPP_WARN(this->get_logger(),
                    "ExplorationBase::generateRandomPose(): reference position is not inside the map. Changed to map center with big radius.");
        reference_pose.position = getMapCenter();
        radius = sqrt(pow(map_.info.height * map_.info.resolution / 2, 2) + pow(map_.info.width * map_.info.resolution / 2, 2));
    }

    // If radius is higher than map size, reference is center of the map
    if (radius >= sqrt(pow(map_.info.height * map_.info.resolution / 2, 2) + pow(map_.info.width * map_.info.resolution / 2, 2)))
    {
        reference_pose.position = getMapCenter();
        radius = sqrt(pow(map_.info.height * map_.info.resolution / 2, 2) + pow(map_.info.width * map_.info.resolution / 2, 2));
    }

    // Random radius and angle
    float rand_yaw = 2 * M_PI * (rand() % 100) / 100.0;
    float rand_r = radius * (rand() % 100) / 100.0;

    // Compose over reference pose
    geometry_msgs::msg::Pose goal_pose;
    goal_pose.position.x = reference_pose.position.x + rand_r * cos(rand_yaw);
    goal_pose.position.y = reference_pose.position.y + rand_r * sin(rand_yaw);
    tf2::Quaternion q;
    q.setRPY(0, 0, tf2::getYaw(reference_pose.orientation) + rand_yaw);
    goal_pose.orientation = tf2::toMsg(q);

    return goal_pose;
}

bool ExplorationBase::isValidGoal(const geometry_msgs::msg::Pose &goal, double &path_length)
{
    // not free (also returning false if out of map)
    if (not isFree(goal.position))
    {
        RCLCPP_DEBUG(this->get_logger(), "ExplorationBase::isValidGoal(): Goal is not free or not inside the map");
        return false;
    }
    
    // Line-of-sight check: only reject if straight line crosses OBSTACLES (not unknown)
    // For exploration, unknown space is OK - we want to explore it!
    // Only block if there's a clear obstacle wall in the way
    if (!hasLineOfSight(robot_pose_.position, goal.position))
    {
        RCLCPP_DEBUG(this->get_logger(), "ExplorationBase::isValidGoal(): Line blocked by obstacle");
        // Don't return false here - let the planner handle it
        // The planner with allow_unknown=false will properly navigate around obstacles
    }
    
    // Check clearance around goal (increased to protect expensive lidar)
    const double clearance_radius = 0.7; // meters (increased for better obstacle clearance)
    const int num_angles = 8;
    int failed_checks = 0;
    for (int i = 0; i < num_angles; i++) {
        double angle = 2.0 * M_PI * i / num_angles;
        geometry_msgs::msg::Point check_pt;
        check_pt.x = goal.position.x + clearance_radius * cos(angle);
        check_pt.y = goal.position.y + clearance_radius * sin(angle);
        check_pt.z = 0;
        if (!isFree(check_pt)) {
            failed_checks++;
        }
    }
    // Allow goal if at least 5 out of 8 directions have clearance
    // This allows goals near walls (typical for frontiers)
    if (failed_checks > 3) {
        RCLCPP_DEBUG(this->get_logger(), "ExplorationBase::isValidGoal(): Insufficient clearance (%d/8 failed)", failed_checks);
        return false;
    }
    
    // return true if a path to that pose could be computed
    path_length = computePathLength(goal);

    if (path_length < 0)
    {
        RCLCPP_DEBUG(this->get_logger(), "ExplorationBase::isValidGoal(): computePathLength() provided negative path lenght");
    }
    return path_length >= 0;
}

bool ExplorationBase::isInMap(const geometry_msgs::msg::Point &position) const
{
    auto dx = position.x - map_.info.origin.position.x;
    auto dy = position.y - map_.info.origin.position.y;
    auto map_th = tf2::getYaw(map_.info.origin.orientation);

    auto dx_map = cos(-map_th) * dx - sin(-map_th) * dy;
    auto dy_map = sin(-map_th) * dx + cos(-map_th) * dy;

    return dx_map >= 0 and dy_map >= 0 and dx_map <= map_.info.width * map_.info.resolution and
           dy_map <= map_.info.height * map_.info.resolution;
}

bool ExplorationBase::isFree(const geometry_msgs::msg::Point &point) const
{
    if (not isInMap(point))
        return false;

    // cell
    int x_cell = floor0((point.x - map_.info.origin.position.x) / map_.info.resolution);
    int y_cell = floor0((point.y - map_.info.origin.position.y) / map_.info.resolution);
    int cell = x_cell + y_cell * map_.info.width;

    // return if free
    return map_.data[cell] == 0;
}

bool ExplorationBase::hasLineOfSight(const geometry_msgs::msg::Point &from, const geometry_msgs::msg::Point &to) const
{
    // Bresenham-like line traversal to check if path is clear of OBSTACLES
    // For exploration: Unknown cells (-1) are OK, only block on obstacles (100)
    // Returns false only if obstacle is encountered
    
    if (!isInMap(from) || !isInMap(to))
        return false;
    
    int x0 = floor0((from.x - map_.info.origin.position.x) / map_.info.resolution);
    int y0 = floor0((from.y - map_.info.origin.position.y) / map_.info.resolution);
    int x1 = floor0((to.x - map_.info.origin.position.x) / map_.info.resolution);
    int y1 = floor0((to.y - map_.info.origin.position.y) / map_.info.resolution);
    
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    
    int x = x0;
    int y = y0;
    
    while (true) {
        // Check current cell
        if (x >= 0 && x < (int)map_.info.width && y >= 0 && y < (int)map_.info.height) {
            int cell = x + y * map_.info.width;
            // Block ONLY if obstacle (100), allow unknown (-1) for exploration
            if (map_.data[cell] == 100) {
                return false;
            }
        } else {
            return false; // Out of bounds
        }
        
        if (x == x1 && y == y1)
            break;
            
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x += sx;
        }
        if (e2 < dx) {
            err += dx;
            y += sy;
        }
    }
    
    return true;
}

int ExplorationBase::floor0(const float &value) const
{
    if (value < 0.0)
        return ceil(value);
    else
        return floor(value);
}

geometry_msgs::msg::Point ExplorationBase::getMapCenter() const
{
    geometry_msgs::msg::Point center;

    auto map_th = tf2::getYaw(map_.info.origin.orientation);
    center.x = map_.info.origin.position.x + 0.5 * cos(map_th) * map_.info.width * map_.info.resolution -
               0.5 * sin(map_th) * map_.info.height * map_.info.resolution;
    center.y = map_.info.origin.position.y + 0.5 * sin(map_th) * map_.info.width * map_.info.resolution +
               0.5 * cos(map_th) * map_.info.height * map_.info.resolution;

    return center;
}

// NAVIGATION FUNCTIONS ////////////////////////////////////////////////////////////////////////////////////////
bool ExplorationBase::sendGoal(const geometry_msgs::msg::Pose &goal_pose)
{
    try
    {
        using namespace std::placeholders;

        if (not nav_to_pose_client_->wait_for_action_server(std::chrono::milliseconds(50)))
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return false;
        }

        if (not nav_to_pose_client_->action_server_is_ready())
        {
            RCLCPP_ERROR(this->get_logger(), "Action server nav_to_pose_client_ not ready");
            return false;
        }

        // Fill goal msg
        nav2_msgs::action::NavigateToPose::Goal goal;
        goal.pose.header.frame_id = "map";
        goal.pose.header.stamp = this->now();
        goal.pose.pose = goal_pose;

        // reset goal stats
        goal_ = goal_pose;
        goal_distance_ = 1e9; // initialize far away
        goal_time_ = 0;
        goal_start_ = this->now();
        num_goals_sent_++;

        // Send goal
        auto send_goal_options = rclcpp_action::Client<NavToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&ExplorationBase::goalResponseCallback, this, _1);
        send_goal_options.feedback_callback =
            std::bind(&ExplorationBase::goalFeedbackCallback, this, _1, _2);
        send_goal_options.result_callback =
            std::bind(&ExplorationBase::goalResultCallback, this, _1);
        nav_to_pose_client_->async_send_goal(goal, send_goal_options);

        // publish goal sent
        geometry_msgs::msg::PoseStamped goal_ps;
        goal_ps.header.frame_id = "map";
        goal_ps.header.stamp = this->now();
        goal_ps.pose = goal_pose;
        goal_pub_->publish(goal_ps);
        
        // Publish goal marker visualization (green arrow)
        goal_marker_.header.stamp = this->now();
        goal_marker_.pose = goal_pose;
        goal_marker_pub_->publish(goal_marker_);

        // print
        RCLCPP_INFO(this->get_logger(), "Sending Goal #%d: x=%4.2f, y=%4.2f, yaw=%4.2f",
                    num_goals_sent_,
                    goal.pose.pose.position.x,
                    goal.pose.pose.position.y,
                    tf2::getYaw(goal.pose.pose.orientation));

        rclcpp::Duration t = this->now() - begin_exploration_;
        int elapsed_time_minutes = int(t.seconds()) / 60;
        int elapsed_time_seconds = int(t.seconds()) % 60;

        printf(
            ">>> Exploration status:\n      Sent %d goals (aborted %d). Distance traveled %.2f m. Angle turned %.1f rad. "
            "Duration: %2.2i:%2.2i min. Explored %.2f m^2 (%d cells)\n",
            num_goals_sent_,
            num_goals_ko_,
            distance_traveled_,
            angle_traveled_,
            elapsed_time_minutes,
            elapsed_time_seconds,
            cells_explored_ * map_.info.resolution * map_.info.resolution,
            cells_explored_);

        return true;
    }
    catch (const std::exception &e)
    {
        RCLCPP_WARN(this->get_logger(), "Could not call action server nav_to_pose: %s", e.what());
        return false;
    }
    return false;
}

void ExplorationBase::goalResponseCallback(const GoalHandleNavToPose::SharedPtr &goal_handle)
{
    try
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by nav_to_pose action server");
            // robot_status_ = 2; // cancelled
            num_goals_ko_++;
        }
        else
        {
            RCLCPP_DEBUG(this->get_logger(), "Goal accepted by nav_to_pose action server, robot is moving! %s", rclcpp_action::to_string(goal_handle->get_goal_id()).c_str());
            goal_id_ = goal_handle->get_goal_id();
            robot_status_ = 0; // moving
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "ExplorationBase::goalResponseCallback: %s", e.what());
    }
}

void ExplorationBase::goalFeedbackCallback(const GoalHandleNavToPose::SharedPtr &goal_handle,
                                           const std::shared_ptr<const NavToPose::Feedback> feedback)
{
    try
    {
        if (goal_handle->get_goal_id() != goal_id_)
            return;

        // update goal_distance_ if proper number (first feedback messages contains wrong values)
        double euclidean_distance = sqrt(pow(goal_.position.x - robot_pose_.position.x, 2) + pow(goal_.position.y - robot_pose_.position.y, 2));
        if (feedback->distance_remaining >= euclidean_distance)
            goal_distance_ = feedback->distance_remaining;

        // navigation_time is also wrong in the first feedback messages, compute manually instead
        // goal_time_ = rclcpp::Duration(feedback->navigation_time).seconds();
        goal_time_ = (this->now() - goal_start_).seconds();
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "ExplorationBase::goalFeedbackCallback: %s", e.what());
    }
}

void ExplorationBase::goalResultCallback(const GoalHandleNavToPose::WrappedResult &result)
{
    try
    {
        if (result.goal_id != goal_id_)
            return;

        RCLCPP_DEBUG(this->get_logger(), "Received result from nav_to_pose action");

        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_DEBUG(this->get_logger(), "Result from nav_to_pose: Goal was reached: %s", rclcpp_action::to_string(result.goal_id).c_str());
            robot_status_ = 1; // reached
            return;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_WARN(this->get_logger(), "Result from nav_to_pose: Goal was aborted: %s", rclcpp_action::to_string(result.goal_id).c_str());
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "Result from nav_to_pose: Goal was canceled: %s", rclcpp_action::to_string(result.goal_id).c_str());
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Result from nav_to_pose: Unknown result code: %s", rclcpp_action::to_string(result.goal_id).c_str());
            break;
        }
        robot_status_ = 2; // aborted/cancelled
        num_goals_ko_++;

        if (return_to_origin_)
        {
            RCLCPP_INFO(this->get_logger(), "Return to origin finished -> shutting down node.");
            rclcpp::shutdown();
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "ExplorationBase::goalResultCallback: %s", e.what());
    }
}

double ExplorationBase::computePathLength(const geometry_msgs::msg::Pose &goal_pose)
{
    try
    {
        if (not compute_path_client_->wait_for_action_server(std::chrono::milliseconds(50)))
        {
            RCLCPP_ERROR(this->get_logger(), "Action server compute_path_to_pose not available after waiting 50ms");
            return -1;
        }

        if (not compute_path_client_->action_server_is_ready())
        {
            RCLCPP_ERROR(this->get_logger(), "Action server compute_path_client_ not ready");
            return -1;
        }

        // Populate a goal
        RCLCPP_DEBUG(this->get_logger(), "compute_path_to_pose: x: %f, y: %f", goal_pose.position.x, goal_pose.position.y);
        auto goal_msg = ComputePath::Goal();
        goal_msg.goal.header.frame_id = "map";
        goal_msg.goal.pose = goal_pose;
        goal_msg.use_start = false; // use current robot pose as path start
        goal_msg.planner_id = "GridBased";

        // Wait 50ms for the server to accept the goal
        auto goal_handle_future = compute_path_client_->async_send_goal(goal_msg);
        RCLCPP_DEBUG(this->get_logger(), "compute_path_to_pose: Waiting for goal_handle");
        if (goal_handle_future.wait_for(std::chrono::milliseconds(1000)) == std::future_status::timeout)
        {
            RCLCPP_ERROR(this->get_logger(), "compute_path_to_pose: send goal call failed :(");
            return -1;
        }

        GoalHandleComputePath::SharedPtr goal_handle = goal_handle_future.get();
        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "compute_path_to_pose: Goal was rejected by the server");
            return -1;
        }

        // Wait 50ms more for the server to be done with the goal
        auto result_future = compute_path_client_->async_get_result(goal_handle);
        RCLCPP_DEBUG(this->get_logger(), "compute_path_to_pose: Waiting for result");
        if (result_future.wait_for(std::chrono::milliseconds(50)) == std::future_status::timeout)
        {
            RCLCPP_ERROR(this->get_logger(), "compute_path_to_pose: get result call failed :(");
            return -1;
        }

        // get the result
        GoalHandleComputePath::WrappedResult wrapped_result = result_future.get();

        switch (wrapped_result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "compute_path_to_pose: Goal was aborted");
            return -1;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "compute_path_to_pose: Goal was canceled");
            return -1;
        default:
            RCLCPP_ERROR(this->get_logger(), "compute_path_to_pose: Unknown result code");
            return -1;
        }

        // compute path length
        double length = 0.0;
        if (wrapped_result.result->path.poses.size() >= 1)
        {
            for (unsigned int i = 1; i < wrapped_result.result->path.poses.size(); i++)
            {
                double &x1 = wrapped_result.result->path.poses[i - 1].pose.position.x;
                double &y1 = wrapped_result.result->path.poses[i - 1].pose.position.y;
                double &x2 = wrapped_result.result->path.poses[i].pose.position.x;
                double &y2 = wrapped_result.result->path.poses[i].pose.position.y;
                double d = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
                length += d;
            }
        }

        RCLCPP_DEBUG(this->get_logger(), "computed path length: %f", length);

        return length;
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Could not call action server compute_path_to_pose: %s", e.what());
        return -1;
    }
    return -1;
}

bool ExplorationBase::executeBackup(double distance)
{
    try
    {
        if (!backup_client_->wait_for_action_server(std::chrono::seconds(2)))
        {
            RCLCPP_WARN(this->get_logger(), "Backup action server not available");
            return false;
        }

        // Create backup goal
        auto backup_goal = BackUp::Goal();
        backup_goal.target.x = -distance; // Negative for backward
        backup_goal.target.y = 0.0;
        backup_goal.target.z = 0.0;
        backup_goal.speed = 0.1; // Slow backup speed (m/s)

        RCLCPP_INFO(this->get_logger(), "Executing backup: %.2fm backward", distance);

        // Send goal and wait for result
        auto goal_handle_future = backup_client_->async_send_goal(backup_goal);
        if (goal_handle_future.wait_for(std::chrono::seconds(2)) == std::future_status::timeout)
        {
            RCLCPP_WARN(this->get_logger(), "Backup goal send timeout");
            return false;
        }

        auto goal_handle = goal_handle_future.get();
        if (!goal_handle)
        {
            RCLCPP_WARN(this->get_logger(), "Backup goal rejected");
            return false;
        }

        // Wait for backup to complete (with timeout)
        auto result_future = backup_client_->async_get_result(goal_handle);
        auto timeout = std::chrono::seconds(static_cast<int>(distance / 0.1 + 5)); // time = distance/speed + margin
        
        if (result_future.wait_for(timeout) == std::future_status::timeout)
        {
            RCLCPP_WARN(this->get_logger(), "Backup execution timeout");
            return false;
        }

        auto result = result_future.get();
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(this->get_logger(), "Backup completed successfully");
            return true;
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Backup failed with code: %d", static_cast<int>(result.code));
            return false;
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Backup execution exception: %s", e.what());
        return false;
    }
}

// Compute the current robot pose via TF
bool ExplorationBase::updateRobotPose()
{
    geometry_msgs::msg::TransformStamped t;
    geometry_msgs::msg::Pose prev_robot_pose = robot_pose_;

    // Look up for the transformation between target_frame and turtle2 frames
    // and send velocity commands for turtle2 to reach target_frame
    // std::string source_frame = "map";
    // std::string target_frame = "base_link";
    std::string global_frame, base_frame;
    this->get_parameter_or("global_frame", global_frame, std::string("map"));
    this->get_parameter_or("robot_base_frame", base_frame, std::string("base_link"));

    try
    {
        t = tf_buffer_->lookupTransform(
            // source_frame, target_frame,
            global_frame, base_frame,
            tf2::TimePointZero);
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            // target_frame.c_str(), target_frame.c_str(), ex.what());
            base_frame.c_str(), global_frame.c_str(), ex.what());
        return false;
    }

    // Convert from transform to pose
    robot_pose_.position.x = t.transform.translation.x;
    robot_pose_.position.y = t.transform.translation.y;
    robot_pose_.position.z = 0.0;
    robot_pose_.orientation = t.transform.rotation;

    if (prev_robot_pose.position.x != 0 and prev_robot_pose.position.y != 0)
    {
        float distance_incr = std::sqrt(std::pow(prev_robot_pose.position.x - robot_pose_.position.x, 2) +
                                        std::pow(prev_robot_pose.position.y - robot_pose_.position.y, 2));
        distance_traveled_ += fabs(distance_incr);

        angle_traveled_ += fabs(tf2::getYaw(prev_robot_pose.orientation) - tf2::getYaw(robot_pose_.orientation));
    }

    return true;
}