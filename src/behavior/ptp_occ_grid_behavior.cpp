#include <geometry_common/Utils.h>
#include <yaml_common/Parser2.h>

#include <cabin_nav/utils/print.h>
#include <cabin_nav/structs/context_data.h>
#include <cabin_nav/utils/voronoi_calculator.h>
#include <cabin_nav/utils/lattice_search_utils.h>
#include <cabin_nav/input/velocity_input_data.h>
#include <cabin_nav/input/localisation_input_data.h>
#include <cabin_nav/input/laser_input_data.h>
#include <cabin_nav/input/occupancy_grid_map_input_data.h>
#include <cabin_nav/output/cmd_vel_output_data.h>
#include <cabin_nav/output/visualization_marker_output_data.h>
#include <cabin_nav/action/goto_action.h>
#include <cabin_nav/behavior/ptp_occ_grid_behavior.h>

using kelo::geometry_common::Path;
using kelo::geometry_common::Circle;
using kelo::geometry_common::Pose2D;
using kelo::geometry_common::PointCloud2D;
using kelo::geometry_common::XYTheta;
using kelo::geometry_common::Acceleration2D;
using kelo::geometry_common::TransformMatrix2D;
using GCUtils = kelo::geometry_common::Utils;
using Parser = kelo::yaml_common::Parser2;

namespace cabin {

bool PTPOccGridBehavior::configure(const YAML::Node& config)
{
    if ( !PTPBehavior::configure(config) )
    {
        return false;
    }
    voronoi_obst_cell_dist_threshold_ = Parser::get<size_t>(
            config, "voronoi_obst_cell_dist_threshold", 2);
    return true;
}

void PTPOccGridBehavior::reset()
{
    PTPBehavior::reset();
    planning_thread_active_ = false;
    global_path_success_ = false;
    if ( planning_thread_.joinable() )
    {
        planning_thread_.join(); // blocking call
        global_path_.clear();
    }
}

void PTPOccGridBehavior::planGeometricPath(std::vector<visualization_msgs::Marker>& markers)
{
    TrajectoryPoint start;

    if ( Utils::isWithinTolerance(
                start.pos, goal_.pos,
                geometric_planner_goal_tolerance_linear_,
                geometric_planner_goal_tolerance_angular_) )
    {
        // std::cout << "goal is very close" << std::endl;
        return;
    }

    if ( !planning_thread_active_ )
    {
        if ( planning_thread_.joinable() ) // planning finished
        {
            planning_thread_.join();
        }
        else if ( !global_path_success_ ) // need to start planning
        {
            planning_thread_ = std::thread(&PTPOccGridBehavior::planGlobalPath, this);
        }
    }

    // if ( occ_grid_map_ )
    // {
    //     markers.push_back(occ_grid_map_->getMarker());
    //     markers.back().pose = Utils::invert2DTransformPose(context_data_->current.pos).getPose();
    // }
    // if ( voronoi_map_ )
    // {
    //     markers.push_back(voronoi_map_->getMarker("", 0.0f, 0.0f, 1.0f));
    //     markers.back().pose = Utils::invert2DTransformPose(context_data_->current.pos).getPose();
    // }
    std::lock_guard<std::mutex> guard(planning_mutex_); // acquire lock within current scope
    if ( global_path_.empty() || !global_path_success_ )
    {
        std::cout << "[PTPOccGridBehavior] Waiting for planner to finish." << std::endl;
        goal_ = start;
        return;
    }

    /* transform path from global to local frame */
    TransformMatrix2D tf = localisation_tf_.calcInverse();
    Path geometric_path = tf * global_path_;
    for ( size_t i = 0; i+1 < geometric_path.size(); i++ )
    {
        geometric_path[i].theta = (geometric_path[i+1].position() -
                                   geometric_path[i].position()).angle();
    }
    geometric_path.back().theta = goal_.pos.theta;
    markers.push_back(GCUtils::convertGeometricPathToMarker(
                geometric_path, "robot", 0.0f, 0.33f, 0.0f, 1.0f, 0.02f));

    size_t closest_index = 0;
    float min_dist = 1e3f;
    for ( size_t i = 0; i < geometric_path.size(); i++ )
    {
        float dist = geometric_path[i].position().magnitude();
        if ( dist < min_dist )
        {
            min_dist = dist;
            closest_index = i;
        }
    }
    // prune path from beginning to robot
    geometric_path.erase(geometric_path.begin(), geometric_path.begin()+closest_index);

    int i = GeometricPlanner::calcIndexAtDist(geometric_path, robot_to_goal_dist_);
    if ( i < 0 )
    {
        return;
    }
    goal_.pos = geometric_path[i];
}

void PTPOccGridBehavior::planGlobalPath()
{
    // std::cout << "inside PTPOccGridBehavior::planGlobalPath" << std::endl;
    planning_thread_active_ = true;
    if ( !occ_grid_map_ )
    {
        float footprint_circumcircle_radius = ( is_footprint_box_ )
                                              ? Utils::calcCircumcircleRadius(box_footprint_)
                                              : circle_footprint_.r;
        /* create a local copy of occ grid with inflation of robot'tp circumcircle */
        OccupancyGrid::Ptr occ_grid_map;
        OccupancyGridMapInputData::getOccupancyGridMap(context_data_->input_data_map,
                    required_inputs_map_.at("occ_grid_map"), occ_grid_map);
        occ_grid_map_ = occ_grid_map->calcDilated(footprint_circumcircle_radius);

        if ( is_footprint_box_ )
        {
            occ_grid_map_->generateRobotPts(box_footprint_);
        }
        else
        {
            occ_grid_map_->generateRobotPts(circle_footprint_);
        }
        /* calculate voronoi map of dilated occ grid */
        voronoi_map_ = VoronoiCalculator::calculateVoronoi(
                *occ_grid_map_, voronoi_obst_cell_dist_threshold_);

        neighbour_offsets_ = LatticeSearchUtils::calcNeighbourOffset(
            occ_grid_map_->getGridCellSize(), 1, 0, 0, "");

        lattice_config_.grid_size_x = voronoi_map_->getGridSizeX();
        lattice_config_.grid_size_y = voronoi_map_->getGridSizeY();
        lattice_config_.grid_cell_size = voronoi_map_->getGridCellSize();
        lattice_config_.num_of_angles = 1;
        lattice_config_.tf = voronoi_map_->getTransform();

        dummy_lattice_ = std::make_shared<Lattice<bool>>(lattice_config_, false);
    }
    // std::cout << occ_grid_map_->getGridSizeX() << " " 
    //           << occ_grid_map_->getGridSizeY() << std::endl;

    const GoToAction::Ptr goto_action = std::dynamic_pointer_cast<GoToAction>(
             context_data_->plan[context_data_->plan_index]);
    if ( goto_action == nullptr )
    {
        std::cout << Print::Err << Print::Time() << "[PTPOccGridBehavior] "
                  << "Could not cast current action to GoToAction"
                  << Print::End << std::endl;
        planning_mutex_.unlock();
        global_path_success_ = false;
        planning_mutex_.unlock();
        planning_thread_active_ = false;
        return;
    }

    const Pose2D robot_pose = localisation_tf_.asPose2D();
    Path geometric_path;
    std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();

    Path robot_to_voronoi_path, goal_to_voronoi_path, voronoi_to_voronoi_path;

    bool success = false;
    if ( findNearestVoronoi(robot_pose, robot_to_voronoi_path) &&
         findNearestVoronoi(goto_action->getIntermediateGoal().pos, goal_to_voronoi_path) &&
         findVoronoiPath(robot_to_voronoi_path.back(),
             goal_to_voronoi_path.back(), voronoi_to_voronoi_path) )
    {
        geometric_path.reserve(robot_to_voronoi_path.size() +
                               voronoi_to_voronoi_path.size() +
                               goal_to_voronoi_path.size());
        geometric_path.insert(geometric_path.end(), robot_to_voronoi_path.begin(),
                              robot_to_voronoi_path.end());
        geometric_path.insert(geometric_path.end(), voronoi_to_voronoi_path.begin(),
                              voronoi_to_voronoi_path.end());
        std::reverse(goal_to_voronoi_path.begin(), goal_to_voronoi_path.end());
        geometric_path.insert(geometric_path.end(), goal_to_voronoi_path.begin(),
                              goal_to_voronoi_path.end());
        success = true;
    }

    // std::cout << "success: " << success << std::endl;

    // std::cout << "path size: " << geometric_path.size() << std::endl;
    std::chrono::duration<float> search_duration = std::chrono::steady_clock::now() - start_time;
    // std::cout << search_duration.count() << std::endl;

    planning_mutex_.lock();
    global_path_ = geometric_path;
    global_path_success_ = success;
    planning_mutex_.unlock();
    planning_thread_active_ = false;
}

bool PTPOccGridBehavior::findNearestVoronoi(const Pose2D& start_pose,
                                             Path& pose_path)
{
    return LatticeSearchUtils::search(
            start_pose,
            [this](const LatticeIndex& lat_ind) // GoalTestFunction
            {
                return voronoi_map_->isOccupied(lat_ind.i, lat_ind.j);
            },
            [this](const LatticeIndex& lat_ind) // CollisionTestFunction
            {
                return occ_grid_map_->isOccupied(lat_ind.i, lat_ind.j);
            },
            [](const LatticeIndex& lat_ind) // HeuristicFunction
            {
                return 0.0f;
            },
            neighbour_offsets_,
            lattice_config_,
            pose_path,
            SearchType::DIJKSTRA,
            0.01f); // 10 ms
}

bool PTPOccGridBehavior::findVoronoiPath(
        const Pose2D& start_pose, const Pose2D& goal_pose,
        Path& pose_path)
{
    LatticeIndex goal_lat_ind;
    if ( !dummy_lattice_->calcGridIndex(goal_pose, goal_lat_ind) )
    {
        std::cerr << Print::Warn << Print::Time() << "[PTPOccGridBehavior] "
                  << "Could not initialise goal node."
                  << Print::End << std::endl;
        return false;
    }
    // std::cout << "goal: " << goal_lat_ind << std::endl;

    GoalTestFunction isGoal = std::bind(
            &LatticeSearchUtils::isLatticeIndexSame,
            std::placeholders::_1, goal_lat_ind);

    HeuristicFunction calcHeuristic = std::bind(
            &LatticeSearchUtils::heuristic3d,
            std::placeholders::_1, goal_lat_ind,
            lattice_config_.grid_cell_size,
            lattice_config_.num_of_angles,
            (2*M_PI)/lattice_config_.num_of_angles);

    return LatticeSearchUtils::search(
            start_pose,
            isGoal,
            [this](const LatticeIndex& lat_ind) // CollisionTestFunction
            {
                return ( !voronoi_map_->isOccupied(lat_ind.i, lat_ind.j) );
            },
            calcHeuristic,
            neighbour_offsets_,
            lattice_config_,
            pose_path,
            SearchType::ASTAR,
            0.2f); // 200 ms
}

} // namespace cabin
