#ifndef CABIN_PTP_OCC_GRID_behavior_H
#define CABIN_PTP_OCC_GRID_behavior_H

#include <mutex>
#include <thread>
#include <atomic>

#include <cabin_nav/behavior/ptp_behavior.h>
#include <cabin_nav/utils/lattice_search_utils.h>

namespace cabin {

class PTPOccGridBehavior : public PTPBehavior
{
    public:

        PTPOccGridBehavior():
            PTPBehavior("ptp_occ_grid") {}

        virtual ~PTPOccGridBehavior() = default;

        bool configure(const YAML::Node& params);

        void reset();

    protected:

        std::thread planning_thread_;
        OccupancyGrid::Ptr occ_grid_map_;
        OccupancyGrid::Ptr voronoi_map_;
        AllNeighbourOffsets neighbour_offsets_;
        LatticeConfig lattice_config_;
        std::shared_ptr<Lattice<bool>> dummy_lattice_;

        std::atomic<bool> planning_thread_active_{false};

        std::mutex planning_mutex_;
        kelo::geometry_common::Path global_path_;
        bool global_path_success_{false};

        size_t voronoi_obst_cell_dist_threshold_{2};

        virtual void planGeometricPath(
                std::vector<visualization_msgs::Marker>& markers);

        void planGlobalPath();

        bool findNearestVoronoi(
                const kelo::geometry_common::Pose2D& start_pose,
                kelo::geometry_common::Path& pose_path);

        bool findVoronoiPath(
                const kelo::geometry_common::Pose2D& start_pose,
                const kelo::geometry_common::Pose2D& goal_pose,
                kelo::geometry_common::Path& pose_path);

};

} // namespace cabin

#endif // CABIN_PTP_OCC_GRID_behavior_H
