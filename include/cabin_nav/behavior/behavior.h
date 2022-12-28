#ifndef CABIN_behavior_H
#define CABIN_behavior_H

#include <utility>
#include <functional>

#include <visualization_msgs/Marker.h>

#include <yaml-cpp/yaml.h>

#include <geometry_common/Point2D.h>
#include <geometry_common/XYTheta.h>
#include <geometry_common/Circle.h>
#include <geometry_common/Box2D.h>

#include <cabin_nav/structs/trajectory_point.h>

namespace cabin {

/* forward declaration */
class ContextData;
class BehaviorFeedback;
class Action;

class Behavior
{
    public:

        using Ptr = std::shared_ptr<Behavior>;

        using Map = std::map<std::string, Behavior::Ptr>;

        virtual ~Behavior() = default;

        virtual bool configure(const YAML::Node& params) = 0;

        virtual void reset() = 0;

        virtual void runOnce(const ContextData& context_data, BehaviorFeedback& fb) = 0;

        virtual bool preConditionSatisfied(
                const ContextData& context_data, std::shared_ptr<Action> action) const = 0;

        virtual bool postConditionSatisfied(const ContextData& context_data) const = 0;

        virtual void updateAction(std::shared_ptr<Action>& action);

        std::string getRecoveryBehaviorName(const std::string& failure_code) const;

        const std::vector<std::string>& getRequiredInputs() const;

        const std::string& getType() const;

        const std::string& getName() const;

        void setName(const std::string& name);

        virtual float getIdealLoopDuration() const;

    protected:

        std::vector<float> sample_times_{1.0f};

        kelo::geometry_common::Acceleration2D max_acc_;
        kelo::geometry_common::Velocity2D max_vel_;
        kelo::geometry_common::Velocity2D min_vel_;

        kelo::geometry_common::XYTheta acc_limit_weights_;
        kelo::geometry_common::XYTheta vel_limit_weights_;
        kelo::geometry_common::XYTheta goal_state_pos_weights_;
        kelo::geometry_common::XYTheta goal_state_vel_weights_;

        kelo::geometry_common::Circle circle_footprint_;
        kelo::geometry_common::Box2D box_footprint_;
        bool is_footprint_box_{true};

        /* either fixed inflation dist (inflation_dist_) is used or variable
         * inflation dist is used by calculating braking dist and clipping
         * between min_inflation_dist_ and max_inflation_dist_. The behavior
         * chooses which to use in its cost function. */
        float inflation_dist_{0.2f};
        float min_inflation_dist_{0.1f};
        float max_inflation_dist_{1.0f};


        std::map<std::string, std::string> failure_to_recovery_map_;

        std::vector<std::string> required_inputs_;

        visualization_msgs::Marker behavior_name_marker_;

        visualization_msgs::Marker footprint_marker_;

        Behavior(const std::string& type);

        virtual float calcCost(const std::vector<float>& u) = 0;

        virtual void calcInitialU(
                std::vector<float>& u,
                BehaviorFeedback& fb) = 0;

        /*
         * Setter functions and configure function helpers
         */

        void setAccLimits(
                const kelo::geometry_common::Acceleration2D& max_acc =
                    kelo::geometry_common::Acceleration2D());

        void setVelLimits(
                const kelo::geometry_common::Velocity2D& max_vel =
                    kelo::geometry_common::Velocity2D(),
                const kelo::geometry_common::Velocity2D& min_vel =
                    kelo::geometry_common::Velocity2D());

        bool parseAccLimits(const YAML::Node& config);

        bool parseVelLimits(const YAML::Node& config);

        bool parseControlHorizon(const YAML::Node& config);

        bool parseAccLimitWeights(const YAML::Node& config);

        bool parseVelLimitWeights(const YAML::Node& config);

        bool parseGoalStateWeights(const YAML::Node& config);

        bool parseFootprint(const YAML::Node& config);

        bool parseFailureToRecoveryMap(const YAML::Node& config);

        bool parseRequiredInputs(const YAML::Node& config);

        /*
         * Cost function helpers
         */

        float calcAccLimitCost(
                const std::vector<float>& u,
                bool is_unicycle = false) const;

        float calcVelLimitCost(
                const kelo::geometry_common::Velocity2D& vel) const;

        float calcLaserPtsCostBoxFootprint(
                const kelo::geometry_common::PointCloud2D& laser_pts,
                const kelo::geometry_common::Pose2D& pose,
                float inflation_dist) const;

        float calcLaserPtsCostCircleFootprint(
                const kelo::geometry_common::PointCloud2D& laser_pts,
                const kelo::geometry_common::Pose2D& pose,
                float inflation_dist) const;

        float calcLaserPtsCost(
                const kelo::geometry_common::PointCloud2D& laser_pts,
                const kelo::geometry_common::Pose2D& pose,
                float inflation_dist) const;

        float calcLaserPtsCostVelBased(
                const kelo::geometry_common::PointCloud2D& laser_pts,
                const TrajectoryPoint& traj_pt,
                float min_inflation_dist,
                float max_inflation_dist) const;

        float calcCirclesCostBoxFootprint(
                const std::vector<kelo::geometry_common::Circle>& obstacles,
                const kelo::geometry_common::Pose2D& pose,
                float inflation_dist) const;

        float calcCirclesCostCircleFootprint(
                const std::vector<kelo::geometry_common::Circle>& obstacles,
                const kelo::geometry_common::Pose2D& pose,
                float inflation_dist) const;

        float calcCirclesCost(
                const std::vector<kelo::geometry_common::Circle>& obstacles,
                const kelo::geometry_common::Pose2D& pose,
                float inflation_dist) const;

        float calcCirclesCostVelBased(
                const std::vector<kelo::geometry_common::Circle>& obstacles,
                const TrajectoryPoint& traj_pt,
                float min_inflation_dist,
                float max_inflation_dist) const;

        float calcTrajectoryPointCost(
                const TrajectoryPoint& s1,
                const TrajectoryPoint& s2,
                const kelo::geometry_common::XYTheta& pos_weights,
                const kelo::geometry_common::XYTheta& vel_weights) const;

        float calcGoalCost(
                const TrajectoryPoint& traj_pt,
                const TrajectoryPoint& goal) const;

        Trajectory calcRampedTrajectory(
                const kelo::geometry_common::Velocity2D& curr_vel,
                const kelo::geometry_common::Velocity2D& target_vel =
                    kelo::geometry_common::Velocity2D()) const;

        Trajectory calcRampedTrajectory(
                const TrajectoryPoint& current,
                const kelo::geometry_common::Velocity2D& target_vel =
                    kelo::geometry_common::Velocity2D()) const;

        void addInitialGuessTrajectoryMarkers(
                const Trajectory& traj,
                std::vector<visualization_msgs::Marker>& markers) const;

    private:

        std::string type_;
        std::string name_;

};

} // namespace cabin

#endif // CABIN_behavior_H
