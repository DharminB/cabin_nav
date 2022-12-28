#include <cabin_nav/utils/print.h>
#include <cabin_nav/structs/trajectory_point.h>

using kelo::geometry_common::TransformMatrix2D;
using kelo::geometry_common::Pose2D;

namespace cabin {

bool TrajectoryPoint::operator == (const TrajectoryPoint& other)
{
    return ( pos == other.pos && vel == other.vel && t == other.t );
}

TrajectoryPoint TrajectoryPoint::operator - (const TrajectoryPoint& other) const
{
    return TrajectoryPoint(pos - other.pos, vel - other.vel, t - other.t);
}

TrajectoryPoint TrajectoryPoint::operator * (const TransformMatrix2D& tf) const
{
    const TransformMatrix2D tf_inv = tf.calcInverse();
    return TrajectoryPoint(Pose2D(TransformMatrix2D(pos) * tf_inv), tf * vel, t);
}

TrajectoryPoint operator * (const TransformMatrix2D& tf, const TrajectoryPoint& traj_pt)
{
    return TrajectoryPoint(tf * traj_pt.pos, traj_pt.vel, traj_pt.t);
}

Trajectory operator * (const Trajectory& traj, const TransformMatrix2D& tf)
{
    const TransformMatrix2D tf_inv = tf.calcInverse();
    Trajectory transformed_traj;
    transformed_traj.reserve(traj.size());
    for ( const TrajectoryPoint& tp : traj )
    {
        transformed_traj.push_back(TrajectoryPoint(
                    Pose2D(TransformMatrix2D(tp.pos) * tf_inv), tf * tp.vel, tp.t));
    }
    return transformed_traj;
}

Trajectory operator * (const TransformMatrix2D& tf, const Trajectory& traj)
{
    Trajectory transformed_traj;
    transformed_traj.reserve(traj.size());
    for ( const TrajectoryPoint& tp : traj )
    {
        transformed_traj.push_back(tf * tp);
    }
    return transformed_traj;
}


std::ostream& operator << (std::ostream& out, const TrajectoryPoint& state)
{
    out << "<pos: " << state.pos
        << ", vel: " << state.vel
        << ", t: " << state.t
        << ">";
    return out;
}

std::ostream& operator << (std::ostream& out, const Trajectory& traj)
{
    out << "Trajectory:" << std::endl;
    for ( const TrajectoryPoint& s : traj )
    {
        out << "  " << s << std::endl;
    }
    return out;
}

} // namespace cabin
