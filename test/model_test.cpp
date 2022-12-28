#include <cmath>
#include <stdio.h>
#include <iostream>

#include <gtest/gtest.h>

#include <geometry_common/XYTheta.h>

#include <cabin_nav/mpc/model.h>
#include <cabin_nav/structs/trajectory_point.h>

using kelo::geometry_common::Acceleration2D;
using cabin::TrajectoryPoint;
using cabin::Trajectory;
using cabin::Model;

TEST(Model, predictNextTrajectoryPoint)
{
    TrajectoryPoint start_state(0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f);
    Acceleration2D cmd_acc(0.0f, 0.0f, 0.0f);
    TrajectoryPoint next_state = Model::predictNextTrajectoryPoint(start_state, 1.0f, cmd_acc);
    EXPECT_NEAR(next_state.pos.x, 1.0f, 1e-3f);
    EXPECT_NEAR(next_state.pos.y, 0.0f, 1e-3f);
    EXPECT_NEAR(next_state.pos.theta, 0.0f, 1e-3f);
    EXPECT_NEAR(next_state.vel.x, 1.0f, 1e-3f);
    EXPECT_NEAR(next_state.vel.y, 0.0f, 1e-3f);
    EXPECT_NEAR(next_state.vel.theta, 0.0f, 1e-3f);
    EXPECT_NEAR(next_state.t, 1.0f, 1e-3f);

    cmd_acc = Acceleration2D(0.1f, 0.0f, 0.0f);
    next_state = Model::predictNextTrajectoryPoint(start_state, 1.0f, cmd_acc);
    EXPECT_GT(next_state.pos.x, 1.0f);
    EXPECT_NEAR(next_state.pos.y, 0.0f, 1e-3f);
    EXPECT_NEAR(next_state.pos.theta, 0.0f, 1e-3f);
    EXPECT_NEAR(next_state.vel.x, 1.1f, 1e-3f);
    EXPECT_NEAR(next_state.vel.y, 0.0f, 1e-3f);
    EXPECT_NEAR(next_state.vel.theta, 0.0f, 1e-3f);
    EXPECT_NEAR(next_state.t, 1.0f, 1e-3f);

    start_state = TrajectoryPoint(0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f);
    cmd_acc = Acceleration2D(0.0f, 0.0f, 0.0f);
    next_state = Model::predictNextTrajectoryPoint(start_state, 1.0f, cmd_acc);
    EXPECT_NEAR(next_state.pos.x, 0.0f, 1e-3f);
    EXPECT_NEAR(next_state.pos.y, 1.0f, 1e-3f);
    EXPECT_NEAR(next_state.pos.theta, 0.0f, 1e-3f);
    EXPECT_NEAR(next_state.vel.x, 0.0f, 1e-3f);
    EXPECT_NEAR(next_state.vel.y, 1.0f, 1e-3f);
    EXPECT_NEAR(next_state.vel.theta, 0.0f, 1e-3f);
    EXPECT_NEAR(next_state.t, 1.0f, 1e-3f);

    cmd_acc = Acceleration2D(0.0f, 0.1f, 0.0f);
    next_state = Model::predictNextTrajectoryPoint(start_state, 1.0f, cmd_acc);
    EXPECT_NEAR(next_state.pos.x, 0.0f, 1e-3f);
    EXPECT_GT(next_state.pos.y, 1.0f);
    EXPECT_NEAR(next_state.pos.theta, 0.0f, 1e-3f);
    EXPECT_NEAR(next_state.vel.x, 0.0f, 1e-3f);
    EXPECT_NEAR(next_state.vel.y, 1.1f, 1e-3f);
    EXPECT_NEAR(next_state.vel.theta, 0.0f, 1e-3f);
    EXPECT_NEAR(next_state.t, 1.0f, 1e-3f);

    start_state = TrajectoryPoint(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f);
    cmd_acc = Acceleration2D(0.0f, 0.0f, 0.0f);
    next_state = Model::predictNextTrajectoryPoint(start_state, 1.0f, cmd_acc);
    EXPECT_NEAR(next_state.pos.x, 0.0f, 1e-3f);
    EXPECT_NEAR(next_state.pos.y, 0.0f, 1e-3f);
    EXPECT_NEAR(next_state.pos.theta, 1.0f, 1e-3f);
    EXPECT_NEAR(next_state.vel.x, 0.0f, 1e-3f);
    EXPECT_NEAR(next_state.vel.y, 0.0f, 1e-3f);
    EXPECT_NEAR(next_state.vel.theta, 1.0f, 1e-3f);
    EXPECT_NEAR(next_state.t, 1.0f, 1e-3f);

    cmd_acc = Acceleration2D(0.0f, 0.0f, 0.1f);
    next_state = Model::predictNextTrajectoryPoint(start_state, 1.0f, cmd_acc);
    EXPECT_NEAR(next_state.pos.x, 0.0f, 1e-3f);
    EXPECT_NEAR(next_state.pos.y, 0.0f, 1e-3f);
    EXPECT_GT(next_state.pos.theta, 1.0f);
    EXPECT_NEAR(next_state.vel.x, 0.0f, 1e-3f);
    EXPECT_NEAR(next_state.vel.y, 0.0f, 1e-3f);
    EXPECT_NEAR(next_state.vel.theta, 1.1f, 1e-3f);
    EXPECT_NEAR(next_state.t, 1.0f, 1e-3f);

    start_state = TrajectoryPoint(0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f);
    cmd_acc = Acceleration2D(0.0f, 0.0f, 0.0f);
    next_state = Model::predictNextTrajectoryPoint(start_state, M_PI/2, cmd_acc);
    EXPECT_NEAR(next_state.pos.x, 1.0f, 1e-1f);
    EXPECT_NEAR(next_state.pos.y, 1.0f, 1e-1f);
    EXPECT_NEAR(next_state.pos.theta, M_PI/2, 1e-1f);
    EXPECT_NEAR(next_state.vel.x, 1.0f, 1e-3f);
    EXPECT_NEAR(next_state.vel.y, 0.0f, 1e-3f);
    EXPECT_NEAR(next_state.vel.theta, 1.0f, 1e-3f);
    EXPECT_NEAR(next_state.t, M_PI/2, 1e-3f);
}

TEST(Model, calcTrajectory)
{
    TrajectoryPoint start_state(0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f);
    size_t num_of_states = 5;
    std::vector<float> u(5*3, 0.0f);
    std::vector<float> sample_times(5, 0.2f);
    Trajectory traj = Model::calcTrajectory(start_state, u, sample_times);
    ASSERT_EQ(traj.size(), 5u);
    for ( size_t i = 0; i < num_of_states; i++ )
    {
        EXPECT_FALSE(start_state == traj[i]);
        EXPECT_NEAR(traj[i].t, (i+1)*0.2f, 1e-3f);
        EXPECT_NEAR(traj[i].vel.x, start_state.vel.x, 1e-3f);
        EXPECT_NEAR(traj[i].vel.y, start_state.vel.y, 1e-3f);
        EXPECT_NEAR(traj[i].vel.theta, start_state.vel.theta, 1e-3f);
        if ( i > 0 )
        {
            EXPECT_GT(traj[i].pos.x, traj[i-1].pos.x);
            EXPECT_GT(traj[i].pos.y, traj[i-1].pos.y);
            EXPECT_GT(traj[i].pos.theta, traj[i-1].pos.theta);
        }
    }
}
