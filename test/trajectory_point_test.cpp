#include <cmath>
#include <stdio.h>
#include <iostream>

#include <gtest/gtest.h>

#include <geometry_common/Pose2D.h>
#include <geometry_common/XYTheta.h>
#include <geometry_common/TransformMatrix2D.h>

#include <cabin_nav/structs/trajectory_point.h>

using cabin::TrajectoryPoint;
using kelo::geometry_common::Pose2D;
using kelo::geometry_common::Velocity2D;
using kelo::geometry_common::TransformMatrix2D;

TEST(TrajectoryPoint, simpleCreation)
{
    TrajectoryPoint s1(1.0f, 2.0f, 3.0f, 0.1f, 0.2f, 0.3f, 4.0f);
    EXPECT_NEAR(s1.pos.x, 1.0f, 1e-3f);
    EXPECT_NEAR(s1.pos.y, 2.0f, 1e-3f);
    EXPECT_NEAR(s1.pos.theta, 3.0f, 1e-3f);
    EXPECT_NEAR(s1.vel.x, 0.1f, 1e-3f);
    EXPECT_NEAR(s1.vel.y, 0.2f, 1e-3f);
    EXPECT_NEAR(s1.vel.theta, 0.3f, 1e-3f);
    EXPECT_NEAR(s1.t, 4.0f, 1e-3f);

    TrajectoryPoint s2(s1);
    EXPECT_NEAR(s2.pos.x, 1.0f, 1e-3f);
    EXPECT_NEAR(s2.pos.y, 2.0f, 1e-3f);
    EXPECT_NEAR(s2.pos.theta, 3.0f, 1e-3f);
    EXPECT_NEAR(s2.vel.x, 0.1f, 1e-3f);
    EXPECT_NEAR(s2.vel.y, 0.2f, 1e-3f);
    EXPECT_NEAR(s2.vel.theta, 0.3f, 1e-3f);
    EXPECT_NEAR(s2.t, 4.0f, 1e-3f);

    TrajectoryPoint s3(Pose2D(1.0f, 2.0f, 3.0f), Velocity2D(0.1f, 0.2f, 0.3f), 4.0f);
    EXPECT_NEAR(s3.pos.x, 1.0f, 1e-3f);
    EXPECT_NEAR(s3.pos.y, 2.0f, 1e-3f);
    EXPECT_NEAR(s3.pos.theta, 3.0f, 1e-3f);
    EXPECT_NEAR(s3.vel.x, 0.1f, 1e-3f);
    EXPECT_NEAR(s3.vel.y, 0.2f, 1e-3f);
    EXPECT_NEAR(s3.vel.theta, 0.3f, 1e-3f);
    EXPECT_NEAR(s3.t, 4.0f, 1e-3f);
}

TEST(TrajectoryPoint, equality)
{
    TrajectoryPoint s1(1.0f, 2.0f, 3.0f, 0.1f, 0.2f, 0.3f, 4.0f);
    TrajectoryPoint s2(s1);

    EXPECT_TRUE(s1 == s2);

    TrajectoryPoint s3(Pose2D(0.0f, 2.0f, 3.0f), Velocity2D(0.1f, 0.2f, 0.3f), 4.0f);

    EXPECT_FALSE(s1 == s3);
}

TEST(TrajectoryPoint, substraction)
{
    TrajectoryPoint s1(1.0f, 2.0f, 3.0f, 0.1f, 0.2f, 0.3f, 4.0f);
    TrajectoryPoint s2;

    TrajectoryPoint diff = s1 - s2;
    EXPECT_NEAR(diff.pos.x, 1.0f, 1e-3f);
    EXPECT_NEAR(diff.pos.y, 2.0f, 1e-3f);
    EXPECT_NEAR(diff.pos.theta, 3.0f, 1e-3f);
    EXPECT_NEAR(diff.vel.x, 0.1f, 1e-3f);
    EXPECT_NEAR(diff.vel.y, 0.2f, 1e-3f);
    EXPECT_NEAR(diff.vel.theta, 0.3f, 1e-3f);
    EXPECT_NEAR(diff.t, 4.0f, 1e-3f);
}

TEST(TrajectoryPoint, stateMulTf)
{
    TrajectoryPoint load_state(1.0f, 2.0f, 1.0f, 0.5f, 0.0f, 0.3f, 4.0f);
    TransformMatrix2D tf(1.0f, 0.0f, 0.0f);

    TrajectoryPoint robot_state = load_state * tf;

    EXPECT_FALSE(load_state == robot_state);
    EXPECT_GT(robot_state.pos.x, load_state.pos.x);
    EXPECT_GT(robot_state.pos.y, load_state.pos.y);
    EXPECT_NEAR(robot_state.pos.theta, load_state.pos.theta, 1e-3f);
    EXPECT_NEAR(robot_state.vel.x, load_state.vel.x, 1e-3f);
    EXPECT_NEAR(robot_state.vel.y, load_state.vel.theta * tf.x(), 1e-3f);
    EXPECT_NEAR(robot_state.vel.theta, load_state.vel.theta, 1e-3f);
    EXPECT_NEAR(robot_state.t, 4.0f, 1e-3f);

    TrajectoryPoint load_state_2 = robot_state * tf.calcInverse();

    EXPECT_TRUE(load_state == load_state_2);
}

TEST(TrajectoryPoint, tfMulState)
{
    TrajectoryPoint load_state_frame_1(1.0f, 2.0f, 1.0f, 0.5f, 0.0f, 0.3f, 4.0f);
    TransformMatrix2D tf(1.0f, 0.0f, 0.0f);

    TrajectoryPoint load_state_frame_2 = tf * load_state_frame_1;

    EXPECT_FALSE(load_state_frame_1 == load_state_frame_2);
    EXPECT_EQ(tf * load_state_frame_1.pos, load_state_frame_2.pos);
    EXPECT_EQ(load_state_frame_1.vel, load_state_frame_2.vel);
    EXPECT_NEAR(load_state_frame_1.t, load_state_frame_2.t, 1e-3f);

    TrajectoryPoint load_state_frame_1_new = tf.calcInverse() * load_state_frame_2;

    EXPECT_TRUE(load_state_frame_1 == load_state_frame_1_new);
}
