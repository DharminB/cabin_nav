#include <cmath>
#include <stdio.h>
#include <iostream>

#include <gtest/gtest.h>

#include <cabin_nav/mpc/optimiser.h>

float calcCostRosenBrock(const std::vector<float>& u)
{
    return std::pow(1 - u[0], 2) + 100 * std::pow(u[1] - std::pow(u[0], 2), 2);
}

float calcCost(const std::vector<float>& u)
{
    return std::pow(std::pow(u[0], 2) + u[1] - 11, 2) + std::pow(u[0] + std::pow(u[1], 2) - 7, 2);
}

TEST(Optimiser, normalCGD)
{
    std::vector<float> u{0.0f, 0.0f};
    std::vector<float> ans{3.0f, 2.0f};
    cabin::CostFunction cf = std::bind(calcCost, std::placeholders::_1);

    std::chrono::system_clock::time_point start_time = std::chrono::system_clock::now();

    cabin::Optimiser::conjugateGradientDescent(1.0f, cf, u, 1e-4f, 1e-4f, 1.0f, 1e-8f);

    std::chrono::duration<float> time_duration = std::chrono::system_clock::now() - start_time;
    // std::cout << "Time taken: " << time_duration.count() * 1e6f << " us" << std::endl;
    EXPECT_LT(time_duration.count(), 1000);
    EXPECT_EQ(u.size(), ans.size());
    for ( size_t i = 0; i < u.size(); i++ )
    {
        EXPECT_NEAR(u[i], ans[i], 1e-3f);
    }
}

TEST(Optimiser, normalQN)
{
    std::vector<float> u{0.0f, 0.0f};
    std::vector<float> ans{3.0f, 2.0f};
    cabin::CostFunction cf = std::bind(calcCost, std::placeholders::_1);

    std::chrono::system_clock::time_point start_time = std::chrono::system_clock::now();

    cabin::Optimiser::quasiNewton(1.0f, cf, u, 1e-4f, 1e-4f, 1e-3f, 1e-8f);

    std::chrono::duration<float> time_duration = std::chrono::system_clock::now() - start_time;
    // std::cout << "Time taken: " << time_duration.count() * 1e6f << " us" << std::endl;
    EXPECT_LT(time_duration.count(), 1000);
    EXPECT_EQ(u.size(), ans.size());
    for ( size_t i = 0; i < u.size(); i++ )
    {
        EXPECT_NEAR(u[i], ans[i], 1e-3f);
    }
}

TEST(Optimiser, rosenBrockCGD)
{
    std::vector<float> u{0.0f, -1.0f};
    std::vector<float> ans{1.0f, 1.0f};
    cabin::CostFunction cf = std::bind(calcCostRosenBrock, std::placeholders::_1);

    std::chrono::system_clock::time_point start_time = std::chrono::system_clock::now();

    cabin::Optimiser::conjugateGradientDescent(1.0f, cf, u, 1e-4f, 1e-4f, 1.0f, 1e-8f);

    std::chrono::duration<float> time_duration = std::chrono::system_clock::now() - start_time;
    // std::cout << "Time taken: " << time_duration.count() * 1e6f << " us" << std::endl;
    EXPECT_LT(time_duration.count(), 1000);
    EXPECT_EQ(u.size(), ans.size());
    for ( size_t i = 0; i < u.size(); i++ )
    {
        EXPECT_NEAR(u[i], ans[i], 1e-1f);
    }
}

TEST(Optimiser, rosenBrockQN)
{
    std::vector<float> u{0.0f, -1.0f};
    std::vector<float> ans{1.0f, 1.0f};
    cabin::CostFunction cf = std::bind(calcCostRosenBrock, std::placeholders::_1);

    std::chrono::system_clock::time_point start_time = std::chrono::system_clock::now();

    cabin::Optimiser::quasiNewton(1.0f, cf, u, 1e-4f, 1e-4f, 1e-3f, 1e-8f);

    std::chrono::duration<float> time_duration = std::chrono::system_clock::now() - start_time;
    // std::cout << "Time taken: " << time_duration.count() * 1e6f << " us" << std::endl;
    EXPECT_LT(time_duration.count(), 1000);
    EXPECT_EQ(u.size(), ans.size());
    for ( size_t i = 0; i < u.size(); i++ )
    {
        EXPECT_NEAR(u[i], ans[i], 1e-1f);
    }
}
