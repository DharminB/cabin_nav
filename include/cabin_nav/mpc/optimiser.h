#ifndef CABIN_OPTIMISER_H
#define CABIN_OPTIMISER_H

#include <functional>
#include <vector>
#include <chrono>

namespace cabin {

using CostFunction = std::function<float (const std::vector<float>& )>;

class Optimiser
{
    public:

        static void gradientDescent(
                float time_threshold,
                CostFunction calculateCost,
                std::vector<float>& u,
                float h = 1e-4f,
                float alpha = 1e-5f,
                float cost_threshold = 0.1f);

        static void lineSearch(
                float time_threshold,
                CostFunction calculateCost,
                std::vector<float>& u,
                float h = 1e-4f,
                float starting_alpha = 1e-8,
                float ending_alpha = 0.1f,
                float cost_threshold = 0.1f);

        static void conjugateGradientDescent(
                float time_threshold,
                CostFunction calculateCost,
                std::vector<float>& u,
                float h = 1e-4f,
                float starting_alpha = 1e-8,
                float ending_alpha = 0.1f,
                float cost_threshold = 0.1f);

        static void quasiNewton(
                float time_threshold,
                CostFunction calculateCost,
                std::vector<float>& u,
                float h = 1e-4f,
                float smallest_alpha = 1e-8,
                float gradient_threshold = 1e-3f,
                float cost_threshold = 0.1f);

    protected:
        static void calculateGradient(
                const std::vector<float>& u,
                float h,
                CostFunction calculateCost,
                float current_cost,
                std::vector<float>& gradient);

        static float calculateCostWithAlpha(
                const std::vector<float>& u,
                CostFunction calculateCost,
                const std::vector<float>& gradient,
                float alpha,
                std::vector<float>& new_u);

        static float calculateOptimalAlphaQN(
                const std::vector<float>& u,
                CostFunction calculateCost,
                float current_cost,
                const std::vector<float>& gradient,
                float smallest_alpha = 1e-8f,
                float cost_threshold = 0.1f);

        static float calculateOptimalAlpha(
                const std::vector<float>& u,
                CostFunction calculateCost,
                float current_cost,
                const std::vector<float>& gradient,
                float starting_alpha,
                float ending_alpha,
                float cost_threshold);
};

} // namespace cabin

#endif // CABIN_OPTIMISER_H
