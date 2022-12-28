#include <geometry_common/Utils.h>

#include <cabin_nav/utils/utils.h>
#include <cabin_nav/mpc/optimiser.h>

using GCUtils = kelo::geometry_common::Utils;

namespace cabin {

void Optimiser::gradientDescent(
        float time_threshold,
        CostFunction calculateCost,
        std::vector<float>& u,
        float h,
        float alpha,
        float cost_threshold)
{
    std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
    std::chrono::duration<float> timeout_duration(time_threshold);

    float current_cost = calculateCost(u);
    std::vector<float> gradient(u.size());
    size_t itr_num = 0;
    // std::cout << "itr " << itr_num << " " << current_cost << std::endl;

    while ( true )
    {
        itr_num ++;

        // calculate gradient
        Optimiser::calculateGradient(u, h, calculateCost, current_cost, gradient);

        /* update u based on gradient and provided alpha */
        for ( size_t i = 0; i < u.size(); ++i )
        {
            u[i] -= alpha * gradient[i];
        }
        float new_cost = calculateCost(u);
        float delta_cost = current_cost - new_cost;
        current_cost = new_cost;
        // std::cout << "itr " << itr_num << " " << current_cost << std::endl;

        if ( std::fabs(delta_cost) < cost_threshold ||
             std::chrono::steady_clock::now() - start_time > timeout_duration )
        {
            break;
        }
    }
}

void Optimiser::lineSearch(
        float time_threshold,
        CostFunction calculateCost,
        std::vector<float>& u,
        float h,
        float starting_alpha,
        float ending_alpha,
        float cost_threshold)
{
    std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
    std::chrono::duration<float> timeout_duration(time_threshold);

    float current_cost = calculateCost(u);
    std::vector<float> gradient(u.size());
    size_t itr_num = 0;
    // std::cout << "itr " << itr_num << " " << current_cost << std::endl;

    while ( true )
    {
        itr_num ++;
        // calculate gradient
        Optimiser::calculateGradient(u, h, calculateCost, current_cost, gradient);

        // calculate new u with "loosely" optimal alpha
        float alpha = Optimiser::calculateOptimalAlpha(
                u, calculateCost, current_cost, gradient,
                starting_alpha, ending_alpha, cost_threshold);
        // std::cout << "alpha " << alpha << std::endl;

        for ( size_t i = 0; i < u.size(); ++i )
        {
            u[i] -= alpha * gradient[i];
        }
        float new_cost = calculateCost(u);
        float delta_cost = current_cost - new_cost;
        current_cost = new_cost;
        // std::cout << "itr " << itr_num << " " << current_cost << std::endl;

        if ( std::fabs(delta_cost) < cost_threshold ||
             std::chrono::steady_clock::now() - start_time > timeout_duration )
        {
            break;
        }
    }
}

void Optimiser::conjugateGradientDescent(
        float time_threshold,
        CostFunction calculateCost,
        std::vector<float>& u,
        float h,
        float starting_alpha,
        float ending_alpha,
        float cost_threshold)
{
    std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
    std::chrono::duration<float> timeout_duration(time_threshold);

    float current_cost = calculateCost(u);
    std::vector<float> prev_gradient;
    std::vector<float> conjugate_direction;
    std::vector<float> prev_conjugate_direction;
    std::vector<float> gradient(u.size());
    size_t itr_num = 0;
    // std::cout << "itr " << itr_num << " " << current_cost << std::endl;

    while ( true )
    {
        itr_num ++;
        // calculate gradient
        Optimiser::calculateGradient(u, h, calculateCost, current_cost, gradient);

        // calculate conjugate direction with polak ribiere method
        if ( prev_conjugate_direction.empty() && prev_gradient.empty() ) // for first iteration
        {
            conjugate_direction = std::vector<float>(gradient);
        }
        else
        {
            float numerator = 0.0f;
            float denominator = 0.0f;
            for ( size_t i = 0; i < u.size(); ++i )
            {
                numerator += gradient[i] * (gradient[i] - prev_gradient[i]);
                denominator += prev_gradient[i] * prev_gradient[i];
            }
            // float beta_ratio = std::max(0.0f, numerator/denominator);
            float beta_ratio = GCUtils::clip(numerator/denominator, 1.0f, 0.0f);
            // std::cout << "beta " << beta_ratio << std::endl;
            if ( std::isnan(beta_ratio) )
            {
                break;
            }

            conjugate_direction = std::vector<float>(gradient);
            for ( size_t i = 0; i < u.size(); ++i )
            {
                conjugate_direction[i] -= beta_ratio * prev_conjugate_direction[i];
            }
        }
        prev_gradient = std::vector<float>(gradient);
        prev_conjugate_direction = std::vector<float>(conjugate_direction);

        // calculate new u with "loosely" optimal alpha
        float alpha = Optimiser::calculateOptimalAlpha(
                u, calculateCost, current_cost, conjugate_direction,
                starting_alpha, ending_alpha, cost_threshold);

        // std::cout << "alpha " << alpha << std::endl;
        for ( size_t i = 0; i < u.size(); ++i )
        {
            u[i] -= alpha * conjugate_direction[i];
        }
        float new_cost = calculateCost(u);
        float delta_cost = current_cost - new_cost;
        current_cost = new_cost;

        // std::cout << "itr " << itr_num << " " << current_cost << std::endl;
        if ( std::fabs(delta_cost) < cost_threshold ||
             std::chrono::steady_clock::now() - start_time > timeout_duration )
        {
            break;
        }
    }
}

void Optimiser::quasiNewton(
        float time_threshold,
        CostFunction calculateCost,
        std::vector<float>& u,
        float h,
        float smallest_alpha,
        float gradient_threshold,
        float cost_threshold)
{
    std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
    std::chrono::duration<float> timeout_duration(time_threshold);

    float current_cost = calculateCost(u);
    size_t u_size = u.size();
    std::vector<float> gradient(u_size);
    std::vector<float> prev_gradient(u_size);
    std::vector<float> descent_direction(u_size);
    std::vector<float> s(u_size);
    std::vector<float> y(u_size);
    std::vector<float> identity(u_size*u_size, 0.0f);
    for ( size_t i = 0; i < u_size; i++ )
    {
        identity[i*u_size + i] = 1.0f;
    }
    // inverse hessian approximation initialised as identity matrix
    std::vector<float> hessian(identity);
    std::vector<float> rho_y_s(identity.size());
    std::vector<float> second_bracket(identity.size());
    std::vector<float> rho_s_y(identity.size());
    std::vector<float> first_bracket(identity.size());
    std::vector<float> intermediate_prod(identity.size());
    std::vector<float> first_term(identity.size());
    std::vector<float> rho_s_s(identity.size());

    size_t itr_num = 0;
    // std::cout << "itr " << itr_num << " " << current_cost << std::endl;

    // calculate gradient for first iteration
    Optimiser::calculateGradient(u, h, calculateCost, current_cost, gradient);

    while ( true )
    {
        itr_num ++;

        // calculate descent_direction from hessian and gradient
        Utils::matMulNxNToVector(hessian, gradient, descent_direction, u_size);

        // calculate new u with "loosely" optimal alpha
        float alpha = Optimiser::calculateOptimalAlphaQN(
                u, calculateCost, current_cost, descent_direction,
                smallest_alpha, cost_threshold);

        // std::cout << "alpha " << alpha << std::endl;
        for ( size_t i = 0; i < u_size; ++i )
        {
            s[i] = -alpha * descent_direction[i];
            u[i] += s[i];
        }
        float new_cost = calculateCost(u);
        float delta_cost = current_cost - new_cost;
        current_cost = new_cost;

        // std::cout << "itr " << itr_num << " " << current_cost << std::endl;
        if ( std::fabs(delta_cost) < cost_threshold ||
             std::chrono::steady_clock::now() - start_time > timeout_duration )
        {
            break;
        }

        prev_gradient = gradient;
        // calculate gradient
        Optimiser::calculateGradient(u, h, calculateCost, current_cost, gradient);
        // break if gradient is too small
        float max_gradient = 0.0f;
        for ( size_t i = 0; i < u_size; i++ )
        {
            if ( std::fabs(gradient[i]) > max_gradient )
            {
                max_gradient = std::fabs(gradient[i]);
            }
        }
        if ( max_gradient < gradient_threshold )
        {
            break;
        }

        // calculate BFGS inverse hessian approximation
        // H_{k+1} = (I − ρ_k*s_k*y_k^T) * H_k * (I − ρ_k*y_k*s_k^T) + ρ_k*s_k*s_k^T

        Utils::matSubNxN(gradient, prev_gradient, y);

        // calculate ρ_k
        float rho = 1.0f / Utils::innerVecMulNxN(y, s, u_size);
        Utils::outerVecMulNxN(s, y, rho_s_y, u_size);
        Utils::matMulNxNScalar(rho_s_y, rho);
        Utils::matSubNxN(identity, rho_s_y, first_bracket);
        Utils::outerVecMulNxN(y, s, rho_y_s, u_size);
        Utils::matMulNxNScalar(rho_y_s, rho);
        Utils::matSubNxN(identity, rho_y_s, second_bracket);
        Utils::matMulNxN(hessian, second_bracket, intermediate_prod, u_size);
        Utils::matMulNxN(first_bracket, intermediate_prod, first_term, u_size);
        Utils::outerVecMulNxN(s, s, rho_s_s, u_size);
        Utils::matMulNxNScalar(rho_s_s, rho);
        Utils::matAddNxN(first_term, rho_s_s, hessian);
    }
}

void Optimiser::calculateGradient(
        const std::vector<float>& u,
        float h,
        CostFunction calculateCost,
        float current_cost,
        std::vector<float>& gradient)
{
    std::vector<float> new_u(u);
    for ( size_t i = 0; i < u.size(); i++ )
    {
        new_u[i] += h;
        gradient[i] = (calculateCost(new_u) - current_cost)/h;
        new_u[i] -= h;
    }
}

float Optimiser::calculateCostWithAlpha(
        const std::vector<float>& u,
        CostFunction calculateCost,
        const std::vector<float>& gradient,
        const float alpha,
        std::vector<float>& new_u)
{
    for ( size_t i = 0; i < new_u.size(); i++ )
    {
        new_u[i] = u[i] - (alpha * gradient[i]);
    }
    return calculateCost(new_u);
}

float Optimiser::calculateOptimalAlphaQN(
        const std::vector<float>& u,
        CostFunction calculateCost,
        float current_cost,
        const std::vector<float>& gradient,
        float smallest_alpha,
        float cost_threshold)
{
    std::vector<float> new_u(u.size());

    float new_cost = Optimiser::calculateCostWithAlpha(
            u, calculateCost, gradient, 1.0f, new_u);
    if ( new_cost < current_cost )
    {
        return 1.0f;
    }

    return Optimiser::calculateOptimalAlpha(
            u, calculateCost, current_cost, gradient,
            smallest_alpha, 1.0f, cost_threshold);

}

float Optimiser::calculateOptimalAlpha(
        const std::vector<float>& u,
        CostFunction calculateCost,
        float current_cost,
        const std::vector<float>& gradient,
        float starting_alpha,
        float ending_alpha,
        float cost_threshold)
{
    float small_alpha = 0.0f;
    float big_alpha = ending_alpha;
    float small_alpha_cost = current_cost;
    float big_alpha_cost = 0.0f;
    std::vector<float> new_u(u.size());

    /* bracketing */
    for ( float alpha = starting_alpha; alpha <= ending_alpha; alpha *= 10.0f )
    {
        float new_cost = Optimiser::calculateCostWithAlpha(
                u, calculateCost, gradient, alpha, new_u);
        // std::cout << "bracketing " << alpha << " " << new_cost << std::endl;
        if ( new_cost > small_alpha_cost )
        {
            big_alpha = alpha;
            big_alpha_cost = new_cost;
            break;
        }

        float delta_alpha = alpha * 0.1f;
        float alpha_gradient_cost = Optimiser::calculateCostWithAlpha(
                u, calculateCost, gradient, alpha+delta_alpha, new_u);
        float alpha_gradient = (alpha_gradient_cost-new_cost)/delta_alpha;

        if ( alpha_gradient > 0.0f )
        {
            big_alpha = alpha;
            big_alpha_cost = new_cost;
            break;
        }

        small_alpha = alpha;
        small_alpha_cost = new_cost;
    }

    if ( big_alpha == starting_alpha )
    {
        return 0.0f;
    }

    if ( std::fabs(small_alpha - ending_alpha) < 1e-8 )
    {
        return small_alpha;
    }

    float alpha_threshold = starting_alpha;

    /* selection (with limited iteration) */
    for ( size_t itr_num = 0; itr_num < 10; itr_num++ )
    {
        float alpha = (small_alpha + big_alpha) / 2;
        float new_cost = Optimiser::calculateCostWithAlpha(
                u, calculateCost, gradient, alpha, new_u);

        // std::cout << "selection " << alpha << " " << new_cost << std::endl;
        if ( new_cost > small_alpha_cost )
        {
            big_alpha = alpha;
            big_alpha_cost = new_cost;
        }
        else
        {
            float delta_alpha = alpha * 0.1f;
            float alpha_gradient_cost = Optimiser::calculateCostWithAlpha(
                    u, calculateCost, gradient, alpha+delta_alpha, new_u);
            float alpha_gradient = (alpha_gradient_cost-new_cost)/delta_alpha;
            if ( alpha_gradient > 0.0f )
            {
                big_alpha = alpha;
                big_alpha_cost = new_cost;
            }
            else
            {
                small_alpha = alpha;
                small_alpha_cost = new_cost;
            }
        }

        if ( big_alpha - small_alpha < alpha_threshold || 
             std::fabs(big_alpha_cost - small_alpha_cost) < cost_threshold )
        {
            break;
        }
    }
    return small_alpha;
}

} // namespace cabin
