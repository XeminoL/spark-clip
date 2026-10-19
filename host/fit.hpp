#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include "linalg.hpp"

template <std::size_t N>
struct FitResult {
    Vec<N> parameters;
    Mat<N> covariance;
    double chi_square = 0.0;
    int residual_count = 0;
    int iterations = 0;
    bool converged = false;

    double sigma(std::size_t i) const { return std::sqrt(std::max(0.0, covariance(i, i))); }

};

template <std::size_t N, class Residuals>
std::vector<std::vector<double>> jacobian_columns(Residuals& residuals, const Vec<N>& at, const std::vector<double>& base, const Vec<N>& steps) {
    std::vector<std::vector<double>> columns(N);
    for (std::size_t i = 0; i < N; ++i) {
        Vec<N> shifted = at;
        shifted[i] += steps[i];
        std::vector<double> moved = residuals(shifted);
        columns[i].resize(base.size());
        for (std::size_t k = 0; k < base.size(); ++k) columns[i][k] = (moved[k] - base[k]) / steps[i];
    }
    return columns;
}

inline double sum_of_squares(const std::vector<double>& values) {
    double sum = 0.0;
    for (double value : values) sum += value * value;
    return sum;
}

template <std::size_t N>
Mat<N> normal_matrix(const std::vector<std::vector<double>>& columns) {
    Mat<N> matrix;
    for (std::size_t r = 0; r < N; ++r)
        for (std::size_t c = 0; c < N; ++c)
            for (std::size_t k = 0; k < columns[r].size(); ++k) matrix(r, c) += columns[r][k] * columns[c][k];
    return matrix;
}

template <std::size_t N>
Mat<N> with_ridge(Mat<N> matrix, double share) {
    double largest = 0.0;
    for (std::size_t i = 0; i < N; ++i) largest = std::max(largest, matrix(i, i));
    for (std::size_t i = 0; i < N; ++i) matrix(i, i) += share * std::max(largest, 1e-300);
    return matrix;
}

template <std::size_t N, class Residuals>
FitResult<N> levenberg_marquardt(Residuals residuals, Vec<N> start, const Vec<N>& steps, int max_iterations) {
    const double kRelativeStop = 1e-9;
    const double kRidge = 1e-12;
    const double kUnknownVariance = 1e30;
    FitResult<N> result;
    Vec<N> at = start;
    std::vector<double> base = residuals(at);
    double cost = sum_of_squares(base);
    double damping = 1e-3;
    for (int iteration = 0; iteration < max_iterations; ++iteration) {
        result.iterations = iteration + 1;
        std::vector<std::vector<double>> columns = jacobian_columns<N>(residuals, at, base, steps);
        Mat<N> normal = normal_matrix<N>(columns);
        Vec<N> gradient;
        for (std::size_t i = 0; i < N; ++i)
            for (std::size_t k = 0; k < base.size(); ++k) gradient[i] += columns[i][k] * base[k];
        bool improved = false;
        for (int attempt = 0; attempt < 12 && !improved; ++attempt) {
            Mat<N> damped = normal;
            for (std::size_t i = 0; i < N; ++i) damped(i, i) *= 1 + damping;
            Vec<N> step;
            try {
                step = solve(with_ridge(damped, kRidge), gradient) * -1.0;
            } catch (const std::runtime_error&) {
                damping *= 8;
                continue;
            }
            Vec<N> trial = at + step;
            std::vector<double> trial_residuals = residuals(trial);
            double trial_cost = sum_of_squares(trial_residuals);
            if (std::isfinite(trial_cost) && trial_cost < cost) {
                double drop = (cost - trial_cost) / std::max(cost, 1e-300);
                at = trial;
                base = trial_residuals;
                cost = trial_cost;
                damping = std::max(damping / 5, 1e-9);
                improved = true;
                if (drop < kRelativeStop) result.converged = true;
            } else {
                damping *= 8;
            }
        }
        if (!improved) result.converged = true;
        if (result.converged) break;
    }
    std::vector<std::vector<double>> columns = jacobian_columns<N>(residuals, at, base, steps);
    Mat<N> normal = normal_matrix<N>(columns);
    result.parameters = at;
    result.chi_square = cost;
    result.residual_count = static_cast<int>(base.size());
    double dof = std::max(1, result.residual_count - static_cast<int>(N));
    double inflation = std::max(1.0, cost / dof);
    try {
        result.covariance = inverse(with_ridge(normal, kRidge));
    } catch (const std::runtime_error&) {
        result.covariance = Mat<N>::identity();
        for (std::size_t i = 0; i < N; ++i) result.covariance(i, i) = kUnknownVariance;
    }
    for (std::size_t r = 0; r < N; ++r)
        for (std::size_t c = 0; c < N; ++c) result.covariance(r, c) *= inflation;
    return result;
}