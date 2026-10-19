#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>

template <std::size_t N>
struct Vec {
    std::array<double, N> items{};

    double& operator[](std::size_t i) { return items[i]; }
    double operator[](std::size_t i) const { return items[i]; }

    Vec& operator+=(const Vec& other) {
        for (std::size_t i = 0; i < N; ++i) items[i] += other.items[i];
        return *this;
    }
    Vec& operator-=(const Vec& other) {
        for (std::size_t i = 0; i < N; ++i) items[i] -= other.items[i];
        return *this;
    }
    Vec& operator*=(double factor) {
        for (double& item : items) item *= factor;
        return *this;
    }
};

template <std::size_t N>
Vec<N> operator+(Vec<N> left, const Vec<N>& right) { return left += right; }

template <std::size_t N>
Vec<N> operator-(Vec<N> left, const Vec<N>& right) { return left -= right; }

template <std::size_t N>
Vec<N> operator*(Vec<N> vector, double factor) { return vector *= factor; }

template <std::size_t N>
Vec<N> operator*(double factor, Vec<N> vector) { return vector *= factor; }

template <std::size_t N>
double dot(const Vec<N>& left, const Vec<N>& right) {
    double sum = 0.0;
    for (std::size_t i = 0; i < N; ++i) sum += left[i] * right[i];
    return sum;
}

template <std::size_t N>
struct Mat {
    std::array<std::array<double, N>, N> cells{};

    double& operator()(std::size_t row, std::size_t column) { return cells[row][column]; }
    double operator()(std::size_t row, std::size_t column) const { return cells[row][column]; }

    static Mat identity() {
        Mat result;
        for (std::size_t i = 0; i < N; ++i) result(i, i) = 1.0;
        return result;
    }
};

template <std::size_t N>
Vec<N> operator*(const Mat<N>& matrix, const Vec<N>& vector) {
    Vec<N> result;
    for (std::size_t r = 0; r < N; ++r)
        for (std::size_t c = 0; c < N; ++c) result[r] += matrix(r, c) * vector[c];
    return result;
}

template <std::size_t N>
Mat<N> inverse(Mat<N> matrix) {
    Mat<N> result = Mat<N>::identity();
    for (std::size_t column = 0; column < N; ++column) {
        std::size_t pivot = column;
        for (std::size_t row = column + 1; row < N; ++row)
            if (std::fabs(matrix(row, column)) > std::fabs(matrix(pivot, column))) pivot = row;
        if (std::fabs(matrix(pivot, column)) < 1e-300) throw std::runtime_error("singular matrix");
        std::swap(matrix.cells[pivot], matrix.cells[column]);
        std::swap(result.cells[pivot], result.cells[column]);
        double scale = matrix(column, column);
        for (std::size_t c = 0; c < N; ++c) {
            matrix(column, c) /= scale;
            result(column, c) /= scale;
        }
        for (std::size_t row = 0; row < N; ++row) {
            if (row == column) continue;
            double factor = matrix(row, column);
            for (std::size_t c = 0; c < N; ++c) {
                matrix(row, c) -= factor * matrix(column, c);
                result(row, c) -= factor * result(column, c);
            }
        }
    }
    return result;
}

template <std::size_t N>
Vec<N> solve(const Mat<N>& matrix, const Vec<N>& right) { return inverse(matrix) * right; }