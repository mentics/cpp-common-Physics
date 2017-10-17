#pragma once

// All dv and dp assume that the source is at rest at the origin and the target is somewhere else and moving.
// Therefore, don't use dv and dp, rather use vt0 and pt0.

#include <Eigen/Dense>
#include <random>

std::default_random_engine generator;
std::uniform_real_distribution<double> distribution(-1, 1);
auto nextDouble = std::bind(distribution, generator);

template <int SIZE>
using vector = Eigen::Matrix<double, SIZE, 1>;
using vect3 = Eigen::Vector3d;
using vect8 = vector<8>;
using mat3x8 = Eigen::Matrix<double, 3, 8>;
