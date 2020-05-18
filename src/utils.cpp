/**
 * MIT License
 *
 * Copyright (c) 2020 Sean Crutchlow
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file utils.cpp
 * @brief Utility for converting sensor measurements.
 * @author Sean Crutchlow <sean.GH1231@gmail.com>
 * @version 1.0
 */

#include "kalman_filter/utils.h"


namespace kalman_filter {

const coord2D_t Cartesian2Polar(const double _x, const double _y) {
  double r = sqrt(std::pow(_x, 2.0) + std::pow(_y, 2.0));
  double theta = atan2(_y, _x);

  return coord2D_t(r, theta);
}

const coord2D_t Polar2Cartesian(const double _r, const double _theta) {
  double x = _r * cos(_theta);
  double y = _r * sin(_theta);

  return coord2D_t(x, y);
}

const coord3D_t Cartesian2Spherical(const double _x,
                                    const double _y,
                                    const double _z) {
  double rho = sqrt(std::pow(_x, 2.0) + std::pow(_y, 2.0) + std::pow(_z, 2.0));
  double theta = atan2(sqrt(std::pow(_x, 2.0) + std::pow(_y, 2.0)), _z);
  double phi = atan2(_y, _x);

  return coord3D_t(rho, theta, phi);
}

const coord3D_t Cartesian2Cylindrical(const double _x,
                                      const double _y,
                                      const double _z) {
  double r = sqrt(std::pow(_x, 2.0) + std::pow(_y, 2.0));
  double theta = atan2(_y, _x);
  double z = _z;

  return coord3D_t(r, theta, z);
}

const coord3D_t Spherical2Cartesian(const double _rho,
                                    const double _theta,
                                    const double _phi) {
  double x = _rho * sin(_phi) * cos(_theta);
  double y = _rho * sin(_phi) * sin(_theta);
  double z = _rho * cos(_phi);

  return coord3D_t(x, y, z);
}

const coord3D_t Cylindrical2Cartesian(const double _r,
                                      const double _theta,
                                      const double _z) {
  double x = _r * cos(_theta);
  double y = _r * sin(_theta);
  double z = _z;

  return coord3D_t(x, y, z);
}

const double Deg2Rad(const double _deg) {
  return ((_deg * M_PI) / 180.0);
}

const double Rad2Deg(const double _rad) {
  return ((_rad * 180.0) / M_PI);
}

const double Feet2Meters(const double _ft) {
  return (_ft * 0.3048);
}

const double Meters2Feet(const double _m) {
  return (_m * 3.28084);
}

const Eigen::VectorXd CalcRMSE(const std::vector<Eigen::VectorXd> _estimate,
                               const std::vector<Eigen::VectorXd> _actual) {
  /// Assert both std::vectors are equal length
  assert(_estimate.size() == _actual.size());
  /// Assert both std::vectors are non-zero
  assert(_estimate.size() != 0);

  /// Assert both Eigen::vectors are equal length
  assert(_estimate[0].rows() == _actual[0].rows()
          && _estimate[0].cols() == _actual[0].cols());

  /// Store size n
  size_t n = _estimate.size();

  /// Set default RMSE
  Eigen::VectorXd rmse = Eigen::VectorXd::Zero(_estimate[0].size());

  /// Calculate sum of squared error
  for (int i = 0; i < n; i++) {
    Eigen::VectorXd error = _estimate[i] - _actual[i];
    error = error.array() * error.array();
    rmse += error;
  }

  /// Normalize error and take square root
  rmse /= n;
  rmse = rmse.array().sqrt();

  return rmse;
}
}   // namespace kalman_filter
