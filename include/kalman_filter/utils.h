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
 * @file utils.h
 * @brief Utilities for converting sensor measurements.
 * @author Sean Crutchlow <sean.GH1231@gmail.com>
 * @version 1.0
 */

#ifndef UTILS_H_
#define UTILS_H_

/// System
#include <cassert>
#include <cmath>
#include <tuple>
#include <vector>

/// Library
#include <Eigen/Dense>


namespace kalman_filter {

/**
 * Typedef for 2D and 3D coordinates.
 */
typedef std::tuple<double, double> coord2D_t;
typedef std::tuple<double, double, double> coord3D_t;

/**
 * @brief      Converts Cartesian coordinates to Polar.
 *
 * @param[in]  _x    The x position
 * @param[in]  _y    The y position
 *
 * @return     Polar coordinates r and theta.
 */
const coord2D_t Cartesian2Polar(const double _x, const double _y);

/**
 * @brief      Converts Polar coordinates to Cartesian.
 *
 * @param[in]  _r      The radius
 * @param[in]  _theta  The theta angle
 *
 * @return     Cartesian coordinates x, y.
 */
const coord2D_t Polar2Cartesian(const double _r, const double _theta);

/**
 * @brief      Converts Cartesian coordinates to Spherical.
 *
 * @param[in]  _x    The x position
 * @param[in]  _y    The y position
 * @param[in]  _z    The z position
 *
 * @return     Spherical coordinates rho, theta, and phi.
 */
const coord3D_t Cartesian2Spherical(const double _x,
                                    const double _y,
                                    const double _z);

/**
 * @brief      Converts Cartesian coordinates to Cyclindrical
 *
 * @param[in]  _x    The x position
 * @param[in]  _y    The y position
 * @param[in]  _z    The z position
 *
 * @return     Cylindrical coordinates r, theta, and z.
 */
const coord3D_t Cartesian2Cylindrical(const double _x,
                                      const double _y,
                                      const double _z);

/**
 * @brief      Converts Spherical coordinates to Cartesian.
 *
 * @param[in]  _rho    The rho angle
 * @param[in]  _theta  The theta angle
 * @param[in]  _phi    The phi angle
 *
 * @return     Cartesian coordinates x, y, and z.
 */
const coord3D_t Spherical2Cartesian(const double _rho,
                                    const double _theta,
                                    const double _phi);

/**
 * @brief      Converts Cylindrical coordinates to Cartesian.
 *
 * @param[in]  _r      The radius
 * @param[in]  _theta  The theta angle
 * @param[in]  _z      The z position
 *
 * @return     Cartesian coordinates x, y, and z.
 */
const coord3D_t Cylindrical2Cartesian(const double _r,
                                      const double _theta,
                                      const double _z);

/**
 * @brief      Converts Degrees to Radians.
 *
 * @param[in]  _deg  The degrees
 *
 * @return     The radians.
 */
const double Deg2Rad(const double _deg);

/**
 * @brief      Converts Radians to Degrees.
 *
 * @param[in]  _rad  The radians
 *
 * @return     The degrees.
 */
const double Rad2Deg(const double _rad);

/**
 * @brief      Converts Feet to Meters.
 *
 * @param[in]  _ft   The feet
 *
 * @return     The meters.
 */
const double Feet2Meters(const double _ft);

/**
 * @brief      Converts Meters to Feet.
 *
 * @param[in]  _ft   The meters
 *
 * @return     The feet.
 */
const double Meters2Feet(const double _m);

/**
 * @brief      Calculates the root-mean square error.
 *
 * @param[in]  _estimate  The estimate
 * @param[in]  _actual    The actual
 *
 * @return     The root-mean square error.
 */
const Eigen::VectorXd CalcRMSE(const std::vector<Eigen::VectorXd> _estimate,
                               const std::vector<Eigen::VectorXd> _actual);

}   // namespace kalman_filter

#endif  // UTILS_H_