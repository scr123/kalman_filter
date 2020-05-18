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
 * @file ekf.cpp
 * @brief Derived class for Kalman Filter library.
 * @author Sean Crutchlow <sean.GH1231@gmail.com>
 * @version 1.0
 */

#include <kalman_filter/ekf.h>

namespace kalman_filter {
  EKF::EKF(const vec_t& _x, const mat_t& _P, const vec_t& _u, const double _dt)
    : KF(_x, _P, _u, _dt) {
    assert((_x.rows() == 6 && _x.cols() == 6,
            "State Vector [x]: Expect 6 rows & 1 column (x,y,vx,vy,ax,ay)"));
    x_ = _x;
    assert((_P.rows() == 6 && _P.cols() == 6,
            "Covariance Matrix [P]: Expect 6 rows & 6 columns"));
    P_ = _P;
    assert((_u.rows() == 6 && _u.cols() == 1,
            "Control Vector [u]: Expect 6 rows & 1 column (x,y,vx,vy,ax,ay)"));
    u_ = _u;

    /// Set dt if constant discrete time delta is used
    dt_ = _dt;

    /// If constant discrete time delta is not used, set dt
    /// to 1.0 for state transition matrix
    double dt = std::fabs(_dt) > 1e-8 ? _dt : 1.0;

    /// Calculate derivate for dt
    double dt_prime = std::pow(0.5 * dt, 2.0);

    /// Construct temporary state transition matrix
    mat_t F = mat_t(6, 6);
    F  << 1.0, 0.0,  dt, 0.0, dt_prime,      0.0,
          0.0, 1.0, 0.0,  dt,      0.0, dt_prime,
          0.0, 0.0, 1.0, 0.0,       dt,      0.0,
          0.0, 0.0, 0.0, 1.0,      0.0,       dt,
          0.0, 0.0, 0.0, 0.0,      1.0,      0.0,
          0.0, 0.0, 0.0, 0.0,      0.0,      1.0;

    /// Update State Transition Matrix
    F_ = F;

    /// Set first iteration flag
    is_first_iteration_ = true;
  }

  /// Need empty body for V-Table
  EKF::~EKF() {}
}   // namespace kalman_filter
