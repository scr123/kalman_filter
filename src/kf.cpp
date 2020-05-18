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
 * @file kf.cpp
 * @brief Base class for Kalman Filter library.
 * @author Sean Crutchlow <sean.GH1231@gmail.com>
 * @version 1.0
 */

#include <kalman_filter/kf.h>

/// Definition for DEBUG statements
#define DEBUG(x) do { if (false) { std::cerr << x << std::endl; } } while (0)

namespace kalman_filter {
  KF::KF(const vec_t& _x, const mat_t& _P, const vec_t& _u, const double _dt) {
    assert((_x.rows() == 4 && _x.cols() == 1,
            "State Vector [x]: Expect 4 rows & 1 column (x,y,vx,vy)"));
    x_ = _x;
    assert((_P.rows() == 4 && _P.cols() == 4,
            "Covariance Matrix [P]: Expect 4 rows & 4 columns"));
    P_ = _P;
    assert((_u.rows() == 4 && _u.cols() == 1,
            "Control Vector [u]: Expect 4 rows & 1 column (x,y,vx,vy)"));
    u_ = _u;

    /// Set dt if constant discrete time delta is used
    dt_ = _dt;

    /// If constant discrete time delta is not used, set dt
    /// to 1.0 for state transition matrix
    double dt = std::fabs(_dt) > 1e-8 ? _dt : 1.0;

    /// Construct temporary state transition matrix
    mat_t F = mat_t(4, 4);
    F <<  1.0, 0.0,  dt, 0.0,
          0.0, 1.0, 0.0,  dt,
          0.0, 0.0, 1.0, 0.0,
          0.0, 0.0, 0.0, 1.0;

    /// Update State Transition Matrix
    F_ = F;

    /// Set first iteration flag
    is_first_iteration_ = true;
  }

  /// Need empty body for V-Table
  KF::~KF() {}

  void KF::Predict(const mat_t& _Q, int64_t _timestamp) {
    /// Check for first iteration
    if (is_first_iteration_ == true) {
      /// Update previous timestamp
      prev_timestamp_ = _timestamp;
      /// Unset flag
      is_first_iteration_ = false;
      /// Return, while apriori information is needed for prediction
      return;
    }

    /// Time Delta
    double dt = std::fabs(dt_) > 1e-8 ? dt_ :
                  (static_cast<double>(_timestamp - prev_timestamp_) / 1e6);

    /// Update previous timestamp
    prev_timestamp_ = _timestamp;

    /// Update State Transition
    mat_t F = F_ * dt;

    /// Debug Statement
    DEBUG("\nF:\n" << F << "\n\nF_:\n" << F_
                   << "\n\ndt: " << dt << "\n"
                   << "_timestamp: " << _timestamp << "\n"
                   << "prev_timestamp_: " << prev_timestamp_
                   << "\n\n");

    /// State
    x_ = (F * x_) + u_.matrix();

    /// Update Covariance
    P_ = (F * P_ * F.transpose()) + _Q;
  }

  void KF::Update(const vec_t& _z,
                  const mat_t& _H,
                  const mat_t& _R) {
    /// Measurement Residual
    mat_t y = _z.matrix() - (_H * x_.matrix());

    /// Debug Statement
    DEBUG("\ny:\n" << y
                   << "\n\n_z.matrix():\n" << _z.matrix()
                   << "\n\n_H:\n" << _H
                   << "\n\nx_.matrix():\n" << x_.matrix()
                   << "\n\nH * x_matrix():\n" << (_H * x_.matrix()) << "\n\n");

    /// Residual Covariance
    mat_t S = (_H * P_ * _H.transpose()) + _R;

    /// Kalman Gain
    mat_t K = P_ * _H.transpose() * S.inverse();

    /// Debug Statement
    DEBUG("\nS:\n" << S << "\n\nK:\n" << K << "\n\n");

    /// Update State
    x_ = x_ + (K * y);

    /// Identity
    mat_t I = mat_t::Identity(x_.size(), x_.size());

    /// Update Covariance
    P_ = (I - (K * _H)) * P_;
  }

  KF::vec_t KF::GetState() {
    return x_;
  }

  void KF::Print(std::ostream& _stream) const {
    _stream << "--- KF Object ---\n";

    _stream << "Covariance Matrix - P("
            << P_.rows() << ", "
            << P_.cols() << "):\n";
    _stream << P_ << "\n\n";

    _stream << "State Transition Matrix - F("
            << F_.rows() << ", "
            << F_.cols() << "):\n";
    _stream << F_ << "\n\n";

    _stream << "State Vector - x("
            << x_.rows() << ", "
            << x_.cols() << "):\n";
    _stream << x_ << "\n\n";

    _stream << "------------\n\n";
  }

  std::ostream& operator<<(std::ostream& _stream, const KF& _kf) {
    _kf.Print(_stream);
    return _stream;
  }
}   // namespace kalman_filter
