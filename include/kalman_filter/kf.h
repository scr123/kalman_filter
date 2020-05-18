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
 * @file kf.h
 * @brief Base class for Kalman Filter library.
 * @author Sean Crutchlow <sean.GH1231@gmail.com>
 * @version 1.0
 */

#ifndef KF_H_
#define KF_H_

/// System
#include <cassert>
#include <cmath>
#include <iostream>
#include <ostream>

/// Library
#include <Eigen/Dense>

namespace kalman_filter {
class KF {
 public:
  /**
    * Typedef for vector type
    */
  typedef Eigen::VectorXd vec_t;
  /**
    * Typedef for matrix type
    */
  typedef Eigen::MatrixXd mat_t;

  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  _x     The state vector
   * @param[in]  _P     The covariance matrix
   * @param[in]  _u     The control vector
   * @param[in]  _dt    The time delta
   */
  KF(const vec_t& _x,
     const mat_t& _P,
     const vec_t& _u,
     const double _dt = 0.0);

  /**
   * @brief      Destroys the object.
   */
  virtual ~KF();

  /**
   * @brief      Prediction step.
   *
   * @param[in]  _Q           The state covariance matrix
   * @param[in]  _timestamp   The time
   */
  virtual void Predict(const mat_t& _Q, int64_t _timestamp);

  /**
   * @brief      The update step.
   *
   * @param[in]  _z    The measurement vector
   * @param[in]  _H    The measurement matrix
   * @param[in]  _R    The measurement covariance matrix
   */
  virtual void Update(const vec_t& _z,
                      const mat_t& _H,
                      const mat_t& _R = Eigen::MatrixXd::Identity(2, 2) * 0.1);

  /**
   * @brief      Gets the state.
   *
   * @return     The state.
   */
  virtual vec_t GetState();

  /**
  * @brief      Output operator.
  *
  * @param      _stream  The stream
  * @param[in]  _node    The Kalman Filter object.
  *
  * @return     The output string
  */
  friend std::ostream& operator<<(std::ostream& _stream, const KF& _node);

 protected:
  virtual void Print(std::ostream& _stream) const;

  /// Matrices
  mat_t P_, F_;
  /// Vectors
  vec_t x_, u_;
  /// Timestamp
  int64_t prev_timestamp_;
  /// Time Delta
  double dt_;
  /// Flag
  bool is_first_iteration_;
};
}   // namespace kalman_filter

#endif  // KF_H_
