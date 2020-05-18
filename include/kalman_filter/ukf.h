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
 * @file ukf.h
 * @brief Derived class for Kalman Filter library.
 * @author Sean Crutchlow <sean.GH1231@gmail.com>
 * @version 1.0
 */

# ifndef UKF_H_
# define UKF_H_

#include <kalman_filter/kf.h>

namespace kalman_filter {
class UKF : public KF {
 public:
  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  _x     The state vector
   * @param[in]  _P     The covariance matrix
   * @param[in]  _dt    The time delta
   */
  UKF(const vec_t& _x,
      const mat_t& _P,
      const double _dt = 0.0);

  /**
   * @brief      Destroys the object.
   */
  ~UKF();

  /**
   * @brief      Prediction step.
   *
   * @param[in]  _Q           The state covariance matrix
   * @param[in]  _timestamp   The time
   */
  void Predict(const mat_t& _Q, double _timestamp);

  /**
   * @brief      The update step.
   *
   * @param[in]  _z    The measurement vector
   * @param[in]  _H    The measurement matrix
   * @param[in]  _R    The measurement covariance matrix
   */
  void Update(const vec_t& _z,
              const mat_t& _H,
              const mat_t& _R = Eigen::MatrixXd::Identity(2, 2) * 0.1);
};
}   // namespace kalman_filter

#endif // UKF_H_