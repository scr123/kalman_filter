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
 * @file ukf.cpp
 * @brief Derived class for Kalman Filter library.
 * @author Sean Crutchlow <sean.GH1231@gmail.com>
 * @version 1.0
 */

#include <kalman_filter/ukf.h>


/*
 *  TODO(Sean): Implement UKF:
 *  
 *  1. Compute Set of Sigma Points
 *  2. Assign Weights to each sigma point
 *  3. Transform the points through non linear function
 *  4. Compute Gaussian from weighted and transformed points
 *  5. Compute Mean and Variance of the new Gaussian.
 *
 *	λ  -> Scaling Factor
 *	Σ  -> Covariance Matrix
 *	Σ′ -> Predicted Covariance
 *	g  -> Non Linear Function
 *	h  -> Mapping Function between Sigma Points and Measurement Space
 *	K  -> Kalman Gain
 *	n  -> Dimentionality of System
 *	Q  -> Noise
 *	S  -> Predicted Covariance Matrix
 *	T  -> Cross Correlation Matrix for State and Predicted Spaces
 *	μ  -> Mean of the Gaussian
 *	μ′ -> Predicted Mean
 *	w  -> Simga Point Weights
 * 	χ  -> Sigma Points Matrix
 *	z  -> Measurement
 *	ẑ  -> Mean in Measurement Space
 *	Z  -> Transformed Sigma Points in Measurement Space
 *
 */

namespace kalman_filter {
/*
  UKF::UKF(const vec_t& _x, const mat_t& _P, const double _dt) {
    assert((_x.rows() == 6 && _x.cols() == 6,
            "State Vector [x]: Expect 6 rows & 1 column (x,y,vx,vy,ax,ay)"));
    x_ = _x;
    assert((_P.rows() == 6 && _P.cols() == 6,
            "Covariance Matrix [P]: Expect 6 rows & 6 columns"));
    P_ = _P;

    dt_ = _dt;

    is_first_iteration_ = true;
  }

  /// Need empty body for V-Table
  UKF::~UKF() {}

  void UKF::Predict(const mat_t& _Q, double _timestamp) {
    /// Check for first iteration

    /// Time Delta

    /// Update previous timestamp

    /// Normalized Mean - u_norm

    /// Prior State - x0

    /// Current State - x

    //// Normalized Sigma - E_norm
  }

  void UKF::Update(const vec_t& _z,
                   const mat_t& _H,
                   const mat_t& _R) {
  	/// Measurement - z

  	/// Weighted Measurement - z_hat

  	/// Residual Covariance -s

  	/// Normalized Sigma - E_norm

  	/// Kalman Gain - K

  	/// Mean - u

  	/// Sigma - E
  }
 */
}   // namespace kalman_filter
