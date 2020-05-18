/**
 * MIT License
 *
 * Copyright (c) 2019 Sean Crutchlow
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
 * @file utils_test.cpp
 * @brief Unit tests using Google Test framework to validate class
 *  implementation.
 * @author Sean Crutchlow <sean.GH1231@gmail.com>
 * @version 1.0
 */

/// System
#include <gtest/gtest.h>

#include <cmath>
#include <tuple>
#include <type_traits>
#include <vector>

/// Library
#include <Eigen/Dense>

/// Project
#include "kalman_filter/utils.h"

namespace kalman_filter {
class UtilsTest : public ::testing::Test {
 protected:
  double error;

  virtual void SetUp() {
    error = 1e-6;
  }

  virtual void TearDown() {}

  const bool Compare(const coord2D_t& _lhs,
                     const coord2D_t& _rhs,
                     const double _epsilon) {
    if (std::abs(std::get<0>(_lhs) - std::get<0>(_rhs)) > _epsilon)
      return false;
    if (std::abs(std::get<1>(_lhs) - std::get<1>(_rhs)) > _epsilon)
      return false;

    return true;
  }

  const bool Compare(const coord3D_t& _lhs,
                     const coord3D_t& _rhs,
                     const double _epsilon) {
    if (std::abs(std::get<0>(_lhs) - std::get<0>(_rhs)) > _epsilon)
      return false;
    if (std::abs(std::get<1>(_lhs) - std::get<1>(_rhs)) > _epsilon)
      return false;
    if (std::abs(std::get<2>(_lhs) - std::get<2>(_rhs)) > _epsilon)
      return false;

    return true;
  }
};

TEST_F(UtilsTest, Cartesian2Polar) {
  coord2D_t expected(5.0, 0.92729521800161);     /// r, theta
  coord2D_t result = Cartesian2Polar(3.0, 4.0);  /// x, y

  EXPECT_TRUE(Compare(result, expected, error));
}

TEST_F(UtilsTest, Polar2Cartesian) {
  coord2D_t expected(-2.0807341827357, 4.5464871341284);  /// x, y
  coord2D_t result = Polar2Cartesian(5.0, 2.0);           /// r, theta

  EXPECT_TRUE(Compare(result, expected, error));
}

TEST_F(UtilsTest, Cartesian2Spherical) {
  coord3D_t expected(7.6426435217142,
                     0.74847023342654,
                     0.90806681890191);                     /// rho, theta, phi
  coord3D_t result = Cartesian2Spherical(3.2, 4.1, 5.6);    /// x, y, z

  EXPECT_TRUE(Compare(result, expected, error));
}

TEST_F(UtilsTest, Cartesian2Cylindrical) {
  coord3D_t expected(5.3413481444295,
                     0.90482708941579,
                     5.7);                                    /// rho, theta, z
  coord3D_t result = Cartesian2Cylindrical(3.3, 4.2, 5.7);    /// x, y, z

  EXPECT_TRUE(Compare(result, expected, error));
}

TEST_F(UtilsTest, Spherical2Cartesian) {
  coord3D_t expected(-6.352477916,
                      0.9055237668,
                      3.265892074);                         /// x, y, z
  coord3D_t result = Spherical2Cartesian(7.2, 3.0, 1.1);    /// rho, theta, phi

  EXPECT_TRUE(Compare(result, expected, error));
}

TEST_F(UtilsTest, Cylindrical2Cartesian) {
  coord3D_t expected(-2.802096119,
                      2.566760086,
                      4.8);                                 /// x, y, z
  coord3D_t result = Cylindrical2Cartesian(3.8, 2.4, 4.8);  /// r, theta, z

  EXPECT_TRUE(Compare(result, expected, error));
}


TEST_F(UtilsTest, Deg2Rad) {
  double expected = 0.021537363;   /// radians
  double result = Deg2Rad(1.234);  /// degrees

  EXPECT_NEAR(result, expected, error);
}

TEST_F(UtilsTest, Rad2Deg) {
  double expected = 70.702992;     /// degrees
  double result = Rad2Deg(1.234);  /// radians

  EXPECT_NEAR(result, expected, error);
}

TEST_F(UtilsTest, Feet2Meters) {
  double expected = 0.3761232;         /// meters
  double result = Feet2Meters(1.234);  /// feet

  EXPECT_NEAR(result, expected, error);
}

TEST_F(UtilsTest, Meters2Feet) {
  double expected = 4.0485564;         /// feet
  double result = Meters2Feet(1.234);  /// meters

  EXPECT_NEAR(result, expected, error);
}

TEST_F(UtilsTest, CalcRMSE) {
  Eigen::VectorXd e1(2), e2(2), a1(2), a2(2);
  e1 << 0.00001234, 0.12340000;
  e2 << 0.00999900, 0.99990000;
  a1 << 0.00987600, 0.12000000;
  a2 << 0.00001200, 0.98760000;


  std::vector<Eigen::VectorXd> estimated = {e1, e2};
  std::vector<Eigen::VectorXd> actual = {a1, a2};

  Eigen::VectorXd expected(2);
  expected << 0.009925522,
              0.00902358;

  Eigen::VectorXd result = CalcRMSE(estimated, actual);  /// meters

  EXPECT_TRUE((result - expected).norm() < error);
}
}   //  namespace kalman_filter

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
