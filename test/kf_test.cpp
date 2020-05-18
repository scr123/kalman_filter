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
 * @file kf_test.cpp
 * @brief Unit tests using Google Test framework to validate class
 *  implementation.
 * @author Sean Crutchlow <sean.GH1231@gmail.com>
 * @version 1.0
 */

/// System
#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>


/// Library
#include <Eigen/Dense>

/// Project
#include "kalman_filter/kf.h"
#include "kalman_filter/ekf.h"
#include "kalman_filter/ukf.h"
#include "kalman_filter/utils.h"

namespace kalman_filter {
class KfTest : public ::testing::Test {
 protected:
  /// Pointer for Kalman Filter object
  std::shared_ptr<KF> kf_ptr;
  /// Pointers for Eigen Matrices
  Eigen::MatrixXd *kf_H_lidar_ptr;
  Eigen::MatrixXd *kf_H_radar_ptr;
  Eigen::MatrixXd *kf_R_lidar_ptr;
  Eigen::MatrixXd *kf_R_radar_ptr;
  Eigen::MatrixXd *kf_Q_ptr;

  /// Pointer for Extended Kalman Filter object
  std::shared_ptr<EKF> ekf_ptr;
  /// Pointers for Eigen Matrices
  Eigen::MatrixXd *ekf_H_lidar_ptr;
  Eigen::MatrixXd *ekf_H_radar_ptr;
  Eigen::MatrixXd *ekf_R_lidar_ptr;
  Eigen::MatrixXd *ekf_R_radar_ptr;
  Eigen::MatrixXd *ekf_Q_ptr;

  /// File names for input data
  std::string f_lidar_radar;

  /// Generic structure for sensor measurements
  struct Measurement {
    std::string type;
    std::vector<double> data;
    double gt_x, gt_y, gt_vx, gt_vy;
    int64_t timestamp;
  };

  virtual void SetUp() {
    /// Files containing measurement data
    f_lidar_radar = "lidar_and_radar.csv";

    /// state vector - x
    Eigen::VectorXd kf_x(4);
    // covariance matrix - P
    Eigen::MatrixXd kf_P(4, 4);
    /// measurement matrix - H
    kf_H_lidar_ptr = new Eigen::MatrixXd(2, 4);
    kf_H_radar_ptr = new Eigen::MatrixXd(2, 4);
    /// measurement covariance matrix - R
    kf_R_lidar_ptr = new Eigen::MatrixXd(2, 2);
    kf_R_radar_ptr = new Eigen::MatrixXd(2, 2);
    /// state noise covariance matrix - Q
    kf_Q_ptr = new Eigen::MatrixXd(4, 4);
    /// control vector - u
    Eigen::VectorXd kf_u(4);

    /// Initialize Kalman Filter Variables
    InitKfVars(&kf_x,
               &kf_P,
               kf_H_lidar_ptr,
               kf_H_radar_ptr,
               kf_R_lidar_ptr,
               kf_R_radar_ptr,
               kf_Q_ptr,
               &kf_u);

    /// Create Kalman Filter Object
    kf_ptr = std::make_shared<KF>(kf_x, kf_P, kf_u);

    /// state vector - x
    Eigen::VectorXd ekf_x(6);
    // covariance matrix - P
    Eigen::MatrixXd ekf_P(6, 6);
    /// measurement matrix - H
    ekf_H_lidar_ptr = new Eigen::MatrixXd(2, 6);
    ekf_H_radar_ptr = new Eigen::MatrixXd(2, 6);
    /// measurement covariance matrix - R
    ekf_R_lidar_ptr = new Eigen::MatrixXd(2, 2);
    ekf_R_radar_ptr = new Eigen::MatrixXd(2, 2);
    /// state noise covariance matrix - Q
    ekf_Q_ptr = new Eigen::MatrixXd(6, 6);
    /// control vector - u
    Eigen::VectorXd ekf_u(6);

    /// Initialize Kalman Filter Variables
    InitEkfVars(&ekf_x,
               &ekf_P,
               ekf_H_lidar_ptr,
               ekf_H_radar_ptr,
               ekf_R_lidar_ptr,
               ekf_R_radar_ptr,
               ekf_Q_ptr,
               &ekf_u);

    /// Create Extended Kalman Filter Object
    ekf_ptr = std::make_shared<EKF>(ekf_x, ekf_P, ekf_u);
  }

  virtual void TearDown() {
    delete kf_H_lidar_ptr;
    delete kf_H_radar_ptr;
    delete kf_R_lidar_ptr;
    delete kf_R_radar_ptr;
    delete kf_Q_ptr;

    delete ekf_H_lidar_ptr;
    delete ekf_H_radar_ptr;
    delete ekf_R_lidar_ptr;
    delete ekf_R_radar_ptr;
    delete ekf_Q_ptr;
  }

  /**
   * @brief      Initializes the Kalman Filter variables.
   *
   * @param      _x        State vector
   * @param      _P        The covariance matrix
   * @param      _lidar_H  The lidar measurment matrix
   * @param      _radar_H  The radar measurment matrix
   * @param      _lidar_R  The lidar measurment noise matrix
   * @param      _radar_R  The radar measurment noise matrix
   * @param      _Q        The state covariance matrix
   * @param      _u        The control vector
   */
  void InitKfVars(KF::vec_t *_x,
                  KF::mat_t *_P,
                  KF::mat_t *_lidar_H,
                  KF::mat_t *_radar_H,
                  KF::mat_t *_lidar_R,
                  KF::mat_t *_radar_R,
                  KF::mat_t *_Q,
                  KF::vec_t *_u) {
    *_x << 0.0,  /// x
           0.0,  /// y
           0.0,  /// vx
           0.0;  /// vy

    *_P << 0.1, 0.0,  0.0,  0.0,  /// x
           0.0, 0.1,  0.0,  0.0,  /// y
           0.0, 0.0, 10.0,  0.0,  /// vx
           0.0, 0.0,  0.0, 10.0;  /// vy

    *_lidar_H << 1.0, 0.0, 0.0, 0.0,  /// x
                 0.0, 1.0, 0.0, 0.0;  /// y

    *_radar_H << 1.0, 0.0, 0.0, 0.0,  /// x
                 0.0, 1.0, 0.0, 0.0;  /// y

    *_lidar_R << 0.0225, 0.0,     /// x
                 0.0,    0.0225;  /// y

    *_radar_R << 0.09, 0.0,       /// x
                 0.0,  0.0009;    /// y

    *_Q << 0.1, 0.0, 0.0, 0.0,  /// x
           0.0, 0.1, 0.0, 0.0,  /// y
           0.0, 0.0, 0.1, 0.0,  /// vx
           0.0, 0.0, 0.0, 0.1;  /// vy

    *_u << 0.0,  /// x
           0.0,  /// y
           0.0,  /// vx
           0.0;  /// vy
  }

  /**
   * @brief      Initializes the Extended Kalman Filter variables.
   *
   * @param      _x        State vector
   * @param      _P        The covariance matrix
   * @param      _lidar_H  The lidar measurment matrix
   * @param      _radar_H  The radar measurment matrix
   * @param      _lidar_R  The lidar measurment noise matrix
   * @param      _radar_R  The radar measurment noise matrix
   * @param      _Q        The state covariance matrix
   * @param      _u        The control vector
   */
  void InitEkfVars(KF::vec_t *_x,
                   KF::mat_t *_P,
                   KF::mat_t *_lidar_H,
                   KF::mat_t *_radar_H,
                   KF::mat_t *_lidar_R,
                   KF::mat_t *_radar_R,
                   KF::mat_t *_Q,
                   KF::vec_t *_u) {
    *_x << 0.0,  /// x
           0.0,  /// y
           0.0,  /// vx
           0.0,  /// vy
           0.0,  /// ax
           0.0;  /// ay

    *_P << 0.1, 0.0,  0.0,   0.0,   0.0,   0.0,  /// x
           0.0, 0.1,  0.0,   0.0,   0.0,   0.0,  /// y
           0.0, 0.0, 10.0,   0.0,   0.0,   0.0,  /// vx
           0.0, 0.0,  0.0,  10.0,   0.0,   0.0,  /// vy
           0.0, 0.0,  0.0,   0.0, 100.0,   0.0,  /// ax
           0.0, 0.0,  0.0,   0.0,   0.0, 100.0;  /// ay

    *_lidar_H << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,  /// x
                 0.0, 1.0, 0.0, 0.0, 0.0, 0.0;  /// y

    *_radar_H << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,  /// x
                 0.0, 1.0, 0.0, 0.0, 0.0, 0.0;  /// y

    *_lidar_R << 0.0225, 0.0,     /// x
                 0.0,    0.0225;  /// y

    *_radar_R << 0.09, 0.0,       /// x
                 0.0,  0.0009;    /// y

    *_Q << 0.1, 0.0, 0.0, 0.0, 0.0, 0.0,   /// x
           0.0, 0.1, 0.0, 0.0, 0.0, 0.0,   /// y
           0.0, 0.0, 0.1, 0.0, 0.0, 0.0,   /// vx
           0.0, 0.0, 0.0, 0.1, 0.0, 0.0,  /// vy
           0.0, 0.0, 0.0, 0.0, 0.1, 0.0,   /// ax
           0.0, 0.0, 0.0, 0.0, 0.0, 0.1;   /// ay

    *_u << 0.0,  /// x
           0.0,  /// y
           0.0,  /// vx
           0.0,  /// vy
           0.0,  /// ax
           0.0;  /// ay
  }

  /**
   * @brief      Parses file with sensor measurements.
   *
   * @param[in]  _file  The file
   *
   * @return     Vector of sensor measurements.
   */
  std::vector<Measurement> Parse(const std::string& _file) {
    /// Store parsed measurements
    std::vector<Measurement> measurements;
    /// Parsed line
    std::string line;
    /// File handle for measurements
    std::ifstream f(_file);

    /// If file can be opened
    if (f.is_open()) {
      /// While there is a line to be retrieved from file handle
      while (getline(f, line)) {
        /// Convert string to stringstream
        std::stringstream stream(line);
        /// Define type for storing string without delimeters
        std::vector<std::string> delim_line;
        /// While there are no errors in stringstream
        while (stream.good()) {
          /// Remove delimeter and store data
          std::string substr;
          getline(stream, substr, ',');
          delim_line.push_back(substr);
        }
          Measurement measurement;
          /// If measurement is lidar, parse directly, otherwise,
          // measurement is radar, convert from polar to Cartesian & parse
          if (delim_line[0].compare("L") == 0) {
            measurement.type      = "Lidar";

            measurement.data.push_back(std::stod(delim_line[1]));  /// x
            measurement.data.push_back(std::stod(delim_line[2]));  /// y

            measurement.timestamp = std::stoll(delim_line[3]);
            measurement.gt_x      = std::stod(delim_line[4]);
            measurement.gt_y      = std::stod(delim_line[5]);
            measurement.gt_vx     = std::stod(delim_line[6]);
            measurement.gt_vy     = std::stod(delim_line[7]);
          } else {
            double rho       = std::stod(delim_line[1]);
            double phi       = std::stod(delim_line[2]);
            double rho_dot   = std::stod(delim_line[3]);

            /**
             * This is now a converted measurement Kalman Filter - Jacobian 
             * and Symbolic Differentiation can be used in other applications
             * to linearize the measurement error. For this test, the main
             * focus is library.
             */
            coord2D_t radar_cartesian = Polar2Cartesian(rho, phi);

            measurement.type      = "Radar";

            measurement.data.push_back(std::get<0>(radar_cartesian));
            measurement.data.push_back(std::get<1>(radar_cartesian));

            measurement.timestamp = std::stoll(delim_line[4]);
            measurement.gt_x      = std::stod(delim_line[5]);
            measurement.gt_y      = std::stod(delim_line[6]);
            measurement.gt_vx     = std::stod(delim_line[7]);
            measurement.gt_vy     = std::stod(delim_line[8]);
          }
          measurements.push_back(measurement);
      }
      f.close();
    }

    return measurements;
  }

  /**
   * @brief      Processes the measurements'
   *
   * @param[in]  _z         The measurement vector
   * @param[in]  _H_lidar   The lidar measurement matrix
   * @param[in]  _H_radar   The radar measurement matrix
   * @param[in]  _R_lidar   The lidar measurment noise matrix
   * @param[in]  _R_radar   The radar measurment noise matrix
   * @param[in]  _Q         The state covariance matrix
   * @param      _estimate  The estimated states
   * @param      _actual    The actual states (ground truth)
   * @param[in]  padding    The padding
   *
   * @return     The final state vector.
   */
  Eigen::VectorXd Process(const std::vector<Measurement>& _z,
                          const Eigen::MatrixXd* _H_lidar,
                          const Eigen::MatrixXd* _H_radar,
                          const Eigen::MatrixXd* _R_lidar,
                          const Eigen::MatrixXd* _R_radar,
                          const Eigen::MatrixXd* _Q,
                          std::vector<Eigen::VectorXd>* _estimate,
                          std::vector<Eigen::VectorXd>* _actual,
                          size_t padding = 0) {
    for (int i = 0; i < _z.size(); i++) {
      /// Define measurment vector based on raw measurement data
      Eigen::VectorXd measurement(
        Eigen::Map<const Eigen::VectorXd>(
          _z[i].data.data(), _z[i].data.size()));

      /// Have Kalman Filter complete a single iteration (Update & Predict)
      if (_z[i].type.compare("Lidar") == 0) {
        kf_ptr->Update(measurement,
                       *_H_lidar,
                       *_R_lidar);
      } else {
        kf_ptr->Update(measurement,
                       *_H_radar,
                       *_R_radar);
      }
      kf_ptr->Predict(*_Q, _z[i].timestamp);

      /// Populate estimated & actual states if needed for RMSE calcuation
      if (_estimate != nullptr) {
        _estimate->push_back(kf_ptr->GetState());
      }
      if (_actual != nullptr) {
        Eigen::VectorXd ground_truth(4);
        ground_truth << _z[i].gt_x,
                        _z[i].gt_y,
                        _z[i].gt_vx,
                        _z[i].gt_vy;
        /// Added padding to state vector since data set in this
        /// test is 4-dimensional
        if (padding > 0) {
            Eigen::VectorXd ground_truth_padded(4 + padding);
            Eigen::VectorXd padded = Eigen::VectorXd::Zero(padding);
            ground_truth_padded << ground_truth,
                                   padded;

          _actual->push_back(ground_truth_padded);
        } else {
          _actual->push_back(ground_truth);
        }
      }
    }
    /// Return final state
    return kf_ptr->GetState();
  }
};


TEST_F(KfTest, CMKF_RMSE) {
  /// Vectors to hold data used in RMSE calculation
  std::vector<Eigen::VectorXd> estimated_states;
  std::vector<Eigen::VectorXd> actual_states;

  /// Parse CSV to extract sensor measurements
  std::vector<Measurement> measurements = Parse(f_lidar_radar);

  /// Process measurements to estimate final state
  Eigen::VectorXd curr_state = Process(measurements,
                                       kf_H_lidar_ptr,
                                       kf_H_radar_ptr,
                                       kf_R_lidar_ptr,
                                       kf_R_radar_ptr,
                                       kf_Q_ptr,
                                       &estimated_states,
                                       &actual_states);
  /// Define allowable RMSE
  Eigen::VectorXd allowable_rmse(4);
  allowable_rmse << 1.0,
                    1.0,
                    10.0,
                    10.0;

  /// Calculate RMSE for estimated & actual state vectors
  Eigen::VectorXd calculated_rmse = CalcRMSE(estimated_states, actual_states);

  /// Check that calculated RMSE is less than allowable
  bool result = !(calculated_rmse.array() > allowable_rmse.array()).any();

  /// RMSE for state vector
  std::cerr << "\nCMKF RMSE:\n" << calculated_rmse << std::endl;

  EXPECT_TRUE(result);
}

TEST_F(KfTest, CMKF_FinalPose) {
  /// Parse CSV to extract sensor measurements
  std::vector<Measurement> measurements = Parse(f_lidar_radar);

  /// Extract expected state from final ground truth measurement
  Eigen::VectorXd expected_state(4);
  Measurement final_state = measurements[measurements.size() - 1];
  expected_state << final_state.gt_x,
                    final_state.gt_y,
                    final_state.gt_vx,
                    final_state.gt_vy;

  /// Process measurements to estimate final state
  Eigen::VectorXd curr_state = Process(measurements,
                                       kf_H_lidar_ptr,
                                       kf_H_radar_ptr,
                                       kf_R_lidar_ptr,
                                       kf_R_radar_ptr,
                                       kf_Q_ptr,
                                       nullptr,
                                       nullptr);

  /// Print out states as reference
  std::cerr << "\nCMKF curr_state:\n" << curr_state << std::endl;
  std::cerr << "\nCMKF expected_state:\n" << expected_state << std::endl;

  EXPECT_TRUE(curr_state.isApprox(expected_state));
}

TEST_F(KfTest, CMEKF_RMSE) {
  /// Vectors to hold data used in RMSE calculation
  std::vector<Eigen::VectorXd> estimated_states;
  std::vector<Eigen::VectorXd> actual_states;

  /// Parse CSV to extract sensor measurements
  std::vector<Measurement> measurements = Parse(f_lidar_radar);

  /// Set pointer to derived class
  kf_ptr = ekf_ptr;

  /// Process measurements to estimate final state
  Eigen::VectorXd curr_state = Process(measurements,
                                       ekf_H_lidar_ptr,
                                       ekf_H_radar_ptr,
                                       ekf_R_lidar_ptr,
                                       ekf_R_radar_ptr,
                                       ekf_Q_ptr,
                                       &estimated_states,
                                       &actual_states,
                                       2);
  /// Define allowable RMSE
  Eigen::VectorXd allowable_rmse(6);
  allowable_rmse << 1.0,
                    1.0,
                    10.0,
                    10.0,
                    100.0,
                    100.0;

  /// Calculate RMSE for estimated & actual state vectors
  Eigen::VectorXd calculated_rmse = CalcRMSE(estimated_states, actual_states);

  /// Check that calculated RMSE is less than allowable
  bool result = !(calculated_rmse.array() > allowable_rmse.array()).any();

  /// RMSE for state vector
  std::cerr << "\nCMEKF RMSE:\n" << calculated_rmse << std::endl;

  EXPECT_TRUE(result);
}

TEST_F(KfTest, CMEKF_FinalPose) {
  /// Parse CSV to extract sensor measurements
  std::vector<Measurement> measurements = Parse(f_lidar_radar);

  /// Extract expected state from final ground truth measurement
  Eigen::VectorXd expected_state(6);
  Measurement final_state = measurements[measurements.size() - 1];
  expected_state << final_state.gt_x,
                    final_state.gt_y,
                    final_state.gt_vx,
                    final_state.gt_vy,
                    0.0,
                    0.0;

  /// Set pointer to derived class
  kf_ptr = ekf_ptr;

  /// Process measurements to estimate final state
  Eigen::VectorXd curr_state = Process(measurements,
                                       ekf_H_lidar_ptr,
                                       ekf_H_radar_ptr,
                                       ekf_R_lidar_ptr,
                                       ekf_R_radar_ptr,
                                       ekf_Q_ptr,
                                       nullptr,
                                       nullptr,
                                       2);

  /// Print out states as reference
  std::cerr << "\nCMEKF curr_state:\n" << curr_state << std::endl;
  std::cerr << "\nCMEKF expected_state:\n" << expected_state << std::endl;

  EXPECT_TRUE(curr_state.isApprox(expected_state));
}
}   //  namespace kalman_filter


int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
