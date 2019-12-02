// Copyright (c) 2018 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef OBJECT_ANALYTICS_NODE__FILTER__KALMAN_HPP_
#define OBJECT_ANALYTICS_NODE__FILTER__KALMAN_HPP_

#include <opencv2/opencv.hpp>

#include <vector>

namespace filter
{

class KalmanFilter
{
public:
  KalmanFilter();
  /** @overload
  @param dynamDim Dimensionality of the state.
  @param measureDim Dimensionality of the measurement.
  @param controlParams Dimensionality of the control vector.
  @param type Type of the created matrices that should be CV_32F or CV_64F.
  */
  KalmanFilter(
    int dynamDim, int measureDim, int controlParams = 0,
    int type = CV_32F);

  /** @brief Initializes Kalman filter. The previous content is destroyed.

  @param dynamDim Dimensionality of the state.
  @param measureDim Dimensionality of the measurement.
  @param controlParams Dimensionality of the control vector.
  @param type Type of the created matrices that should be CV_32F or CV_64F.
   */
  void init(
    int dynamDim, int measureDim, int controlParams = 0,
    int type = CV_32F);

  /** @brief Initializes parameters for Kalman filter.

  @param state Initial state.
  @param initialCov Initial covariance.
   */
  bool initialParams(cv::Mat & state, cv::Mat & initialCov, timespec & stp);

  /** @brief Config Process Noise and Measurement Noise.

  @param deltaT Time interval for next predict.
   */
  void configDeltaT(timespec deltaT);

  /** @brief Computes a predicted state.

  @param time stamp for this prediction
  @param control The optional input control
   */
  const cv::Mat & predict(timespec & stp, const cv::Mat & control = cv::Mat());

  /** @brief Updates the predicted state from the measurement.

  @param measurement The measured system parameters
   */
  const cv::Mat & correct(
    const std::vector<cv::Mat> & measurements,
    cv::Mat & beta, const float & miss_measure);

  bool correct(const cv::Mat & measurement, cv::Mat & measureCov);

  void operator=(const KalmanFilter & src)
  {
    if (this != &src) {
      src.statePre.copyTo(statePre);
      src.statePost.copyTo(statePost);
      src.measurementPre.copyTo(measurementPre);
      src.transitionMatrix.copyTo(transitionMatrix);
      src.controlMatrix.copyTo(controlMatrix);
      src.measurementMatrix.copyTo(measurementMatrix);
      src.processNoiseCov.copyTo(processNoiseCov);
      src.measurementNoiseCov.copyTo(measurementNoiseCov);
      src.errorCovPre.copyTo(errorCovPre);
      src.innoCov.copyTo(innoCov);
      src.gain.copyTo(gain);
      src.errorCovPost.copyTo(errorCovPost);
      src.temp1.copyTo(temp1);
      src.temp2.copyTo(temp2);
      src.temp3.copyTo(temp3);
      src.temp4.copyTo(temp4);
      src.temp5.copyTo(temp5);
    }
  }

  cv::Mat statePre;  //!< predicted state (x'(k)): x(k)=A*x(k-1)+B*u(k)
  cv::Mat
    statePost;    //!< corrected state (x(k)): x(k)=x'(k)+K(k)*(z(k)-H*x'(k))
  cv::Mat measurementPre;    //!< predicted measurement (z'(k)): z(k)=H*x'(k)
  cv::Mat transitionMatrix;  //!< state transition matrix (A)
  cv::Mat
    controlMatrix;    //!< control matrix (B) (not used if there is no control)
  cv::Mat measurementMatrix;    //!< measurement matrix (H)
  cv::Mat processNoiseCov;      //!< process noise covariance matrix (Q)
  cv::Mat measurementNoiseCov;  //!< measurement noise covariance matrix (R)
  cv::Mat
    measurementCovPre;    //!< measurement noise covariance matrix (H*P'(k)*Ht)
  cv::Mat errorCovPre;    //!< priori error estimate covariance matrix (P'(k)):
  //!< P'(k)=A*P(k-1)*At + Q)*/
  cv::Mat
    innoCov;     //!< Kalman gain matrix (K(k)): K(k)=P'(k)*Ht*inv(H*P'(k)*Ht+R)
  cv::Mat gain;  //!< Kalman gain matrix (K(k)): K(k)=P'(k)*Ht*inv(H*P'(k)*Ht+R)
  cv::Mat errorCovPost;  //!< posteriori error estimate covariance matrix
  //!< (P(k)): P(k)=(I-K(k)*H)*P'(k)

  // temporary matrices
  cv::Mat temp1;
  cv::Mat temp2;
  cv::Mat temp3;
  cv::Mat temp4;
  cv::Mat temp5;

  // last updated stamp;
  timespec stamp;
};

}  // namespace filter

#endif  // OBJECT_ANALYTICS_NODE__FILTER__KALMAN_HPP_
