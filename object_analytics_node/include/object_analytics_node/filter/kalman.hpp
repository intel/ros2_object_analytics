
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

#pragma once

#include <opencv2/opencv.hpp>

namespace filter
{

class KalmanFilter
{
public:
    KalmanFilter();
    /** @overload
    @param dynamParams Dimensionality of the state.
    @param measureParams Dimensionality of the measurement.
    @param controlParams Dimensionality of the control vector.
    @param type Type of the created matrices that should be CV_32F or CV_64F.
    */
    KalmanFilter( int dynamParams, int measureParams, int controlParams = 0, int type = CV_32F );

    /** @brief Re-initializes Kalman filter. The previous content is destroyed.

    @param dynamParams Dimensionality of the state.
    @param measureParams Dimensionality of the measurement.
    @param controlParams Dimensionality of the control vector.
    @param type Type of the created matrices that should be CV_32F or CV_64F.
     */
    void init( int dynamParams, int measureParams, int controlParams = 0, int type = CV_32F );

    /** @brief Computes a predicted state.

    @param control The optional input control
     */
    const cv::Mat& predict( const cv::Mat& control = cv::Mat() );

    /** @brief Computes a predicted state.

    @param control The optional input control
     */
    void updateGain( const float & miss_measure);
    /** @brief Updates the predicted state from the measurement.

    @param measurement The measured system parameters
     */
    const cv::Mat& correct( const std::vector<cv::Mat>& measurements, cv::Mat& beta, const float& miss_measure);

    const cv::Mat& correct( const cv::Mat &measurement, timespec &stp);

    void operator =(const KalmanFilter& src)
    {
      if (this != &src)
      {
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
    };

    cv::Mat statePre;           //!< predicted state (x'(k)): x(k)=A*x(k-1)+B*u(k)
    cv::Mat statePost;          //!< corrected state (x(k)): x(k)=x'(k)+K(k)*(z(k)-H*x'(k))
    cv::Mat measurementPre;         //!< predicted measurement (z'(k)): z(k)=H*x'(k)
    cv::Mat transitionMatrix;   //!< state transition matrix (A)
    cv::Mat controlMatrix;      //!< control matrix (B) (not used if there is no control)
    cv::Mat measurementMatrix;  //!< measurement matrix (H)
    cv::Mat processNoiseCov;    //!< process noise covariance matrix (Q)
    cv::Mat measurementNoiseCov;//!< measurement noise covariance matrix (R)
    cv::Mat errorCovPre;        //!< priori error estimate covariance matrix (P'(k)): P'(k)=A*P(k-1)*At + Q)*/
    cv::Mat innoCov;               //!< Kalman gain matrix (K(k)): K(k)=P'(k)*Ht*inv(H*P'(k)*Ht+R)
    cv::Mat gain;               //!< Kalman gain matrix (K(k)): K(k)=P'(k)*Ht*inv(H*P'(k)*Ht+R)
    cv::Mat errorCovPost;       //!< posteriori error estimate covariance matrix (P(k)): P(k)=(I-K(k)*H)*P'(k)

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


