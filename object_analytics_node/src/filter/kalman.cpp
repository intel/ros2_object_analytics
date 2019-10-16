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

#include "filter/kalman.hpp"

namespace filter
{

KalmanFilter::KalmanFilter() {}
KalmanFilter::KalmanFilter(int dynamDim, int measureDim, int controlParams, int type)
{
    init(dynamDim, measureDim, controlParams, type);
}

void KalmanFilter::init(int dynamDim, int measureDim, int controlParams, int type)
{
    CV_Assert( dynamDim > 0 && measureDim > 0 );
    CV_Assert( type == CV_32F || type == CV_64F );
    controlParams = std::max(controlParams, 0);

    statePre = cv::Mat::zeros(dynamDim, 1, type);
    statePost = cv::Mat::zeros(dynamDim, 1, type);
    measurementPre = cv::Mat::zeros(measureDim, 1, type);
    transitionMatrix = cv::Mat::eye(dynamDim, dynamDim, type);

    processNoiseCov = cv::Mat::eye(dynamDim, dynamDim, type);
    measurementMatrix = cv::Mat::zeros(measureDim, dynamDim, type);
    measurementNoiseCov = cv::Mat::eye(measureDim, measureDim, type);
    innoCov = cv::Mat::eye(measureDim, measureDim, type);

    errorCovPre = cv::Mat::zeros(dynamDim, dynamDim, type);
    errorCovPost = cv::Mat::zeros(dynamDim, dynamDim, type);
    gain = cv::Mat::zeros(dynamDim, measureDim, type);

    if( controlParams > 0 )
        controlMatrix = cv::Mat::zeros(dynamDim, controlParams, type);
    else
        controlMatrix.release();

    temp1.create(dynamDim, dynamDim, type);
    temp2.create(measureDim, dynamDim, type);
    temp3.create(measureDim, measureDim, type);
    temp4.create(measureDim, dynamDim, type);
    temp5.create(measureDim, 1, type);
}

bool KalmanFilter::initialParams(cv::Mat& state, cv::Mat& initialCov, timespec& stp)
{

  if (state.size() == statePost.size())
  {
    state.copyTo(statePost);
  } else {
    return false;
  }

  if (initialCov.size() == errorCovPost.size())
  {
    initialCov.copyTo(errorCovPost);
  } else {
    return false;
  }
 
  stamp = stp;

  return true;
}

void KalmanFilter::configDeltaT(timespec deltaT)
{
  // 10ms unit
  float dt = (deltaT.tv_sec*1e2 + deltaT.tv_nsec*1e-7);

  /** DYNAMIC MODEL **/
  //  [1 0 dt 0  ]
  //  [0 1 0  dt ]
  //  [0 0 1  0  ]
  //  [0 0 0  1  ]

  // speed
  transitionMatrix.at<float>(0,2) = dt;
  transitionMatrix.at<float>(1,3) = dt;

  /** MEASUREMENT MODEL **/
  //  [1 0 0 0]
  //  [0 1 0 0]
  
  measurementMatrix.at<float>(0,0) = 1;  // x
  measurementMatrix.at<float>(1,1) = 1;  // y

  float n1 = std::pow(dt, 4.) / 4.;
  float n2 = std::pow(dt, 3.) / 2.;
  float n3 = std::pow(dt, 2.);
  processNoiseCov = (cv::Mat_<float>(4, 4) <<
                                       n1, 0,  n2, 0,
                                       0,  n1, 0,  n2,
                                       n2,  0,  n3, 0, 
                                       0,  n2,  0,  n3);
}

const cv::Mat& KalmanFilter::predict(timespec &stp, const cv::Mat& control)
{
    timespec deltaT;
    deltaT.tv_sec = stp.tv_sec - stamp.tv_sec;
    deltaT.tv_nsec = stp.tv_nsec - stamp.tv_nsec;

    configDeltaT(deltaT);
#if 0
    std::cout << "\n-------------------------------------------"<< std::endl;
    std::cout << "predict begin func statePost:\n" << statePost << std::endl;
    std::cout << "predict begin func errorCovPost:\n" << errorCovPost << std::endl;
#endif
    // update the state: x'(k) = A*x(k)
    statePre = transitionMatrix*statePost;

    if( !control.empty() )
        // x'(k) = x'(k) + B*u(k)
        statePre += controlMatrix*control;

    // update error covariance matrices: temp1 = A*P(k)
    temp1 = transitionMatrix*errorCovPost;

    // P'(k) = temp1*At + Q
    gemm(temp1, transitionMatrix, 1, processNoiseCov, 1, errorCovPre, cv::GEMM_2_T);

    // temp2 = H*P'(k)
    temp2 = measurementMatrix * errorCovPre;

    // get predit measurement noise
    measurementCovPre = temp2 * measurementMatrix.t();

    // handle the case when there will be measurement before the next predict.
    statePre.copyTo(statePost);
    errorCovPre.copyTo(errorCovPost);
    measurementPre = measurementMatrix * statePre;

#if 1
    std::cout << "\n-------------------------------------------"<< std::endl;
    std::cout << "predict func delta T-sec:" << deltaT.tv_sec << ", T-milisec:"<<deltaT.tv_nsec*1e-6 << std::endl;
    std::cout << "predict func statePre:\n" << statePre << std::endl;
    std::cout << "predict func errorCovPre:\n" << errorCovPre << std::endl;
#endif

    stamp = stp;

    return measurementPre;
}


const cv::Mat& KalmanFilter::correct( const std::vector<cv::Mat>& measurements, cv::Mat& beta, const float& miss_measure)
{
    statePost.setTo(cv::Scalar(0));

    // x(k) = x'(k) + K(k)*temp5
    for(unsigned int i = 0; i<measurements.size(); i++)
    {
      temp5 = statePre + gain*(measurements[i] - measurementPre);
      statePost = statePre + beta.at<float>(i)*temp5;
    }
    statePost = statePost + miss_measure*statePre;

    errorCovPost = errorCovPost + miss_measure*(statePre*statePre.t() - statePost*statePost.t());
    for(unsigned int i = 0; i<measurements.size(); i++)
    {
      temp5 = statePre + gain*(measurements[i] - measurementPre);
      errorCovPost = errorCovPost + beta.at<float>(i)*(temp5*temp5.t() - statePost*statePost.t());
    }
    return statePost;
}

bool KalmanFilter::correct(const cv::Mat &measurement, cv::Mat &measureCov)
{
    if (measureCov.size() != measurementNoiseCov.size())
      return false;

    measurementNoiseCov = measureCov.clone();
     
    // innoCov = temp2*Ht + R
    gemm(temp2, measurementMatrix, 1, measurementNoiseCov, 1, innoCov, cv::GEMM_2_T);
    //innoCov = measurementCovPre + measurementNoiseCov;    

    // temp4 = inv(innoCov)*temp2 = Kt(k)
    solve(innoCov, temp2, temp4, cv::DECOMP_SVD);

    // K(k)
    gain = temp4.t();

    statePost.setTo(cv::Scalar(0));

    // x(k) = x'(k) + K(k)*temp5
    statePost = statePre + gain*(measurement - measurementPre);
    errorCovPost = errorCovPre - gain*temp2;

#if 0
    std::cout << "*******************************************"<< std::endl;
    std::cout << "predict func measurementNoiseCov:\n" << measurementNoiseCov << std::endl;
    std::cout << "predict func gain:\n" << gain << std::endl;

    std::cout << "correct func measurementPrediction:\n" << measurementPre << std::endl;
    std::cout << "correct func measurement:\n" << measurement << std::endl;
    std::cout << "correct func statePost:\n" << statePost << std::endl;
    std::cout << "correct func errorCovPost\n:" << errorCovPost << std::endl;
    std::cout << "-------------------------------------------\n"<< std::endl;
#endif
    return true;
}

}
