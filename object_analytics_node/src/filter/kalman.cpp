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

#include "object_analytics_node/filter/kalman.hpp"

namespace filter
{

KalmanFilter::KalmanFilter() {}
KalmanFilter::KalmanFilter(int dynamParams, int measureParams, int controlParams, int type)
{
    init(dynamParams, measureParams, controlParams, type);
}

void KalmanFilter::init(int DP, int MP, int CP, int type)
{
    CV_Assert( DP > 0 && MP > 0 );
    CV_Assert( type == CV_32F || type == CV_64F );
    CP = std::max(CP, 0);

    statePre = cv::Mat::zeros(DP, 1, type);
    statePost = cv::Mat::zeros(DP, 1, type);
    measurementPre = cv::Mat::zeros(MP, 1, type);
    transitionMatrix = cv::Mat::eye(DP, DP, type);

    processNoiseCov = cv::Mat::eye(DP, DP, type);
    measurementMatrix = cv::Mat::zeros(MP, DP, type);
    measurementNoiseCov = cv::Mat::eye(MP, MP, type);
    innoCov = cv::Mat::eye(MP, MP, type);

    errorCovPre = cv::Mat::zeros(DP, DP, type);
    errorCovPost = cv::Mat::zeros(DP, DP, type);
    gain = cv::Mat::zeros(DP, MP, type);

    if( CP > 0 )
        controlMatrix = cv::Mat::zeros(DP, CP, type);
    else
        controlMatrix.release();

    temp1.create(DP, DP, type);
    temp2.create(MP, DP, type);
    temp3.create(MP, MP, type);
    temp4.create(MP, DP, type);
    temp5.create(MP, 1, type);
}

const cv::Mat& KalmanFilter::predict(const cv::Mat& control)
{
    // update the state: x'(k) = A*x(k)
    statePre = transitionMatrix*statePost;

    if( !control.empty() )
        // x'(k) = x'(k) + B*u(k)
        statePre += controlMatrix*control;

    // update error covariance matrices: temp1 = A*P(k)
    temp1 = transitionMatrix*errorCovPost;

    // P'(k) = temp1*At + Q
    gemm(temp1, transitionMatrix, 1, processNoiseCov, 1, errorCovPre, cv::GEMM_2_T);

    // handle the case when there will be measurement before the next predict.
//    statePre.copyTo(statePost);
//    errorCovPre.copyTo(errorCovPost);
    measurementPre = measurementMatrix * statePre;

    // temp2 = H*P'(k)
    temp2 = measurementMatrix * errorCovPre;

    // innoCov = temp2*Ht + R
    gemm(temp2, measurementMatrix, 1, measurementNoiseCov, 1, innoCov, cv::GEMM_2_T);

    // temp4 = inv(innoCov)*temp2 = Kt(k)
    solve(innoCov, temp2, temp4, cv::DECOMP_SVD);

    // K(k)
    gain = temp4.t();

    std::cout << "\n-------------------------------------------"<< std::endl;
    std::cout << "predict func statePre:\n" << statePre << std::endl;
    std::cout << "predict func errorCovPre:\n" << errorCovPre << std::endl;
    std::cout << "predict func measurementNoiseCov:\n" << measurementNoiseCov << std::endl;
    std::cout << "predict func gain:\n" << gain << std::endl;

    return statePre;
}

void KalmanFilter::updateGain(const float &miss_measure)
{
    // P(k) = P'(k) - K(k)*temp2
    errorCovPost = errorCovPre - (1 - miss_measure)*gain*temp2;
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

const cv::Mat& KalmanFilter::correct(const cv::Mat &measurement, timespec &stp)
{
    statePost.setTo(cv::Scalar(0));

    // x(k) = x'(k) + K(k)*temp5
    statePost = statePre + gain*(measurement - measurementPre);
    errorCovPost = errorCovPre - gain*temp2;
    stamp = stp;

    std::cout << "*******************************************"<< std::endl;
    std::cout << "correct func measurementPrediction:\n" << measurementPre << std::endl;
    std::cout << "correct func measurement:\n" << measurement << std::endl;
    std::cout << "correct func statePost:\n" << statePost << std::endl;
    std::cout << "correct func errorCovPost\n:" << errorCovPost << std::endl;
    std::cout << "-------------------------------------------\n"<< std::endl;
    return statePost;
}

#if 0
TBD: assign differnt measurement Noise for differnt situation.
//type == 1: correct by tracker
//type == 2: correct by detect
//type == 3: continuous correct KF, eg, 1.updated by tracker, 2.updated by detec.
const cv::Mat& KalmanFilter::correct(const cv::Mat &measurement, int type)
{
    statePost.setTo(cv::Scalar(0));

    // x(k) = x'(k) + K(k)*temp5
    statePost = statePre + gain*(measurement - measurementPre);
    errorCovPost = errorCovPre - (1 - miss_measure)*gain*temp2;

    return statePost;
}
#endif

}
