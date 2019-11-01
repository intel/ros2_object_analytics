/*M///////////////////////////////////////////////////////////////////////////////////////
 //
 //  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
 //
 //  By downloading, copying, installing or using the software you agree to this
 license.
 //  If you do not agree to this license, do not download, install,
 //  copy or use the software.
 //
 //
 //                           License Agreement
 //                For Open Source Computer Vision Library
 //
 // Copyright (C) 2013, OpenCV Foundation, all rights reserved.
 // Third party copyrights are property of their respective owners.
 //
 // Redistribution and use in source and binary forms, with or without
 modification,
 // are permitted provided that the following conditions are met:
 //
 //   * Redistribution's of source code must retain the above copyright notice,
 //     this list of conditions and the following disclaimer.
 //
 //   * Redistribution's in binary form must reproduce the above copyright
 notice,
 //     this list of conditions and the following disclaimer in the
 documentation
 //     and/or other materials provided with the distribution.
 //
 //   * The name of the copyright holders may not be used to endorse or promote
 products
 //     derived from this software without specific prior written permission.
 //
 // This software is provided by the copyright holders and contributors "as is"
 and
 // any express or implied warranties, including, but not limited to, the
 implied
 // warranties of merchantability and fitness for a particular purpose are
 disclaimed.
 // In no event shall the Intel Corporation or contributors be liable for any
 direct,
 // indirect, incidental, special, exemplary, or consequential damages
 // (including, but not limited to, procurement of substitute goods or services;
 // loss of use, data, or profits; or business interruption) however caused
 // and on any theory of liability, whether in contract, strict liability,
 // or tort (including negligence or otherwise) arising in any way out of
 // the use of this software, even if advised of the possibility of such damage.
 //
 //M*/

#ifndef OBJECT_ANALYTICS_NODE__TRACKER__TRACKERKCF_HPP_
#define OBJECT_ANALYTICS_NODE__TRACKER__TRACKERKCF_HPP_

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/ml.hpp>

#include <complex>
#include <cmath>
#include <vector>

#include "tracker/featureColorName.bin"
#include "filter/kalman.hpp"
#include "common/utility.hpp"

struct Params
{
  /**
  * \brief Constructor
  */
  Params();

  /**
  * \brief Read parameters from file, currently unused
  */
  void read(const FileNode & /*fn*/);

  /**
  * \brief Read parameters from file, currently unused
  */
  void write(FileStorage & /*fs*/) const;

  float detect_thresh;         //!<  detection confidence threshold
  float sigma;                 //!<  gaussian kernel bandwidth
  float lambda;                //!<  regularization
  float interp_factor;         //!<  linear interpolation factor for adaptation
  float output_sigma_factor;   //!<  spatial bandwidth (proportional to target)
  float pca_learning_rate;     //!<  compression learning rate
  bool resize;                  //!<  activate the resize feature to improve the processing speed
  bool split_coeff;             //!<  split the training coefficients into two matrices
  bool wrap_kernel;             //!<  wrap around the kernel values
  bool compress_feature;        //!<  activate the pca method to compress the features
  int max_patch_size;           //!<  threshold for the ROI size
  int compressed_size;          //!<  feature size after compression
  int desc_pca;        //!<  compressed descriptors of TrackerKCF::MODE
  int desc_npca;       //!<  non-compressed descriptors of TrackerKCF::MODE
};

enum MODE
{
  GRAY   = (1 << 0),
  CN     = (1 << 1),
  HOG     = (1 << 2),
  CUSTOM = (1 << 3)
};

/*
* Prototype
*/
class TrackerKCFImpl
{
public:
  TrackerKCFImpl();
  void setFeatureExtractor(void (* f)(const cv::Mat, const cv::Rect2d,cv::Mat &), 
    bool pca_func = false);
  Params params;
  bool isInit;

  /*
  * basic functions for tracker
  */
  bool initImpl(const cv::Mat & image, cv::Rect2d & boundingBox);

  bool detectImpl(const cv::Mat & image, cv::Rect2d & boundingBox, float & confidence,
    bool debug = false);

  bool updateWithTrackImpl(
    const cv::Mat & image, cv::Rect2d & boundingBox, float confidence,
    bool debug = false);

  bool updateWithDetectImpl(
    const cv::Mat & imageDet, cv::Rect2d & boundingBox, const cv::Mat & imageTrack,
    cv::Rect2d & trackBox, float confidence, bool debug = false);


  /*
   * feature functions
   */
  bool extractFeature(const cv::Mat & image, cv::Rect2d u_roi, cv::Mat & featureSet);

  bool extractKernelMap(const cv::Mat & srcFeature, const cv::Mat & dstFeature,
    cv::Mat & kernelMap);

  bool extractCovar(
    const cv::Mat & map, float threshold, cv::Mat & resMean, cv::Mat & resCovar, cv::Mat & eigVal,
    cv::Mat & eigVec);

  void constructGaussian(cv::Size size, cv::Mat & map);

  void drawFeature(cv::Rect2d u_roi, cv::Mat & feature, cv::Mat & disp);

  void drawDetectProcess(
    cv::Rect2d u_roi, cv::Mat & feature, cv::Mat & base, cv::Mat & kernel, cv::Mat & response,
    cv::Mat & alpha_fft);

  void drawUpdateWithDetProcess(
    cv::Rect2d u_roi, cv::Mat & feature, cv::Mat & base, cv::Mat & kernel,
    cv::Mat & response, cv::Mat & new_base, cv::Mat & alpha_fft);

  void drawUpdateWithTrackProcess(
    cv::Rect2d u_roi, cv::Mat & feature, cv::Mat & base, cv::Mat & new_base,
    cv::Mat & alpha_fft);

  cv::Mat getCovar();

  cv::Rect2d roi_scale;
  bool kalman_enable;
  bool scale_update_enable;
  filter::KalmanFilter kalman;
  cv::Mat corrMean, corrCovar;
  cv::Mat corrEigVal, corrEigVec;

protected:
  /*
  * KCF functions and vars
  */
  void createHanningWindow(OutputArray dest, const cv::Size winSize, const int type) const;
  void inline fft2(constcv::Mat src, std::vector<Mat> & dest, std::vector<Mat> & layers_data) const;
  void inline fft2(constcv::Mat src, cv::Mat & dest) const;
  void inline ifft2(constcv::Mat src, cv::Mat & dest) const;
  void inline pixelWiseMult(
    const std::vector<Mat> src1, const std::vector<Mat> src2,
    std::vector<Mat> & dest, const int flags,
    const bool conjB = false) const;
  void inline sumChannels(std::vector<Mat> src, cv::Mat & dest) const;
  void inline updateProjectionMatrix(
    constcv::Mat src, cv::Mat & old_cov,cv::Mat & proj_matrix, float pca_rate, int compressed_sz,
    std::vector<Mat> & layers_pca, std::vector<Scalar> & average,
    cv::Mat pca_data, cv::Mat new_cov,cv::Mat w, cv::Mat u,cv::Mat v);
  void inline compress(
    constcv::Mat proj_matrix, constcv::Mat src, cv::Mat & dest, cv::Mat & data,
   cv::Mat & compressed) const;
  bool getSubWindow(
    constcv::Mat img, const cv::Rect2d roi, cv::Mat & feat, cv::Mat & patch,
    MODE desc = GRAY) const;
  bool getSubWindow(
    const cv::Mat img, const cv::Rect2d roi, cv::Mat & feat, void (* f)(const cv::Mat, const cv::Rect2d,
    cv::Mat &)) const;

  void extractCN(Mat patch_data, cv::Mat & cnFeatures) const;

  void extractHOG(Mat & im, cv::Mat & hogFeatures) const;

  void denseGaussKernel(
    const float sigma, const Mat, constcv::Mat y_data, cv::Mat & k_data,
    std::vector<Mat> & layers_data, std::vector<Mat> & xf_data, std::vector<Mat> & yf_data,
    std::vector<Mat> xyf_v, cv::Mat xy,i cv::Mat xyf) const;
  void calcResponse(
    constcv::Mat alphaf_data, constcv::Mat kf_data, cv::Mat & response_data,
   cv::Mat & spec_data) const;
  void calcResponse(
    constcv::Mat alphaf_data, constcv::Mat alphaf_den_data, constcv::Mat kf_data,
   cv::Mat & response_data, cv::Mat & spec_data, cv::Mat & spec2_data) const;

  void shiftRows(Mat & mat) const;
  void shiftRows(Mat & mat, int n) const;
  void shiftCols(Mat & mat, int n) const;
#ifdef HAVE_OPENCL
  bool inline oclTransposeMM(constcv::Mat src, float alpha, UMat & dst);
#endif

private:
  float output_sigma;
  Rect2d roi;
 cv::Mat hann;  // hann window filter
 cv::Mat hann_cn;  // 10 dimensional hann-window filter for CN features,

 cv::Mat y, yf;  // training response and its FFT
 cv::Mat x;  // observation and its FFT
 cv::Mat k, kf;  // dense gaussian kernel and its FFT
 cv::Mat kf_lambda;  // kf+lambda
 cv::Mat new_alphaf, alphaf;  // training coefficients
 cv::Mat new_alphaf_den, alphaf_den;  // for splitted training coefficients
 cv::Mat z;  // model
 cv::Mat response;  // detection result
 cv::Mat old_cov_mtx, proj_mtx;  // for feature compression

  // pre-definedcv::Mat variables for optimization of private functions
 cv::Mat spec, spec2;
  std::vector<Mat> layers;
  std::vector<Mat> vxf, vyf, vxyf;
 cv::Mat xy_data, xyf_data;
 cv::Mat data_temp, compress_data;
  std::vector<Mat> layers_pca_data;
  std::vector<Scalar> average_data;
 cv::Mat img_Patch;

  // storage of the extracted features
  std::vector<Mat> features_pca;
  std::vector<Mat> features_npca;
  std::vector<MODE> descriptors_pca;
  std::vector<MODE> descriptors_npca;

  // optimization variables for updateProjectionMatrix
 cv::Mat data_pca, new_covar, w_data, u_data, vt_data;

  // custom feature extractor
  bool use_custom_extractor_pca;
  bool use_custom_extractor_npca;
  std::vector<void (*)(constcv::Mat img, const Rect2d roi, cv::Mat & output)> extractor_pca;
  std::vector<void (*)(constcv::Mat img, const Rect2d roi, cv::Mat & output)> extractor_npca;

  // resize the image whenever needed and the patch size is large
  bool resizeImage;
  // resize ratio, should above 1.0
  float resizeRatio;

  float paddingRatio;

#ifdef HAVE_OPENCL
  ocl::Kernel transpose_mm_ker;  // OCL kernel to compute transpose matrix multiply matrix.
#endif

  int frame;
};

#endif  // OBJECT_ANALYTICS_NODE__TRACKER__TRACKERKCF_HPP_
