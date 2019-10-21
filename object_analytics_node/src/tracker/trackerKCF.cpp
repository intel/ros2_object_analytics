/*M///////////////////////////////////////////////////////////////////////////////////////
 //
 //  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
 //
 //  By downloading, copying, installing or using the software you agree to this license.
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
 // Redistribution and use in source and binary forms, with or without modification,
 // are permitted provided that the following conditions are met:
 //
 //   * Redistribution's of source code must retain the above copyright notice,
 //     this list of conditions and the following disclaimer.
 //
 //   * Redistribution's in binary form must reproduce the above copyright notice,
 //     this list of conditions and the following disclaimer in the documentation
 //     and/or other materials provided with the distribution.
 //
 //   * The name of the copyright holders may not be used to endorse or promote products
 //     derived from this software without specific prior written permission.
 //
 // This software is provided by the copyright holders and contributors "as is" and
 // any express or implied warranties, including, but not limited to, the implied
 // warranties of merchantability and fitness for a particular purpose are disclaimed.
 // In no event shall the Intel Corporation or contributors be liable for any direct,
 // indirect, incidental, special, exemplary, or consequential damages
 // (including, but not limited to, procurement of substitute goods or services;
 // loss of use, data, or profits; or business interruption) however caused
 // and on any theory of liability, whether in contract, strict liability,
 // or tort (including negligence or otherwise) arising in any way out of
 // the use of this software, even if advised of the possibility of such damage.
 //
 //M*/

#include "tracker/trackerKCF.hpp"
#include "filter/kalman.hpp"

/*---------------------------
|  TrackerKCF
|---------------------------*/

/*
* Parameters
*/
Params::Params(){
  detect_thresh = exp(-0.5f);
  sigma=0.2f;
  lambda=0.0000f;
  interp_factor=0.2f;
  output_sigma_factor=1.0f / 4.0f;
  resize=true;
  max_patch_size=60*60;
  wrap_kernel=false;
  desc_npca = GRAY|CN;
  desc_pca = 0;

  //feature compression
  compress_feature=false;
  compressed_size=2;
  pca_learning_rate=0.15f;
};

TrackerKCFImpl::TrackerKCFImpl() 
{
  isInit = false;

  resizeImage = false;
  resizeRatio = 1.0f;

  paddingRatio = 1.0f;

  use_custom_extractor_pca = false;
  use_custom_extractor_npca = false;
  kalman_enable = false;
  scale_update_enable = false;
}


bool TrackerKCFImpl::extractFeature(const Mat& image, Rect2d u_roi, cv::Mat& featureSet)
{

  cv::Mat img = image.clone();
  std::vector<Mat> features;

  u_roi.x =cvRound(u_roi.x);
  u_roi.y =cvRound(u_roi.y);
  u_roi.width =cvRound(u_roi.width);
  u_roi.height=cvRound(u_roi.height);

  // check the channels of the input image, grayscale is preferred
  CV_Assert(img.channels() == 1 || img.channels() == 3);

  features.resize(descriptors_npca.size());

  // extract the patch for learning purpose
  // get non compressed descriptors
  for(unsigned i=0;i<descriptors_npca.size()-extractor_npca.size();i++){
    if(!getSubWindow(img,u_roi, features[i], img_Patch, descriptors_npca[i]))
      return false;
  }

  // get non-compressed custom descriptors
  for(unsigned i=0,j=(unsigned)(descriptors_npca.size()-extractor_npca.size());i<extractor_npca.size();i++,j++){
    if(!getSubWindow(img,u_roi, features[j], extractor_npca[i]))
      return false;
  }

  if(features.size()>1)
    cv::merge(features, featureSet);
  else
    featureSet = features[0];

  return true;
} 


bool TrackerKCFImpl::extractKernelMap(const Mat& srcFeatures, const Mat& dstFeatures, cv::Mat& kernelMap)
{

  if(srcFeatures.channels() != dstFeatures.channels())
    return false;

  if(srcFeatures.size() != dstFeatures.size())
    return false;

  std::vector<Mat> layers_t;
  std::vector<Mat> vxf_t,vyf_t,vxyf_t;
  Mat xy_data_t,xyf_data_t;
  layers_t.resize(srcFeatures.channels());
  vxf_t.resize(srcFeatures.channels());
  vyf_t.resize(srcFeatures.channels());
  vxyf_t.resize(vyf_t.size());

  // Kernel Regularized Least-Squares, calculate alphas
  denseGaussKernel(params.sigma,srcFeatures,dstFeatures,kernelMap,
                   layers_t,vxf_t,vyf_t,vxyf_t,xy_data_t,xyf_data_t);

  return true;
}


bool TrackerKCFImpl::extractCovar(const Mat& map, float threshold, Mat& resMean, Mat& resCovar, Mat& eigVal, Mat& eigVec)
{

  Mat m;
  float disThresh = exp(-0.5f);
  for(int i=0;i<map.rows;i++){
    for(int j=0;j<map.cols;j++){
      float val = map.at<float>(i, j);
      if (val >= disThresh)
      {
          Mat sol = Mat::zeros(1, 2, CV_32F);
          sol.at<float>(0) = j;
          sol.at<float>(1) = i;
          m.push_back(sol);
      }
    }
  }
  
  if (m.empty())
    return false;

  cv::calcCovarMatrix(m, resCovar, resMean, CV_COVAR_NORMAL|CV_COVAR_ROWS|CV_COVAR_SCALE, CV_32F);
  eigen(resCovar, eigVal, eigVec);
 
  return true;
}

void TrackerKCFImpl::constructGaussian(cv::Size size, Mat& map)
{
  map = Mat::zeros((int)size.height,(int)size.width,CV_32F);

  float sigma=std::sqrt(static_cast<float>(size.width*size.height))*params.output_sigma_factor;
  sigma=-0.5f/(sigma*sigma);

  for(int i=0;i<int(size.height);i++){
    for(int j=0;j<int(size.width);j++){
      float val = static_cast<float>((i-size.height/2+1)*
                   (i-size.height/2+1)+(j-size.width/2+1)*(j-size.width/2+1));

      map.at<float>(i,j) = val;
    }
  }

  map*=(float)sigma;
  cv::exp(map,map);
}


void TrackerKCFImpl::drawFeature(cv::Rect2d u_roi, Mat& feature, Mat& disp)
{
  if(!feature.empty())
  {

    std::vector<cv::Mat> featureMat(feature.channels());
    split(feature, featureMat);

    cv::Mat hog = disp(u_roi);
    cv::normalize(featureMat[0], featureMat[0], 0.0f, 1.0f, cv::NORM_MINMAX);
    cv::resize(featureMat[0], hog, hog.size(), 0, 0, cv::INTER_LINEAR);

    
    u_roi.x += u_roi.width;
    cv::Mat cMat = disp(u_roi);
    cv::normalize(featureMat[1], featureMat[1], 0.0f, 1.0f, cv::NORM_MINMAX);
    cv::resize(featureMat[1], cMat, cMat.size(), 0, 0, cv::INTER_LINEAR);
  }
}


/*
 * Initialization:
 * - creating hann window filter
 * - ROI padding
 * - creating a gaussian response for the training ground-truth
 * - perform FFT to the gaussian response
 */
bool TrackerKCFImpl::initImpl( const Mat& image, Rect2d& boundingBox )
{
  frame=0;

  Rect2d image_area(0, 0, image.cols, image.rows);

  float centra_x = (float)boundingBox.x + boundingBox.width/2.0f;
  float centra_y = (float)boundingBox.y + boundingBox.height/2.0f;

  roi.width = boundingBox.width*paddingRatio;
  roi.height = boundingBox.height*paddingRatio;
  roi.x = centra_x - roi.width/2;
  roi.y = centra_y - roi.height/2;

  //resize the ROI whenever needed
  if(params.resize && roi.width*roi.height>params.max_patch_size){

    resizeRatio = sqrt((roi.width*roi.height)/params.max_patch_size);
    if (resizeRatio < 1.0f) resizeRatio = 1.0f;

    resizeImage=true;
    roi.x/=resizeRatio;
    roi.y/=resizeRatio;
    roi.width/=resizeRatio;
    roi.height/=resizeRatio;
  }
  
  // resize the image whenever needed
  cv::Mat img = image.clone();
  if(resizeImage)
    cv::resize(img,img,Size(img.cols/resizeRatio,img.rows/resizeRatio),0,0,cv::INTER_LINEAR);


  if (image.channels() == 1) { // disable CN for grayscale images
    params.desc_pca &= ~(CN);
    params.desc_npca &= ~(CN);

    params.desc_pca &= ~(HOG);
    params.desc_npca &= ~(HOG);
  }

  // record the non-compressed descriptors
  if((params.desc_npca & GRAY) == GRAY)descriptors_npca.push_back(GRAY);
  if((params.desc_npca & CN) == CN)descriptors_npca.push_back(CN);
  if((params.desc_npca & HOG) == HOG)descriptors_npca.push_back(HOG);
  if(use_custom_extractor_npca)descriptors_npca.push_back(CUSTOM);
    features_npca.resize(descriptors_npca.size());

  // accept only the available descriptor modes
  CV_Assert(
    (params.desc_pca & GRAY) == GRAY
    || (params.desc_npca & GRAY) == GRAY
    || (params.desc_pca & CN) == CN
    || (params.desc_npca & CN) == CN
    || (params.desc_pca & HOG) == HOG
    || (params.desc_npca & HOG) == HOG
    || use_custom_extractor_pca
    || use_custom_extractor_npca
  );

  // return true only if roi has intersection with the image
  if((roi & Rect2d(0,0, resizeImage ? image.cols / resizeRatio : image.cols,
                   resizeImage ? image.rows / resizeRatio : image.rows)) == Rect2d())
      return false;


  // initialize the hann window filter
  createHanningWindow(hann, roi.size(), CV_32F);

  // hann window filter for CN feature
  Mat _layer[] = {hann, hann, hann, hann, hann, hann, hann, hann, hann, hann};
  merge(_layer, 10, hann_cn);

  extractFeature(img, roi, x);
  z = x.clone();

  cv::Size feature_size = x.size();

  constructGaussian(feature_size, y);

  extractCovar(y, exp(-0.5f), corrMean, corrCovar, corrEigVal, corrEigVec);

  // perform fourier transfor to the gaussian response
  fft2(y,yf);

  extractKernelMap(x, x, k);

  // compute the fourier transform of the kernel and add a small value
  fft2(k,kf);
  kf_lambda=kf+params.lambda;

  alphaf=Mat_<Vec2f >(yf.rows, yf.cols);

  float den;
  for(int i=0;i<yf.rows;i++){
    for(int j=0;j<yf.cols;j++){
      den = 1.0f/(kf_lambda.at<Vec2f>(i,j)[0]*kf_lambda.at<Vec2f>(i,j)[0]+kf_lambda.at<Vec2f>(i,j)[1]*kf_lambda.at<Vec2f>(i,j)[1]);

      alphaf.at<Vec2f>(i,j)[0]=
      (yf.at<Vec2f>(i,j)[0]*kf_lambda.at<Vec2f>(i,j)[0]+yf.at<Vec2f>(i,j)[1]*kf_lambda.at<Vec2f>(i,j)[1])*den;
      alphaf.at<Vec2f>(i,j)[1]=
      (yf.at<Vec2f>(i,j)[1]*kf_lambda.at<Vec2f>(i,j)[0]-yf.at<Vec2f>(i,j)[0]*kf_lambda.at<Vec2f>(i,j)[1])*den;
    }
  }

  if (kalman_enable)
  {
    // init kalman filter
    kalman.init(4, 2, 0, CV_32F);

    cv::Mat state = cv::Mat::zeros(4, 1, CV_32F);
    state.at<float>(0) = centra_x;
    state.at<float>(1) = centra_y;

    timespec stp;
    stp.tv_sec = frame/1000;
    stp.tv_nsec = (frame%1000)*1e6;

    cv::Mat initialCov  = cv::Mat::eye(4, 4, CV_32F);
    initialCov.at<float>(0, 0) = corrCovar.at<float>(0, 0);
    initialCov.at<float>(1, 1) = corrCovar.at<float>(1, 1);
    initialCov.at<float>(2, 2) = 0;
    initialCov.at<float>(3, 3) = 0;

    kalman.initialParams(state, initialCov, stp);
  }

  isInit = true;

  return true;
}

cv::Mat TrackerKCFImpl::getCovar()
{
  return corrCovar;
}

bool TrackerKCFImpl::detectImpl(const Mat& image, Rect2d& boundingBox, float& confidence, bool debug)
{
	// min-max response
  double minVal, maxVal;
	// min-max location
  cv::Point minLoc, maxLoc;

  frame++;

  if (!isInit) return false;

  //predict centra point
  if (kalman_enable)
  {
    timespec stp;
    stp.tv_sec = frame/1000;
    stp.tv_nsec = (frame%1000)*1e6;
    cv::Mat bcentra = kalman.predict(stp);
    boundingBox.x = bcentra.at<float>(0) - boundingBox.width/2;
    boundingBox.y = bcentra.at<float>(1) - boundingBox.height/2;
  }

  Mat img = image.clone();
  if(resizeImage)
    cv::resize(img,img,Size(img.cols/resizeRatio,img.rows/resizeRatio),0,0,cv::INTER_LINEAR);

  // check the channels of the input image, grayscale is preferred
  CV_Assert(img.channels() == 1 || img.channels() == 3);

  float centra_x = (float)boundingBox.x + boundingBox.width/2.0f;
  float centra_y = (float)boundingBox.y + boundingBox.height/2.0f;

  if (centra_x < 0 || centra_y < 0)
    return false;

#ifndef NDEBUG
  std::cout << "\nDetect input boundingBox:" << boundingBox << std::endl;
  std::cout << "\ninput centra point x:" << centra_x <<",y:" << centra_y << std::endl;
#endif


  roi_scale.width = boundingBox.width*paddingRatio;
  roi_scale.height = boundingBox.height*paddingRatio;
  roi_scale.x = centra_x - roi_scale.width/2;
  roi_scale.y = centra_y - roi_scale.height/2;

  if (resizeImage)
  {
    roi_scale.x/=resizeRatio;
    roi_scale.y/=resizeRatio;
    roi_scale.width/=resizeRatio;
    roi_scale.height/=resizeRatio;
  }

  cv::Rect2d img_rect(0, 0, img.cols, img.rows);
  cv::Rect2d overlap = img_rect & roi_scale;
  if (overlap.area() <= 0 || overlap.width <=1 || overlap.height <=1)
  {
    TRACE_ERR("\nDetect: bounding box out of image range");
    return false;
  }

  bool ret  = false;
  ret = extractFeature(img, roi_scale, x);
  if (!ret)
  {
    TRACE_ERR("\nextractFeature failed!!!");
    CV_Assert(ret == true);
  }

  ret = extractKernelMap(x, z, k);
  if (!ret)
  {
    TRACE_ERR("\nextractKernelMap failed!!!");
    return false;
  }

  // compute the fourier transform of the kernel
  fft2(k,kf);

  // calculate filter response
  calcResponse(alphaf,kf,response, spec);

  ret = extractCovar(response, exp(-0.5f), corrMean, corrCovar, corrEigVal, corrEigVec);
  if (!ret) 
  {
     TRACE_ERR("\nDid not get valid response!!!");
  }

  // extract the maximum response
  minMaxLoc(response, &minVal, &maxVal, &minLoc, &maxLoc );
  confidence = maxVal;
  TRACE_INFO("\nMax response:%f, loc(%d, %d)\n", maxVal, maxLoc.x, maxLoc.y);

  if (debug)
    drawDetectProcess(roi_scale, x, z, k, response, alphaf);

//  roi_scale.x+=(maxLoc.x-roi_scale.width/2.0f);
//  roi_scale.y+=(maxLoc.y-roi_scale.height/2.0f);

  cv::Point2d tl(maxLoc.x-roi_scale.width/2.0f, maxLoc.y-roi_scale.height/2.0f);
  cv::Point2d tr(maxLoc.x+roi_scale.width/2.0f, maxLoc.y-roi_scale.height/2.0f);
  cv::Point2d br(maxLoc.x+roi_scale.width/2.0f, maxLoc.y+roi_scale.height/2.0f);
  cv::Point2d bl(maxLoc.x-roi_scale.width/2.0f, maxLoc.y+roi_scale.height/2.0f);
  if (resizeImage)
  {
    tl += roi_scale.tl();
    tr += roi_scale.tl();
    bl += roi_scale.tl();
    br += roi_scale.tl();

    tl*=resizeRatio;
    tr*=resizeRatio;
    bl*=resizeRatio;
    br*=resizeRatio;
  }

  if (maxVal < params.detect_thresh)
  {
     TRACE_ERR("\nDetect end: Fail!!!");
     return false;
  }

  float new_centra_x = maxLoc.x;
  float new_centra_y = maxLoc.y;
  if (resizeImage)
  {
    new_centra_x*=resizeRatio;
    new_centra_y*=resizeRatio;
  }

  float shift_x = boundingBox.width/2.0f - new_centra_x;
  float shift_y = boundingBox.height/2.0f - new_centra_y;
  if (resizeImage)
  {
    if (fabs(shift_x) < resizeRatio)
      shift_x = 0;
    if (fabs(shift_y) < resizeRatio)
      shift_y = 0;
  }
   
  // update the bounding box
  boundingBox.x = boundingBox.x - shift_x;
  boundingBox.y = boundingBox.y - shift_y;
  
  TRACE_INFO("\nshift_x:%f, shift_y:%f", shift_x, shift_y);


  //correct centra point
  if (kalman_enable)
  {
    cv::Mat bcentra = cv::Mat::zeros(2, 1, CV_32F);
    bcentra.at<float>(0) = boundingBox.x + boundingBox.width/2;
    bcentra.at<float>(1) = boundingBox.y + boundingBox.height/2;
    kalman.correct(bcentra, corrCovar);
  }

#ifndef NDEBUG
  std::cout << "\nDetect output reponse:" << boundingBox << std::endl;
  cv::Point2d centra = (tl+tr+bl+br)/4.0f;
  std::cout << "\nScale centra point:" << centra << std::endl;
#endif

  TRACE_INFO("\nDetect end: Success!\n");

  return true;
}

void TrackerKCFImpl::drawDetectProcess(cv::Rect2d u_roi, Mat& feature, Mat& base, Mat& kernel, Mat& response, Mat& alpha_fft)
{
  int width = u_roi.width;
  int height = u_roi.height;

  cv::Mat showImage(2*height, 4*width, CV_32F, cv::Scalar(255));

  cv::Mat res_show = showImage(cv::Rect2d(width, height,
                                           width, height));
  cv::resize(response, res_show, res_show.size(), 0, 0, cv::INTER_LINEAR);
  {
    cv::Mat eigVal = corrEigVal.clone();
    cv::Mat eigVec= corrEigVec.clone();

    cv::sqrt(eigVal, eigVal);
    Point2d center(corrMean.at<float>(0), corrMean.at<float>(1));
    // 9.210340f from Chi-square distribution
    eigVal = eigVal*sqrt(9.210340f);
    cv::Point pa_major;
    pa_major.x = center.x + eigVal.at<float>(0)*eigVec.row(0).at<float>(0); 
    pa_major.y = center.y + eigVal.at<float>(0)*eigVec.row(0).at<float>(1);
    cv::Point pb_major;
    pb_major.x = center.x + eigVal.at<float>(1)*eigVec.row(1).at<float>(0);
    pb_major.y = center.y + eigVal.at<float>(1)*eigVec.row(1).at<float>(1);

    line(res_show, center, pa_major, Scalar(255), 1, LINE_AA);
    line(res_show, center, pb_major, Scalar(255), 1, LINE_AA);

    cv::Point maxLoc;
    minMaxLoc(response, NULL, NULL, NULL, &maxLoc );
    cv::circle(res_show, maxLoc, 4,  Scalar(0));

   }
   
  cv::Mat res_thd = showImage(cv::Rect2d(0, 0, width, height));
  cv::resize(res_show, res_thd, res_thd.size(), 0, 0, cv::INTER_LINEAR);
  for (int i=0; i<res_thd.rows; i++)
    for (int j=0; j<res_thd.cols; j++)
    {
      if (res_thd.at<float>(i, j) < params.detect_thresh)
       res_thd.at<float>(i, j) = 0; 
    }

  cv::Rect2d disp_x(2*width, height, width, height);
  drawFeature(disp_x, feature, showImage);

  cv::Rect2d disp_z(2*width, 0, width, height);
  drawFeature(disp_z, base, showImage);

  cv::Mat kernel_show = showImage(cv::Rect2d(0, height, width, height));
  cv::normalize(k, kernel_show, 0.0f, 1.0f, cv::NORM_MINMAX);
  Mat k_eigVal, k_eigVec, k_corr, k_mean;
  bool ret = extractCovar(kernel_show, exp(-0.5f), k_mean, k_corr, k_eigVal, k_eigVec);
  if (ret)
  {
    cv::sqrt(k_eigVal, k_eigVal);
    k_eigVal = k_eigVal*sqrt(9.210340f);

    Point2d center(k_mean.at<float>(0), k_mean.at<float>(1));
    cv::Point pa_major;
    pa_major.x = center.x + k_eigVal.at<float>(0)*k_eigVec.row(0).at<float>(0); 
    pa_major.y = center.y + k_eigVal.at<float>(0)*k_eigVec.row(0).at<float>(1);
    cv::Point pb_major;
    pb_major.x = center.x + k_eigVal.at<float>(1)*k_eigVec.row(1).at<float>(0);
    pb_major.y = center.y + k_eigVal.at<float>(1)*k_eigVec.row(1).at<float>(1);

    line(kernel_show, center, pa_major, Scalar(255, 0, 0), 1, LINE_AA);
    line(kernel_show, center, pb_major, Scalar(255, 0, 0), 1, LINE_AA);
  }

  cv::Mat alpha_show = showImage(cv::Rect2d(width, 0,
                                           width, height));
  Mat alpha_tmp;
  ifft2(alpha_fft,alpha_tmp);
  cv::normalize(alpha_tmp, alpha_tmp, 0.0f, 1.0f, cv::NORM_MINMAX);
  cv::resize(alpha_tmp, alpha_show, alpha_show.size(), 0, 0, cv::INTER_LINEAR);

  imshow("detection process", showImage);
}


/*
 * Main part of the KCF algorithm
 */
bool TrackerKCFImpl::updateWithDetectImpl(const Mat& imageDet, Rect2d& detBox, const Mat& imageTrack, Rect2d& trackBox, float confidence, bool debug) {

  bool template_scale = false;
  cv::Rect2d roi_track;
  cv::Rect2d roi_detect;
  cv::Mat x_d, x_t;
  cv::Mat k_d, k_t;
  cv::Mat k_df, k_tf;
  cv::Mat alpha_df;

  cv::Point track_centra = (trackBox.tl() + trackBox.br())/2;

  if (!isInit) return false;

  cv::Rect2d img_rect(0, 0, imageDet.cols, imageDet.rows);

  cv::Rect2d overlap = img_rect & detBox;
  if (overlap.area() <= 0)
  {
    TRACE_ERR("\nUpdate: bounding box out of image area!!!");
    return false;
  }

  float centra_x = (float)detBox.x + detBox.width/2.0f;
  float centra_y = (float)detBox.y + detBox.height/2.0f;
  roi_detect.width = detBox.width*paddingRatio;
  roi_detect.height = detBox.height*paddingRatio;
  roi_detect.x = centra_x - roi_detect.width/2;
  roi_detect.y = centra_y - roi_detect.height/2;

  if (resizeImage)
  {
    resizeRatio = sqrt((roi_detect.width*roi_detect.height)/params.max_patch_size);
    if (resizeRatio < 1.0f) resizeRatio = 1.0f;

    TRACE_INFO("\nresizeRatio:%f", resizeRatio);

    roi_detect.x /= resizeRatio;
    roi_detect.y /= resizeRatio;
    roi_detect.width /= resizeRatio;
    roi_detect.height /= resizeRatio;

    track_centra /= resizeRatio;
  }

  roi_track.width = roi_detect.width;
  roi_track.height = roi_detect.height;
  roi_track.x = track_centra.x - roi_track.width/2;
  roi_track.y = track_centra.y - roi_track.height/2;

  if (roi.size() != roi_detect.size())
  {
    template_scale = true;
    roi = roi_detect;
    createHanningWindow(hann, cv::Size(cvRound(roi.width), cvRound(roi.height)), CV_32F);
    Mat _layer[] = {hann, hann, hann, hann, hann, hann, hann, hann, hann, hann};
    merge(_layer, 10, hann_cn);
  }

  Mat imgDet = imageDet.clone();
  Mat imgTrack = imageTrack.clone();

  if(resizeImage)
  {
    cv::resize(imgDet,imgDet,Size(imgDet.cols/resizeRatio,imgDet.rows/resizeRatio),0,0,cv::INTER_LINEAR);
    cv::resize(imgTrack,imgTrack,Size(imgTrack.cols/resizeRatio,imgTrack.rows/resizeRatio),0,0,cv::INTER_LINEAR);
  }

  // check the channels of the input image, grayscale is preferred
  CV_Assert(imgDet.channels() == 1 || imgDet.channels() == 3);
  CV_Assert(imgTrack.channels() == 1 || imgTrack.channels() == 3);

  extractFeature(imgDet, roi_detect, x_d);
  extractFeature(imgTrack, roi_track, x_t);
  cv::Size feature_size = x_d.size();

  if(x_d.empty() || x_t.empty())
  {
    return false;
  }

  if (confidence > 1.0f)
    confidence = 0.999f;
  else if (confidence < 0.0f)
    confidence = 0.0f;

  if(template_scale )
  {
    constructGaussian(feature_size, y);
    fft2(y,yf);
  }

  extractKernelMap(x_d, x_d, k_d);
  fft2(k_d, k_df);

  float den;
  alpha_df = k_df.clone();
  for(int i=0;i<yf.rows;i++){
    for(int j=0;j<yf.cols;j++){
      den = 1.0f/(k_df.at<Vec2f>(i,j)[0]*k_df.at<Vec2f>(i,j)[0]+k_df.at<Vec2f>(i,j)[1]*k_df.at<Vec2f>(i,j)[1]);

      alpha_df.at<Vec2f>(i,j)[0]=
      (yf.at<Vec2f>(i,j)[0]*k_df.at<Vec2f>(i,j)[0]+yf.at<Vec2f>(i,j)[1]*k_df.at<Vec2f>(i,j)[1])*den;
      alpha_df.at<Vec2f>(i,j)[1]=
      (yf.at<Vec2f>(i,j)[1]*k_df.at<Vec2f>(i,j)[0]-yf.at<Vec2f>(i,j)[0]*k_df.at<Vec2f>(i,j)[1])*den;
    }
  }

  cv::Mat res;
  extractKernelMap(x_t, x_d, k_t);
  fft2(k_t, k_tf);
  calcResponse(alpha_df,k_tf,res, spec);

  Point minLoc, maxLoc;
  double minVal, maxVal;
  minMaxLoc(res, &minVal, &maxVal, &minLoc, &maxLoc);

  if (maxVal > 1.0f)
    maxVal = 1.0f;

  if (maxVal < params.detect_thresh)
  {
    TRACE_ERR("\nUpdate with detect: Failed!!!");
    return false;
  }

  extractCovar(res, exp(-0.5f), corrMean, corrCovar, corrEigVal, corrEigVec);

  cv::Mat z_orig = z.clone();

  roi_track.x+=(maxLoc.x-roi_track.width/2);
  roi_track.y+=(maxLoc.y-roi_track.height/2);

  extractFeature(imgTrack, roi_track, z);
  extractKernelMap(z, z, k);

  // compute the fourier transform of the kernel
  fft2(k, kf);

  alphaf=Mat_<Vec2f >(yf.rows, yf.cols);
  for(int i=0;i<yf.rows;i++){
    for(int j=0;j<yf.cols;j++){
      den = 1.0f/(kf.at<Vec2f>(i,j)[0]*kf.at<Vec2f>(i,j)[0]+kf.at<Vec2f>(i,j)[1]*kf.at<Vec2f>(i,j)[1]);

      alphaf.at<Vec2f>(i,j)[0]=
      (yf.at<Vec2f>(i,j)[0]*kf.at<Vec2f>(i,j)[0]+yf.at<Vec2f>(i,j)[1]*kf.at<Vec2f>(i,j)[1])*den;
      alphaf.at<Vec2f>(i,j)[1]=
      (yf.at<Vec2f>(i,j)[1]*kf.at<Vec2f>(i,j)[0]-yf.at<Vec2f>(i,j)[0]*kf.at<Vec2f>(i,j)[1])*den;
    }
  }

  track_centra = (roi_track.tl() + roi_track.br())/2;

  if (resizeImage)
  {
    track_centra *= resizeRatio;
  }

  trackBox = detBox;
  trackBox.x = cvRound(track_centra.x - trackBox.width/2);
  trackBox.y = cvRound(track_centra.y - trackBox.height/2);

  if (debug)
    drawUpdateWithDetProcess(roi_detect, x_d, z_orig, k_t, res,z, alphaf);

  return true;
}

void TrackerKCFImpl::drawUpdateWithDetProcess(cv::Rect2d u_roi, Mat& feature, Mat& base, Mat& kernel, Mat& response, Mat& new_base, Mat& alpha_fft)
{
  int width = u_roi.width;
  int height = u_roi.height;

  cv::Mat showImage(3*height, 3*width, CV_32F, cv::Scalar(255));

  cv::Rect2d disp_z(0, 0, width, height);
  drawFeature(disp_z, base, showImage);

  cv::Rect2d disp_x(0, height, width, height);
  drawFeature(disp_x, feature, showImage);

  cv::Rect2d disp_new(0, 2*height, width, height);
  drawFeature(disp_new, new_base, showImage);

  cv::Rect2d disp_res(2*width, 0, width, height);
  cv::Mat res_show = showImage(disp_res);
  cv::resize(response, res_show, disp_res.size(), 0, 0, cv::INTER_LINEAR);
  cv::Point maxLoc;
  minMaxLoc(response, NULL, NULL, NULL, &maxLoc );
  cv::circle(res_show, maxLoc, 4,  Scalar(0));

  cv::Mat kernel_show = showImage(cv::Rect2d(2*width, height, width, height));
  cv::normalize(k, kernel_show, 0.0f, 1.0f, cv::NORM_MINMAX);
  Mat k_eigVal, k_eigVec, k_corr, k_mean;
  bool ret = extractCovar(kernel_show, exp(-0.5f), k_mean, k_corr, k_eigVal, k_eigVec);
  if (ret)
  {
    cv::sqrt(k_eigVal, k_eigVal);
    k_eigVal = k_eigVal*sqrt(9.210340f);

    Point2d center(k_mean.at<float>(0), k_mean.at<float>(1));
    cv::Point pa_major;
    pa_major.x = center.x + k_eigVal.at<float>(0)*k_eigVec.row(0).at<float>(0); 
    pa_major.y = center.y + k_eigVal.at<float>(0)*k_eigVec.row(0).at<float>(1);
    cv::Point pb_major;
    pb_major.x = center.x + k_eigVal.at<float>(1)*k_eigVec.row(1).at<float>(0);
    pb_major.y = center.y + k_eigVal.at<float>(1)*k_eigVec.row(1).at<float>(1);

    line(kernel_show, center, pa_major, Scalar(255, 0, 0), 1, LINE_AA);
    line(kernel_show, center, pb_major, Scalar(255, 0, 0), 1, LINE_AA);

    line(kernel_show, cv::Point(width/2, 0), cv::Point(width/2, height), Scalar(255, 0, 0), 1, LINE_AA);
    line(kernel_show, cv::Point(0, height/2), cv::Point(width, height/2), Scalar(255, 0, 0), 1, LINE_AA);
  }

  cv::Mat alpha_show = showImage(cv::Rect2d(2*width, 2*height,
                                           width, height));
  Mat alpha_tmp;
  ifft2(alpha_fft,alpha_tmp);
  cv::normalize(alpha_tmp, alpha_tmp, 0.0f, 1.0f, cv::NORM_MINMAX);
  cv::resize(alpha_tmp, alpha_show, alpha_show.size(), 0, 0, cv::INTER_LINEAR);

  imshow("update with detection process", showImage);
}

/*
 * Main part of the KCF algorithm
 */
bool TrackerKCFImpl::updateWithTrackImpl(const Mat& image, Rect2d& boundingBox, float confidence, bool debug)
{

  bool template_scale = false;

  if (!isInit) return false;

  cv::Rect2d img_rect(0, 0, image.cols, image.rows);

  cv::Rect2d overlap = img_rect & boundingBox;
  if (overlap.area() <= 0)
  {
    return false;
  }

  float centra_x = (float)boundingBox.x + boundingBox.width/2.0f;
  float centra_y = (float)boundingBox.y + boundingBox.height/2.0f;
  roi_scale.width = boundingBox.width*paddingRatio;
  roi_scale.height = boundingBox.height*paddingRatio;
  roi_scale.x = centra_x - roi_scale.width/2;
  roi_scale.y = centra_y - roi_scale.height/2;

  if (resizeImage)
  {
    roi_scale.x /= resizeRatio;
    roi_scale.y /= resizeRatio;
    roi_scale.width /= resizeRatio;
    roi_scale.height /= resizeRatio;
  }

  if (roi.size() != roi_scale.size())
  {
    template_scale = true;
    roi_scale.width = roi.width;
    roi_scale.height = roi.height;
  }

  Mat img=image.clone();
  if(resizeImage)
    cv::resize(img,img,Size(img.cols/resizeRatio,img.rows/resizeRatio),0,0,cv::INTER_LINEAR);
  // check the channels of the input image, grayscale is preferred
  CV_Assert(img.channels() == 1 || img.channels() == 3);
//  CV_Assert(template_scale == false);

  extractFeature(img, roi_scale, x);
  cv::Size feature_size = x.size();

  if(z.empty() || x.empty())
  {
    return false;
  }

  if (confidence > 1.0f)
    confidence = 0.999f;
  else if (confidence < 0.0f)
    confidence = 0.0f;

  // make composed model to be base model
  cv::Mat z_orig = z.clone();

  z= x*(1-confidence) + z*confidence;

#if 0
  extractKernelMap(z, z, k);

  // compute the fourier transform of the kernel
  fft2(k, kf);

  float den;
  new_alphaf = kf.clone();
  for(int i=0;i<yf.rows;i++){
    for(int j=0;j<yf.cols;j++){
      den = 1.0f/(kf.at<Vec2f>(i,j)[0]*kf.at<Vec2f>(i,j)[0]+kf.at<Vec2f>(i,j)[1]*kf.at<Vec2f>(i,j)[1]);

      new_alphaf.at<Vec2f>(i,j)[0]=
      (yf.at<Vec2f>(i,j)[0]*kf.at<Vec2f>(i,j)[0]+yf.at<Vec2f>(i,j)[1]*kf.at<Vec2f>(i,j)[1])*den;
      new_alphaf.at<Vec2f>(i,j)[1]=
      (yf.at<Vec2f>(i,j)[1]*kf.at<Vec2f>(i,j)[0]-yf.at<Vec2f>(i,j)[0]*kf.at<Vec2f>(i,j)[1])*den;
    }
  }
  alphaf=new_alphaf;

  if (debug)
    drawUpdateWithTrackProcess(roi_scale, x, z_orig, z, alphaf);
#endif
  return true;
}

void TrackerKCFImpl::drawUpdateWithTrackProcess(cv::Rect2d u_roi, Mat& feature, Mat& base, Mat& new_base, Mat& alpha_fft)
{
  int width = u_roi.width;
  int height = u_roi.height;

  cv::Mat showImage(3*height, 3*width, CV_32F, cv::Scalar(255));

  cv::Rect2d disp_z(0, 0, width, height);
  drawFeature(disp_z, base, showImage);

  cv::Rect2d disp_x(0, height, width, height);
  drawFeature(disp_x, feature, showImage);

  cv::Rect2d disp_new(0, 2*height, width, height);
  drawFeature(disp_new, new_base, showImage);

  cv::Mat alpha_show = showImage(cv::Rect2d(2*width, 2*height,
                                           width, height));
  Mat alpha_tmp;
  ifft2(alpha_fft,alpha_tmp);
  cv::normalize(alpha_tmp, alpha_tmp, 0.0f, 1.0f, cv::NORM_MINMAX);
  cv::resize(alpha_tmp, alpha_show, alpha_show.size(), 0, 0, cv::INTER_LINEAR);

  imshow("update with track process", showImage);
}

/*-------------------------------------
|  implementation of the KCF functions
|-------------------------------------*/

/*
 * hann window filter
 */
void TrackerKCFImpl::createHanningWindow(OutputArray dest, const cv::Size winSize, const int type) const {
    CV_Assert( type == CV_32FC1 || type == CV_64FC1 );

    dest.create(winSize, type);
    Mat dst = dest.getMat();

    int rows = dst.rows, cols = dst.cols;

    AutoBuffer<float> _wc(cols);
    float * const wc = (float *)_wc;

    const float coeff0 = 2.0f * (float)CV_PI / (cols - 1);
    const float coeff1 = 2.0f * (float)CV_PI / (rows - 1);
    for(int j = 0; j < cols; j++)
      wc[j] = 0.5f * (1.0f - cos(coeff0 * j));

    if(dst.depth() == CV_32F){
      for(int i = 0; i < rows; i++){
        float* dstData = dst.ptr<float>(i);
        float wr = 0.5f * (1.0f - cos(coeff1 * i));
        for(int j = 0; j < cols; j++)
          dstData[j] = (float)(wr * wc[j]);
      }
    }else{
      for(int i = 0; i < rows; i++){
        double* dstData = dst.ptr<double>(i);
        double wr = 0.5f * (1.0f - cos(coeff1 * i));
        for(int j = 0; j < cols; j++)
          dstData[j] = wr * wc[j];
      }
    }

    // perform batch sqrt for SSE performance gains
    //cv::sqrt(dst, dst); //matlab do not use the square rooted version
}

/*
 * simplification of fourier transform function in opencv
 */
void inline TrackerKCFImpl::fft2(const Mat src, Mat & dest) const {
  dft(src,dest,DFT_COMPLEX_OUTPUT);
}

void inline TrackerKCFImpl::fft2(const Mat src, std::vector<Mat> & dest, std::vector<Mat> & layers_data) const {
  split(src, layers_data);

  for(int i=0;i<src.channels();i++){
    dft(layers_data[i],dest[i],DFT_COMPLEX_OUTPUT);
  }
}

/*
 * simplification of inverse fourier transform function in opencv
 */
void inline TrackerKCFImpl::ifft2(const Mat src, Mat & dest) const {
  idft(src,dest,DFT_SCALE+DFT_REAL_OUTPUT);
}

/*
 * Point-wise multiplication of two Multichannel Mat data
 */
void inline TrackerKCFImpl::pixelWiseMult(const std::vector<Mat> src1, const std::vector<Mat>  src2, std::vector<Mat>  & dest, const int flags, const bool conjB) const {
  for(unsigned i=0;i<src1.size();i++){
    mulSpectrums(src1[i], src2[i], dest[i],flags,conjB);
  }
}

/*
 * Combines all channels in a multi-channels Mat data into a single channel
 */
void inline TrackerKCFImpl::sumChannels(std::vector<Mat> src, Mat & dest) const {
  dest=src[0].clone();
  for(unsigned i=1;i<src.size();i++){
    dest+=src[i];
  }
}

#ifdef HAVE_OPENCL
bool inline TrackerKCFImpl::oclTransposeMM(const Mat src, float alpha, UMat &dst){
  // Current kernel only support matrix's rows is multiple of 4.
  // And if one line is less than 512KB, CPU will likely be faster.
  if (transpose_mm_ker.empty() ||
      src.rows % 4 != 0 ||
      (src.rows * 10) < (1024 * 1024 / 4))
    return false;

  Size s(src.rows, src.cols);
  const Mat tmp = src.t();
  const UMat uSrc = tmp.getUMat(ACCESS_READ);
  transpose_mm_ker.args(
      ocl::KernelArg::PtrReadOnly(uSrc),
      (int)uSrc.rows,
      (int)uSrc.cols,
      alpha,
      ocl::KernelArg::PtrWriteOnly(dst));
  size_t globSize[2] = {static_cast<size_t>(src.cols * 64), static_cast<size_t>(src.cols)};
  size_t localSize[2] = {64, 1};
  if (!transpose_mm_ker.run(2, globSize, localSize, true))
    return false;
  return true;
}
#endif

/*
 * obtains the projection matrix using PCA
 */
void inline TrackerKCFImpl::updateProjectionMatrix(const Mat src, Mat & old_cov,Mat &  proj_matrix, float pca_rate, int compressed_sz,
                                                   std::vector<Mat> & layers_pca,std::vector<Scalar> & average, Mat pca_data, Mat new_cov, Mat w, Mat u, Mat vt) {
  CV_Assert(compressed_sz<=src.channels());

  split(src,layers_pca);

  for (int i=0;i<src.channels();i++){
    average[i]=mean(layers_pca[i]);
    layers_pca[i]-=average[i];
  }

  // calc covariance matrix
  merge(layers_pca,pca_data);
  pca_data=pca_data.reshape(1,src.rows*src.cols);

#ifdef HAVE_OPENCL
  bool oclSucceed = false;
  Size s(pca_data.cols, pca_data.cols);
  UMat result(s, pca_data.type());
  if (oclTransposeMM(pca_data, 1.0f/(float)(src.rows*src.cols-1), result)) {
    if(old_cov.rows==0) old_cov=result.getMat(ACCESS_READ).clone();
    SVD::compute((1.0-pca_rate)*old_cov + pca_rate * result.getMat(ACCESS_READ), w, u, vt);
    oclSucceed = true;
  }
#define TMM_VERIFICATION 0

  if (oclSucceed == false || TMM_VERIFICATION) {
    new_cov=1.0f/(float)(src.rows*src.cols-1)*(pca_data.t()*pca_data);
#if TMM_VERIFICATION
    for(int i = 0; i < new_cov.rows; i++)
      for(int j = 0; j < new_cov.cols; j++)
        if (abs(new_cov.at<float>(i, j) - result.getMat(ACCESS_RW).at<float>(i , j)) > abs(new_cov.at<float>(i, j)) * 1e-3)
          printf("error @ i %d j %d got %G expected %G \n", i, j, result.getMat(ACCESS_RW).at<float>(i , j), new_cov.at<float>(i, j));
#endif
    if(old_cov.rows==0)old_cov=new_cov.clone();
    SVD::compute((1.0f - pca_rate) * old_cov + pca_rate * new_cov, w, u, vt);
  }
#else
  new_cov=1.0/(float)(src.rows*src.cols-1)*(pca_data.t()*pca_data);
  if(old_cov.rows==0)old_cov=new_cov.clone();

  // calc PCA
  SVD::compute((1.0-pca_rate)*old_cov+pca_rate*new_cov, w, u, vt);
#endif
  // extract the projection matrix
  proj_matrix=u(Rect2d(0,0,compressed_sz,src.channels())).clone();
  Mat proj_vars=Mat::eye(compressed_sz,compressed_sz,proj_matrix.type());
  for(int i=0;i<compressed_sz;i++){
    proj_vars.at<float>(i,i)=w.at<float>(i);
  }

  // update the covariance matrix
  old_cov=(1.0-pca_rate)*old_cov+pca_rate*proj_matrix*proj_vars*proj_matrix.t();
}

/*
 * compress the features
 */
void inline TrackerKCFImpl::compress(const Mat proj_matrix, const Mat src, Mat & dest, Mat & data, Mat & compressed) const {
  data=src.reshape(1,src.rows*src.cols);
  compressed=data*proj_matrix;
  dest=compressed.reshape(proj_matrix.cols,src.rows).clone();
}

/*
 * obtain the patch and apply hann window filter to it
 */
bool TrackerKCFImpl::getSubWindow(const Mat img, const Rect2d _roi, Mat& feat, Mat& patch, MODE desc) const {

  Rect2d region=_roi;

  // return false if roi is outside the image
  if((_roi & Rect2d(0,0, img.cols, img.rows)) == Rect2d() )
      return false;

  // extract patch inside the image
  if(_roi.x<0){region.x=0;region.width+=_roi.x;}
  if(_roi.y<0){region.y=0;region.height+=_roi.y;}
  if(_roi.x+_roi.width>img.cols)region.width=img.cols-_roi.x;
  if(_roi.y+_roi.height>img.rows)region.height=img.rows-_roi.y;
  if(region.width>img.cols)region.width=img.cols;
  if(region.height>img.rows)region.height=img.rows;

  patch=img(region).clone();

  // add some padding to compensate when the patch is outside image border
  int addTop,addBottom, addLeft, addRight;
  addTop=region.y-_roi.y;
  addBottom=(_roi.height+_roi.y>img.rows?_roi.height+_roi.y-img.rows:0);
  addLeft=region.x-_roi.x;
  addRight=(_roi.width+_roi.x>img.cols?_roi.width+_roi.x-img.cols:0);

  copyMakeBorder(patch,patch,addTop,addBottom,addLeft,addRight,BORDER_REPLICATE);
  if(patch.rows==0 || patch.cols==0)return false;

  // extract the desired descriptors
  switch(desc){
    case CN:
      CV_Assert(img.channels() == 3);
      extractCN(patch,feat);
      //feat=feat.mul(hann_cn); // hann window filter
      break;
    case HOG:
      CV_Assert(img.channels() == 3);
      extractHOG(patch,feat);
      break;

    default: // GRAY
      if(img.channels()>1)
        cvtColor(patch,feat, CV_BGR2GRAY);
      else
        feat=patch;
      //feat.convertTo(feat,CV_32F);
        feat.convertTo(feat,CV_32F, 1.0/255.0, -0.5);
      //feat=feat/255.0-0.5; // normalize to range -0.5 .. 0.5
       // feat=feat.mul(hann); // hann window filter
      break;
  }

  return true;

}

/* Convert BGR to ColorNames
 */
void TrackerKCFImpl::extractCN(Mat patch_data, Mat & cnFeatures) const {
  Vec3b & pixel = patch_data.at<Vec3b>(0,0);
  unsigned index;

  if(cnFeatures.type() != CV_32FC(10))
    cnFeatures = Mat::zeros(patch_data.rows,patch_data.cols,CV_32FC(10));

  if(cnFeatures.size() != patch_data.size())
    resize(cnFeatures, cnFeatures, patch_data.size());

  for(int i=0;i<patch_data.rows;i++){
    for(int j=0;j<patch_data.cols;j++){
      pixel=patch_data.at<Vec3b>(i,j);
      index=(unsigned)(floor((float)pixel[2]/8)+32*floor((float)pixel[1]/8)+32*32*floor((float)pixel[0]/8));

      //copy the values
      for(int _k=0;_k<10;_k++){
        cnFeatures.at<Vec<float,10> >(i,j)[_k]=ColorNames[index][_k];
      }
    }
  }

}

/* Convert BGR to HOG features 
 */
void TrackerKCFImpl::extractHOG(Mat &im, Mat& hogFeatures) const
{
    std::vector<float> descriptors;
    cv::Size img_size = im.size();
    cv::Size block_size(16,16);
    cv::Size block_stride(1,1);
    cv::Size cell_size(2,2);
    cv::Size block_res(block_size.width/cell_size.width, block_size.height/cell_size.height);
    cv::Size blocks(cvCeil((img_size.width-block_size.width)/block_stride.width + 1), cvCeil((img_size.height-block_size.height)/block_stride.height + 1));
    int rows = blocks.height*block_res.height;


    HOGDescriptor hog(img_size,block_size,block_stride,cell_size,9);
    hog.compute(im, descriptors);
    hogFeatures = cv::Mat(descriptors).reshape(9, rows).clone();
}


/*
 * get feature using external function
 */
bool TrackerKCFImpl::getSubWindow(const Mat img, const Rect2d _roi, Mat& feat, void (*f)(const Mat, const Rect2d, Mat& )) const{

  // return false if roi is outside the image
  if((_roi.x+_roi.width<0)
    ||(_roi.y+_roi.height<0)
    ||(_roi.x>=img.cols)
    ||(_roi.y>=img.rows)
  )return false;

  f(img, _roi, feat);

  if(_roi.width != feat.cols || _roi.height != feat.rows){
    printf("error in customized function of features extractor!\n");
    printf("Rules: roi.width==feat.cols && roi.height = feat.rows \n");
  }

  Mat hann_win;
  std::vector<Mat> _layers;

  for(int i=0;i<feat.channels();i++)
    _layers.push_back(hann);

  merge(_layers, hann_win);

  feat=feat.mul(hann_win); // hann window filter

  return true;
}

/*
 *  dense gauss kernel function
 */
void TrackerKCFImpl::denseGaussKernel(const float sigma, const Mat x_data, const Mat y_data, Mat & k_data,
                                      std::vector<Mat> & layers_data,std::vector<Mat> & xf_data,std::vector<Mat> & yf_data, std::vector<Mat> xyf_v, Mat xy, Mat xyf ) const {
  double normX, normY;

  fft2(x_data,xf_data,layers_data);
  fft2(y_data,yf_data,layers_data);

  normX=norm(x_data);
  normX*=normX;
  normY=norm(y_data);
  normY*=normY;

  pixelWiseMult(xf_data,yf_data,xyf_v,0,true);
  sumChannels(xyf_v,xyf);
  ifft2(xyf,xyf);

  if(params.wrap_kernel){
    shiftRows(xyf, x_data.rows/2);
    shiftCols(xyf, x_data.cols/2);
  }

  //(xx + yy - 2 * xy) / numel(x)
  xy=(normX+normY-2*xyf)/(x_data.rows*x_data.cols*x_data.channels());

  // TODO: check wether we really need thresholding or not
  //threshold(xy,xy,0.0,0.0,THRESH_TOZERO);//max(0, (xx + yy - 2 * xy) / numel(x))
  for(int i=0;i<xy.rows;i++){
    for(int j=0;j<xy.cols;j++){
      if(xy.at<float>(i,j)<0.0)xy.at<float>(i,j)=0.0;
    }
  }

  float sig=-1.0f/(sigma*sigma);
  xy=sig*xy;
  exp(xy,k_data);

}

/* CIRCULAR SHIFT Function
 * http://stackoverflow.com/questions/10420454/shift-like-matlab-function-rows-or-columns-of-a-matrix-in-opencv
 */
// circular shift one row from up to down
void TrackerKCFImpl::shiftRows(Mat& mat) const {

    Mat temp;
    Mat m;
    int _k = (mat.rows-1);
    mat.row(_k).copyTo(temp);
    for(; _k > 0 ; _k-- ) {
      m = mat.row(_k);
      mat.row(_k-1).copyTo(m);
    }
    m = mat.row(0);
    temp.copyTo(m);

}

// circular shift n rows from up to down if n > 0, -n rows from down to up if n < 0
void TrackerKCFImpl::shiftRows(Mat& mat, int n) const {
    if( n < 0 ) {
      n = -n;
      flip(mat,mat,0);
      for(int _k=0; _k < n;_k++) {
        shiftRows(mat);
      }
      flip(mat,mat,0);
    }else{
      for(int _k=0; _k < n;_k++) {
        shiftRows(mat);
      }
    }
}

//circular shift n columns from left to right if n > 0, -n columns from right to left if n < 0
void TrackerKCFImpl::shiftCols(Mat& mat, int n) const {
    if(n < 0){
      n = -n;
      flip(mat,mat,1);
      transpose(mat,mat);
      shiftRows(mat,n);
      transpose(mat,mat);
      flip(mat,mat,1);
    }else{
      transpose(mat,mat);
      shiftRows(mat,n);
      transpose(mat,mat);
    }
}

/*
 * calculate the detection response
 */
void TrackerKCFImpl::calcResponse(const Mat alphaf_data, const Mat kf_data, Mat & response_data, Mat & spec_data) const {
  //alpha f--> 2channels ; k --> 1 channel;
  mulSpectrums(alphaf_data,kf_data,spec_data,0,false);
  ifft2(spec_data,response_data);
}


void TrackerKCFImpl::setFeatureExtractor(void (*f)(const Mat, const Rect2d, Mat&), bool pca_func){
  if(pca_func){
    extractor_pca.push_back(f);
    use_custom_extractor_pca = true;
  }else{
    extractor_npca.push_back(f);
    use_custom_extractor_npca = true;
  }
}

