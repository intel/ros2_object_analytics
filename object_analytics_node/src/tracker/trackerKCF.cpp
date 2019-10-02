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

#include "object_analytics_node/tracker/trackerKCF.hpp"
#include "object_analytics_node/filter/kalman.hpp"

/*---------------------------
|  TrackerKCF
|---------------------------*/

/*
* Parameters
*/
Params::Params(){
    detect_thresh = 0.60653f;
//    detect_thresh = 0.5f;
    sigma=0.2f;
    lambda=0.0000f;
    interp_factor=0.2f;
    output_sigma_factor=1.0f / 16.0f;
    resize=true;
    max_patch_size=40*40;
    wrap_kernel=true;
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
  use_custom_extractor_pca = false;
  use_custom_extractor_npca = false;
  kalman_enable = false;
  scale_update_enable = false;
}

/*
 * Initialization:
 * - creating hann window filter
 * - ROI padding
 * - creating a gaussian response for the training ground-truth
 * - perform FFT to the gaussian response
 */
bool TrackerKCFImpl::initImpl( const Mat& image, Rect& boundingBox ){
  frame=0;

  Rect image_area(0, 0, image.cols, image.rows);

  roi = boundingBox;

  std::cout << "Initialize boundingbox\n" 
    << boundingBox
    << "\n"
    << "roi"
    << roi
    << "\n"
    << std::endl;


  //imshow("initial image", image(boundingBox));
  //calclulate output sigma
  output_sigma=std::sqrt(static_cast<float>(roi.width*roi.height))*params.output_sigma_factor;
  std::cout << "output_sigma :\t" << output_sigma<< std::endl;
  std::cout << "output_sigma squal:\t" << output_sigma*output_sigma << std::endl;

  float disThresh = output_sigma*output_sigma;
  output_sigma=-0.5f/(output_sigma*output_sigma);

  //resize the ROI whenever needed
  if(params.resize && roi.width*roi.height>params.max_patch_size){
    resizeImage=true;
    roi.x/=2.0;
    roi.y/=2.0;
    roi.width/=2.0;
    roi.height/=2.0;
  }

  // initialize the hann window filter
  createHanningWindow(hann, roi.size(), CV_32F);

  // hann window filter for CN feature
  Mat _layer[] = {hann, hann, hann, hann, hann, hann, hann, hann, hann, hann};
  merge(_layer, 10, hann_cn);

  Mat m;
  // create gaussian response
  y=Mat::zeros((int)roi.height,(int)roi.width,CV_32F);
  for(int i=0;i<int(roi.height);i++){
    for(int j=0;j<int(roi.width);j++){
      float val = static_cast<float>((i-roi.height/2+1)*(i-roi.height/2+1)+(j-roi.width/2+1)*(j-roi.width/2+1));

      y.at<float>(i,j) = val;
      if (val <= disThresh)
      {
          Mat sol = Mat::zeros(1, 2, CV_32F);
          sol.at<float>(0) = j;
          sol.at<float>(1) = i;
          m.push_back(sol);
      }

    }
  }

  y*=(float)output_sigma;
  cv::exp(y,y);
//y*=(float)(-output_sigma/CV_PI);

  cv::Mat resMean, resCovar;
  cv::calcCovarMatrix(m, resCovar, resMean, CV_COVAR_NORMAL | CV_COVAR_ROWS|CV_COVAR_SCALE, CV_32F);
  std::cout << "init Y count about sigma:"<< m.rows <<"\n" << std::endl;
  std::cout << "init mean and covariance!!!!!!!!!!!!!!!!!!!!:" << std::endl;
  std::cout << "init mean:\n"
            << resMean
            << "\n"
            << "init covar:\n"
            << resCovar
            << "\n"
            << std::endl;

  corrCovar = resCovar;
  if (scale_update_enable)
  {
    eigen(corrCovar, corrEigVal, corrEigVec);
  }

  // perform fourier transfor to the gaussian response
  fft2(y,yf);

  if (image.channels() == 1) { // disable CN for grayscale images
    params.desc_pca &= ~(CN);
    params.desc_npca &= ~(CN);
  }

  // record the non-compressed descriptors
  if((params.desc_npca & GRAY) == GRAY)descriptors_npca.push_back(GRAY);
  if((params.desc_npca & CN) == CN)descriptors_npca.push_back(CN);
  if(use_custom_extractor_npca)descriptors_npca.push_back(CUSTOM);
    features_npca.resize(descriptors_npca.size());

  // record the compressed descriptors
  if((params.desc_pca & GRAY) == GRAY)descriptors_pca.push_back(GRAY);
  if((params.desc_pca & CN) == CN)descriptors_pca.push_back(CN);
  if(use_custom_extractor_pca)descriptors_pca.push_back(CUSTOM);
    features_pca.resize(descriptors_pca.size());

  // accept only the available descriptor modes
  CV_Assert(
    (params.desc_pca & GRAY) == GRAY
    || (params.desc_npca & GRAY) == GRAY
    || (params.desc_pca & CN) == CN
    || (params.desc_npca & CN) == CN
    || use_custom_extractor_pca
    || use_custom_extractor_npca
  );

  // return true only if roi has intersection with the image
  if((roi & Rect(0,0, resizeImage ? image.cols / 2 : image.cols,
                   resizeImage ? image.rows / 2 : image.rows)) == Rect())
      return false;

  // initialize templates for HOG/ColorNames 
  Mat img=image.clone();
  // check the channels of the input image, grayscale is preferred
  CV_Assert(img.channels() == 1 || img.channels() == 3);

  // resize the image whenever needed
  if(resizeImage)cv::resize(img,img,Size(img.cols/2,img.rows/2),0,0,cv::INTER_LINEAR);

  // extract the patch for learning purpose
  // get non compressed descriptors
  for(unsigned i=0;i<descriptors_npca.size()-extractor_npca.size();i++){
    if(!getSubWindow(img,roi, features_npca[i], img_Patch, descriptors_npca[i]))return false;
  }
  // get non-compressed custom descriptors
  for(unsigned i=0,j=(unsigned)(descriptors_npca.size()-extractor_npca.size());i<extractor_npca.size();i++,j++){
    if(!getSubWindow(img,roi, features_npca[j], extractor_npca[i]))return false;
  }
  if(features_npca.size()>0)merge(features_npca,X[1]);

  //update the training data
  Z[0] = X[0].clone();
  Z[1] = X[1].clone();

  // merge all features
  x = X[1];

  layers.resize(x.channels());
  vxf.resize(x.channels());
  vyf.resize(x.channels());
  vxyf.resize(vyf.size());
  new_alphaf=Mat_<Vec2f >(yf.rows, yf.cols);

  // Kernel Regularized Least-Squares, calculate alphas
  denseGaussKernel(params.sigma,x,x,k,layers,vxf,vyf,vxyf,xy_data,xyf_data);
//  cv::normalize(k, k, 0.0f, 1.0f, cv::NORM_MINMAX);

  // compute the fourier transform of the kernel and add a small value
  fft2(k,kf);
  kf_lambda=kf+params.lambda;

  float den;
  for(int i=0;i<yf.rows;i++){
    for(int j=0;j<yf.cols;j++){
      den = 1.0f/(kf_lambda.at<Vec2f>(i,j)[0]*kf_lambda.at<Vec2f>(i,j)[0]+kf_lambda.at<Vec2f>(i,j)[1]*kf_lambda.at<Vec2f>(i,j)[1]);

      new_alphaf.at<Vec2f>(i,j)[0]=
      (yf.at<Vec2f>(i,j)[0]*kf_lambda.at<Vec2f>(i,j)[0]+yf.at<Vec2f>(i,j)[1]*kf_lambda.at<Vec2f>(i,j)[1])*den;
      new_alphaf.at<Vec2f>(i,j)[1]=
      (yf.at<Vec2f>(i,j)[1]*kf_lambda.at<Vec2f>(i,j)[0]-yf.at<Vec2f>(i,j)[0]*kf_lambda.at<Vec2f>(i,j)[1])*den;
    }
  }

  // update the RLS model
  alphaf=new_alphaf.clone();

  if (kalman_enable)
  {
    // init kalman filter
    kalman.init(4, 2, 0, CV_32F);

    cv::Mat state = cv::Mat::zeros(4, 1, CV_32F);
    state.at<float>(0) = (float)boundingBox.x + boundingBox.width/2.0f;
    state.at<float>(1) = (float)boundingBox.y + boundingBox.height/2.0f;

    timespec stp;
    stp.tv_sec = frame/1000;
    stp.tv_nsec = (frame%1000)*1e6;

    cv::Mat initialCov  = cv::Mat::eye(4, 4, CV_32F);
    initialCov.at<float>(0, 0) = resCovar.at<float>(0, 0);
    initialCov.at<float>(1, 1) = resCovar.at<float>(1, 1);
  //  initialCov.at<float>(2, 2) = std::pow(roi.width/2, 2);
  //  initialCov.at<float>(3, 3) = std::pow(roi.height/2, 2);
    initialCov.at<float>(2, 2) = 0;
    initialCov.at<float>(3, 3) = 0;

    kalman.initialParams(state, initialCov, stp);
  }

  return true;
}

cv::Mat TrackerKCFImpl::getCovar()
{
  return corrCovar;
}

bool TrackerKCFImpl::detectImpl(const Mat& image, Rect& boundingBox, Mat& covar, float& confidence, float scale) {
  double minVal, maxVal;	// min-max response
  float scaleX = 1.0f;
  float scaleY = 1.0f;
  Point minLoc, maxLoc;	// min-max location
  cv::Rect img_rect(0, 0, image.cols, image.rows);

  frame++;

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

  cv::Rect overlap = img_rect & boundingBox;
  if (overlap.area() <= 0)
  {
    std::cout << "Detect: bounding box wrong:" << boundingBox << std::endl;
    return false;
  }

  Mat img = image.clone();
  // check the channels of the input image, grayscale is preferred
  CV_Assert(img.channels() == 1 || img.channels() == 3);
  // scale the image whenever needed
  cv::resize(img,img,Size(img.cols*scale,img.rows*scale),0,0,cv::INTER_LINEAR);

  // resize the image whenever needed
  if (resizeImage) cv::resize(img,img,Size(img.cols/2,img.rows/2),0,0,cv::INTER_LINEAR);

  std::cout << "detect boundingBox\n" << boundingBox << std::endl;

  roi_scale = boundingBox;

  Point2d central = scale*(boundingBox.tl() + boundingBox.br())/2;
  roi_scale.x = central.x - roi_scale.width/2; 
  roi_scale.y = central.y - roi_scale.height/2; 

  if (resizeImage)
  {
    roi_scale.x/=2.0;
    roi_scale.y/=2.0;
    roi_scale.width/=2.0;
    roi_scale.height/=2.0;
  }

  cv::Mat showImage(2*roi_scale.height, 4*roi_scale.width, features_npca[0].type(), cv::Scalar(255));
  // detection part
  // extract and pre-process the patch
  // get non compressed descriptors
  for(unsigned i=0;i<descriptors_npca.size()-extractor_npca.size();i++){
    if(!getSubWindow(img,roi_scale, features_npca[i], img_Patch, descriptors_npca[i]))return false;
  }
  //get non-compressed custom descriptors
  for(unsigned i=0,j=(unsigned)(descriptors_npca.size()-extractor_npca.size());i<extractor_npca.size();i++,j++){
    if(!getSubWindow(img,roi_scale, features_npca[j], extractor_npca[i]))return false;
  }
  if(features_npca.size()>0)merge(features_npca,X[1]);

  if(!X[1].empty())
  {
    cv::Mat hog = showImage(cv::Rect(2*roi_scale.width, roi_scale.height,
                                           roi_scale.width, roi_scale.height));
    cv::normalize(features_npca[0], hog, 0.0f, 1.0f, cv::NORM_MINMAX);
    
    cv::Mat cMat = showImage(cv::Rect(3*roi_scale.width, roi_scale.height,
                                           roi_scale.width, roi_scale.height));
    cv::Mat colorMat[10];
    split(features_npca[1], colorMat);
    cv::normalize(colorMat[0], cMat, 0.0f, 1.0f, cv::NORM_MINMAX);
  }


  if(!Z[1].empty())
  {
    cv::Mat featureMat[11];
    split(Z[1], featureMat);
    cv::Mat hog = showImage(cv::Rect(2*roi_scale.width, 0,
                                           roi_scale.width, roi_scale.height));
    cv::normalize(featureMat[0], hog, 0.0f, 1.0f, cv::NORM_MINMAX);
    
    cv::Mat cMat = showImage(cv::Rect(3*roi_scale.width, 0,
                                           roi_scale.width, roi_scale.height));
    cv::normalize(featureMat[1], cMat, 0.0f, 1.0f, cv::NORM_MINMAX);
  }

  // merge all features
  x = X[1];
  z = Z[1];

  // compute the gaussian kernel
  denseGaussKernel(params.sigma,x,z,k,layers,vxf,vyf,vxyf,xy_data,xyf_data);
//  cv::normalize(k, k, 0.0f, 1.0f, cv::NORM_MINMAX);

  Mat kernel = k.clone();
  cv::normalize(kernel, kernel, 0.0f, 1.0f, cv::NORM_MINMAX);
  cv::Mat kernel_show = showImage(cv::Rect(0, roi_scale.height,
                                           roi_scale.width, roi_scale.height));
  cv::resize(kernel, kernel_show, cv::Size(roi_scale.width, roi_scale.height));

  cv::Mat m_k;
  for(int i=0;i<kernel.rows;i++){
    for(int j=0;j<kernel.cols;j++){
      float val = kernel.at<float>(i, j);
      if (val >= exp(-0.5))
      {
          Mat sol = Mat::zeros(1, 2, CV_32F);
          sol.at<float>(0) = j;
          sol.at<float>(1) = i;
          m_k.push_back(sol);
      }
    }
  }
  
  if (!m_k.empty())
  {
      cv::PCA pca;
      pca(m_k, Mat(), CV_PCA_DATA_AS_ROW, 2);
      Mat eigVal = pca.eigenvalues.clone();
      Mat eigVec = pca.eigenvectors.clone();
      cv::sqrt(eigVal, eigVal);
      eigVal = eigVal*sqrt(9.210340f);

      Point2d center(pca.mean.at<float>(0), pca.mean.at<float>(1));
      line(kernel_show, center, Point(center.x + eigVal.at<float>(0)*eigVec.row(0).at<float>(0), center.y + eigVal.at<float>(0)*eigVec.row(0).at<float>(1)),Scalar(255, 0, 0), 1, LINE_AA);
      line(kernel_show, center, Point(center.x + eigVal.at<float>(1)*eigVec.row(1).at<float>(0), center.y + eigVal.at<float>(1)*eigVec.row(1).at<float>(1)),Scalar(255, 0, 0), 1, LINE_AA);
  }


  // compute the fourier transform of the kernel
  fft2(k,kf);

  // calculate filter response
  calcResponse(alphaf,kf,response, spec);

  Mat alpha;
  ifft2(alphaf,alpha);
  cv::normalize(alpha, alpha, 0.0f, 1.0f, cv::NORM_MINMAX);
  cv::Mat alpha_show = showImage(cv::Rect(roi_scale.width, 0,
                                           roi_scale.width, roi_scale.height));
  cv::resize(alpha, alpha_show, cv::Size(roi_scale.width, roi_scale.height));
  
  // detection result
  Mat res = response.clone();

  Mat m;
  float disThresh = exp(-0.5f);
  for(int i=0;i<res.rows;i++){
    for(int j=0;j<res.cols;j++){
      float val = res.at<float>(i, j);
      if (val >= disThresh)
      {
          Mat sol = Mat::zeros(1, 2, CV_32F);
          sol.at<float>(0) = j;
          sol.at<float>(1) = i;
          m.push_back(sol);
      }
    }
  }
  

  cv::Mat res_show = showImage(cv::Rect(roi_scale.width, roi_scale.height,
                                           roi_scale.width, roi_scale.height));
  cv::Mat res_draw = res.clone();

  cv::Mat resMean, resCovar, eigVal, eigVec;
  if (m.empty())
  {
    std::cout << "Did not get valid response!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
    //return false;
  }
  else 
  {
      cv::calcCovarMatrix(m, resCovar, resMean, CV_COVAR_NORMAL|CV_COVAR_ROWS|CV_COVAR_SCALE, CV_32F);
      eigen(resCovar, eigVal, eigVec);

      std::cout << "detect mean and covariance!!!!!!!!!!!!!!!!!!!!:" << std::endl;
      std::cout << "detect mean:\n"
                << resMean
                << "\n"
                << "detect covar:\n"
                << resCovar
                << "\n"
                << std::endl;

      Point2d center(resMean.at<float>(0), resMean.at<float>(1));
      std::cout << "center:\n"
                << center
                << std::endl;


      if (scale_update_enable)
      {
          scaleX = eigVal.at<float>(0)/corrEigVal.at<float>(0);
          scaleY = eigVal.at<float>(1)/corrEigVal.at<float>(1);
          scaleX = std::sqrt(scaleX);
          scaleY = std::sqrt(scaleY);
          std::cout << "\n"
                    << "ScaleX:\n"
                    << scaleX
                    << "\n"
                    << "scaleY:\n"
                    << scaleY
                    << "\n" 
                    << "\n"
                    << "eigVal:\n"
                    << eigVal
                    << "\n" 
                    << "corrEigVal:\n"
                    << corrEigVal
                    << "\n" 
                    << std::endl;
      }

      {
        cv::sqrt(eigVal, eigVal);
        eigVal = eigVal*sqrt(9.210340f);
        line(res_draw, center, Point(center.x + eigVal.at<float>(0)*eigVec.row(0).at<float>(0), center.y + eigVal.at<float>(0)*eigVec.row(0).at<float>(1)),Scalar(255, 0, 0), 1, LINE_AA);
        line(res_draw, center, Point(center.x + eigVal.at<float>(1)*eigVec.row(1).at<float>(0), center.y + eigVal.at<float>(1)*eigVec.row(1).at<float>(1)),Scalar(255, 0, 0), 1, LINE_AA);
      }
    
  }

  res_draw.copyTo(res_show);

  cv::Mat res_thd_show = showImage(cv::Rect(0, 0, roi_scale.width, roi_scale.height));
  for(int i=0;i<res.rows;i++){
    for(int j=0;j<res.cols;j++){
      float val = res.at<float>(i, j);
      if (val < params.detect_thresh)
      {
        res.at<float>(i, j) = .0f;
      }
    }
  }
  res.copyTo(res_thd_show);


  imshow("detection process", showImage);

  // extract the maximum response
  minMaxLoc(response, &minVal, &maxVal, &minLoc, &maxLoc );
  confidence = maxVal;
  printf("Max response:%f, loc(%d, %d), scale(%f)\n", maxVal, maxLoc.x, maxLoc.y, scale);

  roi_scale.x+=(maxLoc.x-roi_scale.width/2 + 1);
  roi_scale.y+=(maxLoc.y-roi_scale.height/2 + 1);
  roi_scale.width = roi_scale.width*scaleX;
  roi_scale.height = roi_scale.height*scaleY;

  {
    std::cout << "detect response ROI scale:\n" 
      << "ROI:\t" 
      << roi
      << "\n"
      << "ROI SCALE:\t" 
      << roi_scale
      << std::endl;
  }

  if (maxVal < params.detect_thresh)
  {
     params.interp_factor = 0;
     return false;
  }
  else
  {
     params.interp_factor = maxVal/(1.0f+maxVal);
     resCovar.copyTo(covar);
     corrCovar = resCovar;
  }

//  roi_scale.x+=(ex.at<float>(0)-roi_scale.width/2+1);
//  roi_scale.y+=(ex.at<float>(1)-roi_scale.height/2+1);
  // update the bounding box
  boundingBox.x = resizeImage?roi_scale.x*2:roi_scale.x;
  boundingBox.y = resizeImage?roi_scale.y*2:roi_scale.y;
  boundingBox.width = resizeImage?roi_scale.width*2:roi_scale.width;
  boundingBox.height = resizeImage?roi_scale.height*2:roi_scale.height;
  boundingBox.x /= scale;
  boundingBox.y /= scale;
  boundingBox.width /= scale;
  boundingBox.height /= scale;

  std::cout << "response detect boundingBox\n" 
    << boundingBox
    << std::endl;


  //correct centra point
  if (kalman_enable)
  {
    cv::Mat bcentra = cv::Mat::zeros(2, 1, CV_32F);
    bcentra.at<float>(0) = boundingBox.x + boundingBox.width/2;
    bcentra.at<float>(1) = boundingBox.y + boundingBox.height/2;
    kalman.correct(bcentra,resCovar);
  }

  return true;
}

/*
 * Main part of the KCF algorithm
 */
bool TrackerKCFImpl::updateImpl(const Mat& image, Rect& boundingBox, Mat &covar, float confidence, float scale) {

  bool template_scale = false;
  cv::Rect img_rect(0, 0, image.cols, image.rows);

  std::cout << "\nUpdate: bounding box:" << boundingBox << std::endl;

  cv::Rect overlap = img_rect & boundingBox;
  if (overlap.area() <= 0)
  {
    std::cout << "Update: bounding box wrong:" << boundingBox << std::endl;
    return false;
  }

  roi_scale = boundingBox;

  Point2d central = scale*(boundingBox.tl() + boundingBox.br())/2;
  roi_scale.x = central.x - roi_scale.width/2; 
  roi_scale.y = central.y - roi_scale.height/2; 

  if (resizeImage)
  {
    roi_scale.x/=2.0;
    roi_scale.y/=2.0;
    roi_scale.width/=2.0;
    roi_scale.height/=2.0;
  }

  if (roi.size() != roi_scale.size())
  {
    template_scale = true;
    roi = roi_scale;
  }

  Mat img=image.clone();
  // check the channels of the input image, grayscale is preferred
  CV_Assert(img.channels() == 1 || img.channels() == 3);

  // resize the image whenever needed
  if(resizeImage)cv::resize(img,img,Size(img.cols/2,img.rows/2),0,0,cv::INTER_LINEAR);

  // extract the patch for learning purpose
  // get non compressed descriptors
  for(unsigned i=0;i<descriptors_npca.size()-extractor_npca.size();i++){
    if(!getSubWindow(img,roi_scale, features_npca[i], img_Patch, descriptors_npca[i]))return false;
  }
  // get non-compressed custom descriptors
  for(unsigned i=0,j=(unsigned)(descriptors_npca.size()-extractor_npca.size());i<extractor_npca.size();i++,j++){
    if(!getSubWindow(img,roi_scale, features_npca[j], extractor_npca[i]))return false;
  }
  if(features_npca.size()>0)merge(features_npca,X[1]);

  if (Z[1].empty() || X[1].empty())
  {
    std::cout << "\nError: Can not get feature maps\n" << std::endl;
  }

  if(template_scale )
  {
    cv::resize(Z[1], Z[1], roi_scale.size());
    cv::resize(X[1], X[1], roi_scale.size());

    output_sigma=std::sqrt(static_cast<float>(roi_scale.width*roi_scale.height))*params.output_sigma_factor;

    float disThresh = output_sigma*output_sigma;
    output_sigma=-0.5f/(output_sigma*output_sigma);


    Mat m;
    // create gaussian response
    y=Mat::zeros((int)roi_scale.height,(int)roi_scale.width,CV_32F);
    for(int i=0;i<int(roi_scale.height);i++){
      for(int j=0;j<int(roi_scale.width);j++){
        float val = 
                static_cast<float>((i-roi.height/2+1)*(i-roi.height/2+1)+(j-roi.width/2+1)*(j-roi.width/2+1));
        y.at<float>(i,j) = val;
        if (val <= disThresh)
        {
          Mat sol = Mat::zeros(1, 2, CV_32F);
          sol.at<float>(0) = j;
          sol.at<float>(1) = i;
          m.push_back(sol);
        }
      }
    }

    y*=(float)output_sigma;
    cv::exp(y,y);
    fft2(y,yf);

    cv::Mat resMean, resCovar;
    cv::calcCovarMatrix(m, resCovar, resMean, CV_COVAR_NORMAL | CV_COVAR_ROWS|CV_COVAR_SCALE, CV_32F);
    corrCovar = resCovar;
    if (scale_update_enable)
    {
      std::cout << "\nupdate mean:\n"
            << resMean
            << "\n"
            << "update covar:\n"
            << resCovar
            << "\n"
            << std::endl;

      eigen(corrCovar, corrEigVal, corrEigVec);
    }

  }

  cv::Mat updateImage(2*roi_scale.height, 2*roi_scale.width, features_npca[0].type(), cv::Scalar(255));
  if(!X[1].empty())
  {
    cv::Mat hog = updateImage(cv::Rect(0, roi_scale.height,
                                           roi_scale.width, roi_scale.height));
    cv::normalize(features_npca[0], hog, 0.0f, 1.0f, cv::NORM_MINMAX);

    
    cv::Mat cMat = updateImage(cv::Rect(roi_scale.width, roi_scale.height,
                                           roi_scale.width, roi_scale.height));
    cv::Mat colorMat[10];
    split(features_npca[1], colorMat);
    cv::normalize(colorMat[0], cMat, 0.0f, 1.0f, cv::NORM_MINMAX);
  }


  if(!Z[1].empty())
  {
    cv::Mat featureMat[11];
    split(Z[1], featureMat);

    cv::Mat hog = updateImage(cv::Rect(0, 0,
                                           roi_scale.width, roi_scale.height));
    cv::normalize(featureMat[0], hog, 0.0f, 1.0f, cv::NORM_MINMAX);

    
    cv::Mat cMat = updateImage(cv::Rect(roi_scale.width, 0,
                                           roi_scale.width, roi_scale.height));
    cv::normalize(featureMat[1], cMat, 0.0f, 1.0f, cv::NORM_MINMAX);
  }


  imshow("Update Process", updateImage);

  // merge all features
  x = X[1];
  z = Z[1];

  // Kernel Regularized Least-Squares, calculate alphas
  denseGaussKernel(params.sigma,x,z,k,layers,vxf,vyf,vxyf,xy_data,xyf_data);
//  cv::normalize(k, k, 0.0f, 1.0f, cv::NORM_MINMAX);

  if(!z.empty() && !x.empty())
  {

  } else {
    return false;
  }

  if (confidence > 1.0f)
    confidence = 0.999f;
  else if (confidence < 0.0f)
    confidence = 0.0f;

//  k = confidence*k;
  Mat k_inv = k.clone();
  k_inv = 1.0f - k;

  cv::Mat k_ratio;
  cv::Mat k_inv_ratio;
  std::vector<cv::Mat> k_ratio_v(x.channels(), k);
  std::vector<cv::Mat> k_inv_ratio_v(z.channels(), k_inv);
  merge(k_ratio_v, k_ratio);
  merge(k_inv_ratio_v, k_inv_ratio);

  cv::Mat x_filter = X[1].clone();
  cv::Mat z_filter = Z[1].clone();
  x_filter = x_filter.mul(k_inv_ratio);
  z_filter = z_filter.mul(k_ratio);
  z = Z[1] = z_filter + x_filter;

  // compute the gaussian kernel
  denseGaussKernel(params.sigma,z, X[1], k,layers,vxf,vyf,vxyf,xy_data,xyf_data);
//  cv::normalize(k, k, 0.0f, 1.0f, cv::NORM_MINMAX);

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

  return true;
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
  proj_matrix=u(Rect(0,0,compressed_sz,src.channels())).clone();
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
bool TrackerKCFImpl::getSubWindow(const Mat img, const Rect _roi, Mat& feat, Mat& patch, MODE desc) const {

  Rect region=_roi;

  // return false if roi is outside the image
  if((roi & Rect(0,0, img.cols, img.rows)) == Rect() )
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
     // feat=feat.mul(hann_cn); // hann window filter
      break;
    default: // GRAY
      if(img.channels()>1)
        cvtColor(patch,feat, CV_BGR2GRAY);
      else
        feat=patch;
      //feat.convertTo(feat,CV_32F);
        feat.convertTo(feat,CV_32F, 1.0/255.0, -0.5);
      //feat=feat/255.0-0.5; // normalize to range -0.5 .. 0.5
      //  feat=feat.mul(hann); // hann window filter
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


/*
 * get feature using external function
 */
bool TrackerKCFImpl::getSubWindow(const Mat img, const Rect _roi, Mat& feat, void (*f)(const Mat, const Rect, Mat& )) const{

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


void TrackerKCFImpl::setFeatureExtractor(void (*f)(const Mat, const Rect, Mat&), bool pca_func){
  if(pca_func){
    extractor_pca.push_back(f);
    use_custom_extractor_pca = true;
  }else{
    extractor_npca.push_back(f);
    use_custom_extractor_npca = true;
  }
}

