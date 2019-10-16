#pragma once

#include <complex>
#include <cmath>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/ml.hpp>

#include "tracker/featureColorName.hpp"
#include "filter/kalman.hpp"

using namespace cv;
using namespace cv::ml;

struct Params
{
  /**
  * \brief Constructor
  */
  Params();

  /**
  * \brief Read parameters from file, currently unused
  */
  void read(const FileNode& /*fn*/);

  /**
  * \brief Read parameters from file, currently unused
  */
  void write(FileStorage& /*fs*/) const;

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

enum MODE {
  GRAY   = (1 << 0),
  CN     = (1 << 1),
  HOG     = (1 << 2),
  CUSTOM = (1 << 3)
};

/*
* Prototype
*/
class TrackerKCFImpl {
public:
  TrackerKCFImpl();
  void setFeatureExtractor(void (*f)(const Mat, const Rect2d, Mat&), bool pca_func = false);
  Params params;
  bool isInit;

   /*
  * basic functions and vars
  */
  bool initImpl( const Mat& image, Rect2d& boundingBox );

  bool detectImpl(const Mat& image, Rect2d& boundingBox, float& confidence, bool debug=false);

  bool updateWithTrackImpl(const Mat& image, Rect2d& boundingBox, Mat& covar, float confidence, bool debug=false);

  bool updateWithDetectImpl(const Mat& imageDet, Rect2d& boundingBox, const Mat& imageTrack, Rect2d& trackBox, Mat &covar, float confidence, bool debug=false);



  bool extractFeature(const Mat& image, Rect2d u_roi, cv::Mat& featureSet);

  bool extractKernelMap(const Mat& srcFeature, const Mat& dstFeature, cv::Mat& kernelMap);

  bool extractCovar(const Mat& map, float threshold, Mat& resMean, Mat& resCovar, Mat& eigVal, Mat& eigVec);

  void constructGaussian(cv::Size size, Mat& map);

  void drawFeature(cv::Rect2d u_roi, Mat& feature, Mat& disp);

  void drawDetectProcess(cv::Rect2d u_roi, Mat& feature, Mat& base, Mat& kernel, Mat& response, Mat& alpha_fft);

  void drawUpdateWithDetProcess(cv::Rect2d u_roi, Mat& feature, Mat& base, Mat& kernel, Mat& response, Mat& new_base, Mat& alpha_fft);

  void drawUpdateWithTrackProcess(cv::Rect2d u_roi, Mat& feature, Mat& base, Mat& new_base, Mat& alpha_fft);

  cv::Mat getCovar();

  Rect2d roi_scale;
  bool kalman_enable;
  bool scale_update_enable;
  filter::KalmanFilter kalman;
  cv::Mat corrMean,corrCovar;
  cv::Mat corrEigVal, corrEigVec;

protected:
  /*
  * KCF functions and vars
  */
  void createHanningWindow(OutputArray dest, const cv::Size winSize, const int type) const;
  void inline fft2(const Mat src, std::vector<Mat> & dest, std::vector<Mat> & layers_data) const;
  void inline fft2(const Mat src, Mat & dest) const;
  void inline ifft2(const Mat src, Mat & dest) const;
  void inline pixelWiseMult(const std::vector<Mat> src1, const std::vector<Mat>  src2, std::vector<Mat>  & dest, const int flags, const bool conjB=false) const;
  void inline sumChannels(std::vector<Mat> src, Mat & dest) const;
  void inline updateProjectionMatrix(const Mat src, Mat & old_cov,Mat &  proj_matrix,float pca_rate, int compressed_sz,
                                     std::vector<Mat> & layers_pca,std::vector<Scalar> & average, Mat pca_data, Mat new_cov, Mat w, Mat u, Mat v);
  void inline compress(const Mat proj_matrix, const Mat src, Mat & dest, Mat & data, Mat & compressed) const;
  bool getSubWindow(const Mat img, const Rect2d roi, Mat& feat, Mat& patch, MODE desc = GRAY) const;
  bool getSubWindow(const Mat img, const Rect2d roi, Mat& feat, void (*f)(const Mat, const Rect2d, Mat& )) const;

  void extractCN(Mat patch_data, Mat & cnFeatures) const;

  void extractHOG(Mat &im, Mat& hogFeatures) const;

  void denseGaussKernel(const float sigma, const Mat , const Mat y_data, Mat & k_data,
                        std::vector<Mat> & layers_data,std::vector<Mat> & xf_data,std::vector<Mat> & yf_data, std::vector<Mat> xyf_v, Mat xy, Mat xyf ) const;
  void calcResponse(const Mat alphaf_data, const Mat kf_data, Mat & response_data, Mat & spec_data) const;
  void calcResponse(const Mat alphaf_data, const Mat alphaf_den_data, const Mat kf_data, Mat & response_data, Mat & spec_data, Mat & spec2_data) const;

  void shiftRows(Mat& mat) const;
  void shiftRows(Mat& mat, int n) const;
  void shiftCols(Mat& mat, int n) const;
#ifdef HAVE_OPENCL
  bool inline oclTransposeMM(const Mat src, float alpha, UMat &dst);
#endif

private:
  float output_sigma;
  Rect2d roi;
  Mat hann; 	//hann window filter
  Mat hann_cn; //10 dimensional hann-window filter for CN features,

  Mat y,yf; 	// training response and its FFT
  Mat x; 	// observation and its FFT
  Mat k,kf;	// dense gaussian kernel and its FFT
  Mat kf_lambda; // kf+lambda
  Mat new_alphaf, alphaf;	// training coefficients
  Mat new_alphaf_den, alphaf_den; // for splitted training coefficients
  Mat z; // model
  Mat response; // detection result
  Mat old_cov_mtx, proj_mtx; // for feature compression

  // pre-defined Mat variables for optimization of private functions
  Mat spec, spec2;
  std::vector<Mat> layers;
  std::vector<Mat> vxf,vyf,vxyf;
  Mat xy_data,xyf_data;
  Mat data_temp, compress_data;
  std::vector<Mat> layers_pca_data;
  std::vector<Scalar> average_data;
  Mat img_Patch;

  // storage of the extracted features
  std::vector<Mat> features_pca;
  std::vector<Mat> features_npca;
  std::vector<MODE> descriptors_pca;
  std::vector<MODE> descriptors_npca;

  // optimization variables for updateProjectionMatrix
  Mat data_pca, new_covar,w_data,u_data,vt_data;

  // custom feature extractor
  bool use_custom_extractor_pca;
  bool use_custom_extractor_npca;
  std::vector<void(*)(const Mat img, const Rect2d roi, Mat& output)> extractor_pca;
  std::vector<void(*)(const Mat img, const Rect2d roi, Mat& output)> extractor_npca;

  // resize the image whenever needed and the patch size is large
  bool resizeImage;
  // resize ratio, should above 1.0 
  float resizeRatio;

  float paddingRatio;

#ifdef HAVE_OPENCL
  ocl::Kernel transpose_mm_ker; // OCL kernel to compute transpose matrix multiply matrix.
#endif

  int frame;
};

