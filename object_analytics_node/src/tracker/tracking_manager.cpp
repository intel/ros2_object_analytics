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

#include "object_analytics_node/tracker/tracking_manager.hpp"
#include "object_analytics_node/tracker/munkres.h"

#include <omp.h>
#include <memory>
#include <string>
#include <vector>
#include <cmath>

namespace tracker
{
// TrackingManager class implementation
const float TrackingManager::kMatchThreshold = 0.3;
const float TrackingManager::kProbabilityThreshold = 0.8;
int32_t TrackingManager::tracking_cnt = 0;
const int32_t TrackingManager::kNumOfThread = 4;

TrackingManager::TrackingManager()
{
  algo_ = "MEDIAN_FLOW";
}

void TrackingManager::track(
  std::shared_ptr<sFrame> frame)
{
  std::vector<std::shared_ptr<Tracking>>::iterator t = trackings_.begin();

  timespec stamp = frame->stamp;

  TRACE_INFO("\nTrackingManager track stamp_sec(%ld), stamp_nanosec(%ld)\n", stamp.tv_sec, stamp.tv_nsec );

  while (t != trackings_.end()) {
    if (!(*t)->updateTracker(frame)) {
      TRACE_ERR( "Tracking[%d][%s] failed, may need remove!",
        (*t)->getTrackingId(), (*t)->getObjName().c_str());
      // TBD: Add mechanism to check whether need erase the object.
     // t = trackings_.erase(t);
     // TRACE_ERR( "Erase Tracking[%d]!",(*t)->getTrackingId());
      ++t;
    } else {
      TRACE_INFO( "Tracking[%d][%s] updated",
        (*t)->getTrackingId(), (*t)->getObjName().c_str());
      ++t;
    }
  }

}


cv::Mat TrackingManager::calcTrackDetMahaDistance(std::vector<Object>& dets,
                            std::vector<std::shared_ptr<Tracking>>& tracks,
                            struct timespec stamp)
{
  cv::Mat distance;
  int size_dets = dets.size();
  int size_tracks = tracks.size();
  
  /*Check detects/tracks counts*/
  if (size_dets <= 0 || size_tracks <= 0)
    return distance;

  /*tracks as rows, dets as cols*/
  distance = cv::Mat(size_tracks, size_dets, CV_32F, cv::Scalar(INFINITY));

  for(int i=0; i<distance.rows; i++)
  {
    std::shared_ptr<Tracking> tracker = tracks[i];

    tracker::Traj traj;
    bool ret = tracker->getTraj(stamp, traj);
    if (!ret)
      continue;

    Mat t_centra = Mat::zeros(1, 2, CV_32F);
    t_centra.at<float>(0) = traj.rect_.x + traj.rect_.width/2.0f;
    t_centra.at<float>(1) = traj.rect_.y + traj.rect_.height/2.0f;

    cv::Mat covar = traj.covar_.clone();

    float prob = sqrt(determinant(covar));
    prob = 1.0f/(prob*2.0f*CV_PI);

    for(int j=0; j<distance.cols; j++)
    {
      Mat d_centra = Mat::zeros(1, 2, CV_32F);
      d_centra.at<float>(0) = dets[j].BoundBox_.x + dets[j].BoundBox_.width/2.0f;
      d_centra.at<float>(1) = dets[j].BoundBox_.y + dets[j].BoundBox_.height/2.0f;

      float m_dist = cv::Mahalanobis(t_centra, d_centra, covar.inv());
      /*2 times variance as threshold*/
      if (m_dist > 2.0f)
        continue;
     
      distance.at<float>(i, j) = std::pow(m_dist,2);

    }
  }

  return distance;
}



cv::Mat TrackingManager::calcTrackDetWeights(std::vector<Object>& dets,
                            std::vector<std::shared_ptr<Tracking>>& tracks,
                            struct timespec stamp)
{
  cv::Mat weights;
  int size_dets = dets.size();
  int size_tracks = tracks.size();
  
  /*Check detects/tracks counts*/
  if (size_dets <= 0 || size_tracks <= 0)
    return weights;

  /*tracks as rows, dets as cols*/
  weights = cv::Mat::zeros(size_tracks, size_dets, CV_32F);

  for(int i=0; i<weights.rows; i++)
  {
    std::shared_ptr<Tracking> tracker = tracks[i];

    tracker::Traj traj;
    bool ret = tracker->getTraj(stamp, traj);
    if (!ret)
      continue;

    Mat t_centra = Mat::zeros(1, 2, CV_32F);
    t_centra.at<float>(0) = traj.rect_.x + traj.rect_.width/2.0f;
    t_centra.at<float>(1) = traj.rect_.y + traj.rect_.height/2.0f;

    cv::Mat covar = traj.covar_.clone();

    float prob = sqrt(determinant(covar));
    prob = 1.0f/(prob*2.0f*CV_PI);

    for(int j=0; j<weights.cols; j++)
    {
      Mat d_centra = Mat::zeros(1, 2, CV_32F);
      d_centra.at<float>(0) = dets[j].BoundBox_.x + dets[j].BoundBox_.width/2.0f;
      d_centra.at<float>(1) = dets[j].BoundBox_.y + dets[j].BoundBox_.height/2.0f;

      float m_dist = cv::Mahalanobis(t_centra, d_centra, covar.inv());
      /*2 times variance as threshold*/
      if (m_dist > 2.0f)
        continue;
     
      /*No need to compute probabilities, Mahalanobis more suitable*/ 
      /*float likely_prob = prob;*/
      float likely_prob = exp(-std::pow(m_dist,2)/2.0f);
      weights.at<float>(i, j) = likely_prob;

    }
  }

  return weights;
}


void TrackingManager::detectNew(
  std::shared_ptr<sFrame> frame,
  std::vector<Object>& objs)
{
  struct timespec stamp = frame->stamp;
  TRACE_INFO("\nTrackingManager detectNew stamp_sec(%ld), stamp_nanosec(%ld)\n", stamp.tv_sec, stamp.tv_nsec);

  //cv::Mat weights = calcTrackDetWeights(objs, trackings_, stamp);
  cv::Mat distance = calcTrackDetMahaDistance(objs, trackings_, stamp);

  std::cout << "\nweights:\n"<< distance <<"\n"<< std::endl;

  cv::Mat det_matches = cv::Mat(1, objs.size(), CV_32SC1, cv::Scalar(-1));
  cv::Mat tracker_matches = cv::Mat(1, trackings_.size(), CV_32SC1, cv::Scalar(-1));
  if (!distance.empty()) {
    matchTrackDetWithDistance(distance, tracker_matches, det_matches);
  }

  std::cout << "\nTracker matches:\n"<< tracker_matches << std::endl;
  std::cout << "\nDet matches:\n"<< det_matches << std::endl;

  
  for(int i=0; i<objs.size(); i++)
  {
    int32_t tracker_idx = det_matches.at<int32_t>(i);
    if (tracker_idx >= 0)
    {
      std::cout << "\nTrackId:\n"<< tracker_idx << ", need update to detection" <<"\n"<< std::endl;
    }
    else
    {
      std::shared_ptr<Tracking> tracker = 
                       addTracking(objs[i].Category_, objs[i].Confidence_, objs[i].BoundBox_);
      tracker->rectifyTracker(frame, objs[i].BoundBox_);
    }
  }

}

void TrackingManager::matchTrackDetWithDistance(cv::Mat& distance, cv::Mat& row_match, cv::Mat& col_match)
{
  int origin_rows = distance.rows, origin_cols = distance.cols;
  int size_squal = (origin_rows>origin_cols)?origin_rows:origin_cols;

  if (distance.empty()) {
    return;
  }

  if (row_match.cols != origin_rows || col_match.cols != origin_cols) {
    return;
  }

  /*initial Matrix for KM algorithm*/
	Matrix<float> matrix(origin_rows, origin_cols);
	for ( int row = 0 ; row < origin_rows; row++ ) {
		for ( int col = 0 ; col < origin_cols ; col++ ) {
      matrix(row,col) = distance.ptr<float>(row)[col];
		}
	}

  Munkres<float> m;
  m.solve(matrix);

  // Display solved matrix.
	std::cout <<"KM algorithm result:"<<std::endl;
	for ( int row = 0 ; row < origin_rows; row++ ) {
		for ( int col = 0 ; col < origin_cols; col++ ) {
			std::cout.width(2);
			std::cout << matrix(row,col) << ",";
		}
		std::cout << std::endl;
	}
	std::cout << std::endl;

	for ( int row = 0 ; row < origin_rows; row++ ) {
		for ( int col = 0 ; col < origin_cols ; col++ ) {
      if (matrix(row,col) == 0) {
        row_match.ptr<int32_t>(0)[row] = col;
        col_match.ptr<int32_t>(0)[col] = row;
      }
		}
	}

}

void TrackingManager::matchTrackDetWithProb(cv::Mat& weights, cv::Mat& matches)
{
  int origin_rows = weights.rows, origin_cols = weights.cols;
  int size_squal = (origin_rows>origin_cols)?origin_rows:origin_cols;

  /*extend weight to be squal size*/
  cv::Mat correlations = cv::Mat::zeros(size_squal, size_squal, CV_32F);
  cv::Mat clip = correlations(cv::Rect(0, 0, origin_cols, origin_rows));
  weights.copyTo(clip);

  /*initial matches*/
  cv::Mat row_mask = cv::Mat(1, size_squal, CV_32SC1, cv::Scalar(-1)); 
  cv::Mat col_mask = cv::Mat(1, size_squal, CV_32SC1, cv::Scalar(-1)); 

  /*max weights*/
  cv::Mat row_weight = cv::Mat::zeros(1, size_squal, CV_32F); 
  cv::Mat col_weight = cv::Mat::zeros(1, size_squal, CV_32F); 

  /*initial visit mask for each round of hungarian search*/
  cv::Mat row_visit = cv::Mat::zeros(1, size_squal, CV_8UC1); 
  cv::Mat col_visit = cv::Mat::zeros(1, size_squal, CV_8UC1); 

  /*initial visit mask for each round of hungarian search*/
  cv::Mat col_gap= cv::Mat(1, size_squal, CV_32F, cv::Scalar(INFINITY)); 

  /*initialize row_weight, col_weight*/
  for (int i=0; i<size_squal; i++) {
    int row_maxId = -1, col_maxId = -1;
    float row_max = 0.0f, col_max = 0.0f;

    for (int j=0;j<size_squal; j++) {
      float row_value = correlations.ptr<float>(i)[j];
      if (row_max <= row_value)
      {
        row_max = row_value;
        row_maxId = j;
      }

      float col_value = correlations.ptr<float>(j)[i];
      if (col_max <= col_value)
        col_max = col_value;
        col_maxId = j;
    }

    // row_mask.at<uint8_t>(i) = row_maxId;
    row_weight.ptr<float>(0)[i] = row_max;

    // col_mask.at<int32_t>(i) = col_maxId;
  }

  /*search from tracker to detection*/
  for (int i=0; i<size_squal; i++) {
    /*reinitialize min weight gap for each search*/
    col_gap = cv::Scalar(INFINITY);
 
    while (true) {
        /*reinitialize visit flags*/
        row_visit = 0; 
        col_visit = 0; 
        bool ret = searchMatch(i, row_visit, row_weight, col_visit,col_weight,
                               col_mask,col_gap,correlations); 
        if (!ret) {
          int min_idx[2];
          cv::minMaxIdx(col_gap, NULL, NULL, min_idx);
          float min_gap = col_gap.ptr<float>(min_idx[0])[min_idx[1]];

          for (int j = 0; j<size_squal; j++) {
            if (*col_visit.ptr<uint8_t>(j) == 1)
              col_weight.ptr<float>(0)[j] += min_gap;

            if (row_visit.ptr<uint8_t>(0)[j] == 1)
              row_weight.ptr<float>(0)[j] -= min_gap;
          }

       } else {

          break;
       } 

    }

  }

  cv::Mat res = col_mask(cv::Rect(0,0,matches.cols, matches.rows));
  res.copyTo(matches);

}

/*search for each row item, to match col item*/
bool TrackingManager::searchMatch(
  int srcId,
  cv::Mat& srcVisit,
  cv::Mat& srcCorr,
  cv::Mat& tgtVisit,
  cv::Mat& tgtCorr,
  cv::Mat& tgtMatch,
  cv::Mat& weightDelta,
  cv::Mat& correlations)
{
  int tgt_size = tgtCorr.cols;

  srcVisit.ptr<uint8_t>(0)[srcId] = 1;

  float srcCorrValue = srcCorr.ptr<float>(0)[srcId];
  for (int i=0; i<tgt_size; i++)
  {
    if (tgtVisit.ptr<uint8_t>(0)[i] == 1)
      continue;

    float gap = srcCorrValue + tgtCorr.ptr<float>(0)[i] - correlations.ptr<float>(srcId)[i];
    if (abs(gap <= 1e-04)) gap = 0.0f;

    if (gap == 0.0f)
    {
      tgtVisit.ptr<uint8_t>(0)[i] = 1;
      int tgtSrcIdx = tgtMatch.ptr<int32_t>(0)[i];
      if ((tgtSrcIdx == -1) || 
           searchMatch(tgtSrcIdx, srcVisit, srcCorr, tgtVisit, tgtCorr,
                       tgtMatch, weightDelta, correlations) )
      {
        tgtMatch.ptr<int32_t>(0)[i] = srcId;
        //srcMatch.at<int32_t>(srcId) = i;

        return true;

      }

    } else {
      weightDelta.ptr<float>(0)[i] = std::min(gap, weightDelta.ptr<float>(0)[i]);
    }
  }

  return false;
}


void TrackingManager::detect(
  std::shared_ptr<sFrame> frame,
  std::vector<Object>& objs)
{
  uint32_t i, j, k;
  struct timespec stamp = frame->stamp;

  for (auto t : trackings_) {
//    t->clearDetected();
  }

  
  std::cout << "\n TrackingManager detect entry\n" << std::endl;
  TRACE_INFO( "****detected objects: %zu",objs.size());

  /*Compute associations between tracks and detections*/
  cv::Mat validation(objs.size(), trackings_.size() + 1, CV_8U, cvScalar(0));
  cv::Mat association(objs.size(), trackings_.size() + 1, CV_64F, cvScalar(0.));
  cv::Mat association_pre(objs.size(), trackings_.size() + 1, CV_64F, cvScalar(0.));
  cv::Mat association_pre_prob(objs.size(), trackings_.size() + 1, CV_64F, cvScalar(0.));
  std::vector<cv::Mat> hypothesis;
  std::vector<cv::Mat> occupies;
  float newDetEllipse;


  if (trackings_.size() > 0)
    newDetEllipse = 0.0f;
  else
    newDetEllipse = 1.0f;

  for (j = 0; j < trackings_.size(); j++)
  {
    std::shared_ptr<Tracking> t = trackings_[j];
    cv::Rect2d tr_r = t->getTrackedRect();
    cv::Mat tr_mat(2, 1, CV_64F);
    tr_mat.at<double>(0) = tr_r.x + tr_r.width/2;
    tr_mat.at<double>(1) = tr_r.y + tr_r.height/2;
    cv::Mat predict_mat, innoCov;

    bool ret = t->getPrediction(stamp, predict_mat, innoCov);
    if (!ret)
      continue;

    newDetEllipse += determinant(innoCov);
    std::cout << "predict detect measure mat " << predict_mat << std::endl;
    std::cout << "innoCov " << innoCov << std::endl;

    for (i = 0; i < objs.size(); i++)
    {
#if 0
      cv::Mat det_mat(4, 1, CV_64F);
      det_mat.at<double>(0) = objs[i].BoundBox_.x;
      det_mat.at<double>(1) = objs[i].BoundBox_.y;
      det_mat.at<double>(2) = objs[i].BoundBox_.width;
      det_mat.at<double>(3) = objs[i].BoundBox_.height;
#endif
      cv::Mat det_mat(2, 1, CV_64F);
      det_mat.at<double>(0) = objs[i].BoundBox_.x + objs[i].BoundBox_.width/2;
      det_mat.at<double>(1) = objs[i].BoundBox_.y + objs[i].BoundBox_.height/2;
      std::cout << "det mat " << det_mat << std::endl;
      float m_dist = cv::Mahalanobis(predict_mat, det_mat, innoCov.inv());
      association_pre.at<double>(i, j) = m_dist;
      double likely_prob = 1 /(2 * std::pow(CV_PI, det_mat.rows/2) *  sqrt(determinant(innoCov)));
      likely_prob *= exp(-std::pow(m_dist,2)/2.0f);
      association_pre_prob.at<double>(i, j) = likely_prob;
      association.at<double>(i, j) = cv::norm(tr_mat, det_mat);


      if (likely_prob > 1.0E-15)
        validation.at<uchar>(i, j) = 1;

      validation.at<uchar>(i, trackings_.size()) = 1;
    }
  }

  double probNewDetect = 1e-15f/newDetEllipse;
  association_pre_prob.col(association_pre_prob.cols - 1) = probNewDetect;

#if 1
  std::cout << "association_pre " << association_pre << std::endl;
  std::cout << "association_pre_prob " << association_pre_prob << std::endl;
  std::cout << "association " << association << std::endl;
  std::cout << "validation " << validation << std::endl;
  std::cout << "newDetEllipse " << newDetEllipse << std::endl;
#endif

  cv::Mat scan_instant(objs.size(), trackings_.size() + 1, CV_8U, cvScalar(0));
  scan_instant.col(scan_instant.cols - 1) = 1;
  cv::Mat occupies_instant(1, trackings_.size() + 1, CV_8U, cvScalar(0));
//  std::cout << "scan_instant " << scan_instant << std::endl;
  hypothesis.push_back(scan_instant);
  cv::reduce(scan_instant, occupies_instant, 0, CV_REDUCE_MAX, CV_8U);
  occupies.push_back(occupies_instant);
//  std::cout << "occupies_instant " << occupies_instant << std::endl;

  for (i = 0; i < objs.size(); i++)
  {
    std::vector<cv::Mat> hypothesis_row;
    std::vector<cv::Mat> occupies_row;
    for (j = 0; j < trackings_.size(); j++)
    {
      if (validation.at<uchar>(i, j) == 1)
      {
          for (k=0; k < occupies.size(); k++)
          {
            if (occupies[k].at<uchar>(j))
            {
              continue;
            }
            else
            {
              cv::Mat hyp_new = hypothesis[k].clone();
              hyp_new.at<uchar>(i, j) = 1;
              hyp_new.at<uchar>(i, hyp_new.cols-1) = 0;
              hypothesis_row.push_back(hyp_new);

              cv::Mat occupies_new(1, hyp_new.cols, CV_8U, cvScalar(0));
              cv::reduce(hyp_new, occupies_new, 0, CV_REDUCE_MAX, CV_8U);
              occupies_row.push_back(occupies_new);
            }
          }
      }
    }
    hypothesis.insert(hypothesis.end(), hypothesis_row.begin(), hypothesis_row.end());
    occupies.insert(occupies.end(), occupies_row.begin(), occupies_row.end());
  }

#if 0
  std::cout << "hypothesis count " << hypothesis.size() << std::endl;
#endif
  cv::Mat hyp_prob_vec(1, hypothesis.size(), CV_64F, cvScalar(0.0f));
  double hyp_prob_sum = 0.0f;
  for (k = 0; k < hypothesis.size(); k++)
  {
#if 0
    std::cout << "hypothesis index: "<< k << std::endl;
    std::cout <<  hypothesis[k] << std::endl << std::endl;
#endif
    double hyp_prob = 1.0f;
    for (i = 0; i < objs.size(); i++)
    {
      for (j = 0; j < trackings_.size()+1; j++)
      {
        if(hypothesis[k].at<uchar>(i, j))
        {
          hyp_prob *=  association_pre_prob.at<double>(i, j);
        }
      }

    }
    hyp_prob_vec.at<double>(k) = hyp_prob;
    hyp_prob_sum += hyp_prob;
  }
#if 0
  std::cout << "hypothesis prob " << hyp_prob_vec << std::endl;
#endif
  hyp_prob_vec = hyp_prob_vec/hyp_prob_sum;
#if 0
  std::cout << "hypothesis prob normalized " << hyp_prob_vec << std::endl;
#endif
  cv::Mat track_beta(objs.size(), trackings_.size() + 1, CV_64F, cvScalar(0.0f));
  cv::Mat miss_det_beta(1, trackings_.size() + 1, CV_64F, cvScalar(0.0f));
  for (k = 0; k < hypothesis.size(); k++)
  {
    for (i = 0; i < objs.size(); i++)
    {
      for (j = 0; j < trackings_.size()+1; j++)
      {
         track_beta.at<double>(i, j) += hyp_prob_vec.at<double>(k) * hypothesis[k].at<uchar>(i, j);
      }
    }

  }
  cv::reduce(track_beta, miss_det_beta, 0, CV_REDUCE_SUM, CV_64F);
  miss_det_beta = 1 - miss_det_beta;
  std::cout << "track_beta " << track_beta << std::endl;
  std::cout << "miss_det_beta " << miss_det_beta << std::endl;

#if 0
  double* minVal, double* maxVal=0, cv::Point* minLoc=0, cv::Point* maxLoc=0;
  cv::minMaxLoc(hyp_prob_vec, &minVal, &maxVal, &minLoc, &maxLoc);
  cv::Mat hypothesis_max = hypothesis[maxLoc];
  std::cout << "hypothesis max " << hypothesis_max << std::endl;
#endif

  /*Update tracks*/
  for (j = 0; j < trackings_.size(); j++)
  {
    std::shared_ptr<Tracking> t = trackings_[j];
    if (miss_det_beta.at<double>(j) < 0.4f)
    {
      double minVal=0.0f, maxVal=0.0f;
      cv::Point minLoc(0,0), maxLoc(0, 0);
      cv::Mat track_beta_t = track_beta.col(j);
      cv::minMaxLoc(track_beta_t, &minVal, &maxVal, &minLoc, &maxLoc);
      /*For certain associations, tracker no need to rectify*/
      /*For uncertain associations, tracker need to rectify and update
       * predictions*/
//      if (maxVal < 0.95f)
//      {
        /*Todo:: rectify is not suitable, prodiction should not be reset but
         * update!!!!*/
//        std::cout << "maxVal "<< maxVal << "maxLoc x="<<maxLoc.x<<" y="<<maxLoc.y <<std::endl;
        cv::Rect2d droi = objs[maxLoc.y].BoundBox_;
        if (!validateROI(frame->frame, droi)) {
          TRACE_ERR(
            "unexptected ROI [%d %d %d %d] against mat size [%d %d]",
            droi.x, droi.y, droi.width, droi.height,
            frame->frame.cols, frame->frame.rows);
          continue;
        }
        droi &= cv::Rect2d(0, 0, frame->frame.cols, frame->frame.rows);
        t->rectifyTracker(frame, droi);
//      }

      //t->setDetected();
    }

  }

  /*For the non-associated tracks, erase the ones which long term no detected*/
#if 0
  for (j = 0; j < trackNotAssoc.size(); j++)
  {
    std::shared_ptr<Tracking> t = trackNotAssoc[j];

  }
#endif

  cv::Mat det_beta_t = track_beta.col(trackings_.size());
  /*Check for new detections*/
  std::cout << "det_beta_t "<< det_beta_t << std::endl;
  for (i = 0; i < objs.size(); i++) {

    if (objs[i].Confidence_ < 0.9f)
      continue;

    if (det_beta_t.at<double>(i) > 0.9f)
    {
      std::string n = objs[i].Category_;
      float probability = objs[i].Confidence_;
      /* some trackers do not accept an ROI beyond the size of a Mat*/
      cv::Rect2d droi = objs[i].BoundBox_;
      if (!validateROI(frame->frame, droi)) {
        TRACE_ERR(
          "unexptected ROI [%d %d %d %d] against mat size [%d %d]",
          droi.x, droi.y, droi.width, droi.height,
          frame->frame.cols, frame->frame.rows);
        continue;
      }
      droi &= cv::Rect2d(0, 0, frame->frame.cols, frame->frame.rows);
      std::shared_ptr<Tracking> t = addTracking(n, probability, droi);
//      t->setDetected();
      /* rectify tracking ROI with detected ROI*/
      t->rectifyTracker(frame, droi);
    }
  }

  /* clean up inactive trackings*/
  cleanTrackings();
}

std::vector<std::shared_ptr<Tracking>> TrackingManager::getTrackedObjs()
{
  return trackings_;
}

std::shared_ptr<Tracking> TrackingManager::addTracking(
  const std::string & name,
  const float & probability,
  const cv::Rect2d & rect)
{
  std::shared_ptr<Tracking> t =
    std::make_shared<Tracking>(tracking_cnt++, name, probability, rect);
  if (tracking_cnt == -1) {
    TRACE_ERR( "tracking count overflow");
  }
  TRACE_INFO( "addTracking[%d] +++", t->getTrackingId());
  t->setAlgo(algo_);
  trackings_.push_back(t);
  return t;
}

void TrackingManager::cleanTrackings()
{
  std::vector<std::shared_ptr<Tracking>>::iterator t = trackings_.begin();
  while (t != trackings_.end()) {
    if (!(*t)->isActive()) {
      TRACE_INFO( "removeTracking[%d] ---",
        (*t)->getTrackingId());
      t = trackings_.erase(t);
    } else {
      ++t;
    }
  }
}

/* get matched tracking for each detected object,
 * with the same object name,
 * and the most matching ROI
 */
std::shared_ptr<Tracking> TrackingManager::getTracking(
  const std::string & obj_name, const cv::Rect2d & rect, float probability,
  struct timespec stamp)
{
  double match = 0;
  int in_timezone = 0;
  std::shared_ptr<Tracking> tracking = std::shared_ptr<Tracking>();

  /* searching over all trackings*/
  for (auto t : trackings_) {
    cv::Rect2d trect;
#if 0
    if (!t->checkTimeZone(stamp)) {
      TRACE_INFO( "Not match tracker(%s)",
        t->getObjName().c_str());
      continue;
    }
#endif
    in_timezone = 1;

    /* seek for the one with the same object name (class), and not yet
     * rectified*/
    if (/*!t->isDetected() && */0 == obj_name.compare(t->getObjName())) {
      cv::Rect2d trect = t->getTrackedRect();

      cv::Point2i c1(trect.x + (trect.width/2.0f), trect.y + (trect.height/2.0f));
      /* calculate center of rectangle #2*/
      cv::Point2i c2(rect.x + (rect.width/2.0f), rect.y + (rect.height/2.0f));

      double a1 = trect.area(), a2 = rect.area(), a0 = (trect & rect).area();
      /* calculate the overlap rate*/
      double overlap = a0 / (a1 + a2 - a0);
      /* calculate the deviation between centers #1 and #2*/
      double deviate = sqrt(powf((c1.x - c2.x), 2) + powf((c1.y - c2.y), 2));
      /* calculate the match rate. The more overlap, the more matching */
      double m = overlap * 100 / deviate;

      TRACE_INFO( "tr[%d] %s [%d %d %d %d]%.2f",
        t->getTrackingId(), t->getObjName().c_str(), (int)trect.x,
        (int)trect.y, (int)trect.width, (int)trect.height, m);

      /* seek for the one with the most matching ROI*/
      if (m > match) {
        TRACE_INFO( "Found right tracking(%s)",
          t->getObjName().c_str());
        tracking = t;
        match = m;
      }
    }
  }
  /* if matching above the threshold, return the tracking*/
  if (match > TrackingManager::kMatchThreshold) {
    return tracking;
  } else if (trackings_.size() == 0 || in_timezone > 0) {
    return addTracking(obj_name, probability, rect);
  } else {
    return std::shared_ptr<Tracking>();
  }
}

bool TrackingManager::validateROI(
  const cv::Mat & mat, const cv::Rect2d& droi)
{
  cv::Rect2d rect = droi & cv::Rect2d(0, 0, mat.cols, mat.rows); 
  return (rect.area() > 0);
}

}  // namespace tracker
