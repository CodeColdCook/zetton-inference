#include "zetton_inference/tracker/mot/local_object.h"

namespace zetton {
namespace inference {
namespace tracker {

inline float cal_reid_score(const Eigen::VectorXf &query,
                            const Eigen::VectorXf &gallery) {
  return (query - gallery).squaredNorm();
}

LocalObject::LocalObject(const int id_init, const cv::Rect2d &bbox_init,
                         const KalmanFilterParam &kf_param_init,
                         const ros::Time &time_now, const Eigen::VectorXf &feat,
                         const cv::Mat &image)
    : id(id_init),
      bbox(bbox_init),
      features({feat}),
      features_now(feat),
      example_image(image),
      bbox_last_update_time(time_now) {
  // init kalman filters
  kf = new KalmanFilter(kf_param_init);
  std::cout << bbox_init << std::endl;
  kf->init(bbox_init);

  // init the color
  cv::RNG rng(std::time(0));
  color =
      cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
}

void LocalObject::track_bbox_by_optical_flow(const ros::Time &time_now) {
  // predict the bbox at current timestamp by kalman filter
  bbox = kf->predict((time_now - bbox_last_update_time).toSec());

  // if track succeed, use kalman filter to update the
  if (is_track_succeed && is_opt_enable) {
    bbox =
        kf->update(T_measurement, (time_now - bbox_last_update_time).toSec());
    tracking_fail_count = 0;
  } else {
    tracking_fail_count++;
    AINFO_F("Object {} tracking failure detected!", id);
  }

  // update the bbox last updated time
  bbox_last_update_time = time_now;

  // increase the ticks after last update by detector
  detector_update_count++;
}

void LocalObject::track_bbox_by_detector(const cv::Rect2d &bbox_detector,
                                         const ros::Time &update_time) {
  // re-initialized the important state and data
  tracking_fail_count = 0;
  detector_update_count = 0;
  is_opt_enable = true;
  keypoints_pre.clear();

  // update by kalman filter to the timestamp of the detector
  bbox = bbox_detector;
  kf->init(bbox);
}

float LocalObject::find_min_query_score(const Eigen::VectorXf &query) {
  // float min_score = 100000.0;
  // for (auto feat : features)
  // {
  //     float score = cal_reid_score(query, feat);
  //     if (score < min_score)
  //     {
  //         min_score = score;
  //     }
  // }
  return cal_reid_score(query, features_now);
}

cv::Rect2d LocalObject::bbox_of_lidar_time(const ros::Time &time_now) {
  return kf->predict_only((time_now - bbox_last_update_time).toSec());
}

void LocalObject::update_feat(const Eigen::VectorXf &feature_new,
                              float smooth_ratio) {
  features_now = smooth_ratio * features_now + (1 - smooth_ratio) * feature_new;
}

}  // namespace tracker
}  // namespace inference
}  // namespace zetton