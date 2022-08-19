#include <csignal>
#include <string>

#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "opencv2/opencv.hpp"
#include "ros/package.h"
#include "ros/ros.h"
#include "sensor_msgs/image_encodings.h"
#include "zetton_common/util/ros_util.h"
#include "zetton_inference/detector/yolo_object_detector.h"

void signalHandler(int sig)
{
  AWARN_F("Trying to exit!");
  ros::shutdown();
}

class RosYoloObjectDetector
{
private:
  inline void RosImageCallback(const sensor_msgs::ImageConstPtr &msg)
  {
    // convert image msg to cv::Mat
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
      AERROR_F("cv_bridge exception: {}", e.what());
      return;
    }

    // do detection
    zetton::inference::ObjectDetectionResults results;
    detector_.Detect(cv_ptr->image, results);

    // print results
    for (auto &result : results)
    {
      AINFO << result;
      result.Draw(cv_ptr->image);
      // publish results
      image_pub_.publish(
          cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_ptr->image)
              .toImageMsg());
    }
  }

  ros::NodeHandle *nh_;

  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  zetton::inference::YoloObjectDetector detector_;

public:
  RosYoloObjectDetector(ros::NodeHandle *nh) : nh_(nh), it_(*nh_)
  {
    // load params
    // hardcoded or using GPARAM
    std::string image_topic_sub = "/pointgrey/image_color";
    AWARN << "subscribe topic: " << image_topic_sub;

    // subscribe to input video feed
    image_sub_ = it_.subscribe(image_topic_sub, 1,
                               &RosYoloObjectDetector::RosImageCallback, this);

    // publish images
    image_pub_ = it_.advertise("/camera/result", 1);
    AWARN << "advertise topic: " << "/camera/result";

    // prepare yolo config
    yolo_trt::Config config_v4;
    std::string package_path = ros::package::getPath("zetton_inference");
    config_v4.net_type = yolo_trt::ModelType::YOLOV4_TINY;
    config_v4.file_model_cfg = package_path + "/asset/yolov4-tiny-usv-16.cfg";
    config_v4.file_model_weights = package_path + "/asset/yolov4-tiny-usv-16_best.weights";
    config_v4.inference_precision = yolo_trt::Precision::FP16;
    config_v4.detect_thresh = 0.4;

    // initialize detector
    detector_.Init(config_v4);
    detector_.SetWidthLimitation(50, 1920);
    detector_.SetHeightLimitation(50, 1920);
  }

  ~RosYoloObjectDetector()
  {
    if (nh_)
      delete nh_;
  }
};

int main(int argc, char **argv)
{
  // init node
  ros::init(argc, argv, "example_ros_yolo_detector");
  auto nh = new ros::NodeHandle("~");

  // catch external interrupt initiated by the user and exit program
  signal(SIGINT, signalHandler);

  // init instance
  RosYoloObjectDetector detector(nh);

  ros::spin();
  return 0;
}