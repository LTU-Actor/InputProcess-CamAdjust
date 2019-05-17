#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp> // TODO: opencv3?
#include <cv_bridge/cv_bridge.h>
#include <ctime>
#include <cstdlib>
#include <string>
#include <dynamic_reconfigure/server.h>
#include <ltu_actor_inputprocess_camadjust/CamPubConfig.h>
#include <vector>

/**
 * Image publisher for cv images
 * Publishes images at ~30fps
 */

ltu_actor_inputprocess_camadjust::CamPubConfig config;

void dynConfigCB(ltu_actor_inputprocess_camadjust::CamPubConfig &config_, uint32_t level)
{
    config = config_;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cam_pub");

  std::string source;
  ros::NodeHandle nh("~");
  cv::VideoCapture cap;

  // Open the video source
  if (nh.getParam("source", source))
  {
    cap.open(source);
    ROS_INFO_STREAM("cam_pub: publishing using video source " << source << "...");
  } else {
    cap.open(0);
    ROS_INFO_STREAM("param '~source' not defined, using default camera 0");
  }

  if(!cap.isOpened()) {
    ROS_ERROR_STREAM("video device cannot be opened");
    return 1;
  }

  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("image_raw", 1);
  image_transport::Publisher pub_orig = it.advertise("image_orig", 1);

  cv::Ptr<cv::CLAHE> clahe;

  dynamic_reconfigure::Server<ltu_actor_inputprocess_camadjust::CamPubConfig> server;
  dynamic_reconfigure::Server<ltu_actor_inputprocess_camadjust::CamPubConfig>::CallbackType dynConfigCB_;

  ros::Rate loop_rate(30);
  bool hflip = false;
  int empty_frame_count = 0;

  dynConfigCB_ = boost::bind(&dynConfigCB, _1, _2);
  nh.getParam("hflip", hflip);
  clahe = cv::createCLAHE();
  server.setCallback(dynConfigCB_);

  server.getConfigDefault(config);

  while (nh.ok())
  {
    sensor_msgs::ImagePtr msg;
    cv::Mat frame;

    cap >> frame;

    // Check if grabbed frame is actually full with some content
    if(!frame.empty()) {
      empty_frame_count = 0;

      // Flip the image upside down
      if (hflip) cv::flip(frame, frame, -1);

      if(pub_orig.getNumSubscribers() > 0)
          pub_orig.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg());

      if(config.enable_clahe)
      {
          std::vector<cv::Mat> channels(3);
          cv::split(frame, channels);
          clahe->setClipLimit(config.clahe_clip);
          clahe->apply(channels[0], channels[0]);
          clahe->apply(channels[1], channels[1]);
          clahe->apply(channels[2], channels[2]);
          cv::merge(channels, frame);
      }

      if(config.enable_color_correct)
          frame = frame*config.cc_alpha + config.cc_beta;

      if(config.enable_sharpen)
      {
          cv::Mat out;
          cv::GaussianBlur(frame, out, cv::Size(0, 0), config.sharp_kernel*2+1);
          cv::addWeighted(frame, 1.0+config.sharp_weight, out, -1.0*config.sharp_weight, 0, out);
          out.copyTo(frame);
      }

      msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
      pub.publish(msg);
      cv::waitKey(1);

      ros::spinOnce();
      loop_rate.sleep();
    }
    else
    {
      empty_frame_count++;
      if (empty_frame_count > 20) {
        ROS_ERROR_STREAM("Could not read input, closing.");
        return 1;
      }
    }
  }
}
