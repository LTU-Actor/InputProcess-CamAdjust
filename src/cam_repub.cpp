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

image_transport::Publisher *pub = 0;
image_transport::Publisher *pub_orig = 0;
image_transport::Subscriber *img_input = 0;

cv::Ptr<cv::CLAHE> clahe;

dynamic_reconfigure::Server<ltu_actor_inputprocess_camadjust::CamPubConfig> *server = 0;
dynamic_reconfigure::Server<ltu_actor_inputprocess_camadjust::CamPubConfig>::CallbackType dynConfigCB_;

void dynConfigCB(ltu_actor_inputprocess_camadjust::CamPubConfig &config_, uint32_t level)
{
    config = config_;
}

void inputCB(const sensor_msgs::ImageConstPtr &input)
{
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(input, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exceptions: %s", e.what());
        return;
    }

    sensor_msgs::ImagePtr msg;
    cv::Mat frame = cv_ptr->image;

    cv::resize(frame, frame, cv::Size(), config.resize, config.resize, cv::INTER_AREA);

      if(pub_orig->getNumSubscribers() > 0)
          pub_orig->publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg());

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
      if(pub->getNumSubscribers() > 0)
         pub->publish(msg);
      cv::waitKey(3);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cam_repub");

  ros::NodeHandle nh{"~"};
  image_transport::ImageTransport it(nh);

  image_transport::Publisher pub_;
  image_transport::Publisher pub_orig_;
  image_transport::Subscriber img_input_;
  dynamic_reconfigure::Server<ltu_actor_inputprocess_camadjust::CamPubConfig> server_;

  pub = &pub_;
  pub_orig = &pub_orig_;
  img_input = &img_input_;
  server = &server_;

  pub_ = it.advertise("image_raw", 1);
  pub_orig_ = it.advertise("image_orig", 1);
  img_input_ = it.subscribe("/camera/image_raw", 1, &inputCB, 0);

  dynConfigCB_ = boost::bind(&dynConfigCB, _1, _2);
  clahe = cv::createCLAHE();
  server_.setCallback(dynConfigCB_);
  server_.getConfigDefault(config);

  ros::spin();
}
