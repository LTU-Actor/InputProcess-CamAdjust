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
image_transport::Subscriber *img_input = 0;

cv::Ptr<cv::CLAHE> clahe;

dynamic_reconfigure::Server<ltu_actor_inputprocess_camadjust::CamPubConfig> *server = 0;

std::string cam_topic;
bool enabled_;

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

    if(config.resize < 0.99)
        cv::resize(frame, frame, cv::Size(), config.resize, config.resize, cv::INTER_AREA);

    if(config.enable_less_color)
    {
      cv::cvtColor(frame, frame, cv::COLOR_BGR2HSV);
      std::vector<cv::Mat> channels(3);
      cv::split(frame, channels);
      channels[2] -= config.less_color_mux * channels[1];
      cv::merge(channels, frame);
      cv::cvtColor(frame, frame, cv::COLOR_HSV2BGR);
    }

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
    {
        frame.convertTo(frame, CV_16UC3);
        std::vector<cv::Mat> channels(3);
        cv::split(frame, channels);
        channels[0] = channels[0]*config.cc_alpha + config.cc_beta;
        channels[1] = channels[1]*config.cc_alpha + config.cc_beta;
        channels[2] = channels[2]*config.cc_alpha + config.cc_beta;
        cv::merge(channels, frame);
        frame.convertTo(frame, CV_8UC3);
    }

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

bool hasSub(image_transport::Publisher &pub_){
    return (pub_.getNumSubscribers());
}

bool isEnabled(){
    return enabled_;
}

void startup(image_transport::ImageTransport &it, image_transport::Subscriber &img_input_){
    img_input_ = it.subscribe(cam_topic, 1, &inputCB, 0);
    enabled_ = true;
}

void shutdown(image_transport::Subscriber &img_input_){
    img_input_ = image_transport::Subscriber();
    enabled_ =  false;
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
  img_input = &img_input_;
  server = &server_;

  clahe = cv::createCLAHE();
  server_.setCallback(dynConfigCB);
  server_.getConfigDefault(config);

  if (nh.hasParam("resize")) { nh.getParam("resize", config.resize); }
  if (nh.hasParam("enable_less_color")) { nh.getParam("enable_less_color", config.enable_less_color); }
  if (nh.hasParam("less_color_mux")) { nh.getParam("less_color_mux", config.less_color_mux); }
  if (nh.hasParam("enable_clahe")) { nh.getParam("enable_clahe", config.enable_clahe); }
  if (nh.hasParam("clahe_clip")) { nh.getParam("clahe_clip", config.clahe_clip); }
  if (nh.hasParam("enable_color_correct")) { nh.getParam("enable_color_correct", config.enable_color_correct); }
  if (nh.hasParam("cc_alpha")) { nh.getParam("cc_alpha", config.cc_alpha); }
  if (nh.hasParam("cc_beta")) { nh.getParam("cc_beta", config.cc_beta); }
  if (nh.hasParam("enable_sharpen")) { nh.getParam("enable_sharpen", config.enable_sharpen); }
  if (nh.hasParam("sharp_weight")) { nh.getParam("sharp_weight", config.sharp_weight); }
  if (nh.hasParam("sharp_kernel")) { nh.getParam("sharp_kernel", config.sharp_kernel); }
  server_.updateConfig(config);

  if (!nh.getParam("cam_topic", cam_topic))
  {
      ROS_ERROR_STREAM("[FATAL] Sign detection: param 'camera_topic' not defined");
      exit(0);
  }

  pub_ = it.advertise("image", 1);

  ros::Rate r(10);

    while (ros::ok()){
        if (hasSub(pub_)){
            if (!isEnabled()){
                startup(it, img_input_);
            }
        } else {
            if (isEnabled()){
                shutdown(img_input_);
            }
        }
        ros::spinOnce();
        r.sleep();
    }
}
