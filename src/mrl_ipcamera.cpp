#include "mrl_ipcamera/mrl_ipcamera.hpp"
#include <opencv2/video/video.hpp>
#include <sensor_msgs/image_encodings.h>

MrlIpCamera::MrlIpCamera(ros::NodeHandle *nodeHandle) : nh_(nodeHandle),
                                                        imagetransport_(*nodeHandle)
{    
    nh_->param<std::string>("video_url", video_url_, "http://192.168.10.33:80/mjpeg?video.mjpg");
    ROS_INFO_STREAM("Video_url is " << video_url_);
    nh_->param<std::string>("topic", topic_, "/camera/image");
    nh_->param<std::string>("frame_id", frame_id_, "manipulator_cam_link");
    
    camera_pub_ = imagetransport_.advertiseCamera(topic_, 10);
    refresh_serviceServer_ = nh_->advertiseService("refresh", &MrlIpCamera::refreshSrvCallback, this);
    cap_.open(video_url_);
}

MrlIpCamera::~MrlIpCamera()
{
}

bool MrlIpCamera::publish()
{
    cv::Mat frame;
    ros::Rate loop(33);
    while (ros::ok())
    {
        if (cap_.isOpened())
        {
            ROS_INFO_ONCE("Connection established");
            cap_ >> frame;
            if (!frame.empty())
            {
                cv_bridge::CvImage out_msg;
                out_msg.header.frame_id = frame_id_;
                out_msg.header.stamp = ros::Time::now();
                out_msg.encoding = sensor_msgs::image_encodings::BGR8;
                out_msg.image = frame;
                sensor_msgs::CameraInfo camera_info;
                sensor_msgs::Image rosimg;
                out_msg.toImageMsg(rosimg);
                camera_pub_.publish(rosimg, camera_info, ros::Time::now());
            }
            else
            {
                cap_.release();
            }
        }
        else
        {
            cap_.open(video_url_);
            ROS_ERROR_STREAM("IP Camera is not avaliable");
        }

        ros::spinOnce();
        loop.sleep();
    }
    return true;
}

bool MrlIpCamera::refreshSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    ROS_INFO("Refreshing");
    cap_.release();
    if (!cap_.open(video_url_))
    {
        ROS_ERROR_STREAM("Connecting to " << video_url_ << " failed.");
    }
    return true;
}
