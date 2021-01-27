#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_publisher_right");
    ros::NodeHandle nh;

    ///Publish right image
    cv_bridge::CvImage cv_image_right;
    cv_image_right.image = cv::imread("/home/ubuntu/Desktop/image_right.jpg",CV_LOAD_IMAGE_GRAYSCALE);
    //cv_image_right.encoding = "MONO8";


    sensor_msgs::ImagePtr ros_image_right;
    //ros_image_right->header.frame_id = "my_frame";
    ros_image_right = cv_bridge::CvImage(std_msgs::Header(), "mono8", cv_image_right.image).toImageMsg();
    //cv_image_right.toImageMsg(ros_image_right);

//    ros_image_right.width = 320;///2;
//    ros_image_right.height = 240;///2;
//    ros_image_right.step = 320*3;

    ros::Publisher pub = nh.advertise<sensor_msgs::Image>("/image_right", 1000);

    ros::Rate loop_rate(2);

    while (nh.ok())
    {
        pub.publish(ros_image_right);
        loop_rate.sleep();
    }
}
