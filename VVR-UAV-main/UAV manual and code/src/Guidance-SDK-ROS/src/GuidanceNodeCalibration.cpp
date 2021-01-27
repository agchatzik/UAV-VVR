/*
 * GuidanceNode.cpp
 *
 *  Created on: Apr 29, 2015
 */

#include <stdio.h>
#include <string.h>
#include <iostream>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <ctime>
#include <fstream>


#include <opencv2/opencv.hpp>
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/ximgproc.hpp"

#include "DJI_guidance.h"
#include "DJI_utility.h"

#include <geometry_msgs/TransformStamped.h> //IMU
#include <geometry_msgs/Vector3Stamped.h> //velocity
#include <sensor_msgs/LaserScan.h> //obstacle distance & ultrasonic
#include <sensor_msgs/CameraInfo.h> // camera info message. Contains cam params
#include "yaml-cpp/yaml.h" // use to parse YAML calibration file
#include <fstream> // required to parse YAML

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <message_filters/time_synchronizer.h>

ros::Publisher disp_image_pub;

ros::Publisher depth_image_pub;
ros::Publisher left_image_pub;
ros::Publisher right_image_pub;
ros::Publisher imu_pub;
ros::Publisher obstacle_distance_pub;
ros::Publisher velocity_pub;
ros::Publisher ultrasonic_pub;
ros::Publisher cam_info_left_pub; // camera info msg publishers
ros::Publisher cam_info_right_pub;

ros::Publisher image_pub;
ros::Publisher camera_info_pub;

using namespace cv;
using namespace cv::ximgproc;
using namespace std;

using namespace sensor_msgs;
using namespace message_filters;

#define WIDTH 320
#define HEIGHT 240
#define IMAGE_SIZE (HEIGHT * WIDTH)

char        	key       = 0;
e_vbus_index	CAMERA_ID = e_vbus5;
DJI_lock        g_lock;
DJI_event       g_event;
Mat             g_greyscale_image_left(HEIGHT, WIDTH, CV_8UC1);
Mat				g_greyscale_image_right(HEIGHT, WIDTH, CV_8UC1);
Mat				g_depth(HEIGHT,WIDTH,CV_16SC1);
Mat				depth8(HEIGHT, WIDTH, CV_8UC1);

Mat				depth8b(HEIGHT, WIDTH, CV_8UC1);
Mat				disp8b;

Mat             g_disparity, disp8;
Mat             filt_disp, filt_depth;

//counter for image capture
int image_counter = 0;


//set time now
time_t now = time(0);
tm *ltm = localtime(&now);

std::ostream& operator<<(std::ostream& out, const e_sdk_err_code value) {
    const char* s = 0;
    static char str[100]= {0};
#define PROCESS_VAL(p) case(p): s = #p; break;
    switch(value) {
        PROCESS_VAL(e_OK);
        PROCESS_VAL(e_load_libusb_err);
        PROCESS_VAL(e_sdk_not_inited);
        PROCESS_VAL(e_disparity_not_allowed);
        PROCESS_VAL(e_image_frequency_not_allowed);
        PROCESS_VAL(e_config_not_ready);
        PROCESS_VAL(e_online_flag_not_ready);
        PROCESS_VAL(e_stereo_cali_not_ready);
        PROCESS_VAL(e_libusb_io_err);
        PROCESS_VAL(e_timeout);
    default:
        strcpy(str, "Unknown error");
        s = str;
        break;
    }
#undef PROCESS_VAL

    return out << s;
}

// A nice TODO will be to wrap the driver using the ROS standard cam_info_manager.
// Hackish code to read cam params from YAML
// adapted from cam_info_manager and camera_calibration_parser https://github.com/ros-perception/image_common/blob/hydro-devel/camera_calibration_parsers/src/parse_yml.cpp

std::string camera_params_left;
std::string camera_params_right;

static const char CAM_YML_NAME[]    = "camera_name";
static const char WIDTH_YML_NAME[]  = "image_width";
static const char HEIGHT_YML_NAME[] = "image_height";
static const char K_YML_NAME[]      = "camera_matrix";
static const char D_YML_NAME[]      = "distortion_coefficients";
static const char R_YML_NAME[]      = "rectification_matrix";
static const char P_YML_NAME[]      = "projection_matrix";
static const char DMODEL_YML_NAME[] = "distortion_model";

// struct to parse camera calibration YAML
struct SimpleMatrix
{
    int rows;
    int cols;
    double* data;

    SimpleMatrix(int rows, int cols, double* data)
        : rows(rows), cols(cols), data(data)
    {}
};

void transfer_SimpleMatrix_from_YML_to_ROSmsg(const YAML::Node& node, SimpleMatrix& m)
{
    int rows, cols;
    rows = node["rows"].as<int>();
    cols = node["cols"].as<int>();
    const YAML::Node& data = node["data"];
    for (int i = 0; i < rows*cols; ++i)
    {
        m.data[i] = data[i].as<double>();
    }
}

void read_params_from_yaml_and_fill_cam_info_msg(std::string& file_name, sensor_msgs::CameraInfo& cam_info)
{

    std::ifstream fin(file_name.c_str());
    YAML::Node doc = YAML::Load(fin);

    cam_info.width = doc[WIDTH_YML_NAME].as<int>();
    cam_info.height = doc[HEIGHT_YML_NAME].as<int>();

    SimpleMatrix K_(3, 3, &cam_info.K[0]);
    transfer_SimpleMatrix_from_YML_to_ROSmsg(doc[K_YML_NAME], K_);
    SimpleMatrix R_(3, 3, &cam_info.R[0]);
    transfer_SimpleMatrix_from_YML_to_ROSmsg(doc[R_YML_NAME], R_);
    SimpleMatrix P_(3, 4, &cam_info.P[0]);
    transfer_SimpleMatrix_from_YML_to_ROSmsg(doc[P_YML_NAME], P_);

    cam_info.distortion_model = doc[DMODEL_YML_NAME].as<std::string>();

    const YAML::Node& D_node = doc[D_YML_NAME];
    int D_rows, D_cols;
    D_rows = D_node["rows"].as<int>();
    D_cols = D_node["cols"].as<int>();
    const YAML::Node& D_data = D_node["data"];
    cam_info.D.resize(D_rows*D_cols);
    for (int i = 0; i < D_rows*D_cols; ++i)
    {
        cam_info.D[i] = D_data[i].as<float>();
    }
}
// Compute ROI for block matching
Rect computeROI(Size2i src_sz, Ptr<StereoMatcher> matcher_instance)
{
    int min_disparity = matcher_instance->getMinDisparity();
    int num_disparities = matcher_instance->getNumDisparities();
    int block_size = matcher_instance->getBlockSize();

    int bs2 = block_size/2;
    int minD = min_disparity, maxD = min_disparity + num_disparities - 1;

    int xmin = maxD + bs2;
    int xmax = src_sz.width + minD - bs2;
    int ymin = bs2;
    int ymax = src_sz.height - bs2;

    Rect r(xmin, ymin, xmax - xmin, ymax - ymin);
    return r;
}
void callback(const ImageConstPtr& image,  const CameraInfoConstPtr& cam_info)
{
    // Solve all of perception here...
    ROS_INFO("Sync_Callback");

    image_pub.publish(image);
    camera_info_pub.publish(cam_info);
}
int my_callback(int data_type, int data_len, char *content)
{
    /// image counter raize
    image_counter ++;

    g_lock.enter();

    /* image data */
    if (e_image == data_type && NULL != content)
    {
        ros::Time time_in_this_loop = ros::Time::now();
        image_data* data = (image_data*)content;

        if ( data->m_greyscale_image_left[CAMERA_ID] ) {
            memcpy(g_greyscale_image_left.data, data->m_greyscale_image_left[CAMERA_ID], IMAGE_SIZE);
            //imshow("left",  g_greyscale_image_left);
            // publish left greyscale image
            cv_bridge::CvImage left_8;
            g_greyscale_image_left.copyTo(left_8.image);
            left_8.header.frame_id  = "guidance";
            left_8.header.stamp	= time_in_this_loop;
            left_8.encoding		= sensor_msgs::image_encodings::MONO8;
            left_image_pub.publish(left_8.toImageMsg());

            sensor_msgs::CameraInfo g_cam_info_left;
            g_cam_info_left.header.stamp = time_in_this_loop;
            g_cam_info_left.header.frame_id = "guidance";

            try {
                read_params_from_yaml_and_fill_cam_info_msg(camera_params_left, g_cam_info_left);
                cam_info_left_pub.publish(g_cam_info_left);
            } catch(...) {
                // if yaml fails to read data, don't try to publish
            }


//        std::ostringstream image_distance;
//        image_distance << "/home/ubuntu/Desktop/Drone_stereo_Images/images_left/Scene"<< 1 + ltm->tm_hour <<":"<< 1 + ltm->tm_min <<1 + ltm->tm_sec<<"Gray_Image_left"<<image_counter<<".jpg";
 //       std::string copyOfStr = image_distance.str();

 //       imwrite(copyOfStr, g_greyscale_image_left );

        }
        if ( data->m_greyscale_image_right[CAMERA_ID] ) {
            memcpy(g_greyscale_image_right.data, data->m_greyscale_image_right[CAMERA_ID], IMAGE_SIZE);
            //imshow("right", g_greyscale_image_right);
            // publish right greyscale image
            cv_bridge::CvImage right_8;
            g_greyscale_image_right.copyTo(right_8.image);
            right_8.header.frame_id  = "guidance";
            right_8.header.stamp	 = time_in_this_loop;
            right_8.encoding  	 = sensor_msgs::image_encodings::MONO8;
            right_image_pub.publish(right_8.toImageMsg());

            sensor_msgs::CameraInfo g_cam_info_right;
            g_cam_info_right.header.stamp = time_in_this_loop;
            g_cam_info_right.header.frame_id = "guidance";

            try {
                read_params_from_yaml_and_fill_cam_info_msg(camera_params_right, g_cam_info_right);
                cam_info_right_pub.publish(g_cam_info_right);
            } catch(...) {
                // if yaml fails to read data, don't try to publish
            }

    //        std::ostringstream image_distance;
    //        image_distance << "/home/ubuntu/Desktop/Drone_stereo_Images/images_right/Scene"<< 1 + ltm->tm_hour <<":"<< 1 + ltm->tm_min << 1 + ltm->tm_sec <<"Gray_Image_right"<<image_counter<<".jpg";
   //         std::string copyOfStr = image_distance.str();

    //        imwrite(copyOfStr, g_greyscale_image_right );
        
        }
   }



    /* velocity */
    if ( e_velocity == data_type && NULL != content )
    {
        velocity *vo = (velocity*)content;
        //printf( "frame index: %d, stamp: %d\n", vo->frame_index, vo->time_stamp );
        //printf( "vx:%f vy:%f vz:%f\n", 0.001f * vo->vx, 0.001f * vo->vy, 0.001f * vo->vz );

        // publish velocity
        geometry_msgs::Vector3Stamped g_vo;
        g_vo.header.frame_id = "guidance";
        g_vo.header.stamp    = ros::Time::now();
        g_vo.vector.x = 0.001f * vo->vx;
        g_vo.vector.y = 0.001f * vo->vy;
        g_vo.vector.z = 0.001f * vo->vz;
        velocity_pub.publish(g_vo);
    }



    g_lock.leave();
    g_event.set_event();

    return 0;
}

#define RETURN_IF_ERR(err_code) { if( err_code ){ release_transfer(); \
std::cout<<"Error: "<<(e_sdk_err_code)err_code<<" at "<<__LINE__<<","<<__FILE__<<std::endl; return -1;}}

int main(int argc, char** argv)
{
   //CommandLineParser parser(argc,argv,keys);

    if(argc>1) {
        printf("This is demo program showing data from Guidance.\n\t"
               " 'a','d','w','s','x' to select sensor direction.\n\t"
               " 'j','k' to change the exposure parameters.\n\t"
               " 'm' to switch between AEC and constant exposure modes.\n\t"
               " 'n' to return to default exposure mode and parameters.\n\t"
               " 'q' to quit.");
        return 0;
    }

    /* initialize ros */
    ros::init(argc, argv, "GuidanceNode");
    ros::NodeHandle my_node;
    my_node.getParam("/left_param_file", camera_params_left);
    my_node.getParam("/right_param_file", camera_params_right);


    left_image_pub			= my_node.advertise<sensor_msgs::Image>("/guidance/left/image_raw",1);
    right_image_pub			= my_node.advertise<sensor_msgs::Image>("/guidance/right/image_raw",1);
    //imu_pub  				= my_node.advertise<geometry_msgs::TransformStamped>("/guidance/imu",1);
    velocity_pub  			= my_node.advertise<geometry_msgs::Vector3Stamped>("/guidance/velocity",1);
    cam_info_right_pub      = my_node.advertise<sensor_msgs::CameraInfo>("/guidance/right/camera_info",1);
    cam_info_left_pub       = my_node.advertise<sensor_msgs::CameraInfo>("/guidance/left/camera_info",1);

    /* initialize guidance */
    reset_config();
    int err_code = init_transfer();
    RETURN_IF_ERR(err_code);

    int online_status[CAMERA_PAIR_NUM];
    err_code = get_online_status(online_status);
    RETURN_IF_ERR(err_code);
    std::cout<<"Sensor online status: ";
    for (int i=0; i<CAMERA_PAIR_NUM; i++)
        std::cout<<online_status[i]<<" ";
    std::cout<<std::endl;

    // get cali param
    stereo_cali cali[CAMERA_PAIR_NUM];
    err_code = get_stereo_cali(cali);
    RETURN_IF_ERR(err_code);
    std::cout<<"cu\tcv\tfocal\tbaseline\n";
    for (int i=0; i<CAMERA_PAIR_NUM; i++)
    {
        std::cout<<cali[i].cu<<"\t"<<cali[i].cv<<"\t"<<cali[i].focal<<"\t"<<cali[i].baseline<<std::endl;
    }

    /* select data */
    err_code = select_greyscale_image(CAMERA_ID, true);
    RETURN_IF_ERR(err_code);
    err_code = select_greyscale_image(CAMERA_ID, false);
    RETURN_IF_ERR(err_code);
  //  err_code = select_depth_image(CAMERA_ID);
 //   RETURN_IF_ERR(err_code);
  //  select_imu();
  //  select_ultrasonic();
 //   select_obstacle_distance();
    select_velocity();
    /* start data transfer */
    err_code = set_sdk_event_handler(my_callback);
    RETURN_IF_ERR(err_code);
    err_code = start_transfer();
    RETURN_IF_ERR(err_code);

    // for setting exposure
    exposure_param para;
    para.m_is_auto_exposure = 1;
    para.m_step = 10;
    para.m_expected_brightness = 120;
    para.m_camera_pair_index = CAMERA_ID;


    /////// imu file
  
    


    std::cout << "start_transfer" << std::endl;

    while (ros::ok())
    {
        g_event.wait_event();
        if (key > 0)
            // set exposure parameters
            if(key=='j' || key=='k' || key=='m' || key=='n') {
                if(key=='j')
                    if(para.m_is_auto_exposure) para.m_expected_brightness += 20;
                    else para.m_exposure_time += 3;
                else if(key=='k')
                    if(para.m_is_auto_exposure) para.m_expected_brightness -= 20;
                    else para.m_exposure_time -= 3;
                else if(key=='m') {
                    para.m_is_auto_exposure = !para.m_is_auto_exposure;
                    std::cout<<"exposure is "<<para.m_is_auto_exposure<<std::endl;
                }
                else if(key=='n') { //return to default
                    para.m_expected_brightness = para.m_exposure_time = 0;
                }

                std::cout<<"Setting exposure parameters....SensorId="<<CAMERA_ID<<std::endl;
                para.m_camera_pair_index = CAMERA_ID;
                set_exposure_param(&para);
                key = 0;
            }
            else {// switch image direction
                err_code = stop_transfer();
                RETURN_IF_ERR(err_code);
                reset_config();

                if (key == 'q') break;
                if (key == 'w') CAMERA_ID = e_vbus1;
                if (key == 'd') CAMERA_ID = e_vbus2;
                if (key == 'x') CAMERA_ID = e_vbus3;
                if (key == 'a') CAMERA_ID = e_vbus4;
                if (key == 's') CAMERA_ID = e_vbus5;

                select_greyscale_image(CAMERA_ID, true);
                select_greyscale_image(CAMERA_ID, false);
                select_depth_image(CAMERA_ID);

                err_code = start_transfer();
                RETURN_IF_ERR(err_code);
                key = 0;
            }
        ros::spinOnce();
    }

    /* release data transfer */
    err_code = stop_transfer();
    RETURN_IF_ERR(err_code);
    //make sure the ack packet from GUIDANCE is received
    sleep(1);
    std::cout << "release_transfer" << std::endl;
    err_code = release_transfer();
    RETURN_IF_ERR(err_code);
    
  
    return 0;
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
