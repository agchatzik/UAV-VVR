/** @file demo_flight_control.cpp
 *  @version 3.3
 *  @date September, 2017
 *
 *  @brief
 *  demo sample of how to use Local position control
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */
#include "dji_sdk_demo/demo_flight_control.h"
#include "dji_sdk_demo/landing_local_position_control.h"
#include "dji_sdk/dji_sdk.h"
#include "std_msgs/Bool.h"

ros::ServiceClient set_local_pos_reference;
ros::ServiceClient sdk_ctrl_authority_service;
ros::ServiceClient drone_task_service;
ros::ServiceClient query_version_service;

ros::Publisher ctrlPosYawPub;

ros::Publisher in_hover_pub;
std_msgs::Bool hover;

// global variables for subscribed topics
uint8_t flight_status = 255;
uint8_t display_mode  = 255;
uint8_t current_gps_health = 0;
int num_targets = 0;
geometry_msgs::PointStamped local_position;
geometry_msgs::Quaternion current_atti;
sensor_msgs::NavSatFix current_gps_position;
bool landing_is_allowed = false;
float cmdz = 2.0;
float cmdx, cmdy;
float x_pos,y_pos,z_pos;
double yawInRad, pitchInRad, rollInRad;
float vel_x, vel_y;

int FLAG = 0;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "demo_local_position_control_node");
    ros::NodeHandle nh;
    // Subscribe to messages from dji_sdk_node
    ros::Subscriber landingPoint   = nh.subscribe("/landing_point", 10, &landing_point_callback);
    ros::Subscriber allowedLanding  = nh.subscribe("/landing_allowed", 10, &landing_allowed_callback);
    ros::Subscriber flightStatusSub = nh.subscribe("dji_sdk/flight_status", 10, &flight_status_callback);
    ros::Subscriber displayModeSub = nh.subscribe("dji_sdk/display_mode", 10, &display_mode_callback);
    ros::Subscriber localPosition = nh.subscribe("dji_sdk/local_position", 10, &local_position_callback);
    ros::Subscriber gpsSub      = nh.subscribe("dji_sdk/gps_position", 10, &gps_position_callback);
    ros::Subscriber gpsHealth      = nh.subscribe("dji_sdk/gps_health", 10, &gps_health_callback);
    ros::Subscriber attitudeSub = nh.subscribe("dji_sdk/attitude", 10, &attitude_callback);
    ros::Subscriber velocitySub =  nh.subscribe("/guidance/velocity",10, &velocity_callback);

    // Publish the control signal
    ctrlPosYawPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUposition_yaw", 10);
    // Basic services
    sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
    drone_task_service         = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
    query_version_service      = nh.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");
    set_local_pos_reference    = nh.serviceClient<dji_sdk::SetLocalPosRef> ("dji_sdk/set_local_pos_ref");
  
 sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");

    in_hover_pub = nh.advertise<std_msgs::Bool>("is_hover", 10);

    bool obtain_control_result = obtain_control();
    bool takeoff_result;
    if (!set_local_position())
    {
        ROS_ERROR("GPS health insufficient - No local frame reference for height. Exiting.");
        return 1;
    }

    
   // if(landing_is_allowed)
    //{
/*
        ROS_INFO("M100 taking off!");
        takeoff_result = M100monitoredTakeoff();
    }
    else
    {
        ROS_INFO("A3/N3 taking off!");
        takeoff_result = monitoredTakeoff();
    }

    if(takeoff_result)
    {
*/
        //! Enter total number of Targets
       // num_targets = 1;
        //! Start Mission by setting Target state to 1
       // target_set_state = 1;
   //}

    //////////////////////////


    ros::spin();
    return 0;
}

/*!
 * This function is called when local position data is available.
 * In the example below, we make use of two arbitrary targets as
 * an example for local position control.
 *
 */
void landing_allowed_callback(const std_msgs::Bool::ConstPtr& allowed){
	landing_is_allowed = allowed->data;

	x_pos = local_position.point.x;
	y_pos = local_position.point.y;
        z_pos = local_position.point.z;

        yawInRad = toEulerAngle(current_atti).z;
        ROS_INFO("caurrent YAW %f",yawInRad);

	ROS_INFO("local_position x %f", x_pos);
	ROS_INFO("local_position y %f", y_pos);

	int upper = 6;
	int lower = -6;

        if(landing_is_allowed){
		
        //! Enter total number of Targets
          num_targets = 1;
        //! Start Mission by setting Target state to 1
          target_set_state = 1;
        }else{

		if(FLAG == 0){

			//! Enter total number of Targets
			  num_targets = 1;
			//! Start Mission by setting Target state to 1
			  target_set_state = 4;

			  cmdx = (rand()%(upper - lower + 1)) + lower;
			  cmdy = (rand()%(upper - lower + 1)) + lower;

			  if(z_pos > 18.0) cmdz = 0;

		}
	}
}

void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg) {
    static ros::Time start_time = ros::Time::now();
    ros::Duration elapsed_time = ros::Time::now() - start_time;
    local_position = *msg;
    double xCmd, yCmd, zCmd;
    sensor_msgs::Joy controlPosYaw;

    pitchInRad = toEulerAngle(current_atti).y;
    rollInRad = toEulerAngle(current_atti).x;

    if ((abs(pitchInRad) < 0.09)&&(abs(rollInRad) < 0.09)&&(abs(vel_x)<0.05)&&(abs(vel_y)<0.05)) hover.data = true;
    else hover.data = false;

    //ROS_INFO("IN HOVER %d", hover.data);
    in_hover_pub.publish(hover);

    // Down sampled to 50Hz loop
    if (elapsed_time > ros::Duration(0.02)) {
        start_time = ros::Time::now();
        if (target_set_state == 1) {
            //! First arbitrary target
            //if (current_gps_health > 3) {

                //setTarget();
               
                local_position_ctrl(xCmd, yCmd, zCmd);	
		

            //}
            //else
            //{
               // ROS_INFO("Cannot execute Local Position Control");
               // ROS_INFO("Not enough GPS Satellites");
                //! Set Target set state to 0 in order to stop Local position control mission
               // target_set_state = 0;
           // }
        }
        else if(target_set_state == 2) {
            //! Land on
                FLAG =1;
                ROS_INFO("M100 Land on!");
                land_on();
		//land_on();
                //target_set_state ++;///////////////////////////////////////////////////////////////
        }
        else if(target_set_state == 4) {
            //! First arbitrary target
            //if (current_gps_health > 3) {
	       
               setTarget(cmdx + x_pos, cmdy + y_pos, cmdz + z_pos, 1.5708);              
               local_position_ctrl(xCmd, yCmd, zCmd);
	       //comd +=5;
            //}
            //else
            //{
               // ROS_INFO("Cannot execute Local Position Control");
               // ROS_INFO("Not enough GPS Satellites");
                //! Set Target set state to 0 in order to stop Local position control mission
               // target_set_state = 0;
        }
    }
}

/*!
 * This function calculates the difference between target and current local position
 * and sends the commands to the Position and Yaw control topic.
 *
 */
void local_position_ctrl(double &xCmd, double &yCmd, double &zCmd)
{
    xCmd = target_offset_x - local_position.point.x;
    yCmd = target_offset_y - local_position.point.y;
    zCmd = target_offset_z;
        // ROS_INFO("inside y %f", target_offset_y);
    sensor_msgs::Joy controlPosYaw;
    controlPosYaw.axes.push_back(xCmd);
    controlPosYaw.axes.push_back(yCmd);
    controlPosYaw.axes.push_back(zCmd);
    controlPosYaw.axes.push_back(target_yaw);
    ctrlPosYawPub.publish(controlPosYaw);

    // 0.1m or 10cms is the minimum error to reach target in x y and z axes.
    // This error threshold will have to change depending on aircraft/payload/wind conditions.
    if (((std::abs(xCmd)) < 0.2) && ((std::abs(yCmd)) < 0.2) &&
        (local_position.point.z > (target_offset_z - 0.2)) && (local_position.point.z < (target_offset_z + 0.2))) {
        if(target_set_state <= num_targets) {
            ROS_INFO("%d of %d target(s) complete", target_set_state, num_targets);
            target_set_state ++;
   
        }
        else
        {
            target_set_state = 0;
        }
    }
}

void landing_point_callback(const PointCloud::ConstPtr& msg)
{

   // pcl::PointXYZ lp = msg->points.at(0);
   // if(FLAG == 0) setTarget(lp.x + x_pos, -lp.y + y_pos, 2, 1.5708);
    
   // ROS_INFO("X COORDINATE  %f",lp.x );


    pcl::PointXYZ lp = msg->points.at(0);
    //setTarget(lp.x + x_pos, -lp.y + y_pos, -2+z_pos, yawInRad);


    cmdy = (-lp.y)*cos(1.5708-yawInRad) - lp.x*sin(1.5708-yawInRad);
    cmdx = (-lp.y)*sin(1.5708-yawInRad) + lp.x*cos(1.5708-yawInRad);

    //ROS_INFO("X cmd  %f",cmdx );
    //ROS_INFO("Y cmd  %f",cmdy );  

    setTarget(cmdx + x_pos, cmdy + y_pos, -2+z_pos, yawInRad);
}

bool takeoff_land(int task)
{
    dji_sdk::DroneTaskControl droneTaskControl;

    droneTaskControl.request.task = task;

    drone_task_service.call(droneTaskControl);

    if(!droneTaskControl.response.result)
    {
        ROS_ERROR("takeoff_land fail");
        return false;
    }

    return true;
}

void velocity_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
  vel_x =  msg->vector.x;
  vel_y = msg->vector.y;
}

void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
  current_atti = msg->quaternion;
}
geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat)
{
  geometry_msgs::Vector3 ans;

  tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
  R_FLU2ENU.getRPY(ans.x, ans.y, ans.z);
  return ans;
}
///////////////////
bool land_on()
{
    dji_sdk::DroneTaskControl droneTaskControl;

    droneTaskControl.request.task = 6;

    drone_task_service.call(droneTaskControl);

    if(!droneTaskControl.response.result)
    {
        ROS_ERROR("Land on fail");
        return false;
    }

    return true;
}
/////////////////////////
bool obtain_control()
{
    dji_sdk::SDKControlAuthority authority;
    authority.request.control_enable=1;
    sdk_ctrl_authority_service.call(authority);

    if(!authority.response.result)
    {
        ROS_ERROR("obtain control failed!");
        return false;
    }

    return true;
}

bool is_M100()
{
    dji_sdk::QueryDroneVersion query;
    query_version_service.call(query);

    if(query.response.version == DJISDK::DroneFirmwareVersion::M100_31)
    {
        return true;
    }

    return false;
}


void gps_position_callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    current_gps_position = *msg;
}

void gps_health_callback(const std_msgs::UInt8::ConstPtr& msg) {
    current_gps_health = msg->data;
}

void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg)
{
    flight_status = msg->data;
}

void display_mode_callback(const std_msgs::UInt8::ConstPtr& msg)
{
    display_mode = msg->data;
}


/*!
 * This function demos how to use the flight_status
 * and the more detailed display_mode (only for A3/N3)
 * to monitor the take off process with some error
 * handling. Note M100 flight status is different
 * from A3/N3 flight status.
 */
bool
monitoredTakeoff()
{
    ros::Time start_time = ros::Time::now();

    if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF)) {
        return false;
    }

    ros::Duration(0.01).sleep();
    ros::spinOnce();

    // Step 1.1: Spin the motor
    while (flight_status != DJISDK::FlightStatus::STATUS_ON_GROUND &&
           display_mode != DJISDK::DisplayMode::MODE_ENGINE_START &&
           ros::Time::now() - start_time < ros::Duration(5)) {
        ros::Duration(0.01).sleep();
        ros::spinOnce();
    }

    if(ros::Time::now() - start_time > ros::Duration(5)) {
        ROS_ERROR("Takeoff failed. Motors are not spinnning.");
        return false;
    }
    else {
        start_time = ros::Time::now();
        ROS_INFO("Motor Spinning ...");
        ros::spinOnce();
    }


    // Step 1.2: Get in to the air
    while (flight_status != DJISDK::FlightStatus::STATUS_IN_AIR &&
           (display_mode != DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF || display_mode != DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
           ros::Time::now() - start_time < ros::Duration(20)) {
        ros::Duration(0.01).sleep();
        ros::spinOnce();
    }

    if(ros::Time::now() - start_time > ros::Duration(20)) {
        ROS_ERROR("Takeoff failed. Aircraft is still on the ground, but the motors are spinning.");
        return false;
    }
    else {
        start_time = ros::Time::now();
        ROS_INFO("Ascending...");
        ros::spinOnce();
    }

    // Final check: Finished takeoff
    while ( (display_mode == DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF || display_mode == DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
            ros::Time::now() - start_time < ros::Duration(20)) {
        ros::Duration(0.01).sleep();
        ros::spinOnce();
    }

    if ( display_mode != DJISDK::DisplayMode::MODE_P_GPS || display_mode != DJISDK::DisplayMode::MODE_ATTITUDE)
    {
        ROS_INFO("Successful takeoff!");
        start_time = ros::Time::now();
    }
    else
    {
        ROS_ERROR("Takeoff finished, but the aircraft is in an unexpected mode. Please connect DJI GO.");
        return false;
    }

    return true;
}


/*!
 * This function demos how to use M100 flight_status
 * to monitor the take off process with some error
 * handling. Note M100 flight status is different
 * from A3/N3 flight status.
 */
bool
M100monitoredTakeoff()
{
    ros::Time start_time = ros::Time::now();

    float home_altitude = current_gps_position.altitude;
    if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF))
    {
        return false;
    }

    ros::Duration(0.01).sleep();
    ros::spinOnce();

    // Step 1: If M100 is not in the air after 10 seconds, fail.
    while (ros::Time::now() - start_time < ros::Duration(10))
    {
        ros::Duration(0.01).sleep();
        ros::spinOnce();
    }

    if(flight_status != DJISDK::M100FlightStatus::M100_STATUS_IN_AIR ||
       current_gps_position.altitude - home_altitude < 1.0)
    {
        ROS_ERROR("Takeoff failed.");
        return false;
    }
    else
    {
        start_time = ros::Time::now();
        ROS_INFO("Successful takeoff!");
        ros::spinOnce();
    }
    return true;
}

bool set_local_position()
{
    dji_sdk::SetLocalPosRef localPosReferenceSetter;
    set_local_pos_reference.call(localPosReferenceSetter);

    return (bool)localPosReferenceSetter.response.result;
}
