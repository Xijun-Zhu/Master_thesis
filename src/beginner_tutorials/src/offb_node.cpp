#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Twist.h>
#include "GeographicLib/Geoid.hpp"
#include <cmath>
#include <stdio.h>
#include <mavros_msgs/GPSRAW.h>
#include <sensor_msgs/Range.h>

#define GLOBAL_MODE
#define PI                      3.1415926
#define EARTH_RADIUS            6378137        
#define SD_PERIOD               100

bool raw_gps_data_flag = true;
// lib[][0] = alt; lib[][1] = lat; lib[][2] = lon; lib[][3] = distence; lib[][4] = fix_status;
double lib[5000][4];		
int data_cnt;
int fix_status;
// home[0] = alt; home[1] = lat; home[2] = lon;
double home[3];
double tmp_sd;
double tmp_rmse;
double range_value;

static double radian(double d)
{
    return d * PI / 180.0;   //角度1˚ = π / 180
}

double get_distance(double lat1, double lng1, double lat2, double lng2)
{
    double radLat1 = radian(lat1);
    double radLat2 = radian(lat2);
    double a = radLat1 - radLat2;
    double b = radian(lng1) - radian(lng2);
    
    double dst = 2 * asin((sqrt(pow(sin(a / 2), 2) + cos(radLat1) * cos(radLat2) * pow(sin(b / 2), 2) )));
    
    dst = dst * EARTH_RADIUS;
	if(dst<0)
		{
		dst=-dst;
		}
    return dst;
}

double calculateSD() {
    double sum = 0.0, mean, standardDeviation = 0.0;
    int i;

    for(i = 0; i < SD_PERIOD; ++i) {
        sum += lib[data_cnt-SD_PERIOD+i][3];
    }

    mean = sum / SD_PERIOD;

    for(i = 0; i < SD_PERIOD; ++i) {
        standardDeviation += pow(lib[data_cnt-SD_PERIOD+i][3] - mean, 2);
    }

    return sqrt(standardDeviation / SD_PERIOD);
}

// Function that Calculate Root Mean Square
double calculateRMSE()
{
    double square = 0;
    double mean = 0.0, root = 0.0;
 
    // Calculate square.
    for (int i = 0; i < SD_PERIOD; i++) {
        square += pow(lib[data_cnt-SD_PERIOD+i][3], 2);
    }
 
    // Calculate Mean.
    mean = (square / (double)(SD_PERIOD));
 
    // Calculate Root.
    root = sqrt(mean);
 
    return root;
}

// global variables
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

#ifdef GLOBAL_MODE
// sensor_msgs::NavSatFix global_position;
geographic_msgs::GeoPoseStamped global_position;
geographic_msgs::GeoPoseStamped raw_global_position;
bool global_position_received = false;
GeographicLib::Geoid _egm96("egm96-5");  // WARNING: not thread safe

double calc_geoid_height(double lat, double lon) {
    return _egm96(lat, lon);
}
double amsl_to_ellipsoid_height(double lat, double lon, double amsl) {
  return amsl + GeographicLib::Geoid::GEOIDTOELLIPSOID * calc_geoid_height(lat, lon);
}
double ellipsoid_height_to_amsl(double lat, double lon, double ellipsoid_height) {
  return ellipsoid_height + GeographicLib::Geoid::ELLIPSOIDTOGEOID * calc_geoid_height(lat, lon);
}

// callback functions
void globalPosition_cb(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    global_position.pose.position.altitude = ellipsoid_height_to_amsl(msg->latitude, msg->longitude, msg->altitude);
    //global_position.pose.position.altitude = msg->altitude;
    global_position.header.stamp = msg->header.stamp;
    global_position.pose.position.latitude = msg->latitude;
    global_position.pose.position.longitude = msg->longitude;
    ROS_INFO_ONCE("Received GNSS signal");
    global_position_received = true;
}

void raw_globalPosition_cb(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    raw_global_position.pose.position.altitude = ellipsoid_height_to_amsl(msg->latitude, msg->longitude, msg->altitude);
    //raw_global_position.pose.position.altitude = msg->altitude;
    raw_global_position.header.stamp = msg->header.stamp;
    raw_global_position.pose.position.latitude = msg->latitude;
    raw_global_position.pose.position.longitude = msg->longitude;
    ROS_INFO_ONCE("Received GNSS signal");
    global_position_received = true;
}

geographic_msgs::GeoPoseStamped goal_position;
void goal_position_cb(const geographic_msgs::GeoPoseStamped::ConstPtr& pose_){
    goal_position = *pose_;
}

void rangeCallback(const sensor_msgs::Range::ConstPtr& msg)
{
    range_value = msg->range;
}

void gps_callback(const mavros_msgs::GPSRAW::ConstPtr& msg) {
  fix_status = msg->fix_type;
//   switch (msg->fix_type) {
//     case 0:
//       ROS_INFO("GPS Fix type: No fix");
//       break;
//     case 1:
//       ROS_INFO("GPS Fix type: Estimation");
//       break;
//     case 2:
//       ROS_INFO("GPS Fix type: 2D Fix");
//       break;
//     case 3:
//       ROS_INFO("GPS Fix type: 3D Fix");
//       break;
//     case 4:
//       ROS_INFO("GPS Fix type: 3D Differential");
//       break;
//     case 5:
//       ROS_INFO("GPS Fix type: RTK Fixed");
//       break;
//     case 6:
//       ROS_INFO("GPS Fix type: RTK Float");
//       break;
//     default:
//       ROS_INFO("GPS Fix type: Unknown");
//       break;
//   }
}

#else
geometry_msgs::PoseStamped pose;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& pose_){
    pose = *pose_;
}
#endif



// main function
int main(int argc, char **argv) {
#ifdef GLOBAL_MODE
  puts("Global mode!");
#else
  puts("Local mode!");
#endif
    ros::init(argc, argv, "take_off");
    ros::NodeHandle nh;

    ros::ServiceClient arming_client = nh.serviceClient < mavros_msgs::CommandBool > ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient < mavros_msgs::SetMode > ("mavros/set_mode");
    ros::Subscriber state_sub = nh.subscribe < mavros_msgs::State > ("mavros/state", 10, state_cb);
#ifdef GLOBAL_MODE
    ros::Subscriber global_pos_sub = nh.subscribe < sensor_msgs::NavSatFix > ("mavros/global_position/global", 1, globalPosition_cb);
    ros::Publisher goal_pos_pub = nh.advertise < geographic_msgs::GeoPoseStamped > ("mavros/setpoint_position/global", 10);
    ros::Subscriber goal_pos_sub = nh.subscribe<geographic_msgs::GeoPoseStamped> ("mavros/setpoint_position/global", 10, goal_position_cb);
    ros::Publisher vec_pub = nh.advertise<geometry_msgs::Twist> ("mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    ros::Subscriber gps_sub = nh.subscribe<mavros_msgs::GPSRAW>("/mavros/gpsstatus/gps1/raw", 10, gps_callback);
    ros::Subscriber sub = nh.subscribe<sensor_msgs::Range>("/mavros/distance_sensor/hrlv_ez4_pub", 10, rangeCallback);
    ros::Subscriber raw_global_pos_sub = nh.subscribe("mavros/global_position/raw/fix", 10, raw_globalPosition_cb);

#else    
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10, pose_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
#endif

    ros::Rate rate(10);
    ros::Rate pub_rate(1);

    // wait for fcu connection
    while (ros::ok() && !current_state.connected) {
        ROS_INFO_ONCE("Waiting for FCU connection...");
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("FCU connected");

    while(global_position_received == false){
        ROS_INFO_ONCE("Waiting for gps...");
        ros::spinOnce();
        rate.sleep();
    }

#ifdef GLOBAL_MODE
    // set target position
    goal_position.header.stamp = ros::Time::now();
    goal_position.pose.position.latitude = global_position.pose.position.latitude;
    goal_position.pose.position.longitude = global_position.pose.position.longitude;
    goal_position.pose.position.altitude = global_position.pose.position.altitude;

    geometry_msgs::Twist vector;
    vector.linear.x = 1.0;
    vector.linear.y = 1.0;
    vector.linear.z = 1.0;

    // send a few setpoints before starting
    for (int i=0; i<20; ++i) {
        goal_position.header.stamp = ros::Time::now();
        goal_position.pose.position.latitude = global_position.pose.position.latitude;
        goal_position.pose.position.longitude = global_position.pose.position.longitude;
        goal_position.pose.position.altitude = global_position.pose.position.altitude + 2.0;
        goal_pos_pub.publish(goal_position);
        ros::spinOnce();
        rate.sleep();
    }
#else
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;
    // pose.pose.orientation.x = 0.7043544;
    // pose.pose.orientation.y = 0.7098483;
    // pose.pose.orientation.z = 4.3129272;
    // pose.pose.orientation.w = 4.3465678;

    // send a few setpoints before starting
    for (int i=0; i<20; ++i) {
        pose.header.stamp = ros::Time::now();
        pose.pose.position.z = 2.0;
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
#endif

/*
    // set mode
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.base_mode = 0;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
        ROS_INFO("OFFBOARD enabled");
    } else {
        ROS_ERROR("Failed to set OFFBOARD");
    }

    // arm
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
        ROS_INFO("Vehicle armed");
    } else {
        ROS_ERROR("Arming failed");
    }
*/
    FILE *fp;
    char filename[100];
    int i = 1;

    sprintf(filename, "Log_%d.txt", i);
    fp = fopen(filename, "r");

    while (fp != NULL) {
        fclose(fp);
        i++;
        sprintf(filename, "Log_%d.txt", i);
        fp = fopen(filename, "r");
    }

    fp = fopen(filename, "w");

    FILE *fp_2;
    char filename_2[100];
    int j = 1;

    sprintf(filename_2, "Log2_%d.txt", i);
    fp_2 = fopen(filename_2, "r");

    while (fp_2 != NULL) {
        fclose(fp_2);
        j++;
        sprintf(filename_2, "Log2_%d.txt", i);
        fp_2 = fopen(filename_2, "r");
    }

    fp_2 = fopen(filename_2, "w");
    // fprintf(f, "Hello1\tHello2\nHello3");
    // fclose(f);
    ROS_INFO_ONCE("Writing Log_%d.txt", i); 

#ifdef GLOBAL_MODE

    home[0] = global_position.pose.position.altitude;
    home[1] = global_position.pose.position.latitude;
    home[2] = global_position.pose.position.longitude;
    ROS_INFO_ONCE("Got home global position: [%.7f, %.7f, %.2f]", global_position.pose.position.latitude, global_position.pose.position.longitude, global_position.pose.position.altitude);

    // take off to 2m above ground
    goal_position.pose.position.latitude = global_position.pose.position.latitude;
    goal_position.pose.position.longitude = global_position.pose.position.longitude;
    goal_position.pose.position.altitude = global_position.pose.position.altitude + 2.0;
    goal_position.header.stamp = ros::Time::now();
    goal_pos_pub.publish(goal_position);
    while (ros::ok()) {
        ros::spinOnce();

        if(raw_gps_data_flag == false)
        {
            // Put gps data in library;
            lib[data_cnt][0] = global_position.pose.position.altitude;
            lib[data_cnt][1] = global_position.pose.position.latitude;
            lib[data_cnt][2] = global_position.pose.position.longitude;
            lib[data_cnt][3] = get_distance(goal_position.pose.position.latitude, goal_position.pose.position.longitude, 
                global_position.pose.position.latitude, global_position.pose.position.longitude);

            if(data_cnt >= 10){
                tmp_sd = calculateSD()*100;
                tmp_rmse = calculateRMSE()*100;
                ROS_INFO("\n data_cnt = %d\n fix_type = %d\n Distance to goal position: %.6f cm\n  Distance sensor value: %.2f cm\n Standard deviation in last 10s: %.6f cm\n RMSE in last 10s: %.6f cm\n Current latitude:%.8f\tGoal latitude:%.8f\tHome latitude:%.8f\n Current longitude:%.8f\tGoal longitude %.8f\tHome longitude:%.8f\n Current altitude:%.2f\t\tGoal altitude %.2f\t\tHome altitude:%.2f\n",
                data_cnt, fix_status, lib[data_cnt][3]*100, range_value*100, tmp_sd,tmp_rmse ,global_position.pose.position.latitude,goal_position.pose.position.latitude,home[1], 
                global_position.pose.position.longitude, goal_position.pose.position.longitude, home[2], global_position.pose.position.altitude, goal_position.pose.position.altitude, home[0]);
                
                // if(data_cnt % 10 == 0)
                fprintf(fp, "%d\t%d\t%.7f\t%.7f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.7f\t%.7f\t%.2f\n",data_cnt,fix_status,lib[data_cnt][1],lib[data_cnt][2],lib[data_cnt][0],lib[data_cnt][3],range_value,tmp_sd,tmp_rmse,goal_position.pose.position.latitude,goal_position.pose.position.longitude,goal_position.pose.position.altitude);
                if(data_cnt % 10 == 0)
                    fprintf(fp_2, "%d\t%d\t%.7f\t%.7f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.7f\t%.7f\t%.2f\n",data_cnt,fix_status,lib[data_cnt][1],lib[data_cnt][2],lib[data_cnt][0],lib[data_cnt][3],range_value,tmp_sd,tmp_rmse,goal_position.pose.position.latitude,goal_position.pose.position.longitude,goal_position.pose.position.altitude);
            }
        }
        else
        {
            // Put gps data in library;
            lib[data_cnt][0] = raw_global_position.pose.position.altitude;
            lib[data_cnt][1] = raw_global_position.pose.position.latitude;
            lib[data_cnt][2] = raw_global_position.pose.position.longitude;
            lib[data_cnt][3] = get_distance(goal_position.pose.position.latitude, goal_position.pose.position.longitude, 
                raw_global_position.pose.position.latitude, raw_global_position.pose.position.longitude);

            if(data_cnt >= 10){
                tmp_sd = calculateSD()*100;
                tmp_rmse = calculateRMSE()*100;
                ROS_INFO("\n data_cnt = %d\n fix_type = %d\n Distance to goal position: %.6f cm\n  Distance sensor value: %.2f cm\n Standard deviation in last 10s: %.6f cm\n RMSE in last 10s: %.6f cm\n Current latitude:%.8f\tGoal latitude:%.8f\tHome latitude:%.8f\n Current longitude:%.8f\tGoal longitude %.8f\tHome longitude:%.8f\n Current altitude:%.2f\t\tGoal altitude %.2f\t\tHome altitude:%.2f\n",
                data_cnt, fix_status, lib[data_cnt][3]*100, range_value*100, tmp_sd,tmp_rmse ,raw_global_position.pose.position.latitude,goal_position.pose.position.latitude,home[1], 
                raw_global_position.pose.position.longitude, goal_position.pose.position.longitude, home[2], raw_global_position.pose.position.altitude, goal_position.pose.position.altitude, home[0]);
                
                // if(data_cnt % 10 == 0)
                fprintf(fp, "%d\t%d\t%.7f\t%.7f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.7f\t%.7f\t%.2f\n",data_cnt,fix_status,lib[data_cnt][1],lib[data_cnt][2],lib[data_cnt][0],lib[data_cnt][3],range_value,tmp_sd,tmp_rmse,goal_position.pose.position.latitude,goal_position.pose.position.longitude,goal_position.pose.position.altitude);
                if(data_cnt % 10 == 0)
                    fprintf(fp_2, "%d\t%d\t%.7f\t%.7f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.7f\t%.7f\t%.2f\n",data_cnt,fix_status,lib[data_cnt][1],lib[data_cnt][2],lib[data_cnt][0],lib[data_cnt][3],range_value,tmp_sd,tmp_rmse,goal_position.pose.position.latitude,goal_position.pose.position.longitude,goal_position.pose.position.altitude);
            }
        }
        
        
        

        if(data_cnt <= 4998) data_cnt++;
        else data_cnt = 0;

        // Publish goal position
        goal_position.header.stamp = ros::Time::now();
        goal_pos_pub.publish(goal_position);
        //ROS_INFO("At altitude %.2f. Goal altitude %.2f", global_position.pose.position.altitude, goal_position.pose.position.altitude);
        rate.sleep();
    }

#else

    // take off to 5m above ground
    pose.pose.position.z = 2.0;
    pose.header.stamp = ros::Time::now();
    local_pos_pub.publish(pose);
    while (ros::ok()) {
        ros::spinOnce();
        pose.header.stamp = ros::Time::now();
        local_pos_pub.publish(pose);
        //ROS_INFO_THROTTLE(1, "At altitude %.2f. Goal altitude %.2f", global_position.pose.position.altitude, goal_position.pose.position.altitude);
        rate.sleep();
    }
#endif

    return 0;
}

