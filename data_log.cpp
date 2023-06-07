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


#define PI                      3.1415926
#define EARTH_RADIUS            6378137        
#define SD_PERIOD               100

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
    return d * PI / 180.0;   // 1˚ = π / 180
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


// sensor_msgs::NavSatFix global_position;
geographic_msgs::GeoPoseStamped global_position;
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
}





// main function
int main(int argc, char **argv) {
    ros::init(argc, argv, "DataLog");
    ros::NodeHandle nh;

    ros::ServiceClient arming_client = nh.serviceClient < mavros_msgs::CommandBool > ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient < mavros_msgs::SetMode > ("mavros/set_mode");
    ros::Subscriber state_sub = nh.subscribe < mavros_msgs::State > ("mavros/state", 10, state_cb);
    ros::Subscriber global_pos_sub = nh.subscribe < sensor_msgs::NavSatFix > ("mavros/global_position/global", 1, globalPosition_cb);
    ros::Publisher goal_pos_pub = nh.advertise < geographic_msgs::GeoPoseStamped > ("mavros/setpoint_position/global", 10);
    ros::Subscriber goal_pos_sub = nh.subscribe<geographic_msgs::GeoPoseStamped> ("mavros/setpoint_position/global", 10, goal_position_cb);
    ros::Subscriber gps_sub = nh.subscribe<mavros_msgs::GPSRAW>("/mavros/gpsstatus/gps1/raw", 10, gps_callback);
    ros::Subscriber range_sub = nh.subscribe<sensor_msgs::Range>("/mavros/distance_sensor/hrlv_ez4_pub", 10, rangeCallback);

    ros::Rate rate(10);

    while (ros::ok() && !current_state.connected) {
        ROS_INFO_ONCE("Waiting for FCU connection...");
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("FCU connected");

    while(global_position_received == false){
        ROS_INFO_ONCE("Waiting for gps reception...");
        ros::spinOnce();
        rate.sleep();
    }

    // set target position
    goal_position.header.stamp = ros::Time::now();
    goal_position.pose.position.latitude = global_position.pose.position.latitude;
    goal_position.pose.position.longitude = global_position.pose.position.longitude;
    goal_position.pose.position.altitude = global_position.pose.position.altitude;

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
        
        

        if(data_cnt <= 4998) data_cnt++;
        else data_cnt = 0;

        // Publish goal position
        goal_position.header.stamp = ros::Time::now();
        goal_pos_pub.publish(goal_position);
        //ROS_INFO("At altitude %.2f. Goal altitude %.2f", global_position.pose.position.altitude, goal_position.pose.position.altitude);
        rate.sleep();
    }


    return 0;
}

