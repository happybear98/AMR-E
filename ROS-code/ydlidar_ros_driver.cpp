#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
//#include "ydlidar_ros_driver/LaserFan.h"
#include "std_srvs/Empty.h"
#include "src/CYdLidar.h"
#include "ydlidar_config.h"
#include <limits>       // std::numeric_limits
#include <iostream>
#include <array>
#include <cmath>

#include "std_msgs/String.h"
#include <sstream>

using namespace std;

#define SDKROSVerision "1.0.2"
#define PI 3.14159256358979323846
#define V_max 70.683  //선속도, 확인필요

float goal_x = 0.0;
float goal_y = 0.0;

CYdLidar laser;

bool stop_scan(std_srvs::Empty::Request& req,
    std_srvs::Empty::Response& res) {
    ROS_DEBUG("Stop scan");
    return laser.turnOff();
}

bool start_scan(std_srvs::Empty::Request& req,
    std_srvs::Empty::Response& res) {
    ROS_DEBUG("Start scan");
    return laser.turnOn();
}

int mypartition (float arr[], int low, int high) {
    float pivot = arr[high];    // pivot
    int i = (low - 1);  // Index of smaller element

    for (int j = low; j <= high- 1; j++) {
        // If current element is smaller than the pivot
        if (arr[j] > pivot) {
            i++;    // increment index of smaller element
            std::swap(arr[i], arr[j]);
        }
    }
    std::swap(arr[i + 1], arr[high]);
    return (i + 1);
}

void quickSort(float arr[], int low, int high) {
    if (low < high) {
        /* pi is partitioning index, arr[p] is now
        at right place */
        int pi = mypartition(arr, low, high);

        // Separately sort elements before
        // partition and after partition
        quickSort(arr, low, pi - 1);
        quickSort(arr, pi + 1, high);
    }
}
// This function takes last element as pivot, places
// the pivot element at its correct position in sorted
// array, and places all smaller (smaller than pivot)
// to left of pivot and all greater elements to right
// of pivot

void goalCallback(const std_msgs::String::ConstPtr& msg){
    std::string goalPointString = msg->data;
    std::istringstream iss(goalPointString);
    float pre_goal_x = 0.0;
    float pre_goal_y = 0.0;
    if(!(iss >> pre_goal_x >> pre_goal_y)) {
        ROS_ERROR("Received invalid goal point: %s", goalPointString.c_str());
        return;
    }

    if( (pre_goal_x != goal_x) && (pre_goal_y != goal_y)) {
        goal_x = pre_goal_x;
        goal_y = pre_goal_y;
    } else { goal_x = goal_y = 0.0; }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ydlidar_ros_driver");
    ROS_INFO("YDLIDAR ROS Driver Version: %s", SDKROSVerision);
    ros::NodeHandle nh;
    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1);
    ros::Publisher pc_pub = nh.advertise<sensor_msgs::PointCloud>("point_cloud",1);
    //  ros::Publisher laser_fan_pub =
    //    nh.advertise<ydlidar_ros_driver::LaserFan>("laser_fan", 1);

    ros::NodeHandle nh_private("~");
    std::string str_optvalue = "/dev/ydlidar";
    nh_private.param<std::string>("port", str_optvalue, "/dev/ydlidar");
    ///lidar port
    laser.setlidaropt(LidarPropSerialPort, str_optvalue.c_str(),
        str_optvalue.size());

    ///ignore array
    nh_private.param<std::string>("ignore_array", str_optvalue, "");
    laser.setlidaropt(LidarPropIgnoreArray, str_optvalue.c_str(),
        str_optvalue.size());

    std::string frame_id = "laser_frame";
    nh_private.param<std::string>("frame_id", frame_id, "laser_frame");

    //////////////////////int property/////////////////
    /// lidar baudrate
    int optval = 230400;
    nh_private.param<int>("baudrate", optval, 230400);
    laser.setlidaropt(LidarPropSerialBaudrate, &optval, sizeof(int));
    /// tof lidar
    optval = TYPE_TRIANGLE;
    nh_private.param<int>("lidar_type", optval, TYPE_TRIANGLE);
    laser.setlidaropt(LidarPropLidarType, &optval, sizeof(int));
    /// device type
    optval = YDLIDAR_TYPE_SERIAL;
    nh_private.param<int>("device_type", optval, YDLIDAR_TYPE_SERIAL);
    laser.setlidaropt(LidarPropDeviceType, &optval, sizeof(int));
    /// sample rate
    optval = 9;
    nh_private.param<int>("sample_rate", optval, 9);
    laser.setlidaropt(LidarPropSampleRate, &optval, sizeof(int));
    /// abnormal count
    optval = 4;
    nh_private.param<int>("abnormal_check_count", optval, 4);
    laser.setlidaropt(LidarPropAbnormalCheckCount, &optval, sizeof(int));
    //intensity bit count
    optval = 10;
    nh_private.param<int>("intensity_bit", optval, 10);
    laser.setlidaropt(LidarPropIntenstiyBit, &optval, sizeof(int));

    //////////////////////bool property/////////////////
    /// fixed angle resolution
    bool b_optvalue = false;
    nh_private.param<bool>("resolution_fixed", b_optvalue, true);
    laser.setlidaropt(LidarPropFixedResolution, &b_optvalue, sizeof(bool));
    /// rotate 180
    nh_private.param<bool>("reversion", b_optvalue, true);
    laser.setlidaropt(LidarPropReversion, &b_optvalue, sizeof(bool));
    /// Counterclockwise
    nh_private.param<bool>("inverted", b_optvalue, true);
    laser.setlidaropt(LidarPropInverted, &b_optvalue, sizeof(bool));
    b_optvalue = true;
    nh_private.param<bool>("auto_reconnect", b_optvalue, true);
    laser.setlidaropt(LidarPropAutoReconnect, &b_optvalue, sizeof(bool));
    /// one-way communication
    b_optvalue = false;
    nh_private.param<bool>("isSingleChannel", b_optvalue, false);
    laser.setlidaropt(LidarPropSingleChannel, &b_optvalue, sizeof(bool));
    /// intensity
    b_optvalue = false;
    nh_private.param<bool>("intensity", b_optvalue, false);
    laser.setlidaropt(LidarPropIntenstiy, &b_optvalue, sizeof(bool));
    /// Motor DTR
    b_optvalue = false;
    nh_private.param<bool>("support_motor_dtr", b_optvalue, false);
    laser.setlidaropt(LidarPropSupportMotorDtrCtrl, &b_optvalue, sizeof(bool));

    //////////////////////float property/////////////////
    /// unit: °
    float f_optvalue = 180.0f;
    nh_private.param<float>("angle_max", f_optvalue, 180.f);
    laser.setlidaropt(LidarPropMaxAngle, &f_optvalue, sizeof(float));
    f_optvalue = -180.0f;
    nh_private.param<float>("angle_min", f_optvalue, -180.f);
    laser.setlidaropt(LidarPropMinAngle, &f_optvalue, sizeof(float));
    /// unit: m
    f_optvalue = 16.f;
    nh_private.param<float>("range_max", f_optvalue, 16.f);
    laser.setlidaropt(LidarPropMaxRange, &f_optvalue, sizeof(float));
    f_optvalue = 0.1f;
    nh_private.param<float>("range_min", f_optvalue, 0.1f);
    laser.setlidaropt(LidarPropMinRange, &f_optvalue, sizeof(float));
    /// unit: Hz
    f_optvalue = 10.f;
    nh_private.param<float>("frequency", f_optvalue, 10.f);
    laser.setlidaropt(LidarPropScanFrequency, &f_optvalue, sizeof(float));

    bool invalid_range_is_inf = false;
    nh_private.param<bool>("invalid_range_is_inf", invalid_range_is_inf,
        invalid_range_is_inf);

    bool point_cloud_preservative = false;
    nh_private.param<bool>("point_cloud_preservative", point_cloud_preservative,
        point_cloud_preservative);

    ros::ServiceServer stop_scan_service = nh.advertiseService("stop_scan",
        stop_scan);
    ros::ServiceServer start_scan_service = nh.advertiseService("start_scan",
        start_scan);

    // initialize SDK and LiDAR
    bool ret = laser.initialize();

    if (ret) {//success
        //Start the device scanning routine which runs on a separate thread and enable motor.
        ret = laser.turnOn();
    }
    else {
        ROS_ERROR("%s\n", laser.DescribeError());
    }

    //x, y coord_pub
    ros::Publisher xy_pub = nh.advertise<std_msgs::String>("/XY_coord", 1);

    ros::Rate r(30);

    float S_max = 12.0;
    float S_min = 0.6;
    float FRmat[505] = {0.0};
    float FAmat[505] = {0.0};
    float AVGDD[505] = {0.0};
    float DDmat[505] = {0.0};
    float ANGmat[505] = {0.0};
    float FRvec_x[505] = {0.0};
    float FRvec_y[505] = {0.0};
    float Distance;
    float FAng = 0.0;
    float Fr = 0.0, Fa = 0.0;
    float C0 = V_max;
    const int C1 = 0;
    const int Tf = 1;
    float Ds = (V_max * Tf) / 2;
    float Ka = V_max / pow(Ds, 3);
    float C2 = 3 * (0 - V_max) / pow(S_max, 2);
    float C3 = -2 * (0 - V_max) / pow(S_max, 3);
    float angle = 0.0;
    float FAvec_x = 0.0, FAvec_y = 0.0;
    float TotVec_x = 0.0, TotVec_y = 0.0;
    float Next_Ang = 0.0, FAdeg_Ang = 0.0;
    float Pre_x = 0.0;
    float Pre_y = 0.0;
    float Next_x = 0.0, Next_y = 0.0;
    float Next_degang = 0.0, Next_dist = 0.0;
    float temp1, temp2;
    float temp3, temp4, temp5;
    float Vec_test_x = 0.0, Vec_test_y = 0.0;

            //goalPoint sub
            ros::Subscriber sub = nh.subscribe("/goal_point", 10, goalCallback);
            cout << goal_x << "  " << goal_y << "\n\n" <<endl;

    //if (Pre_x >= goal_x && Pre_y >= goal_y) { Pre_x = Pre_y = goal_x = goal_y = 0.0; }

    int Scount = 0;

    while (ret && ros::ok()) {
        LaserScan scan;

        if (laser.doProcessSimple(scan)) {
            sensor_msgs::LaserScan scan_msg;
            sensor_msgs::PointCloud pc_msg;
            //      ydlidar_ros_driver::LaserFan fan;
            ros::Time start_scan_time;
            start_scan_time.sec = scan.stamp / 1000000000ul;
            start_scan_time.nsec = scan.stamp % 1000000000ul;
            scan_msg.header.stamp = start_scan_time;
            scan_msg.header.frame_id = frame_id;
            pc_msg.header = scan_msg.header;
            //      fan.header = scan_msg.header;
            scan_msg.angle_min = (scan.config.min_angle);
            scan_msg.angle_max = (scan.config.max_angle);
            scan_msg.angle_increment = (scan.config.angle_increment);
            scan_msg.scan_time = scan.config.scan_time;
            scan_msg.time_increment = scan.config.time_increment;
            scan_msg.range_min = (scan.config.min_range);
            scan_msg.range_max = (scan.config.max_range);
            //      fan.angle_min = (scan.config.min_angle);
            //      fan.angle_max = (scan.config.max_angle);
            //      fan.scan_time = scan.config.scan_time;
            //      fan.time_increment = scan.config.time_increment;
            //      fan.range_min = (scan.config.min_range);
            //      fan.range_max = (scan.config.max_range);

            int size = (scan.config.max_angle - scan.config.min_angle) /
                scan.config.angle_increment + 1;
            scan_msg.ranges.resize(size,
                invalid_range_is_inf ? std::numeric_limits<float>::infinity() : 0.0);
            scan_msg.intensities.resize(size);
            pc_msg.channels.resize(2);
            int idx_intensity = 0;
            pc_msg.channels[idx_intensity].name = "intensities";
            int idx_timestamp = 1;
            pc_msg.channels[idx_timestamp].name = "stamps";

            for (size_t i = 0; i < scan.points.size(); i++) {
                int index = std::ceil((scan.points[i].angle - scan.config.min_angle) /
                    scan.config.angle_increment);

                if (index >= 0 && index < size) {
                    if (scan.points[i].range >= scan.config.min_range) {
                        scan_msg.ranges[index] = scan.points[i].range;
                        scan_msg.intensities[index] = scan.points[i].intensity;
                    }
                }

                if (point_cloud_preservative ||
                    (scan.points[i].range >= scan.config.min_range &&
                        scan.points[i].range <= scan.config.max_range)) {
                    geometry_msgs::Point32 point;
                    point.x = scan.points[i].range * cos(scan.points[i].angle);
                    point.y = scan.points[i].range * sin(scan.points[i].angle);
                    point.z = 0.0;
                    pc_msg.points.push_back(point);
                    pc_msg.channels[idx_intensity].values.push_back(scan.points[i].intensity);
                    pc_msg.channels[idx_timestamp].values.push_back(i * scan.config.time_increment);
                }

                //        fan.angles.push_back(scan.points[i].angle);
                //        fan.ranges.push_back(scan.points[i].range);
                //        fan.intensities.push_back(scan.points[i].intensity);
            }

        //Potential Field=========================================================
        if((goal_x != 0.0) && (goal_y != 0.0)) {

            //input data to array, generate Attractive
            for (int i = 0; i < 505; i++) {
                float distance = scan.points[i].range;
                if(i==0) { ANGmat[0] = angle = TotVec_x = TotVec_y = 0; }
                if(scan.points[i].angle*180/PI < 0) {
                    angle = (scan.points[i].angle*(-180/PI));
                    ANGmat[i] = angle;
                } else if(scan.points[i].angle*180/PI > 0) {
                    angle = (180+(180-(scan.points[i].angle*180/PI)));
                    ANGmat[i] = angle;
                } else { ANGmat[i] = 0; }

                if ((angle <= 360) && (distance < S_max) && (distance >= S_min) && (scan.points[i].intensity > 0)) {
                    Distance = (sqrt((Pre_x - goal_x) * (Pre_x - goal_x) + (Pre_y - goal_y) * (Pre_y - goal_y)));
                    if ((distance == 0) && (i == 0)) {
                        DDmat[i] = S_max;
                    } else if((distance == 0) && (i != 0)) {
                        DDmat[i] = DDmat[i-1];
                    } else {
                        DDmat[i] = distance;
                    }

                    //if(Distance < 1) { DDmat[i] = DDmat[i]/2; }
                    

                    FAng = atan2((goal_x - Pre_x), (goal_y - Pre_y));
                    if(FAng > 2*PI) {
                        FAng -= 2*PI;
                    } else if(FAng < -2*PI) {
                        FAng += 2*PI;
                    } else {
                        FAng = FAng;
                    }
                    FAdeg_Ang = FAng*(180/PI);
                    FAvec_x = Distance * sin(FAng);
                    FAvec_y = Distance * cos(FAng);
                    if (Ds > Distance) {
                        Fa = V_max - Ka * pow((Ds - Distance),3);
                    } else {
                        Fa = V_max;
                    }
                }
            }

            temp1 = temp2 = temp3 = temp4 = temp5 = 0;
            //bubble sorting
            float temp1, temp2;
            for (int i = 0; i < 505; i++) {
                for (int j = i + 1; j < 505; j++) {
                    if (ANGmat[i] > ANGmat[j]) {
                        temp1 = ANGmat[i];
                        ANGmat[i] = ANGmat[j];
                        ANGmat[j] = temp1;
                        temp2 = DDmat[i];
                        DDmat[i] = DDmat[j];
                        DDmat[j] = temp2;
                    }
                }
            }

            //average DDmat
            for (int p = 0; p < 505; p++) {
                if (p == 0) {
                    AVGDD[0] = (DDmat[504] + DDmat[0] + DDmat[1]) / 3;
                } else if (p == 504) {
                    AVGDD[504] = (DDmat[503] + DDmat[504] + DDmat[0]) / 3;
                } else {
                    AVGDD[p] = (DDmat[p - 1] + DDmat[p] + DDmat[p + 1]) / 3;
                }
            }

            //generate Repersive
            for (int i = 0; i < 505; i++) {
                Fr = C0 + (C2 * pow(AVGDD[i], 2)) + (C3 * pow(AVGDD[i], 3));
                FRmat[i] = Fr;
            }

            //bubble sorting
            for (int j = 0; j < 505; j++) {
                for (int i = i + 1; i < 505; i++) {
                    if (FRmat[i] > FRmat[j]) {
                        temp3 = FRmat[i];
                        FRmat[i] = FRmat[j];
                        FRmat[j] = temp3;
                        temp4 = AVGDD[i];
                        AVGDD[i] = AVGDD[j];
                        AVGDD[j] = temp4;
                        temp5 = ANGmat[i];
                        ANGmat[i] = ANGmat[j];
                        ANGmat[j] = temp5;
                    }
                }
            }

            //Repulsive vector
            for (int i = 0; i < 505; i++) {
                FRvec_x[i] = AVGDD[i] * sin(ANGmat[i] * PI / 180);
                FRvec_y[i] = AVGDD[i] * cos(ANGmat[i] * PI / 180);
            }
            Vec_test_x = FRvec_x[0];
            Vec_test_y = FRvec_y[0];
            /*for (int i = 0; i < 505; i++) {
                TotVec_x += FRvec_x[i];
                TotVec_y += FRvec_y[i];
            }*/

            //to move x,y angle
            Next_x = floor((Pre_x + FAvec_x + Vec_test_x)*100) / 100;
            Next_y = floor((Pre_y + FAvec_y + Vec_test_y)*100) / 100 - 2.75;

            //if((Next_x == Pre_x) && (Next_y == Pre_y)) { Next_x = 0.0; Next_y = 0.0; }

            if ((Next_x < 0) && (Next_y < 0)) {
                Next_Ang = -PI + atan(Next_x / Next_y);
            } else if ((Next_x >= 0) && (Next_y < 0)) {
                Next_Ang = PI + atan(Next_x / Next_y);
            } else {
                Next_Ang = atan(Next_x / Next_y);
            }
            Next_degang = Next_Ang * 180 / PI;
            Next_dist = sqrt(pow(Next_x, 2) + pow(Next_y, 2));

            /*for(int i = 0; i < scan.points.size(); i++) {
                if(ANGmat[i] > 180){
                    ANGmat[i] = ANGmat[i] - 360;
                }
            }*/

            cout << "---------------------------------------------------------------1" << endl;
            cout << Distance << "\t" << FAdeg_Ang << "\t" << Fr << "\t" << Fa << "\t" << angle << "\t" << Pre_x << "\t" << Pre_y << endl;
            cout << "---------------------------------------------------------------2" << endl;
            cout << FAvec_x << "\t" << FAvec_y << "\t" << TotVec_x << "\t" << TotVec_y << "\t" << Next_Ang << endl;
            cout << "---------------------------------------------------------------3" << endl;
            cout << Next_x << "\t" << Next_y << "\t" << Next_degang << "\t" << Next_dist << "\t" << Scount << "\n\n" << endl;

            std_msgs::String xy_msg;
            std::stringstream xy;
            xy << Next_x << " " << Next_y << " " << " " << Scount;
            xy_msg.data = xy.str();

            ROS_INFO("point: %s", xy_msg.data.c_str());

            xy_pub.publish(xy_msg);
            ++Scount;
        }

            //Potential Field=========================================================

            scan_pub.publish(scan_msg);
            pc_pub.publish(pc_msg);
            //      laser_fan_pub.publish(fan);

        }
        else {
            ROS_ERROR("Failed to get Lidar Data");
        }

        r.sleep();
        ros::spinOnce();
    }

    laser.turnOff();
    ROS_INFO("[YDLIDAR INFO] Now YDLIDAR is stopping .......");
    laser.disconnecting();
    return 0;
}
