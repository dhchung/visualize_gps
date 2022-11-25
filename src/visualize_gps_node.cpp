#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <string.h>
#include <fstream>
#include "utm.h"

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

cv::Mat *H;
cv::Mat map_img;


void OnSubscribeGPSMsgs(const sensor_msgs::NavSatFixConstPtr & msg) {
    std::cout<<"Subscribed GPS: latitude: "<<msg->latitude<<", longitude: "<<msg->longitude<<std::endl;

    double latitude = msg->latitude * M_PI/180.0;
    double longitude = msg->longitude * M_PI/180.0;

    std::cout<<"Cov: "<<msg->position_covariance[0]<<std::endl;

    long zone;
    char hemisphere;
    double easting;
    double northing;
    Convert_Geodetic_To_UTM(latitude, longitude, &zone, &hemisphere, &easting, &northing);

    std::vector<cv::Point2d> utm_xy, pixel_xy;
    utm_xy.push_back(cv::Point2d(easting, northing));

    cv::perspectiveTransform(utm_xy, pixel_xy, *H);
    cv::Mat img_show;
    map_img.copyTo(img_show);
    cv::circle(img_show, pixel_xy[0], 3, cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_AA);
    cv::circle(img_show, pixel_xy[0], msg->position_covariance[0], cv::Scalar(0, 0, 0), 2);


    std::cout<<pixel_xy[0].x<<", "<<pixel_xy[0].y<<std::endl;

    cv::imshow("Test", img_show);
    cv::waitKey(1);


}


int main(int argc, char ** argv) {
    ros::init(argc, argv, "visualize_gps_node");
    ros::NodeHandle nh;
    ros::Subscriber subGPS = nh.subscribe<sensor_msgs::NavSatFix>("/ublox/fix", 1, OnSubscribeGPSMsgs);

    H = new cv::Mat(3, 3, CV_64F);
    map_img = cv::imread("satellite_img/img.png");

    std::string homography_path = "homography.txt";

    std::ifstream file(homography_path);
    std::string str;

    while(std::getline(file, str)) {
        std::string phrase;
        std::stringstream ss(str);
        std::vector<std::string> row;
        while(std::getline(ss, phrase, '\t')) {
            row.push_back(phrase);
        }

        for(int i = 0; i < 3; ++i) {
            for(int j = 0; j < 3; ++j) {
                H->at<double>(i, j) = std::stod(row[j + i*3]);
            }
        }
    }
    
    ros::spin();
    return 0;
}