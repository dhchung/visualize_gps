#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <string.h>
#include <fstream>
#include "utm.h"

cv::Mat inputImage;
cv::Mat mouseOverImage;

std::vector<std::pair<int, int>> clicked_points;

std::vector<cv::Point2d> utm_coords;
std::vector<cv::Point2d> img_coords;



void Draw(int event, int x, int y, int flags, void* param)
{
    inputImage.copyTo(mouseOverImage);
    if (event == cv::EVENT_MOUSEMOVE) {
        cv::line(mouseOverImage, cv::Point(x, 0), cv::Point(x, mouseOverImage.rows), cv::Scalar(0, 0, 0), 1);
        cv::line(mouseOverImage, cv::Point(0, y), cv::Point(mouseOverImage.cols, y), cv::Scalar(0, 0, 0), 1);
    }    

    if (event & cv::EVENT_LBUTTONDOWN) {
        std::cout << "X " << x << " Y " << y << std::endl;
        cv::circle(inputImage, cv::Point(x, y), 2, cv::Scalar(0, 255, 0), cv::FILLED, cv::LINE_AA);
        clicked_points.push_back(std::pair<int, int>{x, y});
        img_coords.push_back(cv::Point2d(double(x), double(y)));
    }
}


int main(int argc, char ** argv) {

    std::string img_path = "satellite_img/img.png";
    std::string img_gps_coordinates_path = "satellite_img/sat_coord.txt";

    std::ifstream file(img_gps_coordinates_path);
    std::string str;

    std::vector<std::pair<double, double>> utm_coordinates;

    while(std::getline(file, str)) {
        std::string phrase;
        std::stringstream ss(str);
        std::vector<std::string> row;
        while(std::getline(ss, phrase, '\t')) {
            row.push_back(phrase);
        }
        double latitude = std::stod(row[0]) * M_PI/ 180.0;
        double longitude = std::stod(row[1]) * M_PI/ 180.0;
        long zone;
        char hemisphere;
        double easting;
        double northing;
        Convert_Geodetic_To_UTM(latitude, longitude, &zone, &hemisphere, &easting, &northing);

        double utm_x = easting;
        double utm_y = northing;

        std::cout<<"UTM: "<<utm_x<<", "<<utm_y<<std::endl;
        utm_coordinates.push_back(std::pair<double, double>{utm_x, utm_y});
        utm_coords.push_back(cv::Point2d(utm_x, utm_y));
    }


    cv::Mat outputImage;

    inputImage = cv::imread(img_path);
    inputImage.copyTo(mouseOverImage);
    inputImage.copyTo(outputImage);
    cv::namedWindow("Image");
    cv::setMouseCallback("Image", Draw, NULL);
    while(clicked_points.size() < 4) {
        
        cv::imshow("Image", mouseOverImage);
        cv::waitKey(1);
    }

    cv::Mat H = cv::findHomography(utm_coords, img_coords);
    std::cout << "H:\n" << H << std::endl;


    std::string homography_path = "homography.txt";

    FILE * homography_file = fopen(homography_path.c_str(), "w");
    //save waypoints
    char h_buf[256];
    sprintf(h_buf, "%0.9f\t%0.9f\t%0.9f\t%0.9f\t%0.9f\t%0.9f\t%0.9f\t%0.9f\t%0.9f",
                    H.at<double>(0, 0), H.at<double>(0, 1), H.at<double>(0, 2),
                    H.at<double>(1, 0), H.at<double>(1, 1), H.at<double>(1, 2),
                    H.at<double>(2, 0), H.at<double>(2, 1), H.at<double>(2, 2));
    fwrite(h_buf, 1, strlen(h_buf), homography_file);
    fclose(homography_file);


    std::vector<cv::Point2d> pixel_xy;
    cv::perspectiveTransform(utm_coords, pixel_xy, H);
    for(int i = 0; i < pixel_xy.size(); ++i) {
        cv::circle(outputImage, pixel_xy[i], 2, cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_AA);
    }

    cv::imshow("output", outputImage);
    cv::waitKey(0);


    return 0;

}