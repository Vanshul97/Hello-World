#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "nsra_odrive_interface/coords.h"
#include <ros/package.h>

using namespace std;
using namespace cv;

cv::Mat points4d;
Mat cam_left_pnts(1,1,CV_64FC2);
Mat cam_right_pnts(1,1,CV_64FC2);
Mat PL, PR;

std::vector<Point3d> coordBuffer;

ros::ServiceClient cameras;

ros::Publisher pub;
ros::Publisher pub1;

cv::FileStorage fs1(ros::package::getPath("nsra_computer_vision") + "/" + "storedPoints.yml", cv::FileStorage::WRITE);

void saveCallback(const std_msgs::StringConstPtr& str)
{
    cv::Mat mat(coordBuffer, false);
    fs1 << "Points" << mat;
}

void calcCallback(const std_msgs::StringConstPtr& str)
{
    nsra_odrive_interface::coords camera_coords;
    camera_coords.request.test = 1;
    cameras.call(camera_coords);
    cam_right_pnts.at<double>(0,0) = camera_coords.response.x_right;
    cam_right_pnts.at<double>(0,1) = camera_coords.response.y_right;
    cam_left_pnts.at<double>(0,0) = camera_coords.response.x_left;
    cam_left_pnts.at<double>(0,1) = camera_coords.response.y_left;

    triangulatePoints(PL,PR,cam_left_pnts,cam_right_pnts,points4d);
    /*
    cv::Mat1f Thomogeneous(4, 1); 
    Thomogeneous(0) = pnts3D.at<double>(0,0);
    Thomogeneous(1) = pnts3D.at<double>(0,1);
    Thomogeneous(2) = pnts3D.at<double>(0,2);
    Thomogeneous(3) = pnts3D.at<double>(0,3);

    Mat Th = Thomogeneous.reshape(4);

    cv::Mat T;
    cv::convertPointsFromHomogeneous(Th, T);
    */

    std::vector<Point3d> results;

    Point3d point = Point3d(points4d.at<double>(0, 0) / points4d.at<double>(3, 0),
                            points4d.at<double>(1, 0) / points4d.at<double>(3, 0),
                            points4d.at<double>(2, 0) / points4d.at<double>(3, 0));
    results.emplace_back(point);
    std_msgs::String msg;
    msg.data = to_string(results[0].x) + "/" + to_string(results[0].y) + "/" + to_string(results[0].z);
    pub.publish(msg);

    coordBuffer.push_back(results[0]);

    cout << results << endl;
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "bottle_detection");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("calc3Dcoords", 1, calcCallback);
    ros::Subscriber sub1 = n.subscribe("saveCoords", 1, saveCallback);
    cameras = n.serviceClient<nsra_odrive_interface::coords>("get2dcoords");
    pub = n.advertise<std_msgs::String>("PointCoords", 5);

    FileStorage fs(ros::package::getPath("nsra_computer_vision") + "/" + "cam_stereo.yml", FileStorage::READ);

    fs["PL"] >> PL;
    fs["PR"] >> PR;
    
    cout << PL << endl;
    cout << PR << endl;

    ros::spin();

}