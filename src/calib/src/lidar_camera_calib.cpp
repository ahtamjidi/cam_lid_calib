// NOTE: this code is inspired by example in
// https://www.learnopencv.com/head-pose-estimation-using-opencv-and-dlib/
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;
 
int main(int argc, char **argv)
{

    // Read input image
    cv::Mat im1 = cv::imread("/home/amirhossein/Desktop/Mywork/Research_TAMU/autonomous_cars/lidar_camera_calibration/my_sol/data/laser_camera_ass/frame0034.jpg");
    cv::Mat im2 = cv::imread("/home/amirhossein/Desktop/Mywork/Research_TAMU/autonomous_cars/lidar_camera_calibration/my_sol/data/laser_camera_ass/frame0000.jpg");
    cv::Mat im3 = cv::imread("/home/amirhossein/Desktop/Mywork/Research_TAMU/autonomous_cars/lidar_camera_calibration/my_sol/data/laser_camera_ass/frame0002.jpg");
     
    // 2D image points. If you change the image, you need to change vector
    std::vector<cv::Point2d> image_points;

    // frame0034.jpg
    image_points.push_back( cv::Point2d(597, 323) );    
    image_points.push_back( cv::Point2d(788, 472) );    
    image_points.push_back( cv::Point2d(577, 454) ); 

    // frame0000.jpg
    image_points.push_back( cv::Point2d(272, 284) );    
    image_points.push_back( cv::Point2d(515, 292) );    
    image_points.push_back( cv::Point2d(263, 432) );  

    // frame0002.jpg
    image_points.push_back( cv::Point2d(290, 59) );    
    image_points.push_back( cv::Point2d(278, 194) );   
    image_points.push_back( cv::Point2d(513, 50) );    

    // // frame0004.jpg
    image_points.push_back( cv::Point2d(323, 183) );  
    image_points.push_back( cv::Point2d(648, 184) );  

    // // frame0005.jpg
    image_points.push_back( cv::Point2d(269, 113) );  

    // 3D model points.
    std::vector<cv::Point3d> model_points;

    // frame0034.jpg
    model_points.push_back(cv::Point3d( 0.597752, 0.133348, 1.4020)); // uv = [597, 323]
    model_points.push_back(cv::Point3d(1.1233, 0.47985, 1.3947)); // uv = [788, 472]
    model_points.push_back(cv::Point3d( 0.58211, 0.42912, 1.492));  // uv = [577, 454]

    // frame0000.jpg
    model_points.push_back(cv::Point3d( 0.16672, -0.068351, 1.2935)); // uv = [272, 284]
    model_points.push_back(cv::Point3d( 0.37849, -0.071282, 1.3064));  // uv = [515, 292]
    model_points.push_back(cv::Point3d( -0.17589, 0.21963, 1.3755)); // uv = [263, 432]

    // frame0002.jpg
    model_points.push_back(cv::Point3d( -0.14028, -0.36493, 1.3547)); // uv = [290, 59]
    model_points.push_back(cv::Point3d( -0.15089, -0.17647, 1.49293));  // uv = [278, 194]
    model_points.push_back(cv::Point3d( 0.39149, -0.35044, 1.2479));  // uv = [513, 50]    

    // // frame0004.jpg
    model_points.push_back(cv::Point3d( -0.67974, -0.1343, 1.0917)); //  uv = [323, 183]
    model_points.push_back(cv::Point3d( 0.51324, -0.14795, 1.0902)); // uv = [648, 184]

    // frame0005.jpg
    model_points.push_back(cv::Point3d( -0.1604, -0.30503, 1.3114)); //  uv = [269, 113]

    // distortion_coefficients:
    // rows: 1
    // cols: 5
    // data: [-0.196606, 0.064156, 0.002826, -0.000069, 0.000000]
    double data[4] = { -0.196606, 0.064156, 0.002826, -0.000069};
    cv::Mat dist_coeffs = cv::Mat(1, 4, CV_64F, data);


    // camera_matrix:
    // rows: 3
    // cols: 3
    // data: [483.761692, 0.000000, 456.184555, 0.000000, 483.550078, 365.883083, 0.000000, 0.000000, 1.000000]    
    cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << 483.761692, 0, 456.184555, 0 , 483.550078, 365.883083, 0, 0, 1);
     
    cout << "Camera Matrix " << endl << camera_matrix << endl ;
    // Output rotation and translation
    cv::Mat rotation_vector; // Rotation in axis-angle form
    cv::Mat translation_vector;
     
    // Solve for pose
    cv::solvePnP(model_points, image_points, camera_matrix, dist_coeffs, rotation_vector, translation_vector, false, ITERATIVE);
 
    vector<Point2d> projected_lidar_point2D;
     
    projectPoints(model_points, rotation_vector, translation_vector, camera_matrix, dist_coeffs, projected_lidar_point2D);
     
     
    for(int i=0; i < 3; i++)
    {
        circle(im1, image_points[i], 3, Scalar(0,0,255), -1);
        circle(im1, projected_lidar_point2D[i], 3, Scalar(0,255,255), -1);
    }

    for(int i=3; i < 6; i++)
    {
        circle(im2, image_points[i], 3, Scalar(0,0,255), -1);
        circle(im2, projected_lidar_point2D[i], 3, Scalar(0,255,255), -1);
    }

    for(int i=6; i < 9; i++)
    {
        circle(im3, image_points[i], 3, Scalar(0,0,255), -1);
        circle(im3, projected_lidar_point2D[i], 3, Scalar(0,255,255), -1);
    }
    cout << "Rotation Vector " << endl << rotation_vector << endl;
    cout << "Translation Vector" << endl << translation_vector << endl;
     
    // Camera Matrix 
    // [483.761692, 0, 456.184555;
    //   0, 483.550078, 365.883083;
    //   0, 0, 1]
    // Rotation Vector 
    // [2.554245369973336; 1.703527296239992; 2.40930112483438]
    // Translation Vector
    // [0.5502380796317662; -0.09583337938532677; -1.952046156849584]
     
    // Display image.
    cv::imshow("frame0034", im1);
    cv::imshow("frame0000", im2);
    cv::imshow("frame0002", im3);


    cv::waitKey(0);
 
}