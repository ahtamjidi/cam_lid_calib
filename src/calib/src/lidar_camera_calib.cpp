// NOTE: this code is inspired by example in
// https://www.learnopencv.com/head-pose-estimation-using-opencv-and-dlib/
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;
 
int main(int argc, char **argv)
{

    // Read input image
    cv::Mat im = cv::imread("/home/amirhossein/Desktop/Mywork/Research_TAMU/autonomous_cars/lidar_camera_calibration/my_sol/data/laser_camera_ass/frame0034.jpg");
     
    // 2D image points. If you change the image, you need to change vector
    std::vector<cv::Point2d> image_points;

    // frame0034.jpg
    image_points.push_back( cv::Point2d(597, 323) );    
    image_points.push_back( cv::Point2d(809, 345) );    
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

    // 3D model points.
    std::vector<cv::Point3d> model_points;

    // frame0034.jpg
    model_points.push_back(cv::Point3d(1.4020, -0.597752, -0.133348)); // uv = [597, 323]
    model_points.push_back(cv::Point3d(4.9928, -1.7213, -1.0266));  // uv = [809, 345]
    model_points.push_back(cv::Point3d(1.3947, -1.1233, -0.47985)); // uv = [788, 472]
    model_points.push_back(cv::Point3d(1.492, -0.58211, -0.42912));  // uv = [577, 454]

    // frame0000.jpg
    model_points.push_back(cv::Point3d(1.2935, -0.16672, 0.068351)); // uv = [272, 284]
    model_points.push_back(cv::Point3d(1.3064, -0.37849, 0.071282));  // uv = [515, 292]
    model_points.push_back(cv::Point3d(1.3755, 0.17589, -0.21963)); // uv = [263, 432]

    // frame0002.jpg
    model_points.push_back(cv::Point3d(1.3547, 0.14028, 0.36493)); // uv = [290, 59]
    model_points.push_back(cv::Point3d(1.49293, 0.15089, 0.17647));  // uv = [278, 194]
    model_points.push_back(cv::Point3d(1.2479, -0.39149, 0.35044));  // uv = [513, 50]    
     
    // Camera internals
    double focal_length = im.cols; // Approximate focal length.
    Point2d center = cv::Point2d(im.cols/2,im.rows/2);

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
    cv::solvePnP(model_points, image_points, camera_matrix, dist_coeffs, rotation_vector, translation_vector);
 
    vector<Point2d> projected_lidar_point2D;
     
    projectPoints(model_points, rotation_vector, translation_vector, camera_matrix, dist_coeffs, projected_lidar_point2D);
     
     
    for(int i=0; i < 4; i++)
    {
        circle(im, image_points[i], 3, Scalar(0,0,255), -1);
        circle(im, projected_lidar_point2D[i], 3, Scalar(0,255,255), -1);
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
    cv::imshow("Output", im);
    cv::waitKey(0);
 
}