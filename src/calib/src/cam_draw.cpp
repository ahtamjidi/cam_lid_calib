#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <string>
// #include <yaml-cpp/yaml.h>

static const std::string OPENCV_WINDOW = "Image window";

// class MyData
// {
// public:
//     MyData() : A(0), X(0), id()
//     {}
//     explicit MyData(int) : A(97), X(CV_PI), id("mydata1234") // explicit to avoid implicit conversion
//     {}
//     void write(cv::FileStorage& fs) const                        //Write serialization for this class
//     {
//         fs << "{" << "A" << A << "X" << X << "id" << id << "}";
//     }
//     void read(const cv::FileNode& node)                          //Read serialization for this class
//     {
//         A = (int)node["A"];
//         X = (double)node["X"];
//         id = (std::string)node["id"];
//     }
// public:   // Data Members
//     int A;
//     double X;
//     std::string id;
// };

// static void read(const cv::FileNode& node, MyData& x, const MyData& default_value = MyData()){
//     if(node.empty())
//         x = default_value;
//     else
//         x.read(node);
// }


class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("sensors/camera/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Size patternsize(5, 7); //interior number of corners
    cv::Mat gray;
    cv::cvtColor(cv_ptr->image,gray,cv::COLOR_BGR2GRAY);//source image
    cv::vector<cv::Point2f> corners; //this will be filled by the detected corners

    //CALIB_CB_FAST_CHECK saves a lot of time on images
    //that do not contain any chessboard corners
    bool patternfound = cv::findChessboardCorners(gray, patternsize, corners,
                        cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE
                        + cv::CALIB_CB_FAST_CHECK);

    if(patternfound)
    {
        cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
        ROS_WARN("Good");
    }
    else
    {
        // ROS_WARN("something wrong");
    }

// /////////////////////////////////////////////////////
//     {//read
//         std::cout << std::endl << "Reading: " << std::endl;
//         cv::FileStorage fs;
//         std::string filename("asdfaf");
//         fs.open(filename, cv::FileStorage::READ);
//         int itNr;
//         //fs["iterationNr"] >> itNr;
//         itNr = (int) fs["iterationNr"];
//         std::cout << itNr;
//         if (!fs.isOpened())
//         {
//             std::cerr << "Failed to open " << filename << std::endl;
//             // help(av);
//             // return 1;
//         }
//         cv::FileNode n = fs["strings"];                         // Read string sequence - Get node
//         if (n.type() != cv::FileNode::SEQ)
//         {
//             std::cerr << "strings is not a sequence! FAIL" << std::endl;
//             // return 1;
//         }
//         cv::FileNodeIterator it = n.begin(), it_end = n.end(); // Go through the node
//         for (; it != it_end; ++it)
//             std::cout << (std::string)*it << std::endl;
//         n = fs["Mapping"];                                // Read mappings from a sequence
//         std::cout << "Two  " << (int)(n["Two"]) << "; ";
//         std::cout << "One  " << (int)(n["One"]) << std::endl << std::endl;
//         MyData m;
//         cv::Mat R, T;
//         fs["R"] >> R;                                      // Read cv::Mat
//         fs["T"] >> T;
//         fs["MyData"] >> m;                                 // Read your own structure_
//         std::cout << std::endl
//             << "R = " << R << std::endl;
//         std::cout << "T = " << T << std::endl << std::endl;
//         // std::cout << "MyData = " << endl << m << std::endl << std::endl;
//         //Show default behavior for non existing nodes
//         std::cout << "Attempt to read NonExisting (should initialize the data structure with its default).";
//         fs["NonExisting"] >> m;
//         // std::cout << std::endl << "NonExisting = " << std::endl << m << std::endl;
//     }
// //////////////////////////////////////////////










    cv::drawChessboardCorners(cv_ptr->image, patternsize, cv::Mat(corners), patternfound);

    // // Draw an example circle on the video stream
    // if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    //   cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));


    cv::vector<cv::Mat> rvecs, tvecs;

    // YAML::Node lconf = YAML::LoadFile("/home/amirhossein/Desktop/Mywork/Research_TAMU/autonomous_cars/lidar_camera_calibration/camcal/ost.yaml");

    // cv::saveCameraParams(s, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, reprojErrs, imagePoints, totalAvgErr);    

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(1);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};



int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
