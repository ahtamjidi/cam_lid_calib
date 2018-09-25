#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <pcl/io/pcd_io.h>
#include <string>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <image_transport/image_transport.h>
#include <boost/foreach.hpp>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
// #include <cv_bridge/CvBridge.h>
#include <image_geometry/pinhole_camera_model.h>

class CloudImageOverlay
{
  public:
    void
    cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud)
    {
        if ((cloud->width * cloud->height) == 0)
            return; //return if the cloud is not dense!
        try
        {
            pcl::toROSMsg(*cloud, image_); //convert the cloud
        }
        catch (std::runtime_error e)
        {
            ROS_ERROR_STREAM("Error in converting cloud to image message: "
                             << e.what());
        }
        image_pub_.publish(image_); //publish our cloud image

        // Decompose the projection matrix into:
        cv::Mat K(3, 3, cv::DataType<double>::type);    // intrinsic parameter matrix
        cv::Mat rvec(3, 3, cv::DataType<double>::type); // rotation matrix

        cv::Mat Thomogeneous(4, 1, cv::DataType<double>::type); // translation vector
        // Create the known projection matrix
        cv::Mat P(3,4,cv::DataType<double>::type);
        cv::decomposeProjectionMatrix(P, K, rvec, Thomogeneous);

        cv::Mat T(3, 1, cv::DataType<double>::type); // translation vector
        //cv::Mat T;
        cv::convertPointsHomogeneous(Thomogeneous, T);

        std::cout << "K: " << K << std::endl;
        std::cout << "rvec: " << rvec << std::endl;
        std::cout << "T: " << T << std::endl;

        // Create zero distortion
        cv::Mat distCoeffs(4, 1, cv::DataType<double>::type);
        distCoeffs.at<double>(0) = 0;
        distCoeffs.at<double>(1) = 0;
        distCoeffs.at<double>(2) = 0;
        distCoeffs.at<double>(3) = 0;

        std::vector<cv::Point2f> projectedPoints;

        cv::Mat rvecR(3, 1, cv::DataType<double>::type); //rodrigues rotation matrix
        cv::Rodrigues(rvec, rvecR);
        std::vector<cv::Point3d> objectPoints;
        cv::projectPoints(objectPoints, rvecR, T, K, distCoeffs, projectedPoints);

        // for (unsigned int i = 0; i < projectedPoints.size(); ++i)
        // {
        //     std::cout << "Image point: " << imagePoints[i] << " Projected to " << projectedPoints[i] << std::endl;
        // }
    }

    void
    image_cb(const sensor_msgs::PointCloud2ConstPtr &cloud)
    {
        if ((cloud->width * cloud->height) == 0)
            return; //return if the cloud is not dense!
        try
        {
            pcl::toROSMsg(*cloud, image_); //convert the cloud
        }
        catch (std::runtime_error e)
        {
            ROS_ERROR_STREAM("Error in converting cloud to image message: "
                             << e.what());
        }
        image_pub_.publish(image_); //publish our cloud image
    }

    void
    cam_info_cb(const sensor_msgs::CameraInfoConstPtr &cam_info)
    {
        cam_model_.fromCameraInfo(cam_info);
    }

    CloudImageOverlay() : cloud_topic_("input"),
                          image_in_topic_("image_in"),
                          image_out_topic_("image_out"),
                          camera_info_topic_("camera_info")
    //   it(nh_)
    {
        sub_cloud = nh_.subscribe(cloud_topic_, 30, &CloudImageOverlay::cloud_cb, this);
        sub_cam_info = nh_.subscribe(image_in_topic_, 30, &CloudImageOverlay::image_cb, this);
        sub_cam_image = nh_.subscribe(camera_info_topic_, 30, &CloudImageOverlay::cam_info_cb, this);
        // sub = it.subscribe("camera/image_raw", 1, &CloudImageOverlay::cam_info_cb);

        image_pub_ = nh_.advertise<sensor_msgs::Image>(image_out_topic_, 30);

        //print some info about the node
        std::string r_ct = nh_.resolveName(cloud_topic_);
        std::string r_it = nh_.resolveName(image_in_topic_);
        std::string r_cit = nh_.resolveName(camera_info_topic_);

        ROS_INFO_STREAM("Listening for incoming data on topic " << r_ct);
        ROS_INFO_STREAM("Listening for incoming data on topic " << r_it);
        ROS_INFO_STREAM("Listening for incoming data on topic " << r_cit);

        ROS_INFO_STREAM("Publishing image on topic " << r_it);
    }

  private:
    ros::NodeHandle nh_;
    sensor_msgs::Image image_; //cache the image message
    std::string cloud_topic_;
    std::string image_in_topic_;
    std::string image_out_topic_;
    std::string camera_info_topic_;
    // image_transport::ImageTransport it;
    image_geometry::PinholeCameraModel cam_model_;
    // image_transport::Subscriber sub;

    ros::Subscriber sub_cloud;     //cloud subscriber
    ros::Subscriber sub_cam_info;  //cam info subscriber
    ros::Subscriber sub_cam_image; //cam image subscriber
    ros::Publisher image_pub_;     //image message publisher
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "convert_pointcloud_to_image");
    CloudImageOverlay pci; //this loads up the node
    ros::spin();           //where she stops nobody knows
    return 0;
}
