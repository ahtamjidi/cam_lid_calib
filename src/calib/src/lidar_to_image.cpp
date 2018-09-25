// #!/usr/bin/env python

// import math
// import rospy
// import sys
// import cv2
// import cv_bridge
// from image_geometry import PinholeCameraModel
// import tf
// from sensor_msgs.msg import Image, CameraInfo, PointCloud2
// import struct
// import numpy as np

// camera ={}
// imageOverlay = {}
// cameraInfo = CameraInfo()
// cameraModel = PinholeCameraModel()
// bridge = cv_bridge.CvBridge()
// tf_ = tf.TransformListener()

// velodyneData = []

// def camera_callback(data):
// 	global cameraModel, camera

// 	cameraInfo = data
// 	print('Receiving Camera Info.. ')

// 	# Unregister the camera after cameraInfo is received
// 	# camera.unregister()

// 	# Set the camera parameters from the sensor_msgs.msg.CameraInfo message
// 	cameraModel.fromCameraInfo( cameraInfo )

// def velodyne_callback(data):
// 	global velodyneData
// 	print('In velodyne callback - received point cloud')

// 	# print("data type :", type( data.data ))
// 	# print( "data len :",len( data.data ))

// 	formatString = 'ffff'
// 	if data.is_bigendian:
// 		formatString = '>' + formatString
// 	else:
// 		formatString = '<' + formatString

// 	points = []

// 	for index in range( 0, len( data.data ), 16 ):
// 		points.append( struct.unpack( formatString, data.data[ index:index + 16 ] ) )

// 	# print(len( points ))
// 	velodyneData = points

// def image_callback(data):
// 	global velodyneData, bridge, tf_
// 	print('In input image callback - received rectified image')

// 	cv_image = {}

// 	try:
// 		cv_image = bridge.imgmsg_to_cv2(data, 'rgb8')
// 	except cv_bridge.CvBridgeError as e:
// 			print('Failed to convert image', e)
// 			return

// 	# the transform is same for every frame
// 	(trans, rot) = tf_.lookupTransform( 'world', 'velodyne', rospy.Time( 0 ) )

// 	# print("transformation: ", trans, rot)
// 	trans = tuple(trans) + ( 1,  )
// 	# print(trans)
// 	rotationMatrix = tf.transformations.quaternion_matrix( rot )
// 	# append translation to the last column of rotation matrix(4x4)
// 	rotationMatrix[ :, 3 ] = trans
// 	# print('rotationMatrix::  ', rotationMatrix)

// 	if velodyneData:

// 		for i in range(0, len(velodyneData) - 1):
// 			try:
// 				# converting to homogeneous coordinates
// 				point = [velodyneData[i][0], velodyneData[i][1], velodyneData[i][2],1]
// 				# print( point )

// 				# calculating dist, if dist 4m away, ignore the point
// 				if math.sqrt( np.sum( np.array( point[ :3 ] ) ** 2 ) ) > 4.0:
// 					continue

// 			except IndexError:
// 				print("Index Error!!!!!")
// 				break

// 			#  project 3D point to 2D uv
// 			rotatedPoint = rotationMatrix.dot( point )
// 			uv = cameraModel.project3dToPixel( rotatedPoint )

// 			# check if the uv point is valid
// 			if uv[0] >= 0 and uv[0] <= data.width and uv[1] >= 0 and uv[1] <= data.height:
// 				# Writing on image
// 				cv2.line(cv_image,(int( uv[0] ),int( uv[1] )),(int( uv[0] )+2,int( uv[1] ) +2),(255,0,0),3)

// 	try:
// 		imageOverlay.publish(bridge.cv2_to_imgmsg( cv_image, 'bgr8' ) )
// 	except cv_bridge.CvBridgeError as e:
// 		print( 'Failed to convert image', e )
// 		return

// if __name__ == '__main__':
// 	try:

// 		# Initialize the node and name it.
// 		rospy.init_node('lidar_overlay_image')

// 		# look up camera name, after remapping
// 		cameraName = rospy.resolve_name( 'camera' )
// 		print('Waiting for camera_info from ' + cameraName)

// 		# look up image_rect_color
// 		imageRectName = rospy.resolve_name( 'image' )
// 		print('Waiting for input image from ' + imageRectName)

// 		# look up velodyne data
// 		velodynePointName = rospy.resolve_name('velodyne')
// 		print('Waiting for velodyne point data from ' + velodynePointName)

// 		# Subscribe to topic, cameraInfo and callback function.
// 		camera = rospy.Subscriber( cameraName, CameraInfo, callback = camera_callback)
// 		imageRect = rospy.Subscriber(imageRectName, Image, callback = image_callback)
// 		velodynePoint = rospy.Subscriber(velodynePointName, PointCloud2, callback = velodyne_callback)

// 		# look up lidar overlay image
// 		imageOverlayName = rospy.resolve_name( 'image_overlay')

// 		# Publish the lidar overlay image
// 		imageOverlay = rospy.Publisher( imageOverlayName, Image, queue_size = 1)

// 		rospy.spin()

// 	except rospy.ROSInterruptException: pass

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

    CloudImageOverlay() : cloud_topic_("input"),
                          image_in_topic_("image_in"),
                          image_out_topic_("image_out"),
                          camera_info_topic_("camera_info")
    {
        sub_cloud = nh_.subscribe(cloud_topic_, 30, &CloudImageOverlay::cloud_cb, this);
        sub_cam_info = nh_.subscribe(image_in_topic_, 30, &CloudImageOverlay::image_cb, this);
        sub_cam_image = nh_.subscribe(camera_info_topic_, 30, &CloudImageOverlay::cloud_cb, this);

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
    sensor_msgs::Image image_;     //cache the image message
    std::string cloud_topic_;      
    std::string image_in_topic_;    
    std::string image_out_topic_;     
    std::string camera_info_topic_;    
    image_geometry::PinholeCameraModel cam_model_;


    ros::Subscriber sub_cloud;     //cloud subscriber
    ros::Subscriber sub_cam_info;  //cam info subscriber
    ros::Subscriber sub_cam_image; //cam image subscriber
    ros::Publisher image_pub_; //image message publisher
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "convert_pointcloud_to_image");
    CloudImageOverlay pci; //this loads up the node
    ros::spin();           //where she stops nobody knows
    return 0;
}