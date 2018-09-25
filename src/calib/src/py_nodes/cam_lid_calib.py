#!/usr/bin/env python
 
import cv2
import numpy as np
import rospy

def worker():

    # Read Image
    im = cv2.imread("/home/amirhossein/Desktop/Mywork/Research_TAMU/autonomous_cars/lidar_camera_calibration/my_sol/data/laser_camera_ass/frame0034.jpg");
    size = im.shape
    # // frame0034.jpg
    # image_points.push_back( cv::Point2d(597, 323) );    
    # image_points.push_back( cv::Point2d(809, 345) );    
    # image_points.push_back( cv::Point2d(788, 472) );    
    # image_points.push_back( cv::Point2d(577, 454) ); 

    # // frame0000.jpg
    # image_points.push_back( cv::Point2d(272, 284) );    
    # image_points.push_back( cv::Point2d(515, 292) );    
    # image_points.push_back( cv::Point2d(263, 432) );  

    # // frame0002.jpg
    # image_points.push_back( cv::Point2d(290, 59) );    
    # image_points.push_back( cv::Point2d(278, 194) );   
    # image_points.push_back( cv::Point2d(513, 50) );    

    # // 3D model points.
    # std::vector<cv::Point3d> model_points;

    # // frame0034.jpg
    # model_points.push_back(cv::Point3d(1.4020, -0.597752, -0.133348)); // uv = [597, 323]
    # model_points.push_back(cv::Point3d(4.9928, -1.7213, -1.0266));  // uv = [809, 345]
    # model_points.push_back(cv::Point3d(1.3947, -1.1233, -0.47985)); // uv = [788, 472]
    # model_points.push_back(cv::Point3d(1.492, -0.58211, -0.42912));  // uv = [577, 454]

    # // frame0000.jpg
    # model_points.push_back(cv::Point3d(1.2935, -0.16672, 0.068351)); // uv = [272, 284]
    # model_points.push_back(cv::Point3d(1.3064, -0.37849, 0.071282));  // uv = [515, 292]
    # model_points.push_back(cv::Point3d(1.3755, 0.17589, -0.21963)); // uv = [263, 432]

    # // frame0002.jpg
    # model_points.push_back(cv::Point3d(1.3547, 0.14028, 0.36493)); // uv = [290, 59]
    # model_points.push_back(cv::Point3d(1.49293, 0.15089, 0.17647));  // uv = [278, 194]
    # model_points.push_back(cv::Point3d(1.2479, -0.39149, 0.35044));  // uv = [513, 50]    
        
    #2D image points. If you change the image, you need to change vector
    image_points = np.array([
                                (597, 323),     # Nose tip
                                (809, 345),     # Chin
                                (788, 472),     # Left eye left corner
                                (577, 454),     # Right eye right corne
                                (272, 284),     # Left Mouth corner
                                (515, 292),      # Right mouth corner
                                (263, 432),     # Chin
                                (290, 59),     # Left eye left corner
                                (278, 194),     # Right eye right corne
                                (513, 50)     # Left Mouth corner
                                
                            ], dtype="double")
    
    # 3D model points.
    ```
    # x_c = -y_l
    # y_c = z_l
    # z_c = x_l
```
    model_points = np.array([
                                (-0.597752, 0.133348, 1.4020),             # Nose tip
                                (4.9928, 1.7213, -1.0266),        # Chin
                                (1.3947, 1.1233, -0.47985),     # Left eye left corner
                                (1.492, 0.58211, -0.42912),      # Right eye right corne
                                (1.2935, 0.16672, 0.068351),    # Left Mouth corner
                                (1.3064, 0.37849, 0.071282),      # Right mouth corner
                                (1.3755, -0.17589, -0.21963),        # Chin
                                (1.3547, -0.14028, 0.36493),     # Left eye left corner
                                (1.49293, -0.15089, 0.17647),      # Right eye right corne
                                (1.2479, 0.39149, 0.35044),    # Left Mouth corner
                            ])
    
    
    # Camera internals

    #    // distortion_coefficients:
    # // rows: 1
    # // cols: 5
    # // data: [-0.196606, 0.064156, 0.002826, -0.000069, 0.000000]
    # double data[4] = { -0.196606, 0.064156, 0.002826, -0.000069};
    # cv::Mat dist_coeffs = cv::Mat(1, 4, CV_64F, data);


    # // camera_matrix:
    # // rows: 3
    # // cols: 3
    # // data: [483.761692, 0.000000, 456.184555, 0.000000, 483.550078, 365.883083, 0.000000, 0.000000, 1.000000]    
    # cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << 483.761692, 0, 456.184555, 0 , 483.550078, 365.883083, 0, 0, 1);

    
    focal_length = size[1]
    center = (size[1]/2, size[0]/2)
    camera_matrix = np.array(
                            [[483.761692, 0, 456.184555],
                            [0, 483.550078, 365.883083],
                            [0, 0, 1]], dtype = "double"
                            )
    
    print "Camera Matrix :\n {0}".format(camera_matrix)
    
    # dist_coeffs = np.zeros((4,1)) # Assuming no lens distortion
    dist_coeffs = np.array([ -0.196606, 0.064156, 0.002826, -0.000069]) # Assuming no lens distortion

    (success, rotation_vector, translation_vector) = cv2.solvePnP(model_points, image_points, camera_matrix, dist_coeffs, flags=cv2.EPNP)
    
    print "Rotation Vector:\n {0}".format(rotation_vector)
    print "Translation Vector:\n {0}".format(translation_vector)
    
    
    # Project a 3D point (0, 0, 1000.0) onto the image plane.
    # We use this to draw a line sticking out of the nose
    
    
    (projected_lidar, jacobian) = cv2.projectPoints(model_points, rotation_vector, translation_vector, camera_matrix, dist_coeffs)
    
    for p in image_points:
        cv2.circle(im, (int(p[0]), int(p[1])), 3, (0,0,255), -1)

    # print((image_points.shape()))    
    # print((projected_lidar.shape()))    

    print((image_points))    
    print((projected_lidar))    

    for q in projected_lidar:
        cv2.circle(im, (int(q[0][0]), int(q[0][1])), 3, (0,255,255), -1)

        # (temp, temp2) = cv2.projectPoints(q, rotation_vector, translation_vector, camera_matrix, dist_coeffs)
        # cv2.circle(im, (int(temp[0]), int(temp[1])), 3, (0,255,255), -1)
    
    # p1 = ( int(image_points[0][0]), int(image_points[0][1]))
    # p2 = ( int(nose_end_point2D[0][0][0]), int(nose_end_point2D[0][0][1]))
    
    # cv2.line(im, p1, p2, (255,0,0), 2)
    
    # Display image
    cv2.imshow("Output", im)
    cv2.waitKey(0)

if __name__ == '__main__':
    try:
        worker()
    except rospy.ROSInterruptException:
        pass