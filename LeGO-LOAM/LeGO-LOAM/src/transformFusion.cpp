// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is an implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

#include "utility.h"

class TransformFusion{

private:

    ros::NodeHandle nh;


    ros::Publisher pubLaserOdometry2;
    ros::Subscriber subLaserOdometry;
    ros::Subscriber subOdomAftMapped;
  /*add*/
    ros::Publisher pubGlobalPath;
    ros::Publisher pubLaserMap2baselink;
    nav_msgs::Odometry laserMap2baselink;
    nav_msgs::Path Map2Baselink_path;
    geometry_msgs::PoseStamped Map2Baselink_path_points;

    nav_msgs::Odometry laserOdometry2;
    tf::StampedTransform laserOdometryTrans2;
    tf::TransformBroadcaster tfBroadcaster2;

    tf::StampedTransform map_2_camera_init_Trans;
    tf::TransformBroadcaster tfBroadcasterMap2CameraInit;

    tf::StampedTransform camera_2_base_link_Trans;
    tf::TransformBroadcaster tfBroadcasterCamera2Baselink;

    float transformSum[6];
    float transformIncre[6];
    float transformMapped[6];
    float transformBefMapped[6];
    float transformAftMapped[6];

    std_msgs::Header currentHeader;

public:

    TransformFusion(){

        pubLaserOdometry2 = nh.advertise<nav_msgs::Odometry> ("/integrated_to_init", 5);
        subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/laser_odom_to_init", 5, &TransformFusion::laserOdometryHandler, this);
        subOdomAftMapped = nh.subscribe<nav_msgs::Odometry>("/aft_mapped_to_init", 5, &TransformFusion::odomAftMappedHandler, this);

        /*add*/
        pubGlobalPath = nh.advertise<nav_msgs::Path>("lidarOdom",100);
        pubLaserMap2baselink = nh.advertise<nav_msgs::Odometry> ("/map_to_baselink", 5);


        laserOdometry2.header.frame_id = "/camera_init";
        laserOdometry2.child_frame_id = "/camera";

        laserOdometryTrans2.frame_id_ = "/camera_init";
        laserOdometryTrans2.child_frame_id_ = "/camera";

        map_2_camera_init_Trans.frame_id_ = "/map";
        map_2_camera_init_Trans.child_frame_id_ = "/camera_init";

        camera_2_base_link_Trans.frame_id_ = "/camera";
        camera_2_base_link_Trans.child_frame_id_ = "/base_link";

        /*add*/
        laserMap2baselink.header.frame_id = "/map";
        laserMap2baselink.child_frame_id = "base_link";

        for (int i = 0; i < 6; ++i)
        {
            transformSum[i] = 0;
            transformIncre[i] = 0;
            transformMapped[i] = 0;
            transformBefMapped[i] = 0;
            transformAftMapped[i] = 0;
        }
    }

    void transformAssociateToMap()
    {
        float x1 = cos(transformSum[1]) * (transformBefMapped[3] - transformSum[3]) 
                 - sin(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);
        float y1 = transformBefMapped[4] - transformSum[4];
        float z1 = sin(transformSum[1]) * (transformBefMapped[3] - transformSum[3]) 
                 + cos(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);

        float x2 = x1;
        float y2 = cos(transformSum[0]) * y1 + sin(transformSum[0]) * z1;
        float z2 = -sin(transformSum[0]) * y1 + cos(transformSum[0]) * z1;

        transformIncre[3] = cos(transformSum[2]) * x2 + sin(transformSum[2]) * y2;
        transformIncre[4] = -sin(transformSum[2]) * x2 + cos(transformSum[2]) * y2;
        transformIncre[5] = z2;

        float sbcx = sin(transformSum[0]);
        float cbcx = cos(transformSum[0]);
        float sbcy = sin(transformSum[1]);
        float cbcy = cos(transformSum[1]);
        float sbcz = sin(transformSum[2]);
        float cbcz = cos(transformSum[2]);

        float sblx = sin(transformBefMapped[0]);
        float cblx = cos(transformBefMapped[0]);
        float sbly = sin(transformBefMapped[1]);
        float cbly = cos(transformBefMapped[1]);
        float sblz = sin(transformBefMapped[2]);
        float cblz = cos(transformBefMapped[2]);

        float salx = sin(transformAftMapped[0]);
        float calx = cos(transformAftMapped[0]);
        float saly = sin(transformAftMapped[1]);
        float caly = cos(transformAftMapped[1]);
        float salz = sin(transformAftMapped[2]);
        float calz = cos(transformAftMapped[2]);

        float srx = -sbcx*(salx*sblx + calx*cblx*salz*sblz + calx*calz*cblx*cblz)
                  - cbcx*sbcy*(calx*calz*(cbly*sblz - cblz*sblx*sbly)
                  - calx*salz*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sbly)
                  - cbcx*cbcy*(calx*salz*(cblz*sbly - cbly*sblx*sblz) 
                  - calx*calz*(sbly*sblz + cbly*cblz*sblx) + cblx*cbly*salx);
        transformMapped[0] = -asin(srx);

        float srycrx = sbcx*(cblx*cblz*(caly*salz - calz*salx*saly)
                     - cblx*sblz*(caly*calz + salx*saly*salz) + calx*saly*sblx)
                     - cbcx*cbcy*((caly*calz + salx*saly*salz)*(cblz*sbly - cbly*sblx*sblz)
                     + (caly*salz - calz*salx*saly)*(sbly*sblz + cbly*cblz*sblx) - calx*cblx*cbly*saly)
                     + cbcx*sbcy*((caly*calz + salx*saly*salz)*(cbly*cblz + sblx*sbly*sblz)
                     + (caly*salz - calz*salx*saly)*(cbly*sblz - cblz*sblx*sbly) + calx*cblx*saly*sbly);
        float crycrx = sbcx*(cblx*sblz*(calz*saly - caly*salx*salz)
                     - cblx*cblz*(saly*salz + caly*calz*salx) + calx*caly*sblx)
                     + cbcx*cbcy*((saly*salz + caly*calz*salx)*(sbly*sblz + cbly*cblz*sblx)
                     + (calz*saly - caly*salx*salz)*(cblz*sbly - cbly*sblx*sblz) + calx*caly*cblx*cbly)
                     - cbcx*sbcy*((saly*salz + caly*calz*salx)*(cbly*sblz - cblz*sblx*sbly)
                     + (calz*saly - caly*salx*salz)*(cbly*cblz + sblx*sbly*sblz) - calx*caly*cblx*sbly);
        transformMapped[1] = atan2(srycrx / cos(transformMapped[0]), 
                                   crycrx / cos(transformMapped[0]));
        
        float srzcrx = (cbcz*sbcy - cbcy*sbcx*sbcz)*(calx*salz*(cblz*sbly - cbly*sblx*sblz)
                     - calx*calz*(sbly*sblz + cbly*cblz*sblx) + cblx*cbly*salx)
                     - (cbcy*cbcz + sbcx*sbcy*sbcz)*(calx*calz*(cbly*sblz - cblz*sblx*sbly)
                     - calx*salz*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sbly)
                     + cbcx*sbcz*(salx*sblx + calx*cblx*salz*sblz + calx*calz*cblx*cblz);
        float crzcrx = (cbcy*sbcz - cbcz*sbcx*sbcy)*(calx*calz*(cbly*sblz - cblz*sblx*sbly)
                     - calx*salz*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sbly)
                     - (sbcy*sbcz + cbcy*cbcz*sbcx)*(calx*salz*(cblz*sbly - cbly*sblx*sblz)
                     - calx*calz*(sbly*sblz + cbly*cblz*sblx) + cblx*cbly*salx)
                     + cbcx*cbcz*(salx*sblx + calx*cblx*salz*sblz + calx*calz*cblx*cblz);
        transformMapped[2] = atan2(srzcrx / cos(transformMapped[0]), 
                                   crzcrx / cos(transformMapped[0]));

        x1 = cos(transformMapped[2]) * transformIncre[3] - sin(transformMapped[2]) * transformIncre[4];
        y1 = sin(transformMapped[2]) * transformIncre[3] + cos(transformMapped[2]) * transformIncre[4];
        z1 = transformIncre[5];

        x2 = x1;
        y2 = cos(transformMapped[0]) * y1 - sin(transformMapped[0]) * z1;
        z2 = sin(transformMapped[0]) * y1 + cos(transformMapped[0]) * z1;

        transformMapped[3] = transformAftMapped[3] 
                           - (cos(transformMapped[1]) * x2 + sin(transformMapped[1]) * z2);
        transformMapped[4] = transformAftMapped[4] - y2;
        transformMapped[5] = transformAftMapped[5] 
                           - (-sin(transformMapped[1]) * x2 + cos(transformMapped[1]) * z2);
    }

    void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry)
    {
        currentHeader = laserOdometry->header;

        double roll, pitch, yaw;
        geometry_msgs::Quaternion geoQuat = laserOdometry->pose.pose.orientation;
        tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

        transformSum[0] = -pitch;
        transformSum[1] = -yaw;
        transformSum[2] = roll;

        transformSum[3] = laserOdometry->pose.pose.position.x;
        transformSum[4] = laserOdometry->pose.pose.position.y;
        transformSum[5] = laserOdometry->pose.pose.position.z;

        transformAssociateToMap();

        geoQuat = tf::createQuaternionMsgFromRollPitchYaw
                  (transformMapped[2], -transformMapped[0], -transformMapped[1]);

        laserOdometry2.header.stamp = laserOdometry->header.stamp;
        laserOdometry2.pose.pose.orientation.x = -geoQuat.y;
        laserOdometry2.pose.pose.orientation.y = -geoQuat.z;
        laserOdometry2.pose.pose.orientation.z = geoQuat.x;
        laserOdometry2.pose.pose.orientation.w = geoQuat.w;
        laserOdometry2.pose.pose.position.x = transformMapped[3];
        laserOdometry2.pose.pose.position.y = transformMapped[4];
        laserOdometry2.pose.pose.position.z = transformMapped[5];
        pubLaserOdometry2.publish(laserOdometry2);

        laserOdometryTrans2.stamp_ = laserOdometry->header.stamp;
        laserOdometryTrans2.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
        laserOdometryTrans2.setOrigin(tf::Vector3(transformMapped[3], transformMapped[4], transformMapped[5]));
        tfBroadcaster2.sendTransform(laserOdometryTrans2);

        /*发布base_link->map 的里程计*/
        Eigen::Matrix4d camera_2_baselink = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d map_2_init = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d map_2_baselink = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d init_2_camera = Eigen::Matrix4d::Identity();
        double roll_map2init,pitch_map2init,yaw_map2init,
                roll_camera2baselink,pitch_camera2baselink,yaw_camera2baselink;
        tf::Quaternion quat_map2init,quat_camera2baselink;
        roll_camera2baselink = 0.0;  pitch_camera2baselink = -1.570795; yaw_camera2baselink = -1.570795;
        roll_map2init = 1.570795;   pitch_map2init = 0.0;   yaw_map2init = 1.570795;
        quat_map2init = tf::createQuaternionFromRPY(roll_map2init,pitch_map2init,yaw_map2init);
        quat_camera2baselink = tf::createQuaternionFromRPY(roll_camera2baselink,pitch_camera2baselink,yaw_camera2baselink);
        camera_2_baselink.block<3, 3>(0,0) = Eigen::Quaterniond(quat_camera2baselink.getW(),quat_camera2baselink.getX(),quat_camera2baselink.getY(),quat_camera2baselink.getZ()).toRotationMatrix();
        camera_2_baselink.block<3, 1>(0,3) = Eigen::Vector3d(0,0,0);
        map_2_init.block<3, 3>(0,0) = Eigen::Quaterniond(quat_map2init.getW(),quat_map2init.getX(),quat_map2init.getY(),quat_map2init.getZ()).toRotationMatrix();
        map_2_init.block<3, 1>(0,3) = Eigen::Vector3d(0,0,0);
        /*  why init to camera :y and z <0 */
        init_2_camera.block<3, 3>(0,0) = Eigen::Quaterniond(geoQuat.w,-geoQuat.y,-geoQuat.z,geoQuat.x).toRotationMatrix();
        init_2_camera.block<3, 1>(0,3) = Eigen::Vector3d(transformMapped[3],transformMapped[4],transformMapped[5]);
//        map_2_baselink = camera_2_baselink * init_2_camera * map_2_init;
        map_2_baselink = map_2_init * init_2_camera * camera_2_baselink;
//        map_2_baselink = map_2_init.inverse() * map_2_baselink;
//        cout<<"map_2_baselink.w()"<<map_2_baselink.w()<<endl;
//        cout<<"map_2_baselink.x()"<<map_2_baselink.x()<<endl;
//        cout<<"map_2_baselink.y()"<<map_2_baselink.y()<<endl;
//        cout<<"map_2_baselink.z()"<<map_2_baselink.z()<<endl;

        Eigen::Quaterniond testQ ;
        testQ = map_2_init.block<3, 3>(0,0);
//         cout<<"testQw"<<testQ.w()<<endl;
//          cout<<"testQx"<<testQ.x()<<endl;
//           cout<<"testQy"<<testQ.y()<<endl;
//            cout<<"testQz"<<testQ.z()<<endl;

        Eigen::Quaterniond lidarQ ;
        lidarQ= map_2_baselink.block<3, 3>(0,0);
        Eigen::Vector3d lidarP = map_2_baselink.block<3, 1>(0,3);
//        cout<<"map_2_baselink"<<map_2_baselink<<endl;
//        cout<<"quat_map2init"<<quat_map2init.getW()<<endl;
//        cout<<"quat_map2init"<<quat_map2init.getX()<<endl;
//        cout<<"quat_map2init"<<quat_map2init.getY()<<endl;
//        cout<<"quat_map2init"<<quat_map2init.getZ()<<endl;
//        cout<<"quat_camera2baselink"<<quat_camera2baselink<<endl;
//        cout<<"quat_camera2baselink"<<quat_camera2baselink.getW()<<endl;
//        cout<<"quat_camera2baselink"<<quat_camera2baselink.getX()<<endl;
//        cout<<"quat_camera2baselink"<<quat_camera2baselink.getY()<<endl;
//        cout<<"quat_camera2baselink"<<quat_camera2baselink.getZ()<<endl;
        laserMap2baselink.header.stamp = laserOdometry2.header.stamp;
        laserMap2baselink.pose.pose.orientation.x = lidarQ.x();
        laserMap2baselink.pose.pose.orientation.y = lidarQ.y();
        laserMap2baselink.pose.pose.orientation.z = lidarQ.z();
        laserMap2baselink.pose.pose.orientation.w = lidarQ.w();
        laserMap2baselink.pose.pose.position.x = lidarP.x();
        laserMap2baselink.pose.pose.position.y = lidarP.y();
        laserMap2baselink.pose.pose.position.z = lidarP.z();
        pubLaserMap2baselink.publish(laserMap2baselink);


        Map2Baselink_path_points.header.stamp = laserOdometry2.header.stamp;
        Map2Baselink_path_points.pose.orientation.x = lidarQ.x();
        Map2Baselink_path_points.pose.orientation.y = lidarQ.y();
        Map2Baselink_path_points.pose.orientation.z = lidarQ.z();
        Map2Baselink_path_points.pose.orientation.w = lidarQ.w();
        Map2Baselink_path_points.pose.position.x = lidarP.x();
        Map2Baselink_path_points.pose.position.y = lidarP.y();
        Map2Baselink_path_points.pose.position.z = lidarP.z();
        Map2Baselink_path.header.stamp = laserOdometry2.header.stamp;
        Map2Baselink_path.poses.push_back(Map2Baselink_path_points);
        pubGlobalPath.publish(Map2Baselink_path);



    }

    void odomAftMappedHandler(const nav_msgs::Odometry::ConstPtr& odomAftMapped)
    {
        double roll, pitch, yaw;
        geometry_msgs::Quaternion geoQuat = odomAftMapped->pose.pose.orientation;
        tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

        transformAftMapped[0] = -pitch;
        transformAftMapped[1] = -yaw;
        transformAftMapped[2] = roll;

        transformAftMapped[3] = odomAftMapped->pose.pose.position.x;
        transformAftMapped[4] = odomAftMapped->pose.pose.position.y;
        transformAftMapped[5] = odomAftMapped->pose.pose.position.z;

        transformBefMapped[0] = odomAftMapped->twist.twist.angular.x;
        transformBefMapped[1] = odomAftMapped->twist.twist.angular.y;
        transformBefMapped[2] = odomAftMapped->twist.twist.angular.z;

        transformBefMapped[3] = odomAftMapped->twist.twist.linear.x;
        transformBefMapped[4] = odomAftMapped->twist.twist.linear.y;
        transformBefMapped[5] = odomAftMapped->twist.twist.linear.z;
    }
};




int main(int argc, char** argv)
{
    ros::init(argc, argv, "lego_loam");
    
    TransformFusion TFusion;

    ROS_INFO("\033[1;32m---->\033[0m Transform Fusion Started.");

    ros::spin();

    return 0;
}
