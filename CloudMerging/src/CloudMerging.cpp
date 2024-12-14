#include <ros/ros.h>
#include <vector>
#include <livox_ros_driver/CustomMsg.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <livox_ros_driver2/CustomMsg.h>
#include <iostream>

struct PointType {
    PCL_ADD_POINT4D;                  // quad-word XYZ
    float intensity;
    float curvature;
    uint16_t ring;                    // Ring number
    double time;                      // Time information
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // Ensure proper alignment
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointType,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, curvature, curvature)
    (uint16_t, ring, ring)
    (double, time, time)
)

typedef pcl::PointCloud<PointType> PointCloudXYZI;

class SubscribeAndPublish
{
public:
    double time_piancha;
    std::vector<double> extrinT = {0.0, 0.0, 0.0};
    std::vector<double> extrinR = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};

    SubscribeAndPublish() {
        pub2 = node.advertise<sensor_msgs::PointCloud2>("/test_pointcloud", 10);
        pub = node.advertise<livox_ros_driver::CustomMsg>("/livox/lidar", 10);
        sub_front = new message_filters::Subscriber<livox_ros_driver::CustomMsg>(node, "/livox/lidar_3JEDL8C00164531", 200000);
        sub_end = new message_filters::Subscriber<livox_ros_driver2::CustomMsg>(node, "/livox/lidar_192_168_1_139", 200000);
        node.getParam("time_piancha", time_piancha);
        node.param<std::vector<double>>("merg/extrinsic_T", extrinT, std::vector<double>());
        node.param<std::vector<double>>("merg/extrinsic_R", extrinR, std::vector<double>());

        typedef message_filters::sync_policies::ApproximateTime<livox_ros_driver::CustomMsg, livox_ros_driver2::CustomMsg> syncPolicy;
        message_filters::Synchronizer<syncPolicy> sync(syncPolicy(9), *sub_front, *sub_end);
        sync.registerCallback(boost::bind(&SubscribeAndPublish::callBack, this, _1, _2));
        ros::spin();
    }

    void callBack(const livox_ros_driver::CustomMsgConstPtr& lidar_front, const livox_ros_driver2::CustomMsgConstPtr& lidar_end) {
        PointCloudXYZI pointCloud_front;
        PointCloudXYZI pointCloud_end;
        PointCloudXYZI pointCloud_end_out;
        PointCloudXYZI finalPointCloud;

        convert2PointCloud2(lidar_front, pointCloud_front, true);
        convert2PointCloud2(lidar_end, pointCloud_end, false);

        Eigen::Matrix4f transform2end = Eigen::Matrix4f::Identity();
        transform2end(0, 0) = extrinR[0];
        transform2end(0, 1) = extrinR[1];
        transform2end(0, 2) = extrinR[2];
        transform2end(0, 3) = extrinT[0];
        transform2end(1, 0) = extrinR[3];
        transform2end(1, 1) = extrinR[4];
        transform2end(1, 2) = extrinR[5];
        transform2end(1, 3) = extrinT[1];
        transform2end(2, 0) = extrinR[6];
        transform2end(2, 1) = extrinR[7];
        transform2end(2, 2) = extrinR[8];
        transform2end(2, 3) = extrinT[2];

        pcl::transformPointCloud(pointCloud_end, pointCloud_end_out, transform2end);

        finalPointCloud = pointCloud_front + pointCloud_end_out;

        livox_ros_driver::CustomMsg finalMsg;
        sensor_msgs::PointCloud2 msg2;
        finalMsg.header = lidar_front->header;
        finalMsg.timebase = lidar_front->timebase;
        finalMsg.point_num = finalPointCloud.size();
        finalMsg.lidar_id = lidar_front->lidar_id;

        for (unsigned int i = 0; i < finalMsg.point_num; i++) {
            livox_ros_driver::CustomPoint p;
            p.x = finalPointCloud[i].x;
            p.y = finalPointCloud[i].y;
            p.z = finalPointCloud[i].z;
            p.reflectivity = finalPointCloud[i].intensity;
            p.offset_time = finalPointCloud[i].curvature * float(1000000);
            p.line = finalPointCloud[i].ring;
            finalMsg.points.push_back(p);
        }

        pcl::toROSMsg(finalPointCloud, msg2);
        msg2.header = lidar_front->header;
        msg2.header.frame_id = "horizon";
        msg2.header.stamp.fromNSec(lidar_front->timebase + lidar_front->points.back().offset_time);

        pub.publish(finalMsg);
        pub2.publish(msg2);
    }

    void convert2PointCloud2(const livox_ros_driver::CustomMsgConstPtr& lidarMsg, PointCloudXYZI& pclPointCloud, bool is_front) {
        for (unsigned int i = 0; i < lidarMsg->point_num; i++) {
            PointType point;
            point.x = lidarMsg->points[i].x;
            point.y = lidarMsg->points[i].y;
            point.z = lidarMsg->points[i].z;
            point.intensity = lidarMsg->points[i].reflectivity;
            point.curvature = lidarMsg->points[i].offset_time / float(1000000);
            point.ring = lidarMsg->points[i].line; // Using line as ring
            point.time = lidarMsg->timebase + lidarMsg->points[i].offset_time / 1e9; // Convert ns to seconds
            // if(point.z*point.z>1.2)
			// continue;
            pclPointCloud.push_back(point);
        }
    }

    void convert2PointCloud2(const livox_ros_driver2::CustomMsgConstPtr& lidarMsg, PointCloudXYZI& pclPointCloud, bool is_front) {
        for (unsigned int i = 0; i < lidarMsg->point_num; i++) {
            PointType point;
            point.x = lidarMsg->points[i].x;
            point.y = lidarMsg->points[i].y;
            point.z = lidarMsg->points[i].z;
            point.intensity = lidarMsg->points[i].reflectivity;
            point.curvature = lidarMsg->points[i].offset_time / float(1000000);
            point.ring = lidarMsg->points[i].line; // Using line as ring
            point.time = lidarMsg->timebase + lidarMsg->points[i].offset_time / 1e9; // Convert ns to seconds
            if (point.x * point.x + point.y * point.y + point.z * point.z < 0.06)
                continue;
            // if(abs(point.y)>1.2)
			// continue;
            pclPointCloud.push_back(point);
        }
    }

private:
    ros::NodeHandle node;
    ros::Publisher pub;
    ros::Publisher pub2;
    message_filters::Subscriber<livox_ros_driver::CustomMsg>* sub_front;
    message_filters::Subscriber<livox_ros_driver2::CustomMsg>* sub_end;
};

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "pointCloudMerge");

    SubscribeAndPublish sap;

    return 0;
}
