

// gstam
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <ikd-Tree/ikd_Tree.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include "esekfom.hpp"
#include "kinematicsestimate.hpp"
#include "wheellegged_slam/jointangle.h"
#include <cmath>

struct PointXYZIRPYT
{
    PCL_ADD_POINT4D     
    PCL_ADD_INTENSITY;  
    float roll;         
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   
} EIGEN_ALIGN16;                    

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time))

typedef PointXYZIRPYT  PointTypePose;

float transformTobeMapped[6];

vector<pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFrames; 
vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames;   

pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D(new pcl::PointCloud<PointType>());         
pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D(new pcl::PointCloud<PointTypePose>()); 
pcl::PointCloud<PointType>::Ptr copy_cloudKeyPoses3D(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointTypePose>::Ptr copy_cloudKeyPoses6D(new pcl::PointCloud<PointTypePose>());

pcl::PointCloud<PointTypePose>::Ptr fastlio_unoptimized_cloudKeyPoses6D(new pcl::PointCloud<PointTypePose>()); 
pcl::PointCloud<PointTypePose>::Ptr gnss_cloudKeyPoses6D(new pcl::PointCloud<PointTypePose>()); 

// Surrounding map
float surroundingkeyframeAddingDistThreshold;  
float surroundingkeyframeAddingAngleThreshold; 
float surroundingKeyframeDensity;
float surroundingKeyframeSearchRadius;

PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());

bool aLoopIsClosed = false;
map<int, int> loopIndexContainer; // from new to old
vector<pair<int, int>> loopIndexQueue;
vector<gtsam::Pose3> loopPoseQueue;
vector<gtsam::noiseModel::Diagonal::shared_ptr> loopNoiseQueue;
deque<std_msgs::Float64MultiArray> loopInfoVec;

esekfom::esekf kf;

std::mutex mtx;
std::mutex mtxLoopInfo;
std::mutex wlhmtx;

// CPU Params
int numberOfCores = 4;
double mappingProcessInterval;

// global map visualization radius
float globalMapVisualizationSearchRadius;
float globalMapVisualizationPoseDensity;
float globalMapVisualizationLeafSize;

nav_msgs::Path globalPath;

bool    recontructKdTree = false;
int updateKdtreeCount = 0 ;        
bool visulize_IkdtreeMap = false;            //  visual iktree submap

state_ikfom state_point;

double lidar_end_time = 0;

// gtsam
gtsam::NonlinearFactorGraph gtSAMgraph;
gtsam::Values initialEstimate;
gtsam::Values optimizedEstimate;
gtsam::ISAM2 *isam;
gtsam::Values isamCurrentEstimate;
Eigen::MatrixXd poseCovariance;

// 局部关键帧构建的map点云，对应kdtree，用于scan-to-map找相邻点
pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap(new pcl::KdTreeFLANN<PointType>());
pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap(new pcl::KdTreeFLANN<PointType>());

pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurroundingKeyPoses(new pcl::KdTreeFLANN<PointType>());
pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses(new pcl::KdTreeFLANN<PointType>());

/*loop clousre*/
bool startFlag = true;
bool loopClosureEnableFlag;
float loopClosureFrequency; //   回环检测频率
int surroundingKeyframeSize;
float historyKeyframeSearchRadius;   // 回环检测 radius kdtree搜索半径
float historyKeyframeSearchTimeDiff; //  帧间时间阈值
int historyKeyframeSearchNum;        //   回环时多少个keyframe拼成submap
float historyKeyframeFitnessScore;   // icp 匹配阈值
bool potentialLoopFlag = false;


pcl::VoxelGrid<PointType> downSizeFilterICP;
ros::Publisher pubHistoryKeyFrames; //  发布 loop history keyframe submap
ros::Publisher pubIcpKeyFrames;
ros::Publisher pubLoopConstraintEdge;

float mappingSurfLeafSize;

deque<gtsam::Vector3> vecgravity;

int frameflag = 0;

//轮腿参数
double thigh_length;
double shank_length;
double wheel_radius;
double body_length;
double body_width;
double distance_front;
double distance_height;
double deltapitch;
double deltaroll;
double longitudinal_gradient;
double lateral_gradient;
bool save_gradient;

Wheelleg *wheelleg;

void angleHandler(const wheellegged_slam::jointangleConstPtr& msgIn);

void getCurPose(state_ikfom cur_state);

Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint);

Eigen::Affine3f trans2Affine3f(float transformIn[]);

gtsam::Pose3 pclPointTogtsamPose3(PointTypePose thisPoint);

gtsam::Pose3 trans2gtsamPose(float transformIn[]);

Eigen::Quaterniond  EulerToQuat(float roll_, float pitch_, float yaw_);

bool saveFrame();

void updatePath(const PointTypePose &pose_in);

void addOdomFactor();

float pointDistance(PointType p);

float pointDistance(PointType p1, PointType p2);

pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose *transformIn);

sensor_msgs::PointCloud2 publishCloud(ros::Publisher *thisPub, pcl::PointCloud<PointType>::Ptr thisCloud, ros::Time thisStamp, std::string thisFrame);

void visualizeLoopClosure();

bool detectLoopClosureDistance(int *latestID, int *closestID);

void loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr &nearKeyframes, const int &key, const int &searchNum);

void performLoopClosure();

void loopClosureThread();

void addLoopFactor();

void addGravityFactor();

void addWlheightFactor();

void longitudinal_gradient_calculate(double &longitudinal_gradient_);

void lateral_gradient_calculate(double &lateral_gradient_);

void saveKeyFramesAndFactor();

class GravitybetweenFactor:public gtsam::NoiseModelFactor2<gtsam::Pose3,gtsam::Pose3>
{

    private:
    
    gtsam::Vector3 pre_gravity_;
    gtsam::Vector3 cur_gravity_;

    public:
    GravitybetweenFactor(gtsam::Key preKey, gtsam::Key curKey,const gtsam::Vector3 &pre_gravity,const gtsam::Vector3 &cur_gravity, gtsam::SharedNoiseModel model) :
    gtsam::NoiseModelFactor2<gtsam::Pose3,gtsam::Pose3>(model, preKey, curKey),pre_gravity_(pre_gravity),cur_gravity_(cur_gravity){}


    gtsam::Vector evaluateError(const gtsam::Pose3& p1, const gtsam::Pose3& p2, boost::optional<gtsam::Matrix&> H1 = boost::none, boost::optional<gtsam::Matrix&> H2 = boost::none) const
    {
        gtsam::Matrix36 DR1;
        gtsam::Vector3 V1=p1.rotation().matrix().inverse()*pre_gravity_;
        gtsam::Vector3 V2=p2.rotation().matrix().inverse()*cur_gravity_;
        gtsam::Matrix33 skew1=(gtsam::Matrix33()<<0.0,-V1(2),V1(1),
            V1(2),0.0,-V1(0),
            -V1(1),V1(0),0.0).finished();
        gtsam::Matrix33 skew2=(gtsam::Matrix33()<<0.0,V2(2),-V2(1),
            -V2(2),0.0,V2(0),
            V2(1),-V2(0),0.0).finished();
        DR1.leftCols<3>()= skew1;
        DR1.rightCols<3>()= gtsam::Z_3x3;
        gtsam::Matrix36 DR2;
        DR2.leftCols<3>()= skew2;
        DR2.rightCols<3>()= gtsam::Z_3x3;
        
        if (H1) *H1 = DR1;
        if (H2) *H2 = DR2;

        return (gtsam::Vector3()<< p1.rotation().inverse()*pre_gravity_-p2.rotation().inverse()*cur_gravity_).finished();
    }

};//重力因子自定义

void addGravityFactor();



