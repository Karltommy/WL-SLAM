#include "factormap_optimization.hpp"

void angleHandler(const wheellegged_slam::jointangleConstPtr& msgIn)
{
    wlhmtx.lock();
    wheelleg->JA_.T = msgIn->header.stamp.toSec();
    wheelleg->JA_.phi1_lf = msgIn->phi1_lf;
    wheelleg->JA_.phi1_lr = msgIn->phi1_lr;
    wheelleg->JA_.phi1_rf = msgIn->phi1_rf;
    wheelleg->JA_.phi1_rr = msgIn->phi1_rr;
    wheelleg->JA_.phi2_lf = msgIn->phi2_lf;
    wheelleg->JA_.phi2_lr = msgIn->phi2_lr;
    wheelleg->JA_.phi2_rf = msgIn->phi2_rf;
    wheelleg->JA_.phi2_rr = msgIn->phi2_rr;
    wheelleg->updateBodyheight();
    if(!wheelleg->getINIT())
    {
        wheelleg->InitH();
        wheelleg->modifyINIT();
    }
    wheelleg->angle_updated = true;
    wlhmtx.unlock();
}

void getCurPose(state_ikfom cur_state)
{
    Eigen::Vector3d eulerAngle = cur_state.rot.matrix().eulerAngles(2,1,0);   

    transformTobeMapped[0] = eulerAngle(2);              
    transformTobeMapped[1] = eulerAngle(1);            
    transformTobeMapped[2] = eulerAngle(0);           
    transformTobeMapped[3] = cur_state.pos(0);         
    transformTobeMapped[4] = cur_state.pos(1);         
    transformTobeMapped[5] = cur_state.pos(2);          
}

Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint)
{
    return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
}

Eigen::Affine3f trans2Affine3f(float transformIn[])
{
    return pcl::getTransformation(transformIn[3], transformIn[4], transformIn[5], transformIn[0], transformIn[1], transformIn[2]);
}


gtsam::Pose3 pclPointTogtsamPose3(PointTypePose thisPoint)
{
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(double(thisPoint.roll), double(thisPoint.pitch), double(thisPoint.yaw)),
                        gtsam::Point3(double(thisPoint.x), double(thisPoint.y), double(thisPoint.z)));
}


gtsam::Pose3 trans2gtsamPose(float transformIn[])
{
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(transformIn[0], transformIn[1], transformIn[2]),
                        gtsam::Point3(transformIn[3], transformIn[4], transformIn[5]));
}

//  eulerAngle 2 Quaterniond
Eigen::Quaterniond  EulerToQuat(float roll_, float pitch_, float yaw_)
{
    Eigen::Quaterniond q ;           
    Eigen::AngleAxisd roll(double(roll_), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch(double(pitch_), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw(double(yaw_), Eigen::Vector3d::UnitZ());
    q = yaw * pitch * roll ;
    q.normalize();
    return q ;
}

bool saveFrame()
{
    if (cloudKeyPoses3D->points.empty())
        return true;


    Eigen::Affine3f transStart = pclPointToAffine3f(cloudKeyPoses6D->back());

    Eigen::Affine3f transFinal = trans2Affine3f(transformTobeMapped);

    Eigen::Affine3f transBetween = transStart.inverse() * transFinal;
    float x, y, z, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw); 


    if (abs(roll) < surroundingkeyframeAddingAngleThreshold &&
        abs(pitch) < surroundingkeyframeAddingAngleThreshold &&
        abs(yaw) < surroundingkeyframeAddingAngleThreshold &&
        sqrt(x * x + y * y + z * z) < surroundingkeyframeAddingDistThreshold)
        return false;
    return true;
}



void updatePath(const PointTypePose &pose_in)
{
    string odometryFrame = "camera_init";
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time().fromSec(pose_in.time);

    pose_stamped.header.frame_id = odometryFrame;
    pose_stamped.pose.position.x =  pose_in.x;
    pose_stamped.pose.position.y = pose_in.y;
    pose_stamped.pose.position.z =  pose_in.z;
    tf::Quaternion q = tf::createQuaternionFromRPY(pose_in.roll, pose_in.pitch, pose_in.yaw);
    pose_stamped.pose.orientation.x = q.x();
    pose_stamped.pose.orientation.y = q.y();
    pose_stamped.pose.orientation.z = q.z();
    pose_stamped.pose.orientation.w = q.w();

    globalPath.poses.push_back(pose_stamped);
}


void addOdomFactor()
{
    if (cloudKeyPoses3D->points.empty())
    {

        gtsam::noiseModel::Diagonal::shared_ptr priorNoise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) <<1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12).finished()); // rad*rad, meter*meter   // indoor 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12    //  1e-2, 1e-2, M_PI*M_PI, 1e8, 1e8, 1e8
        gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(0, trans2gtsamPose(transformTobeMapped), priorNoise));

        initialEstimate.insert(0, trans2gtsamPose(transformTobeMapped));
    }
    else
    {

        gtsam::noiseModel::Diagonal::shared_ptr odometryNoise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-9, 1e-9, 1e-9, 1e-4, 1e-4, 1e-3).finished());
        gtsam::Pose3 poseFrom = pclPointTogtsamPose3(cloudKeyPoses6D->points.back()); /// pre
        gtsam::Pose3 poseTo = trans2gtsamPose(transformTobeMapped);                   // cur

        gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(cloudKeyPoses3D->size() - 1, cloudKeyPoses3D->size(), poseFrom.between(poseTo), odometryNoise));

        initialEstimate.insert(cloudKeyPoses3D->size(), poseTo);
    }
}


float pointDistance(PointType p)
{
    return sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}


float pointDistance(PointType p1, PointType p2)
{
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
}



pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose *transformIn)
{
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);
    
   
    Eigen::Isometry3d T_b_lidar(state_point.offset_R_L_I.matrix() );      
    T_b_lidar.pretranslate(state_point.offset_T_L_I);        

    Eigen::Affine3f T_w_b_ = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);
    Eigen::Isometry3d T_w_b ;            
    T_w_b.matrix() = T_w_b_.matrix().cast<double>();

    Eigen::Isometry3d  T_w_lidar  =  T_w_b * T_b_lidar  ;          

    Eigen::Isometry3d transCur = T_w_lidar;        

#pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < cloudSize; ++i)
    {
        const auto &pointFrom = cloudIn->points[i];
        cloudOut->points[i].x = transCur(0, 0) * pointFrom.x + transCur(0, 1) * pointFrom.y + transCur(0, 2) * pointFrom.z + transCur(0, 3);
        cloudOut->points[i].y = transCur(1, 0) * pointFrom.x + transCur(1, 1) * pointFrom.y + transCur(1, 2) * pointFrom.z + transCur(1, 3);
        cloudOut->points[i].z = transCur(2, 0) * pointFrom.x + transCur(2, 1) * pointFrom.y + transCur(2, 2) * pointFrom.z + transCur(2, 3);
        cloudOut->points[i].intensity = pointFrom.intensity;
    }
    return cloudOut;
}

sensor_msgs::PointCloud2 publishCloud(ros::Publisher *thisPub, pcl::PointCloud<PointType>::Ptr thisCloud, ros::Time thisStamp, std::string thisFrame)
{
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    if (thisPub->getNumSubscribers() != 0)
        thisPub->publish(tempCloud);
    return tempCloud;
}


void visualizeLoopClosure()
{
    ros::Time timeLaserInfoStamp = ros::Time().fromSec(lidar_end_time); //  时间戳
    string odometryFrame = "camera_init";

    if (loopIndexContainer.empty())
        return;

    visualization_msgs::MarkerArray markerArray;

    visualization_msgs::Marker markerNode;
    markerNode.header.frame_id = odometryFrame;
    markerNode.header.stamp = timeLaserInfoStamp;
    markerNode.action = visualization_msgs::Marker::ADD;
    markerNode.type = visualization_msgs::Marker::SPHERE_LIST;
    markerNode.ns = "loop_nodes";
    markerNode.id = 0;
    markerNode.pose.orientation.w = 1;
    markerNode.scale.x = 0.3;
    markerNode.scale.y = 0.3;
    markerNode.scale.z = 0.3;
    markerNode.color.r = 0;
    markerNode.color.g = 0.8;
    markerNode.color.b = 1;
    markerNode.color.a = 1;

    visualization_msgs::Marker markerEdge;
    markerEdge.header.frame_id = odometryFrame;
    markerEdge.header.stamp = timeLaserInfoStamp;
    markerEdge.action = visualization_msgs::Marker::ADD;
    markerEdge.type = visualization_msgs::Marker::LINE_LIST;
    markerEdge.ns = "loop_edges";
    markerEdge.id = 1;
    markerEdge.pose.orientation.w = 1;
    markerEdge.scale.x = 0.1;
    markerEdge.color.r = 0.9;
    markerEdge.color.g = 0.9;
    markerEdge.color.b = 0;
    markerEdge.color.a = 1;

  
    for (auto it = loopIndexContainer.begin(); it != loopIndexContainer.end(); ++it)
    {
        int key_cur = it->first;
        int key_pre = it->second;
        geometry_msgs::Point p;
        p.x = copy_cloudKeyPoses6D->points[key_cur].x;
        p.y = copy_cloudKeyPoses6D->points[key_cur].y;
        p.z = copy_cloudKeyPoses6D->points[key_cur].z;
        markerNode.points.push_back(p);
        markerEdge.points.push_back(p);
        p.x = copy_cloudKeyPoses6D->points[key_pre].x;
        p.y = copy_cloudKeyPoses6D->points[key_pre].y;
        p.z = copy_cloudKeyPoses6D->points[key_pre].z;
        markerNode.points.push_back(p);
        markerEdge.points.push_back(p);
    }

    markerArray.markers.push_back(markerNode);
    markerArray.markers.push_back(markerEdge);
    pubLoopConstraintEdge.publish(markerArray);
}


bool detectLoopClosureDistance(int *latestID, int *closestID)
{

    int loopKeyCur = copy_cloudKeyPoses3D->size() - 1; 
    int loopKeyPre = -1;


    auto it = loopIndexContainer.find(loopKeyCur);
    if (it != loopIndexContainer.end())
        return false;

    std::vector<int> pointSearchIndLoop;                        
    std::vector<float> pointSearchSqDisLoop;                    
    kdtreeHistoryKeyPoses->setInputCloud(copy_cloudKeyPoses3D); 
    kdtreeHistoryKeyPoses->radiusSearch(copy_cloudKeyPoses3D->back(), historyKeyframeSearchRadius, pointSearchIndLoop, pointSearchSqDisLoop, 0);

    for (int i = 0; i < (int)pointSearchIndLoop.size(); ++i)
    {
        int id = pointSearchIndLoop[i];
        if (abs(copy_cloudKeyPoses6D->points[id].time - lidar_end_time) > historyKeyframeSearchTimeDiff)
        {
            loopKeyPre = id;
            break;
        }
    }
    if (loopKeyPre == -1 || loopKeyCur == loopKeyPre)
        return false;
    *latestID = loopKeyCur;
    *closestID = loopKeyPre;

    ROS_INFO("Find loop clousre frame ");
    return true;
}


void loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr &nearKeyframes, const int &key, const int &searchNum)
{

    nearKeyframes->clear();
    int cloudSize = copy_cloudKeyPoses6D->size();
    auto surfcloud_keyframes_size = surfCloudKeyFrames.size() ;
    for (int i = -searchNum; i <= searchNum; ++i)
    {
        int keyNear = key + i;
        if (keyNear < 0 || keyNear >= cloudSize)
            continue;

        if (keyNear < 0 || keyNear >= surfcloud_keyframes_size)
            continue;


        *nearKeyframes += *transformPointCloud(surfCloudKeyFrames[keyNear], &copy_cloudKeyPoses6D->points[keyNear]); 
    }

    if (nearKeyframes->empty())
        return;


    pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
    downSizeFilterICP.setInputCloud(nearKeyframes);
    downSizeFilterICP.filter(*cloud_temp);
    *nearKeyframes = *cloud_temp;
}


void performLoopClosure()
{
    ros::Time timeLaserInfoStamp = ros::Time().fromSec(lidar_end_time); //  时间戳
    string odometryFrame = "camera_init";

    if (cloudKeyPoses3D->points.empty() == true)
    {
        return;
    }

    mtx.lock();
    *copy_cloudKeyPoses3D = *cloudKeyPoses3D;
    *copy_cloudKeyPoses6D = *cloudKeyPoses6D;
    mtx.unlock();


    int loopKeyCur;
    int loopKeyPre;

    if (detectLoopClosureDistance(&loopKeyCur, &loopKeyPre) == false)
    {
        return;
    }


    pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>()); //  cue keyframe
    pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(new pcl::PointCloud<PointType>()); //   history keyframe submap
    {

        loopFindNearKeyframes(cureKeyframeCloud, loopKeyCur, 0); 

        loopFindNearKeyframes(prevKeyframeCloud, loopKeyPre, historyKeyframeSearchNum); 

        if (pubHistoryKeyFrames.getNumSubscribers() != 0)
            publishCloud(&pubHistoryKeyFrames, prevKeyframeCloud, timeLaserInfoStamp, odometryFrame);
    }


    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(150); 
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);

    icp.setInputSource(cureKeyframeCloud);
    icp.setInputTarget(prevKeyframeCloud);
    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    icp.align(*unused_result);

    if (icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore)
        return;

    std::cout << "icp  success  " << std::endl;


    if (pubIcpKeyFrames.getNumSubscribers() != 0)
    {
        pcl::PointCloud<PointType>::Ptr closed_cloud(new pcl::PointCloud<PointType>());
        pcl::transformPointCloud(*cureKeyframeCloud, *closed_cloud, icp.getFinalTransformation());
        publishCloud(&pubIcpKeyFrames, closed_cloud, timeLaserInfoStamp, odometryFrame);
    }


    float x, y, z, roll, pitch, yaw;
    Eigen::Affine3f correctionLidarFrame;
    correctionLidarFrame = icp.getFinalTransformation();


    Eigen::Affine3f tWrong = pclPointToAffine3f(copy_cloudKeyPoses6D->points[loopKeyCur]);

    Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;
    pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw); 
    gtsam::Pose3 poseFrom = gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw), gtsam::Point3(x, y, z));

    gtsam::Pose3 poseTo = pclPointTogtsamPose3(copy_cloudKeyPoses6D->points[loopKeyPre]);
    gtsam::Vector Vector6(6);
    float noiseScore = icp.getFitnessScore() ; //  loop_clousre  noise from icp
    Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
    gtsam::noiseModel::Diagonal::shared_ptr constraintNoise = gtsam::noiseModel::Diagonal::Variances(Vector6);
    std::cout << "loopNoiseQueue   =   " << noiseScore << std::endl;


    mtx.lock();
    loopIndexQueue.push_back(make_pair(loopKeyCur, loopKeyPre));
    loopPoseQueue.push_back(poseFrom.between(poseTo));
    loopNoiseQueue.push_back(constraintNoise);
    mtx.unlock();

    loopIndexContainer[loopKeyCur] = loopKeyPre; 
}


void loopClosureThread()
{
    if (loopClosureEnableFlag == false)
    {
        std::cout << "loopClosureEnableFlag   ==  false " << endl;
        return;
    }

    ros::Rate rate(loopClosureFrequency); 
    while (ros::ok() && startFlag)
    {
        rate.sleep();
        performLoopClosure();   
        visualizeLoopClosure(); 
    }
}


void addLoopFactor()
{
    if (loopIndexQueue.empty())
        return;

    for (int i = 0; i < (int)loopIndexQueue.size(); ++i)
    {

        int indexFrom = loopIndexQueue[i].first; //   cur
        int indexTo = loopIndexQueue[i].second;  //    pre

        gtsam::Pose3 poseBetween = loopPoseQueue[i];
        gtsam::noiseModel::Diagonal::shared_ptr noiseBetween = loopNoiseQueue[i];
        gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(indexFrom, indexTo, poseBetween, noiseBetween));
    }

    loopIndexQueue.clear();
    loopPoseQueue.clear();
    loopNoiseQueue.clear();
    aLoopIsClosed = true;
}

void addGravityFactor()
{
    if(vecgravity.size()<2)
       return;
    gtsam::noiseModel::Diagonal::shared_ptr gravityNoise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(3) << 1e-6, 1e-6, 1e-6).finished());
    gtsam::Vector3 pre_gravity=vecgravity.front();
    gtsam::Vector3 cur_gravity=vecgravity.back();
    // std::cout<<"grav.size:"<<vecgravity.size()<<std::endl;
    vecgravity.pop_front();
    GravitybetweenFactor gravity_factor(cloudKeyPoses3D->size() - 1, cloudKeyPoses3D->size(),pre_gravity,cur_gravity,gravityNoise);
    gtSAMgraph.add(gravity_factor);
}

void addWlheightFactor()
{
    wlhmtx.lock();
    if(wheelleg->angle_updated = false)
    return;
    gtsam::noiseModel::Diagonal::shared_ptr wlheightNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(1) << 1e-12).finished());
    WlheightFactor wlheight_factor(cloudKeyPoses3D->size(),wheelleg->getdeltaheight(),wlheightNoise);
    gtSAMgraph.add(wlheight_factor);
    wheelleg->angle_updated = false;
    wlhmtx.unlock();
}

void longitudinal_gradient_calculate(double &longitudinal_gradient_)
{
    gtsam::Vector3 cur_gravity=state_point.grav;
    double arctanValue = atan(cur_gravity[0]/cur_gravity[2]);
    double pitchvalue = wheelleg->getpitchvalue();
    double arcsinValue = asin(pitchvalue);
    double gradient = (arctanValue-arcsinValue)* (180.0 / M_PI)-deltapitch;
    // std::cout<<"g:"<<cur_gravity[0]<<","<<cur_gravity[1]<<","<<cur_gravity[2]<<std::endl;
    longitudinal_gradient_ =gradient;
    // std::cout<<"longitudinal_gradient:"<<longitudinal_gradient_<<std::endl;
}

void lateral_gradient_calculate(double &lateral_gradient_)
{
    gtsam::Vector3 cur_gravity=state_point.grav;
    double arctanValue = atan(cur_gravity[1]/cur_gravity[2]);
    double rollvalue = wheelleg->getrollvalue();
    double arcsinValue = asin(rollvalue);
    double gradient = (-arctanValue-arcsinValue)* (180.0 / M_PI)-deltaroll;
    lateral_gradient_ = gradient;
    // std::cout<<"lateral_gradient:"<<lateral_gradient_<<std::endl;
}

void saveKeyFramesAndFactor()
{

    longitudinal_gradient_calculate(longitudinal_gradient);

    lateral_gradient_calculate(lateral_gradient);

    vecgravity.push_back(state_point.grav);
    

    addOdomFactor();

    addGravityFactor();

    // if(abs(longitudinal_gradient)<0.5 || abs(lateral_gradient)<0.5)
    addWlheightFactor();

    addLoopFactor();
 
    isam->update(gtSAMgraph, initialEstimate);
    isam->update();
    
    if (aLoopIsClosed == true) 
    {
        isam->update();
        isam->update();
        isam->update();
        isam->update();
        isam->update();
    }

    gtSAMgraph.resize(0);
    initialEstimate.clear();

    PointType thisPose3D;
    PointTypePose thisPose6D;
    gtsam::Pose3 latestEstimate;


    isamCurrentEstimate = isam->calculateBestEstimate();

    latestEstimate = isamCurrentEstimate.at<gtsam::Pose3>(isamCurrentEstimate.size() - 1);


    thisPose3D.x = latestEstimate.translation().x();
    thisPose3D.y = latestEstimate.translation().y();
    thisPose3D.z = latestEstimate.translation().z();

    thisPose3D.intensity = cloudKeyPoses3D->size(); 
    cloudKeyPoses3D->push_back(thisPose3D);         


    thisPose6D.x = thisPose3D.x;
    thisPose6D.y = thisPose3D.y;
    thisPose6D.z = thisPose3D.z;
    thisPose6D.intensity = thisPose3D.intensity;
    thisPose6D.roll = latestEstimate.rotation().roll();
    thisPose6D.pitch = latestEstimate.rotation().pitch();
    thisPose6D.yaw = latestEstimate.rotation().yaw();
    thisPose6D.time = lidar_end_time;
    cloudKeyPoses6D->push_back(thisPose6D);


    poseCovariance = isam->marginalCovariance(isamCurrentEstimate.size() - 1);


    state_ikfom state_updated = kf.get_x(); 
    Eigen::Vector3d pos(latestEstimate.translation().x(), latestEstimate.translation().y(), latestEstimate.translation().z());
    Eigen::Quaterniond q = EulerToQuat(latestEstimate.rotation().roll(), latestEstimate.rotation().pitch(), latestEstimate.rotation().yaw());


    state_updated.pos = pos;
    state_updated.rot.matrix() =  q.matrix();
    state_point = state_updated; 


    
    kf.change_x(state_updated);  

    state_ikfom state_1=kf.get_x();




    pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(new pcl::PointCloud<PointType>());
 
    pcl::copyPointCloud(*feats_undistort, *thisSurfKeyFrame); 


    surfCloudKeyFrames.push_back(thisSurfKeyFrame);

    updatePath(thisPose6D); 
}