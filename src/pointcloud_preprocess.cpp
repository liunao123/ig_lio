#include "ig_lio/pointcloud_preprocess.h"
#include "ig_lio/timer.h"

extern Timer timer;

void PointCloudPreprocess::Process(
    const livox_ros_driver::CustomMsg::ConstPtr& msg,
    pcl::PointCloud<PointType>::Ptr& cloud_out,
    const double last_start_time) {
  double time_offset =
      (msg->header.stamp.toSec() - last_start_time) * 1000.0;  // ms

  for (size_t i = 1; i < msg->point_num; ++i) {
    if ((msg->points[i].line < num_scans_) &&
        ((msg->points[i].tag & 0x30) == 0x10 ||
         (msg->points[i].tag & 0x30) == 0x00) &&
        !HasInf(msg->points[i]) && !HasNan(msg->points[i]) &&
        !IsNear(msg->points[i], msg->points[i - 1]) &&
        (i % config_.point_filter_num == 0)) {
      PointType point;
      point.normal_x = 0;
      point.normal_y = 0;
      point.normal_z = 0;
      point.x = msg->points[i].x;
      point.y = msg->points[i].y;
      point.z = msg->points[i].z;
      point.intensity = msg->points[i].reflectivity;
      point.curvature = time_offset + msg->points[i].offset_time * 1e-6;  // ms
      cloud_out->push_back(point);
    }
  }
}

void PointCloudPreprocess::Process(
    const sensor_msgs::PointCloud2::ConstPtr& msg,
    pcl::PointCloud<PointType>::Ptr& cloud_out) {
  switch (config_.lidar_type) {
  case LidarType::VELODYNE:
    ProcessVelodyne(msg, cloud_out);
    break;
  case LidarType::OUSTER:
    ProcessOuster(msg, cloud_out);
    break;
  case LidarType::Robosense:
    RobosenseHandler(msg, cloud_out);
    break;
  default:
    LOG(INFO) << "Error LiDAR Type!!!" << std::endl;
    exit(0);
  }
}

void PointCloudPreprocess::ProcessVelodyne(
    const sensor_msgs::PointCloud2::ConstPtr& msg,
    pcl::PointCloud<PointType>::Ptr& cloud_out) {
  pcl::PointCloud<VelodynePointXYZIRT> cloud_origin;
  pcl::fromROSMsg(*msg, cloud_origin);

  // These variables only works when no point timestamps given
  int plsize = cloud_origin.size();
  double omega_l = 3.61;  // scan angular velocity
  std::vector<bool> is_first(num_scans_, true);
  std::vector<double> yaw_fp(num_scans_, 0.0);    // yaw of first scan point
  std::vector<float> yaw_last(num_scans_, 0.0);   // yaw of last scan point
  std::vector<float> time_last(num_scans_, 0.0);  // last offset time
  if (cloud_origin.back().time > 0) {
    has_time_ = true;
  } else {
    LOG(INFO) << "origin cloud has not timestamp";
    has_time_ = false;
    double yaw_first =
        atan2(cloud_origin.points[0].y, cloud_origin.points[0].x) * 57.29578;
    double yaw_end = yaw_first;
    int layer_first = cloud_origin.points[0].ring;
    for (uint i = plsize - 1; i > 0; i--) {
      if (cloud_origin.points[i].ring == layer_first) {
        yaw_end = atan2(cloud_origin.points[i].y, cloud_origin.points[i].x) *
                  57.29578;
        break;
      }
    }
  }

  cloud_out->reserve(cloud_origin.size());

  for (size_t i = 0; i < cloud_origin.size(); ++i) {
    if ((i % config_.point_filter_num == 0) && !HasInf(cloud_origin.at(i)) &&
        !HasNan(cloud_origin.at(i))) {
      PointType point;
      point.normal_x = 0;
      point.normal_y = 0;
      point.normal_z = 0;
      point.x = cloud_origin.at(i).x;
      point.y = cloud_origin.at(i).y;
      point.z = cloud_origin.at(i).z;
      point.intensity = cloud_origin.at(i).intensity;
      if (has_time_) {
        // curvature unit: ms
        point.curvature = cloud_origin.at(i).time * config_.time_scale;
        // std::cout<<point.curvature<<std::endl;
        // if(point.curvature < 0){
        //     std::cout<<"time < 0 : "<<point.curvature<<std::endl;
        // }
      } else {
        int layer = cloud_origin.points[i].ring;
        double yaw_angle = atan2(point.y, point.x) * 57.2957;

        if (is_first[layer]) {
          yaw_fp[layer] = yaw_angle;
          is_first[layer] = false;
          point.curvature = 0.0;
          yaw_last[layer] = yaw_angle;
          time_last[layer] = point.curvature;
          continue;
        }

        // compute offset time
        if (yaw_angle <= yaw_fp[layer]) {
          point.curvature = (yaw_fp[layer] - yaw_angle) / omega_l;
        } else {
          point.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;
        }

        if (point.curvature < time_last[layer])
          point.curvature += 360.0 / omega_l;

        if (!std::isfinite(point.curvature)) {
          continue;
        }

        yaw_last[layer] = yaw_angle;
        time_last[layer] = point.curvature;
      }

      cloud_out->push_back(point);
    }
  }
}

void PointCloudPreprocess::ProcessOuster(
    const sensor_msgs::PointCloud2::ConstPtr& msg,
    pcl::PointCloud<PointType>::Ptr& cloud_out) {
  pcl::PointCloud<OusterPointXYZIRT> cloud_origin;
  pcl::fromROSMsg(*msg, cloud_origin);

  for (size_t i = 0; i < cloud_origin.size(); ++i) {
    if ((i % config_.point_filter_num == 0) && !HasInf(cloud_origin.at(i)) &&
        !HasNan(cloud_origin.at(i))) {
      PointType point;
      point.normal_x = 0;
      point.normal_y = 0;
      point.normal_z = 0;
      point.x = cloud_origin.at(i).x;
      point.y = cloud_origin.at(i).y;
      point.z = cloud_origin.at(i).z;
      point.intensity = cloud_origin.at(i).intensity;
      // ms
      point.curvature = cloud_origin.at(i).t * 1e-6;
      cloud_out->push_back(point);
    }
  }
}

void PointCloudPreprocess::RobosenseHandler(
    const sensor_msgs::PointCloud2::ConstPtr& msg,
    pcl::PointCloud<PointType>::Ptr& cloud_out) {

    cloud_out->clear();

    pcl::PointCloud< RsPointXYZIRT > pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    
    // ROS_WARN("first, last , msg->header.stamp.toSec() : %f . %f. %f ", pl_orig.points[0].timestamp , pl_orig.points.back().timestamp , msg->header.stamp.toSec());
    auto first_point_time = pl_orig.points[0].timestamp;
    
    // 激光雷达，去除 nan 点
    pl_orig.is_dense = false; // 万集的雷达必须加这一句
    std::vector<int> save_index;
    // ROS_ERROR("pl_orig->size()is %d",  pl_orig.size());
    pcl::removeNaNFromPointCloud(pl_orig, pl_orig, save_index);
    // ROS_ERROR("pl_orig->size()is %d",  pl_orig.size());
    // ROS_ERROR("RobosenseHandler" );

    int plsize = pl_orig.points.size();
    if (plsize == 0)
    {
        ROS_ERROR("NO POINTS ......");
        return;
    }
    cloud_out->reserve(plsize);
    for (int i = 0; i < plsize; i++) {
        PointType added_pt;
        added_pt.normal_x = 0;
        added_pt.normal_y = 0;
        added_pt.normal_z = 0;
        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;
        added_pt.curvature =  (pl_orig.points[i].timestamp - first_point_time) * 1000.0 ;  // curvature unit: ms

        if ( 1 ) {
            float range_temp_sqrt = added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z ;
            if(range_temp_sqrt < 0.5 * 0.5 ) // max_blind 认为是 雷达的有效探测范围
            {
              continue;
            }

        // if ( added_pt.x < -100 || added_pt.x > 200 || 
        //      added_pt.y < -10.0 || added_pt.y > 10.0 || 
        //      added_pt.z < -5.0 || added_pt.z > 10 )
        // {
        //   continue;
        // }


            cloud_out->points.push_back(added_pt);
        }
    }
}


template <typename T>
inline bool PointCloudPreprocess::HasInf(const T& p) {
  return (std::isinf(p.x) || std::isinf(p.y) || std::isinf(p.z));
}

template <typename T>
inline bool PointCloudPreprocess::HasNan(const T& p) {
  return (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z));
}

template <typename T>
inline bool PointCloudPreprocess::IsNear(const T& p1, const T& p2) {
  return ((abs(p1.x - p2.x) < 1e-7) || (abs(p1.y - p2.y) < 1e-7) ||
          (abs(p1.z - p2.z) < 1e-7));
}
