#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <fstream>
#include <iostream>

#include <boost/filesystem.hpp>

#include "include/btc.h"
#include "include/utils.h"
#include "ibow-lcd/lcdetector.h"
#include "ibow-lcd/alignment.h"
#include "obindex2/binary_descriptor.h"

// Read KITTI data                                                                        FIGURE OUT INCLUSIONS
std::vector<float> read_lidar_data(const std::string lidar_data_path) {

  std::ifstream lidar_data_file;
  lidar_data_file.open(lidar_data_path,
                       std::ifstream::in | std::ifstream::binary);
  if (!lidar_data_file) {
    std::cout << "Read End..." << std::endl;
    std::vector<float> nan_data;
    return nan_data;
    // exit(-1);
  }
  lidar_data_file.seekg(0, std::ios::end);
  const size_t num_elements = lidar_data_file.tellg() / sizeof(float);
  lidar_data_file.seekg(0, std::ios::beg);

  std::vector<float> lidar_data_buffer(num_elements);
  lidar_data_file.read(reinterpret_cast<char *>(&lidar_data_buffer[0]),
                       num_elements * sizeof(float));
  return lidar_data_buffer;

}

int main(int argc, char **argv) {

  // Setup                                                                             DONT WORRY ABOUT THIS ?
  ros::init(argc, argv, "test_main");
  ros::NodeHandle nh;
  std::string setting_path = "";
  std::string pcds_dir = "";
  std::string pose_file = "";
  std::string gt_file = "";
  std::string result_file = "";
  double cloud_overlap_thr = 0.5;
  bool calc_gt_enable = false;
  bool read_bin = true;

  ros::Rate loop(50000);
  ros::Rate slow_loop(1000);

  nh.param<double>("cloud_overlap_thr", cloud_overlap_thr, 0.5);
  nh.param<std::string>("setting_path", setting_path, "");
  nh.param<std::string>("pcds_dir", pcds_dir, "");
  nh.param<std::string>("pose_file", pose_file, "");
  nh.param<std::string>("gt_file", gt_file, "");
  nh.param<bool>("read_bin", read_bin, true);

  ConfigSetting config_setting;
  load_config_setting(setting_path, config_setting);

  // pose is only for visulization and gt overlap calculation
  std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> pose_list;
  std::vector<double> time_list;
  load_evo_pose_with_time(pose_file, pose_list, time_list);
  std::string print_msg = "Successfully load pose file:" + pose_file +
                          ". pose size:" + std::to_string(time_list.size());
  ROS_INFO_STREAM(print_msg.c_str());

  BtcDescManager *btc_manager = new BtcDescManager(config_setting);
  btc_manager->print_debug_info_ = false;

  bool finish = false;

  pcl::PCDReader reader;

  while (ros::ok() && !finish) {

    // Creating the loop closure detector object
    ibow_lcd::LCDetectorParams params;  // Assign desired parameters
    ibow_lcd::LCDetector lcdet(params);

    for (size_t submap_id = 0; submap_id < pose_list.size(); ++submap_id) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZI>());
      pcl::PointCloud<pcl::PointXYZI> transform_cloud;
      cloud->reserve(1000000);
      transform_cloud.reserve(1000000);
      
      // Get pose information, only for gt overlap calculation
      Eigen::Vector3d translation = pose_list[submap_id].first;
      Eigen::Matrix3d rotation = pose_list[submap_id].second;
    
      std::stringstream ss;
      ss << pcds_dir << "/" << std::setfill('0') << std::setw(6) << submap_id
         << ".bin";
      std::string pcd_file = ss.str();
      auto t_load_start = std::chrono::high_resolution_clock::now();
      std::vector<float> lidar_data = read_lidar_data(ss.str());
      if (lidar_data.size() == 0) {
        break;
      }

      for (std::size_t i = 0; i < lidar_data.size(); i += 4) {
        pcl::PointXYZI point;
        point.x = lidar_data[i];
        point.y = lidar_data[i + 1];
        point.z = lidar_data[i + 2];
        point.intensity = lidar_data[i + 3];
        Eigen::Vector3d pv = point2vec(point);
        pv = rotation * pv + translation;
        cloud->points.push_back(point);
        point = vec2point(pv);
        transform_cloud.points.push_back(vec2point(pv));
      }
      auto t_load_end = std::chrono::high_resolution_clock::now();
      std::cout << "[Time] load cloud: " << time_inc(t_load_end, t_load_start) << "ms, " << std::endl;
      
      
      // Extract desriptors
      std::vector<BTC> btcs_vec;
      btc_manager->GenerateBtcDescs(transform_cloud.makeShared(), submap_id, btcs_vec);

      btc_manager->AddBtcDescs(btcs_vec);

      down_sampling_voxel(transform_cloud, 0.5);
      btc_manager->key_cloud_vec_.push_back(transform_cloud.makeShared());

      std::pair<int, double> search_result(-1, 0);

      std::vector<cv::Point3f> kps;
      cv::Mat dscs= cv::Mat(btcs_vec.size(), 128, CV_8U);
      cv::Point3f p_kps;
      std::vector<bool> ABC;
      obindex2::BinaryDescriptor d1(128);
      std::cout << "Number of descriptors: " << btcs_vec.size() << std::endl;

      //Convert descriptors
      for (int ind = 0; ind < btcs_vec.size(); ++ind){
        p_kps.x = btcs_vec[ind].center_[0];
        p_kps.y = btcs_vec[ind].center_[1];
        p_kps.z = btcs_vec[ind].center_[2];
        kps.push_back(p_kps);
        if (ind == 0){std::cout << "kps:" << p_kps.x << " , " << p_kps.y << " , " << p_kps.z << " , " << std::endl;}

        for(int bin_ind = 0; bin_ind < btcs_vec[0].binary_A_.occupy_array_.size(); bin_ind++){
          if (ind == 0){ std::cout << btcs_vec[ind].binary_A_.occupy_array_[bin_ind];}
          if (btcs_vec[ind].binary_A_.occupy_array_[bin_ind]) {
            d1.set(bin_ind);
          } else {
            d1.reset(bin_ind);
          }
        }
        if (ind == 0){ std::cout << std::endl;}
        for(int bin_ind = 0; bin_ind < btcs_vec[0].binary_B_.occupy_array_.size(); bin_ind++){
          if (ind == 0){ std::cout << btcs_vec[ind].binary_B_.occupy_array_[bin_ind];}
          if (btcs_vec[ind].binary_B_.occupy_array_[bin_ind]) {
            d1.set(int(bin_ind + 40));
          } else {
            d1.reset(int(bin_ind + 40));
          }
        }
        if (ind == 0){ std::cout << std::endl;}
        for(int bin_ind = 0; bin_ind < btcs_vec[0].binary_C_.occupy_array_.size(); bin_ind++){
          if (ind == 0){ std::cout << btcs_vec[ind].binary_C_.occupy_array_[bin_ind];}
          if (btcs_vec[ind].binary_C_.occupy_array_[bin_ind]) {
            d1.set(int(bin_ind + 80));
          } else {
            d1.reset(int(bin_ind + 80));
          }
        }
        if (ind == 0){ std::cout << std::endl;}

        if (ind == 0){ std::cout << d1.toString() << std::endl;}
        cv::Mat m = d1.toCvMat();

        dscs.row(ind) = m.row(0);
      }

      lcdet.process(submap_id, kps, dscs, pcds_dir, search_result);

      std::cout << "[Loop Detection] triggle loop: " << submap_id << "--"
                  << search_result.first << ", score:" << search_result.second
                  << std::endl << std::endl;

    }
  }
}