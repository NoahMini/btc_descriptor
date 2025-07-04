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
  ros::init(argc, argv, "btc_place_recognition");
  ros::NodeHandle nh;
  std::string setting_path = "";
  std::string pcds_dir = "";
  std::string pose_file = "";
  std::string gt_file = "";
  std::string result_file = "";
  double cloud_overlap_thr = 0.5;
  bool calc_gt_enable = false;
  bool read_bin = true;
  int prev_match = -1;
  int count_tp_ov = 0;
  int count_fp_ov = 0;
  int count_tp_in = 0;
  int count_fp_in = 0;

  int count_fn = 0;
  int count_tn = 0;

  nh.param<double>("cloud_overlap_thr", cloud_overlap_thr, 0.5);
  nh.param<std::string>("setting_path", setting_path, "");
  nh.param<std::string>("pcds_dir", pcds_dir, "");
  nh.param<std::string>("pose_file", pose_file, "");
  nh.param<std::string>("gt_file", gt_file, "");
  nh.param<bool>("read_bin", read_bin, true);

  ros::Publisher pubOdomAftMapped =
      nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 10);
  ros::Publisher pubCureentCloud =
      nh.advertise<sensor_msgs::PointCloud2>("/cloud_current", 100);
  ros::Publisher pubCurrentBinary =
      nh.advertise<sensor_msgs::PointCloud2>("/cloud_key_points", 100);
  ros::Publisher pubPath =
      nh.advertise<visualization_msgs::MarkerArray>("descriptor_line", 10);
  ros::Publisher pubCurrentPose =
      nh.advertise<nav_msgs::Odometry>("/current_pose", 10);
  ros::Publisher pubMatchedPose =
      nh.advertise<nav_msgs::Odometry>("/matched_pose", 10);
  ros::Publisher pubMatchedCloud =
      nh.advertise<sensor_msgs::PointCloud2>("/cloud_matched", 100);
  ros::Publisher pubMatchedBinary =
      nh.advertise<sensor_msgs::PointCloud2>("/cloud_matched_key_points", 100);
  ros::Publisher pubLoopStatus =
      nh.advertise<visualization_msgs::MarkerArray>("/loop_status", 100);
  ros::Publisher pubBTC =
      nh.advertise<visualization_msgs::MarkerArray>("descriptor_line", 10);

  std_msgs::ColorRGBA color_tp;
  std_msgs::ColorRGBA color_fp;
  std_msgs::ColorRGBA color_tn;
  std_msgs::ColorRGBA color_fn;

  double scale_tp = 5.0;
  double scale_fp = 5.0;
  double scale_tn = 3.0;
  double scale_fn = 5.0;

  color_tp.a = 1.0;
  color_tp.r = 0.0;
  color_tp.g = 1.0;
  color_tp.b = 0.0;

  color_fp.a = 1.0;
  color_fp.r = 1.0;
  color_fp.g = 0.0;
  color_fp.b = 0.0;

  color_tn.a = 1.0;
  color_tn.r = 1.0;
  color_tn.g = 1.0;
  color_tn.b = 1.0;

  color_fn.a = 1.0;
  color_fn.r = 0.0;
  color_fn.g = 0.0;
  color_fn.b = 1.0;


  ros::Rate loop(50000);
  ros::Rate slow_loop(1000);

  ConfigSetting config_setting;
  load_config_setting(setting_path, config_setting);

  // pose is only for visulization and gt overlap calculation
  std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> pose_list;
  std::vector<double> time_list;
  load_evo_pose_with_time(pose_file, pose_list, time_list);
  std::string print_msg = "Successfully load pose file:" + pose_file +
                          ". pose size:" + std::to_string(time_list.size());
  ROS_INFO_STREAM(print_msg.c_str());

  int n = time_list.size();
  std::ifstream myfile;
  
  myfile.open (gt_file);
  if (!myfile.is_open()) {
    std::cout << "Failed to open file for reading.\n";
    return 1;
  }
  
  int **loop_mat = new int*[n];
  for (int i = 0; i < n; ++i)
      loop_mat[i] = new int[n];

  std::cout << n << std::endl;
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      int c;
      myfile >> c;
      loop_mat[i][j] = c;
      myfile.ignore();
      
      // std::cout << loop_mat[i][j];
    }
    // std::cout << std::endl;
  }
  std::vector<int> loop_sum;

  // Compute row sums
  for(int i = 0; i < n; i++) {
    int sum = 0;
    for(int j = 0; j < n; j++) {
        sum += loop_mat[i][j];
    }
    // if (sum > 5){std::cout << "id: " << i << " with sum of " << sum << std::endl;}
    loop_sum.push_back(sum);
  }
  //std::cout << loop_mat[0][1];   0
  //std::cout << loop_mat[1][0];   1

  BtcDescManager *btc_manager = new BtcDescManager(config_setting);
  btc_manager->print_debug_info_ = false;
  pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(
      new pcl::PointCloud<pcl::PointXYZI>());

  std::vector<double> descriptor_time;
  std::vector<double> querying_time;
  std::vector<double> update_time;
  int triggle_loop_num = 0;
  int true_loop_num = 0;
  bool finish = false;

  pcl::PCDReader reader;

  std::ofstream loopFile;
  loopFile.open("/home/noah/tfm/src/btc_descriptor/evaluation/results/KITTI00/loops.txt");

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
      if (read_bin) {
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
        std::cout << "[Time] load cloud: " << time_inc(t_load_end, t_load_start)
                  << "ms, " << std::endl;
      } else {
        // Load point cloud from pcd file
        std::stringstream ss;
        ss << pcds_dir << "/" << std::setfill('0') << std::setw(6) << submap_id
           << ".pcd";
        std::string pcd_file = ss.str();

        auto t_load_start = std::chrono::high_resolution_clock::now();
        // pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_file, *cloud)
        if (reader.read(pcd_file, *cloud) == -1) {
          ROS_ERROR_STREAM("Couldn't read file " << pcd_file);
          continue;
        }
        auto t_load_end = std::chrono::high_resolution_clock::now();
        std::cout << "[Time] load cloud: " << time_inc(t_load_end, t_load_start)
                  << "ms, " << std::endl;
        transform_cloud = *cloud;

        for (size_t j = 0; j < transform_cloud.size(); j++) {
          Eigen::Vector3d pv = point2vec(transform_cloud.points[j]);
          pv = rotation * pv + translation;
          transform_cloud.points[j] = vec2point(pv);
        }
      }

      // step1. Descriptor Extraction                                                   THIS IS FINE
      std::cout << "[Description] submap id:" << submap_id << std::endl;
      auto t_descriptor_begin = std::chrono::high_resolution_clock::now();
      std::vector<BTC> btcs_vec;
      btc_manager->GenerateBtcDescs(transform_cloud.makeShared(), submap_id,
                                    btcs_vec);
      auto t_descriptor_end = std::chrono::high_resolution_clock::now();
      descriptor_time.push_back(time_inc(t_descriptor_end, t_descriptor_begin));
      
      // step2_1. Transform BTC descriptor to fit iBoW::process requirements                                                        
      auto t_query_begin = std::chrono::high_resolution_clock::now();
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
          if (btcs_vec[ind].binary_A_.occupy_array_[bin_ind]) {
            d1.set(bin_ind);
          } else {
            d1.reset(bin_ind);
          }
        }
        for(int bin_ind = 0; bin_ind < btcs_vec[0].binary_B_.occupy_array_.size(); bin_ind++){
          if (btcs_vec[ind].binary_B_.occupy_array_[bin_ind]) {
            d1.set(int(bin_ind + 40));
          } else {
            d1.reset(int(bin_ind + 40));
          }
        }
        for(int bin_ind = 0; bin_ind < btcs_vec[0].binary_C_.occupy_array_.size(); bin_ind++){
          if (btcs_vec[ind].binary_C_.occupy_array_[bin_ind]) {
            d1.set(int(bin_ind + 80));
          } else {
            d1.reset(int(bin_ind + 80));
          }
        }

        if (ind == 0){std::cout << d1.toString() << std::endl;}
        cv::Mat m = d1.toCvMat();

        dscs.row(ind) = m.row(0);
      }

      // step2_2. Searching Loop   
      auto t_transform_end = std::chrono::high_resolution_clock::now();
      std::cout << "[Time] Transform input from BTC: " << time_inc(t_transform_end, t_query_begin) << "ms, " << std::endl;

      lcdet.debug(submap_id, kps, dscs, search_result, prev_match, loopFile);

      std::cout << "[Loop Detection] triggle loop: " << submap_id << "--"
                  << search_result.first << ", score:" << search_result.second << std::endl << std::endl;

      // down sample to save memory
      down_sampling_voxel(transform_cloud, 0.5);
      btc_manager->key_cloud_vec_.push_back(transform_cloud.makeShared());
      
      //Check for inliers
      if (search_result.first >= 0){
        pcl::PointCloud<pcl::PointXYZ>::Ptr qtransform_cloud(new pcl::PointCloud<pcl::PointXYZ>());

        for (std::size_t i = 0; i < btc_manager->key_cloud_vec_[submap_id]->points.size(); i++) {
        pcl::PointXYZ point;
        point.x = btc_manager->key_cloud_vec_[submap_id]->points[i].x;
        point.y = btc_manager->key_cloud_vec_[submap_id]->points[i].y;
        point.z = btc_manager->key_cloud_vec_[submap_id]->points[i].z;
        qtransform_cloud->points.push_back(point);
        }
        std::cout << "Read image_id fine" << std::endl;

        pcl::PointCloud<pcl::PointXYZ>::Ptr ttransform_cloud(new pcl::PointCloud<pcl::PointXYZ>());

        for (std::size_t i = 0; i < btc_manager->key_cloud_vec_[search_result.first]->points.size(); i++) {
        pcl::PointXYZ point;
        point.x = btc_manager->key_cloud_vec_[search_result.first]->points[i].x;
        point.y = btc_manager->key_cloud_vec_[search_result.first]->points[i].y;
        point.z = btc_manager->key_cloud_vec_[search_result.first]->points[i].z;
        ttransform_cloud->points.push_back(point);
        }
        std::cout << "Read best_img fine" << std::endl;

        if (!qtransform_cloud->empty() && !ttransform_cloud->empty()){
        
          std::cout << "Query cloud size: " << qtransform_cloud->points.size() << std::endl;

          std::cout << "Train cloud size: " << ttransform_cloud->points.size() << std::endl;

          ibow_lcd::AlignmentResult result = ibow_lcd::computeCloudTransform(qtransform_cloud, ttransform_cloud);
          std::cout << "got out with inliers: " << result.inliers << std::endl;

          loopFile << result.inliers << "\t";                    // Inliers
          loopFile << std::endl;

          if ((result.inliers >= 2000) && (search_result.second == 1)) {  
            search_result.second = result.inliers;
            lcdet.consecutive_loops_++;
            prev_match = search_result.first;
            std::cout << " Loop detected: Enough inliers" << std::endl;
          } else {
            search_result.first = -1;
            search_result.second = result.inliers;
            prev_match = -1;
            std::cout << " No loop: Not enough inliers" << std::endl;
            lcdet.consecutive_loops_ = 0;
          }
        }
      }
      
      prev_match = search_result.first;
      
      loop.sleep();
    }
    finish = true;
  }

  double mean_descriptor_time =
      std::accumulate(descriptor_time.begin(), descriptor_time.end(), 0) * 1.0 /
      descriptor_time.size();
  double mean_query_time =
      std::accumulate(querying_time.begin(), querying_time.end(), 0) * 1.0 /
      querying_time.size();
  double mean_update_time =
      std::accumulate(update_time.begin(), update_time.end(), 0) * 1.0 /
      update_time.size();
  std::cout << "Total submap number:" << pose_list.size()
            << ", triggle loop number:" << triggle_loop_num
            << ", true loop number:" << true_loop_num << std::endl;
  std::cout << "Mean time for descriptor extraction: " << mean_descriptor_time
            << "ms, query: " << mean_query_time
            << "ms, update: " << mean_update_time << "ms, total: "
            << mean_descriptor_time + mean_query_time + mean_update_time << "ms"
            << std::endl;
  std::cout << "True positives due to overlap: " << count_tp_ov << std::endl
            << "False positives due to overlap:" << count_fp_ov << std::endl
            << "True positives due to inliers: " << count_tp_in << std::endl
            << "False positives due to inliers:" << count_fp_in << std::endl

            << "True negatives: " << count_tn << std::endl
            << "False negatives:" << count_fn << std::endl
            << std::endl;
  return 0;
}