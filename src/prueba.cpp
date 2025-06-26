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

      // lcdet.process(submap_id, kps, dscs, pcds_dir, search_result);

      // Check for inliers
      if (search_result.second == 1){
        pcl::PointCloud<pcl::PointXYZ>::Ptr qtransform_cloud(new pcl::PointCloud<pcl::PointXYZ>());

        std::stringstream qss;
        qss << "/home/noah/tfm/images/KITTI/00g/velodyne" << "/" << std::setfill('0') << std::setw(6) << submap_id
            << ".bin";
        
        std::cout << qss.str() << std::endl;
        std::vector<float> lidar_data = read_lidar_data(qss.str());

        for (std::size_t i = 0; i < lidar_data.size(); i += 4) {
        pcl::PointXYZ point;
        point.x = lidar_data[i];
        point.y = lidar_data[i + 1];
        point.z = lidar_data[i + 2];
        qtransform_cloud->points.push_back(point);
        }
        std::cout << "Read image_id fine" << std::endl;

        pcl::PointCloud<pcl::PointXYZ>::Ptr ttransform_cloud(new pcl::PointCloud<pcl::PointXYZ>());

        std::stringstream tss;
        tss << "/home/noah/tfm/images/KITTI/00g/velodyne" << "/" << std::setfill('0') << std::setw(6) << search_result.first
            << ".bin";

        std::cout << tss.str() << std::endl;
            lidar_data = read_lidar_data(tss.str());

        for (std::size_t i = 0; i < lidar_data.size(); i += 4) {
        pcl::PointXYZ point;
        point.x = lidar_data[i];
        point.y = lidar_data[i + 1];
        point.z = lidar_data[i + 2];
        ttransform_cloud->points.push_back(point);
        }
        std::cout << "Read best_img fine" << std::endl;

        unsigned inliers = 0;
        if (!qtransform_cloud->empty() && !ttransform_cloud->empty()){
        
        std::cout << "Query cloud size: " << qtransform_cloud->points.size() << std::endl;

        std::cout << "Train cloud size: " << ttransform_cloud->points.size() << std::endl;

        // ibow_lcd::AlignmentResult result = ibow_lcd::computeCloudTransform(qtransform_cloud, ttransform_cloud);
        // std::cout << "got out with inliers: " << result.inliers << std::endl;
        }

      }
      // lcdet.consecutive_loops_++;
      // std::cout << "Consecutive loops: " << lcdet.consecutive_loops_ << std::endl;

      std::cout << "[Loop Detection] triggle loop: " << submap_id << "--"
                  << search_result.first << ", score:" << search_result.second
                  << std::endl << std::endl;
    
      // // down sample to save memory
      // down_sampling_voxel(transform_cloud, 0.5);
      // btc_manager->key_cloud_vec_.push_back(transform_cloud.makeShared());

      // // visulization                                                                     DONT WORRY ABOUT THIS 
      // sensor_msgs::PointCloud2 pub_cloud;
      // pcl::toROSMsg(transform_cloud, pub_cloud);
      // pub_cloud.header.frame_id = "camera_init";
      // pubCureentCloud.publish(pub_cloud);

      // pcl::PointCloud<pcl::PointXYZ> key_points_cloud;
      // for (auto var : btc_manager->history_binary_list_.back()) {
      //   pcl::PointXYZ pi;
      //   pi.x = var.location_[0];
      //   pi.y = var.location_[1];
      //   pi.z = var.location_[2];
      //   key_points_cloud.push_back(pi);
      // }
      // pcl::toROSMsg(key_points_cloud, pub_cloud);
      // pub_cloud.header.frame_id = "camera_init";
      // pubCurrentBinary.publish(pub_cloud);

      // visualization_msgs::MarkerArray marker_array;
      // visualization_msgs::Marker marker;
      // marker.header.frame_id = "camera_init";
      // marker.ns = "colored_path";
      // marker.id = submap_id;
      // marker.type = visualization_msgs::Marker::LINE_LIST;
      // marker.action = visualization_msgs::Marker::ADD;
      // marker.pose.orientation.w = 1.0;
      // if (search_result.first >= 0) {                                 //loop found (can be false positive)
      //   triggle_loop_num++;
      //   //Eigen::Matrix4d transform1 = Eigen::Matrix4d::Identity();
      //   //Eigen::Matrix4d transform2 = Eigen::Matrix4d::Identity();
      //   // publish_std(loop_std_pair, transform1, transform2, pubBTC);
      //   slow_loop.sleep();
      //   // double cloud_overlap =
      //   //     calc_overlap(transform_cloud.makeShared(),
      //   //                  btc_manager->key_cloud_vec_[search_result.first], 0.5);
      //   int loop_match = loop_mat[submap_id][search_result.first - 2] + loop_mat[submap_id][search_result.first - 1] + 
      //                     loop_mat[submap_id][search_result.first] + loop_mat[submap_id][search_result.first + 1] +
      //                     loop_mat[submap_id][search_result.first + 2];
      //   pcl::PointCloud<pcl::PointXYZ> match_key_points_cloud;
      //   for (auto var :
      //        btc_manager->history_binary_list_[search_result.first]) {
      //     pcl::PointXYZ pi;
      //     pi.x = var.location_[0];
      //     pi.y = var.location_[1];
      //     pi.z = var.location_[2];
      //     match_key_points_cloud.push_back(pi);
      //   }
      //   pcl::toROSMsg(match_key_points_cloud, pub_cloud);
      //   pub_cloud.header.frame_id = "camera_init";
      //   pubMatchedBinary.publish(pub_cloud);
      //   // true positive
      //   if (loop_match >= 1) {
      //     true_loop_num++;
      //     if (search_result.second > 0){count_tp_in++;}else{count_tp_ov++;}             //CHECK ORIGIN OF TRUE POSITIVE
          
      //     pcl::PointCloud<pcl::PointXYZRGB> matched_cloud;                              //THIS IS FOR VISUALIZATION DON TOUCH IT
      //     matched_cloud.resize(
      //         btc_manager->key_cloud_vec_[search_result.first]->size());
      //     for (size_t i = 0;
      //          i < btc_manager->key_cloud_vec_[search_result.first]->size();
      //          i++) {
      //       pcl::PointXYZRGB pi;
      //       pi.x =
      //           btc_manager->key_cloud_vec_[search_result.first]->points[i].x;
      //       pi.y =
      //           btc_manager->key_cloud_vec_[search_result.first]->points[i].y;
      //       pi.z =
      //           btc_manager->key_cloud_vec_[search_result.first]->points[i].z;
      //       pi.r = 0;
      //       pi.g = 255;
      //       pi.b = 0;
      //       matched_cloud.points[i] = pi;
      //     }
          
      //     pcl::toROSMsg(matched_cloud, pub_cloud);
      //     pub_cloud.header.frame_id = "camera_init";
      //     pubMatchedCloud.publish(pub_cloud);
      //     slow_loop.sleep();

      //     marker.scale.x = scale_tp;
      //     marker.color = color_tp;
      //     geometry_msgs::Point point1;
      //     point1.x = pose_list[submap_id - 1].first[0];
      //     point1.y = pose_list[submap_id - 1].first[1];
      //     point1.z = pose_list[submap_id - 1].first[2];
      //     geometry_msgs::Point point2;
      //     point2.x = pose_list[submap_id].first[0];
      //     point2.y = pose_list[submap_id].first[1];
      //     point2.z = pose_list[submap_id].first[2];
      //     marker.points.push_back(point1);
      //     marker.points.push_back(point2);

      //   } else {
      //     if (search_result.second > 0){count_fp_in++;}else{count_fp_ov++;}             //CHECK SOURCE OF FALSE POSITIVE
          
      //     pcl::PointCloud<pcl::PointXYZRGB> matched_cloud;                              //THIS IS FOR VISUALIZATION DON TOUCH IT
      //     matched_cloud.resize(
      //         btc_manager->key_cloud_vec_[search_result.first]->size());
      //     for (size_t i = 0;
      //          i < btc_manager->key_cloud_vec_[search_result.first]->size();
      //          i++) {
      //       pcl::PointXYZRGB pi;
      //       pi.x =
      //           btc_manager->key_cloud_vec_[search_result.first]->points[i].x;
      //       pi.y =
      //           btc_manager->key_cloud_vec_[search_result.first]->points[i].y;
      //       pi.z =
      //           btc_manager->key_cloud_vec_[search_result.first]->points[i].z;
      //       pi.r = 255;
      //       pi.g = 0;
      //       pi.b = 0;
      //       matched_cloud.points[i] = pi;
      //     }
          
      //     pcl::toROSMsg(matched_cloud, pub_cloud);
      //     pub_cloud.header.frame_id = "camera_init";
      //     pubMatchedCloud.publish(pub_cloud);
      //     slow_loop.sleep();
      //     marker.scale.x = scale_fp;
      //     marker.color = color_fp;
      //     geometry_msgs::Point point1;
      //     point1.x = pose_list[submap_id - 1].first[0];
      //     point1.y = pose_list[submap_id - 1].first[1];
      //     point1.z = pose_list[submap_id - 1].first[2];
      //     geometry_msgs::Point point2;
      //     point2.x = pose_list[submap_id].first[0];
      //     point2.y = pose_list[submap_id].first[1];
      //     point2.z = pose_list[submap_id].first[2];
      //     marker.points.push_back(point1);
      //     marker.points.push_back(point2);
      //   }

      // } else {                                                      //loop not found
      //   slow_loop.sleep();
      //   if (submap_id > 0) {
      //     if (loop_sum[submap_id] <= 5){
      //       count_tn++;
      //       marker.scale.x = scale_tn;
      //       marker.color = color_tn;
      //     }else{
      //       count_fn++;
      //       marker.scale.x = scale_fn;
      //       marker.color = color_fn;
      //     }
      //     geometry_msgs::Point point1;
      //     point1.x = pose_list[submap_id - 1].first[0];
      //     point1.y = pose_list[submap_id - 1].first[1];
      //     point1.z = pose_list[submap_id - 1].first[2];
      //     geometry_msgs::Point point2;
      //     point2.x = pose_list[submap_id].first[0];
      //     point2.y = pose_list[submap_id].first[1];
      //     point2.z = pose_list[submap_id].first[2];
      //     marker.points.push_back(point1);
      //     marker.points.push_back(point2);
      //   }
      // }
      // marker_array.markers.push_back(marker);
      // pubLoopStatus.publish(marker_array);
      // loop.sleep();
    }
    finish = true;
  }
}