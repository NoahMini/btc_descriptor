#include <fstream>
#include <iostream>
#include <pcl/io/pcd_io.h>

#include <pcl/point_types.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/kdtree/kdtree_flann.h>

struct AlignmentResult {
    Eigen::Matrix4f transformation;
    int inliers;
    float fitness_score;
};

AlignmentResult computeCloudTransform(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& target,
    float max_correspondence_distance = 0.05f) 
{
    AlignmentResult result;
    // Configuración de ICP
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(source);
    icp.setInputTarget(target);
    icp.setMaxCorrespondenceDistance(max_correspondence_distance);
    icp.setMaximumIterations(100);
    std::cout << "Fine after setmaxiterations" << std::endl;
    // Ejecutar alineación
    pcl::PointCloud<pcl::PointXYZ> final_cloud;
    icp.align(final_cloud);

    std::cout << "Fine after align" << std::endl;

    if (icp.hasConverged ())
    {
        std::cout << "got into converged" << std::endl;
        // Obtener resultados
        result.transformation = icp.getFinalTransformation();
        result.fitness_score = icp.getFitnessScore();
        // Calcular inliers usando KD-Tree
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(target);
        int inliers_count = 0;
        for (const auto& point : final_cloud) {
            std::vector<int> indices(1);
            std::vector<float> distances(1);
            if (kdtree.nearestKSearch(point, 1, indices, distances) > 0) {
                if (distances[0] <= max_correspondence_distance) {
                    inliers_count++;
                }
            }
        }

        result.inliers = inliers_count;
    }
    else
    {
        std::cout << "didnt converge" << std::endl;
        result.inliers = 0;
        std::cout << "still fine: result inliers equals " << result.inliers << std::endl;
    }
    
    std::cout << "ready to return: inliers = " << result.inliers << std::endl;
    return result;
}

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

    for (unsigned one_id = 0 ; one_id < 10 ; one_id++){
        for (unsigned two_id = 0 ; two_id < 10 ; two_id++){
            pcl::PointCloud<pcl::PointXYZ>::Ptr qtransform_cloud(new pcl::PointCloud<pcl::PointXYZ>());

            std::stringstream qss;
            qss << "/home/noah/tfm/images/KITTI/00g/velodyne" << "/" << std::setfill('0') << std::setw(6) << one_id
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
            tss << "/home/noah/tfm/images/KITTI/00g/velodyne" << "/" << std::setfill('0') << std::setw(6) << two_id
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

            AlignmentResult cloud_result = computeCloudTransform(qtransform_cloud, ttransform_cloud);
            std::cout << "got out" << std::endl;
            inliers = 0;
            }
        }
    }
    for (unsigned one_id = 250 ; one_id < 260 ; one_id++){
        pcl::PointCloud<pcl::PointXYZ>::Ptr qtransform_cloud(new pcl::PointCloud<pcl::PointXYZ>());

        std::stringstream qss;
        qss << "/home/noah/tfm/images/KITTI/00g/velodyne" << "/" << std::setfill('0') << std::setw(6) << one_id
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
        tss << "/home/noah/tfm/images/KITTI/00g/velodyne" << "/" << std::setfill('0') << std::setw(6) << one_id - 240
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

        // ibow_lcd::computeCloudTransform(qtransform_cloud, ttransform_cloud, inliers);
        std::cout << "got out" << std::endl;
        }
    }

    return 0;
}