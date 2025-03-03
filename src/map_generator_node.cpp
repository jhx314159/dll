#include <ros/ros.h>
#include <string>
#include "grid3d.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <omp.h> // OpenMP for parallel processing

int main(int argc, char **argv)
{
	std::string node_name = "map_generator_node";
	std::string m_pclMapPath, m_octomapPath;
	double m_leafSize = 0.1;

	ros::init(argc, argv, node_name);  
	
	// Load parameters
	ros::NodeHandle lnh("~");
	if(!lnh.getParam("/dll_ns/pcl_map_path", m_pclMapPath))
		m_pclMapPath = "map.pcd";
	if(!lnh.getParam("/dll_ns/leaf_size", m_leafSize))
		m_leafSize = 0.1; // m_resolution = 0.05

	// Step 1: Load PCD file using PCL
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
	// 从指定的pcl文件中加载pcl点云数据到point_cloud中,
	// 成功加载: 返回非负数, 表示成功加载的点数. 
	// 失败: 返回负数, 通常是-1或其他错误代码, 表示加载失败. 
	if(
		(
		m_pclMapPath.compare(m_pclMapPath.length() - 4, 4, ".pcd") == 0 && 
		pcl::io::loadPCDFile(m_pclMapPath, *point_cloud) >= 0
		) || (
		m_pclMapPath.compare(m_pclMapPath.length() - 4, 4, ".ply") == 0 && 
		pcl::io::loadPLYFile(m_pclMapPath, *point_cloud) >= 0
		)
	)
	{
		std::cout << "point_cloud->size() = " << point_cloud->size() << std::endl;
		
		// pcl点云降采样
		pcl::VoxelGrid<pcl::PointXYZINormal> tmp_voxel_grid; 

		// 设置输入点云
		tmp_voxel_grid.setInputCloud(point_cloud);
		tmp_voxel_grid.setLeafSize(m_leafSize, m_leafSize, m_leafSize); 
		// voxel降采样并传入downsampled_cloud中
		tmp_voxel_grid.filter(*downsampled_cloud);
	}
	else 
	{
		std::cerr << "Error: Couldn't read file " << m_pclMapPath << std::endl;
		return -1;
	}

	std::cout << "leaf_size = " << m_leafSize << ", downsampled_cloud->size() = " << downsampled_cloud->size() << std::endl;

	// Step 2: Convert PCL point cloud to OctoMap point cloud
	octomap::Pointcloud octomap_cloud;
	#pragma omp parallel for // 使用 OpenMP 并行化
	for (const auto& point : downsampled_cloud->points) {
		// Ignore invalid points (e.g., NaN)
		if (!std::isnan(point.x) && !std::isnan(point.y) && !std::isnan(point.z)) {
			#pragma omp critical // 确保线程安全地访问 octomap_cloud
			octomap_cloud.push_back(point.x, point.y, point.z);
		}
	}
	std::cout << "Converted to OctoMap Pointcloud with " << octomap_cloud.size() << " points" << std::endl;

	// Step 3: Create an OctoMap octree

	octomap::OcTree tree(m_leafSize);

	// Step 4: Insert the point cloud into the octree
	octomap::point3d sensor_origin(0.0, 0.0, 0.0); // Assumes the sensor is at the origin
	tree.insertPointCloud(octomap_cloud, sensor_origin);
	std::cout << "Insert the point cloud into the octree" << std::endl;

	// Step 5: Save the octree to a .bt file
	m_octomapPath = m_pclMapPath.substr(0, m_pclMapPath.size() - 4) + "-" + std::to_string(m_leafSize) + ".bt";
	tree.updateInnerOccupancy(); // Update the occupancy probabilities of inner nodes
	tree.writeBinary(m_octomapPath);
	std::cout << "Saved OctoMap to " << m_octomapPath << std::endl;

	// Step 6: Convert OctoMap point cloud to Grid3d point cloud and save
	Grid3d pf(node_name, m_octomapPath);
	
	std::cout << "map_generator_node finished!" << std::endl;
	return 0;
}




