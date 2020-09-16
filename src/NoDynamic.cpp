/**
 * @file NoDynamic.cpp
 * @author Yu-wei XU
 * @brief 从关键帧中删除动态物体
 * @version 0.1
 * @date 2020-09-13
 * 
 * @copyright Copyright (c) 2020
 * 
 */
//pcl
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid_covariance.h>
#include <pcl/console/time.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/common/centroid.h>
//std
#include <cstdio>
#include <fstream>
#include <iostream>
#include <dirent.h>
#include <sys/types.h>
#include <algorithm>
#include <utility>
//third party
#include "yaml-cpp/yaml.h"
#include <pclomp/ndt_omp.h>
#include <pclomp/gicp_omp.h>
//our
#include "keyframe.hpp"
int user_data;
using namespace std;
using namespace xu_remove;
using PointT = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<PointT>;

//变量
float voxel_leafsize = 0.1;
float ceiling_height = 2.3;
float floor_height = 0.2;
string registration_method;
Eigen::Matrix4f keyframe_pose;
Eigen::Matrix4f prev_trans;
//NDT
float ndt_resolution = 1.0;
float ndt_epsilon = 0.1;
int ndt_maximum_iterations = 64;
string ndt_search_method = "DIRECT7";
//GICP
float gicp_resolution = 0.1;
float gicp_epsilon = 0.1;
int gicp_maximum_iterations = 64;
bool gicp_use_reciprocal_correspondences = false;
int gicp_correspondence_randomness = 20;
int gicp_max_optimizer_iterations = 20;
//cluster
float cluster_tolerance = 0.25;
int min_cluster_size = 40;
int max_cluster_size = 100;

PointCloud::Ptr prev_cloud(new PointCloud);

/**
 * @brief 导入yaml文件，并赋值相关变量。
 */
void parseYamlFile()
{
    YAML::Node node = YAML::LoadFile("./config.yaml");
    //滤波
    voxel_leafsize = node["voxel"]["leafsize"].as<float>();
    ceiling_height = node["passthrough"]["ceiling_height"].as<float>();
    floor_height = node["passthrough"]["floor_height"].as<float>();
    //配准
    registration_method = node["registration_method"].as<string>();
    //NDT
    ndt_resolution = node["NDT_OMP"]["resolution"].as<float>();
    ndt_search_method = node["NDT_OMP"]["search_method"].as<string>();
    ndt_epsilon = node["NDT_OMP"]["epsilon"].as<float>();
    ndt_maximum_iterations = node["NDT_OMP"]["maximum_iterations"].as<int>();
    //GICP
    gicp_resolution = node["GICP_OMP"]["resolution"].as<float>();
    gicp_epsilon = node["GICP_OMP"]["epsilon"].as<float>();
    gicp_maximum_iterations = node["GICP_OMP"]["maximum_iterations"].as<int>();
    gicp_use_reciprocal_correspondences = node["GICP_OMP"]["use_reciprocal_correspondences"].as<bool>();
    gicp_correspondence_randomness = node["GICP_OMP"]["correspondence_randomness"].as<int>();
    gicp_max_optimizer_iterations = node["GICP_OMP"]["max_optimizer_iterations"].as<int>();
    //cluster
    cluster_tolerance = node["cluster"]["cluster_tolerance"].as<float>();
    min_cluster_size = node["cluster"]["min_cluster_size"].as<int>();
    max_cluster_size = node["cluster"]["max_cluster_size"].as<int>();
}
/**
 * @brief 包含了高程滤波，离散点滤波
 * 
 * @param input 
 * @param output 
 */
void filters(PointCloud::Ptr &input, PointCloud::Ptr &output)
{
    //高程滤波
    pcl::PassThrough<PointT> pf(false); //false表示不想管被删除的索引
    pf.setInputCloud(input);
    pf.setFilterFieldName("z");
    pf.setFilterLimits(floor_height, ceiling_height); //只保留小车高度内的点
    PointCloud::Ptr cloud_pf(new PointCloud);
    pf.filter(*cloud_pf);
    //统计学滤波
    pcl::StatisticalOutlierRemoval<PointT> sf;
    sf.setMeanK(25);
    sf.setStddevMulThresh(0.5);
    sf.setInputCloud(cloud_pf);
    PointCloud::Ptr cloud_spf(new PointCloud);
    sf.filter(*output);
    //体素化,目的是为了建立索引
    // pcl::VoxelGrid<PointT> vf;
    // vf.setInputCloud(cloud_spf);
    // vf.setLeafSize(leafsize, leafsize, leafsize);
    // vf.setSaveLeafLayout(true);
    // vf.filter(*output);
}

/**
 * @brief Get the File Names object
 * 
 * @param path 
 * @param filenames 
 */
void getFileNames(std::string &path, std::vector<std::string> &filenames)
{
    DIR *pDir;
    struct dirent *ptr;
    if (!(pDir = opendir(path.c_str())))
        return;
    while ((ptr = readdir(pDir)) != 0)
    {
        if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0)
            filenames.push_back(path + "/" + ptr->d_name);
    }
    closedir(pDir);
    //TODO:用lamnda表达式处理排序
    int num = filenames.size();
    filenames.clear();
    for (int i = 0; i < num; i++)
    {
        string temp = path + "/key_frame_" + to_string(i) + ".pcd";
        filenames.push_back(temp);
    }
}
/**
 * @brief 初始化配准器
 * 
 * @return pcl::Registration<PointT, PointT>::Ptr 
 */
pcl::Registration<PointT, PointT>::Ptr initRegistration()
{
    if (registration_method == "NDT_OMP")
    {
        int num_threads = 0;
        boost::shared_ptr<pclomp::NormalDistributionsTransform<PointT, PointT>> ndt(new pclomp::NormalDistributionsTransform<PointT, PointT>());
        if (num_threads > 0)
        {
            ndt->setNumThreads(num_threads);
        }
        ndt->setTransformationEpsilon(ndt_epsilon);
        ndt->setMaximumIterations(ndt_maximum_iterations);
        ndt->setResolution(ndt_resolution);
        if (ndt_search_method == "KDTREE")
        {
            ndt->setNeighborhoodSearchMethod(pclomp::KDTREE);
        }
        else if (ndt_search_method == "DIRECT1")
        {
            ndt->setNeighborhoodSearchMethod(pclomp::DIRECT1);
        }
        else
        {
            ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
        }
        return ndt;
    }
    else if (registration_method == "GICP_OMP")
    {
        boost::shared_ptr<pclomp::GeneralizedIterativeClosestPoint<PointT, PointT>> gicp(new pclomp::GeneralizedIterativeClosestPoint<PointT, PointT>());
        gicp->setTransformationEpsilon(gicp_epsilon);
        gicp->setMaximumIterations(gicp_maximum_iterations);
        gicp->setUseReciprocalCorrespondences(gicp_use_reciprocal_correspondences);
        gicp->setCorrespondenceRandomness(gicp_correspondence_randomness);
        gicp->setMaximumOptimizerIterations(gicp_max_optimizer_iterations);

        return gicp;
    }
    else if (registration_method == "NDT")
    {
        cout << "NDT" << endl;
        boost::shared_ptr<pcl::NormalDistributionsTransform<PointT, PointT>> ndt(new pcl::NormalDistributionsTransform<PointT, PointT>());
        ndt->setTransformationEpsilon(ndt_epsilon);
        ndt->setMaximumIterations(ndt_maximum_iterations);
        ndt->setResolution(ndt_resolution);
        cout << "NDT finish" << endl;
        return ndt;
    }
}
/**
 * @brief 用传入的注册器配准两个点云，并返回odom
 * 
 * @param registration 注册器
 * @param target_cloud 目标点云
 * @param source_cloud 源点云
 * @return Eigen::Matrix4f odom
 */
Eigen::Matrix4f matching(pcl::Registration<PointT, PointT>::Ptr registration, const PointCloud::Ptr &target_cloud, const PointCloud::Ptr &source_cloud)
{
    static int align_count = 0;
    if (!align_count)
    {
        prev_trans.setIdentity();
        keyframe_pose.setIdentity();
    }
    registration->setInputTarget(target_cloud);
    registration->setInputSource(source_cloud);
    PointCloud::Ptr aligned(new PointCloud());
    // registration->align(*aligned, prev_trans.matrix());
    registration->align(*aligned);
    if (!registration->hasConverged())
    {
        PCL_WARN("scan matching has not converged!!");
        string warning_msgs = "ignore this frame: " + to_string(align_count);
        cout << warning_msgs << endl;
        return keyframe_pose * prev_trans;
    }
    Eigen::Matrix4f trans = registration->getFinalTransformation();
    Eigen::Matrix4f odom = keyframe_pose * trans;
    cout << align_count << ":" << registration->getFitnessScore() << endl;
    prev_trans = trans;
    keyframe_pose = odom;
    align_count++;
    return odom;
}

/**
 * @brief 对输入点云进行聚类，提取聚类点云另外保存，并计算聚类质心坐标。
 * 
 * @param cloud 
 * @param cluster_pairs 聚类点云和质心坐标的pair集合
 * @return int 点云中的聚类个数
 */
int cluster(PointCloud::Ptr &cloud, vector<pair<PointCloud::Ptr, Eigen::Vector4f>> &cluster_pairs)
{
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    vector<pcl::PointIndices> cluster_indices;
    tree->setInputCloud(cloud);
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(cluster_tolerance);
    ec.setMinClusterSize(min_cluster_size);
    ec.setMaxClusterSize(max_cluster_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);
    for (auto it = cluster_indices.begin(); it != cluster_indices.end(); it++)
    {
        PointCloud::Ptr cloud_cluster(new PointCloud);
        for (auto pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            cloud_cluster->push_back((*cloud)[*pit]);
        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        Eigen::Vector4f centroid;
        unsigned int p_num = pcl::compute3DCentroid(*cloud_cluster, centroid);
        pair<PointCloud::Ptr, Eigen::Vector4f> cluster_pair = make_pair(cloud_cluster, centroid);
        cluster_pairs.push_back(cluster_pair);
    }
    cout << "use indiches: " << *(cloud->begin()+cluster_indices.begin()->indices.front()) << endl;
    cout << "use subscript: " << cloud->points.at(cluster_indices.begin()->indices.front()) << endl;
    return cluster_indices.size();
}
/**
 * @brief 向一个文件流写入odom
 * 
 * @param ofs 
 * @param odom 
 * @return true  
 * @return false 
 */
bool saveOdom(std::ofstream &ofs, const Eigen::Matrix4f &odom)
{
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            ofs << odom(i, j);

            if (i == 2 && j == 3)
            {
                ofs << std::endl;
            }
            else
            {
                ofs << " ";
            }
        }
    }

    return true;
}
/**
 * @brief Create a File object
 * 
 * @param ofs 
 * @param file_path 
 * @return true 
 * @return false 
 */
bool createFile(std::ofstream &ofs, std::string file_path)
{
    ofs.close();
    boost::filesystem::remove(file_path.c_str());

    ofs.open(file_path.c_str(), std::ios::out);
    if (!ofs)
    {
        cout << "无法生成文件： " << file_path << endl;
        return false;
    }

    return true;
}

int getClusterCentroid(vector<pcl::PointIndices> &cluster_indices, vector<pair<pcl::PointIndices, Eigen::Vector3f>> &cluster_pair)
{
}

int main(int argc, char **argv)
{
    std::string file_path;
    if (argc >= 2)
    {
        file_path = argv[1];
    }
    else
    {
        PCL_WARN("You must add a pcd path!\n");
        return 0;
    }

    parseYamlFile();
    //将关键帧统一存入vector
    vector<KeyFrame> keyframes;

    vector<string> filenames;
    getFileNames(file_path, filenames);
    pcl::Registration<PointT, PointT>::Ptr registration = initRegistration();

    PointCloud::Ptr cloud20(new PointCloud);
    PointCloud::Ptr cloud21(new PointCloud);
    pcl::io::loadPCDFile(filenames.at(20), *cloud20);
    pcl::io::loadPCDFile(filenames.at(21), *cloud21);
    matching(registration, cloud20, cloud21);
    cout << "20: " << registration->getFinalTransformation() << endl;
    for (auto file : filenames)
    {

        PointCloud::Ptr cloud(new PointCloud);
        pcl::io::loadPCDFile(file, *cloud);
        PointCloud::Ptr cloud_filtered(new PointCloud);
        filters(cloud, cloud_filtered);
        Eigen::Matrix4f odom1;
        Eigen::Matrix4f odom2(Eigen::Matrix4f::Identity());
        if (file == filenames.front())
        {
            odom1.setIdentity();
            cout << odom1 << endl;
        }
        else
        {
            odom1 = matching(registration, prev_cloud, cloud);
        }
        prev_cloud = cloud;
        vector<pair<PointCloud::Ptr, Eigen::Vector4f>> cluster_pairs;
        cluster(cloud_filtered, cluster_pairs);
        KeyFrame keyframe(odom1, odom2, cloud, cloud_filtered, cluster_pairs);
        keyframes.push_back(keyframe);
    }
    std::ofstream ofs;
    createFile(ofs, file_path + "origin_odom.txt");
    for (auto keyframe : keyframes)
    {
        saveOdom(ofs, keyframe.odom1);
    }
    ofs.close();
    cout << "have already finished" << endl;
    //TODO:动态物体检测算法

    return 0;
}
