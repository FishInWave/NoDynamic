#include <keyframe.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace xu_remove
{
    KeyFrame::KeyFrame(Eigen::Matrix4f &odom1,Eigen::Matrix4f &odom2, 
    const pcl::PointCloud<PointT>::ConstPtr &cloud, 
    pcl::PointCloud<PointT>::Ptr &cloud_filtered ,
    std::vector<std::pair<pcl::PointCloud<PointT>::Ptr, Eigen::Vector4f>> &cluster_pairs):odom1(odom1),odom2(odom2),cloud(cloud),
    cloud_filtered(cloud_filtered),cluster_pairs(cluster_pairs)  {}
    KeyFrame::~KeyFrame() {}
} // namespace xu_remove