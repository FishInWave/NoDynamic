#ifndef KEYFRAME_HPP
#define KEYFRAME_HPP
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <vector>
#include <utility>

namespace xu_remove
{
    struct KeyFrame
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        using PointT = pcl::PointXYZI;
        using Ptr = std::shared_ptr<KeyFrame>;

        KeyFrame(Eigen::Matrix4f &odom1,Eigen::Matrix4f &odom2, const pcl::PointCloud<PointT>::ConstPtr &cloud, pcl::PointCloud<PointT>::Ptr &cloud_filtered ,std::vector<std::pair<pcl::PointCloud<PointT>::Ptr, Eigen::Vector4f>> &cluster_pairs);
        virtual ~KeyFrame();

    public:
        Eigen::Matrix4f odom1;
        Eigen::Matrix4f odom2;
        pcl::PointCloud<PointT>::ConstPtr cloud;
        pcl::PointCloud<PointT>::ConstPtr cloud_filtered;
        std::vector<std::pair<pcl::PointCloud<PointT>::Ptr, Eigen::Vector4f>> cluster_pairs;
    };

} // namespace xu_remove

#endif