#ifndef NEAREST_NEIGHBOR_LOOKUP_HPP
#define NEAREST_NEIGHBOR_LOOKUP_HPP

#include <iostream>
#include <limits>
#include <opencv2/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "util.hpp"

void get_grid(int num_grid,
              float margin_rate,
              const std::vector<cv::Vec3f>& model_vertices,
              pcl::PointCloud<pcl::PointXYZ>::Ptr& grid_vertices,
              std::vector<float>& distances_to_nearest)
{
    // lbn: left-bottom-near
    // rtf: right-top-far
    // these are corners of boudning box
    float inf = std::numeric_limits<float>::infinity();
    cv::Vec3f lbn_vertex = cv::Vec3f(+inf, +inf, +inf);
    cv::Vec3f rtf_vertex = cv::Vec3f(-inf, -inf, -inf);
    
    for(int i = 0; i < model_vertices.size(); ++i)
    {
        cv::Vec3f p = model_vertices[i];
        
        // compute the 3D bounding box of the model
        if (p[0] < lbn_vertex[0]) lbn_vertex[0] = p[0];
        if (p[1] < lbn_vertex[1]) lbn_vertex[1] = p[1];
        if (p[2] < lbn_vertex[2]) lbn_vertex[2] = p[2];
        if (p[0] > rtf_vertex[0]) rtf_vertex[0] = p[0];
        if (p[1] > rtf_vertex[1]) rtf_vertex[1] = p[1];
        if (p[2] > rtf_vertex[2]) rtf_vertex[2] = p[2];
    }

    // apply margin
    cv::Vec3f  bb_center = (rtf_vertex + lbn_vertex) / 2.0;
    lbn_vertex = bb_center + margin_rate * (lbn_vertex - bb_center);
    rtf_vertex = bb_center + margin_rate * (rtf_vertex - bb_center);

    cv::Vec3f grid_delta = (rtf_vertex - lbn_vertex) / num_grid;

    // convert model to pcl format
    pcl::PointCloud<pcl::PointXYZ>::Ptr model_clouds(new pcl::PointCloud<pcl::PointXYZ>);
    convert_cv2pcl(model_vertices, model_clouds);

    // create kdtree
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(model_clouds);
    int K = 1;

    // init return data 
    grid_vertices->width  = num_grid*num_grid*num_grid;
    grid_vertices->height = 1;
    grid_vertices->points.resize(grid_vertices->width * grid_vertices->height);
    distances_to_nearest.clear();

    int grid_idx = 0;

    for(int x_idx = 0; x_idx < num_grid; ++x_idx){
        for(int y_idx = 0; y_idx < num_grid; ++y_idx){
            for(int z_idx = 0; z_idx < num_grid; ++z_idx){
                pcl::PointXYZ searchPoint;
                searchPoint.x = lbn_vertex[0] + (x_idx + 0.5) * grid_delta[0];
                searchPoint.y = lbn_vertex[1] + (y_idx + 0.5) * grid_delta[1];
                searchPoint.z = lbn_vertex[2] + (z_idx + 0.5) * grid_delta[2];

                std::vector<int> pointIdxNKNSearch(K);
                std::vector<float> pointNKNSquaredDistance(K);

                kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);

                grid_vertices->points[grid_idx].x = searchPoint.x;
                grid_vertices->points[grid_idx].y = searchPoint.y;
                grid_vertices->points[grid_idx].z = searchPoint.z;

                distances_to_nearest.push_back(std::sqrt(pointNKNSquaredDistance[0]));

                ++grid_idx;
            }
        }
    }
}

#endif /* NEAREST_NEIGHBOR_LOOKUP_HPP */