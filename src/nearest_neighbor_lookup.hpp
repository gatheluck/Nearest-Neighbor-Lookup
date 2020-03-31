#ifndef NEAREST_NEIGHBOR_LOOKUP_HPP
#define NEAREST_NEIGHBOR_LOOKUP_HPP

#include <iostream>
#include <limits>
#include <opencv2/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

void get_lookup_table(int num_grid,
					  float margin_rate,
					  const std::vector<cv::Vec3f>& model_vertices,
					  pcl::KdTreeFLANN<pcl::PointXYZ>& lookup_table_vertices)
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
	cv::Vec3f bb_center = (rtf_vertex + lbn_vertex) / 2.0;
	lbn_vertex = bb_center + margin_rate * (lbn_vertex - bb_center);
	rtf_vertex = bb_center + margin_rate * (rtf_vertex - bb_center);

	cv::Vec3f grid_delta = (rtf_vertex - lbn_vertex) / num_grid;

	int K = 1;
	std::vector<cv::Vec3f> grid_vertices;
	std::vector<float> grid_distances;


	for(int x_idx = 0; x_idx < num_grid; ++x_index){
		for(int y_idx = 0; y_idx < num_grid; ++y_index){
			for(int z_idx = 0; z_idx < num_grid; ++z_index){
				pcl::PointXYZ searchPoint;
				searchPoint.x = lbn_vertex.x + (x_idx + 0.5)*grid_delta.x
				searchPoint.y = lbn_vertex.y + (y_idx + 0.5)*grid_delta.y
				searchPoint.z = lbn_vertex.z + (z_idx + 0.5)*grid_delta.z



				kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
			}
		}
	}
}

#endif /* NEAREST_NEIGHBOR_LOOKUP_HPP */