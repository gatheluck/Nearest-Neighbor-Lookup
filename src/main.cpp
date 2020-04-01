#include <iostream>
#include <opencv2/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "util.hpp"
#include "nearest_neighbor_lookup.hpp"

int main()
{
    // load obj
    const std::string obj_path = "../data/bunny2.obj";

    cv::Vec3f lbn_vertex;
    cv::Vec3f rtf_vertex;
    std::vector<cv::Vec3f> vertices;
    std::vector<cv::Vec3f> normals;
    std::vector<int> indices;

    load_obj(obj_path, lbn_vertex, rtf_vertex, vertices, normals, indices);

    std::cout << "lbn_vertices: " << lbn_vertex << std::endl;
    std::cout << "rtf_vertices: " << rtf_vertex << std::endl;
    std::cout << "norm: " << cv::norm(rtf_vertex - lbn_vertex) << std::endl;
    std::cout << "vertices: " << vertices.size() << std::endl;
    std::cout << "normals: " << normals.size() << std::endl;
    std::cout << "indices: " << indices.size() << std::endl;

    // convert to pcl
    pcl::PointCloud<pcl::PointXYZ>::Ptr clouds(new pcl::PointCloud<pcl::PointXYZ>);
    convert_cv2pcl(vertices, clouds);

    std::cout << "points: " << clouds->points[0].x << std::endl;
    std::cout << "points: " << clouds->points[0].y << std::endl;
    std::cout << "points: " << clouds->points[0].z << std::endl;

    std::cout << "vertice: " << vertices[0][0] << std::endl;
    std::cout << "vertice: " << vertices[0][1] << std::endl;
    std::cout << "vertice: " << vertices[0][2] << std::endl;

    int num_grid = 10;
    float margin_rate = 1.2;
    pcl::PointCloud<pcl::PointXYZ>::Ptr grid_vertices(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<float> distances_to_nearest;
    get_grid(num_grid, margin_rate, vertices, grid_vertices, distances_to_nearest);

    std::cout << "grid_vertices: " << grid_vertices->points[0].x << std::endl;
    std::cout << "grid_vertices: " << grid_vertices->points[0].y << std::endl;
    std::cout << "grid_vertices: " << grid_vertices->points[0].z << std::endl;

    std::cout << "distance_to_nearest: " << distances_to_nearest[0] << std::endl;
    std::cout << "distance_to_nearest: " << distances_to_nearest.size() << std::endl;

}