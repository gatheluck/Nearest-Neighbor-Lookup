#include <iostream>
#include <opencv2/core.hpp>
#include "util.hpp"


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

    std::cout << "lbn_vertices: " << lbn_vertex  << std::endl;
    std::cout << "rtf_vertices: " << rtf_vertex  << std::endl;
    std::cout << "vertices: " << vertices.size()  << std::endl;
    std::cout << "normals: " << normals.size()  << std::endl;
    std::cout << "indices: " << indices.size()  << std::endl;


    //  
}