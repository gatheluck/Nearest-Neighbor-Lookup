#ifndef UTIL_HPP
#define UTIL_HPP

#include <iostream>
#include <limits>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/mesh.h>
#include <assimp/postprocess.h>
#include <opencv2/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

void load_obj(const std::string obj_path,
			  cv::Vec3f& lbn_vertex,
			  cv::Vec3f& rtf_vertex,
			  std::vector<cv::Vec3f>& vertices,
			  std::vector<cv::Vec3f>& normals,
			  std::vector<int>& indices)
{
	Assimp::Importer importer;

	const aiScene* scene = importer.ReadFile(obj_path, aiProcessPreset_TargetRealtime_Fast);
    
    aiMesh *mesh = scene->mMeshes[0];
    bool has_normals  = mesh->HasNormals();

	// lbn: left-bottom-near
	// rtf: right-top-far
	// these are corners of boudning box
	float inf = std::numeric_limits<float>::infinity();
    lbn_vertex = cv::Vec3f(inf, inf, inf);
    rtf_vertex = cv::Vec3f(-inf, -inf, -inf);

	// load face
	for(int i = 0; i < mesh->mNumFaces; i++)
    {
        aiFace f = mesh->mFaces[i];
        
        indices.push_back(f.mIndices[0]);
        indices.push_back(f.mIndices[1]);
        indices.push_back(f.mIndices[2]);
    }

	// load vertices
	for(int i = 0; i < mesh->mNumVertices; i++)
    {
        aiVector3D v = mesh->mVertices[i];
        cv::Vec3f p(v.x, v.y, v.z);
        
        // compute the 3D bounding box of the model
        if (p[0] < lbn_vertex[0]) lbn_vertex[0] = p[0];
        if (p[1] < lbn_vertex[1]) lbn_vertex[1] = p[1];
        if (p[2] < lbn_vertex[2]) lbn_vertex[2] = p[2];
        if (p[0] > rtf_vertex[0]) rtf_vertex[0] = p[0];
        if (p[1] > rtf_vertex[1]) rtf_vertex[1] = p[1];
        if (p[2] > rtf_vertex[2]) rtf_vertex[2] = p[2];
        
        vertices.push_back(p);
    }

	// load normals
	if(has_normals)
    {
        for(int i = 0; i < mesh->mNumVertices; i++)
        {
            aiVector3D n = mesh->mNormals[i];
            cv::Vec3f vn = cv::Vec3f(n.x, n.y, n.z);
            
            normals.push_back(vn);
        }
    }
}

void convert_cv2pcl(const std::vector<cv::Vec3f>& vertices, pcl::PointCloud<pcl::PointXYZ>::Ptr& clouds)
{
	clouds->width  = vertices.size();
	clouds->height = 1;
	clouds->points.resize(clouds->width * clouds->height);

	for (std::size_t i=0; i<clouds->points.size(); ++i)
	{
		clouds->points[i].x = vertices[i][0];
		clouds->points[i].y = vertices[i][1];
		clouds->points[i].z = vertices[i][2];
	}
}

#endif /* UTIL_HPP */