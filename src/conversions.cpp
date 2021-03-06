/*
 * UOS-ROS packages - Robot Operating System code by the University of Osnabrück
 * Copyright (C) 2013 University of Osnabrück
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * created on: 30.04.2014
 *
 * conversions.cpp
 *
 * Author: Sebastian Pütz <spuetz@uos.de>,
 *         Henning Deeken <hdeeken@uos.de>,
 *         Marcel Mrozinski <mmrozins@uos.de>,
 *         Tristan Igelbrink <tigelbri@uos.de>
 *
 */

#include "lvr_ros/conversions.h"
#include "lvr_ros/colors.h"
#include <cmath>

namespace lvr_ros
{

  bool fromMeshBufferToTriangleMesh(const lvr::MeshBufferPtr& buffer, mesh_msgs::TriangleMesh& mesh)
  {
    return fromMeshBufferToTriangleMesh(*buffer, mesh);
  }

  bool fromMeshBufferToTriangleMesh(lvr::MeshBuffer& buffer, mesh_msgs::TriangleMesh& mesh)
  {
    size_t numVertices = 0;
    size_t numFaces = 0;
    size_t numNormals = 0;
    lvr::coord3fArr verticesArray = buffer.getIndexedVertexArray(numVertices);
    lvr::coord3fArr normalsArray = buffer.getIndexedVertexNormalArray(numNormals);
    lvr::uintArr facesArray = buffer.getFaceArray(numFaces);

    ROS_DEBUG_STREAM("number vertices: " << numVertices);
    ROS_DEBUG_STREAM("number triangles: " << numFaces);
    ROS_DEBUG_STREAM("number normals: " << numNormals);


    mesh.vertices.resize(numVertices);
    mesh.vertex_normals.resize(numNormals);
    mesh.triangles.resize(numFaces);

    if (numVertices && numFaces)
    {
      // copy vertices
      for (unsigned int i = 0; i < numVertices; i++)
      {
        mesh.vertices[i].x = verticesArray[i].x;
        mesh.vertices[i].y = verticesArray[i].y;
        mesh.vertices[i].z = verticesArray[i].z;
      }

      // copy triangles
      for (unsigned int i = 0; i < numFaces; i++)
      {
        mesh.triangles[i].vertex_indices[0] = facesArray[i * 3];
        mesh.triangles[i].vertex_indices[1] = facesArray[i * 3 + 1];
        mesh.triangles[i].vertex_indices[2] = facesArray[i * 3 + 2];
      }

      // copy point normals
      for (unsigned int i = 0; i < numNormals; i++){
        mesh.vertex_normals[i].x = normalsArray[i].x;
        mesh.vertex_normals[i].y = normalsArray[i].y;
        mesh.vertex_normals[i].z = normalsArray[i].z;
      }

      /* 
            optional:
            geometry_msgs/Point[] vertex_normals
            std_msgs/ColorRGBA[] vertex_colors
            geometry_msgs/Point[] vertex_texture_coords
            mesh_msgs/Material[] face_materials
            sensor_msgs/Image[] textures
            mesh_msgs/Cluster[] clusters
      */

      return true;
    }
    else
    {
      return false;
    }
  }

  bool fromTriangleMeshToMeshBuffer(const mesh_msgs::TriangleMesh& mesh, lvr::MeshBuffer& buffer)
  {
    // copy vertices
    vector<float> vertices;
    for (unsigned int i = 0; i < mesh.vertices.size(); i++)
    {
      vertices.push_back((float) mesh.vertices[i].x);
      vertices.push_back((float) mesh.vertices[i].y);
      vertices.push_back((float) mesh.vertices[i].z);
    }
    buffer.setVertexArray(vertices);

    // copy faces
    vector<unsigned int> faces;
    for (unsigned int i = 0; i < mesh.triangles.size(); i++)
    {
      faces.push_back((unsigned int) mesh.triangles[i].vertex_indices[0]);
      faces.push_back((unsigned int) mesh.triangles[i].vertex_indices[1]);
      faces.push_back((unsigned int) mesh.triangles[i].vertex_indices[2]);
    }
    buffer.setFaceArray(faces);

    // copy normals
    vector<float> normals;
    for(unsigned int i = 0; i < mesh.vertex_normals.size(); i++){
      normals.push_back((float) mesh.vertex_normals[i].x);
      normals.push_back((float) mesh.vertex_normals[i].y);
      normals.push_back((float) mesh.vertex_normals[i].z);
    }
    buffer.setVertexNormalArray(normals);

    return true;
  }

  bool fromPolygonMeshToTriangleMesh(
      mesh_msgs::PolygonMesh& polygon_mesh,
      mesh_msgs::TriangleMesh& triangle_mesh
      ){
    lvr::Tesselator<lvr::Vertexf, lvr::Normalf>::init();
    for (unsigned int i = 0; i < polygon_mesh.polygons.size(); i++ )
    {
      vector<vector<lvr::Vertexf> > vectorBorderPoints;
      vector<lvr::Vertexf > borderPoints;
      for (unsigned int j = 0; j < polygon_mesh.polygons[i].vertex_indices.size(); j++)
      {
        geometry_msgs::Point tmp = polygon_mesh.vertices[polygon_mesh.polygons[i].vertex_indices[j]];
        lvr::Vertexf vertex( tmp.x, tmp.y, tmp.z );
        borderPoints.push_back( vertex );
      }
      vectorBorderPoints.push_back( borderPoints );
      lvr::Tesselator<lvr::Vertexf, lvr::Normalf>::tesselate(vectorBorderPoints);
    }

    vector<float> vertices;
    std::vector<unsigned int> indices;
    std::vector<std::vector<lvr::Vertexf> >vectorBorderPoints;
    lvr::Tesselator<lvr::Vertexf, lvr::Normalf>::getFinalizedTriangles(
        vertices,
        indices,
        vectorBorderPoints
        );
    lvr::Tesselator<lvr::Vertexf, lvr::Normalf>::clear();

    for (unsigned int i = 0; i < vertices.size(); i+=3)
    {
      geometry_msgs::Point vertex;
      vertex.x = vertices[i];
      vertex.y = vertices[i+1];
      vertex.z = vertices[i+2];
      triangle_mesh.vertices.push_back(vertex);
    }

    for (unsigned int i = 0; i < indices.size(); i+=3)
    {
      mesh_msgs::TriangleIndices triangle;
      triangle.vertex_indices[0] = indices[i];
      triangle.vertex_indices[1] = indices[i+1];
      triangle.vertex_indices[2] = indices[i+2];
      triangle_mesh.triangles.push_back(triangle);
    }
    return true;
  }

  bool readMeshBuffer( lvr::MeshBufferPtr& buffer, string path )
  {
    lvr::ModelFactory io_factory;
    lvr::ModelPtr model = io_factory.readModel( path );

    if ( !model )
    {
      return false;
    }
    else
    {
      buffer = model->m_mesh;
      return true;
    }
  }

  bool writeMeshBuffer( lvr::MeshBufferPtr& buffer, string path )
  {
    lvr::ModelPtr model( new lvr::Model(buffer) );
    lvr::ModelFactory::saveModel( model, path );
    return true;
  }

  bool readTriangleMesh( mesh_msgs::TriangleMesh& mesh, string path )
  {
    lvr::ModelFactory io_factory;
    lvr::ModelPtr model = io_factory.readModel( path );

    if ( !model )
    {
      return false;
    }
    return fromMeshBufferToTriangleMesh( model->m_mesh, mesh);
  }

  bool writeTriangleMesh( mesh_msgs::TriangleMesh& mesh, string path )
  {
    lvr::MeshBuffer buffer;
    if( fromTriangleMeshToMeshBuffer( mesh, buffer ))
    {
      lvr::MeshBufferPtr buffer_ptr = boost::make_shared<lvr::MeshBuffer>(buffer);

      lvr::ModelPtr model( new lvr::Model( buffer_ptr ) );
      lvr::ModelFactory::saveModel(model, path);
      return true;
    }
    else return false;
  }

  void removeDuplicates( lvr::MeshBuffer& buffer)
  {
    lvr::floatArr old_vertexBuffer; lvr::uintArr old_indexBuffer;
    std::vector<float> new_vertexBuffer; std::vector<unsigned int> new_indexBuffer;

    size_t old_numVertices, old_numIndices;
    size_t new_numVertices, new_numIndices;

    old_vertexBuffer = buffer.getVertexArray( old_numVertices );
    old_indexBuffer = buffer.getFaceArray( old_numIndices);

    std::map<lvr::Vertex<float>, unsigned int> vertexMap;
    size_t pos;
    int index;

    for( int i = 0; i < old_numIndices ; i++ )
    {
      for (int j = 0; j < 3; j++)
      {
        index = old_indexBuffer[ 3 * i + j ];

        lvr::Vertex<float> vertex =
          lvr::Vertex<float>( old_vertexBuffer[  3 * index ],
              old_vertexBuffer[  3 * index + 1],
              old_vertexBuffer[  3 * index + 2] );

        if( vertexMap.find( vertex ) != vertexMap.end() )
        {
          pos = vertexMap[ vertex ];
        }
        else
        {
          pos = new_vertexBuffer.size() / 3;
          new_vertexBuffer.push_back( vertex[0] );
          new_vertexBuffer.push_back( vertex[1] );
          new_vertexBuffer.push_back( vertex[2] );

          vertexMap.insert( pair<lvr::Vertex<float>, unsigned int>( vertex, pos ) );
        }

        new_indexBuffer.push_back( pos );
      }
    }
    buffer.setVertexArray( new_vertexBuffer);
    buffer.setFaceArray( new_indexBuffer );
  }

  void intensityToTriangleRainbowColors(const std::vector<float>& intensity, mesh_msgs::TriangleMesh& mesh, float min, float max){
    float range = max - min;
    
    float r, g, b;
    float a = 1;
    
    for(size_t i=0; i<intensity.size(); i++){
      float norm = (intensity[i] - min) / range;
      getRainbowColor(norm, r, g, b);
      std_msgs::ColorRGBA color;
      color.a = a;
      color.r = r;
      color.g = g;
      color.b = b;
      mesh.triangle_colors.push_back(color);
    }
  }
  
  void intensityToTriangleRainbowColors(const std::vector<float>& intensity, mesh_msgs::TriangleMesh& mesh){
    float min = std::numeric_limits<float>::max();
    float max = std::numeric_limits<float>::min();
  
    for(size_t i=0; i<intensity.size(); i++){
      if(!std::isfinite(intensity[i])) continue;
      if(min > intensity[i]) min = intensity[i];
      if(max < intensity[i]) max = intensity[i];
    }
    intensityToTriangleRainbowColors(intensity, mesh, min, max);
  }
  
  void intensityToVertexRainbowColors(const std::vector<float>& intensity, mesh_msgs::TriangleMesh& mesh, float min, float max){
    float range = max - min;
    
    float r, g, b;
    float a = 1;
    
    for(size_t i=0; i<intensity.size(); i++){
      float norm = (intensity[i] - min) / range;
      getRainbowColor(norm, r, g, b);
      std_msgs::ColorRGBA color;
      color.a = a;
      color.r = r;
      color.g = g;
      color.b = b;
      mesh.vertex_colors.push_back(color);
    } 
  }

  void intensityToVertexRainbowColors(const std::vector<float>& intensity, mesh_msgs::TriangleMesh& mesh){
    float min = std::numeric_limits<float>::max();
    float max = std::numeric_limits<float>::min();
  
    for(size_t i=0; i<intensity.size(); i++){
      if(!std::isfinite(intensity[i])) continue;
      if(min > intensity[i]) min = intensity[i];
      if(max < intensity[i]) max = intensity[i];
    }
    intensityToVertexRainbowColors(intensity, mesh, min, max);
  }
  
  void removeDuplicates( mesh_msgs::TriangleMesh& mesh)
  {
    lvr::MeshBuffer buffer;
    fromTriangleMeshToMeshBuffer(mesh, buffer );
    removeDuplicates(buffer);

    lvr::MeshBufferPtr buffer_ptr = boost::make_shared<lvr::MeshBuffer>(buffer);
    fromMeshBufferToTriangleMesh(buffer_ptr, mesh);
  }

  static inline bool hasCloudChannel (const sensor_msgs::PointCloud2& cloud, const std::string& field_name)
  {
    // Get the index we need
    for (size_t d = 0; d < cloud.fields.size (); ++d)
      if (cloud.fields[d].name == field_name)
        return true;
    return false;
  }

  bool fromPointCloud2ToPointBuffer(const sensor_msgs::PointCloud2& cloud, lvr::PointBuffer& buffer)
  {
    ROS_DEBUG_STREAM("convert from PointCloud2 to PointBuffer.");

    size_t size = cloud.height * cloud.width;

    typedef sensor_msgs::PointCloud2ConstIterator<float> CloudIterFloat;
    typedef sensor_msgs::PointCloud2ConstIterator<uint8_t> CloudIterUInt8;


    std::list<int> filter_nan;


    // copy point data
    CloudIterFloat iter_x_filter(cloud, "x");
    CloudIterFloat iter_y_filter(cloud, "y");
    CloudIterFloat iter_z_filter(cloud, "z");

    // size without NaN values
    size = 0;
    for(int i=0; iter_x_filter != iter_x_filter.end();
      ++iter_x_filter, ++iter_y_filter, ++iter_z_filter, i++) {
      if(!std::isnan(*iter_x_filter) && !std::isnan(*iter_y_filter) && !std::isnan(*iter_z_filter))
        size++;
      else{
        filter_nan.push_back(i);
      }
    }

    filter_nan.sort();

    float* pointData = new float[size * 3];

        // copy point data
    CloudIterFloat iter_x(cloud, "x");
    CloudIterFloat iter_y(cloud, "y");
    CloudIterFloat iter_z(cloud, "z");


    std::list<int> tmp_filter = filter_nan;
    int index = 0;
    for(int i=0; 
        iter_x != iter_x.end();
        ++iter_x, ++iter_y, ++iter_z,
        index++) {
      // skip NaN point values
      if(!tmp_filter.empty() && index == tmp_filter.front()){
        tmp_filter.pop_front();
        continue;
      }

      // copy point 
      pointData[i]   = *iter_x;
      pointData[i+1] = *iter_y;
      pointData[i+2] = *iter_z; 

      i+=3; 

    }
    buffer.setPointArray(lvr::floatArr(pointData), size);


    // copy point normals if available
    bool normalsAvailable = 
        hasCloudChannel(cloud, "normal_x") 
        && hasCloudChannel(cloud, "normal_y")
        && hasCloudChannel(cloud, "normal_z");

    if(normalsAvailable){
      ROS_DEBUG_STREAM("include normals in conversion.");
      CloudIterFloat iter_n_x(cloud, "normal_x");
      CloudIterFloat iter_n_y(cloud, "normal_y");
      CloudIterFloat iter_n_z(cloud, "normal_z");
      float* normalsData = new float[size * 3];
      tmp_filter = filter_nan;
      int index = 0;
      for(int i=0; 
          iter_n_x != iter_n_x.end();
          ++iter_n_x, ++iter_n_y, ++iter_n_z,
          index++) {

        // skip NaN point values
        if(!tmp_filter.empty() && index == tmp_filter.front()){
          tmp_filter.pop_front();
          continue;
        }

        // copy normal 
        normalsData[i]   = *iter_n_x;
        normalsData[i+1] = *iter_n_y;
        normalsData[i+2] = *iter_n_z;

        i+=3;
      }
      buffer.setPointNormalArray(lvr::floatArr(normalsData), size);
    }


    // copy color data if available    
    if(hasCloudChannel(cloud, "rgb")){
      ROS_DEBUG_STREAM("include rgb in conversion.");
      CloudIterUInt8 iter_rgb(cloud, "rgb");
      uint8_t* colorData = new uint8_t[size*3];
      tmp_filter = filter_nan;
      int index = 0;
      for(int i=0; iter_rgb != iter_rgb.end();
          ++iter_rgb, index++) {

        // skip NaN point values
        if(!tmp_filter.empty() && index == tmp_filter.front()){
          tmp_filter.pop_front();
          continue;
        }

        // copy color rgb
        colorData[i]   = iter_rgb[0];
        colorData[i+1] = iter_rgb[1];
        colorData[i+2] = iter_rgb[2];

        i+=3;
      }
      buffer.setPointColorArray(lvr::ucharArr(colorData), size);
    }


    // copy intensity if available
    if(hasCloudChannel(cloud, "intensities")){
      ROS_DEBUG_STREAM("include intensities in conversion.");
      CloudIterFloat iter_int(cloud, "intensities");
      float* intensityData = new float[size];
      tmp_filter = filter_nan;
      int index = 0;
      for(int i=0; iter_int != iter_int.end();
          ++iter_int, index++) {

        // skip NaN point values
        if(!tmp_filter.empty() && index == tmp_filter.front()){
          tmp_filter.pop_front();
          continue;
        }

        // copy intensity
        intensityData[i]   = *iter_int;
        i++;
      }
      buffer.setPointIntensityArray(lvr::floatArr(intensityData), size);
    }
    ROS_DEBUG_STREAM("conversion finished.");
    return true;
  }
} // end namespace
