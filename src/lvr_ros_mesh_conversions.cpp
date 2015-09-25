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
 * lvr_ros_conversions.cpp
 *
 * Author: Henning Deeken <hdeeken@uos.de>,
 *         Sebastian Pütz <spuetz@uos.de>,
 *         Marcel Mrozinski <mmrozins@uos.de>,
 *         Tristan Igelbrink <tigelbri@uos.de>
 *
 */

#include "lvr_ros/lvr_ros_conversions.h"

namespace lvr_ros
{

  bool fromMeshBufferToTriangleMesh(lvr::MeshBufferPtr buffer, mesh_msgs::TriangleMesh& mesh)
  {
    size_t numVertices;
    size_t numFaces;
    size_t numNormals;
    lvr::coord3fArr verticesArray = buffer->getIndexedVertexArray(numVertices);
    lvr::coord3fArr normalsArray = buffer->getIndexedVertexNormalArray(numNormals);
    lvr::uintArr facesArray = buffer->getFaceArray(numFaces);

    mesh.vertices.resize(numVertices);
    mesh.vertex_normals.resize(numNormals);
    mesh.triangles.resize(numFaces);

    if (numVertices && numFaces)
    {
      for (unsigned int i = 0; i < numVertices; i++)
      {
        mesh.vertices[i].x = verticesArray[i].x;
        mesh.vertices[i].y = verticesArray[i].y;
        mesh.vertices[i].z = verticesArray[i].z;
      }

      for (unsigned int i = 0; i < numFaces; i++)
      {
        mesh.triangles[i].vertex_indices[0] = facesArray[i * 3];
        mesh.triangles[i].vertex_indices[1] = facesArray[i * 3 + 1];
        mesh.triangles[i].vertex_indices[2] = facesArray[i * 3 + 2];
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
      for (unsigned int i = 0; i < numNormals; i++){
        mesh.vertex_normals[i].x = normalsArray[i].x;
        mesh.vertex_normals[i].y = normalsArray[i].y;
        mesh.vertex_normals[i].z = normalsArray[i].z;
      }
      return true;
    }
    else
    {
      return false;
    }
  }

  bool fromTriangleMeshToMeshBuffer(mesh_msgs::TriangleMesh mesh, lvr::MeshBuffer& buffer)
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
    return true;
  }

  bool fromPolygonMeshToTriangleMesh(
      mesh_msgs::PolygonMesh polygon_mesh,
      mesh_msgs::TriangleMesh& triangle_mesh
      ){
    lvr::Tesselator<lvr::Vertexf, lvr::Normalf>::init();
    for (unsigned int i = 0; i < polygon_mesh.polygons.size(); i++ )
    {
      vector<vector<lvr::Vertexf>> vectorBorderPoints;
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

  bool writeMeshBuffer( lvr::MeshBufferPtr buffer, string path )
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

  bool writeTriangleMesh( mesh_msgs::TriangleMesh mesh, string path )
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

  void removeDuplicates( mesh_msgs::TriangleMesh& mesh)
  {
    lvr::MeshBuffer buffer;
    fromTriangleMeshToMeshBuffer(mesh, buffer );
    removeDuplicates(buffer);

    lvr::MeshBufferPtr buffer_ptr = boost::make_shared<lvr::MeshBuffer>(buffer);
    fromMeshBufferToTriangleMesh(buffer_ptr, mesh);
  }

  /*
     bool convertMeshBufferToTextureMessage(lvr::MeshBufferPtr buffer, lvr_ros::Textures& message)
     {
     GroupVector textureMaterials;
     GroupVector colorMaterials;
     generateMaterialGroupsFromMeshBuffer(buffer, textureMaterials, colorMaterials);

     size_t numVertices;
     size_t numVertexTextureCoordinates;
     size_t numTextures;

     lvr::coord3fArr vertices = buffer->getIndexedVertexArray(numVertices);
     lvr::coord3fArr vertexTextureCoordinates = 
     buffer->getIndexedVertexTextureCoordinateArray(numVertexTextureCoordinates);

     lvr::textureArr textures = buffer->getTextureArray(numTextures);

     lvr_ros::Textures texturesMsg;
     texturesMsg.header.stamp = stamp;
     texturesMsg.header.frame_id = frame;
     texturesMsg.texturecoords.resize(numVertices);
     texturesMsg.materialgroups.resize(textureMaterials.size() + colorMaterials.size());

     for (int i = 0; i < numVertices; i++)
     {
  // texture stuff
  texturesMsg.texturecoords[i].x = vertexTextureCoordinates[i][0];
  texturesMsg.texturecoords[i].y = 1 - vertexTextureCoordinates[i][1];
  texturesMsg.texturecoords[i].z = vertexTextureCoordinates[i][2];
  }

  // fill materialgroups message type with material groups WITH texture
  for (int i = 0; i < textureMaterials.size(); i++)
  {
  MaterialGroupPtr g = textureMaterials[i];

  int width  = textures[g->texture_index]->m_width;
  int height = textures[g->texture_index]->m_height;

  // fill image in materialgroup message
  texturesMsg.materialgroups[i].image.width  = width;
  texturesMsg.materialgroups[i].image.height = height;
  texturesMsg.materialgroups[i].image.data.resize(width * height * 3);

  for (int j = 0; j < width * height * 3; j++)
  {
  texturesMsg.materialgroups[i].image.data[j] = 
  (unsigned int8_t) textures[g->texture_index]->m_pixels[j];
  }

  // fill default color in message
  texturesMsg.materialgroups[i].defaultcolor.r = g->r;
  texturesMsg.materialgroups[i].defaultcolor.g = g->g;
  texturesMsg.materialgroups[i].defaultcolor.b = g->b;

  //TODO: put in intensity values
  texturesMsg.materialgroups[i].defaultcolor.a = 1;
  texturesMsg.materialgroups[i].face_indices.resize(g->faceBuffer.size());

  // fill associated faces of texture in message
  for (int k = 0; k < g->faceBuffer.size(); k++)
  {
  texturesMsg.materialgroups[i].face_indices[k] = g->faceBuffer[k];
  }
  g->faceBuffer.clear();
  }

  // fill materialgroups WITHOUT texture in message
  for (int i = 0; i < colorMaterials.size(); i++)
  {
  MaterialGroupPtr g = colorMaterials[i];
  int index = i + textureMaterials.size();
  // fill default color in message
  texturesMsg.materialgroups[index].defaultcolor.r = g->r;
  texturesMsg.materialgroups[index].defaultcolor.g = g->g;
  texturesMsg.materialgroups[index].defaultcolor.b = g->b;
  texturesMsg.materialgroups[index].image.width  = 0;
  texturesMsg.materialgroups[index].image.height = 0;
  //TODO: put in intensity values
  texturesMsg.materialgroups[index].defaultcolor.a = 1;
  texturesMsg.materialgroups[index].face_indices.resize(g->faceBuffer.size());

  // fill associated faces of texture in message
  for (int k = 0; k < g->faceBuffer.size(); k++)
  {
    texturesMsg.materialgroups[index].face_indices[k] = g->faceBuffer[k];
  }
  g->faceBuffer.clear();
}
textureMaterials.clear();
colorMaterials.clear();
return texturesMsg;
}

void generateMaterialGroupsFromMeshBuffer(lvr::MeshBufferPtr buffer,
    GroupVector &textureMaterials,
    GroupVector &colorMaterials)
{
  int countera = 0;
  size_t numMaterials;
  size_t numFaceMaterialIndices;
  size_t numFaces;

  lvr::materialArr faceMaterials = buffer->getMaterialArray(numMaterials);
  lvr::uintArr faceMaterialIndices = buffer->getFaceMaterialIndexArray(numFaceMaterialIndices);
  lvr::uintArr facesArray = buffer->getFaceArray(numFaces);

  std::map<int, MaterialGroupPtr > texMatMap;
  std::map<lvr::Vertex<unsigned char>, MaterialGroupPtr > colorMatMap;

  // Iterate over face material buffer and
  // sort faces by their material
  for (unsigned int i = 0; i < numMaterials; i++)
  {
    std::map<int, MaterialGroupPtr>::iterator texIt;
    std::map<lvr::Vertex<unsigned char>, MaterialGroupPtr >::iterator colIt;

    // Get material by index and lookup in map. If present
    // add face index to the corresponding group. Create a new
    // group if none was found. For efficient rendering we have to
    // create groups by color and texture index,
    lvr::Material* m = faceMaterials[i];

    if (m->texture_index != -1)
    {

      texIt = texMatMap.find(m->texture_index);
      if (texIt == texMatMap.end())
      {
        MaterialGroupPtr g = MaterialGroupPtr(new MaterialGroup());
        g->texture_index = m->texture_index;
        g->r = 1;
        g->g = 1;
        g->b = 1;
        textureMaterials.push_back(g);
        texMatMap[m->texture_index] = g;
        countera++;
      }
    }

    else
    {
      lvr::Vertex<unsigned char> coloru = lvr::Vertex<unsigned char>(m->r, m->g, m->b);
      colIt = colorMatMap.find(coloru);

      if (colIt == colorMatMap.end())
      {
        MaterialGroupPtr g = MaterialGroupPtr(new MaterialGroup());
        g->texture_index = m->texture_index;
        g->r = m->r;
        g->g = m->g;
        g->b = m->b;
        colorMaterials.push_back(g);
        colorMatMap[coloru] = g;
        countera++;
      }
    }
  }

  // fill MaterialGroups with associated faces
  for (unsigned int i = 0; i < numFaces; i++)
  {
    std::map<int, MaterialGroupPtr>::iterator texIt;
    std::map<lvr::Vertex<unsigned char>, MaterialGroupPtr >::iterator colIt;

    lvr::Material* m = faceMaterials[faceMaterialIndices[i]];

    if (m->texture_index != -1)
    {
      texIt = texMatMap.find(m->texture_index);
      texIt->second->faceBuffer.push_back(i);
    }

    else
    {
      colIt = colorMatMap.find(lvr::Vertex<unsigned char>(m->r, m->g, m->b));
      colIt->second->faceBuffer.push_back(i);
    }
  }
}
*/
} // end namespace
