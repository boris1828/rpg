#pragma once

#include <glm/glm.hpp>
#include "VertexArray.h"
#include "VertexBuffer.h"
#include "IndexBuffer.h"
#include "Shaders.h"
#include "Types.h"

enum class MeshType
{
    mesh_default, 
    wireframe,
    aabb, // TODO: update aabb mesh to wf something
    animated
};

using FrameIndex = uint8_t;

struct AnimatedMeshData
{
    FrameIndex current_frame;
    FrameIndex total_frames;
    GLuint indices_per_frame;
};

struct Mesh 
{
    Mesh();
    Mesh(const float*        vertices,       
         unsigned int        vertexCount,
         const unsigned int* indices, 
         unsigned int        indexCount, 
         GLenum usage, 
         MeshType type = MeshType::mesh_default);

    ~Mesh();

    // Mesh(const Mesh&)            = delete;
    // Mesh& operator=(const Mesh&) = delete;

    virtual void draw(Shader& shader)          const;
    virtual void drawWireframe(Shader& shader) const;
    virtual void drawDefault(Shader& shader)   const;
    virtual void drawAnimated(Shader& shader)  const;

    void updateVertices(const float* vertices, unsigned int vertexCount);

    void setFrame(FrameIndex frame_index);
    void nextFrame();

    GLenum       m_usage;
    MeshType     m_type;
    VertexArray  m_VAO;
    VertexBuffer m_VBO;
    IndexBuffer  m_IBO;
    unsigned int m_VertexCount = 0;

    union
    {
        AnimatedMeshData animated_data;
    };
};

struct DebugLineRenderer
{
    unsigned int VAO, VBO;
    
    DebugLineRenderer();
    void draw(const Real3& start, const Real3& end, Shader& shader);
    ~DebugLineRenderer();
};

Mesh createXWireframeMesh();
Mesh createZAAHemisphereWireframeMesh(unsigned int segments);
Mesh createSphereWireframeMesh(unsigned int segments);
Mesh createZAACapsuleWireframeMesh(unsigned int segments);
Mesh createZAlignedCylinderWireframeMesh(unsigned int segments);
Mesh createZAlignedCapsuleBodyWireframeMesh(unsigned int segments);
Mesh createZARectangleWireframeMesh();
Mesh createAABBWireframeMesh();
Mesh createCenteredAABBWireframeMesh();
Mesh createZA4SidedPyramidWireframeMesh();
Mesh createZAConeWireframeMesh(unsigned int segments);
Mesh createXYSquareWireframeMesh();

Mesh createXMesh();
Mesh createUnitCircleMesh(unsigned int segments);
Mesh createSegmentedDynamicLineMesh(unsigned int segments);

Mesh* loadOBJ(const std::string& path);
Mesh loadOBJStatic(const std::string& path);

Mesh* loadAnimatedOBJ(const std::string& directory_path, const std::string& prefix, FrameIndex num_frames);