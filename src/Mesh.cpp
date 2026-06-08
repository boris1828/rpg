#define TINYOBJLOADER_IMPLEMENTATION
#include <tiny_obj_loader.h>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <iostream>

#include "Mesh.h"
#include "Macro.h"
#include "Types.h"

Mesh::Mesh(
    const float*        vertices, 
    unsigned int        vertexCount,
    const unsigned int* indices, 
    unsigned int        indexCount, 
    GLenum              usage, 
    MeshType            type) 
    : 
    m_VBO(vertices, vertexCount * sizeof(float), usage),
    m_IBO(indices, indexCount),
    m_VertexCount(vertexCount), 
    m_usage(usage), 
    m_type(type)
{
    VertexBufferLayout layout;

    switch(type)
    {
        case MeshType::mesh_default:
        {
            layout.push<float>(3); // vertex
            layout.push<float>(3); // normals
            break;
        }
        case MeshType::animated:
        {
            layout.push<float>(3); // vertex
            layout.push<float>(3); // normals
            break;
        }
        case MeshType::wireframe:
        {
            layout.push<float>(3); // vertex
            break;
        }
        default:
        {
            ASSERT(false, "Unhandled or unknown mesh type");
        }
    }

    m_VAO.addVertexBuffer(m_VBO, layout);
    m_VAO.setIndexBuffer(m_IBO);
    m_VAO.unbind();
}

Mesh::Mesh() {}

Mesh::~Mesh() {}

void Mesh::updateVertices(const float* vertices, unsigned int vertexCount)
{
    ASSERT(vertexCount == m_VertexCount, "Vertex count mismatch in Mesh::updateVertices");
    ASSERT(m_usage != GL_STATIC_DRAW, "Mesh can't be static if i want to update data");

    m_VBO.updateData(vertices, vertexCount * sizeof(float));
}

void Mesh::drawWireframe(Shader& shader) const 
{
    shader.use();
    m_VAO.bind();

    glDrawElements(GL_LINES, m_IBO.getCount(), GL_UNSIGNED_INT, nullptr);
}

void Mesh::drawDefault(Shader& shader) const 
{
    shader.use();
    m_VAO.bind();

    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glDrawElements(GL_TRIANGLES, m_IBO.getCount(), GL_UNSIGNED_INT, nullptr);
}

void Mesh::drawAnimated(Shader& shader) const
{
    ASSERT(m_type == MeshType::animated, "Mesh::drawAnimated called on a non-animated mesh");

    const AnimatedMeshData& data = animated_data;

    ASSERT(data.total_frames > 0,                  "AnimatedMeshData has zero total_frames");
    ASSERT(data.current_frame < data.total_frames, "AnimatedMeshData current_frame out of range");
    ASSERT(data.indices_per_frame % 3 == 0,        "Animated mesh indices_per_frame must be a multiple of 3 for GL_TRIANGLES");
    ASSERT((data.current_frame + 1) * data.indices_per_frame <= m_IBO.getCount(), "Animated mesh frame indices exceed index buffer size");
    
    GLuint startIndex = data.current_frame * data.indices_per_frame;
    GLuint count      = data.indices_per_frame;

    shader.use();
    m_VAO.bind();

    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    glDrawElements(
        GL_TRIANGLES,
        count,
        GL_UNSIGNED_INT,
        reinterpret_cast<void*>(startIndex * sizeof(GLuint))
    );
}

void Mesh::setFrame(FrameIndex frame)
{
    ASSERT(m_type == MeshType::animated, "Mesh::drawAnimated called on a non-animated mesh");
    ASSERT(frame < animated_data.total_frames, "AnimatedMeshData current_frame out of range");

    animated_data.current_frame = frame;
}

void Mesh::nextFrame()
{
    ASSERT(m_type == MeshType::animated, "Mesh::drawAnimated called on a non-animated mesh");

    animated_data.current_frame = (animated_data.current_frame + 1) % animated_data.total_frames;
}

void Mesh::draw(Shader& shader) const 
{
    switch(m_type)
    {
        case MeshType::mesh_default:
        {   
            drawDefault(shader); break;
        }
        case MeshType::wireframe:
        {
            drawWireframe(shader); break;
        }
        case MeshType::animated:
        {
            drawAnimated(shader); break;
        }
        default:
        {
            ASSERT(false, "Unhandled or unknown mesh type in draw");
        }
    }
}


DebugLineRenderer::DebugLineRenderer()
{
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    
    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, 6 * sizeof(float), nullptr, GL_DYNAMIC_DRAW);
    
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
}

void DebugLineRenderer::draw(const Real3& start, const Real3& end, Shader& shader)
{
    float vertices[] = {
        start.x, start.y, start.z,
        end.x,   end.y,   end.z
    };
    
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vertices), vertices);
    
    shader.use();
    glBindVertexArray(VAO);
    glDrawArrays(GL_LINES, 0, 2);
}

DebugLineRenderer::~DebugLineRenderer()
{
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
}

// =======================================================
// PROCEDURAL/CALCULATED MESH
// =======================================================

constexpr float PI = 3.14159265358979323846f;

Mesh createXWireframeMesh()
{
    float vertices[4*3*2] = 
    {
        -1.0f, -1.0f, 0.0f,
         1.0f,  1.0f, 0.0f,
         1.0f, -1.0f, 0.0f,
        -1.0f,  1.0f, 0.0f
    };

    unsigned int indices[2 * 2] =
    {
        0, 1,   2, 3
    };

    return Mesh(vertices, 4*3*2, indices, 2*2, GL_STATIC_DRAW, MeshType::wireframe);
}

Mesh createXMesh()
{
    float vertices[4*3*2] = 
    {
        -1.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
         1.0f,  1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
         1.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        -1.0f,  1.0f, 0.0f, 0.0f, 0.0f, 0.0f
    };

    unsigned int indices[2 * 2] =
    {
        0, 1,   2, 3
    };

    return Mesh(vertices, 4*3*2, indices, 2*2, GL_STATIC_DRAW);
}

Mesh createZAAHemisphereWireframeMesh(unsigned int segments)
{
    if (segments < 3) segments = 3;

    const float radius = 1.0f;
    std::vector<float> vertices;
    std::vector<unsigned int> indices;

    unsigned int baseStartIdx = 0;
    float dthetaFull = 2.0f * PI / segments;
    
    for (unsigned int i = 0; i < segments; ++i)
    {
        float angle = i * dthetaFull;
        vertices.push_back(radius * std::cos(angle)); // X
        vertices.push_back(radius * std::sin(angle)); // Y
        vertices.push_back(0.0f);                    // Z

        indices.push_back(i);
        indices.push_back((i + 1) % segments);
    }

    auto addHalfRing = [&](bool isXZ) 
    {
        unsigned int startIdx = (unsigned int)(vertices.size() / 3);
        float dthetaHalf = PI / segments;

        for (unsigned int i = 0; i <= segments; ++i)
        {
            float angle = i * dthetaHalf;
            float horizontal = radius * std::cos(angle);
            float vertical   = radius * std::sin(angle);

            if (isXZ) 
            {
                vertices.push_back(horizontal); // X
                vertices.push_back(0.0f);       // Y
                vertices.push_back(vertical);   // Z
            } 
            else 
            {
                vertices.push_back(0.0f);       // X
                vertices.push_back(horizontal); // Y
                vertices.push_back(vertical);   // Z
            }

            if (i < segments) 
            {
                indices.push_back(startIdx + i);
                indices.push_back(startIdx + i + 1);
            }
        }
    };

    addHalfRing(true);  // XZ Arc
    addHalfRing(false); // YZ Arc

    return Mesh(vertices.data(), vertices.size(), indices.data(), indices.size(), GL_STATIC_DRAW, MeshType::wireframe);
}

Mesh createSphereWireframeMesh(unsigned int segments)
{
    if (segments < 3) segments = 3;

    const float radius = 1.0f;
    std::vector<float> vertices;
    std::vector<unsigned int> indices;

    const float dtheta = 2.0f * PI / segments;

    auto addRing = [&](int axis) 
    {
        unsigned int startIdx = (unsigned int)(vertices.size() / 3);

        for (unsigned int i = 0; i < segments; ++i) 
        {
            float angle = i * dtheta;
            float c = radius * std::cos(angle);
            float s = radius * std::sin(angle);

            if (axis == 0) 
            { 
                vertices.push_back(0.0f); vertices.push_back(c); vertices.push_back(s);
            } 
            else if (axis == 1) 
            {
                vertices.push_back(c); vertices.push_back(0.0f); vertices.push_back(s);
            } 
            else 
            {
                vertices.push_back(c); vertices.push_back(s); vertices.push_back(0.0f);
            }

            indices.push_back(startIdx + i);
            indices.push_back(startIdx + (i + 1) % segments);
        }
    };

    addRing(0); // X-axis ring
    addRing(1); // Y-axis ring
    addRing(2); // Z-axis ring

    return Mesh(vertices.data(), vertices.size(), indices.data(), indices.size(), GL_STATIC_DRAW, MeshType::wireframe);
}

Mesh createZAACapsuleWireframeMesh(unsigned int segments)
{
    if (segments < 3) segments = 3;

    std::vector<float> vertices;
    std::vector<unsigned int> indices;

    float radius = 1.0f; 
    float height = 1.0f; 

    auto addRing = [&](float z) 
    {
        unsigned int startIdx = (unsigned int)(vertices.size() / 3);
        float dtheta = 2.0f * PI / segments;
        for (unsigned int i = 0; i < segments; ++i) 
        {
            float angle = i * dtheta;
            vertices.push_back(radius * std::cos(angle));
            vertices.push_back(radius * std::sin(angle));
            vertices.push_back(z);

            indices.push_back(startIdx + i);
            indices.push_back(startIdx + (i + 1) % segments);
        }
    };

    auto addVerticals = [&]() 
    {
        // Connect the top ring and bottom ring with 4 lines
        // vertices are already added by the rings, but for simplicity in a wireframe,
        // we can just add specific lines at 0, 90, 180, and 270 degrees.
        for (int i = 0; i < 4; ++i) 
        {
            float angle = i * (PI / 2.0f);
            float x = radius * std::cos(angle);
            float y = radius * std::sin(angle);

            unsigned int startIdx = (unsigned int)(vertices.size() / 3);
            vertices.push_back(x); vertices.push_back(y); vertices.push_back(0.0f);   // Bottom
            vertices.push_back(x); vertices.push_back(y); vertices.push_back(height); // Top

            indices.push_back(startIdx);
            indices.push_back(startIdx + 1);
        }
    };

    auto addArc = [&](float baseZ, bool isTop, bool isXZ) 
    {
        unsigned int startIdx = (unsigned int)(vertices.size() / 3);
        float dthetaHalf = PI / segments;

        for (unsigned int i = 0; i <= segments; ++i) 
        {
            float angle = i * dthetaHalf;
            // For top, angle goes 0 to PI. For bottom, we flip it.
            float h = radius * std::cos(angle);
            float v = radius * std::sin(angle);
            if (!isTop) v = -v; 

            if (isXZ) 
            {
                vertices.push_back(h); vertices.push_back(0.0f); vertices.push_back(baseZ + v);
            } 
            else 
            {
                vertices.push_back(0.0f); vertices.push_back(h); vertices.push_back(baseZ + v);
            }

            if (i < segments) 
            {
                indices.push_back(startIdx + i);
                indices.push_back(startIdx + i + 1);
            }
        }
    };

    // 1. Draw the two cylinder-end rings
    addRing(0.0f);
    addRing(height);

    // 2. Draw the 4 side ribs
    addVerticals();

    // 3. Draw the Top Hemisphere Arcs
    addArc(height, true, true);  // XZ Top Arc
    addArc(height, true, false); // YZ Top Arc

    // 4. Draw the Bottom Hemisphere Arcs
    addArc(0.0f, false, true);   // XZ Bottom Arc
    addArc(0.0f, false, false);  // YZ Bottom Arc

    return Mesh(vertices.data(), vertices.size(), indices.data(), indices.size(), GL_STATIC_DRAW, MeshType::wireframe);
}

Mesh createZAlignedCylinderWireframeMesh(unsigned int segments)
{
    if (segments < 3) segments = 3;

    const float radius = 1.0f;

    std::vector<float>        vertices;
    std::vector<unsigned int> indices;
    vertices.reserve(2 * ((segments + 2) * 3));
    indices.reserve(3 * segments * 2);

    const float dtheta = 2.0f * PI / segments;

    for (unsigned int i = 0; i < segments; ++i)
    {
        float angle = i * dtheta;
        float x = radius * std::cos(angle);
        float y = radius * std::sin(angle);

        vertices.push_back(x);
        vertices.push_back(y);
        vertices.push_back(0.0f);
    }

    for (unsigned int i = 0; i < segments; ++i)
    {
        float angle = i * dtheta;
        float x = radius * std::cos(angle);
        float y = radius * std::sin(angle);

        vertices.push_back(x);
        vertices.push_back(y);
        vertices.push_back(1.0f);
    }

    // bottom ring
    for (unsigned int i = 0; i < segments; ++i)
    {
        indices.push_back(i);
        indices.push_back((i + 1) % segments);
    }

    // top ring
    for (unsigned int i = 0; i < segments; ++i)
    {
        indices.push_back(i + segments);
        indices.push_back(((i + 1) % segments) + segments);
    }

    // vertical edges
    for (unsigned int i = 0; i < segments; ++i)
    {
        indices.push_back(i);
        indices.push_back(i + segments);
    }

    return Mesh(
        vertices.data(), static_cast<unsigned int>(vertices.size()),
        indices.data(),  static_cast<unsigned int>(indices.size()), 
        GL_STATIC_DRAW, 
        MeshType::wireframe);
}

Mesh createZAlignedCapsuleBodyWireframeMesh(unsigned int segments)
{
    if (segments < 4) segments = 4;

    const float radius = 1.0f;

    std::vector<float>        vertices;
    std::vector<unsigned int> indices;
    vertices.reserve(2 * ((segments + 2) * 3));
    indices.reserve(3 * segments * 2);

    const float dtheta = 2.0f * PI / segments;

    for (unsigned int i = 0; i < segments; ++i)
    {
        float angle = i * dtheta;
        float x     = radius * std::cos(angle);
        float y     = radius * std::sin(angle);

        vertices.push_back(x);
        vertices.push_back(y);
        vertices.push_back(0.0f);
    }

    for (unsigned int i = 0; i < segments; ++i)
    {
        float angle = i * dtheta;
        float x     = radius * std::cos(angle);
        float y     = radius * std::sin(angle);

        vertices.push_back(x);
        vertices.push_back(y);
        vertices.push_back(1.0f);
    }

    // vertical edges
    for (unsigned int i = 0; i < segments; ++i)
    {
        indices.push_back(i);
        indices.push_back(i + segments);
    }

    return Mesh(
        vertices.data(), static_cast<unsigned int>(vertices.size()),
        indices.data(),  static_cast<unsigned int>(indices.size()), 
        GL_STATIC_DRAW, 
        MeshType::wireframe);
}

Mesh createAABBWireframeMesh()
{
    static constexpr unsigned int num_vertices = 8*3;
    static constexpr unsigned int num_indices = 12*2;

    static constexpr float vertices[num_vertices] = {
        0.0f, 0.0f, 0.0f, 
        1.0f, 0.0f, 0.0f, 
        1.0f, 1.0f, 0.0f, 
        0.0f, 1.0f, 0.0f, 
        0.0f, 0.0f, 1.0f, 
        1.0f, 0.0f, 1.0f, 
        1.0f, 1.0f, 1.0f, 
        0.0f, 1.0f, 1.0f
    };

    static constexpr unsigned int indices[num_indices] = {
        0, 1,  1, 2,  2, 3,  3, 0, // base
        4, 5,  5, 6,  6, 7,  7, 4, // top
        0, 4,  1, 5,  2, 6,  3, 7  // sides
    };

    return Mesh(
        vertices, num_vertices,
        indices,  num_indices, 
        GL_STATIC_DRAW, 
        MeshType::wireframe);
}

Mesh createCenteredAABBWireframeMesh()
{
    static constexpr unsigned int num_vertices = 8*3;
    static constexpr unsigned int num_indices = 12*2;

    static constexpr float vertices[num_vertices] = {
        0.0f - 0.5f, 0.0f - 0.5f, 0.0f - 0.5f, 
        1.0f - 0.5f, 0.0f - 0.5f, 0.0f - 0.5f, 
        1.0f - 0.5f, 1.0f - 0.5f, 0.0f - 0.5f, 
        0.0f - 0.5f, 1.0f - 0.5f, 0.0f - 0.5f, 
        0.0f - 0.5f, 0.0f - 0.5f, 1.0f - 0.5f, 
        1.0f - 0.5f, 0.0f - 0.5f, 1.0f - 0.5f, 
        1.0f - 0.5f, 1.0f - 0.5f, 1.0f - 0.5f, 
        0.0f - 0.5f, 1.0f - 0.5f, 1.0f - 0.5f
    };

    static constexpr unsigned int indices[num_indices] = {
        0, 1,  1, 2,  2, 3,  3, 0, // base
        4, 5,  5, 6,  6, 7,  7, 4, // top
        0, 4,  1, 5,  2, 6,  3, 7  // sides
    };

    return Mesh(
        vertices, num_vertices,
        indices,  num_indices, 
        GL_STATIC_DRAW, 
        MeshType::wireframe);
}

Mesh createZARectangleWireframeMesh()
{
    static constexpr float vertices[4*3]
    {
        -0.5f, 0.0f, -0.5f, 
        -0.5f, 0.0f,  0.5f, 
         0.5f, 0.0f,  0.5f, 
         0.5f, 0.0f, -0.5f, 
    };

    static constexpr unsigned int indices[4*2] = {
        0, 1,  1, 2,  2, 3,  3, 0,
    };

    return Mesh(
        vertices, 4*3,
        indices,  4*2, 
        GL_STATIC_DRAW, 
        MeshType::wireframe);
}

Mesh createZA4SidedPyramidWireframeMesh()
{
    static constexpr float vertices[5 * 3] = 
    {
        -0.5f, -0.5f, 0.0f,  // 0: Bottom-Left
         0.5f, -0.5f, 0.0f,  // 1: Bottom-Right
         0.5f,  0.5f, 0.0f,  // 2: Top-Right
        -0.5f,  0.5f, 0.0f,  // 3: Top-Left
         0.0f,  0.0f, 1.0f   // 4: Apex
    };

    static constexpr unsigned int indices[8 * 2] = {
        0, 1,  1, 2,  2, 3,  3, 0,
        0, 4,  1, 4,  2, 4,  3, 4
    };

    return Mesh(
        vertices, 5 * 3,
        indices,  8 * 2, 
        GL_STATIC_DRAW, 
        MeshType::wireframe);
}

Mesh createZAConeWireframeMesh(unsigned int segments)
{
    if (segments < 3) segments = 3;

    const float radius = 1.0f;
    const float height = 1.0f;

    std::vector<float>        vertices;
    std::vector<unsigned int> indices;

    const float dtheta = 2.0f * PI / segments;

    for (unsigned int i = 0; i < segments; ++i)
    {
        float angle = i * dtheta;
        float x = radius * std::cos(angle);
        float y = radius * std::sin(angle);

        vertices.push_back(x);
        vertices.push_back(y);
        vertices.push_back(0.0f);
    }

    vertices.push_back(0.0f);
    vertices.push_back(0.0f);
    vertices.push_back(height);

    for (unsigned int i = 0; i < segments; ++i)
    {
        indices.push_back(i);
        indices.push_back((i + 1) % segments);
        indices.push_back(i);
        indices.push_back(segments);
    }

    return Mesh(
        vertices.data(), static_cast<unsigned int>(vertices.size()),
        indices.data(),  static_cast<unsigned int>(indices.size()), 
        GL_STATIC_DRAW, 
        MeshType::wireframe);
}

Mesh createXYSquareWireframeMesh()
{
    static constexpr unsigned int num_vertices = 4*3;
    static constexpr unsigned int num_indices  = 4*2;

    static constexpr float vertices[num_vertices] = {
        -0.5f, -0.5f, 0.0f, 
         0.5f, -0.5f, 0.0f, 
         0.5f,  0.5f, 0.0f, 
        -0.5f,  0.5f, 0.0f, 
    };

    static constexpr unsigned int indices[num_indices] = {
        0, 1,  1, 2,  2, 3,  3, 0, // base
    };

    return Mesh(
        vertices, num_vertices,
        indices,  num_indices, 
        GL_STATIC_DRAW, 
        MeshType::wireframe);
}

Mesh createUnitCircleMesh(unsigned int segments)
{
    if (segments < 3) segments = 3;

    const float radius = 1.0f;

    std::vector<float>        vertices;
    std::vector<unsigned int> indices;
    vertices.reserve((segments + 2) * 3);
    indices.reserve(segments * 2);

    const float dtheta = 2.0f * PI / segments;

    for (unsigned int i = 0; i < segments; ++i)
    {
        float angle = i * dtheta;
        float x = radius * std::cos(angle);
        float y = radius * std::sin(angle);

        vertices.push_back(x);
        vertices.push_back(y);
        vertices.push_back(0.0f);

        // Normals
        vertices.push_back(0.0f);
        vertices.push_back(0.0f);
        vertices.push_back(0.0f);

        // TODO: add texcoords if needed
    }

    for (unsigned int i = 0; i < segments; ++i)
    {
        indices.push_back(i);
        indices.push_back((i + 1) % segments);
    }

    return Mesh(vertices.data(), static_cast<unsigned int>(vertices.size()),
                indices.data(),  static_cast<unsigned int>(indices.size()), GL_STATIC_DRAW);
}

Mesh createZAlignedCylinderMesh(unsigned int segments)
{
    if (segments < 3) segments = 3;

    const float radius = 1.0f;

    std::vector<float>        vertices;
    std::vector<unsigned int> indices;
    vertices.reserve(2 * ((segments + 2) * 3));
    indices.reserve(3 * segments * 2);

    const float dtheta = 2.0f * PI / segments;

    for (unsigned int i = 0; i < segments; ++i)
    {
        float angle = i * dtheta;
        float x = radius * std::cos(angle);
        float y = radius * std::sin(angle);

        vertices.push_back(x);
        vertices.push_back(y);
        vertices.push_back(0.0f);

        // Normals
        vertices.push_back(0.0f);
        vertices.push_back(0.0f);
        vertices.push_back(0.0f);

        // TODO: add texcoords if needed
    }

    for (unsigned int i = 0; i < segments; ++i)
    {
        float angle = i * dtheta;
        float x = radius * std::cos(angle);
        float y = radius * std::sin(angle);

        vertices.push_back(x);
        vertices.push_back(y);
        vertices.push_back(1.0f);

        // Normals
        vertices.push_back(0.0f);
        vertices.push_back(0.0f);
        vertices.push_back(0.0f);

        // TODO: add texcoords if needed
    }

    // bottom ring
    for (unsigned int i = 0; i < segments; ++i)
    {
        indices.push_back(i);
        indices.push_back((i + 1) % segments);
    }

    // top ring
    for (unsigned int i = 0; i < segments; ++i)
    {
        indices.push_back(i + segments);
        indices.push_back(((i + 1) % segments) + segments);
    }

    // vertical edges
    for (unsigned int i = 0; i < segments; ++i)
    {
        indices.push_back(i);
        indices.push_back(i + segments);
    }

    return Mesh(vertices.data(), static_cast<unsigned int>(vertices.size()),
                indices.data(),  static_cast<unsigned int>(indices.size()), GL_STATIC_DRAW);
}

Mesh createSegmentedDynamicLineMesh(unsigned int segments) 
{    
    std::vector<float>        vertices;
    std::vector<unsigned int> indices;

    for (unsigned int i = 0; i < segments; ++i) {
        vertices.push_back(0.0f); 
        vertices.push_back(0.0f); 
        vertices.push_back(0.0f); 

        // Normals
        vertices.push_back(0.0f);
        vertices.push_back(0.0f);
        vertices.push_back(0.0f);

        // TODO: add normals and texcoords if needed

        if (i < segments - 1) {
            indices.push_back(i);
            indices.push_back(i + 1);
        }
    }

    return Mesh(vertices.data(), vertices.size(), indices.data(), indices.size(), GL_DYNAMIC_DRAW);
}

// =======================================================
// LOADING MESH
// =======================================================

#define VERTEX_DIMENSION 6

struct Vertex 
{
    float x, y, z;
    float nx, ny, nz;

    bool operator==(const Vertex& other) const 
    {
        return  x==other.x  &&  y==other.y  &&  z==other.z
            && nx==other.nx && ny==other.ny && nz==other.nz;
    }
};

struct VertexHash 
{
    size_t operator()(const Vertex& v) const 
    {
        size_t h1 = std::hash<float>()(v.x);
        size_t h2 = std::hash<float>()(v.y);
        size_t h3 = std::hash<float>()(v.z);
        size_t h4 = std::hash<float>()(v.nx);
        size_t h5 = std::hash<float>()(v.ny);
        size_t h6 = std::hash<float>()(v.nz);
        return (((((h1 ^ (h2<<1)) ^ (h3<<1)) ^ (h4<<1)) ^ (h5<<1)) ^ (h6<<1));
    }
};

void loadOBJVerticesIndicesData(
    const std::string&         path, 
    std::vector<float>&        vertices, 
    std::vector<unsigned int>& indices)
{
    tinyobj::ObjReaderConfig reader_config;
    reader_config.mtl_search_path = "";

    tinyobj::ObjReader reader;

    if (!reader.ParseFromFile((std::string(ASSET_PATH) + path), reader_config)) 
    {
        if (!reader.Error().empty())
            std::cerr << "TinyObjReader error: " << reader.Error() << std::endl;
        ASSERT(false, "Failed to load OBJ file.");
    }

    if (!reader.Warning().empty())
    {
        std::cout << "TinyObjReader warning: " << reader.Warning() << std::endl;
        ASSERT(false, "Failed to load OBJ file.");
    }

    const tinyobj::attrib_t& attrib             = reader.GetAttrib();
    const std::vector<tinyobj::shape_t>& shapes = reader.GetShapes();

    std::unordered_map<Vertex, unsigned int, VertexHash> uniqueVertices;

    for (const auto& shape : shapes) 
    {
        for (const auto& index : shape.mesh.indices) 
        {
            Vertex v;
            v.x = attrib.vertices[3*index.vertex_index+0];
            v.y = attrib.vertices[3*index.vertex_index+1];
            v.z = attrib.vertices[3*index.vertex_index+2];

            v.nx = (index.normal_index>=0) ? attrib.normals[3*index.normal_index+0] : 0.f;
            v.ny = (index.normal_index>=0) ? attrib.normals[3*index.normal_index+1] : 0.f;
            v.nz = (index.normal_index>=0) ? attrib.normals[3*index.normal_index+2] : 0.f;

            if (uniqueVertices.count(v) == 0) 
            {
                uniqueVertices[v] = vertices.size()/VERTEX_DIMENSION;
                vertices.push_back(v.x);
                vertices.push_back(v.y);
                vertices.push_back(v.z);
                vertices.push_back(v.nx);
                vertices.push_back(v.ny);
                vertices.push_back(v.nz);
            }

            indices.push_back(uniqueVertices[v]);
        }
    }

    // std::cout << vertices.size() << ", " << indices.size() << "\n";
}

Mesh* loadOBJ(const std::string& path) 
{
    std::vector<float>        vertices;
    std::vector<unsigned int> indices;

    loadOBJVerticesIndicesData(path, vertices, indices);

    return new Mesh(vertices.data(), vertices.size(), indices.data(), indices.size(), GL_STATIC_DRAW);
}

Mesh loadOBJStatic(const std::string& path) 
{
    std::vector<float>        vertices;
    std::vector<unsigned int> indices;

    loadOBJVerticesIndicesData(path, vertices, indices);

    return Mesh(vertices.data(), vertices.size(), indices.data(), indices.size(), GL_STATIC_DRAW);
}

Mesh* createAnimatedMesh(std::vector<float>& vertices, std::vector<unsigned int>& indices, FrameIndex num_frames, GLuint indices_per_frame)
{
    Mesh* mesh = new Mesh(vertices.data(), vertices.size(), indices.data(), indices.size(), GL_STATIC_DRAW, MeshType::animated);
    mesh->animated_data.indices_per_frame = indices_per_frame;
    mesh->animated_data.current_frame     = 0;
    mesh->animated_data.total_frames      = num_frames;

    return mesh;
}

Mesh* loadAnimatedOBJ(
    const std::string& directory_path,
    const std::string& prefix,
    FrameIndex num_frames)
{
    std::vector<float>        allVertices;
    std::vector<unsigned int> allIndices;
    GLuint indices_per_frame = 0;

    for (FrameIndex frame = 0; frame < num_frames; ++frame)
    {
        std::vector<float>        frameVertices;
        std::vector<unsigned int> frameIndices;

        std::string filename = directory_path + "\\" + prefix + "_" + (frame + 1 < 10 ? "0" : "") + std::to_string(frame + 1) + ".obj";

        loadOBJVerticesIndicesData(filename, frameVertices, frameIndices);

        if (frame == 0)
            indices_per_frame = static_cast<GLuint>(frameIndices.size());
        else
            ASSERT(frameIndices.size() == indices_per_frame, "All animation frames must have the same number of indices!");

        GLuint vertexOffset = static_cast<GLuint>(allVertices.size() / VERTEX_DIMENSION); // TODO: check this number, number of float to define vertex
        for (auto idx : frameIndices)
            allIndices.push_back(idx + vertexOffset);

        allVertices.insert(allVertices.end(), frameVertices.begin(), frameVertices.end());
    }

    return createAnimatedMesh(allVertices, allIndices, num_frames, indices_per_frame);
}
