#pragma once
#include <glad/glad.h>

class VertexBuffer {
public:
    VertexBuffer() = default;
    VertexBuffer(const void* data, unsigned int size, GLenum usage = GL_STATIC_DRAW);
    ~VertexBuffer();

    void bind() const;
    void unbind() const;

    void updateData(const void* data, unsigned int size);

private:
    unsigned int m_RendererID;
};