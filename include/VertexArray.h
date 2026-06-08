#pragma once
#include <vector>
#include <optional>
#include "VertexBuffer.h"
#include "VertexBufferLayout.h"
#include "IndexBuffer.h"

class VertexArray {
public:
    VertexArray();
    ~VertexArray();

    void addVertexBuffer(const VertexBuffer& vb, const VertexBufferLayout& layout);
    void setIndexBuffer(const IndexBuffer& ib);

    void bind() const;
    void unbind() const;

private:
    unsigned int m_RendererID;
    std::vector<VertexBuffer> m_VertexBuffers;
    IndexBuffer m_IndexBuffer;
    unsigned int m_CurrentAttrib = 0;  // tiene traccia delle location disponibili
};
