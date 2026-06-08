#include "VertexArray.h"
#include <glad/glad.h>

VertexArray::VertexArray() {
    glGenVertexArrays(1, &m_RendererID);
}

VertexArray::~VertexArray() {
    glDeleteVertexArrays(1, &m_RendererID);
}

void VertexArray::bind() const {
    glBindVertexArray(m_RendererID);
}

void VertexArray::unbind() const {
    glBindVertexArray(0);
}

void VertexArray::addVertexBuffer(const VertexBuffer& vb, const VertexBufferLayout& layout) {
    bind();
    vb.bind();

    const auto& elements = layout.getElements();
    unsigned int offset = 0;

    for (auto& element : elements) {
        glEnableVertexAttribArray(m_CurrentAttrib);
        glVertexAttribPointer(
            m_CurrentAttrib,
            element.count,
            element.type,
            element.normalized,
            layout.getStride(),
            (const void*)offset
        );
        offset += element.count * VertexBufferElement::getSizeOfType(element.type);
        m_CurrentAttrib++;
    }

    m_VertexBuffers.push_back(vb);
}

void VertexArray::setIndexBuffer(const IndexBuffer& ib) {
    bind();
    ib.bind();
    m_IndexBuffer = ib;
}
