#include "VertexBufferLayout.h"
#include <glad/glad.h>

template<>
void VertexBufferLayout::push<float>(unsigned int count) {
    m_Elements.push_back({ GL_FLOAT, count, GL_FALSE });
    m_Stride += count * sizeof(float);
}

template<>
void VertexBufferLayout::push<unsigned int>(unsigned int count) {
    m_Elements.push_back({ GL_UNSIGNED_INT, count, GL_FALSE });
    m_Stride += count * sizeof(unsigned int);
}

template<>
void VertexBufferLayout::push<unsigned char>(unsigned int count) {
    m_Elements.push_back({ GL_UNSIGNED_BYTE, count, GL_TRUE });
    m_Stride += count * sizeof(unsigned char);
}
