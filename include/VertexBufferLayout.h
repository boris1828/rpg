#pragma once
#include <vector>
#include <glad/glad.h>

struct VertexBufferElement {
    unsigned int type;     // GL_FLOAT, GL_UNSIGNED_INT, ecc.
    unsigned int count;    // quante componenti (es: 3 per vec3)
    unsigned char normalized; // GL_TRUE o GL_FALSE

    static unsigned int getSizeOfType(unsigned int type) {
        switch (type) {
            case GL_FLOAT: return sizeof(float);
            case GL_UNSIGNED_INT: return sizeof(unsigned int);
            case GL_UNSIGNED_BYTE: return sizeof(unsigned char);
        }
        return 0;
    }
};

class VertexBufferLayout {
public:
    VertexBufferLayout() : m_Stride(0) {}

    template<typename T>
    void push(unsigned int count);

    inline const std::vector<VertexBufferElement>& getElements() const { return m_Elements; }
    inline unsigned int getStride() const { return m_Stride; }

private:
    std::vector<VertexBufferElement> m_Elements;
    unsigned int m_Stride;
};
