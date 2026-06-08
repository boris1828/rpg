#pragma once
#include <glad/glad.h>

class IndexBuffer {
public:
    
    IndexBuffer(const unsigned int* indices, unsigned int count);
    IndexBuffer();

    ~IndexBuffer();

    void bind() const;  
    void unbind() const; 

    unsigned int getCount() const { return m_Count; }

private:
    unsigned int m_RendererID;
    unsigned int m_Count;      
};
