#pragma once
#include <string>
#include <map>
#include <glm/glm.hpp>
#include <glad/glad.h>

#include "Utility.h"

struct ShaderSource 
{
    GLenum type;
    std::string path;
};

class Shader {
public:
    unsigned int ID;
    std::string name;

    Shader(const std::string& vertexPath, const std::string& fragmentPath);
    Shader(const std::string& vertexPath, const std::string& geometryPath, const std::string& fragmentPath);
    Shader(std::initializer_list<ShaderSource> sources);
    Shader(const std::string& dir_name);

    ~Shader();

    void use() const;

    // Funzioni per impostare uniform
    void setBool(std::string_view name, bool value);
    void setInt(std::string_view name, int value);
    void setFloat(std::string_view name, float value);
    void setVec2(std::string_view name, const glm::vec2 &value);
    void setVec3(std::string_view name, const glm::vec3 &value);
    void setMat4(std::string_view name, const glm::mat4 &mat);
    void bindTexture(std::string_view name, unsigned int textureID, int unit);

private:
    // std::map<const char*, GLint> cache;

    StaticMap<const char*, GLint, 32> cache;


    GLint getLoc(std::string_view name);
    void checkError(const char* func) const;
    static std::string loadFile(const std::string& path);
    static void checkCompileErrors(unsigned int shader, const std::string_view& type);

    unsigned int compileShader(const std::string& path, GLenum type);
};

