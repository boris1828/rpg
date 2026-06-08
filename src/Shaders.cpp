#include "Shaders.h"
#include "Macro.h"

#include <glad/glad.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <string_view>
#include <filesystem>

namespace fs = std::filesystem;

static constexpr std::string_view getShaderName(unsigned int type) 
{
    switch (type) 
    {
        case GL_VERTEX_SHADER:          return "VERTEX";          
        case GL_FRAGMENT_SHADER:        return "FRAGMENT";        
        case GL_GEOMETRY_SHADER:        return "GEOMETRY";        
        case GL_TESS_CONTROL_SHADER:    return "TESS_CONTROL";    
        case GL_TESS_EVALUATION_SHADER: return "TESS_EVALUATION"; 
        case GL_COMPUTE_SHADER:         return "COMPUTE";         
        default:                        return "UNKNOWN";
    }
}

unsigned int Shader::compileShader(const std::string& path, GLenum type) 
{
    std::string code = loadFile(std::string(SHADER_PATH) + path);
    const char* shaderCode = code.c_str();

    unsigned int shader = glCreateShader(type);
    glShaderSource(shader, 1, &shaderCode, nullptr);
    glCompileShader(shader);
    checkCompileErrors(shader, getShaderName(type));
    
    return shader;
}

// Now your constructors are very slim:
Shader::Shader(const std::string& vertexPath, const std::string& fragmentPath) {
    unsigned int v = compileShader(vertexPath, GL_VERTEX_SHADER);
    unsigned int f = compileShader(fragmentPath, GL_FRAGMENT_SHADER);

    ID = glCreateProgram();
    glAttachShader(ID, v);
    glAttachShader(ID, f);
    glLinkProgram(ID);
    checkCompileErrors(ID, "PROGRAM");

    glDeleteShader(v);
    glDeleteShader(f);
}

Shader::Shader(const std::string& vertexPath, const std::string& geometryPath, const std::string& fragmentPath)
{
    unsigned int v = compileShader(vertexPath, GL_VERTEX_SHADER);
    unsigned int g = compileShader(geometryPath, GL_GEOMETRY_SHADER);
    unsigned int f = compileShader(fragmentPath, GL_FRAGMENT_SHADER);

    ID = glCreateProgram();
    glAttachShader(ID, v);
    glAttachShader(ID, g);
    glAttachShader(ID, f);
    glLinkProgram(ID);
    checkCompileErrors(ID, "PROGRAM");

    glDeleteShader(v);
    glDeleteShader(g);
    glDeleteShader(f);
}

Shader::Shader(std::initializer_list<ShaderSource> sources) 
{
    ID = glCreateProgram();
    std::vector<unsigned int> compiledShaders;

    for (const auto& source : sources) 
    {
        unsigned int s = compileShader(source.path, source.type);
        glAttachShader(ID, s);
        compiledShaders.push_back(s);
    }

    glLinkProgram(ID);
    checkCompileErrors(ID, "PROGRAM");

    for (unsigned int s : compiledShaders) 
    {
        glDeleteShader(s);
    }
}

Shader::Shader(const std::string& dir_name) 
{
    std::string full_path = std::string(SHADER_PATH) + dir_name;

    name = dir_name;
    
    if (!fs::exists(full_path) || !fs::is_directory(full_path)) 
    {
        ASSERT(false, ("Shader directory not found: " + full_path).c_str());
        return;
    }

    ID = glCreateProgram();
    std::vector<unsigned int> compiledShaders;

    struct ExtensionMap {
        const char* ext;
        GLenum type;
    };

    ExtensionMap allowed_extensions[] = {
        { ".vert", GL_VERTEX_SHADER },
        { ".frag", GL_FRAGMENT_SHADER },
        { ".geom", GL_GEOMETRY_SHADER }
    };

    bool found_any = false;

    for (const auto& mapping : allowed_extensions) 
    {
        std::string expected_file = dir_name + "/" + dir_name + mapping.ext;
        std::string absolute_path = std::string(SHADER_PATH) + expected_file;

        if (fs::exists(absolute_path)) 
        {
            unsigned int s = compileShader(expected_file, mapping.type);
            glAttachShader(ID, s);
            compiledShaders.push_back(s);
            found_any = true;
        }
    }

    if (found_any) 
    {
        glLinkProgram(ID);
        checkCompileErrors(ID, "PROGRAM");
    }
    else 
    {
        ASSERT(false, ("No valid shader files found in directory: " + dir_name).c_str());
    }

    for (unsigned int s : compiledShaders) 
    {
        glDeleteShader(s);
    }
}

Shader::~Shader() {
    glDeleteProgram(ID);
}

void Shader::use() const {
    glUseProgram(ID);
}

#if 0
void Shader::checkError(const char* func) const 
{
    GLenum err;
    while ((err = glGetError()) != GL_NO_ERROR) 
    {
        std::string error;
        switch (err) {
            case GL_INVALID_ENUM:                  error = "INVALID_ENUM"; break;
            case GL_INVALID_VALUE:                 error = "INVALID_VALUE"; break;
            case GL_INVALID_OPERATION:             error = "INVALID_OPERATION"; break;
            case GL_OUT_OF_MEMORY:                 error = "OUT_OF_MEMORY"; break;
            case GL_INVALID_FRAMEBUFFER_OPERATION:  error = "INVALID_FRAMEBUFFER_OPERATION"; break;
            default:                               error = "UNKNOWN"; break;
        }
        std::cerr << "OpenGL Error in " << func << " [" << error << "] for Shader ID: " << ID << ", name: " << name << std::endl;
    }
}
#else
void Shader::checkError(const char* func) const 
{

}
#endif

// static uint32_t cache_miss = 0;
// static uint32_t cache_hit  = 0;

GLint Shader::getLoc(std::string_view loc_name) 
{
    ASSERT(loc_name.data()[loc_name.size()] == '\0', "string_view must be null-terminated for OpenGL!");

    // if (ID == 1)
    //     std::cout << "hit: " << cache_hit << ", miss: " << cache_miss << "\n";

    const char* c_str = loc_name.data();

    auto loc = cache.tryGet(c_str);

    if (loc.has_value()) return *loc;

    GLint location = glGetUniformLocation(ID, loc_name.data());

    if (location == -1) 
    {
        std::cerr << "Warning: Uniform '" << loc_name << "' not found or optimized out in Shader ID: " << ID << ", name: " << name << std::endl;
        return location;
    }

    cache.set(c_str, location);

    return location;
}

void Shader::setFloat(std::string_view name, float value) 
{
    glUniform1f(getLoc(name), value);
    checkError(__func__);
}

void Shader::setBool(std::string_view name, bool value) 
{
    glUniform1i(getLoc(name), (int)value);
    checkError(__func__);
}

void Shader::setInt(std::string_view name, int value) 
{
    glUniform1i(getLoc(name), value);
    checkError(__func__);
}

void Shader::setVec2(std::string_view name, const glm::vec2 &value) 
{
    glUniform2fv(getLoc(name), 1, &value[0]);
    checkError(__func__);
}

void Shader::setVec3(std::string_view name, const glm::vec3 &value) 
{
    glUniform3fv(getLoc(name), 1, &value[0]);
    checkError(__func__);
}

void Shader::setMat4(std::string_view name, const glm::mat4 &mat) 
{
    glUniformMatrix4fv(getLoc(name), 1, GL_FALSE, &mat[0][0]);
    checkError(__func__);
}

void Shader::bindTexture(std::string_view name, unsigned int textureID, int unit)
{
    glActiveTexture(GL_TEXTURE0 + unit);
    glBindTexture(GL_TEXTURE_2D, textureID);
    setInt(name, unit);
}

// Helpers
std::string Shader::loadFile(const std::string& path) 
{
    std::ifstream file(path);
    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

void Shader::checkCompileErrors(unsigned int shader, const std::string_view& type) 
{
    int success;
    char infoLog[1024];
    if (type != "PROGRAM") {
        glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
        if (!success) {
            glGetShaderInfoLog(shader, 1024, nullptr, infoLog);
            std::cerr << "ERROR::SHADER_COMPILATION_ERROR of type: " << type
                      << "\n" << infoLog << std::endl;
        }
    } 
    else 
    {
        glGetProgramiv(shader, GL_LINK_STATUS, &success);
        if (!success) {
            glGetProgramInfoLog(shader, 1024, nullptr, infoLog);
            std::cerr << "ERROR::PROGRAM_LINKING_ERROR\n" << infoLog << std::endl;
        }
    }
}
