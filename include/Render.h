#pragma once

#include <iostream>
#include <random>
#include <string>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

// #define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include "Types.h"
#include "Macro.h"
#include "Shaders.h"

struct SceneBuffer 
{
    unsigned int fbo;
    unsigned int colorTex;
    unsigned int depthTex;
    int width, height;

    SceneBuffer(int width, int height) : width(width), height(height)
    {
        glGenFramebuffers(1, &fbo);
        glBindFramebuffer(GL_FRAMEBUFFER, fbo);

        glGenTextures(1, &colorTex);
        glBindTexture(GL_TEXTURE_2D, colorTex);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, colorTex, 0);

        glGenTextures(1, &depthTex);
        glBindTexture(GL_TEXTURE_2D, depthTex);
        // glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, width, height, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, width, height, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depthTex, 0);

        // glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_NONE); // CRITICAL

        if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
            std::cout << "ERROR::FRAMEBUFFER:: Framebuffer is not complete!" << std::endl;

        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

    ~SceneBuffer() 
    {
        if (colorTex != 0) glDeleteTextures(1, &colorTex);
        if (depthTex != 0) glDeleteTextures(1, &depthTex);

        if (fbo != 0) glDeleteFramebuffers(1, &fbo);
    }

    void reset()
    {
        glBindFramebuffer(GL_FRAMEBUFFER, fbo);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    }

    void drawToScreen()
    {
        glBindFramebuffer(GL_READ_FRAMEBUFFER, fbo);
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0); 

        glBlitFramebuffer(
            0, 0, width, height, 
            0, 0, width, height, 
            GL_COLOR_BUFFER_BIT, 
            GL_NEAREST          
        );
    }

    void dumpDepth() 
    {
        glBindFramebuffer(GL_READ_FRAMEBUFFER, fbo);
        float depthValue = 0;

        glReadPixels(width / 4, height / 4, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depthValue);
        
        std::cout << "Depth at center: " << depthValue << std::endl;
    }

    Real2 getSize() const 
    {
        return Real2(static_cast<Real>(width), static_cast<Real>(height));
    }

    // void drawDepthToScreen()
    // {
    //     glBindFramebuffer(GL_READ_FRAMEBUFFER, fbo);
    //     glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0); 

    //     glReadBuffer(GL_DEPTH_ATTACHMENT); 

    //     glBlitFramebuffer(
    //         0, 0, width, height, 
    //         0, 0, width, height, 
    //         GL_DEPTH_BUFFER_BIT, 
    //         GL_NEAREST          
    //     );

    //     glReadBuffer(GL_COLOR_ATTACHMENT0);
    // }
};

struct RenderStateGuard 
{
    RenderStateGuard() 
    {

    }

    ~RenderStateGuard() 
    {
        glEnable(GL_DEPTH_TEST);
        glDepthMask(GL_TRUE);
        glEnable(GL_CULL_FACE);
        glCullFace(GL_BACK);
        
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glBlendEquation(GL_FUNC_ADD);
    }

    void setAdditiveBlending() 
    {
        glBlendFunc(GL_SRC_ALPHA, GL_ONE);
    }

    void setNoDepthWrite() 
    {
        glDepthMask(GL_FALSE);
    }

    void disableCulling() 
    {
        glDisable(GL_CULL_FACE);
    }
};

struct TextureManager 
{
private:
    TextureManager()  {}
    ~TextureManager() {}

    TextureManager(const TextureManager&) = delete;
    TextureManager& operator=(const TextureManager&) = delete;

    std::unordered_map<std::string, unsigned int> textures;

public:
    
    static TextureManager& Get() 
    {
        static TextureManager instance; 
        return instance;
    }

    unsigned int GetTexture(const std::string& fileName) 
    {
        auto it = textures.find(fileName);
        if (it != textures.end()) return it->second;

        int width, height, nrChannels;
        std::string path = std::string(ASSET_PATH) + fileName;
        unsigned char *data = stbi_load(path.c_str(), &width, &height, &nrChannels, 0);

        if (!data) 
        {
            ASSERT(false, "Failed to load texture!");
            return 0;
        }

        unsigned int textureID;
        glGenTextures(1, &textureID);
        glBindTexture(GL_TEXTURE_2D, textureID);

        GLenum format = (nrChannels == 4) ? GL_RGBA : (nrChannels == 3 ? GL_RGB : GL_RED);
        
        glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data);
        glGenerateMipmap(GL_TEXTURE_2D);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        stbi_image_free(data);
        textures[fileName] = textureID;
        return textureID;
    }

    void cleanup() 
    {
        for (auto const& [name, id] : textures) 
            if (id != 0) glDeleteTextures(1, &id);
        textures.clear();
    }
};

struct PointBatchRenderer 
{
    static void draw(int count) 
    {
        static unsigned int global_empty_vao = 0;
        if (global_empty_vao == 0) glGenVertexArrays(1, &global_empty_vao);

        glBindVertexArray(global_empty_vao);
        glDrawArrays(GL_POINTS, 0, count);
        glBindVertexArray(0);
    }
};

constexpr float vertices_zplane[] = {
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        1.0f, 0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f, 1.0f,
        1.0f, 1.0f, 0.0f, 1.0f, 1.0f
    };

constexpr float vertices_yplane[] = {
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        1.0f, 0.0f, 0.0f, 0.0f, 1.0f,
        0.0f, 0.0f, 1.0f, 1.0f, 0.0f,
        1.0f, 0.0f, 1.0f, 1.0f, 1.0f
    };

constexpr float vertices_xplane[] = {
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f, 1.0f,
        0.0f, 0.0f, 1.0f, 1.0f, 0.0f,
        0.0f, 1.0f, 1.0f, 1.0f, 1.0f
    };

struct InstancedPlane 
{
    unsigned int vao, vbo;

    InstancedPlane(const float* vertices, size_t size) 
    {
        glGenVertexArrays(1, &vao);
        glGenBuffers(1, &vbo);
        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, size, vertices, GL_STATIC_DRAW);

        // Standard 5-float stride (Pos:3, UV:2)
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
        glEnableVertexAttribArray(1);
    }

    ~InstancedPlane()
    {
        if (vao != 0) glDeleteVertexArrays(1, &vao);
        if (vbo != 0) glDeleteBuffers(1, &vbo);
    }

    void draw(int num_instances) const 
    {
        glBindVertexArray(vao);
        glDrawArraysInstanced(GL_TRIANGLE_STRIP, 0, 4, num_instances);
        glBindVertexArray(0);
    }
};

struct InstancedPlanes 
{
    static InstancedPlane& getX() { static InstancedPlane p(vertices_xplane, sizeof(vertices_xplane)); return p; }
    static InstancedPlane& getY() { static InstancedPlane p(vertices_yplane, sizeof(vertices_yplane)); return p; }
    static InstancedPlane& getZ() { static InstancedPlane p(vertices_zplane, sizeof(vertices_zplane)); return p; }
};


// =======================================================
// RENDERERS
// =======================================================

#define MAX_NUMBER_PARTICLES 10

struct Particle
{
    Real x, y, z, life;
    Real ox, oy, oz, padding2;
};

struct ParticleSystem 
{
    unsigned int ssbo;

    ParticleSystem()
    {
        ssbo = 0;
    }   

    void init(Real3 pos)
    {
        ASSERT(ssbo == 0, "When initializing particle system it must be ampty");

        // TODO: singleton CPU random generation system

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<float> dis(-0.1f, 0.1f);    // Range for X and Y
        std::uniform_real_distribution<float> lifeDis(0.0f, 1.0f); // Range for Life

        int particleCount = MAX_NUMBER_PARTICLES;
        std::vector<Particle> particles(particleCount);

        for (int i = 0; i < particleCount; i++) 
        {
            particles[i].x     = dis(gen);
            particles[i].y     = dis(gen);
            particles[i].z     = 0.0f;
            particles[i].life  = lifeDis(gen);
            particles[i].ox    = particles[i].x;
            particles[i].oy    = particles[i].y;
            particles[i].oz    = 0.0;
        }

        glGenBuffers(1, &ssbo);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo);
        glBufferData(GL_SHADER_STORAGE_BUFFER, particles.size() * sizeof(Particle), particles.data(), GL_DYNAMIC_DRAW);

        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, ssbo);
    }

    void destroy()
    {
        ASSERT(ssbo != 0, "Destroy called on uninitialized ParticleSystem!");

        glDeleteBuffers(1, &ssbo);
    }

    ~ParticleSystem() 
    {
        if (ssbo != 0) glDeleteBuffers(1, &ssbo);
    }

    void update(Shader& shader, Real delta_time) const
    {
        ASSERT(ssbo != 0, "Update called on uninitialized ParticleSystem!");

        shader.use();
        shader.setFloat("deltaTime", delta_time);
        shader.setFloat("time", glfwGetTime());
    }

    void draw(Shader& shader, Real3 spawn_pos) const
    {
        ASSERT(ssbo != 0, "Draw called on uninitialized ParticleSystem!");

        // glEnable(GL_BLEND);
        // glBlendFunc(GL_SRC_ALPHA, GL_ONE);
        glDepthMask(GL_FALSE);

        shader.use();

        shader.setVec3("currentSpawnPos", spawn_pos);
       
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, ssbo);
        glDrawArrays(GL_POINTS, 0, MAX_NUMBER_PARTICLES);

        glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

        // glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDepthMask(GL_TRUE);
    }
};

// TODO: snow effect: motion blur
// TODO: snow effect: fade when is about to hut something: sample depth buffer
// TODO: snow effect: fade when particle is close to camera position.z

#define SNOW_PARTICLES 1000

struct SnowflakeData 
{
    float x, y, z, swaySpeed;
};

struct SnowRenderer
{
    unsigned int ssbo;
    unsigned int vao;
    
    SnowRenderer() : ssbo(0)
    {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<float> disXY(-4.0f, 4.f);  
        std::uniform_real_distribution<float> disZ(0.0f, 10.f);
        std::uniform_real_distribution<float> disSpeed(0.5f, 2.0f);

        std::vector<SnowflakeData> snowOffsets(5000);
        for(auto& s : snowOffsets) 
        {
            s.x         = disXY(gen);
            s.y         = disXY(gen);
            s.z         = disZ(gen);
            s.swaySpeed = disSpeed(gen);
        }

        glGenVertexArrays(1, &vao);
        glGenBuffers(1, &ssbo);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo);
        glBufferData(GL_SHADER_STORAGE_BUFFER, snowOffsets.size() * sizeof(SnowflakeData), snowOffsets.data(), GL_STATIC_DRAW);
    }

    ~SnowRenderer()
    {
        if (ssbo != 0) glDeleteBuffers(1, &ssbo);
        if (vao != 0)  glDeleteVertexArrays(1, &vao);
    }

    void draw(Shader& shader, Real3 camera_pos) const
    {
        ASSERT(ssbo != 0, "Draw called on uninitialized ParticleSystem!");

        shader.use();

        shader.setFloat("time", glfwGetTime());
        shader.setVec3("cameraPos", camera_pos);

        glDepthMask(GL_FALSE);

        glBindVertexArray(vao);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, ssbo);
        glDrawArraysInstanced(GL_POINTS, 0, 1, SNOW_PARTICLES);

        glDepthMask(GL_TRUE);
    }
};





struct TrajectoryRenderer
{
    static constexpr int segments = 50;

    void draw(Shader& shader, const Real3& x0, const Real3& direction, Real v0, Real te, Real g, bool is_parabolic = true) 
    {
        shader.use();
        
        shader.setVec3("uStartPos",  x0);
        shader.setVec3("uLaunchDir", direction * v0);
        shader.setFloat("uGravity",  g);
        shader.setFloat("uMaxTime",  te);
        shader.setInt("uSegments",   is_parabolic ? segments : 2);

        RenderStateGuard state; 
        state.setNoDepthWrite();

        PointBatchRenderer::draw(segments);
    }
};

struct ShootingRangeRenderer
{
    static constexpr int segments = 50;

    void draw(Shader& shader, const Real3& shooter_position, Real radius_min, Real radius_max) 
    {
        shader.use();
        
        shader.setVec3("uCenter",     shooter_position);
        shader.setFloat("uRadiusMin", radius_min);
        shader.setFloat("uRadiusMax", radius_max);
        shader.setInt("uSegments",    segments);

        RenderStateGuard state; 
        state.setNoDepthWrite();

        PointBatchRenderer::draw(segments-1);
    }
};


struct EffectSphereRenderer
{
    Mesh sphere_mesh = loadOBJStatic("sphere.obj");

    void draw(Shader& shader, const Real3& position, Real radius, const Real3& color, const Real3& camera_position, const SceneBuffer& scene_buffer) 
    {
        glm::mat4 model(1.0f);

        model = glm::translate(model, position);
        model = glm::scale(model, Real3(radius));

        shader.use();

        shader.bindTexture("depthTexture", scene_buffer.depthTex, 0);

        shader.setVec2("screenSize", scene_buffer.getSize());
        shader.setMat4("model",      model);
        shader.setVec3("color",      color);
        shader.setVec3("cameraPos",  camera_position);

        RenderStateGuard state; 
        state.setNoDepthWrite();
        state.disableCulling();

        sphere_mesh.draw(shader);
    }
};


struct FogRenderer
{
    unsigned int noiseTexture;

    FogRenderer()
    {
        noiseTexture = TextureManager::Get().GetTexture("seamless_perlin_noise.png");
    }

    void draw(Shader& shader, int num_layers, Real3 fog_pos, Real3 fog_size, Real3 player_pos, const SceneBuffer& scene_buffer) 
    {
        glm::mat4 model(1.0f);

        model = glm::translate(model, fog_pos);
        model = glm::scale(model, fog_size);

        shader.use();

        shader.bindTexture("depthTexture", scene_buffer.depthTex, 0);
        shader.bindTexture("noiseTexture", noiseTexture, 1);

        shader.setVec2("screenSize", glm::vec2((float)scene_buffer.width, (float)scene_buffer.height));

        shader.setFloat("time", glfwGetTime());

        shader.setVec3("playerPos", player_pos);
        shader.setMat4("model",     model);
        shader.setInt("numLayers",  num_layers);

        glDepthMask(GL_FALSE);

        InstancedPlanes::getZ().draw(num_layers);

        glDepthMask(GL_TRUE);
    }
};

struct DigitalGridRenderer
{
    static constexpr Real gridStepSize = 0.5f;

    unsigned int noiseTexture;

    DigitalGridRenderer()
    {
        noiseTexture = TextureManager::Get().GetTexture("seamless_perlin_noise.png");
    }

    void draw(Shader& shader, Real3 grid_pos, Real3 grid_size)
    {
        int hor_num_layers = grid_size.z / gridStepSize;
        int ver_num_layers = grid_size.y / gridStepSize;

        ASSERT(hor_num_layers > 0, "need at least a layer");
        ASSERT(ver_num_layers > 0, "need at least a layer");

        glm::mat4 model(1.0f);
        model = glm::translate(model, grid_pos);
        model = glm::scale(model, Real3(grid_size.x, grid_size.y, grid_size.z));

        shader.use();

        shader.setMat4("model", model);
        shader.setFloat("gridStepSize", gridStepSize);
        shader.bindTexture("noiseTexture", noiseTexture, 0);
        shader.setFloat("time", glfwGetTime());

        RenderStateGuard state; 
        state.setNoDepthWrite();
        state.disableCulling();
        state.setAdditiveBlending();

        shader.setBool("horizontal", true);
        shader.setInt("numLayers", hor_num_layers);
        InstancedPlanes::getZ().draw(hor_num_layers);

        shader.setBool("horizontal", false);
        shader.setInt("numLayers", ver_num_layers);
        InstancedPlanes::getY().draw(ver_num_layers);
    }
};




struct DebugDepthRenderer
{
    unsigned int vao, vbo;
    Shader shader;

    DebugDepthRenderer() : shader("debug_depth")
    {
        float vertices[] = 
        {
            -1.0f, -1.0f, 0.0f,      0.0f, 0.0f,
             1.0f, -1.0f, 0.0f,      1.0f, 0.0f,
            -1.0f,  1.0f, 0.0f,      0.0f, 1.0f,

            -1.0f,  1.0f, 0.0f,      0.0f, 1.0f,
             1.0f, -1.0f, 0.0f,      1.0f, 0.0f,
             1.0f,  1.0f, 0.0f,      1.0f, 1.0f
        };

        glGenVertexArrays(1, &vao);
        glGenBuffers(1, &vbo);

        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

        int stride = 5 * sizeof(float); 

        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, (void*)0);

        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, stride, (void*)(3 * sizeof(float)));
    }

    ~DebugDepthRenderer()
    {
        if (vao != 0) glDeleteVertexArrays(1, &vao);
        if (vbo != 0) glDeleteBuffers(1, &vbo);
    }

    void draw(const SceneBuffer& scene_buffer)
    {
        shader.use();

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, scene_buffer.depthTex);
        shader.setInt("depthTexture", 0);

        glDisable(GL_DEPTH_TEST);
        glDepthMask(GL_FALSE);
        
        glBindVertexArray(vao);
        glDrawArrays(GL_TRIANGLES, 0, 6);

        glDepthMask(GL_TRUE);
        glEnable(GL_DEPTH_TEST);
    }
};