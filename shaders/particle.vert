#version 430 core

struct Particle 
{
    vec4 pos_life; // x, y, z, life
    vec4 old;      // spawnWorldX, spawnWorldY, unused, unused
};

layout(std430, binding = 0) buffer ParticleBuffer 
{
    Particle particles[];
};

uniform float deltaTime;
uniform vec3  currentSpawnPos;
uniform float velocity = 0.8;

out float vLife_VS;

void main() 
{
    Particle p = particles[gl_VertexID];
    
    p.pos_life.w += deltaTime; 

    if (p.pos_life.w > 1.0) 
    {
        p.pos_life.w = 0.0; // Reset life
        p.pos_life.z = 0.0; // Reset local height
    }

    particles[gl_VertexID] = p; 

    vec3 currentPos  = p.pos_life.xyz + vec3(0.0, 0.0, p.pos_life.w * velocity) + currentSpawnPos;

    vLife_VS    = p.pos_life.w;
    gl_Position = vec4(currentPos, 1.0);
}