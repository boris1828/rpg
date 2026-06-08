#version 430 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec2 aUV;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
uniform int numLayers;

out vec3 vWorldPos;
out vec3 vLocalPos;
out vec2 vUV;

void main() 
{
    float layerOffset = (float(gl_InstanceID) / float(numLayers - 1)); // - 0.5;
    
    vec3 localPos = aPos;
    localPos.z    = layerOffset; 

    vec4 worldPos = model * vec4(localPos, 1.0);

    vWorldPos = worldPos.xyz;
    vLocalPos = localPos;
    vUV       = aUV;
    
    gl_Position = projection * view * worldPos;
}