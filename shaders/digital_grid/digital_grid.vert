#version 430 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec2 aUV;

uniform mat4  model;
uniform mat4  view;
uniform mat4  projection;
uniform float gridStepSize;
uniform bool  horizontal;
uniform int   numLayers;

out vec3 vWorldPos;
out vec3 vLocalPos;
out vec2 vUV;

void main() 
{
    float layerOffset = float(gl_InstanceID) / float(numLayers - 1);

    vec3 localPos = aPos;

    if (horizontal) localPos.z = layerOffset;
    else            localPos.y = layerOffset;

    vec4 worldPos = model * vec4(localPos, 1.0);

    // if (horizontal) worldPos.z += layerOffset;
    // else            worldPos.y += layerOffset;

    vWorldPos = worldPos.xyz;
    vLocalPos = localPos;
    vUV       = aUV;

    gl_Position = projection * view * worldPos;
}