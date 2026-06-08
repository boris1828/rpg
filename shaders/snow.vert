#version 430 core

layout (std430, binding = 0) buffer SnowBuffer 
{
    vec4 staticOffsets[]; // x, y, z, swaySpeed
};

uniform float time;
uniform vec3 cameraPos;
uniform vec3 boxSize = vec3(8.0, 8.0, 10.0);
uniform vec2 wind    = vec2(1.0, 0.0);

void main() 
{
    vec4 data = staticOffsets[gl_InstanceID];
    
    vec3 pos   = data.xyz;
    float sway = data.w;

    pos.z  -= time * 0.6 * (mod(pos.x, 0.5) + 0.5); 
    pos.x  += sin(time * sway) * 0.2;
    pos.y  += cos(time * (sway + (mod(gl_InstanceID, 10) - 5) * 0.1 )) * 0.2;
    pos.xy += wind * time * 0.5;

    pos = mod(pos - cameraPos + boxSize/2.0, boxSize) + cameraPos - boxSize/2.0;

    gl_Position = vec4(pos, 1.0);
}