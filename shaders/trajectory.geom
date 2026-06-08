#version 430 core
layout (points) in;
layout (triangle_strip, max_vertices = 4) out;  

in float vProgress_VS[]; 
in vec4 vNextPos[]; 

out float vProgress;   
out vec2 TexCoords;

uniform vec3  uLaunchDir;

uniform mat4 projection;
uniform mat4 view;
float size = 0.05;

void emitQuadVertex(vec4 pos, vec2 offset, float progress, vec2 uv) 
{
    TexCoords   = uv;
    vProgress   = progress;
    gl_Position = projection * view * (pos + vec4(offset, 0.0, 0.0));
    EmitVertex();
}

void main() 
{
    vec4 pos       = gl_in[0].gl_Position;
    vec4 next_pos  = vNextPos[0];
    float progress = vProgress_VS[0];

    vec2 offset = normalize(uLaunchDir.yx) * size; // * (progress + 0.5);
    offset.y = -offset.y;

    emitQuadVertex(pos,      -offset, progress, vec2(0.0, 0.0));
    emitQuadVertex(pos,       offset, progress, vec2(1.0, 0.0));
    emitQuadVertex(next_pos, -offset, progress, vec2(0.0, 1.0));
    emitQuadVertex(next_pos,  offset, progress, vec2(1.0, 1.0));

    EndPrimitive();
}