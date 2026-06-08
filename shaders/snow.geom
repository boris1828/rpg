#version 430 core
layout (points) in;
layout (triangle_strip, max_vertices = 4) out;  

uniform mat4 projection;
uniform mat4 view;
float size = 0.02;

out vec2 TexCoords;

void emitQuadVertex(vec4 pos, vec2 offset, vec2 uv) 
{
    vec4 projectedPos = projection * view * pos;
    vec4 screenOffset = 2.0 * vec4(offset, 0.0, 0.0);
    
    TexCoords   = uv; 
    gl_Position = projectedPos + screenOffset;
    EmitVertex();
}

void main() 
{
    vec4 pos = gl_in[0].gl_Position;

    emitQuadVertex(pos, vec2(-size, -size), vec2(0.0, 0.0));
    emitQuadVertex(pos, vec2( size, -size), vec2(1.0, 0.0));
    emitQuadVertex(pos, vec2(-size,  size), vec2(0.0, 1.0));
    emitQuadVertex(pos, vec2( size,  size), vec2(1.0, 1.0));

    EndPrimitive();
}