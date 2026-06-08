#version 430 core
layout (points) in;
layout (triangle_strip, max_vertices = 4) out;

in float vLife_VS[]; 
out float vLife;    

uniform mat4 projection;
uniform mat4 view;
float size = 0.02;

void emitQuadVertex(vec4 pos, vec2 offset, float life) {
    vLife = life;
    gl_Position = projection * view * (pos + vec4(offset, 0.0, 0.0));
    EmitVertex();
}

void main() {
    vec4 pos = gl_in[0].gl_Position;
    float life = vLife_VS[0];

    emitQuadVertex(pos, vec2(-size, -size), life);
    emitQuadVertex(pos, vec2( size, -size), life);
    emitQuadVertex(pos, vec2(-size,  size), life);
    emitQuadVertex(pos, vec2( size,  size), life);

    EndPrimitive();
}