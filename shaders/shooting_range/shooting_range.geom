#version 430 core
layout (points) in;
layout (triangle_strip, max_vertices = 4) out;  
 
in float vAngle[]; 
in float vNextAngle[]; 

out vec2 TexCoords;

uniform float uRadiusMin;
uniform float uRadiusMax;

uniform mat4 projection;
uniform mat4 view;

void emitQuadVertex(vec4 pos, vec2 offset, vec2 uv) 
{
    TexCoords   = uv;
    gl_Position = projection * view * (pos + vec4(offset, 0.0, 0.0));
    EmitVertex();
}

void main() 
{
    vec4 center = gl_in[0].gl_Position;

    float a1 = vAngle[0];
    float a2 = vNextAngle[0];

    vec2 dir1 = vec2(cos(a1), sin(a1));
    vec2 dir2 = vec2(cos(a2), sin(a2));

    emitQuadVertex(center, dir1 * uRadiusMin, vec2(0.0, 0.0)); 
    emitQuadVertex(center, dir1 * uRadiusMax, vec2(1.0, 0.0)); 
    emitQuadVertex(center, dir2 * uRadiusMin, vec2(0.0, 1.0)); 
    emitQuadVertex(center, dir2 * uRadiusMax, vec2(1.0, 1.0)); 

    EndPrimitive();
}

