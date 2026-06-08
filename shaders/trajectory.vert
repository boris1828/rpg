#version 430 core

uniform vec3  uStartPos;  // x0
uniform vec3  uLaunchDir; // Final normalized direction * v0
uniform float uGravity;   // g
uniform float uMaxTime;   // te (Total time of flight)
uniform int   uSegments;  // CROSSHAIR_LINE_SEGMENTS

out float vProgress_VS;
out vec4 vNextPos;

void main() 
{
    int i = gl_VertexID;
    
    vProgress_VS = float(i) / float(uSegments);

    float t = uMaxTime * vProgress_VS;

    vec3 pos = uStartPos + (uLaunchDir * t) + vec3(0.0, 0.0, -0.5 * uGravity * t * t);

    float progress_next = float(i+1) / float(uSegments);
    float t_next        = uMaxTime * progress_next;
    vec3 next_pos       = uStartPos + (uLaunchDir * t_next) + vec3(0.0, 0.0, -0.5 * uGravity * t_next * t_next);

    vNextPos    = vec4(next_pos, 1.0);
    gl_Position = vec4(pos, 1.0);
}