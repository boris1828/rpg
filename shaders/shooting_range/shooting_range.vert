#version 430 core
#define M_PI 3.1415926535897932384626433832795

uniform vec3 uCenter;
uniform int  uSegments;

out float vAngle;
out float vNextAngle;

void main() 
{
    int i = gl_VertexID;
    
    vAngle      = (float(i)   / float(uSegments - 1)) * 2.0 * M_PI;
    vNextAngle  = (float(i+1) / float(uSegments - 1)) * 2.0 * M_PI;
    
    gl_Position = vec4(uCenter, 1.0);
}