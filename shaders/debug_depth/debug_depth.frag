#version 430 core
out vec4 FragColor;
in vec2 TexCoords;

uniform sampler2D depthTexture;
uniform float near       = 0.1;
uniform float far        = 100.0;
uniform float debugRange = 20.0;

float LinearizeDepth(float depth) 
{
    float z = depth * 2.0 - 1.0;
    return (2.0 * near * far) / (far + near - z * (far - near));
}

void main() 
{             
    float depthRaw = texture(depthTexture, TexCoords).r;
    
    float linearDepth = LinearizeDepth(depthRaw);
    
    float visualization = linearDepth / debugRange;

    FragColor = vec4(vec3(visualization), 1.0);
}