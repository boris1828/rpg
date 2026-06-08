#version 330 core

out vec4 FragColor;

in vec3 FragPos;
in vec3 Normal;

uniform vec3 lightDir;
uniform float uR;  
uniform float uG;  
uniform float uB; 
uniform float uAlpha; 

uniform float ambientStrength;
uniform float diffuseStrength;

void main()
{
    vec3 color = vec3(uR, uG, uB);

    vec3 N = normalize(Normal);
    vec3 L = normalize(-lightDir);

    float diff = max(dot(N, L), 0.0);

    vec3 ambient = ambientStrength * color;
    vec3 diffuse = diffuseStrength * diff * color;

    FragColor = vec4(ambient + diffuse, uAlpha);
}