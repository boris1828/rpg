#version 330 core

out vec4 FragColor;

in vec3 FragPos;

uniform vec3 color;  
uniform float alpha;

void main()
{
    FragColor = vec4(color, alpha);
}