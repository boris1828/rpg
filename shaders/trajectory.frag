#version 430 core

in float vProgress;
in vec2 TexCoords;

out vec4 FragColor;

void main() 
{
    vec3 color = vec3(1.0, 0.4, 0.0);
    
    float prog_alpha = 1.0 - (vProgress); 

    float cent_alpha = 1.0 - abs(TexCoords.x - 0.5)*2.0 ;
    
    FragColor = vec4(color, prog_alpha * cent_alpha);
}