#version 430 core

in vec2 TexCoords;

out vec4 FragColor;

void main()
{
    float alpha_max =       smoothstep(0.9, 1.0, TexCoords.x) * 0.5;
    float alpha_min = 1.0 - smoothstep(0.0, 0.1, TexCoords.x) * 0.5;

    FragColor = vec4(0.2, 0.5, 1.0, alpha_max + alpha_min - 0.75);
}