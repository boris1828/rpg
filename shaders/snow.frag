#version 430 core
out vec4 FragColor;
in vec2 TexCoords;

void main() 
{
    float dist = distance(TexCoords, vec2(0.5, 0.5));

    float alpha = 1.0 - smoothstep(0.2, 0.5, dist);

    if (alpha <= 0.0) discard;

    FragColor = vec4(1.0, 1.0, 1.0, alpha);
}