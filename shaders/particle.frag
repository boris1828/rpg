#version 430 core

out vec4 FragColor;
in float vLife;

uniform float time;

void main() {

    float baseAlpha  = 1.0 - vLife*0.8;

    // float flicker    = sin(time * 20.0 + vLife * 100.0) * 0.5 + 0.5;
    // float finalAlpha = baseAlpha * mix(0.4, 1.0, flicker);

    FragColor = vec4(0.6, 0.1, 0.8, baseAlpha);
}