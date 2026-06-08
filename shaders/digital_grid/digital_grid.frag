#version 430 core
out vec4 FragColor;

in vec3 vWorldPos;
in vec3 vLocalPos;
in vec2 vUV;

uniform float gridStepSize;
uniform sampler2D noiseTexture;
uniform float time;

float edgeSoftness = 0.1;

vec2 worldPos = vec2(0.0);
vec2 localPos = vec2(0.0);

vec3 color_dark  = vec3(0.1, 0.2, 1.0);
vec3 color_light = vec3(0.7, 0.9, 0.9);

// vec3 color_dark  = vec3(0.2, 0.8, 0.3);
// vec3 color_light = vec3(0.8, 0.9, 0.7);

void main() 
{
    vec3 fractPos  = abs(mod(vWorldPos, gridStepSize) - (gridStepSize * 0.5));
    vec3 edgeDist  = 1.0 - (fractPos / (gridStepSize * 0.5));
    vec3 gridLines = smoothstep(0.8, 1.0, edgeDist); 
 
    float alpha    = max(gridLines.z, max(gridLines.x, gridLines.y));

    float color_int = alpha * alpha;
    vec3 color      = color_dark * (1-color_int)  + color_light * (color_int);

    vec3 edgeAlpha = smoothstep(0.0, edgeSoftness, vLocalPos) * (1.0 - smoothstep(1.0 - edgeSoftness, 1.0, vLocalPos));
    float boxMask  = edgeAlpha.x * edgeAlpha.y * edgeAlpha.z;

    vec2 noiseUV    = vUV * 0.1 + (time * 0.1);
    noiseUV        += vLocalPos.z * 0.1; 
    float noiseVal  = texture(noiseTexture, noiseUV).r;
    noiseVal        = smoothstep(0.4, 0.9, noiseVal);

    FragColor = vec4(color, clamp(0.0, 1.0, alpha * boxMask * noiseVal));
}