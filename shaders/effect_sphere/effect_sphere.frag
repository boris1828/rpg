#version 430 core
out vec4 FragColor;

in vec3 vNormal;
in vec3 vWorldPos;

uniform sampler2D depthTexture;
uniform vec3 color;
uniform vec2 screenSize;
uniform vec3 cameraPos;

uniform float near = 0.1; 
uniform float far = 100.0;
uniform float margin = 0.1;

float linearize(float depth) 
{
    float z = depth * 2.0 - 1.0;
    return (2.0 * near * far) / (far + near - z * (far - near));
}

void main() 
{
    // --- 1. Soft Intersection Logic ---
    vec2 uv = gl_FragCoord.xy / screenSize;
    float sceneDepthLinear = linearize(texture(depthTexture, uv).r);
    float sphereDepthLinear = linearize(gl_FragCoord.z);
    float diff = sceneDepthLinear - sphereDepthLinear;
    float intersectionFactor = clamp(1.0 - (diff / margin), 0.0, 1.0);
    if (diff < 0.0) intersectionFactor = 0.0;

    // --- 2. Fresnel Logic ---
    vec3 viewDir = normalize(cameraPos - vWorldPos);
    vec3 normal  = normalize(vNormal);
    
    // dot(n, v) is 1.0 when looking straight at the surface, 0.0 at edges
    float viewDotNormal = abs(dot(normal, viewDir));
    
    // Invert it so edges are 1.0 and center is 0.0
    float fresnel = 1.0 - clamp(viewDotNormal, 0.0, 1.0);
    
    // Make the effect sharper/rounder with power
    fresnel = pow(fresnel, 3.0); 

    // --- 3. Final Composition ---
    // Combine intersection glow and fresnel glow
    float finalAlpha = max(intersectionFactor, fresnel);
    
    // Apply a base transparency so the center isn't totally invisible
    finalAlpha = clamp(finalAlpha, 0.0, 1.0);

    FragColor = vec4(color, finalAlpha);     
}
