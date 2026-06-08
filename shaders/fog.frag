#version 430 core
out vec4 FragColor;

in vec3 vWorldPos;
in vec3 vLocalPos;
in vec2 vUV;

uniform vec3  playerPos;
uniform float visibilityRadius = 1.5;
uniform float fogAlpha         = 1.0; 
uniform vec3  fogColor         = vec3(0.9, 0.9, 1.0);
uniform float edgeSoftness     = 0.1;

uniform vec3 wind = vec3(0.05, 0.1, 0.0);

uniform float time;

uniform sampler2D depthTexture;
uniform sampler2D noiseTexture;
uniform vec2 screenSize;

// TOCHECK: if fog doesnt work, check these values
float near = 0.1; 
float far  = 100.0;

float LinearizeDepth(float depth) 
{
    float z = depth * 2.0 - 1.0;
    return (2.0 * near * far) / (far + near - z * (far - near));	
}

void main() 
{
    // 1. Soft Intersection (Depth Check)
    vec2 screenUV       = gl_FragCoord.xy / screenSize;
    float sceneDepthRaw = texture(depthTexture, screenUV).r;
    
    float sceneDepth = LinearizeDepth(sceneDepthRaw);
    float fogDepth   = LinearizeDepth(gl_FragCoord.z);
    
    float diff       = sceneDepth - fogDepth;
    float softMask   = clamp(diff / 1.0, 0.0, 1.0);

    // 2. Existing Masks

    float d          = distance(vWorldPos.xy, playerPos.xy);
    float playerMask = smoothstep(visibilityRadius * 0.6, visibilityRadius, d);

    vec2 edgeAlpha = smoothstep(0.0, edgeSoftness, vLocalPos.xy) * (1.0 - smoothstep(1.0 - edgeSoftness, 1.0, vLocalPos.xy));
    float boxMask  = edgeAlpha.x * edgeAlpha.y;

    // 3. Noise texture

    vec2 noiseUV    = vUV + (time * 0.05);
    noiseUV        += vLocalPos.z * 0.1; 
    float noiseVal  = texture(noiseTexture, noiseUV).r;
    noiseVal        = smoothstep(0.3, 0.5, noiseVal);

    // 4. Final Output

    if(diff < 0.0) discard;

    float finalAlpha = fogAlpha * noiseVal * boxMask * softMask * playerMask;
    FragColor        = vec4(fogColor, finalAlpha); 

}