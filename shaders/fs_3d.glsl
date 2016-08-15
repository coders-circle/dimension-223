#version 330 core

out vec4 color;

in vec3 fNormal;
in vec2 fTexcoords;

uniform sampler2D uTexture;

void main()
{
    color = texture(uTexture, fTexcoords);

    vec3 lightDir = vec3(1, 0, 1);
    float intensity = max(0.0, 0.9+dot(normalize(fNormal), normalize(lightDir)));
    color = color * intensity;
}
