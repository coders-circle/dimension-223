#version 330 core

out vec4 color;

in vec3 fNormal;
in vec2 fTexcoords;

uniform sampler2D uTexture;

void main()
{
    color = texture(uTexture, fTexcoords);

    vec3 lightDir = vec3(1, 1, 0);
    float intensity = max(1.0, 0.2+dot(normalize(fNormal), normalize(lightDir)));
    color = color * intensity;
}
