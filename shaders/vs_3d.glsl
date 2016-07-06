#version 330 core

layout(location = 0) in vec3 position;
layout(location = 1) in vec3 normal;
layout(location = 2) in vec2 texcoords;

uniform mat4 model;
uniform mat4 viewProjection;

out vec3 fNormal;
out vec2 fTexcoords;

void main()
{
    gl_Position = viewProjection * model * vec4(position, 1.0);
    fNormal = mat3(transpose(inverse(model))) * normal;
    fTexcoords = texcoords;
}
