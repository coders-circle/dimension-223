#version 330 core

layout(location = 0) in vec3 position;
layout(location = 1) in vec2 texcoords;

uniform mat4 model;
uniform mat4 viewProjection;

out vec2 tcoords;

void main()
{
    gl_Position = viewProjection * model * vec4(position, 1.0);
    tcoords = texcoords;
}
