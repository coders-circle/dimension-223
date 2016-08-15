attribute vec3 position;
attribute vec3 normal;
attribute vec2 texcoords;

uniform mat4 model;
uniform mat4 mvp;

varying vec2 fTexcoords;
varying vec3 fNormal;

void main() {
    gl_Position = mvp * vec4(position, 1.0);
    fNormal = mat3(model) * normal;
    fTexcoords = texcoords;
}