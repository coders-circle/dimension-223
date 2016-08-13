attribute vec3 position;
attribute vec2 texcoords;

uniform mat4 mvp;

varying vec2 fTexcoords;

void main() {
    gl_Position = mvp * vec4(position, 1.0);
    fTexcoords = texcoords;
}