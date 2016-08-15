precision mediump float;

varying vec2 fTexcoords;
varying vec3 fNormal;

uniform sampler2D uTexture;

void main() {
    gl_FragColor = texture2D(uTexture, fTexcoords);
}