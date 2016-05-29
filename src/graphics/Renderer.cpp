#include <stdinc.h>
#include <graphics/Renderer.h>


void Renderer::init() {
    glClearColor(0.396f, 0.612f, 0.937f, 1.0f);
    glEnable(GL_DEPTH_TEST);
}

void Renderer::draw() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void Renderer::resize(int width, int height) {
    glViewport(0, 0, width, height);
}

void Renderer::finish() {

}
