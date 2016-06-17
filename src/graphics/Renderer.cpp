#include <stdinc.h>
#include <graphics/Renderer.h>
#include <backend/LensBlurImage.h>


Renderer::Renderer()
{}


void Renderer::init() {
    glClearColor(0.396f, 0.612f, 0.937f, 1.0f);
    glEnable(GL_DEPTH_TEST);

    mSimpleProgram = new Program(
        Shader("shaders/vs_simple.glsl", GL_VERTEX_SHADER),
        Shader("shaders/fs_simple.glsl", GL_FRAGMENT_SHADER)
    );

    LensBlurImage* lbi = new LensBlurImage("img/test9.jpg");

    mTestCloud = new PointCloud(lbi->getImage(), lbi->getDepthMap());
    mCamera.setPosition(glm::vec3(0, 0, 1.4f));
}

void Renderer::draw() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    static float angle = 0.01f;
    angle += 0.01f;
    glm::mat4 model = glm::rotate(glm::mat4(), angle, glm::vec3(0, 1, 0));
    mTestCloud->draw(*mSimpleProgram, model, mCamera.getTransform());
}

void Renderer::resize(int width, int height) {
    glViewport(0, 0, width, height);
    mCamera.resize(width, height);
}

void Renderer::finish() {

}
