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

    // LensBlurImage* lbi = new LensBlurImage("img/test9.jpg");
    //
    // mTestCloud1 = new PointCloud(lbi->getImage(), lbi->getDepthMap());

    cv::Mat p1 = cv::imread("img/p1.bmp");
    cv::Mat p1d = cv::imread("img/p1d.bmp");

    cv::Mat p2 = cv::imread("img/p1.bmp");
    cv::Mat p2d = cv::imread("img/p1d.bmp");

    mTestCloud1 = new PointCloud(p1, p1d);
    mTestCloud2 = new PointCloud(p2, p2d);
    mCamera.setPosition(glm::vec3(0, 0, 1.4f));

    mStitcher = new CloudStitcher(*mTestCloud1, *mTestCloud2);
    mStitcher->setTransformation(glm::mat3(glm::rotate(glm::mat4(), glm::radians(20.0f), glm::vec3(0,1,0))), glm::vec3(0));
    mStitcher->stitch();
}

void Renderer::draw() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    static float angle = 0.0f;
    angle += 0.01f;
    // glm::mat4 model = glm::rotate(glm::mat4(), angle, glm::vec3(0, 1, 0));
    // mTestCloud1->draw(*mSimpleProgram, model, mCamera.getTransform());
    // mTestCloud2->draw(*mSimpleProgram, model, mCamera.getTransform());

    mCamera.setTransform(glm::vec3(0, 0, 1.4f), 0, angle, 0);

    mTestCloud1->draw(*mSimpleProgram, glm::mat4(), mCamera.getTransform());
    mTestCloud2->draw(*mSimpleProgram, mStitcher->getTransformation(),
        mCamera.getTransform());
}

void Renderer::resize(int width, int height) {
    glViewport(0, 0, width, height);
    mCamera.resize(width, height);
}

void Renderer::finish() {

}
