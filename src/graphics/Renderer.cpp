#include <stdinc.h>
#include <graphics/Renderer.h>
#include <backend/LensBlurImage.h>


Renderer::Renderer(Project& project)
    : mProject(project)
{}


void Renderer::init() {
    glClearColor(0.396f, 0.612f, 0.937f, 1.0f);
    glEnable(GL_DEPTH_TEST);

    mSimpleProgram = new Program(
        Shader("shaders/vs_simple.glsl", GL_VERTEX_SHADER),
        Shader("shaders/fs_simple.glsl", GL_FRAGMENT_SHADER)
    );
    m3dProgram = new Program(
        Shader("shaders/vs_3d.glsl", GL_VERTEX_SHADER),
        Shader("shaders/fs_3d.glsl", GL_FRAGMENT_SHADER)
    );

    mProject.addModel("models/homer.x");
    mProject.getModel(0).transformation.scale = glm::vec3(0.02);
    mProject.addPointCloud("img/test9.jpg");
    
    mCamera.setPosition(glm::vec3(0, 0, 1.4f));

    // cv::Mat p1 = cv::imread("img/p1.bmp");
    // cv::Mat p1d = cv::imread("img/p1d.bmp");
    //
    // cv::Mat p2 = cv::imread("img/p2.bmp");
    // cv::Mat p2d = cv::imread("img/p2d.bmp");
    //
    // mTestCloud1 = new PointCloud(p1, p1d);
    // mTestCloud2 = new PointCloud(p2, p2d);

    // mStitcher = new CloudStitcher(*mTestCloud1, *mTestCloud2);
    // mStitcher->setTransformation(glm::mat3(glm::rotate(glm::mat4(), glm::radians(30.0f), glm::vec3(0,1,0))), glm::vec3(0.5f));
    // mStitcher->stitch();
}

void Renderer::draw() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    static float angle = 0.0f;
    angle += 0.01f;

    mCamera.setTransformation(glm::vec3(0, 0, 1.4f), 0, angle, 0);

    for (size_t i=0; i<mProject.getNumPointClouds(); ++i) {
        mProject.getPointCloud(i).draw(*mSimpleProgram,
            mCamera.getTransformation());
    }

    for (size_t i=0; i<mProject.getNumModels(); ++i) {
        mProject.getModel(i).draw(*m3dProgram, mCamera.getTransformation());
    }
    // mTestCloud1->draw(*mSimpleProgram, glm::mat4(),
    //     mCamera.getTransformation());
    // mTestCloud2->draw(*mSimpleProgram, mStitcher->getTransformation(),
    //     mCamera.getTransformation());
}

void Renderer::resize(int width, int height) {
    glViewport(0, 0, width, height);
    mCamera.resize(width, height);
}

void Renderer::finish() {

}
