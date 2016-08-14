#include <stdinc.h>
#include <graphics/Renderer.h>
#include <backend/LensBlurImage.h>


Renderer::Renderer(GlxWidget& viewport, Project& project)
    : mViewport(viewport), mProject(project), mInputHandler(project)
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

    mViewport.setScrollCallback([this](float dx, float dy) {
        mInputHandler.scroll(dx, dy);
    });

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
    mProject.getPhysicsWorld().update();

    int x, y;
    mViewport.get_pointer(x, y);

    mInputHandler.update(
        x, y,
        mViewport.isLeftMouseDown(),
        mViewport.isMiddleMouseDown(),
        mViewport.isRightMouseDown(),
        mViewport.getMouseState()
    );

    // static float angle = 0.0f;
    // angle += 0.01f;
    // mProject.getCamera().setTransformation(glm::vec3(0, 0, 1.4f), 0, angle, 0);

    for (size_t i=0; i<mProject.getNumPointClouds(); ++i) {
        mProject.getPointCloud(i).draw(*mSimpleProgram,
            mProject.getCamera().getTransformation());
    }

    for (size_t i=0; i<mProject.getNumModels(); ++i) {
        mProject.getModel(i).update();
        mProject.getModel(i).draw(*m3dProgram,
            mProject.getCamera().getTransformation());
    }
    // mTestCloud1->draw(*mSimpleProgram, glm::mat4(),
    //     mProject.getCamera().getTransformation());
    // mTestCloud2->draw(*mSimpleProgram, mStitcher->getTransformation(),
    //     mProject.getCamera().getTransformation());
}

void Renderer::resize(int width, int height) {
    glViewport(0, 0, width, height);
    mProject.getCamera().resize(width, height);
}

void Renderer::finish() {

}
