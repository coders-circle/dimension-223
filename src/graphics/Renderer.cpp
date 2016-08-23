#include <stdinc.h>
#include <graphics/Renderer.h>
#include <backend/LensBlurImage.h>


Renderer::Renderer(GlxWidget& viewport, Project& project)
    : mViewport(viewport), mProject(project), mInputHandler(project)
{}


void Renderer::init() {
    glClearColor(0.1803f, 0.1803f,0.1803f, 1.0f);
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

    for (size_t i=0; i<mProject.getNumPointClouds(); ++i) {
        mProject.getPointCloud(i).draw(*m3dProgram, //*mSimpleProgram,
            mProject.getCamera().getTransformation());
    }

    for (size_t i=0; i<mProject.getNumModels(); ++i) {
        mProject.getModel(i).update();
        mProject.getModel(i).draw(*m3dProgram,
            mProject.getCamera().getTransformation());
    }
}

void Renderer::resize(int width, int height) {
    glViewport(0, 0, width, height);
    mProject.getCamera().resize(width, height);
}

void Renderer::finish() {

}
