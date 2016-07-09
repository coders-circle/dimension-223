#pragma once
#include <backend/CloudStitcher.h>

#include <Project.h>
#include <ui/SFMLWidget.h>

#include <physics/World.h>
#include <graphics/InputHandler.h>


/**
 * Central graphics class that handles all OpenGL rendering stuffs.
 *
 * Major functions include: initializing OpenGL states, handling camera,
 * handling drawing of models and initializing and destroying OpenGL objects.
 *
 * The methods (init, draw, finish) are called appropriately by the
 * application.
 */
class Renderer {
public:
    Renderer(SFMLWidget& viewport, Project& project);

    ~Renderer() {
        finish();
    }

    void init();
    void draw();
    void resize(int width, int height);
    void finish();

    void setSelectionChangedCallback(
        const std::function<void(size_t)>& callback
    ) {
        mInputHandler.setSelectionChangedCallback(callback);
    }

private:
    SFMLWidget& mViewport;
    Project& mProject;

    Program* mSimpleProgram;
    Program* m3dProgram;

    InputHandler mInputHandler;

    // PointCloud* mTestCloud1;
    // PointCloud* mTestCloud2;

    // CloudStitcher* mStitcher;

};
