#pragma once
#include <graphics/PointCloud.h>
#include <graphics/Model.h>
#include <graphics/Camera.h>
#include <backend/CloudStitcher.h>


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
    Renderer();

    ~Renderer() {
        finish();
    }

    void init();
    void draw();
    void resize(int width, int height);
    void finish();

private:
    Camera mCamera;

    Program* mSimpleProgram;
    Program* m3dProgram;

    Model* mTestModel;
    PointCloud* mTestCloud1;
    PointCloud* mTestCloud2;

    CloudStitcher* mStitcher;
};
