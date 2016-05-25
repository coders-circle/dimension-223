#pragma once


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
    void init();
    void draw();
    void finish();

private:
    // Camera mCamera;
    // std::vector<Model> mModels;

};
