#pragma once


/**
 * OpenGL camera that can be moved around with mouse. It supports panning,
 * zooming and rotating.
 */
class Camera {
public:
    void setPosition(const glm::vec3& position) {
        mView = glm::translate(glm::mat4(), -position);
    }

    glm::mat4 getTransform() {
        return mProjection * mView;
    }

    void resize(float width, float height) {
        mProjection = glm::perspective(45.0f, width/height, 0.001f, 10000.0f);
    }

private:
    glm::mat4 mView, mProjection;
};
