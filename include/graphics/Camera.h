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

    void setTransformation(const glm::vec3& position,
                      float angleX, float angleY, float angleZ) {

        mView =
            glm::translate(glm::mat4(), -position)
            * glm::rotate(glm::mat4(), angleX, glm::vec3(1, 0, 0))
            * glm::rotate(glm::mat4(), angleY, glm::vec3(0, 1, 0))
            * glm::rotate(glm::mat4(), angleZ, glm::vec3(0, 0, 1));
    }

    glm::mat4 getTransformation() {
        return mProjection * mView;
    }

    void resize(float width, float height) {
        mProjection = glm::perspective(45.0f, width/height, 0.001f, 10000.0f);
    }

private:
    glm::mat4 mView, mProjection;
};
