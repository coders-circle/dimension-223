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

    glm::mat4 getTransformation() const {
        return mProjection * mView;
    }

    void resize(float width, float height) {
        mProjection = glm::perspective(45.0f, width/height, 0.001f, 10000.0f);
        mWidth = width;
        mHeight = height;
    }

    float getWidth() const { return mWidth; }
    float getHeight() const { return mHeight; }
    glm::vec2 normalize(const glm::vec2& pos) const {
        return glm::vec2((pos.x/mWidth-0.5f)*2.0f, -(pos.y/mHeight-0.5f)*2.0f);
    }

    glm::vec3 inverse(const glm::vec3& pos) const {
        glm::vec2 npos = normalize(glm::vec2(pos));
        glm::vec4 ray(npos.x, npos.y, pos.z, 1.0f);
        ray = glm::inverse(getTransformation()) * ray;
        ray /= ray.w;
        return glm::vec3(ray);
    }

    glm::vec3 getNormal() const {
        return glm::vec3(mView[2]);
    }

    glm::vec3 getUp() const {
        return glm::vec3(mView[1]);
    }

    glm::vec3 getRight() const {
        return glm::vec3(mView[0]);
    }

private:
    glm::mat4 mView, mProjection;
    float mWidth, mHeight;
};
