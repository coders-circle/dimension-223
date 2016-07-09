#pragma once


/**
 * OpenGL camera that can be moved around with mouse. It supports panning,
 * zooming and rotating.
 */
class Camera {
public:
    Camera()
    : mPosition(0, 0, 1.4f), mTarget(0), mUp(0, 1, 0)
    {}
    // void setTransformation(const glm::vec3& position,
    //                   float angleX, float angleY, float angleZ) {
    //     mView =
    //         glm::translate(glm::mat4(), -position)
    //         * glm::rotate(glm::mat4(), angleX, glm::vec3(1, 0, 0))
    //         * glm::rotate(glm::mat4(), angleY, glm::vec3(0, 1, 0))
    //         * glm::rotate(glm::mat4(), angleZ, glm::vec3(0, 0, 1));
    // }

    void view(const glm::vec3& position, const glm::vec3& target,
        const glm::vec3& up)
    {
        mPosition = position;
        mTarget = target;
        mUp = up;
        mView = glm::lookAt(mPosition, mTarget, mUp);
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

    /**
     * Transform any window space coordinates in [0, w/h] range to normalized
     * viewport coordinates in [-1 to 1] range.
     * @param  pos Window space coordinates where x is in [0, width] and y is
     *             in [0, height].
     * @return     Normalized viewport coordinates with both x and y in [-1, 1]
     *             range.
     */
    glm::vec2 normalize(const glm::vec2& pos) const {
        return glm::vec2(
            (pos.x/mWidth-0.5f)*2.0f,
            -(pos.y/mHeight-0.5f)*2.0f
        );
    }

    /**
     * Inverse transform any NDC point to 3D point.
     * @param  pos A point in normalized device coordinates space.
     * @return     The 3D point obtained by inverse transforming the given
     *             point using this camera.
     */
    glm::vec3 inverse(const glm::vec3& pos) const {
        glm::vec2 npos = normalize(glm::vec2(pos));
        glm::vec4 ray(npos.x, npos.y, pos.z, 1.0f);
        ray = glm::inverse(getTransformation()) * ray;
        ray /= ray.w;
        return glm::vec3(ray);
    }

    const glm::vec3& getPosition() const {
        return mPosition;
    }

    const glm::vec3& getTarget() const {
        return mTarget;
    }

    const glm::vec3& getUp() const {
        return mUp;
    }

private:

    glm::vec3 mPosition, mTarget, mUp;

    glm::mat4 mView, mProjection;
    float mWidth, mHeight;
};
