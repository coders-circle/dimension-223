#pragma once

/**
 * Perspective camera with position, rotation, field-of-view angle,
 * near plane and far plane defined.
 *
 * It can be used to calculate view projection matrix out of its parameters.
 * Camera::Resize(width, height) must be called at least once to Initialize
 * the camera matrices.
 */
class Camera
{
public:

    /**
     * Create camera object with given perspective parameters.
     * @param fov Field of view as angle.
     * @param near Near plane distance.
     * @param far Far plane distance.
     */
    Camera(float fov=glm::degrees(120.0f), float near=0.1f, float far=1000.0f)
        : m_fov(fov), m_near(near), m_far(far)
    {}

    /**
     * Resize the camera to adjust its aspect ratio according to viewport.
     * @param width  Width of the viewport.
     * @param height Height of the viewport.
     */
    void Resize(int width, int height)
    {
        m_aspect = static_cast<float>(width)/height;
        m_projection = glm::perspective(m_fov, m_aspect, m_near, m_far);
    }

    /**
     * Get the perspective projection matrix.
     * @return Projection matrix represented by the camera.
     */
    glm::mat4 GetProjection() const { return m_projection; }

    /**
     * Get the world transformation matrix.
     * @return Transformation matrix that transforms to camera's position
     *         and rotation.
     */
    glm::mat4 GetTransform() const
    {
        return glm::translate(glm::mat4(), position) *
            glm::mat4_cast(rotation);
    }

    /**
     * Get the view transformation matrix.
     * @return View transformation matrix represented by the camera.
     */
    glm::mat4 GetView() const
    { return glm::affineInverse(GetTransform()); }

    /**
     * Get the view-projection matrix.
     * @return Product of View and Projection matrices represented
     *         by the camera.
     */
    glm::mat4 GetViewProjection() const
    { return m_projection * GetView(); }

    /// Position of the camera in the world.
    glm::vec3 position;

    /// Rotation of the camera in the world.
    glm::quat rotation;

private:
    // Projection parameters.
    float m_fov, m_near, m_far, m_aspect;
    glm::mat4 m_projection;
};
