#pragma once


struct Transformation {
    glm::vec3 position;
    glm::quat rotation;     // Euler YXZ: yaw, pitch, roll
    glm::vec3 scale;

    Transformation()
    : scale(1)
    {}

    glm::mat4 getMatrix() const {
        return glm::translate(glm::mat4(), position)
            * glm::mat4_cast(rotation)
            // * glm::eulerAngleYXZ(rotation.y, rotation.x, rotation.z)
            * glm::scale(glm::mat4(), scale);
    }

    void setMatrix(const glm::mat4& transform) {
        position = glm::vec3(transform[3]);
        rotation = glm::quat_cast(transform);
    }
};
