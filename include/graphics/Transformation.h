#pragma once


struct Transformation {
    glm::vec3 position;
    glm::quat rotation;
    glm::vec3 scale;

    Transformation()
    : scale(1)
    {}

    glm::mat4 getMatrix() const {
        return glm::translate(glm::mat4(), position)
            * glm::mat4_cast(rotation) * glm::scale(glm::mat4(), scale);
    }
};
