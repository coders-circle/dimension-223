#pragma once

// Inline outerProduct function.
// TODO: For any dimension.
inline glm::mat3 outerProduct(const glm::vec3& v1, const glm::vec3& v2) {
    glm::mat3 temp;
    for (unsigned int i=0; i<3; ++i)
        for (unsigned int j=0; j<3; ++j)
            temp[i][j] = v1[i] * v2[j];

    return temp;
}

// Singular Value Decomposition:
// m = u * e * v
void svd(const glm::mat3& m, glm::mat3& u, glm::mat3& e, glm::mat3& v);

// Centroid of a point cloud.
// C = Sum(v)/len(v)
inline glm::vec3 getCentroid(const std::vector<glm::vec3>& points) {
    glm::vec3 sum(0.0f, 0.0f, 0.0f);
    for (const glm::vec3& v: points) {
        sum += v;
    }
    return 1.0f/points.size() * sum;
}
