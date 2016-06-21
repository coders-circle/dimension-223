#include <stdinc.h>


// Singular Value Decomposition:
// m = u * e * v
// Use opencv SVD::compute method but with glm matrices.
void svd(const glm::mat3& m, glm::mat3& u, glm::mat3& e, glm::mat3& v) {
    cv::Matx33f ocm(
        m[0][0], m[0][1], m[0][2],
        m[1][0], m[1][1], m[1][2],
        m[2][0], m[2][1], m[2][2]
    );
    cv::Mat ocu, oce, ocv;
    cv::SVD::compute(ocm, oce, ocu, ocv);

    u = glm::mat3(
        ocu.at<float>(0, 0), ocu.at<float>(0, 1), ocu.at<float>(0, 2),
        ocu.at<float>(1, 0), ocu.at<float>(1, 1), ocu.at<float>(1, 2),
        ocu.at<float>(2, 0), ocu.at<float>(2, 1), ocu.at<float>(2, 2)
    );

    e = glm::mat3(
        ocu.at<float>(0, 0), 0, 0,
        0, ocu.at<float>(0, 1), 0,
        0, 0, ocu.at<float>(0, 2)
    );

    v = glm::transpose(glm::mat3(
        ocv.at<float>(0, 0), ocv.at<float>(0, 1), ocv.at<float>(0, 2),
        ocv.at<float>(1, 0), ocv.at<float>(1, 1), ocv.at<float>(1, 2),
        ocv.at<float>(2, 0), ocv.at<float>(2, 1), ocv.at<float>(2, 2)
    ));
}
