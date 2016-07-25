#include <stdinc.h>


// Singular Value Decomposition:
// m = u * e * v
// Use opencv SVD::compute method but with glm matrices.
void svd(const glm::mat3& m, glm::mat3& u, glm::mat3& e, glm::mat3& v) {
    cv::Matx33d ocm(
        m[0][0], m[0][1], m[0][2],
        m[1][0], m[1][1], m[1][2],
        m[2][0], m[2][1], m[2][2]
    );
    cv::Mat ocu, oce, ocv;
    cv::SVD::compute(ocm, oce, ocu, ocv);

    u = glm::mat3(
        (float)ocu.at<double>(0, 0), (float)ocu.at<double>(0, 1), (float)ocu.at<double>(0, 2),
        (float)ocu.at<double>(1, 0), (float)ocu.at<double>(1, 1), (float)ocu.at<double>(1, 2),
        (float)ocu.at<double>(2, 0), (float)ocu.at<double>(2, 1), (float)ocu.at<double>(2, 2)
    );

    e = glm::mat3(
        (float)ocu.at<double>(0, 0), 0, 0,
        0, (float)ocu.at<double>(0, 1), 0,
        0, 0, (float)ocu.at<double>(0, 2)
    );

    v = glm::transpose(glm::mat3(
        (float)ocv.at<double>(0, 0), (float)ocv.at<double>(0, 1), (float)ocv.at<double>(0, 2),
        (float)ocv.at<double>(1, 0), (float)ocv.at<double>(1, 1), (float)ocv.at<double>(1, 2),
        (float)ocv.at<double>(2, 0), (float)ocv.at<double>(2, 1), (float)ocv.at<double>(2, 2)
    ));

    // std::cout
    //     << glm::to_string(m) << std::endl
    //     << glm::to_string(u) << std::endl
    //     << glm::to_string(e) << std::endl
    //     << glm::to_string(v) << std::endl;
}
