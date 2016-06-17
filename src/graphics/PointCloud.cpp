#include <stdinc.h>
#include <graphics/PointCloud.h>


const int MAX_POINTS = 700;
const int SKIP = 4;

PointCloud::PointCloud(cv::Mat& image, cv::Mat& depthMap)
    : mImage(image), mDepthMap(depthMap), mTexture(image)
{
    int height = depthMap.rows;
    int width = depthMap.cols;

    float xoffset = float(MAX_POINTS - width)/2.0f / MAX_POINTS;
    float yoffset = float(MAX_POINTS - height)/2.0f / MAX_POINTS;

    mVertices.resize(width*height/SKIP/SKIP*5);
    mPoints.resize(width*height/SKIP/SKIP);

    int tmp = 0;
    for (int i=0; i<height; i+=SKIP) {
        for (int j=0; j<width; j+=SKIP) {

            float x = j/(float)MAX_POINTS - 0.5f + xoffset;
            float y = -i/(float)MAX_POINTS + 0.5f - yoffset;
            float depth = depthMap.at<cv::Vec3b>(i, j)[0];

            mPoints[i][0] = x * (1-depth/300);
            mPoints[i][1] = y * (1-depth/300);
            mPoints[i][2] = depth/300;

            mVertices[tmp++] = mPoints[i][0];
            mVertices[tmp++] = mPoints[i][1];
            mVertices[tmp++] = mPoints[i][2];
            mVertices[tmp++] = j/float(width-1);
            mVertices[tmp++] = i/float(height-1);
        }
    }

    // Create the vertex and index buffer as well as vertex array
    // objects.

    glGenBuffers(1, &mVbo);
    glGenBuffers(1, &mIbo);
    glGenVertexArrays(1, &mVao);

    glBindVertexArray(mVao);

    glBindBuffer(GL_ARRAY_BUFFER, mVbo);
    glBufferData(GL_ARRAY_BUFFER, mVertices.size()*sizeof(float),
        &mVertices[0], GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5*sizeof(GLfloat),
        (GLvoid*)0);
        glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5*sizeof(GLfloat),
        (GLvoid*)(3*sizeof(GLfloat)));
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);
}


void PointCloud::draw(const Program& program, const glm::mat4& model,
        const glm::mat4& viewProjection) {

    program.use();

    GLint modelLoc = program.getUniformLocation("model");
    glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
    GLint vpLoc = program.getUniformLocation("viewProjection");
    glUniformMatrix4fv(vpLoc, 1, GL_FALSE, glm::value_ptr(viewProjection));
    GLint texLoc = program.getUniformLocation("uTexture");
    glUniform1i(texLoc, 0);
    glActiveTexture(GL_TEXTURE0);
    mTexture.bind();

    glBindVertexArray(mVao);
    glDrawArrays(GL_POINTS, 0, mVertices.size());
    glBindVertexArray(0);
}
