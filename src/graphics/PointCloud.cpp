#include <stdinc.h>
#include <graphics/PointCloud.h>


const int MAX_POINTS = 700;

PointCloud::PointCloud(
    const InputData& inputData
)
    : mInputData(inputData),
      mConstructed(false),
      mTexture(mInputData.getImage())
{
    mSurfaces.clear();
    reconstruct();
}


void PointCloud::reconstruct() {
    if (mConstructed)
        destroy();
    else
        mConstructed = true;

    // Get the depth map and its size.
    cv::Mat& depthMap = mInputData.getDepthMap();
    int height = depthMap.rows;
    int width = depthMap.cols;

    // Fill the index buffer.
    mIndices.resize((width-1)*(height-1)*6);
    int tmp = 0;
    for (int i=0; i<height-1; i++) {
        for (int j=0; j<width-1; j++) {
            int offset = i*width+j;
            mIndices[tmp++] = offset;
            mIndices[tmp++] = offset+1;
            mIndices[tmp++] = offset+width;
            mIndices[tmp++] = offset+1;
            mIndices[tmp++] = offset+1+width;
            mIndices[tmp++] = offset+width;
        }
    }

    // Create the point cloud.
    mPoints.resize(width*height);
    mImageCoordinates.resize(width*height);

    // Get (x, y) coordinate of the first point.
    float xoffset = float(MAX_POINTS - width)/2.0f / MAX_POINTS;
    float yoffset = float(MAX_POINTS - height)/2.0f / MAX_POINTS;

    tmp = 0;
    for (int i=0; i<height; i++) {
        for (int j=0; j<width; j++) {

            float x = j/(float)MAX_POINTS - 0.5f + xoffset;
            float y = -i/(float)MAX_POINTS + 0.5f - yoffset;
            float depth = depthMap.at<cv::Vec3b>(i, j)[0];

            float dd = depth * mInputData.getDepthScale();

            int index = i*width+j;

            mPoints[index][0] = x * (1-dd);
            mPoints[index][1] = y * (1-dd);
            mPoints[index][2] = dd;

            // Check for surface points.
            for (auto& surface: mSurfaces) {
                if (surface.marked[index]) {
                    if (surface.y == -9999999)
                        surface.y = mPoints[index][1];
                    else
                        mPoints[index][1] = surface.y;
                }
            }
            mImageCoordinates[index] = glm::ivec2(j, i);
        }
    }

    blur(mPoints, width, height);

    // Normal estimation.
    std::vector<glm::vec3> normals(width*height);
    for (int i=0; i<mIndices.size(); i+=3) {
        glm::vec3& v0 = mPoints[mIndices[i]];
        glm::vec3& v1 = mPoints[mIndices[i+1]];
        glm::vec3& v2 = mPoints[mIndices[i+2]];

        glm::vec3 normal = glm::normalize(
            glm::cross(v2-v0, v1-v0)
        );
        normals[mIndices[i]] += normal;
        normals[mIndices[i+1]] += normal;
        normals[mIndices[i+2]] += normal;
    }

    // Fill vertex buffer.
    mVertices.resize(width*height*8);
    for (int i=0; i<height; i++) {
        for (int j=0; j<width; j++) {
            int index = i*width+j;

            // Positions.
            mVertices[tmp++] = mPoints[index][0];
            mVertices[tmp++] = mPoints[index][1];
            mVertices[tmp++] = mPoints[index][2];

            // Normals.
            normals[index] = glm::normalize(normals[index]);
            mVertices[tmp++] = normals[index].x;
            mVertices[tmp++] = normals[index].y;
            mVertices[tmp++] = normals[index].z;

            // Texture coordinates.
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

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8*sizeof(GLfloat),
        (GLvoid*)0);
    glEnableVertexAttribArray(0);

    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8*sizeof(GLfloat),
        (GLvoid*)(3*sizeof(GLfloat)));
    glEnableVertexAttribArray(1);

    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8*sizeof(GLfloat),
        (GLvoid*)(6*sizeof(GLfloat)));
    glEnableVertexAttribArray(2);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mIbo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, mIndices.size()*sizeof(GLuint),
        &mIndices[0], GL_STATIC_DRAW);

    glBindVertexArray(0);
}


void PointCloud::draw(const Program& program,
                      const glm::mat4& viewProjection) {

    glm::mat4 model = transformation.getMatrix();
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
    // glDrawArrays(GL_POINTS, 0, mVertices.size());
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mIbo);
    glDrawElements(GL_TRIANGLES, mIndices.size(), GL_UNSIGNED_INT, (void*)0);
    glBindVertexArray(0);
}
