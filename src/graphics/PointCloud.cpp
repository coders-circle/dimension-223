#include <stdinc.h>
#include <graphics/PointCloud.h>


const int MAX_POINTS = 700;
const int SKIP = 1;

PointCloud::PointCloud(
    const std::string& path,
    const std::vector<glm::ivec2>& points
)
    : mLensBlurImage(path), mTexture(mLensBlurImage.getImage())
{

    // Get the depth map and its size.
    cv::Mat& depthMap = mLensBlurImage.getDepthMap();
    int height = depthMap.rows;
    int width = depthMap.cols;

    // Get the floor vertices.
    std::vector<bool> floorPoints(width*height, false);
    for (size_t i=0; i<points.size(); ++i) {
        const glm::ivec2& p = points[i];
        for (int x = glm::max(0, p.x-16); x < glm::min(width, p.x+16); ++x) {
            for (int y = glm::max(0, p.y-16); y < glm::min(height, p.y+16);
                 ++y)
            {
                floorPoints[y*width + x] = true;
            }
        }
    }

    float floorY = -9999;

    // Get (x, y) coordinate of the first point.
    float xoffset = float(MAX_POINTS - width)/2.0f / MAX_POINTS;
    float yoffset = float(MAX_POINTS - height)/2.0f / MAX_POINTS;

    // Fill in the point cloud.
    mVertices.resize(width*height/SKIP/SKIP*5);
    mPoints.resize(width*height/SKIP/SKIP);

    int tmp = 0;
    for (int i=0; i<height; i+=SKIP) {
        for (int j=0; j<width; j+=SKIP) {

            float x = j/(float)MAX_POINTS - 0.5f + xoffset;
            float y = -i/(float)MAX_POINTS + 0.5f - yoffset;
            float depth = depthMap.at<cv::Vec3b>(i, j)[0];

            int index = i*width+j;
            mPoints[index][0] = x * (1-depth/300);
            mPoints[index][1] = y * (1-depth/300);
            mPoints[index][2] = depth/300;

            // Check if a floor point.
            if (floorPoints[index]) {
                if (floorY == -9999)
                    floorY = mPoints[index][1];
                else
                    mPoints[index][1] = floorY;
            }

            mVertices[tmp++] = mPoints[index][0];
            mVertices[tmp++] = mPoints[index][1];
            mVertices[tmp++] = mPoints[index][2];
            mVertices[tmp++] = j/float(width-1);
            mVertices[tmp++] = i/float(height-1);
        }
    }

    // mTriangleMesh = new btTriangleMesh();

    mIndices.resize((width/SKIP-1)*(height/SKIP-1)*6);
    tmp = 0;
    for (int i=0; i<height/SKIP-1; i++) {
        for (int j=0; j<width/SKIP-1; j++) {
            int offset = i*width+j;
            mIndices[tmp++] = offset;
            mIndices[tmp++] = offset+1;
            mIndices[tmp++] = offset+width;
            mIndices[tmp++] = offset+1;
            mIndices[tmp++] = offset+1+width;
            mIndices[tmp++] = offset+width;

            // mTriangleMesh->addTriangle(
            //     glmToBullet(mPoints[offset]),
            //     glmToBullet(mPoints[offset+1]),
            //     glmToBullet(mPoints[offset+width])
            // );
            //
            // mTriangleMesh->addTriangle(
            //     glmToBullet(mPoints[offset+1]),
            //     glmToBullet(mPoints[offset+1+width]),
            //     glmToBullet(mPoints[offset+width])
            // );
        }
    }

    // Create the physics object.
    // mTriangleMeshShape = new btBvhTriangleMeshShape(mTriangleMesh, false);
    // mObject = new Object(this, mTriangleMeshShape, glm::vec3(0),
    //     Object::STATIC);

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
