#include <stdinc.h>
#include <graphics/Model.h>
#include <BulletCollision/CollisionShapes/btShapeHull.h>


inline glm::vec3 aiToGlm(const aiVector3D& v) {
    return glm::vec3(v.x, v.y, v.z);
}

inline glm::vec2 aiToGlm2(const aiVector3D& v) {
    return glm::vec2(v.x, v.y);
}


Model::Model(const std::string& path)
    : mPath(path)
{
    transformation.scale = glm::vec3(0.002f);
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(path,
        aiProcess_Triangulate |
        aiProcess_FlipUVs |
        aiProcess_GenSmoothNormals
    );

    if (!scene || scene->mFlags == AI_SCENE_FLAGS_INCOMPLETE) {

        std::stringstream msg;
        msg << "Model loading error: \t" << importer.GetErrorString();
        msg << std::endl;
        throw Exception(msg.str());
    }

    std::string directory = getFolder(path);

    for (size_t i=0; i<scene->mNumMaterials; ++i) {
        aiString path;
        scene->mMaterials[i]->GetTexture(aiTextureType_DIFFUSE, 0, &path);
        if (std::string(path.C_Str()) != "") {
            std::string spath = directory + "/" + getFilename(path.C_Str());
            mTextures.push_back(Texture(spath));
        }
        else {
            mTextures.push_back(Texture());
        }
    }
    mTextures.push_back(Texture()); // TODO: Use white texture.

    mShape = new btConvexHullShape();
    for (size_t i=0; i<scene->mNumMeshes; ++i) {
        aiMesh* mesh = scene->mMeshes[i];
        loadMesh(mesh, scene);
    }

    btShapeHull hull(mShape);
	hull.buildHull(mShape->getMargin());
    btConvexHullShape* simplifiedShape = new btConvexHullShape(
        (btScalar*)hull.getVertexPointer(), hull.numVertices()
    );
    delete mShape;
    mShape = simplifiedShape;

    mShape->setLocalScaling(glmToBullet(transformation.scale));

    mObject = new Object(this, mShape, transformation.position);
}

void Model::loadMesh(aiMesh* mesh, const aiScene* scene) {
    std::vector<Vertex> vertices(mesh->mNumVertices);
    std::vector<GLuint> indices(mesh->mNumFaces*3);

    for (size_t i=0; i<mesh->mNumVertices; ++i) {
        Vertex& vertex = vertices[i];
        vertex.position = aiToGlm(mesh->mVertices[i]);
        vertex.normal = aiToGlm(mesh->mNormals[i]);
        if (mesh->mTextureCoords[0]) {
            vertex.texCoords = aiToGlm2(mesh->mTextureCoords[0][i]);
        }
        else {
            vertex.texCoords = glm::vec2(0, 0);
        }
        mShape->addPoint(glmToBullet(vertex.position));
    }

    for (size_t i=0; i<mesh->mNumFaces; ++i) {
        for (size_t j=0; j<3; ++j) {
            indices[i*3+j] = mesh->mFaces[i].mIndices[j];
        }
    }

    if (mesh->mMaterialIndex >= 0)
        mMaterialIndices.push_back(mesh->mMaterialIndex);
    else
        mMaterialIndices.push_back(mTextures.size()-1);

    mMeshes.push_back(Mesh(vertices, indices));
}


void Model::draw(const Program& program, const glm::mat4& viewProjection) {
    glm::mat4 model = transformation.getMatrix();
    for (size_t i=0; i<mMeshes.size(); ++i) {
        mMeshes[i].draw(program, model, viewProjection,
            mTextures[mMaterialIndices.size()-1]);
    }
}
