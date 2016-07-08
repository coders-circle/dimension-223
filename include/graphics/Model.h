#pragma once
#include <graphics/Mesh.h>
#include <graphics/Transformation.h>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <physics/Object.h>

/**
 * Collection of meshes that are rendered as one. A model can be loaded
 * from file.
 */
class Model {
public:
    Model(const std::string& path);
    void update() {
        transformation.setMatrix(mObject->getTransformation());
    }
    void draw(const Program& program, const glm::mat4& viewProjection);

    Transformation transformation;
    void transform() {
        mObject->setTransformation(transformation);
    }

    std::string getPath() const { return mPath; }

    Object* getObject() const { return mObject; }

private:
    std::string mPath;
    btConvexHullShape* mShape;
    Object* mObject;

    std::vector<Texture> mTextures;
    std::vector<Mesh> mMeshes;
    std::vector<size_t> mMaterialIndices;

    void loadMesh(aiMesh* mesh, const aiScene* scene);
};
