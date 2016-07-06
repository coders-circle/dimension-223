#pragma once
#include <graphics/Mesh.h>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

/**
 * Collection of meshes that are rendered as one. A model can be loaded
 * from file.
 */
class Model {
public:
    Model(const std::string& path);
    void draw(const Program& program, const glm::mat4& model,
              const glm::mat4& viewProjection);

private:
    std::string mDirectory;

    std::vector<Texture> mTextures;
    std::vector<Mesh> mMeshes;

    void loadMesh(aiMesh* mesh, const aiScene* scene);
};
