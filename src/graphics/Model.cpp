#include <stdinc.h>
#include <graphics/Model.h>


Model::Model(const std::string& path) {
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

    mDirectory = getFolder(path);

    for (size_t i=0; i<scene->mNumMaterials; ++i) {
        aiString path;
        scene->mMaterials[i]->GetTexture(aiTextureType_DIFFUSE, 0, &path);
        if (std::string(path.C_Str()) != "") {
            std::string spath = mDirectory + "/" + getFilename(path.C_Str());
            mTextures.push_back(Texture(spath));
        }
        else {
            mTextures.push_back(Texture());
        }
    }
    mTextures.push_back(Texture()); // TODO: Use white texture.

    for (size_t i=0; i<scene->mNumMeshes; ++i) {
        aiMesh* mesh = scene->mMeshes[i];
        loadMesh(mesh, scene);
    }
}


inline glm::vec3 aiToGlm(aiVector3D& v) {
    return glm::vec3(v.x, v.y, v.z);
}
inline glm::vec2 aiToGlm2(aiVector3D& v) {
    return glm::vec2(v.x, v.y);
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
    }

    for (size_t i=0; i<mesh->mNumFaces; ++i) {
        for (size_t j=0; j<3; ++j) {
            indices[i*3+j] = mesh->mFaces[i].mIndices[j];
        }
    }

    if (mesh->mMaterialIndex >= 0) {
        mMeshes.push_back(Mesh(vertices, indices,
            mTextures[mesh->mMaterialIndex]));
    }
    else {
        mMeshes.push_back(Mesh(vertices, indices,
            mTextures[mTextures.size()-1]));
    }
}


void Model::draw(const Program& program, const glm::mat4& model,
                 const glm::mat4& viewProjection) {

    for (size_t i=0; i<mMeshes.size(); ++i) {
        mMeshes[i].draw(program, model, viewProjection);
    }
 }
