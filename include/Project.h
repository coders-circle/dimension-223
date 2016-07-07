#pragma once
#include <graphics/Model.h>
#include <graphics/PointCloud.h>


/**
 * The repository for project data representing an project state. This data is
 * used for saving and loading.
 */
class Project {
public:
    size_t addModel(const std::string& path) {
        mModels.push_back(Model(path));
        return mModels.size()-1;
    }

    Model& getModel(size_t index) {
        return mModels[index];
    }

    size_t getNumModels() const {
        return mModels.size();
    }

    size_t addPointCloud(const std::string& path) {
        mPointClouds.push_back(PointCloud(path));
        return mPointClouds.size()-1;
    }

    PointCloud& getPointCloud(size_t index) {
        return mPointClouds[index];
    }

    size_t getNumPointClouds() const {
        return mPointClouds.size();
    }

    void clear() {
        mModels.clear();
        mPointClouds.clear();
    }

private:
    std::vector<Model> mModels;
    std::vector<PointCloud> mPointClouds;

};
