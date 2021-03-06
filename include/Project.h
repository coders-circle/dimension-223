#pragma once
#include <graphics/Model.h>
#include <graphics/PointCloud.h>
#include <graphics/Camera.h>
#include <physics/World.h>


/**
 * The repository for project data representing an project state. This data is
 * used for saving and loading.
 */
class Project {
public:
    size_t addModel(const std::string& path) {
        try {
            mModels.push_back(Model(path));
            size_t index = mModels.size() - 1;
            mPhysicsWorld.add(*getModel(index).getObject());
            getModel(index).getObject()->getRigidBody()->setUserPointer(
                (void*)index);
            return index;
        }
        catch (std::exception& e) {
            std::cout << e.what() << std::endl;
            return 0;
        }
    }

    Model& getModel(size_t index) {
        return mModels[index];
    }

    size_t getNumModels() const {
        return mModels.size();
    }

    size_t addPointCloud(const InputData& inputData) {
        try {
            mPointClouds.push_back(PointCloud(inputData));
            size_t index = mPointClouds.size() - 1;
            return index;
        }
        catch (std::exception& e) {
            std::cout << e.what() << std::endl;
            return 0;
        }
    }

    PointCloud& getPointCloud(size_t index) {
        return mPointClouds[index];
    }

    size_t getNumPointClouds() const {
        return mPointClouds.size();
    }

    void clear() {
        for (auto& m: mModels)
            m.destroy();
        for (auto& p: mPointClouds)
            p.destroy();
        mModels.clear();
        mPointClouds.clear();
    }

    World& getPhysicsWorld() { return mPhysicsWorld; }
    Camera& getCamera() { return mCamera; }

private:
    std::vector<Model> mModels;
    std::vector<PointCloud> mPointClouds;
    World mPhysicsWorld;
    Camera mCamera;
};
