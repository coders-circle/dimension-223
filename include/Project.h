#pragma once
#include <graphics/Model.h>
#include <graphics/PointCloud.h>
#include <graphics/Camera.h>
#include <physics/World.h>


struct CloudIntersection {
    size_t otherPointCloud;
    Area myArea, otherArea;
};


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

    size_t addPointCloud(
        const InputData& inputData,
        const std::vector<glm::ivec2>& points
    ) {
        try {
            mPointClouds.push_back(PointCloud(inputData, points));
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

    size_t addIntersection(size_t other, const Area& myArea,
                           const Area& otherArea)
    {
        CloudIntersection intersection;
        intersection.otherPointCloud = other;
        intersection.myArea = myArea;
        intersection.otherArea = otherArea;
        mIntersections.push_back(intersection);
        return mIntersections.size()-1;
    }

    CloudIntersection& getIntersection(size_t index) {
        return mIntersections[index];
    }

    size_t getNumIntersections() const {
        return mIntersections.size();
    }

    void clear() {
        for (auto& m: mModels)
            m.destroy();
        for (auto& p: mPointClouds)
            p.destroy();
        mModels.clear();
        mPointClouds.clear();
        mIntersections.clear();
    }

    World& getPhysicsWorld() { return mPhysicsWorld; }
    Camera& getCamera() { return mCamera; }

private:
    std::vector<Model> mModels;
    std::vector<PointCloud> mPointClouds;
    World mPhysicsWorld;
    Camera mCamera;

    // For each point cloud except the first, store intersection
    // areas for it and the other point cloud.
    std::vector<CloudIntersection> mIntersections;
};
