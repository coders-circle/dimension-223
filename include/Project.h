#pragma once
#include <image/Image.h>
#include <graphics/Model.h>

/**
 * Project class stores all the user configurations, inputs
 * as well as outputs that can be saved, loaded and exported.
 *
 * A project basically refers to the state of the application and stores
 * all variables needed to represent the current state.
 */
class Project
{
public:
    /// List of all input images.
    std::vector<Image<glm::vec4>> inputImages;

    /// Disparity map generated from the input images.
    Image<float> disparityMap;

    /// Camera distance between two input images.
    float camDistance;

    /// Camera's focal length.
    float focalLength;

    /// Depth map generated from the disparity map and camera configurations.
    Image<float> depthMap;

    // TODO: List of models added as extra objects to the scene.
    // std::vector<Model> models;

    /// Filename of the project.
    std::string m_filename = "";

    /**
     * Load project state from a file.
     * @param filename Path of the file to load project from.
     */
    void Load(const std::string& filename)
    {
        // TODO
    }

    /**
     * Save the project state to a file.
     * @param filename Path where the project is to be saved at.
     */
    void Save(const std::string& filename)
    {
        // TODO
    }
};
