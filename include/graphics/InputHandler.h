#pragma once
#include <physics/World.h>
#include <Project.h>


/**
 * Handler of user input.
 */
class InputHandler {
public:
    InputHandler(Project& project);

    void update(float x, float y,
                bool leftMouseDown, bool middleMouseDown, bool rightMouseDown,
                unsigned int mouseState);

    void scroll(float dx, float dy);

    void setSelectionChangedCallback(
        const std::function<void(size_t)>& callback
    ) {
        mSelectionChanged = callback;
    }

    void reset() {
        mYaw = 90;
        mPitch = 0;
        mTarget = glm::vec3(0);
        mZoom = 1.0f;
        updateCamera();
    }

private:
    /// The project database.
    Project& mProject;

    /// Currently selected model.
    size_t mSelection;
    /// Is anything currently selected.
    bool mSelected;

    /// The hit point relative to the currently selected model:
    /// where in the selected model is the mouse clicked.
    glm::vec3 mHitPoint;
    /// The distance from screen to the current selection.
    float mHitDist;

    /// Is camera moving.
    bool mCameraMoving;
    /// Last mouse position.
    float mLastX, mLastY;
    /// Camera angle.
    float mYaw, mPitch;
    /// Camera target position.
    glm::vec3 mTarget;
    /// Camera zoom.
    float mZoom;

    void updateCamera();

    std::function<void(size_t)> mSelectionChanged;
};
