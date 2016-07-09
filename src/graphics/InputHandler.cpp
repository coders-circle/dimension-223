#include <stdinc.h>
#include <graphics/InputHandler.h>

#include <gtk/gtk.h>
#include <gdk/gdkkeysyms.h>


InputHandler::InputHandler(Project& project)
    : mProject(project), mSelected(false),
      mCameraMoving(false), mYaw(90), mPitch(0), mTarget(0,0,0),
      mZoom(1.0f)
{
    updateCamera();
}


void InputHandler::update(float x, float y,
    bool leftMouseDown, bool middleMouseDown, bool rightMouseDown,
    unsigned int mouseState)
{

    // Model movement.
    if (leftMouseDown) {
        // First find the ray originating from the mouse.
        glm::vec3 rayStart = mProject.getCamera().inverse(glm::vec3(x, y, -1));
        glm::vec3 rayEnd = mProject.getCamera().inverse(glm::vec3(x, y, 0));
        glm::vec3 rayDir(rayEnd - rayStart);
        rayDir = glm::normalize(rayDir);

        // Next get the closest hit by this ray.
        btCollisionWorld::ClosestRayResultCallback
            rayCallback(glmToBullet(rayStart), glmToBullet(rayDir*1000.0f));
        mProject.getPhysicsWorld().getWorld()->rayTest(
            glmToBullet(rayStart), glmToBullet(rayDir*1000.0f),
            rayCallback
        );

        if (rayCallback.hasHit()) {
            const btCollisionObject* body = rayCallback.m_collisionObject;

            // Kinematic bodies are models.
            if (body && body->isKinematicObject()) {
                size_t model = (size_t)body->getUserPointer();

                // Update hit-point and distance only when FIRST selected.
                // After that, we need to move the object according
                // to these hit-point and distance rather than obtain new.
                if (mSelection != model || !mSelected) {
                    mSelected = true;
                    mSelection = model;

                    glm::vec3 hit(bulletToGlm(rayCallback.m_hitPointWorld));
                    mHitDist = glm::length(hit-rayStart);
                    mHitPoint = hit -
                        mProject.getModel(mSelection).transformation.position;

                    if (mSelectionChanged)
                        mSelectionChanged(mSelection);
                }
            }
        }

        // Update the model position as per previous hit point and distance.
        if (mSelected) {
            Model& selection = mProject.getModel(mSelection);
            glm::vec3 newPos = rayStart + rayDir * mHitDist - mHitPoint;
            selection.transformation.position = newPos;
            selection.transform();

            if (mSelectionChanged)
                mSelectionChanged(mSelection);
        }
    }
    else {
        mSelected = false;
    }

    // Camera movement.
    if (middleMouseDown) {
        if (mCameraMoving) {
            float xOffset = x - mLastX;
            float yOffset = y - mLastY;

            if (mouseState & GDK_SHIFT_MASK) {
                // Shift Movement to move camera target.
                const float sensitivity = 0.0025f;
                Camera& cam = mProject.getCamera();

                // Move along the right and up vectors of the camera.
                glm::vec3 front = -(mTarget - cam.getPosition());
                glm::vec3 right = glm::normalize(
                    glm::cross(front, cam.getUp()));

                mTarget += right * xOffset * sensitivity;
                mTarget += cam.getUp() * yOffset * sensitivity;
                updateCamera();
            }
            else {
                // Angular movement: yaw and pitch.
                const float sensitivity = 0.25f;
                mYaw += xOffset * sensitivity;
                mPitch += glm::max(-89.0f,
                    glm::min(89.0f, yOffset * sensitivity));
                updateCamera();
            }
        }

        mCameraMoving = true;
        mLastX = x;
        mLastY = y;
    } else {
        mCameraMoving = false;
    }
}


void InputHandler::scroll(float dx, float dy) {
    mZoom = glm::max(0.1f, mZoom+dy*0.1f);
    updateCamera();
}


void InputHandler::updateCamera() {
    Camera& cam = mProject.getCamera();

    glm::vec3 position;
    position.x = cos(glm::radians(mYaw)) * cos(glm::radians(mPitch));
    position.y = sin(glm::radians(mPitch));
    position.z = sin(glm::radians(mYaw)) * cos(glm::radians(mPitch));

    position = mZoom*glm::normalize(position);
    position += mTarget;

    glm::vec3 front = -(mTarget - position);
    glm::vec3 right = glm::normalize(glm::cross(front, cam.getUp()));
    glm::vec3 up = glm::normalize(glm::cross(right, front));

    cam.view(position, mTarget, up);
}
