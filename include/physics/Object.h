#pragma once


/*
Note:
Instead of scaling the rigid body you will need to instead scale the shape
used for collision detection. This is done by calling
btCollisionShape::setLocalScaling(). You may need to call
btCollisionWorld::updateSingleAABB( rigidbody ) to get the new bounding box
of the scale to take effect.
 */

class Object {
public:
    Object(void* userPointer,
        btCollisionShape* shape, const glm::vec3& position=glm::vec3())
    {
        btDefaultMotionState* motionState = new btDefaultMotionState(
            btTransform(btQuaternion(0,0,0,1), glmToBullet(position))
        );
        btRigidBody::btRigidBodyConstructionInfo
                ci(0, motionState, shape);
        mRigidBody = new btRigidBody(ci);

        mRigidBody->setCollisionFlags(mRigidBody->getCollisionFlags() |
            btCollisionObject::CF_KINEMATIC_OBJECT);
        mRigidBody->setActivationState(DISABLE_DEACTIVATION);
    }

    btRigidBody* getRigidBody() const { return mRigidBody; }

    glm::mat4 getTransformation() const {
        btTransform transform;
        mRigidBody->getMotionState()->getWorldTransform(transform);
        glm::mat4 m;
        transform.getOpenGLMatrix(glm::value_ptr(m));
        return m;
    }

    void setTransformation(const Transformation& transform) {
        btTransform btransform(
            glmToBullet(transform.rotation), glmToBullet(transform.position)
        );
        mRigidBody->getMotionState()->setWorldTransform(btransform);
    }

private:
    btRigidBody* mRigidBody;

};
