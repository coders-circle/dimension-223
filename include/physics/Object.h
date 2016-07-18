#pragma once
#include <graphics/Transformation.h>


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
    enum Type { STATIC, KINEMATIC };
    Object(void* userPointer,
           btCollisionShape* shape, const glm::vec3& position=glm::vec3(),
           Type type = KINEMATIC)
    {
        btVector3 inertia(0.0, 0.0, 0.0);
        float mass = 0.0f;
        if (type != STATIC) {
            mass = 1.0f;
            shape->calculateLocalInertia(mass, inertia);
        }

        btDefaultMotionState* motionState = new btDefaultMotionState(
            btTransform(btQuaternion(0,0,0,1), glmToBullet(position))
        );
        btRigidBody::btRigidBodyConstructionInfo
                ci(mass, motionState, shape, inertia);
        mRigidBody = new btRigidBody(ci);

        if (type == KINEMATIC) {
            mRigidBody->setCollisionFlags(mRigidBody->getCollisionFlags() |
                btCollisionObject::CF_KINEMATIC_OBJECT);
            mRigidBody->setActivationState(DISABLE_DEACTIVATION);
        }
        else {
            mRigidBody->setCollisionFlags(mRigidBody->getCollisionFlags() |
                btCollisionObject::CF_STATIC_OBJECT);
        }
        mDynamic = false;
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
        mRigidBody->getCollisionShape()->setLocalScaling(
            glmToBullet(transform.scale));
    }

    void setDynamic(bool dynamic) {
        if (dynamic) {
            mRigidBody->setCollisionFlags(mRigidBody->getCollisionFlags()
                & ~btCollisionObject::CF_KINEMATIC_OBJECT);
        } else {
            mRigidBody->setCollisionFlags(mRigidBody->getCollisionFlags()
                | btCollisionObject::CF_KINEMATIC_OBJECT);
        }
        mDynamic = dynamic;
    }

    bool isDynamic() const {
        return mDynamic;
    }

private:
    btRigidBody* mRigidBody;
    bool mDynamic;

};
