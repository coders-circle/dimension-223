#pragma once
#include <physics/Object.h>


class World {
public:
    World() {
        mBroadphase = new btDbvtBroadphase();
        mCollisionConfiguration = new btDefaultCollisionConfiguration();
        mDispatcher = new btCollisionDispatcher(mCollisionConfiguration);
        mSolver = new btSequentialImpulseConstraintSolver;
        mDynamicsWorld = new btDiscreteDynamicsWorld(mDispatcher, mBroadphase,
            mSolver, mCollisionConfiguration);
        mDynamicsWorld->setGravity(btVector3(0, -10, 0));
    }

    void update() {
        mDynamicsWorld->stepSimulation(1/60.0f, 10);
    }

    void add(const Object& object) {
        mDynamicsWorld->addRigidBody(object.getRigidBody());
    }

    void remove(const Object& object) {
        mDynamicsWorld->removeRigidBody(object.getRigidBody());
    }

    btDiscreteDynamicsWorld* getWorld() const {
        return mDynamicsWorld;
    }

private:
    btBroadphaseInterface* mBroadphase;
    btDefaultCollisionConfiguration* mCollisionConfiguration;
    btCollisionDispatcher* mDispatcher;
    btSequentialImpulseConstraintSolver* mSolver;
    btDiscreteDynamicsWorld* mDynamicsWorld;
};
