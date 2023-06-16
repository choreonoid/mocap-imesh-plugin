#ifndef CNOID_MOCAP_IMESH_PLUGIN_BODY_IMESH_SOLVER_H
#define CNOID_MOCAP_IMESH_PLUGIN_BODY_IMESH_SOLVER_H

#include "BodyIMesh.h"
#include <cnoid/BodyState>
#include <cnoid/NullOut>

namespace cnoid {

class BodyIMeshSolverImpl;

class BodyIMeshSolver
{
public:
    BodyIMeshSolver(std::ostream& os = nullout());
    ~BodyIMeshSolver();

    void clear();
    void setFixedRootMode(bool on);
    void setInteractionMesh(BodyIMeshPtr mesh);

    void setJointDampingWeight(double w);
    void setMaxLaplacianDiffToOrgPosition(double d);
    void setAccTermWeight(double w);

    //! \note This must be called after calling 'setInteractionMesh()'
    void addHardConstraintToKeepOriginalLinkPosition(int bodyIndex, int linkIndex);

    //void setAccelConstraintWeight(double w);

    void setNumSteps(int n);
    void setNumStepsOfShrinkingVelocityLimits(int n);
    void setNumStepsOfKeepingVelocityConstraints(int n);

    bool apply();
    bool applyFrameLocally();

    void enableHalfwayMotionRecording(bool on);
    int numHalfwayMotions() const;
    std::shared_ptr<BodyMotion> halfwayMotion(int bodyIndex, int motionIndex);

    // for testing
    void setInterpolationTestPosePair(
        BodyPtr body, const BodyState& initialPose, const BodyState& finalPose, std::shared_ptr<BodyMotion> output);
    bool doInterpolationTest(int numInterpolatedFrames, bool doInterpolation, bool useQuadraticFormulation);

private:
    BodyIMeshSolver(const BodyIMeshSolver& org);

    BodyIMeshSolverImpl* impl;
};

}

#endif
