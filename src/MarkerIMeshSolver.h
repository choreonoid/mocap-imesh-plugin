#ifndef CNOID_MOCAP_IMESH_PLUGIN_MARKER_IMESH_SOLVER_H
#define CNOID_MOCAP_IMESH_PLUGIN_MARKER_IMESH_SOLVER_H

#include "MarkerIMesh.h"
#include <cnoid/MocapMapping>
#include <cnoid/NullOut>

namespace cnoid {

class MarkerIMeshSolver
{
public:
    MarkerIMeshSolver();
    ~MarkerIMeshSolver();

    void clear();
    void setSingleFrameMode(bool on, int targetFrame = 0);
    void setInteractionMesh(MarkerIMeshPtr mesh);
    void setMocapMappingPair(int motionIndex, MocapMappingPtr org, MocapMappingPtr goal);
    void addSoftPositionalConstraint(
        int motionIndex, int localVertexIndex, const Vector3& pos, int frame = -1, double alpha = 1.0);
    void addHardPositionalConstraint(
        int motionIndex, int localVertexIndex, const Vector3& pos, int frame = -1, double alpha = 1.0);
    void addHardConstraintToKeepOriginalMarkerPosition(
        int motionIndex, int markerIndex, const Vector3& offset = Vector3::Zero());
    void addSoftConstraintToKeepOriginalMarkerPosition(
        int motionIndex, int markerIndex, double weight, const Vector3& offset = Vector3::Zero());

    void updateLaplacianCoordinateConstantEvreyFrame(bool on);
    void excludeBoneEdgesFromLaplacianCoordinate(bool on);
    void setLaplacianWeightPower(double pow);
    void setAccelConstraintWeight(double w);

    void setMotionToOutputSolution(int motionIndex, MarkerMotionPtr motion);
    bool solve(int numSteps, std::ostream& os = nullout());
    MarkerMotionPtr solution(int index);
    const std::string& message() const;

private:
    MarkerIMeshSolver(const MarkerIMeshSolver& org);

    class Impl;
    Impl* impl;
};

}

#endif
