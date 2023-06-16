#include "BodyIMeshSolver.h"
#include <cnoid/Link>
#include <cnoid/MultiVector3Seq>
#include <cnoid/Array2D>
#include <Eigen/Sparse>
#include <Eigen/UmfPackSupport>
#include <cnoid/stdx/optional>
#include <vector>
#include <set>
#include <iostream>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

const bool DEBUG = false;

const bool USE_JOINT_CONSTRAINT_MASK = false;

typedef Eigen::SparseMatrix<double, Eigen::RowMajor> RmSparseMatrix;
typedef Eigen::SparseMatrix<double, Eigen::ColMajor> CmSparseMatrix;

}

namespace cnoid {

class BodyIMeshSolverImpl
{
public:
    int numSteps;
    int numStepsOfShrinkingVelocityLimits;
    int numStepsOfKeepingVelocityConstraints;
    
    struct JointInfo
    {
        Link* link;
        vector<int> affectedMarkerIndices; // body local index
        JointInfo(Link* link) : link(link) { }
    };
    typedef std::shared_ptr<JointInfo> JointInfoPtr;
    
    struct BodyInfo
    {
        BodyPtr body;
        int globalJointSpaceOffset; // for the space including root DOFs
        int globalJointIndexOffset; // for the space including only valid joints
        vector<JointInfoPtr> validJoints;
        vector<int> jointIdToValidJointIndexMap;
        std::shared_ptr<BodyMotion> currentStepMotion;
        vector<std::shared_ptr<BodyMotion>> halfwayMotions;
    };
    typedef std::shared_ptr<BodyInfo> BodyInfoPtr;
    
    vector<BodyInfoPtr> bodies;
    int numMarkers;
    int dimMarkerSpace;
    int dimJointSpace;
    int globalDimJointSpace;
    int numValidJoints;
    BodyIMeshPtr mesh;
    int totalNumFrames;
    vector<Vector3> allMarkerPositions;
    bool isFixedRootMode;
    bool isHalfwayMotionRecordingEnabled;
    bool isSinglePoseMode;
    double jointDampingWeight;
    double maxLaplacianDiffToOrgPosition;
    int numHalfwayMotions;
    
    MatrixXd Jm; // Jacobian for marker positions
    MatrixXd J; // Final Jacobian
    
    struct LinkConstraint {
        BodyInfoPtr bodyInfo;
        Link* link;
        // If this is invalid, the initial position is set in the initalization
        stdx::optional<SE3> pos;
    };
    typedef std::shared_ptr<LinkConstraint> LinkConstraintPtr;
    
    struct VelocityConstraint {
        int jointIndex;
        double goalVelocity;
        VelocityConstraint(int index, double goal)
            : jointIndex(index), goalVelocity(goal) { }
    };
    
    struct FrameVariableSet
    {
        int globalHardConstraintOffset;
        VectorXd jointSpacePositions; // used for setting acc minimization constants
        vector< vector<double> > allWeights;
        vector< vector<double> > allLengths;
        VectorXd L0; // initial Laplacian coordinate
        MatrixXd B; // top left block for the cost minimization 
        MatrixXd C; // hard constraint matrix
        MatrixXd C_right; // additional hard constraint matrix for velocities
        VectorXd h; // hard constraint constant
        vector<LinkConstraintPtr> hardLinkConstraints;
        vector<VelocityConstraint> velocityConstraints;
        vector<bool> velocityConstraintFlags;
    };
    
    vector<FrameVariableSet> allFrameVariables;
    
    CmSparseMatrix S; // final matrix to solve
    VectorXd y; // final constant vector
    VectorXd solution; // final solution vector
    
    struct AccTermInfo {
        int offset;
        double value;
        AccTermInfo(int offset, double value)
            : offset(offset), value(value) { }
    };
    double accTermWeight;
    VectorXd accTermWeights;
    vector< vector<AccTermInfo> > allAccTermInfos;
    vector<AccTermInfo> emptyAccTermInfos;
    bool isAccTermEnabled;
    
    vector<BodyInfoPtr> rootFreeBodies;
    MultiVector3Seq orgRootTranslations;
    
    struct JointConstraint {
        int globalJointIndex;
        double dq;
        JointConstraint(int globalJointIndex, double dq)
            : globalJointIndex(globalJointIndex), dq(dq) { }
        JointConstraint() { }
    };
    vector<JointConstraint> jointRangeConstraints; // for a frame
    Array2D<char> jointConstraintMask;
    
    std::ostream* os_;
    
    std::ostream& os() { return *os_; }
    
    BodyIMeshSolverImpl(std::ostream& os);
    void clear();
    void setInteractionMesh(BodyIMeshPtr mesh);
    void extractValidJoints(BodyInfoPtr bodyInfo, const vector<BodyIMesh::MarkerPtr>& markers);
    void addHardConstraintToKeepOriginalLinkPosition(int bodyIndex, int linkIndex);
    void initAccTermInfo();
    void storeInitialRootTranslations();
    void updateCurrentBodyPositions(int step, int frameIndex);
    void calcMarkerJacobian();
    void calcJacobianRootDOFcolumn(
        Link* rootLink, const vector<BodyIMesh::MarkerPtr>& localMarkers, int markerIndexOffset, int jointIndexOffset);
    void calcJacobianJointColumn(
        JointInfo& jointInfo, const vector<BodyIMesh::MarkerPtr>& localMarkers, int markerIndexOffset, int jointIndex);
    void calcLaplacianCoordinateJacobian(int frameIndex);
    void calcLaplacianCoordinate(int frameIndex, VectorXd& out_L, bool updateWeights);
    void detectJointOverruns(int frameIndex, double dqRatio, bool fixExistingVelocityConstraints);
    void calcLinkPositionConstraintCoefficients(vector<LinkConstraintPtr>& constraints, MatrixXd& Ci, VectorXd& hi);
    bool apply();
    bool applyFrameLocally();
    void setFrameCoefficientsToLeftHalfBlock(
        int frameIndex, VectorXd*& q_prev, vector<VelocityConstraint>*& vclist_prev);
    bool solveFinalMatrixForAllFrames(int globalSize);
    void storeSolution(int stepIndex);
    
    // for pose interpolation test
    struct PosePair
    {
        BodyPtr body;
        BodyState initialPose;
        BodyState finalPose;
        std::shared_ptr<BodyMotion> motion;
    };
    typedef std::shared_ptr<PosePair> PosePairPtr;
    
    vector<PosePairPtr> posePairs;
    
    void setInterpolationTestPosePair(
        BodyPtr body, const BodyState& initialPose, const BodyState& finalPose, std::shared_ptr<BodyMotion> output);
    bool doInterpolationTest(int numInterpolatedFrames, bool doInterpolation);
    bool doInterpolationTest2(int numInterpolatedFrames);
};

}
