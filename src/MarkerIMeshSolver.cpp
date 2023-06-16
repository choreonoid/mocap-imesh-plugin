#include "MarkerIMeshSolver.h"
#include <fmt/format.h>
#include <Eigen/Sparse>
#include <Eigen/UmfPackSupport>
#include <cnoid/stdx/optional>
#include <set>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {

typedef Eigen::SparseMatrix<double, Eigen::RowMajor> RmSparseMatrix;
typedef Eigen::SparseMatrix<double, Eigen::ColMajor> CmSparseMatrix;

}

namespace cnoid {

class MarkerIMeshSolver::Impl
{
public:
    MarkerIMeshPtr mesh;

    bool isSingleFrameMode;
    int currentFrame;
    int numFrames;
    int targetSingleFrame;
    bool doExcludeBoneEdgesFromLaplacianCoordinate;
    string message;

    /**
       Index1 should be smaller than index2 to sequentially insert the coefficients
       of the bone length constraints into the sparse matrix.
    */
    struct Bone
    {
        int localMarkerIndex1;
        int localMarkerIndex2;
        int activeVertexIndex1;
        int activeVertexIndex2;
        stdx::optional<double> goalLength;
    };

    struct MocapMappingInfo
    {
        MocapMappingPtr org;
        MocapMappingPtr goal;
        vector<Bone> bones;
    };
    vector<MocapMappingInfo> mocapMappingInfos;

    struct PositionalConstraint {
        int activeVertexIndex;
        Vector3 pos;
        bool isRelative;
        //stdx::optional<Vector3> pos; // invalid when the original position is used
        double alpha;
        double weight; // only for soft constraint
    };

    struct ConstraintInfo {
        vector<PositionalConstraint> softPositionalConstraints;
        vector<PositionalConstraint> hardPositionalConstraints;
        int globalHardConstraintIndex;
        int softPositionalConstraintSize() const { return softPositionalConstraints.size() * 3; }
        int hardPositionalConstraintSize() const { return hardPositionalConstraints.size() * 3; }
    };

    vector<ConstraintInfo> constraintInfoSeq;

    vector<MarkerMotionPtr> morphedMarkerMotions;

    vector<MarkerMotion::Frame> Vi0_frames; // original positions
    vector<MarkerMotion::Frame> Vi_frames;  // current (morphed) positions

    int m;  // the number of "active" vertices
    int m3; // the number of all the active vertex elements (m * 3)
    int m3n; // m3 * number of frames;
    int numAllBones; // the number of "active" bones

    // key: active vertex index of a vertex of an edge
    // value: global vertex index of the other vertex of the edge
    map<int, set<int> > boneEdgeMap;

    // For the deformation energy
    vector<MatrixXd> M;
    vector<VectorXd> b;
    vector<MatrixXd> MtM;
    double laplacianWeightPowerHalf;
    bool doUpdateLaplacianCoordinateConstantEveryFrame;

    // For hard constraints

    // Hard constraint matrices of all frames
    // preserve all frame values to set them A's top right part later
    vector<RmSparseMatrix> H; 
    VectorXd hi;

    // For soft constraints
    RmSparseMatrix Fi;
    VectorXd fi;
    Eigen::DiagonalMatrix<double, Eigen::Dynamic> Wi;  // Weight matrix

    double accTermWeight;

    struct AccTermInfo {
        int offset;
        double value;
        AccTermInfo(int offset, double value)
            : offset(offset), value(value) { }
    };
    vector< vector<AccTermInfo> > allAccTermInfos;

    vector<AccTermInfo> emptyAccTermInfos;

    // For final linear equation
    MatrixXd A00; // for calculating top left part of A
    CmSparseMatrix A;
    VectorXd y;
    VectorXd x; // solution
    MatrixXd Q; // used in solveLinearEquationForSingleFrame();

    Impl();
    void clear();
    void setInteractionMesh(MarkerIMeshPtr mesh);
    void setMocapMappingPair(int motionIndex, MocapMappingPtr org, MocapMappingPtr goal);
    void addPositionalConstraint(
        int motionIndex, int localMarkerIndex, const Vector3& pos, bool isRelative,
        int frame, double alpha, bool isSoft, double weight = 1000.0);

    const Vector3& orgVertexOfActiveIndex(int activeIndex) {
        const MarkerIMesh::LocalIndex& localIndex = mesh->activeToLocalIndex(activeIndex);
        return Vi0_frames[localIndex.motionIndex][localIndex.markerIndex];
    }
    const Vector3& orgVertexOfGlobalIndex(int globalIndex) {
        const MarkerIMesh::LocalIndex& localIndex = mesh->globalToLocalIndex(globalIndex);
        return Vi0_frames[localIndex.motionIndex][localIndex.markerIndex];
    }

    bool solve(int numSteps, ostream& os);
    void initVariables();
    void extractBones();
    void initFrame(int frame);
    void setLaplacianMatrixOfFrame(MarkerIMesh::Frame neighborsList);
    void setBoneLengthMatrixAndVectorOfFrame(RmSparseMatrix& Hi, double alpha);
    void setPositionalConstraintMatrixAndVectorsOfFrame(
        const vector<PositionalConstraint>& positionalConstraints,
        RmSparseMatrix& Ki, VectorXd& Pi, int rowOffset, double alpha);
    void initAccTermInfo();
    void setLinearEquationCoefficientsOfFrame();
    void setAllFrameUpplerRightHardConstraintTransposeTerm();
    bool solveLinearEquation();
    void solveLinearEquationForSingleFrame();
    void copySolution();        
};

}


MarkerIMeshSolver::MarkerIMeshSolver()
{
    impl = new Impl;
    setLaplacianWeightPower(1.0);
}    


MarkerIMeshSolver::Impl::Impl()
{
    doExcludeBoneEdgesFromLaplacianCoordinate = false;
    isSingleFrameMode = false;
    clear();
    accTermWeight = 0.0001;
}


MarkerIMeshSolver::~MarkerIMeshSolver()
{
    delete impl;
}


void MarkerIMeshSolver::clear()
{
    impl->clear();
}


void MarkerIMeshSolver::Impl::clear()
{
    mesh.reset();
    numAllBones = 0;
    mocapMappingInfos.clear();
    constraintInfoSeq.clear();
    message.clear();
    doUpdateLaplacianCoordinateConstantEveryFrame = true;
    
    // just for releasing memories
    morphedMarkerMotions.clear();
    boneEdgeMap.clear();
    Vi0_frames.clear();
    Vi_frames.clear();
    M.clear();
    MtM.clear();
    H.clear();
}


void MarkerIMeshSolver::setSingleFrameMode(bool on, int targetFrame)
{
    impl->isSingleFrameMode = on;
    impl->targetSingleFrame = targetFrame;
}


void MarkerIMeshSolver::setInteractionMesh(MarkerIMeshPtr mesh)
{
    impl->setInteractionMesh(mesh);
}


void MarkerIMeshSolver::Impl::setInteractionMesh(MarkerIMeshPtr mesh)
{
    this->mesh = mesh;
    m = mesh->numActiveVertices();
    m3 = m * 3;
    constraintInfoSeq.resize(isSingleFrameMode ? 1 : mesh->numFrames());

    morphedMarkerMotions.resize(mesh->numMotions());
}


void MarkerIMeshSolver::setMocapMappingPair(int motionIndex, MocapMappingPtr org, MocapMappingPtr goal)
{
    impl->setMocapMappingPair(motionIndex, org, goal);
}


void MarkerIMeshSolver::Impl::setMocapMappingPair(int motionIndex, MocapMappingPtr org, MocapMappingPtr goal)
{
    if(mesh && motionIndex < mesh->numMotions()){
        if(motionIndex >= mocapMappingInfos.size()){
            mocapMappingInfos.resize(motionIndex + 1);
        }
        MocapMappingInfo& info = mocapMappingInfos[motionIndex];
        if(!info.org){
            info.org = org;
            info.goal = goal;
        }
    }
}


void MarkerIMeshSolver::addSoftPositionalConstraint
(int motionIndex, int localMarkerIndex, const Vector3& pos, int frame, double alpha)
{
    impl->addPositionalConstraint(motionIndex, localMarkerIndex, pos, false, frame, alpha, true);
}


void MarkerIMeshSolver::addHardPositionalConstraint
(int motionIndex, int localMarkerIndex, const Vector3& pos, int frame, double alpha)
{
    impl->addPositionalConstraint(motionIndex, localMarkerIndex, pos, false, frame, alpha, false);
}


void MarkerIMeshSolver::addHardConstraintToKeepOriginalMarkerPosition
(int motionIndex, int markerIndex, const Vector3& offset)
{
    impl->addPositionalConstraint(motionIndex, markerIndex, offset, true, -1, 1.0, false);
}


void MarkerIMeshSolver::addSoftConstraintToKeepOriginalMarkerPosition
(int motionIndex, int markerIndex, double weight, const Vector3& offset)
{
    impl->addPositionalConstraint(motionIndex, markerIndex, offset, true, -1, 1.0, true, weight);
}


void MarkerIMeshSolver::Impl::addPositionalConstraint
(int motionIndex, int localMarkerIndex,
 const Vector3& pos, bool isRelative, int frame, double alpha, bool isSoft, double weight)
{
    if(motionIndex < 0 || localMarkerIndex < 0){
        return;
    }
    
    if(isSingleFrameMode){
        frame = 0;
    }
    int endFrame;
    if(frame >= 0){
        endFrame = frame + 1;
    } else {
        frame = 0;
        endFrame = mesh->numFrames();
    }
    
    PositionalConstraint c;
    c.activeVertexIndex = mesh->localToActiveIndex(motionIndex, localMarkerIndex);
    c.pos = pos;
    c.isRelative = isRelative;
    c.alpha = alpha;
    c.weight = weight;

    for(int i=frame; i < endFrame; ++i){
        ConstraintInfo& cinfo = constraintInfoSeq[i];
        if(isSoft){
            cinfo.softPositionalConstraints.push_back(c);
        } else {
            cinfo.hardPositionalConstraints.push_back(c);
        }
    }
}


void MarkerIMeshSolver::updateLaplacianCoordinateConstantEvreyFrame(bool on)
{
    impl->doUpdateLaplacianCoordinateConstantEveryFrame = on;
}


void MarkerIMeshSolver::excludeBoneEdgesFromLaplacianCoordinate(bool on)
{
    impl->doExcludeBoneEdgesFromLaplacianCoordinate = on;
}


void MarkerIMeshSolver::setLaplacianWeightPower(double pow)
{
    if(pow >= 0.0){
        impl->laplacianWeightPowerHalf = pow / 2.0;
    }
}


void MarkerIMeshSolver::setAccelConstraintWeight(double w)
{
    if(w > 0.0){
        impl->accTermWeight = w;
    }
}


void MarkerIMeshSolver::setMotionToOutputSolution(int motionIndex, MarkerMotionPtr motion)
{
    impl->morphedMarkerMotions[motionIndex] = motion;
}


bool MarkerIMeshSolver::solve(int numSteps, std::ostream& os)
{
    return impl->solve(numSteps, os);
}


bool MarkerIMeshSolver::Impl::solve(int numSteps, ostream& os)
{
    const bool useOldSolverForSingleFrame = false;

    os << _("Retargeting based on the intaraction mesh has started.") << endl;
    

    initAccTermInfo();
    extractBones();
    initVariables();

    numFrames = isSingleFrameMode ? 1 : mesh->numFrames();
    int targetFrame = targetSingleFrame;
    double alpha;

    bool solved = false;

    for(int step=0; step < numSteps; ++step){

        os << format(_("Step {0} is being processed."), step + 1) << endl;

        A.setZero();
        alpha = (double)(step + 1) / numSteps;
        
        for(currentFrame=0; currentFrame < numFrames; ++currentFrame){

            if(isSingleFrameMode){
                if(step == 0){
                    initFrame(targetFrame);
                }
            } else {
                targetFrame = currentFrame;
                initFrame(targetFrame);
            }

            // deformation energy
            if(step == 0){
                setLaplacianMatrixOfFrame(mesh->frame(targetFrame));

            } else if(doUpdateLaplacianCoordinateConstantEveryFrame){
                MatrixXd& Mi = M[currentFrame];
                VectorXd& bi = b[currentFrame];
                VectorXd Vi(m3);
                int index = 0;
                for(int i=0; i < m; ++i){
                    const MarkerIMesh::LocalIndex& lindex = mesh->activeToLocalIndex(i);
                    const Vector3& v = Vi_frames[lindex.motionIndex][lindex.markerIndex];
                    Vi[index++] = v[0];
                    Vi[index++] = v[1];
                    Vi[index++] = v[2];
                    //Vi.segment<3>(i * 3) = Vi_frames[index.motionIndex][index.markerIndex];
                }
                bi = Mi * Vi;
            }
                
            ConstraintInfo& cinfo = constraintInfoSeq[currentFrame];

            RmSparseMatrix& Hi = H[currentFrame];
            const int nhp = cinfo.hardPositionalConstraintSize();
            const int nh = numAllBones + nhp;
            Hi.resize(nh, m3); // resize() makes Hi zero, too.
            hi.resize(nh);
            
            // bone-length constraints
            if(numAllBones > 0){
                setBoneLengthMatrixAndVectorOfFrame(Hi, alpha);
            }

            // hard positional constraints
            if(nhp > 0){
                setPositionalConstraintMatrixAndVectorsOfFrame(
                    cinfo.hardPositionalConstraints,
                    Hi, hi, numAllBones, alpha);
            }

            Hi.finalize();

            const int nsp = cinfo.softPositionalConstraintSize();
            Fi.resize(nsp, m3);
            Fi.setZero();
            fi.resize(nsp);
            Wi.resize(nsp);
            if(nsp > 0){
                const vector<PositionalConstraint>& constraints = cinfo.softPositionalConstraints;
                
                // soft positional constraints
                setPositionalConstraintMatrixAndVectorsOfFrame(constraints, Fi, fi, 0, alpha);

                // set weights
                for(int i=0; i < constraints.size(); ++i){
                    const PositionalConstraint& c = constraints[i];
                    Wi.diagonal().segment<3>(i * 3).setConstant(c.weight);
                }
            }

            Fi.finalize();

            if(useOldSolverForSingleFrame && isSingleFrameMode){
                solveLinearEquationForSingleFrame();
                solved = true;
            } else {
                setLinearEquationCoefficientsOfFrame();
            }
        }

        if(!useOldSolverForSingleFrame || !isSingleFrameMode){
            setAllFrameUpplerRightHardConstraintTransposeTerm();
            A.finalize();
            solved = solveLinearEquation();
            if(!solved){
                break;
            }
        }
    }

    if(solved){
        os << _("Retargeting has completed.") << endl;
    } else {
        os << _("Retargeting failed.") << endl;
    }
    if(!message.empty()){
        os << message << endl;
    }

    return solved;
}


void MarkerIMeshSolver::Impl::initVariables()
{
    int n = isSingleFrameMode ? 1 : mesh->numFrames();
    m3n = m3 * n;

    M.resize(n);
    b.resize(n);
    MtM.resize(n);
    H.resize(n);

    // investigating the numbers of soft / hard constraints
    int totalSoftConstraintSize = 0;
    int totalHardConstraintSize = 0;
    int frame, end;
    if(isSingleFrameMode){
        frame = 0;
        end = 1;
    } else {
        frame = 0;
        end = constraintInfoSeq.size();
    }
    while(frame < end){
        ConstraintInfo& cinfo = constraintInfoSeq[frame++];
        cinfo.globalHardConstraintIndex = totalHardConstraintSize;
        totalSoftConstraintSize += cinfo.softPositionalConstraintSize();
        totalHardConstraintSize += numAllBones + cinfo.hardPositionalConstraintSize();
    }

    const int size = m3n + totalHardConstraintSize;
    A.resize(size, size);
    y.resize(size);
    x.resize(size);

    for(size_t i=0; i < morphedMarkerMotions.size(); ++i){
        MarkerMotionPtr org = mesh->motion(i);
        MarkerMotionPtr& morphed = morphedMarkerMotions[i];
        if(!morphed){
            morphed.reset(new MarkerMotion());
        }
        morphed->copySeqProperties(*org);

        int numMorphedFrames;
        int orgFrameBegin = 0;
        if(isSingleFrameMode){
            numMorphedFrames = 1;
            orgFrameBegin = targetSingleFrame;
        } else if(mesh->numLocalActiveVertices(i) > 0){
            numMorphedFrames = mesh->numFrames();
        } else {
            numMorphedFrames = org->numFrames();
        }
        morphed->setDimension(numMorphedFrames, org->numMarkers());

        for(int j = 0; j < numMorphedFrames; ++j){
            int orgFrameIndex = std::min(j + orgFrameBegin, org->numFrames() - 1);
            MarkerMotion::Frame orgFrame = org->frame(orgFrameIndex);
            MarkerMotion::Frame morphedFrame = morphed->frame(j);
            std::copy(orgFrame.begin(), orgFrame.end(), morphedFrame.begin());
        }
    }
}


void MarkerIMeshSolver::Impl::extractBones()
{
    boneEdgeMap.clear();
    
    for(size_t i=0; i < mocapMappingInfos.size(); ++i){
        MocapMappingInfo& chara = mocapMappingInfos[i];
        chara.bones.clear();
        if(chara.org){
            const MarkerMotionPtr& motion = mesh->motion(i);
            const int vertexIndexOffset = mesh->globalVertexIndexOffset(i);
            const int numBones = chara.org->numMarkerEdges();
            for(int j=0; j < numBones; ++j){
                const MocapMapping::Edge& orgEdge = chara.org->markerEdge(j);
                int pe1LocalIndex = motion->markerIndex(orgEdge.label[0]);
                int pe2LocalIndex = motion->markerIndex(orgEdge.label[1]);

                if(pe1LocalIndex > pe2LocalIndex){
                    std::swap(pe1LocalIndex, pe2LocalIndex); // sort
                }
                const int pe1GlobalIndex = vertexIndexOffset + pe1LocalIndex;
                const int pe1ActiveIndex = mesh->globalToActiveIndex(pe1GlobalIndex);
                if(pe1ActiveIndex >= 0){
                    const int pe2GlobalIndex = vertexIndexOffset + pe2LocalIndex;
                    const int pe2ActiveIndex = mesh->globalToActiveIndex(pe2GlobalIndex);
                    if(pe2ActiveIndex >= 0){
                        Bone bone;
                        bone.localMarkerIndex1 = pe1LocalIndex;
                        bone.localMarkerIndex2 = pe2LocalIndex;
                        bone.activeVertexIndex1 = pe1ActiveIndex;
                        bone.activeVertexIndex2 = pe2ActiveIndex;

                        if(chara.goal){
                            const MocapMapping::Edge& goalEdge = chara.goal->markerEdge(j);
                            bone.goalLength = goalEdge.length;
                        }
                        chara.bones.push_back(bone);
                        ++numAllBones;

                        if(doExcludeBoneEdgesFromLaplacianCoordinate){
                            boneEdgeMap[pe1ActiveIndex].insert(pe2GlobalIndex);
                            boneEdgeMap[pe2ActiveIndex].insert(pe1GlobalIndex);
                        }
                    }
                }
            }
        }
    }
}


void MarkerIMeshSolver::Impl::initFrame(int frame)
{
    Vi0_frames.clear();
    Vi_frames.clear();
    const int morphedFrameIndex = isSingleFrameMode ? 0 : frame;

    for(int i=0; i < mesh->numMotions(); ++i){
        MarkerMotionPtr motion = mesh->motion(i);
        const int maxFrame = motion->numFrames() - 1;
        MarkerMotion::Frame orgFrame = motion->frame(std::min(frame, maxFrame));
        Vi0_frames.push_back(orgFrame);
        MarkerMotion::Frame morphedFrame =
            morphedMarkerMotions[i]->frame(std::min(morphedFrameIndex, maxFrame));
        Vi_frames.push_back(morphedFrame);
    }
}


void MarkerIMeshSolver::Impl::setLaplacianMatrixOfFrame(MarkerIMesh::Frame neighborsList)
{
    MatrixXd& Mi = M[currentFrame];
    Mi.resize(m3, m3);
    Mi.setZero();
    VectorXd& bi = b[currentFrame];
    bi.resize(m3);
    
    vector<int> validNeighbors;
    
    for(int activeMarkerIndex=0; activeMarkerIndex < m; ++activeMarkerIndex){
        const MarkerIMesh::NeighborList& neighbors = neighborsList[activeMarkerIndex];
        const set<int>& boneCounterparts = boneEdgeMap[activeMarkerIndex];
        validNeighbors.clear();
        for(int k=0; k < neighbors.size(); ++k){
            int l = neighbors[k];
            if(boneCounterparts.find(l) == boneCounterparts.end()){ // check if the edge is bone
                validNeighbors.push_back(l);
            }
        }
        const Vector3& pj = orgVertexOfActiveIndex(activeMarkerIndex);
        const int j3 = activeMarkerIndex * 3;
        Eigen::Block<MatrixXd> a = Mi.block(j3, 0, 3, m3);
        VectorXd::FixedSegmentReturnType<3>::Type bij = bi.segment<3>(j3);
        bij.setZero();
        if(!validNeighbors.empty()){
            double wn = 1.0 / validNeighbors.size();
            for(int k=0; k < validNeighbors.size(); ++k){
                int l = validNeighbors[k];
                const Vector3& pl = orgVertexOfGlobalIndex(l);
                const double w = wn * 1.0 / pow((pj - pl).squaredNorm(), laplacianWeightPowerHalf);
                bij += w * pj;
                const int l_active = mesh->globalToActiveIndex(l);
                if(l_active >= 0){
                    for(int s=0; s < 3; ++s){
                        a(s, l_active * 3 + s) = -w;
                    }
                    bij -= w * pl;
                }
                for(int s=0; s < 3; ++s){
                    a(s, j3 + s) +=  w;
                    // Can .diagonal() be used here?
                }
            }
        }
    }

    MtM[currentFrame] = Mi.transpose() * Mi;
}


void MarkerIMeshSolver::Impl::setBoneLengthMatrixAndVectorOfFrame(RmSparseMatrix& Hi, double alpha)
{
    int globalBoneIndex = 0;
    
    for(size_t motionIndex=0; motionIndex < mocapMappingInfos.size(); ++motionIndex){

        MocapMappingInfo& chara = mocapMappingInfos[motionIndex];
        if(chara.org){
            const MarkerMotion::Frame& Vi0_frame = Vi0_frames[motionIndex];
            const int numBones = chara.bones.size();
            for(int j=0; j < numBones; ++j){
                const Bone& bone = chara.bones[j];
                MarkerMotionPtr motion = mesh->motion(motionIndex);

                // Set desired bone length in this morph step
                const Vector3& pe1_0 = Vi0_frame[bone.localMarkerIndex1];
                const Vector3& pe2_0 = Vi0_frame[bone.localMarkerIndex2];
                double lgoal = (pe2_0 - pe1_0).norm(); // set original length first
                if(bone.goalLength){
                    lgoal = alpha * (*bone.goalLength) + (1.0 - alpha) * lgoal;
                }
                
                const MarkerMotion::Frame& Vi_frame = Vi_frames[motionIndex];
                const Vector3& pe1 = Vi_frame[bone.localMarkerIndex1];
                const Vector3& pe2 = Vi_frame[bone.localMarkerIndex2];

                const double lcurrent = (pe1 - pe2).norm();

                double a;
                if(fabs(lcurrent) < 1.0e-6){
                    a = 0.0; // to make the following coefficient zero
                } else {
                    a = lgoal / lcurrent;
                }

                hi(globalBoneIndex) = (lgoal - lcurrent);

                Vector3 dl;
                if(fabs(lcurrent) < 1.0e-6){
                    dl.setZero();
                } else {
                    dl = (1.0 / lcurrent) * (pe1 - pe2);
                }

                Hi.startVec(globalBoneIndex);
                
                int offset = bone.activeVertexIndex1 * 3;
                for(int j=0; j < 3; ++j){
                    Hi.insertBack(globalBoneIndex, offset + j) = dl[j];
                    hi(globalBoneIndex) += dl[j] * pe1[j];
                }
                offset = bone.activeVertexIndex2 * 3;
                for(int j=0; j < 3; ++j){
                    Hi.insertBack(globalBoneIndex, offset + j) = -dl[j];
                    hi(globalBoneIndex) -= dl[j] * pe2[j];
                }
                
                ++globalBoneIndex;
            }
        }
    }
}


void MarkerIMeshSolver::Impl::setPositionalConstraintMatrixAndVectorsOfFrame
(const vector<PositionalConstraint>& positionalConstraints,
 RmSparseMatrix& Ki, VectorXd& Pi, int rowOffset, double alpha)
{
    for(int i=0; i < positionalConstraints.size(); ++i){
        int row = i * 3 + rowOffset;
        const PositionalConstraint& c = positionalConstraints[i];
        int col = c.activeVertexIndex * 3;
        for(int k=0; k < 3; ++k){
            Ki.startVec(row + k);
            Ki.insertBack(row + k, col + k) = 1.0;
        }
        if(c.isRelative){
            Pi.segment<3>(row) = orgVertexOfActiveIndex(c.activeVertexIndex) + c.pos;
        } else {
            alpha *= c.alpha;
            Pi.segment<3>(row) = alpha * c.pos + (1.0 - alpha) * orgVertexOfActiveIndex(c.activeVertexIndex);
        }
    }
}


void MarkerIMeshSolver::Impl::initAccTermInfo()
{
    const double dt = mesh->getTimeStep();
    const double k = accTermWeight / ((dt * dt) * (dt * dt));

    allAccTermInfos.clear();
    allAccTermInfos.resize(6);

    vector<AccTermInfo>& infos0 = allAccTermInfos[0];
    infos0.push_back(AccTermInfo(0,  1.0 * k));
    infos0.push_back(AccTermInfo(1, -2.0 * k));
    infos0.push_back(AccTermInfo(2,  1.0 * k));

    vector<AccTermInfo>& infos1 = allAccTermInfos[1];
    infos1.push_back(AccTermInfo(-1, -2.0 * k));
    infos1.push_back(AccTermInfo( 0,  5.0 * k));
    infos1.push_back(AccTermInfo( 1, -4.0 * k));
    infos1.push_back(AccTermInfo( 2,  1.0 * k));

    vector<AccTermInfo>& infos2 = allAccTermInfos[2];
    infos2.push_back(AccTermInfo(-2,  1.0 * k));
    infos2.push_back(AccTermInfo(-1, -4.0 * k));
    infos2.push_back(AccTermInfo( 0,  6.0 * k));
    infos2.push_back(AccTermInfo( 1, -4.0 * k));
    infos2.push_back(AccTermInfo( 2,  1.0 * k));

    vector<AccTermInfo>& infos3 = allAccTermInfos[3];
    infos3.push_back(AccTermInfo(-2,  1.0 * k));
    infos3.push_back(AccTermInfo(-1, -4.0 * k));
    infos3.push_back(AccTermInfo( 0,  5.0 * k));
    infos3.push_back(AccTermInfo( 1, -2.0 * k));

    vector<AccTermInfo>& infos4 = allAccTermInfos[4];
    infos4.push_back(AccTermInfo(-2,  1.0 * k));
    infos4.push_back(AccTermInfo(-1, -2.0 * k));
    infos4.push_back(AccTermInfo( 0,  1.0 * k));

    // special case for the single frame mode
    vector<AccTermInfo>& infos5 = allAccTermInfos[5];
    infos5.push_back(AccTermInfo(0, 0.0));
}


void MarkerIMeshSolver::Impl::setLinearEquationCoefficientsOfFrame()
{
    const int offset = currentFrame * m3;
    const CmSparseMatrix Hic(H[currentFrame]);
    const int hoffset = m3n + constraintInfoSeq[currentFrame].globalHardConstraintIndex;

    A00 = MtM[currentFrame];

    if(Fi.rows() > 0){
        A00 += RmSparseMatrix(Fi.transpose() * Wi * Fi);
    }

    int accTermInfoIndex;

    if(isSingleFrameMode){
        accTermInfoIndex = 5;
    } else {
        if(currentFrame >= 2){
            if(currentFrame < numFrames - 2){
                accTermInfoIndex = 2;
            } else if(currentFrame == numFrames - 2){
                accTermInfoIndex = 3;
            } else {
                accTermInfoIndex = 4;
            }
        } else if(currentFrame == 1){
            accTermInfoIndex = 1;
        } else {
            accTermInfoIndex = 0;
        }
    }

    const vector<AccTermInfo>& accTermInfos = allAccTermInfos[accTermInfoIndex];
    
    for(int i=0; i < m3; ++i){
        int col = offset + i;
        A.startVec(col);

        for(int j=0; j < accTermInfos.size(); ++j){
            const AccTermInfo& accTerm = accTermInfos[j];
            if(accTerm.offset != 0){
                A.insertBack((currentFrame + accTerm.offset) * m3 + i, col) = accTerm.value;

            } else {
                A00(i, i) += accTerm.value;
                // main part of the top left part (Mt M + Ft W F)
                for(int k=0; k < m3; ++k){
                    const double value = A00(k, i);
                    if(value != 0.0){
                        A.insertBack(offset + k, col) = value;
                    }
                }
            }
        }
        // hard constraint coefficients
        for(CmSparseMatrix::InnerIterator it(Hic, i); it; ++it){
            A.insertBack(hoffset + it.row(), col) = it.value();
        }
    }

    if(fi.size() == 0){
        y.segment(offset, m3) = M[currentFrame].transpose() * b[currentFrame];
    } else {
        y.segment(offset, m3) = M[currentFrame].transpose() * b[currentFrame] + Fi.transpose() * Wi * fi;
    }
    if(hi.size() > 0){    
        y.segment(hoffset, hi.size()) = hi;
    }
}


void MarkerIMeshSolver::Impl::setAllFrameUpplerRightHardConstraintTransposeTerm()
{
    for(int i=0; i < numFrames; ++i){
        const int rowTop = i * m3;
        const int colTop = m3n + constraintInfoSeq[i].globalHardConstraintIndex;
        const RmSparseMatrix& Hi = H[i];
        for(int j=0; j < Hi.rows(); ++j){
            int col = colTop + j;
            A.startVec(col);
            for(RmSparseMatrix::InnerIterator it(Hi, j); it; ++it){
                A.insertBack(rowTop + it.col(), col) = it.value();
            }
        }
    }
}


bool MarkerIMeshSolver::Impl::solveLinearEquation()
{
    Eigen::UmfPackLU<CmSparseMatrix> lu_of_A(A);
    if(lu_of_A.info() != Eigen::Success){
        message += "LU factorizaion failed.";
        return false;
    }
    x = lu_of_A.solve(y);
    if(lu_of_A.info() != Eigen::Success){
        message += "Linear equation cannot be solved.";
        return false;
    }

    copySolution();
    
    return true;
}


void MarkerIMeshSolver::Impl::solveLinearEquationForSingleFrame()
{
    const MatrixXd& Mi = M[currentFrame];
    const MatrixXd& Hi = H[currentFrame];
    
    Q.resize(Mi.cols() + Hi.rows(), Mi.cols() + Hi.rows());
    y.resize(Mi.cols() + Hi.rows());

    if(Fi.rows() == 0){
        Q.topLeftCorner(Mi.cols(), Mi.cols()) = MtM[currentFrame];
        y.head(Mi.cols()) = Mi.transpose() * b[currentFrame];
    } else {
        Q.topLeftCorner(Mi.cols(), Mi.cols()) = MtM[currentFrame];
        Q.topLeftCorner(Mi.cols(), Mi.cols()) += RmSparseMatrix(Fi.transpose() * Wi * Fi);
        y.head(Mi.cols()) = Mi.transpose() * b[currentFrame] + Fi.transpose() * Wi * fi;
    }
    if(Hi.rows() > 0){    
        Q.topRightCorner(Hi.cols(), Hi.rows()) = Hi.transpose();
        Q.bottomLeftCorner(Hi.rows(), Hi.cols()) = Hi;
        Q.bottomRightCorner(Hi.rows(), Hi.rows()).setZero();
        y.tail(Hi.rows()) = hi;
    }

    //x = Q.colPivHouseholderQr().solve(y);
    x = Q.householderQr().solve(y);

    copySolution();
}


void MarkerIMeshSolver::Impl::copySolution()
{
    int numAllFrames = isSingleFrameMode ? 1 : mesh->numFrames();
    for(size_t i=0; i < morphedMarkerMotions.size(); ++i){
        MarkerMotionPtr morphed = morphedMarkerMotions[i];
        int index = 0;
        const std::vector<int>& localToActiveIndexMap = mesh->localToActiveIndexMap(i);
        MarkerMotionPtr orgMotion = mesh->motion(i);
        const int orgMaxFrame = orgMotion->numFrames() - 1;
        const int n = std::min(numAllFrames, morphed->numFrames());
        for(int j=0; j < n; ++j){
            MarkerMotion::Frame morphedFrame = morphed->frame(j);
            const int frameOffset = m3 * j;
            const MarkerMotion::Frame orgFrame = orgMotion->frame(std::min(j, orgMaxFrame));
            for(int k=0; k < morphedFrame.size(); ++k){
                const int activeIndex = localToActiveIndexMap[k];
                if(activeIndex >= 0){
                    morphedFrame[k] = x.segment<3>(frameOffset + activeIndex * 3);
                } else {
                    morphedFrame[k] = orgFrame[k];
                }
            }
        }
    }
}


MarkerMotionPtr MarkerIMeshSolver::solution(int index)
{
    return impl->morphedMarkerMotions[index];
}


const std::string& MarkerIMeshSolver::message() const
{
    return impl->message;
}
