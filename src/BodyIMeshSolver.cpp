#include "BodyIMeshSolverImpl.h"
#include <cnoid/EigenUtil>


BodyIMeshSolver::BodyIMeshSolver(std::ostream& os)
{
    impl = new BodyIMeshSolverImpl(os);
}


BodyIMeshSolverImpl::BodyIMeshSolverImpl(std::ostream& os)
{
    numSteps = 10;
    numStepsOfShrinkingVelocityLimits = 5;
    numStepsOfKeepingVelocityConstraints = 2;
    
    isFixedRootMode = false;
    isHalfwayMotionRecordingEnabled = false;
    numHalfwayMotions = 0;
    jointDampingWeight = 0.02;
    maxLaplacianDiffToOrgPosition = 0.001;
    accTermWeight = 1.0e-8;
    os_ = &os;
}


BodyIMeshSolver::~BodyIMeshSolver()
{
    delete impl;
}


void BodyIMeshSolver::clear()
{
    impl->clear();
}


void BodyIMeshSolverImpl::clear()
{
    mesh->clear();
    bodies.clear();
    posePairs.clear();
    numHalfwayMotions = 0;
}


void BodyIMeshSolver::setNumSteps(int n)
{
    impl->numSteps = n;
}


void BodyIMeshSolver::setNumStepsOfShrinkingVelocityLimits(int n)
{
    impl->numStepsOfShrinkingVelocityLimits = n;
}


void BodyIMeshSolver::setNumStepsOfKeepingVelocityConstraints(int n)
{
    impl->numStepsOfKeepingVelocityConstraints = n;
}


void BodyIMeshSolver::setFixedRootMode(bool on)
{
    impl->isFixedRootMode = on;
}


void BodyIMeshSolver::enableHalfwayMotionRecording(bool on)
{
    impl->isHalfwayMotionRecordingEnabled = on;
    if(!on){
        impl->numHalfwayMotions = 0;
    }
}


int BodyIMeshSolver::numHalfwayMotions() const
{
    return impl->numHalfwayMotions;
}


std::shared_ptr<BodyMotion> BodyIMeshSolver::halfwayMotion(int bodyIndex, int motionIndex)
{
    return impl->bodies[bodyIndex]->halfwayMotions[motionIndex];
}

    
void BodyIMeshSolver::setInteractionMesh(BodyIMeshPtr mesh)
{
    impl->setInteractionMesh(mesh);
}


void BodyIMeshSolverImpl::setInteractionMesh(BodyIMeshPtr mesh)
{
    this->mesh = mesh;
    bodies.clear();
    rootFreeBodies.clear();
    numMarkers = 0;
    dimJointSpace = 0;
    numValidJoints = 0;
    
    for(int i=0; i < mesh->numBodies(); ++i){

        BodyPtr body = mesh->bodyInfo(i).body;

        const vector<BodyIMesh::MarkerPtr>& markers = mesh->bodyInfo(i).markers;
        numMarkers += markers.size();

        BodyInfoPtr bodyInfo = make_shared<BodyInfo>();
        bodies.push_back(bodyInfo);
        bodyInfo->body = body;
        bodyInfo->globalJointSpaceOffset = dimJointSpace;

        if(!body->isFixedRootModel() && !isFixedRootMode){
            dimJointSpace += 6; // Root 6 DOF
            rootFreeBodies.push_back(bodyInfo);
        }
        bodyInfo->globalJointIndexOffset = dimJointSpace;

        extractValidJoints(bodyInfo, markers);

        const int nj = bodyInfo->validJoints.size();
        numValidJoints += nj;
        dimJointSpace += nj;
    }

    dimMarkerSpace = numMarkers * 3;

    allMarkerPositions.resize(numMarkers);
    
    Jm.resize(dimMarkerSpace, dimJointSpace);
    J.resize(dimMarkerSpace, dimJointSpace);

    totalNumFrames = mesh->numFrames();

    globalDimJointSpace = dimJointSpace * totalNumFrames;
    allFrameVariables.clear();
    allFrameVariables.resize(totalNumFrames + 1);

    jointRangeConstraints.reserve(dimJointSpace);
    jointRangeConstraints.clear();

    if(USE_JOINT_CONSTRAINT_MASK){
        jointConstraintMask.resize(totalNumFrames, numValidJoints);
        std::fill(jointConstraintMask.begin(), jointConstraintMask.end(), 0);
    }
}


void BodyIMeshSolverImpl::extractValidJoints(BodyInfoPtr bodyInfo, const vector<BodyIMesh::MarkerPtr>& markers)
{
    BodyPtr body = bodyInfo->body;

    vector<JointInfoPtr> jointInfos(body->numLinks());
    for(size_t i=0; i < jointInfos.size(); ++i){
        jointInfos[i] = make_shared<JointInfo>(body->link(i));
    }

    for(size_t i=0; i < markers.size(); ++i){
        const BodyIMesh::MarkerPtr& marker = markers[i];
        Link* link = marker->link;
        if(!marker->localPos && link->isRevoluteJoint()){
            link = link->parent();
        }
        while(link){
            jointInfos[link->index()]->affectedMarkerIndices.push_back(i);
            link = link->parent();
        }
    }

    vector<JointInfoPtr>& validJoints = bodyInfo->validJoints;
    for(size_t i=0; i < jointInfos.size(); ++i){
        JointInfoPtr& jointInfo = jointInfos[i];
        Link* link = jointInfo->link;
        if(link->jointId() >= 0 &&
           !jointInfo->affectedMarkerIndices.empty() &&
           (link->isRevoluteJoint() || link->isPrismaticJoint())){
            validJoints.push_back(jointInfo);
        }
    }

    vector<int>& jmap = bodyInfo->jointIdToValidJointIndexMap;
    jmap.clear();
    jmap.resize(body->numJoints(), -1);
    for(int i=0; i < validJoints.size(); ++i){
        jmap[validJoints[i]->link->jointId()] = i;
    }
}


void BodyIMeshSolver::addHardConstraintToKeepOriginalLinkPosition(int bodyIndex, int linkIndex)
{
    impl->addHardConstraintToKeepOriginalLinkPosition(bodyIndex, linkIndex);
}


void BodyIMeshSolverImpl::addHardConstraintToKeepOriginalLinkPosition(int bodyIndex, int linkIndex)
{
    LinkConstraint constraint;
    BodyInfoPtr& bodyInfo = bodies[bodyIndex];
    constraint.bodyInfo = bodyInfo;
    constraint.link = bodyInfo->body->link(linkIndex);

    for(int i=0; i < mesh->numFrames(); ++i){
        allFrameVariables[i].hardLinkConstraints.push_back(make_shared<LinkConstraint>(constraint));
    }
}


void BodyIMeshSolver::setJointDampingWeight(double w)
{
    if(w >= 0.0){
        impl->jointDampingWeight = w;
    }
}


void BodyIMeshSolver::setMaxLaplacianDiffToOrgPosition(double d)
{
    if(d >= 0.0){
        impl->maxLaplacianDiffToOrgPosition = d;
    }
}


void BodyIMeshSolver::setAccTermWeight(double w)
{
    if(w >= 0.0){
        impl->accTermWeight = w;
    }
}


void BodyIMeshSolverImpl::initAccTermInfo()
{
    const double dt = mesh->getTimeStep();
    //const double k = accTermWeight / ((dt * dt) * (dt * dt));
    const double k = 1.0 / ((dt * dt) * (dt * dt));

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

    int index = 0;
    accTermWeights.resize(dimJointSpace);
    for(int i=0; i < mesh->numBodies(); ++i){
        BodyInfo& info = *bodies[i];
        BodyPtr& body = info.body;

        // weight different from usual joint one should be specified for the root?
        if(!body->isFixedRootModel() && !isFixedRootMode){
            // root translation
            accTermWeights.segment<3>(index).setConstant(accTermWeight);
            index += 3;
            // root rotation
            accTermWeights.segment<3>(index).setConstant(accTermWeight);
            index += 3;
        }
        const int numValidJoints = info.validJoints.size();
        accTermWeights.segment(index, numValidJoints).setConstant(accTermWeight);
        index += numValidJoints;
    }

}


void BodyIMeshSolverImpl::storeInitialRootTranslations()
{
    if(rootFreeBodies.empty()){
        return;
    }

    orgRootTranslations.setFrameRate(mesh->frameRate());
    orgRootTranslations.setDimension(totalNumFrames, rootFreeBodies.size());

    for(size_t i=0; i < rootFreeBodies.size(); ++i){
        BodyInfo& info = *rootFreeBodies[i];
        auto motion = info.currentStepMotion;
        auto dest = orgRootTranslations.part(i);
        int frameIndex = 0;
        auto srcSeq = motion->stateSeq();
        Vector3 last;
        if(!motion || isSinglePoseMode){
            last = info.body->rootLink()->p();
        } else {
            while(frameIndex < srcSeq->numFrames()){
                const auto& srcFrame = srcSeq->frame(frameIndex);
                dest[frameIndex] = srcFrame.linkPosition(0).translation();
                ++frameIndex;
            }
            last = dest[frameIndex - 1];
        }
        while(frameIndex < dest.size()){
            dest[frameIndex++] = last;
        }
    }
}


void BodyIMeshSolverImpl::updateCurrentBodyPositions(int step, int frameIndex)
{
    VectorXd& jointSpacePositions = allFrameVariables[frameIndex].jointSpacePositions;
    if(step == 0){
        jointSpacePositions.resize(dimJointSpace);
    }
    
    for(size_t i=0; i < bodies.size(); ++i){

        BodyInfo& info = *bodies[i];
        BodyPtr& body = info.body;
        const auto& motion = info.currentStepMotion;

        if(motion){
            auto sseq = motion->stateSeq();

            int actualFrameIndex;
            if(isSinglePoseMode){
                actualFrameIndex = step;
            } else {
                actualFrameIndex = std::min(frameIndex, sseq->numFrames() - 1);
            }
            const auto& frame = sseq->frame(actualFrameIndex);

            int jointSpaceOffset = info.globalJointSpaceOffset;
        
            if(!body->isFixedRootModel() && !isFixedRootMode){
                auto p = frame.linkPosition(0);
                auto rootLink = body->rootLink();
                rootLink->setTranslation(p.translation());
                rootLink->setRotation(p.rotation());
                jointSpacePositions.segment<3>(jointSpaceOffset) = p.translation();
                jointSpacePositions.segment<3>(jointSpaceOffset + 3).setZero();
                jointSpaceOffset += 6;
            }

            const vector<int>& jointIdToValidJointIndexMap = info.jointIdToValidJointIndexMap;
            const int nj = std::min(body->numJoints(), frame.numJointDisplacements());
            const auto displacements = frame.jointDisplacements();
            for(int j=0; j < nj; ++j){
                const double q = displacements[j];
                body->joint(j)->q() = q;
                const int validJointIndex = jointIdToValidJointIndexMap[j];
                if(validJointIndex >= 0){
                    jointSpacePositions[jointSpaceOffset + validJointIndex] = q;
                }
            }

            body->calcForwardKinematics();
        }
    }
}


void BodyIMeshSolverImpl::calcMarkerJacobian()
{
    int jointIndex = 0;

    for(size_t i=0; i < bodies.size(); ++i){

        BodyInfoPtr& bodyInfo = bodies[i];
        BodyPtr& body = bodyInfo->body;
        BodyIMesh::BodyInfo& meshBodyInfo = mesh->bodyInfo(i);
        const int markerIndexOffset = meshBodyInfo.markerIndexOffset;
        const vector<BodyIMesh::MarkerPtr>& markers = meshBodyInfo.markers;

        if(!body->isFixedRootModel() && !isFixedRootMode){
            calcJacobianRootDOFcolumn(body->rootLink(), markers, markerIndexOffset, jointIndex);
            jointIndex += 6;
        }

        vector<JointInfoPtr>& validJoints = bodyInfo->validJoints;
        for(size_t j=0; j < validJoints.size(); ++j){
            JointInfo& jointInfo = *validJoints[j];
            calcJacobianJointColumn(jointInfo, markers, markerIndexOffset, jointIndex);
            jointIndex += 1;
        }
    }
}


//! \todo clear other body elements 
void BodyIMeshSolverImpl::calcJacobianRootDOFcolumn
(Link* rootLink, const vector<BodyIMesh::MarkerPtr>& localMarkers, int markerIndexOffset, int jointIndexOffset)
{
    int row = markerIndexOffset * 3;
    const Vector3& rootPosition = rootLink->p();

    for(size_t i=0; i < localMarkers.size(); ++i){

        // root translation
        Jm.block<3, 3>(row, jointIndexOffset).setIdentity();

        // root rotation
        const BodyIMesh::MarkerPtr& marker = localMarkers[i];
        const Link* link = marker->link;
        const Vector3& p = allMarkerPositions[markerIndexOffset + i];
        Vector3 omega = Vector3::Zero();
        for(int j=0; j < 3; ++j){
            omega[j] = 1.0;
            Jm.block<3, 1>(row, jointIndexOffset + 3 + j) = omega.cross(p - rootPosition);
            omega[j] = 0.0;
        }
        
        row += 3;
    }
}


void BodyIMeshSolverImpl::calcJacobianJointColumn
(JointInfo& jointInfo, const vector<BodyIMesh::MarkerPtr>& localMarkers, int markerIndexOffset, int jointIndex)
{
    MatrixXd::ColXpr Jm_j = Jm.col(jointIndex);
    const Link* joint = jointInfo.link;
    const vector<int>& affectedMarkerIndices = jointInfo.affectedMarkerIndices;

    int row = 0;
    for(size_t i=0; i < affectedMarkerIndices.size(); ++i){
        const int localMarkerIndex = affectedMarkerIndices[i];
        const int globalMarkerIndex = localMarkerIndex + markerIndexOffset;
        const int nextRow = globalMarkerIndex * 3;
        if(nextRow > row){
            Jm_j.segment(row, nextRow - row).setZero();
            row = nextRow;
        }
        const Link* link = localMarkers[localMarkerIndex]->link;

        if(link->isRevoluteJoint()){
            const Vector3 omega = joint->R() * joint->a();
            const Vector3 arm = allMarkerPositions[globalMarkerIndex] - joint->p();
            Jm_j.segment(row, 3) = omega.cross(arm);

        } else if(link->isPrismaticJoint()){
            Jm_j.segment(row, 3) = joint->R() * joint->d();

        } else {
            Jm_j.segment(row, 3).setZero();
        }
        row += 3;
    }

    const int numRemainingRows = Jm_j.rows() - row;
    if(numRemainingRows > 0){
        Jm_j.segment(row, numRemainingRows).setZero();
    }
}


void BodyIMeshSolverImpl::calcLaplacianCoordinateJacobian(int frameIndex)
{
    calcMarkerJacobian();
    
    const BodyIMesh::Frame neighborLists = mesh->frame(frameIndex);
    FrameVariableSet& vars = allFrameVariables[frameIndex];
    
    for(int i=0; i < dimJointSpace; ++i){
        int row = 0;
        for(int j=0; j < numMarkers; ++j){

            Vector3 dL = Jm.block<3, 1>(row, i);
            
            const BodyIMesh::NeighborList& neighbors = neighborLists[j];
            const vector<double>& weights = vars.allWeights[j];
            const vector<double>& lengths = vars.allLengths[j];
            
            for(int l=0; l < neighbors.size(); ++l){
                const int lth = neighbors[l];
                const double w_l = weights[l];
                dL -= w_l * Jm.block<3, 1>(lth * 3, i);

                const Vector3& pj = allMarkerPositions[j];
                const Vector3& pl = allMarkerPositions[lth];
                const Eigen::Block<MatrixXd, 3, 1> dpj = Jm.block<3, 1>(j * 3, i);
                const Eigen::Block<MatrixXd, 3, 1> dpl = Jm.block<3, 1>(lth * 3, i);

                double dw_l = 0.0;
                for(int k=0; k < neighbors.size(); ++k){
                    const int kth = neighbors[k];
                    const Vector3& pk = allMarkerPositions[kth];
                    const Eigen::Block<MatrixXd, 3, 1> dpk = Jm.block<3, 1>(kth * 3, i);
                    const double len_l = lengths[l];
                    const double len_k = lengths[k];
                    const double a = ((pj - pl).dot(dpj - dpl) / len_l) * len_k;
                    const double b = len_l * ((pj - pk).dot(dpj - dpk) / len_k);
                    //dw_l -= (a - b) / len_k * len_k;
                    dw_l -= (a - b) / (len_k * len_k);
                }
                dw_l *= w_l * w_l;
                dL -= dw_l * pl;
            }
            J.block<3, 1>(row, i) = dL;
            row += 3;
        }
    }
}


void BodyIMeshSolverImpl::calcLaplacianCoordinate(int frameIndex, VectorXd& out_L, bool updateWeights)
{
    out_L.resize(dimMarkerSpace);
    BodyIMesh::Frame neighborLists = mesh->frame(frameIndex);
    FrameVariableSet& vars = allFrameVariables[frameIndex];
    vector< vector<double> >& allWeights = vars.allWeights;
    vector< vector<double> >& allLengths = vars.allLengths;

    if(updateWeights){
        allWeights.resize(numMarkers);
        allLengths.resize(numMarkers);
    }
    
    for(int i=0; i < numMarkers; ++i){
        const BodyIMesh::NeighborList& neighbors = neighborLists[i];
        const int numNeighbors = neighbors.size();

        const Vector3& pi = allMarkerPositions[i];

        vector<double>& weights = allWeights[i];
        vector<double>& lengths = allLengths[i];

        Eigen::Block<VectorXd, 3, 1> l = out_L.segment<3>(i * 3);

        if(!updateWeights){
            l = pi;
            for(int j=0; j < numNeighbors; ++j){
                l -= weights[j] * allMarkerPositions[neighbors[j]];
            }

        } else {
            weights.resize(numNeighbors);
            lengths.resize(numNeighbors);
            double weightSum = 0.0;
            for(int j=0; j < numNeighbors; ++j){
                lengths[j] = (pi - allMarkerPositions[neighbors[j]]).norm();
                weights[j] = 1.0 / lengths[j];
                weightSum += weights[j];
            }
            if(DEBUG){
                /*
                cout << i << "th lengths = ";
                for(size_t j=0; j < lengths.size(); ++j){
                    cout << lengths[j] << ",";
                }
                cout << endl;
                */
            }
            l = pi;
            for(int j=0; j < numNeighbors; ++j){
                weights[j] /= weightSum;
                l -= weights[j] * allMarkerPositions[neighbors[j]];
            }
        }
    }
}


void BodyIMeshSolverImpl::detectJointOverruns(int frameIndex, double dqRatio, bool fixExistingVelocityConstraints)
{
    jointRangeConstraints.clear();

    Array2D<char>::Row mask;
    if(USE_JOINT_CONSTRAINT_MASK){
        mask = jointConstraintMask[frameIndex];
    }

    const double frate = mesh->frameRate();
    FrameVariableSet& variables = allFrameVariables[frameIndex];
    vector<VelocityConstraint>& velocityConstraints = variables.velocityConstraints;
    velocityConstraints.clear();
    auto& velocityConstraintFlags = variables.velocityConstraintFlags;
    velocityConstraintFlags.resize(dimJointSpace, false);

    for(size_t i=0; i < bodies.size(); ++i){

        BodyInfo& bodyInfo = *bodies[i];
        int maskIndex = bodyInfo.globalJointIndexOffset;
        int globalJointIndex = bodyInfo.globalJointIndexOffset;

        auto sseq = bodyInfo.currentStepMotion->stateSeq();
        const auto& nextFrame = sseq->frame(std::min(frameIndex + 1, sseq->numFrames() - 1));

        vector<JointInfoPtr>& validJoints = bodyInfo.validJoints;
        for(size_t j=0; j < validJoints.size(); ++j){
            JointInfo& jointInfo = *validJoints[j];
            Link* link = jointInfo.link;
            bool isRangeOver = false;

            if(link->q() > link->q_upper()){
                const double dq = (link->q_upper() - link->q()) * dqRatio;
                jointRangeConstraints.push_back(JointConstraint(globalJointIndex, dq));
                isRangeOver = true;
                if(USE_JOINT_CONSTRAINT_MASK){
                    mask[maskIndex] = 1;
                }

            } else if(link->q() < link->q_lower()){
                const double dq = (link->q_lower() - link->q()) * dqRatio;
                jointRangeConstraints.push_back(JointConstraint(globalJointIndex, dq));
                isRangeOver = true;
                if(USE_JOINT_CONSTRAINT_MASK){
                    mask[maskIndex] = 1;
                }

            } else if(USE_JOINT_CONSTRAINT_MASK && mask[maskIndex]){
                isRangeOver = true;
                jointRangeConstraints.push_back(JointConstraint(globalJointIndex, 0));
            }

            if(!isRangeOver){
                const double qNext = nextFrame.jointDisplacement(link->jointId());
                if(qNext >= link->q_lower() && qNext <= link->q_upper()){
                    const double v = (qNext - link->q()) * frate;
                
                    if(v > link->dq_upper()){
                        const double goal = (v - link->dq_upper()) * (1.0 - dqRatio) + link->dq_upper();
                        //const double goal = (v - link->uvlimit) * (1.0 - dqRatio) + link->uvlimit - 0.005;
                        //const double goal = link->uvlimit;
                        //const double goal = 0.0;
                        velocityConstraints.push_back(VelocityConstraint(globalJointIndex, goal));
                        if(fixExistingVelocityConstraints){
                            velocityConstraintFlags[globalJointIndex] = true;
                        }
                    } else if(v < link->dq_lower()){
                        const double goal = (v - link->dq_lower()) * (1.0 - dqRatio) + link->dq_lower();
                        //const double goal = (v - link->lvlimit) * (1.0 - dqRatio) + link->lvlimit + 0.005;
                        //const double goal = link->lvlimit;
                        //const double goal = 0.0;
                        velocityConstraints.push_back(VelocityConstraint(globalJointIndex, goal));
                        if(fixExistingVelocityConstraints){
                            velocityConstraintFlags[globalJointIndex] = true;
                        }
                    } else if(fixExistingVelocityConstraints && velocityConstraintFlags[globalJointIndex]){
                        double goal;
                        if(fabs(v - link->dq_upper()) <= fabs(v - link->dq_lower())){
                            goal = link->dq_upper();
                        } else {
                            goal = link->dq_lower();
                        }
                        velocityConstraints.push_back(VelocityConstraint(globalJointIndex, goal));
                    }
                }
            }
                
            ++globalJointIndex;

            if(USE_JOINT_CONSTRAINT_MASK){
                ++maskIndex;
            }
        }
    }
    velocityConstraints.push_back(VelocityConstraint(-1, 0.0)); // append the end marker
}


void BodyIMeshSolverImpl::calcLinkPositionConstraintCoefficients
(vector<LinkConstraintPtr>& constraints, MatrixXd& Ci, VectorXd& hi)
{
    int row = 0;
    for(size_t i=0; i < constraints.size(); ++i){

        LinkConstraint& constraint = *constraints[i];
        const BodyInfo& bodyInfo = *constraint.bodyInfo;
        const BodyPtr& body = bodyInfo.body;
        const vector<JointInfoPtr>& joints = bodyInfo.validJoints;
        const Link* targetLink = constraint.link;
        const Vector3 p = targetLink->p();

        if(!constraint.pos){
            constraint.pos = SE3(p, targetLink->R());
        }
            
        int colOffset = bodyInfo.globalJointSpaceOffset;
        if(!body->isFixedRootModel() && !isFixedRootMode){

            Ci.block<3, 3>(row, colOffset).setIdentity();
            colOffset += 3;

            const Vector3 rootPosition = body->rootLink()->p();
            Vector3 omega = Vector3::Zero();
            for(int j=0; j < 3; ++j){
                omega[j] = 1.0;
                Ci.block<3, 1>(row, colOffset + j) = omega.cross(p - rootPosition);
                omega[j] = 0.0;
            }
            Ci.block<3, 3>(row + 3, colOffset).setIdentity();
            colOffset += 3;
        }

        const Link* joint = targetLink;
        const Link* rootLink = body->rootLink();
        while(joint != rootLink){
            const int validJointIndex = bodyInfo.jointIdToValidJointIndexMap[joint->jointId()];
            if(validJointIndex >= 0){
                const int col = colOffset + validJointIndex;
                if(joint->isRevoluteJoint()){
                    const Vector3 omega = joint->R() * joint->a();
                    const Vector3 arm = p - joint->p();
                    Ci.block<3, 1>(row, col) = omega.cross(arm);
                    Ci.block<3, 1>(row + 3, col) = omega;
                    
                } else if(joint->isPrismaticJoint()){
                    Ci.block<3, 1>(row, col) = joint->R() * joint->d();
                }
            }
            joint = joint->parent();
        }

        const SE3& org = *constraint.pos;
        const Vector3 dp = org.translation() - p;
        const Vector3 omega = targetLink->R() * omegaFromRot(targetLink->R().transpose() * org.rotation());
        hi.segment<3>(row) = dp;
        hi.segment<3>(row + 3) = omega;
            
        row += 6;
    }
}


bool BodyIMeshSolver::apply()
{
    return impl->apply();
}


bool BodyIMeshSolverImpl::apply()
{
    isSinglePoseMode = true;

    numHalfwayMotions = 1;

    for(int i=0; i < mesh->numBodies(); ++i){
        auto motion = mesh->bodyInfo(i).motion;
        bodies[i]->currentStepMotion = motion;
        if(motion){
            if(isHalfwayMotionRecordingEnabled){
                isSinglePoseMode = false;
            }
        }
    }
    if(!isSinglePoseMode){
        numHalfwayMotions = numSteps;
    }

    for(size_t i=0; i < bodies.size(); ++i){
        BodyInfo& info = *bodies[i];
        info.halfwayMotions.resize(numHalfwayMotions);
        for(int j=0; j < numHalfwayMotions; ++j){
            auto orgMotion = mesh->bodyInfo(i).motion;
            auto motion = make_shared<BodyMotion>();
            motion->setFrameRate(mesh->frameRate());
            int n;
            if(isSinglePoseMode){
                n = numSteps + 1;
            } else if(orgMotion){
                n = totalNumFrames;
            } else {
                n = 1;
            }
            auto sseq = motion->stateSeq();
            sseq->setNumLinkPositionsHint(1);
            sseq->setNumJointDisplacementsHint(info.body->numJoints());
            motion->setNumFrames(n);
            info.halfwayMotions[j] = motion;
        }
        if(isSinglePoseMode){
            info.currentStepMotion = info.halfwayMotions.front();
            info.currentStepMotion->frame(0) << *info.body;
        }
    }

    initAccTermInfo();
    isAccTermEnabled = (!isSinglePoseMode && accTermWeight != 0.0);

    storeInitialRootTranslations();
    
    mesh->update();

    VectorXd L_current;
    VectorXd dL;

    for(int step = 0; step < numSteps; ++step){

        //const double dqRatio = 1.0 / std::max(1.0, (double)(numSteps - step - 4));
        const double s = std::min(numStepsOfShrinkingVelocityLimits, step + 1);
        const double dqRatio = s / numStepsOfShrinkingVelocityLimits;

        os() << "Step " << step << ", dq ratio " << dqRatio;

        bool keepVelocityConstraints = (step >= (numSteps - numStepsOfKeepingVelocityConstraints - 1));
        if(step >= (numSteps - numStepsOfKeepingVelocityConstraints)){
            os() << ", keep existing velocity constraints";
        }
        os() << ": Calculating coefficients... "; os().flush();

        int globalDimHardConstraints = 0;
        y.resize(globalDimJointSpace);

        for(int frame=0; frame < totalNumFrames; ++frame){

            FrameVariableSet& vars = allFrameVariables[frame];

            // store vertices from the previous output halfway motion
            updateCurrentBodyPositions(step, frame);
            mesh->getVertices(allMarkerPositions);

            if(step == 0){

                vars.B.resize(dimJointSpace, dimJointSpace);
                calcLaplacianCoordinate(frame, vars.L0, true);
                
                if(maxLaplacianDiffToOrgPosition > 0.0){
                    dL.resize(dimMarkerSpace);
                    dL.setZero(dimMarkerSpace);
                }

            } else if(maxLaplacianDiffToOrgPosition > 0.0){
                calcLaplacianCoordinate(frame, L_current, false);
                dL = vars.L0 - L_current;
                // set truncated value towards the original position
                for(int j=0; j < numMarkers; ++j){
                    VectorXd::FixedSegmentReturnType<3>::Type dLj = dL.segment<3>(j * 3);
                    const double n = dLj.norm();
                    if(n > maxLaplacianDiffToOrgPosition){
                        dLj *= maxLaplacianDiffToOrgPosition / n;
                    }
                }
            }

            calcLaplacianCoordinateJacobian(frame);

            const int jointSpaceOffset = dimJointSpace * frame;
        
            if(maxLaplacianDiffToOrgPosition > 0.0){
                y.segment(jointSpaceOffset, dimJointSpace) = J.transpose() * dL;
            } else {
                y.segment(jointSpaceOffset, dimJointSpace).setZero();
            }

            MatrixXd& Bi = vars.B;

            Bi = J.transpose() * J;

            if(jointDampingWeight > 0.0){

                Bi += jointDampingWeight * MatrixXd::Identity(dimJointSpace, dimJointSpace);

                // constants to move back root translations toward the original positions
                if(maxLaplacianDiffToOrgPosition > 0.0){
                    for(size_t i=0; i < rootFreeBodies.size(); ++i){
                        BodyInfo& info = *rootFreeBodies[i];
                        Vector3 dp = orgRootTranslations(frame, i) - info.body->rootLink()->p();
                        const double l = dp.norm();
                        if(l > maxLaplacianDiffToOrgPosition){
                            dp *= maxLaplacianDiffToOrgPosition / l;
                        }
                        // weight different from usual joint one should be specified?
                        //static const double w = 1.0;
                        static const double w = jointDampingWeight;
                        const int index = info.globalJointSpaceOffset;
                        Bi(index, index) = w;
                        Bi(index + 1, index + 1) = w;
                        Bi(index + 2, index + 2) = w;
                        y.segment(jointSpaceOffset + index, 3) += w * dp;
                    }
                }
            }

            vars.globalHardConstraintOffset = globalDimHardConstraints;

            detectJointOverruns(frame, dqRatio, keepVelocityConstraints);
            
            vector<LinkConstraintPtr>& constraints = vars.hardLinkConstraints;
            const int dimPositionConstraints = constraints.size() * 6;
            const int dimConstraints = dimPositionConstraints + jointRangeConstraints.size();

            MatrixXd& Ci = vars.C;
            Ci.resize(dimConstraints, dimJointSpace);
            Ci.setZero(dimConstraints, dimJointSpace);
            VectorXd& hi = vars.h;
            hi.resize(dimConstraints);

            calcLinkPositionConstraintCoefficients(constraints, Ci, hi);

            // set coefficients of the joint range constraints
            for(size_t i=0; i < jointRangeConstraints.size(); ++i){
                const JointConstraint& constraint = jointRangeConstraints[i];
                const int row = i + dimPositionConstraints;
                Ci(row, constraint.globalJointIndex) = 1.0;
                hi(row) = constraint.dq;
            }

            globalDimHardConstraints += dimConstraints;
        }

        if(!solveFinalMatrixForAllFrames(globalDimJointSpace + globalDimHardConstraints)){

            if(step > 0){
                numHalfwayMotions = step;
                return true;
            }
            return false;
        }

        storeSolution(step);
    }
        
    return true;
}


void BodyIMeshSolverImpl::storeSolution(int stepIndex)
{
    os() << "Storing the solution... "; os().flush();
    
    const VectorXd& x = solution;
    
    for(size_t i=0; i < bodies.size(); ++i){

        BodyInfo& info = *bodies[i];

        const auto prevMotion = info.currentStepMotion;
        shared_ptr<BodyMotion> updatedMotion;
        int frameBegin;
        int numFrames;
        int prevLastFrameIndex;
        if(isSinglePoseMode){
            updatedMotion = info.halfwayMotions.back();
            frameBegin = stepIndex + 1;
            numFrames = frameBegin + 1;
            prevLastFrameIndex = stepIndex;
        } else {
            updatedMotion = info.halfwayMotions[std::min(stepIndex, numHalfwayMotions - 1)];
            frameBegin = 0;
            numFrames = totalNumFrames;
            prevLastFrameIndex = prevMotion->numFrames() - 1;
        }
        
        const Body* body = info.body;
        const int numJoints = body->numJoints();
        const bool hasRootDOF = (!body->isFixedRootModel() && !isFixedRootMode);
        auto sseq0 = prevMotion->stateSeq();
        auto sseq1 = updatedMotion->stateSeq();
        
        int offset = info.globalJointSpaceOffset;

        for(int frameIndex = frameBegin; frameIndex < numFrames; ++frameIndex){

            int row = offset;
            const int frameIndex0 = std::min(frameIndex, prevLastFrameIndex);
            const auto& frame0 = sseq0->frame(frameIndex0);
            auto& frame1 = sseq1->allocateFrame(frameIndex);
            
            if(hasRootDOF){
                auto p0 = frame0.linkPosition(0);
                auto p1 = frame1.linkPosition(0);
                p1.translation() = p0.translation() + Eigen::Map<const Vector3>(&x[row]);
                row += 3;
                p1.rotation() = rotFromRpy(x[row], x[row+1], x[row+2]) * p0.rotation();
                row += 3;
            } else {
                frame1.linkPosition(0) = frame0.linkPosition(0);
            }
            
            auto displacement0 = frame0.jointDisplacements();
            auto displacement1 = frame1.jointDisplacements();
            for(int j=0; j < numJoints; ++j){
                const int validJointIndex = info.jointIdToValidJointIndexMap[j];
                if(validJointIndex < 0){
                    displacement1[j] = displacement0[j];
                } else {
                    displacement1[j] = displacement0[j] + x[row + validJointIndex];
                }
            }
            offset += dimJointSpace;
        }

        info.currentStepMotion = updatedMotion;
    }

    os() << "OK!" << endl;
}


bool BodyIMeshSolver::applyFrameLocally()
{
    return impl->applyFrameLocally();
}


bool BodyIMeshSolverImpl::applyFrameLocally()
{
    const int numFramesToShowProgress = 100;
    
    isSinglePoseMode = true;

    numHalfwayMotions = isHalfwayMotionRecordingEnabled ? 1 : 0;

    for(int i=0; i < mesh->numBodies(); ++i){
        if(mesh->bodyInfo(i).motion){
            if(isHalfwayMotionRecordingEnabled){
                numHalfwayMotions = numSteps;
                isSinglePoseMode = false;
            }
            break;
        }
    }

    for(size_t i=0; i < bodies.size(); ++i){
        BodyInfo& info = *bodies[i];
        info.halfwayMotions.resize(numHalfwayMotions);
        for(int j=0; j < numHalfwayMotions; ++j){
            auto motion = make_shared<BodyMotion>();
            motion->setFrameRate(mesh->frameRate());
            int numFrames = isSinglePoseMode ? numSteps : mesh->bodyInfo(i).motion->numFrames();
            motion->setDimension(numFrames, info.body->numJoints(), 1);
            info.halfwayMotions[j] = motion;
        }
    }
    
    mesh->update();

    MatrixXd C;
    MatrixXd Q;  // non-sparse version of Q is locally defined
    VectorXd L_initial;
    VectorXd L_current;
    VectorXd dL;
    VectorXd dq;

    for(int frame=0; frame < totalNumFrames; ++frame){

        FrameVariableSet& vars = allFrameVariables[frame];
        
        mesh->getVertices(frame, allMarkerPositions);

        storeInitialRootTranslations();

        calcLaplacianCoordinate(frame, L_initial, true);

        for(int i=0; i < numSteps; ++i){

            mesh->getVertices(allMarkerPositions);

            if(maxLaplacianDiffToOrgPosition > 0.0){
                //calcLaplacianCoordinate(frame, L_current, true);
                calcLaplacianCoordinate(frame, L_current, false);
                dL = L_initial - L_current;
                
                // set small value towards the original position
                for(int j=0; j < numMarkers; ++j){
                    const double n = dL.segment<3>(j * 3).norm();
                    if(n > maxLaplacianDiffToOrgPosition){
                        dL.segment<3>(j * 3) *= maxLaplacianDiffToOrgPosition / n;
                    }
                }
            }

            calcLaplacianCoordinateJacobian(frame);

            const double dqRatio = 1.0 / std::max(1.0, (double)(numSteps - i - 1));

            detectJointOverruns(frame, dqRatio, false);
            vector<LinkConstraintPtr>& constraints = vars.hardLinkConstraints;
            const int dimPositionConstraints = constraints.size() * 6;
            const int dimConstraints = dimPositionConstraints + jointRangeConstraints.size();

            const int size = dimJointSpace + dimConstraints;
            Q.resize(size, size);
            y.resize(size);
            
            Q.topLeftCorner(dimJointSpace, dimJointSpace) = J.transpose() * J;

            if(maxLaplacianDiffToOrgPosition > 0.0){
                y.head(dimJointSpace) = J.transpose() * dL;
            } else {
                y.head(dimJointSpace).setZero();
            }

            if(jointDampingWeight > 0.0){
                Q.topLeftCorner(dimJointSpace, dimJointSpace) +=
                    jointDampingWeight * MatrixXd::Identity(dimJointSpace, dimJointSpace);

                // constants to move back root translations toward the original positions
                for(size_t i=0; i < rootFreeBodies.size(); ++i){
                    BodyInfo& info = *rootFreeBodies[i];
                    Vector3 dp = orgRootTranslations(frame, i) - info.body->rootLink()->p();
                    const double l = dp.norm();
                    if(l > maxLaplacianDiffToOrgPosition){
                        dp *= maxLaplacianDiffToOrgPosition / l;
                    }
                    y.segment(info.globalJointSpaceOffset, 3) += jointDampingWeight * dp;
                }
            }

            MatrixXd& Ci = vars.C;
            Ci.resize(dimConstraints, dimJointSpace);
            VectorXd& hi = vars.h;
            hi.resize(dimConstraints);
            
            if(dimConstraints > 0){

                Ci.setZero(dimConstraints, dimJointSpace);

                calcLinkPositionConstraintCoefficients(constraints, Ci, hi);

                // set coefficients of the joint range constraints
                for(size_t i=0; i < jointRangeConstraints.size(); ++i){
                    const JointConstraint& constraint = jointRangeConstraints[i];
                    const int row = i + dimPositionConstraints;
                    Ci(row, constraint.globalJointIndex) = 1.0;
                    hi(row) = constraint.dq;
                }

                Q.topRightCorner(Ci.cols(), Ci.rows()) = Ci.transpose();
                Q.bottomLeftCorner(Ci.rows(), Ci.cols()) = Ci;
                y.tail(dimConstraints) = hi;
                Q.bottomRightCorner(dimConstraints, dimConstraints).setZero();
            }
            
            dq = Q.householderQr().solve(y);
            
            if(DEBUG){
                cout << "Jm = \n" << Jm << endl;
                cout << "J = \n" << J << endl;
                cout << "Q = \n" << Q << endl;
                cout << "y = \n" << y << endl;
                cout << "Ci = \n" << Ci << endl;
                cout << "hi = \n" << hi << endl;
            }
            
            int qIndex = 0;
            for(size_t j=0; j < bodies.size(); ++j){
                BodyInfo& info = *bodies[j];
                BodyPtr& body = info.body;
                if(!body->isFixedRootModel() && !isFixedRootMode){
                    Link* rootLink = body->rootLink();
                    rootLink->p() += Eigen::Map<const Vector3>(&dq[qIndex]);
                    qIndex += 3;
                    rootLink->R() = rotFromRpy(dq[qIndex], dq[qIndex+1], dq[qIndex+2]) * rootLink->R();
                    qIndex += 3;
                }
                const vector<JointInfoPtr>& validJoints = info.validJoints;
                for(size_t k=0; k < validJoints.size(); ++k){
                    validJoints[k]->link->q() += dq[qIndex++];
                }
                body->calcForwardKinematics();

                if(isHalfwayMotionRecordingEnabled){
                    if(isSinglePoseMode){
                        info.halfwayMotions[0]->frame(i) << *body;
                    } else {
                        auto motion = info.halfwayMotions[i];
                        if(frame < motion->numFrames()){
                            motion->frame(frame) << *body;
                        }
                    }
                }
            }
        }
        if((frame % numFramesToShowProgress) == 0){
            os() << frame << " ... ";
            os().flush();
        }
    }
    os() << endl;
        
    return true;
}


void BodyIMeshSolver::setInterpolationTestPosePair
(BodyPtr body, const BodyState& initialPose, const BodyState& finalPose, std::shared_ptr<BodyMotion> output)
{
    impl->setInterpolationTestPosePair(body, initialPose, finalPose, output);
}


void BodyIMeshSolverImpl::setInterpolationTestPosePair
(BodyPtr body, const BodyState& initialPose, const BodyState& finalPose, std::shared_ptr<BodyMotion> output)
{
    PosePairPtr pair = make_shared<PosePair>();
    pair->body = body;
    pair->initialPose = initialPose;
    pair->finalPose = finalPose;
    pair->motion = output;
    posePairs.push_back(pair);
}


bool BodyIMeshSolver::doInterpolationTest(int numInterpolatedFrames, bool doInterpolation, bool useQuadraticFormulation)
{
    if(!useQuadraticFormulation){
        return impl->doInterpolationTest(numInterpolatedFrames, doInterpolation);
    } else {
        return impl->doInterpolationTest2(numInterpolatedFrames);
    }
}


bool BodyIMeshSolverImpl::doInterpolationTest(int numInterpolatedFrames, bool doInterpolation)
{
    mesh->clear();

    for(size_t i = 0; i < posePairs.size(); ++i){
        PosePair& pair = *posePairs[i];
        mesh->addBody(pair.body);
        pair.finalPose.restoreStateToBody(pair.body);
        pair.motion->setDimension(numInterpolatedFrames, pair.body->numJoints(), 1);
    }

    setInteractionMesh(mesh);

    mesh->update();

    VectorXd L_final;

    mesh->getVertices(0, allMarkerPositions);
    calcLaplacianCoordinate(0, L_final, true);

    for(size_t i = 0; i < posePairs.size(); ++i){
        PosePair& pair = *posePairs[i];
        pair.initialPose.restoreStateToBody(pair.body);
        pair.motion->frame(0) << *pair.body;
    }

    VectorXd L_initial;

    if(doInterpolation){
        mesh->getVertices(0, allMarkerPositions);
        calcLaplacianCoordinate(0, L_initial, false);
    }

    VectorXd L_current;
    VectorXd dL;
    VectorXd dq;
    MatrixXd JJ;
    Eigen::ColPivHouseholderQR<MatrixXd> QR;
    
    for(int i=1; i < numInterpolatedFrames; ++i){

        mesh->getVertices(0, allMarkerPositions);
        calcLaplacianCoordinate(0, L_current, false);

        if(doInterpolation){
            double r = (double)i / (double)(numInterpolatedFrames - 1);
            VectorXd L_next = L_final * r + L_initial * (1.0 - r);
            dL = L_next - L_current;
        } else {
            dL = L_final - L_current;
        }
        calcLaplacianCoordinateJacobian(0);

        // Calc a solution with the damped least squares method
        const double dampingConstantSqr = 1.0e-6 * 1.0e-6;

        JJ = J * J.transpose() + dampingConstantSqr * MatrixXd::Identity(J.rows(), J.rows());
        dq = J.transpose() * QR.compute(JJ).solve(dL);

        if(DEBUG){
            cout << "Jm = \n" << Jm << endl;
            cout << "J = \n" << J << endl;
            cout << "I = \n" << MatrixXd::Identity(J.rows(), J.rows()) << endl;
            cout << "JJ = \n" << JJ << endl;
            cout << "dL = \n" << dL << endl;
            cout << "dq = \n" << dq << endl;
        }
        
        int qIndex = 0;
        for(size_t j=0; j < bodies.size(); ++j){
            BodyInfo& info = *bodies[j];
            BodyPtr& body = info.body;
            if(!body->isFixedRootModel() && !isFixedRootMode){
                Link* rootLink = body->rootLink();
                rootLink->p() += Eigen::Map<const Vector3>(&dq[qIndex]);
                qIndex += 3;
                rootLink->R() = rotFromRpy(dq[qIndex], dq[qIndex+1], dq[qIndex+2]) * rootLink->R();
                qIndex += 3;
            }
            const vector<JointInfoPtr>& validJoints = info.validJoints;
            for(size_t k=0; k < validJoints.size(); ++k){
                validJoints[k]->link->q() += dq[qIndex++];
            }
            body->calcForwardKinematics();

            posePairs[j]->motion->frame(i) << *body;
        }
    }
        
    return true;
}


bool BodyIMeshSolverImpl::doInterpolationTest2(int numInterpolatedFrames)
{
    mesh->clear();

    for(size_t i = 0; i < posePairs.size(); ++i){
        PosePair& pair = *posePairs[i];
        mesh->addBody(pair.body);
        pair.finalPose.restoreStateToBody(pair.body);
        pair.motion->setDimension(numInterpolatedFrames, pair.body->numJoints(), 1);
    }

    setInteractionMesh(mesh);

    mesh->update();

    VectorXd L_final;

    mesh->getVertices(0, allMarkerPositions);

    storeInitialRootTranslations();
    
    calcLaplacianCoordinate(0, L_final, true);

    for(size_t i = 0; i < posePairs.size(); ++i){
        PosePair& pair = *posePairs[i];
        pair.initialPose.restoreStateToBody(pair.body);
        pair.motion->frame(0) << *pair.body;
    }

    VectorXd L_initial;

    mesh->getVertices(0, allMarkerPositions);
    calcLaplacianCoordinate(0, L_initial, false);

    MatrixXd Q; // non-sparse version of Q is locally defined
    VectorXd L_current;
    VectorXd dL;
    VectorXd dq;
    MatrixXd JJ;
    Eigen::ColPivHouseholderQR<MatrixXd> QR;
    
    for(int i=1; i < numInterpolatedFrames; ++i){

        mesh->getVertices(0, allMarkerPositions);
        calcLaplacianCoordinate(0, L_current, false);

        double r = (double)i / (double)(numInterpolatedFrames - 1);
        VectorXd L_next = L_final * r + L_initial * (1.0 - r);
        dL = L_next - L_current;

        calcLaplacianCoordinateJacobian(0);

        Q.resize(J.cols(), J.cols());
        y.resize(J.cols());

        Q.topLeftCorner(J.cols(), J.cols()) = J.transpose() * J;

        y.head(J.cols()) = J.transpose() * dL;

        if(jointDampingWeight > 0.0){
            Q.topLeftCorner(dimJointSpace, dimJointSpace) +=
                jointDampingWeight * MatrixXd::Identity(dimJointSpace, dimJointSpace);
        }
            
        dq = Q.householderQr().solve(y);
        
        if(DEBUG){
            cout << "Q = \n" << JJ << endl;
            cout << "dL = \n" << dL << endl;
            cout << "dq = \n" << dq << endl;
        }
        
        int qIndex = 0;
        for(size_t j=0; j < bodies.size(); ++j){
            BodyInfo& info = *bodies[j];
            BodyPtr& body = info.body;
            if(!body->isFixedRootModel() && !isFixedRootMode){
                Link* rootLink = body->rootLink();
                rootLink->p() += Eigen::Map<const Vector3>(&dq[qIndex]);
                qIndex += 3;
                rootLink->R() = rotFromRpy(dq[qIndex], dq[qIndex+1], dq[qIndex+2]) * rootLink->R();
                qIndex += 3;
            }
            const vector<JointInfoPtr>& validJoints = info.validJoints;
            for(size_t k=0; k < validJoints.size(); ++k){
                validJoints[k]->link->q() += dq[qIndex++];
            }
            body->calcForwardKinematics();

            posePairs[j]->motion->frame(i) << *body;
        }
    }
        
    return true;
}


void BodyIMeshSolverImpl::setFrameCoefficientsToLeftHalfBlock
(int frameIndex, VectorXd*& q_prev, vector<VelocityConstraint>*& vclist_prev)
{
    int accTermInfoIndex;

    if(!isAccTermEnabled){
        accTermInfoIndex = 5;
    } else {
        if(frameIndex >= 2){
            if(frameIndex < totalNumFrames - 2){
                accTermInfoIndex = 2;
            } else if(frameIndex == totalNumFrames - 2){
                accTermInfoIndex = 3;
            } else {
                accTermInfoIndex = 4;
            }
        } else if(frameIndex == 1){
            accTermInfoIndex = 1;
        } else {
            accTermInfoIndex = 0;
        }
    }

    const vector<AccTermInfo>& accTermInfos = allAccTermInfos[accTermInfoIndex];
    FrameVariableSet& vars = allFrameVariables[frameIndex];
    const MatrixXd& Ci = vars.C;
    VectorXd& q = vars.jointSpacePositions;
    const int offset = frameIndex * dimJointSpace;
    const int hoffset = globalDimJointSpace + vars.globalHardConstraintOffset;
    //static const double vcWeight = 1.0e-6;
    //static const double vcWeight = 1.0e-4;
    //static const double vcWeight = 1.0e10;
    static const double vcWeight = 1.0;
    const double frate = mesh->frameRate();
    const double vck = vcWeight * frate;
    const double vck2 = vcWeight * frate * frate;

    vector<VelocityConstraint>& vclist0 = vars.velocityConstraints;
    int vcIndex0 = 0;
    int vcIndex_prev = 0;

    const VectorXd& q_next = allFrameVariables[frameIndex + 1].jointSpacePositions;

    for(int i=0; i < dimJointSpace; ++i){
        int col = offset + i;
        S.startVec(col);
        MatrixXd::ColXpr bi = vars.B.col(i);

        double vcCoeff_prev = 0.0;
        double vcCoeff0 = 0.0;
        double& yi = y(frameIndex * dimJointSpace + i);
 
        if((*vclist_prev)[vcIndex_prev].jointIndex == i){
            vcCoeff_prev = vck2;
            yi += -(vck2 * (-(*q_prev)[i] + q[i]))
                + vck * (*vclist_prev)[vcIndex_prev].goalVelocity;
            ++vcIndex_prev;
        }
        if(vclist0[vcIndex0].jointIndex == i){
            vcCoeff0 = vck2;
            yi += -(vck2 * (q[i] - q_next[i])) - vck * vclist0[vcIndex0].goalVelocity;
            ++vcIndex0;
        }
        
        for(int j=0; j < accTermInfos.size(); ++j){
            const AccTermInfo& accTerm = accTermInfos[j];
            const int row = (frameIndex + accTerm.offset) * dimJointSpace + i;
            const double a = accTerm.value * accTermWeights[i];

            if(accTerm.offset == 0){
                bi(i) += a + vcCoeff_prev + vcCoeff0;
                y(row) -= a * q[i];
                // main part of the top left part
                for(int k=0; k < dimJointSpace; ++k){
                    const double bki = bi(k);
                    if(bki != 0.0){
                        S.insertBack(offset + k, col) = bki;
                    }
                }
            } else {
                if(accTerm.offset == -1){
                    S.insertBack(row, col) = a - vcCoeff_prev;
                } else if(accTerm.offset == 1){
                    S.insertBack(row, col) = a - vcCoeff0;
                } else {
                    S.insertBack(row, col) = a;
                }
                y(row) -= a * q[i];
            }
        }
        // hard constraint coefficients
        if(Ci.rows() > 0){
            MatrixXd::ConstColXpr ci = Ci.col(i);
            for(int j=0; j < ci.size(); ++j){
                const double cji = ci(j);
                if(cji != 0.0){
                    S.insertBack(hoffset + j, col) = cji;
                }
            }
        }
    }

    q_prev = &q;
    vclist_prev = &vclist0;
}


bool BodyIMeshSolverImpl::solveFinalMatrixForAllFrames(int globalSize)
{
    os() << "Making the final sparse matrix... "; os().flush();

    //S.setZero(); // needed?
    S.resize(globalSize, globalSize); // cleared?

    // set the left half of S
    allFrameVariables.back().jointSpacePositions = allFrameVariables.front().jointSpacePositions;
    VectorXd* q_prev = &(allFrameVariables[0].jointSpacePositions);
    vector<VelocityConstraint> vclist_empty(1, VelocityConstraint(-1, 0.0));
    vector<VelocityConstraint>* vclist_prev = &vclist_empty;
    for(int frame=0; frame < totalNumFrames; ++frame){
        setFrameCoefficientsToLeftHalfBlock(frame, q_prev, vclist_prev);
    }

    // set the right half of S and the bottom of y
    y.conservativeResize(globalSize);
    int col = globalDimJointSpace;
    for(int frame=0; frame < totalNumFrames; ++frame){
        const FrameVariableSet& vars = allFrameVariables[frame];
        const int rowTop = frame * dimJointSpace;
        const MatrixXd& Ci = vars.C;
        if(Ci.rows() > 0){
            for(int i=0; i < Ci.rows(); ++i){
                MatrixXd::ConstRowXpr ci = Ci.row(i);
                S.startVec(col);
                for(int j=0; j < dimJointSpace; ++j){
                    const double cij = ci[j];
                    if(cij != 0.0){
                        S.insertBack(rowTop + j, col) = cij;
                    }
                }
                ++col;
            }
            const VectorXd& h = vars.h;
            y.segment(globalDimJointSpace + vars.globalHardConstraintOffset, h.size()) = h;
        }
    }

    S.finalize();

    os() << "Solving the final matrix... "; os().flush();

    if(DEBUG){
        cout << "S = \n";
        cout << S << "\n";
        cout << "y = \n";
        cout << y << endl;
    }

#if EIGEN_VERSION_AT_LEAST(3,1,0)
    Eigen::UmfPackLU<CmSparseMatrix> lu_of_S(S);
    if(lu_of_S.info() != Eigen::Success){
        os() << "LU factorizaion failed." << endl;
        return false;
    }
    solution.resize(globalSize);

    solution = lu_of_S.solve(y);
    if(lu_of_S.info() != Eigen::Success){
        os() << "Linear equation cannot be solved." << endl;
        return false;
    }
#else
    Eigen::SparseLU<CmSparseMatrix, Eigen::UmfPack> lu_of_S(S);
    if(!lu_of_S.succeeded()){
        os() << "LU factorizaion failed." << endl;
        return false;
    }
    solution.resize(globalSize);
    
    if(!lu_of_S.solve(y, &solution)){
        os() << "Linear equation cannot be solved." << endl;
        return false;
    }
#endif
    
    os() << "OK! "; os().flush();

    return true;
}
