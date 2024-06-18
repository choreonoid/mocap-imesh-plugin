#include "BodyIMesh.h"
#include <cnoid/NullOut>
#include <cnoid/Link>
#include <cnoid/ValueTree>
#include <fmt/format.h>
#include <set>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;


BodyIMesh::BodyIMesh()
    : BaseSeq("MultiIntVectorSeq"),
      os_(&nullout())
{
    setSeqContentName("InteractionMesh");
    numTotalVertices_ = 0;
    isInitialized = false;
}


BodyIMesh::~BodyIMesh()
{

}


void BodyIMesh::clear()
{
    numTotalVertices_ = 0;
    isInitialized = false;
    bodies.clear();
    localIndices.clear();
    setDimension(0, 0);
}


bool BodyIMesh::addBody(BodyPtr body, std::shared_ptr<BodyMotion> motion)
{
    const Listing& markerListing = *body->info()->findListing("bodyMarkers");
    if(!markerListing.isValid()){
        return false;
    }

    BodyInfoPtr info = make_shared<BodyInfo>();
    info->body = body;
    info->motion = motion;

    for(int i=0; i < markerListing.size(); ++i){
        if(markerListing[i].isString()){
            Link* link = body->link(markerListing[i].toString());
            if(link){
                info->markers.push_back(make_shared<Marker>(link));
            } else {
                os() << format(_("Warning: Link {0} specified in \"bodyMarkers\" is not found."),
                               markerListing[i].toString()) << endl;
            }
        } else if(markerListing[i].isListing()){
            const Listing& markerNode = *markerListing[i].toListing();
            if(markerNode.empty()){
                os() << _("Warning: Empty marker node in \"bodyMarkers\".") << endl;
            }
            Link* link = body->link(markerNode[0].toString());
            if(!link){
                os() << format(_("Warning: Link {0} specified in \"bodyMarkers\" is not found."), link->name()) << endl;
            } else {
                if(markerNode.size() == 1){
                    info->markers.push_back(make_shared<Marker>(link));
                } else {
                    int index = 1;
                    while(index < markerNode.size()){
                        const Listing& vectorNode = *markerNode[index++].toListing();
                        if(vectorNode.size() != 3){
                            os() << format(_("Warning: {0}-th element of Link {1} in \"bodyMarkers\" is not a valid vector."),
                                           index, link->name()) << endl;
                        } else {
                            const Vector3 p(vectorNode[0].toDouble(),
                                            vectorNode[1].toDouble(),
                                            vectorNode[2].toDouble());
                            info->markers.push_back(make_shared<Marker>(link, p));
                        }
                    }
                }
            }
        }
    }

    bodies.push_back(info);

    int numMarkers = info->markers.size();
    
    for(int i=0; i < numMarkers; ++i){
        LocalIndex index;
        index.bodyIndex = bodies.size();
        index.localIndex = i;
        localIndices.push_back(index);
    }

    info->markerIndexOffset = numTotalVertices_;
    numTotalVertices_ += numMarkers;

    extractNeighboursToExclude(*info);

    isInitialized = false;

    return true;
}


void BodyIMesh::extractNeighboursToExclude(BodyInfo& info)
{
    vector< vector<int> > linkToMarkerMap(info.body->numLinks());
    vector<MarkerPtr>& markers = info.markers;

    for(int i=0; i < markers.size(); ++i){
        linkToMarkerMap[markers[i]->link->index()].push_back(i);
    }

    // extract same link markers
    for(int i=0; i < markers.size(); ++i){
        Marker& marker = *markers[i];
        Link* link = marker.link;
        vector<int>& sameLinkMarkers = linkToMarkerMap[link->index()];
        for(int j=0; j < sameLinkMarkers.size(); ++j){
            marker.neighborsToExclude.insert(sameLinkMarkers[j]);
        }
    }

    extractNeighboursToExcludeSub(info.body->rootLink(), markers, linkToMarkerMap);
}


void BodyIMesh::extractNeighboursToExcludeSub
(Link* link, std::vector<MarkerPtr>& markers, std::vector< std::vector<int> >& linkToMarkerMap)
{
    vector<int> fixedRelationshipLinks;
    fixedRelationshipLinks.push_back(link->index());
    extractFixedRelationshipLinks(link, false, false, fixedRelationshipLinks);

    const int n = fixedRelationshipLinks.size();
    for(int i=0; i < n - 1; ++i){
        int linkIndex1 = fixedRelationshipLinks[i];
        bool hasRotation1;
        if(linkIndex1 < 0){
            linkIndex1 = -linkIndex1;
            hasRotation1 = true;
        } else {
            hasRotation1 = false;
        }
        vector<int>& link1Markers = linkToMarkerMap[linkIndex1];
        for(int j=i+1; j < n; ++j){
            int linkIndex2 = fixedRelationshipLinks[j];
            bool hasRotation2;
            if(linkIndex2 < 0){
                linkIndex2 = -linkIndex2;
                hasRotation2 = true;
            } else {
                hasRotation2 = false;
            }
            vector<int>& link2Markers = linkToMarkerMap[linkIndex2];
            for(size_t k=0; k < link1Markers.size(); ++k){
                int markerIndex1 = link1Markers[k];
                if(!hasRotation1 || !markers[markerIndex1]->localPos){
                    for(size_t l=0; l < link2Markers.size(); ++l){
                        int markerIndex2 = link2Markers[l];
                        if(!hasRotation2 || !markers[markerIndex2]->localPos){
                            markers[markerIndex1]->neighborsToExclude.insert(markerIndex2);
                            markers[markerIndex2]->neighborsToExclude.insert(markerIndex1);
                        }
                    }
                }
            }
        }
    }

    for(Link* childLink = link->child(); childLink; childLink = childLink->sibling()){
        extractNeighboursToExcludeSub(childLink, markers, linkToMarkerMap);
    }
}


void BodyIMesh::extractFixedRelationshipLinks(Link* link, bool hasRotation, bool hasOffset, std::vector<int>& fixedRelationshipLinks)
{
    for(Link* childLink = link->child(); childLink; childLink = childLink->sibling()){

        if(childLink->isPrismaticJoint()){
            continue;
        }

        bool childHasOffset = (childLink->b() != Vector3::Zero());

        if(hasOffset && hasRotation && childHasOffset){
            continue;
        }
        
        bool hasRotationNext = hasRotation || childLink->isRevoluteJoint();

        if(hasRotationNext){
            // use negative value to inform that rotational joints are included
            fixedRelationshipLinks.push_back(-childLink->index());
        } else {
            fixedRelationshipLinks.push_back(childLink->index());
        }
        extractFixedRelationshipLinks(childLink, hasRotationNext, (hasOffset || childHasOffset), fixedRelationshipLinks);
    }
}


bool BodyIMesh::initialize()
{
    if(bodies.empty()){
        os() << _("No target body motions.") << endl;
        return false;
    }

    allMarkers.clear();

    BodyInfoPtr info = bodies.front();
    int nFrames = 1;
    double frameRate = 0.0;

    for(size_t i=0; i < bodies.size(); ++i){

        info = bodies[i];

        int markerIndex = allMarkers.size();
        allMarkers.resize(markerIndex + info->markers.size());
        std::copy(info->markers.begin(), info->markers.end(), allMarkers.begin() + markerIndex);

        if(info->motion){
            auto sseq = info->motion->stateSeq();
            
            // check frame rate
            double r = sseq->frameRate();
            if(frameRate == 0.0){
                frameRate = r;
            } else if(fabs(r - frameRate) > 1.0e-6){
                os() << format(_("Frame rate {0:.6f} of {1}-th motion is different from {2:.6f} of the first one."),
                               r, i + 1, frameRate) << endl;
                return false;
            }

            // check the number of frames
            int n = sseq->numFrames();
            if(n == 0){
                os() << format(_("Motion {0} is empty."), i) << endl;
                return false;
            }
            nFrames = std::max(nFrames, n);
        }
    }

    setDimension(0, 0);
    setDimension(nFrames, numTotalVertices_);

    if(frameRate == 0.0){
        // just one of the possible values when there is no motion
        frameRate = 100.0;
    }
    setFrameRate(frameRate);

    tetgenInput.numberofpoints = numTotalVertices_;

    //! \todo delete this
    tetgenInput.pointlist = new REAL[numTotalVertices_ * 3];

    isInitialized = true;

    return true;
}


bool BodyIMesh::update()
{
    if(!isInitialized){
        if(!initialize()){
            return false;
        }
    }
    
    bool result = true;
    const int n = numFrames();

    for(int i=0; i < n; ++i){
        if(!updateFrame(i)){
            result = false;
            os() << _("Tetrahedralize failed.") << endl;
            break;
        }
    }

    return result;
}


namespace {
    struct Edge
    {
        int index0;
        int index1;

        bool operator<(const Edge& rhs) const {
            return (index0 < rhs.index0)
                || (!(rhs.index0 < index0) && index1 < rhs.index1);
        }
        bool operator==(const Edge& rhs) const {
            return (index0 == rhs.index0 && index1 == rhs.index1);
        }
    };
}


void BodyIMesh::getVertices(int frameIndex, std::vector<Vector3>& out_vertices) const
{
    out_vertices.resize(numTotalVertices_);
    int vertexIndex = 0;
    
    for(size_t i=0; i < bodies.size(); ++i){

        BodyInfo& info = *bodies[i];
        
        if(frameIndex >= 0){
            if(info.motion){
                auto sseq = info.motion->stateSeq();
                int actualFrameIndex = std::min(frameIndex, sseq->numFrames() - 1);
                auto& frame = sseq->frame(actualFrameIndex);
                Body* body = info.body;
                int nj = std::min(body->numJoints(), frame.numJointDisplacements());
                auto displacements = frame.jointDisplacements();
                for(int j=0; j < nj; ++j){
                    body->joint(j)->q() = displacements[j];
                }
                if(frame.numLinkPositions() > 0){
                    body->rootLink()->setPosition(frame.linkPosition(0).T());
                }
                body->calcForwardKinematics();
            }
        }

        const std::vector<MarkerPtr>& markers = info.markers;

        for(size_t j=0; j < markers.size(); ++j){
            const MarkerPtr& marker = markers[j];
            if(marker->localPos){
                out_vertices[vertexIndex] = marker->link->p() + marker->link->R() * (*marker->localPos);
            } else {
                out_vertices[vertexIndex] = marker->link->p();
            }
            ++vertexIndex;
        }
    }
}


void BodyIMesh::getJointPositions(int bodyIndex, int frameIndex, std::vector<double>& out_q) const
{
    BodyInfo& info = *bodies[bodyIndex];
        
    if(frameIndex >= 0 && info.motion){
        auto sseq = info.motion->stateSeq();
        const int actualFrameIndex = std::min(frameIndex, sseq->numFrames() - 1);
        auto& frame = sseq->frame(actualFrameIndex);
        int nj = frame.numJointDisplacements();
        out_q.resize(nj);
        auto displacements = frame.jointDisplacements();
        for(int i=0; i < nj; ++i){
            out_q[i] = displacements[i];
        }
    } else {
        BodyPtr& body = info.body;
        const int n = body->numJoints();
        for(int i=0; i < n; ++i){
            out_q[i] = body->joint(i)->q();
        }
    }
 }

        
bool BodyIMesh::updateFrame(int frameIndex)
{
    getVertices(frameIndex, tmpVertices);

    int pointIndex = 0;
    for(size_t i=0; i < numTotalVertices_; ++i){
        const Vector3& v = tmpVertices[i];
        tetgenInput.pointlist[pointIndex++] = v.x();
        tetgenInput.pointlist[pointIndex++] = v.y();
        tetgenInput.pointlist[pointIndex++] = v.z();
    }
    
    char switches[] = "zQ";
    tetrahedralize(switches, &tetgenInput, &tetgenOutput);

    set<Edge> edges;
    Edge edge;
    int& index0 = edge.index0;
    int& index1 = edge.index1;

    const int n = tetgenOutput.numberoftetrahedra;
    const int m = tetgenOutput.numberofcorners;
    for(int i=0; i < n; ++i){
        for(int j=0; j < m-1; ++j){
            for(int k=j+1; k < m; ++k){
                index0 = tetgenOutput.tetrahedronlist[i*m + j];
                index1 = tetgenOutput.tetrahedronlist[i*m + k];

                // exclude constant length edges
                set<int>& exclusions0 = allMarkers[index0]->neighborsToExclude;
                if(exclusions0.find(index1) != exclusions0.end()){
                    continue;
                }
                
                if(index0 > index1){
                    std::swap(index0, index1);
                }
                edges.insert(edge); // redundant edges are not inserted
            }
        }
    }

    Frame neighbors = frame(frameIndex);

    for(set<Edge>::iterator p = edges.begin(); p != edges.end(); ++p){
        neighbors[p->index0].push_back(p->index1);
        neighbors[p->index1].push_back(p->index0);
    }

    return true;
}


void BodyIMesh::setMessageOutput(std::ostream& os) const
{
    os_ = &os;
}
