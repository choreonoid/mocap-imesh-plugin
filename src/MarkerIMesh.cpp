#include "MarkerIMesh.h"
#include <tetgen.h>
#include <fmt/format.h>
#include <set>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;


MarkerIMesh::MarkerIMesh()
    : BaseSeq("MultiIntVectorSeq")
{
    setSeqContentName("InteractionMesh");
    numAllVertices_ = 0;
    numActiveVertices_ = 0;
}


MarkerIMesh::MarkerIMesh(const MarkerIMesh& org)
    : BaseSeq(org),
      numAllVertices_(org.numAllVertices_),
      numActiveVertices_(org.numActiveVertices_),
      motions(org.motions),
      globalToLocalIndexMap(org.globalToLocalIndexMap),
      activeToLocalIndexMap(org.activeToLocalIndexMap),
      globalToActiveIndexMap(org.globalToActiveIndexMap)
{

}


MarkerIMesh::~MarkerIMesh()
{

}


void MarkerIMesh::clear()
{
    numAllVertices_ = 0;
    numActiveVertices_ = 0;
    motions.clear();
    globalToLocalIndexMap.clear();
    activeToLocalIndexMap.clear();
    globalToActiveIndexMap.clear();
    setDimension(0, 0);
}


void MarkerIMesh::addMotion(MarkerMotionPtr motion, MocapMappingPtr mocapMapping)
{
    const int motionIndex = motions.size();

    motions.push_back(MotionInfo());
    MotionInfo& info = motions.back();
    info.motion = motion;
    info.activeVertexIndexOffset = numActiveVertices_;
    info.globalVertexIndexOffset = numAllVertices_;

    int numLocalActiveVertices = 0;

    const int numMarkers = motion->numMarkers();
    info.activeVertexIndexMap.resize(numMarkers);
    
    for(int i=0; i < motion->numMarkers(); ++i){
        LocalIndex index;
        index.motionIndex = motionIndex;
        index.markerIndex = i;
        globalToLocalIndexMap.push_back(index);
        
        if(mocapMapping && mocapMapping->isMarkerStatic(motion->markerLabel(i))){
            globalToActiveIndexMap.push_back(-1);
            info.activeVertexIndexMap[i] = -1;
        } else {
            globalToActiveIndexMap.push_back(numActiveVertices_);
            info.activeVertexIndexMap[i] = numActiveVertices_;
            activeToLocalIndexMap.push_back(index);
            ++numActiveVertices_;
            ++numLocalActiveVertices;
        }
        ++numAllVertices_;
    }

    info.numLocalActiveVertices = numLocalActiveVertices;
}


bool MarkerIMesh::update()
{
    message_.clear();
    
    if(motions.empty()){
        message_ = _("No target marker motions.");
        return false;
    }

    MarkerMotionPtr firstMotion = motions.front().motion;
    int maxFrame = firstMotion->numFrames() + firstMotion->offsetTimeFrame();
    double frameRate = firstMotion->frameRate();

    for(size_t i=0; i < motions.size(); ++i){
        MarkerMotionPtr motion = motions[i].motion;
        double r = motion->frameRate();
        if(fabs(r - frameRate) > 1.0e-6){
            message_ = format(_("Frame rate {0:.6f} of {1}-th motion is different from {2:.6f} of the first one."),
                              r, i + 1, frameRate);
            return false;
        }
        int n = motion->numFrames() + motion->offsetTimeFrame();
        if(n == 0){
            message_ = format(_("Motion {0} is empty."), i);;
            return false;
        }
        maxFrame = std::max(maxFrame, n);
    }

    setDimension(0, 0);
    setDimension(maxFrame, numActiveVertices_);
    setFrameRate(frameRate);

    tetgenio in;
    tetgenio out;

    in.numberofpoints = numAllVertices_;
    in.pointlist = new REAL[numAllVertices_ * 3];

    bool result = true;

    for(int i=0; i < maxFrame; ++i){
        if(!updateFrame(i, in, out)){
            result = false;
            message_ = _("Tetrahedralize failed.");
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
    

bool MarkerIMesh::updateFrame(int frameIndex, tetgenio& in, tetgenio& out)
{
    int pointIndex = 0;
    
    for(int i=0; i < motions.size(); ++i){
        MarkerMotionPtr& motion = motions[i].motion;
        int localFrameIndex = motion->clampFrameIndex(frameIndex - motion->offsetTimeFrame());
        MarkerMotion::Frame f = motion->frame(localFrameIndex);
        const Vector3& offset = motion->positionOffset();
        int n = motion->numMarkers();
        for(int j=0; j < n; ++j){
            const Vector3 p(f[j] + offset);
            for(int k=0; k < 3; ++k){
                in.pointlist[pointIndex * 3 + k] = p(k);
            }
            pointIndex++;
        }
    }

    char switches[] = "zQ";
    tetrahedralize(switches, &in, &out);

    set<Edge> edges;
    Edge edge;
    int& index0 = edge.index0;
    int& index1 = edge.index1;

    const int n = out.numberoftetrahedra;
    const int m = out.numberofcorners;
    for(int i=0; i < n; ++i){
        for(int j=0; j < m-1; ++j){
            for(int k=j+1; k < m; ++k){
                index0 = out.tetrahedronlist[i*m + j];
                index1 = out.tetrahedronlist[i*m + k];
                if(index0 > index1){
                    std::swap(index0, index1);
                }
                edges.insert(edge); // redundant edges are not inserted
            }
        }
    }

    Frame neighbors = frame(frameIndex);

    // store the edges of the active vertices as a frame data
    for(set<Edge>::iterator p = edges.begin(); p != edges.end(); ++p){
        const int activeIndex0 = globalToActiveIndexMap[p->index0];
        if(activeIndex0 >= 0){
            neighbors[activeIndex0].push_back(p->index1);
        }
        const int activeIndex1 = globalToActiveIndexMap[p->index1];
        if(activeIndex1 >= 0){
            neighbors[activeIndex1].push_back(p->index0);
        }
    }

    return true;
}
