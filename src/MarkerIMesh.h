#ifndef CNOID_MOCAP_IMESH_PLUGIN_MARKER_IMESH_H
#define CNOID_MOCAP_IMESH_PLUGIN_MARKER_IMESH_H

#include <cnoid/MarkerMotion>
#include <cnoid/MocapMapping>
#include <cnoid/MultiSeq>
#include <tetgen.h>

class tetgenio;

namespace cnoid {

/**
   Interaction mesh for body marker position representation
*/
class MarkerIMesh : public MultiSeq< std::vector<int> >
{
public:
    typedef std::vector<int> NeighborList;
    typedef MultiSeq<std::vector<int> > BaseSeq;
    typedef std::shared_ptr<MarkerIMesh> Ptr;

    MarkerIMesh();
    MarkerIMesh(const MarkerIMesh& org);
    ~MarkerIMesh();

    void clear();
    void addMotion(MarkerMotionPtr motion, MocapMappingPtr mocapMapping = 0);

    int numMotions() const { return motions.size(); }
    MarkerMotionPtr motion(int motionIndex) const { return motions[motionIndex].motion; }

    int localToActiveIndex(int motionIndex, int markerIndex) const {
        return motions[motionIndex].activeVertexIndexMap[markerIndex];
    }

    const std::vector<int>& localToActiveIndexMap(int motionIndex) const {
        return motions[motionIndex].activeVertexIndexMap;
    }

    int numLocalActiveVertices(int motionIndex) const {
        return motions[motionIndex].numLocalActiveVertices;
    }

    int globalVertexIndexOffset(int motionIndex) const {
        return motions[motionIndex].globalVertexIndexOffset;
    }

    int numActiveVertices() const { return numActiveVertices_; }

    int numAllVertices() const { return numAllVertices_; }

    struct LocalIndex
    {
        int motionIndex;
        int markerIndex;
    };

    const LocalIndex& globalToLocalIndex(int globalIndex) const { return globalToLocalIndexMap[globalIndex]; }
    const LocalIndex& activeToLocalIndex(int activeIndex) const { return activeToLocalIndexMap[activeIndex]; }

    const int globalToActiveIndex(int globalIndex) const { return globalToActiveIndexMap[globalIndex]; }

    bool update();

    const std::string& message() { return message_; }

private:
    struct MotionInfo
    {
        MarkerMotionPtr motion;
        std::vector<int> activeVertexIndexMap; // elements are global indices
        int activeVertexIndexOffset;
        int globalVertexIndexOffset;
        int numLocalActiveVertices;
    };

    std::vector<MotionInfo> motions;
    int numAllVertices_;
    int numActiveVertices_;
    std::vector<LocalIndex> globalToLocalIndexMap;
    std::vector<LocalIndex> activeToLocalIndexMap;
    std::vector<int> globalToActiveIndexMap;

    std::string message_;

    bool updateFrame(int frameIndex, tetgenio& in, tetgenio& out);
};

typedef MarkerIMesh::Ptr MarkerIMeshPtr;

}

#endif
