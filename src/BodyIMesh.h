#ifndef CNOID_MOCAP_IMESH_PLUGIN_BODY_IMESH_H
#define CNOID_MOCAP_IMESH_PLUGIN_BODY_IMESH_H

#include <cnoid/MultiSeq>
#include <cnoid/Body>
#include <cnoid/BodyMotion>
#include <cnoid/ValueTree>
#include <cnoid/stdx/optional>
#include <tetgen.h>
#include <set>
#include <memory>

namespace cnoid {

#ifdef _WIN32
template class __declspec(dllimport) MultiSeq< std::vector<int> >;
#endif

class BodyIMesh : public MultiSeq< std::vector<int> >
{
public:
    typedef std::vector<int> NeighborList;
    typedef MultiSeq<std::vector<int> > BaseSeq;
    typedef std::shared_ptr<BodyIMesh> Ptr;

    class Marker
    {
    public:
        Marker(Link* link) : link(link) { }
        Marker(Link* link, const Vector3& localPos) : link(link), localPos(localPos) { }
        Link* link;
        // This should be represented on link->R coordinate (now link->attitude() coordinate)
        stdx::optional<Vector3> localPos;
        std::set<int> neighborsToExclude;
    };
    typedef std::shared_ptr<Marker> MarkerPtr;

    class BodyInfo
    {
    public:
        BodyPtr body;
        std::shared_ptr<BodyMotion> motion;
        std::vector<MarkerPtr> markers;
        int markerIndexOffset;
    };
    typedef std::shared_ptr<BodyInfo> BodyInfoPtr;

    BodyIMesh();
    ~BodyIMesh();

    void clear();

    bool addBody(BodyPtr body, std::shared_ptr<BodyMotion> motion = nullptr);

    int numBodies() const { return bodies.size(); }
    BodyInfo& bodyInfo(int bodyIndex) { return *bodies[bodyIndex]; }

    int vertexIndexOffset(int bodyIndex) const { return bodies[bodyIndex]->markerIndexOffset; }

    int numVertices() const { return numTotalVertices_; }
    int numMarkers() const { return numTotalVertices_; }

    Marker& marker(int markerIndex) { return *allMarkers[markerIndex]; }

    void getVertices(int frameIndex, std::vector<Vector3>& out_vertices) const;
    void getVertices(std::vector<Vector3>& out_vertices) const { getVertices(-1, out_vertices); }

    void getJointPositions(int bodyIndex, int frameIndex, std::vector<double>& out_q) const;
    void getJointPositions(int bodyIndex, std::vector<double>& out_q) const {
        getJointPositions(bodyIndex, -1, out_q);
    }

    struct LocalIndex
    {
        int bodyIndex;
        int localIndex;
    };
    const LocalIndex& localIndex(int globalIndex) const { return localIndices[globalIndex]; }

    bool initialize();
    bool update();

    void setMessageOutput(std::ostream& os) const;

protected:
    std::ostream& os() const { return *os_; }

private:
    int numTotalVertices_;
    std::vector<BodyInfoPtr> bodies;
    std::vector<MarkerPtr> allMarkers;
    std::vector<LocalIndex> localIndices;

    bool isInitialized;

    tetgenio tetgenInput;
    tetgenio tetgenOutput;

    mutable std::vector<Vector3> tmpVertices;
    mutable std::ostream* os_;

    void extractNeighboursToExclude(BodyInfo& info);
    void extractNeighboursToExcludeSub(
        Link* link, std::vector<MarkerPtr>& markers, std::vector< std::vector<int> >& linkToMarkerMap);
    void extractFixedRelationshipLinks(
        Link* link, bool hasRotation, bool hasOffset, std::vector<int>& fixedRelationshipLinks);
    bool updateFrame(int frameIndex);
};

typedef BodyIMesh::Ptr BodyIMeshPtr;

}

#endif
