#ifndef CNOID_MOCAP_IMESH_PLUGIN_MARKER_IMESH_ITEM_H
#define CNOID_MOCAP_IMESH_PLUGIN_MARKER_IMESH_ITEM_H

#include "IMeshItem.h"
#include "MarkerIMesh.h"

namespace cnoid {

class Plugin;
class MarkerMotionItem;
        
class MarkerIMeshItem : public IMeshItem
{
public:
    static void initialize(Plugin* plugin);

    MarkerIMeshItem();

    virtual IMeshBase* interactionMeshBase() override;
        
    MarkerIMeshPtr mesh() { return mesh_; }

    int numMeshedMotionItems() const { return motionItems.size(); }
    MarkerMotionItem* meshedMotionItem(int index) const { return motionItems[index]; }

protected:
    MarkerIMeshItem(const MarkerIMeshItem& org);

    virtual Item* doCloneItem(CloneMap* cloneMap) const override;
        
    virtual void updateMesh() override;
    virtual void getMeshVertices(int frame, SgVertexArray* out_vertices) override;
            
private:
    MarkerIMeshPtr mesh_;
    std::vector<MarkerMotionItem*> motionItems;
    bool doUseCurrentMarkerPositions;

    void onMarkerPositionChangeRequest(int motionIndex, int markerIndex, const Vector3& newPosition);
};

typedef ref_ptr<MarkerIMeshItem> MarkerIMeshItemPtr;

}

#endif
