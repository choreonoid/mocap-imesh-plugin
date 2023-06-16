#ifndef CNOID_MOCAP_IMESH_PLUGIN_BODY_IMESH_ITEM_H
#define CNOID_MOCAP_IMESH_PLUGIN_BODY_IMESH_ITEM_H

#include "IMeshItem.h"
#include "BodyIMesh.h"
#include <cnoid/BodyItem>
#include <cnoid/BodyMotionItem>
#include <cnoid/ItemList>

namespace cnoid {

class BodyIMeshItem : public IMeshItem
{
public:
    static void initialize(ExtensionManager* ext);

    BodyIMeshItem();

    virtual IMeshBase* interactionMeshBase() override;

    BodyIMeshPtr mesh() { return mesh_; }

    void updateMesh(bool pickupMotions);

    int numBodyItems() const { return bodyItems_.size(); }
    BodyItem* bodyItem(int index) { return bodyItems_.get(index); }

protected:
    BodyIMeshItem(const BodyIMeshItem& org);

    virtual void onDisconnectedFromRoot() override;
    virtual Item* doCloneItem(CloneMap* cloneMap) const override;
    virtual void updateMesh() override;
    virtual void getMeshVertices(int frame, SgVertexArray* out_vertices) override;

private:
    BodyIMeshPtr mesh_;
    ItemList<BodyItem> bodyItems_;
};

typedef ref_ptr<BodyIMeshItem> BodyIMeshItemPtr;

}

#endif
