#ifndef CNOID_MOCAP_IMESH_PLUGIN_IMESH_ITEM_H
#define CNOID_MOCAP_IMESH_PLUGIN_IMESH_ITEM_H

#include <cnoid/Item>
#include <cnoid/MultiSeq>
#include <cnoid/EigenTypes>
#include <cnoid/ConnectionSet>
#include <cnoid/SceneProvider>
#include <cnoid/SceneDrawables>

namespace cnoid {

#ifdef _WIN32
template class __declspec(dllimport) MultiSeq< std::vector<int> >;
#endif

class SceneIMesh;

class IMeshItem : public Item, public SceneProvider
{
public:

    typedef MultiSeq< std::vector<int> > IMeshBase;
        
    IMeshItem();
    ~IMeshItem();
        
    virtual IMeshBase* interactionMeshBase() = 0;
    virtual void getMeshVertices(int frame, SgVertexArray* out_vertices) = 0;

    bool isRenderingEnabled() const { return isRenderingEnabled_; }
            
protected:
        
    IMeshItem(const IMeshItem& org);

    virtual void onConnectedToRoot() override;
    virtual void onDisconnectedFromRoot() override;
        
    void addConnectionInCheckState(const Connection& connection) {
        connectionsInCheckState.add(connection);
    }

    void notifyMeshVertiesChanged();

    virtual void updateMesh() = 0;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual SgNode* getScene() override;

private:
    ref_ptr<SceneIMesh> scene_;
    ScopedConnectionSet connectionsInCheckState;
    ConnectionSet glConnections;
    bool isRenderingEnabled_;

    void initializeInstance();
    void onItemCheckToggled();
    void onSceneConnection(bool on);        
};

typedef ref_ptr<IMeshItem> IMeshItemPtr;

}

#endif
