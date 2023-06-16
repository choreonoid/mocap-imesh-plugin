#include "IMeshItem.h"
#include <cnoid/RootItem>
#include <cnoid/MessageView>
#include <cnoid/TimeBar>
#include <cnoid/ItemManager>
#include <cnoid/MenuManager>
#include <cnoid/Archive>
#include <cnoid/EigenUtil>
#include <cnoid/EigenArchive>
#include <cnoid/SceneRenderer>
#include <cnoid/PutPropertyFunction>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace cnoid {

class SceneIMesh : public SgLineSet
{
public:
    IMeshItem* meshItem;
    int frame;
    BoundingBox bbox;
    bool isDirty;
        
    SceneIMesh(IMeshItem* meshItem)
        : SgLineSet(findPolymorphicId<SceneIMesh>()),
          meshItem(meshItem)
    {
        frame = -1;
        isDirty = true;
        getOrCreateVertices();
        getOrCreateMaterial()->setDiffuseColor(Vector3f(0.0f, 1.0f, 0.0f));
        setLineWidth(1.0f);
    }

    bool setTime(double time)
    {
        IMeshItem::IMeshBase* mesh = meshItem->interactionMeshBase();
        int numFrames = mesh->numFrames();
        int newFrame = mesh->frameOfTime(time);
        bool isValid = (newFrame < numFrames);
        if(!isValid){
            newFrame = numFrames - 1;
        }
        if(newFrame != frame){
            frame = newFrame;
            isDirty = true;
            notifyUpdate();
        }
        return isValid;
    }

    void render(SceneRenderer::NodeFunctionSet* renderingFunctions)
    {
        if(frame < 0 || !meshItem->isRenderingEnabled()){
            return;
        }

        if(isDirty){
            meshItem->getMeshVertices(frame, vertices());
            clearLines();
            const IMeshItem::IMeshBase::Frame f = meshItem->interactionMeshBase()->frame(frame);
            for(int i=0; i < f.size(); ++i){
                const IMeshItem::IMeshBase::Element& neighbours = f[i];
                for(int j=0; j < neighbours.size(); ++j){
                    addLine(i, neighbours[j]);
                }
            }
            vertices()->notifyUpdate();
            isDirty = false;
        }

        renderingFunctions->dispatchAs<SgLineSet>(this);
    }

    virtual const BoundingBox& boundingBox() const override {
        return bbox;
    }
};

}

namespace {

struct NodeTypeRegistration {
    NodeTypeRegistration() {
        SgNode::registerType<SceneIMesh, SgLineSet>();
        SceneRenderer::addExtension(
            [](SceneRenderer* renderer){
                auto functions = renderer->renderingFunctions();
                functions->setFunction<SceneIMesh>(
                    [functions](SgNode* node){ static_cast<SceneIMesh*>(node)->render(functions); });
            });
    }
} registration;

}


IMeshItem::IMeshItem()
{
    isRenderingEnabled_ = true;
    initializeInstance();
}


IMeshItem::IMeshItem(const IMeshItem& org)
    : Item(org)
{
    isRenderingEnabled_ = org.isRenderingEnabled_;
    initializeInstance();
}


void IMeshItem::initializeInstance()
{
    sigCheckToggled().connect([&](bool){ onItemCheckToggled(); });

    scene_ = new SceneIMesh(this);
    scene_->sigGraphConnection().connect([&](bool on){ onSceneConnection(on); });
}


IMeshItem::~IMeshItem()
{

}


void IMeshItem::onConnectedToRoot()
{
    onItemCheckToggled();
}


void IMeshItem::notifyMeshVertiesChanged()
{
    scene_->isDirty = true;
    scene_->notifyUpdate();
}


void IMeshItem::onDisconnectedFromRoot()
{
    connectionsInCheckState.disconnect();
}


void IMeshItem::onItemCheckToggled()
{
    connectionsInCheckState.disconnect();

    if(isChecked() && isConnectedToRoot()){
        updateMesh();
    }
}


void IMeshItem::onSceneConnection(bool on)
{
    glConnections.disconnect();

    if(on){
        updateMesh();

        glConnections.add(
            TimeBar::instance()->sigTimeChanged().connect(
                [&](double time){ return scene_->setTime(time); }));

        scene_->setTime(TimeBar::instance()->time());
    }
}


SgNode* IMeshItem::getScene()
{
    return scene_;
}


void IMeshItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Rendering"), isRenderingEnabled_, changeProperty(isRenderingEnabled_));

    IMeshItem::IMeshBase* mesh = interactionMeshBase();
    putProperty(_("Frame rate"), mesh->frameRate());
    putProperty(_("Number of frames"), mesh->numFrames());
    putProperty(_("Time length"), mesh->getTimeLength());
    putProperty(_("Time step"), mesh->getTimeStep());

    putProperty(_("Color"), str(scene_->material()->diffuseColor()),
                [&](const std::string& value){
                    Vector3f color;
                    if(toVector3(value, color)){
                        scene_->material()->setDiffuseColor(color);
                        return true;
                    }
                    return false;
                });

    putProperty(_("Line width"), scene_->lineWidth(),
                [&](double width){ scene_->setLineWidth(width); return true; });
}


bool IMeshItem::store(Archive& archive)
{
    archive.write("visible", isRenderingEnabled_);
    write(archive, "color", scene_->material()->diffuseColor());
    archive.write("line_width", scene_->lineWidth());
    return true;
}


bool IMeshItem::restore(const Archive& archive)
{
    archive.read("visible", isRenderingEnabled_);
    Vector3f color;
    if(read(archive, "color", color)){
        scene_->material()->setDiffuseColor(color);
    }
    double width;
    if(archive.read("line_width", width)){
        scene_->setLineWidth(width);
    }
    return true;
}
