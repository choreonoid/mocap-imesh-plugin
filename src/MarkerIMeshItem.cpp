#include "MarkerIMeshItem.h"
#include "MarkerIMeshSolver.h"
#include <cnoid/MarkerMotionItem>
#include <cnoid/MocapMappingItem>
#include <cnoid/Plugin>
#include <cnoid/RootItem>
#include <cnoid/FolderItem>
#include <cnoid/TimeBar>
#include <cnoid/MessageView>
#include <cnoid/ItemManager>
#include <cnoid/MenuManager>
#include <cnoid/MainMenu>
#include <cnoid/ItemTreeView>
#include <cnoid/Archive>
#include <cnoid/Dialog>
#include <cnoid/Button>
#include <cnoid/SpinBox>
#include <cnoid/LineEdit>
#include <QGridLayout>
#include <QDialogButtonBox>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {

Plugin* plugin = nullptr;

class RetargetingDialog : public Dialog
{
public:
    ostream& os;

    SpinBox stepNumberSpin;
    CheckBox updateLaplacianConstantCheck;
    CheckBox excludeBonesFromLapacianCheck;
    DoubleSpinBox laplacianWeightPowSpin;
    DoubleSpinBox accelConstraintWeightSpin;
    CheckBox followingOrgFootPositionCheck;
    DoubleSpinBox collarWeightSpin;
    
    MarkerIMeshSolver solver;
    
    FolderItemPtr outputTopItem;
    
    struct Output {
        MarkerMotionItemPtr motionItem;
        MocapMappingItemPtr orgMocapMappingItem;
    };
    vector<Output> outputs;
    
    static RetargetingDialog* getOrCreateInstance();
    static RetargetingDialog* instance();
    RetargetingDialog();
    virtual void onAccepted() override;
    void setRetargetingMotionItem(int index, MarkerMotionItem* motionItem);
    bool store(Archive& archive);
    void restore(const Archive& archive);
};

RetargetingDialog* retargetingDialog = nullptr;

}


void MarkerIMeshItem::initialize(Plugin* plugin)
{
    if(!::plugin){
        ::plugin = plugin;
        
        plugin->itemManager()
            .registerClass<MarkerIMeshItem>(N_("MarkerInteractionMeshItem"))
            .addAlias<MarkerIMeshItem>("MarkerInteractionMeshItem", "Mocap")
            .addAlias<MarkerIMeshItem>("MarkerIMeshItem", "Mocap")
            .addCreationPanel<MarkerIMeshItem>();

        MainMenu::instance()->add_Filters_Item(
            _("Retargeting with Interaction Mesh"),
            [](){ RetargetingDialog::getOrCreateInstance()->show(); });

        ItemTreeView::customizeContextMenu<MarkerIMeshItem>(
            [](MarkerIMeshItem* item, MenuManager& menu, ItemFunctionDispatcher menuFunction){
                menu.addItem(_("Retargeting"))->sigTriggered().connect(
                    [](){ RetargetingDialog::getOrCreateInstance()->show(); });
                menu.addSeparator();
                menuFunction.dispatchAs<Item>(item);
            });
        
        plugin->setProjectArchiver(
            "RetargetingDialog",
            [=](Archive& archive){
                if(auto instance = RetargetingDialog::instance()){
                    return instance->store(archive);
                }
                return true;
            },
            [=](const Archive& archive){
                return RetargetingDialog::getOrCreateInstance()->restore(archive);
            });
    }
}


MarkerIMeshItem::MarkerIMeshItem()
    : mesh_(new MarkerIMesh)
{
    doUseCurrentMarkerPositions = true;
}


MarkerIMeshItem::MarkerIMeshItem(const MarkerIMeshItem& org)
    : IMeshItem(org),
      mesh_(new MarkerIMesh(*org.mesh_))
{
    doUseCurrentMarkerPositions = true;
}


MarkerIMeshItem::IMeshBase* MarkerIMeshItem::interactionMeshBase()
{
    return mesh_.get();
}


Item* MarkerIMeshItem::doCloneItem(CloneMap* /* cloneMap */) const
{
    return new MarkerIMeshItem(*this);
}


void MarkerIMeshItem::updateMesh()
{
    mesh_->clear();
    motionItems.clear();
    
    ItemList<MarkerMotionItem> selected;
    selected.extractChildItems(this);

    if(!selected.empty()){
        addConnectionInCheckState(
            TimeBar::instance()->sigTimeChanged().connect(
                [&](double time){ doUseCurrentMarkerPositions = false; return true; }));

        for(int i=0; i < selected.size(); ++i){
            MarkerMotionItemPtr motionItem = selected[i];
            MocapMappingItem* mocapMappingItem = motionItem->findOwnerItem<MocapMappingItem>();
            mesh_->addMotion(
                motionItem->motion(), 
                mocapMappingItem ? mocapMappingItem->mocapMapping() : MocapMappingPtr());
            
            motionItems.push_back(motionItem.get());
            
            addConnectionInCheckState(
                motionItem->sigCurrentMarkerPositionsChanged().connect(
                    [&](){
                        doUseCurrentMarkerPositions = true;
                        notifyMeshVertiesChanged();
                    }));
            
            addConnectionInCheckState(
                motionItem->sigMarkerPositionChangeRequest().connect(
                    [this, i](int markerIndex, const Vector3& newPosition){
                        onMarkerPositionChangeRequest(i, markerIndex, newPosition); }));
        }
    }
            
    MessageView* mv = MessageView::instance();
    mv->put(format(_("Calculating the interaction mesh of {0} ..."), name()));
    if(mesh_->update()){
        mv->putln(_("OK!"));
    } else {
        mv->putln(_("failed."));
    }
    if(!mesh_->message().empty()){
        mv->putln(mesh_->message());
    }
}


void MarkerIMeshItem::onMarkerPositionChangeRequest
(int motionIndex, int markerIndex, const Vector3& newPosition)
{
    MarkerIMeshSolver solver;
    solver.setSingleFrameMode(true, motionItems[motionIndex]->currentFrame());
    solver.excludeBoneEdgesFromLaplacianCoordinate(false);
    solver.setInteractionMesh(mesh_);

    // set mocap mappings
    for(size_t i=0; i < motionItems.size(); ++i){
        auto mocapMappingItem = motionItems[i]->findOwnerItem<MocapMappingItem>();
        if(mocapMappingItem){
            solver.setMocapMappingPair(i, mocapMappingItem->mocapMapping(), nullptr);
        } else {
            MessageView::instance()->putln(
                format(_("Warning: Character for {0} is not found."), motionItems[i]->name()));
        }
        MarkerMotionPtr motion = mesh_->motion(i);

        int leftAnkle = motion->markerIndex("LeftAnkle");
        if(i != motionIndex || leftAnkle != markerIndex){
            const Vector3& p = motionItems[i]->currentMarkerPosition(leftAnkle);
            if(p.z() < 0.1){
                solver.addHardPositionalConstraint(i, leftAnkle, p);
            }
        }
        int rightAnkle = motion->markerIndex("RightAnkle");
        if(i != motionIndex || rightAnkle != markerIndex){
            const Vector3& p = motionItems[i]->currentMarkerPosition(rightAnkle);
            if(p.z() < 0.1){
                solver.addHardPositionalConstraint(i, rightAnkle, p);
            }
        }
    }
        
    solver.addSoftPositionalConstraint(motionIndex, markerIndex, newPosition);

    if(!solver.solve(3)){
        mvout() << _("Interaction mesh deformation failed.\n")
                << solver.message() << endl;
    } else {
        for(size_t i=0; i < motionItems.size(); ++i){
            MarkerMotionItemPtr motionItem = motionItems[i];
            MarkerMotion::Frame solution = solver.solution(i)->frame(0);
            for(int j=0; j < solution.size(); ++j){
                motionItem->currentMarkerPosition(j) = solution[j];
            }
            motionItem->notifyCurrentMarkerPositionsChanged();
        }
    }
}


void MarkerIMeshItem::getMeshVertices(int frame, SgVertexArray* out_vertices)
{
    if(frame < 0){
        frame = 0;
    }
    out_vertices->resize(mesh_->numAllVertices());
    int vertexIndex = 0;

    for(size_t i=0; i < motionItems.size(); ++i){
        MarkerMotionItemPtr motionItem = motionItems[i];
        MarkerMotionPtr motion = motionItem->motion();
        int n = motion->numMarkers();
        if(doUseCurrentMarkerPositions){
            for(int j=0; j < n; ++j){
                out_vertices->at(vertexIndex++) = motionItem->currentMarkerPosition(j).cast<Vector3f::Scalar>();
            }
        } else {
            int actualFrame = motion->clampFrameIndex(frame - motion->offsetTimeFrame());
            MarkerMotion::Frame markerFrame = motion->frame(actualFrame);
            for(int j=0; j < n; ++j){
                out_vertices->at(vertexIndex++) = (markerFrame[j] + motion->positionOffset()).cast<Vector3f::Scalar>();
            }
        }
    }
}


RetargetingDialog* RetargetingDialog::getOrCreateInstance()
{
    if(!retargetingDialog){
        retargetingDialog = plugin->manage(new RetargetingDialog);
    }
    return retargetingDialog;
}


RetargetingDialog* RetargetingDialog::instance()
{
    return retargetingDialog;
}


RetargetingDialog::RetargetingDialog()
    : os(MessageView::instance()->cout())
{
    setWindowTitle(_("Marker Motion Retargeting"));

    QHBoxLayout* hbox;
    QVBoxLayout* vbox = new QVBoxLayout;
    setLayout(vbox);

    hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel(_("Steps")));
    stepNumberSpin.setRange(1, 99);
    stepNumberSpin.setValue(5);
    hbox->addWidget(&stepNumberSpin);
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    updateLaplacianConstantCheck.setText(_("Update laplacian coordinate constant every step"));
    updateLaplacianConstantCheck.setChecked(true);
    hbox->addWidget(&updateLaplacianConstantCheck);
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel(_("Laplacian distance weight power")));
    laplacianWeightPowSpin.setDecimals(1);
    laplacianWeightPowSpin.setRange(0.0, 9.9);
    laplacianWeightPowSpin.setValue(1.0);
    hbox->addWidget(&laplacianWeightPowSpin);
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel(_("Acceleration constraint weight")));
    accelConstraintWeightSpin.setDecimals(1);
    accelConstraintWeightSpin.setRange(0.1, 99.9);
    accelConstraintWeightSpin.setValue(1.0);
    hbox->addWidget(&accelConstraintWeightSpin);
    hbox->addWidget(new QLabel("e-6"));
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    excludeBonesFromLapacianCheck.setText(_("Exclude bones from the laplacian coordinate"));
    excludeBonesFromLapacianCheck.setChecked(false);
    hbox->addWidget(&excludeBonesFromLapacianCheck);
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    followingOrgFootPositionCheck.setText(_("Follow the original foot positions"));
    followingOrgFootPositionCheck.setChecked(true);
    hbox->addWidget(&followingOrgFootPositionCheck);
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel(_("Weight for following the original collar positions")));
    collarWeightSpin.setDecimals(2);
    collarWeightSpin.setRange(0.0, 10000.0);
    collarWeightSpin.setValue(1.0);
    hbox->addWidget(&collarWeightSpin);
    hbox->addStretch();
    vbox->addLayout(hbox);

    PushButton* applyButton = new PushButton(_("&Apply"));
    applyButton->setDefault(true);
    QDialogButtonBox* buttonBox = new QDialogButtonBox(this);
    buttonBox->addButton(applyButton, QDialogButtonBox::AcceptRole);
    applyButton->sigClicked().connect([&](){ accept(); });
    vbox->addWidget(buttonBox);
}


void RetargetingDialog::onAccepted()
{
    auto meshItems = RootItem::instance()->selectedItems<MarkerIMeshItem>();
    if(meshItems.empty()){
        return;
    }

    for(int i=0; i < meshItems.size(); ++i){

        MarkerIMeshItemPtr meshItem = meshItems[i];
        int numMotionItems = meshItem->numMeshedMotionItems();

        if(numMotionItems > 0){
            solver.clear();
            solver.setInteractionMesh(meshItem->mesh());
            solver.updateLaplacianCoordinateConstantEvreyFrame(updateLaplacianConstantCheck.isChecked());
            solver.excludeBoneEdgesFromLaplacianCoordinate(excludeBonesFromLapacianCheck.isChecked());
            solver.setLaplacianWeightPower(laplacianWeightPowSpin.value());
            solver.setAccelConstraintWeight(accelConstraintWeightSpin.value() * 1.0e-6);
            outputTopItem = new FolderItem;
            if(numMotionItems == 1){
                outputTopItem->setName(_("RetargetedMarkerMotion"));
            } else {
                outputTopItem->setName(_("RetargetedMarkerMotions"));
            }
            outputTopItem->setTemporary();
            outputs.clear();
            
            for(int j=0; j < numMotionItems; ++j){
                setRetargetingMotionItem(j, meshItem->meshedMotionItem(j));
            }
            
            if(solver.solve(stepNumberSpin.value(), mvout(true))){
                meshItem->parentItem()->insertChild(meshItem->nextItem(), outputTopItem);
            }
        }
    }

    outputTopItem = nullptr;
    outputs.clear();
}


void RetargetingDialog::setRetargetingMotionItem(int index, MarkerMotionItem* motionItem)
{
    MarkerMotionItemPtr outputMotionItem = new MarkerMotionItem;
    outputMotionItem->setTemporary();
    outputMotionItem->setName(motionItem->name());
    solver.setMotionToOutputSolution(index, outputMotionItem->motion());

    MocapMappingItemPtr orgMocapMappingItem = motionItem->findOwnerItem<MocapMappingItem>();

    if(!orgMocapMappingItem){
        outputTopItem->addChildItem(outputMotionItem);
        
    } else{
        MocapMappingItemPtr retargetMocapMappingItem = orgMocapMappingItem->findChildItem<MocapMappingItem>();
        if(!retargetMocapMappingItem){
            retargetMocapMappingItem = orgMocapMappingItem;
        }
        MocapMappingItemPtr duplicatedMocapMappingItem = new MocapMappingItem(*retargetMocapMappingItem);
        duplicatedMocapMappingItem->setTemporary();
        outputTopItem->addChildItem(duplicatedMocapMappingItem);

        solver.setMocapMappingPair(
            index, orgMocapMappingItem->mocapMapping(), retargetMocapMappingItem->mocapMapping());

        auto& mocapMappingName = retargetMocapMappingItem->mocapMapping()->name();
        if(mocapMappingName.empty()){
            outputMotionItem->setName(format("{0}-retargeted", outputMotionItem->name()));
        } else {
            outputMotionItem->setName(format("{0}-{1}", outputMotionItem->name(), mocapMappingName));
        }
        duplicatedMocapMappingItem->addChildItem(outputMotionItem);
    }
    //solver.addHardConstraintToKeepOriginalMarkerPosition(index, motionItem->motion()->markerIndex("LeftToe"));
    //solver.addHardConstraintToKeepOriginalMarkerPosition(index, motionItem->motion()->markerIndex("RightToe"));

    if(followingOrgFootPositionCheck.isChecked()){    
        solver.addHardConstraintToKeepOriginalMarkerPosition(index, motionItem->motion()->markerIndex("LeftAnkle"));
        solver.addHardConstraintToKeepOriginalMarkerPosition(index, motionItem->motion()->markerIndex("RightAnkle"));
    }

    if(collarWeightSpin.value() > 0.0){
        solver.addSoftConstraintToKeepOriginalMarkerPosition(
            index, motionItem->motion()->markerIndex("FrontCollar"),
            collarWeightSpin.value());
    }
    /*
    solver.addSoftConstraintToKeepOriginalMarkerPosition(
        index, motionItem->motion()->markerIndex("Chest"),
        10.0, Vector3(0.0, 0.0, -0.1));
    */
}


bool RetargetingDialog::store(Archive& archive)
{
    archive.write("num_steps", stepNumberSpin.value());
    archive.write("exclude_bones_from_laplacian", excludeBonesFromLapacianCheck.isChecked());
    archive.write("laplacian_weight_power", laplacianWeightPowSpin.value());
    archive.write("acceleration_constraint_weight", accelConstraintWeightSpin.value());
    archive.write("follow_original_foot_positions", followingOrgFootPositionCheck.isChecked());
    archive.write("original_collar_position_weight", collarWeightSpin.value());
    return true;
}


void RetargetingDialog::restore(const Archive& archive)
{
    stepNumberSpin.setValue(archive.get({ "num_steps", "numSteps" }, stepNumberSpin.value()));

    excludeBonesFromLapacianCheck.setChecked(
        archive.get({ "exclude_bones_from_laplacian", "excludingBonesFromLaplacian", },
                    excludeBonesFromLapacianCheck.isChecked()));
    
    laplacianWeightPowSpin.setValue(
        archive.get({ "laplacian_weight_power", "laplacianWeightPower" }, laplacianWeightPowSpin.value()));
    
    accelConstraintWeightSpin.setValue(
        archive.get({ "acceleration_constraint_weight", "accelConstraintWeight" },
                    accelConstraintWeightSpin.value()));
    
    followingOrgFootPositionCheck.setChecked(
        archive.get({ "follow_original_foot_positions", "followingOrgFootPositions" },
                    followingOrgFootPositionCheck.isChecked()));
    
    collarWeightSpin.setValue(
        archive.get({ "original_collar_position_weight", "originalCollarPositionWeight" },
                    collarWeightSpin.value()));
}
