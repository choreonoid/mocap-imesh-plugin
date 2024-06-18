#include "BodyIMeshItem.h"
#include "BodyIMeshSolver.h"
#include <cnoid/LeggedBodyHelper>
#include <cnoid/MessageView>
#include <cnoid/ItemManager>
#include <cnoid/MainMenu>
#include <cnoid/RootItem>
#include <cnoid/Archive>
#include <cnoid/Dialog>
#include <cnoid/Button>
#include <cnoid/SpinBox>
#include <cnoid/ComboBox>
#include <fmt/format.h>
#include <QLabel>
#include <QLayout>
#include <QDialogButtonBox>
#include <set>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {

class JointLimitterDialog : public Dialog
{
public:
    ostream& os;
    ComboBox solverTypeCombo;
    SpinBox stepNumberSpin;

    CheckBox deformationEnergyTermCheck;
    DoubleSpinBox maxLaplacianDiffSpin;
    DoubleSpinBox jointDampingWeightSpin;
    DoubleSpinBox accTermWeightSpin;
    CheckBox accTermOnlyLastCheck;
    CheckBox velConstraintCheck;
    SpinBox shrinkStepNumberSpin;
    SpinBox velConstraintKeepingStepSpin;

    CheckBox globalJointLimitShrinkModeCheck;
    CheckBox fixFeetCheck;
    CheckBox onlyCurrentPoseCheck;

    JointLimitterDialog();
    virtual void onAccepted() override;
    void applyBodyIMeshSolver();

    bool store(Archive& archive);
    void restore(const Archive& archive);
};

class InterpolationTestDialog : public Dialog
{
public:
    ostream& os;
    SpinBox stepNumberSpin;
    CheckBox interpolationCheck;
    CheckBox useQuadraticCheck;

    InterpolationTestDialog();
    virtual void onAccepted() override;
    bool store(Archive& archive);
    void restore(const Archive& archive);
};
    
}


void BodyIMeshItem::initialize(ExtensionManager* ext)
{
    static bool initialized = false;
    if(!initialized){
        ext->itemManager()
            .registerClass<BodyIMeshItem>(N_("BodyInteractionMeshItem"))
            .addAlias<BodyIMeshItem>("BodyInteractionMeshItem", "Mocap")
            .addAlias<BodyIMeshItem>("BodyIMeshItem", "Mocap")
            .addCreationPanel<BodyIMeshItem>();

        auto mm = MainMenu::instance();

        JointLimitterDialog* jlDialog = ext->manage(new JointLimitterDialog);
        mm->add_Filters_Item(
            _("IM-based Joint Limitter"), [jlDialog](){ jlDialog->show(); });

        ext->setProjectArchiver(
            "JointLimitter",
            [jlDialog](Archive& archive){ return jlDialog->store(archive); },
            [jlDialog](const Archive& archive){ jlDialog->restore(archive); });

        InterpolationTestDialog* itDialog = ext->manage(new InterpolationTestDialog);
        mm->add_Filters_Item(
            _("Interpolation test using a body interaction mesh"), [itDialog](){ itDialog->show(); });
        
        ext->setProjectArchiver("InterpolationTestDialog",
                                [itDialog](Archive& archive){ return itDialog->store(archive); },
                                [itDialog](const Archive& archive){ itDialog->restore(archive); });
        
        initialized = true;
    }
}


BodyIMeshItem::BodyIMeshItem()
    : mesh_(new BodyIMesh)
{
    mesh_->setMessageOutput(mvout());
}


BodyIMeshItem::BodyIMeshItem(const BodyIMeshItem& org)
    : IMeshItem(org),
      mesh_(new BodyIMesh)
{
    mesh_->setMessageOutput(mvout());
}


BodyIMeshItem::IMeshBase* BodyIMeshItem::interactionMeshBase()
{
    return mesh_.get();
}


void BodyIMeshItem::onDisconnectedFromRoot()
{
    bodyItems_.clear();
    IMeshItem::onDisconnectedFromRoot();
}


Item* BodyIMeshItem::doCloneItem(CloneMap* /* cloneMap */) const
{
    return new BodyIMeshItem(*this);
}


void BodyIMeshItem::updateMesh()
{
    updateMesh(true);
}


void BodyIMeshItem::updateMesh(bool pickupMotions)
{
    mesh_->clear();

    bodyItems_.extractChildItems(this);
    for(int i=0; i < bodyItems_.size(); ++i){
        BodyItem* bodyItem = bodyItems_.get(i);
        shared_ptr<BodyMotion> motion;
        if(pickupMotions){
            ItemList<BodyMotionItem> items;
            items.extractChildItems(bodyItem);
            BodyMotionItem* motionItem = items.toSingle();
            if(motionItem){
                motion = motionItem->motion();
            }
        }
        if(!mesh_->addBody(bodyItem->body(), motion)){
            mvout() << format(_("{0} cannot be set to the interaction mesh"), bodyItem->name()) << endl;
        }
    }

    mvout() << format(_("Calculating the interaction mesh of {0} ..."), name()) << endl;
    if(mesh_->update()){
        mvout() << _("OK!") << endl;
    } else {
        mvout() << _("failed.") << endl;
    }
}


void BodyIMeshItem::getMeshVertices(int frame, SgVertexArray* out_vertices)
{
    if(frame < 0){
        frame = 0;
    }
    vector<Vector3> vertices;
    mesh_->getVertices(frame, vertices);
    int numVertices = vertices.size();
    out_vertices->resize(numVertices);
    for(int i=0; i < numVertices; ++i){
        out_vertices->at(i) = vertices[i].cast<Vector3f::Scalar>();
    }
}


JointLimitterDialog::JointLimitterDialog()
    : os(MessageView::instance()->cout())
{
    setWindowTitle(_("IM-based joint limitter"));

    QVBoxLayout* vbox = new QVBoxLayout;
    setLayout(vbox);

    QHBoxLayout* hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel(_("Solver type")));
    solverTypeCombo.enableI18n(CNOID_GETTEXT_DOMAIN_NAME);
    solverTypeCombo.addI18nItem(N_("Lagrange multiplier method solver"));
    hbox->addWidget(&solverTypeCombo);
    hbox->addStretch();
    vbox->addLayout(hbox);
    
    hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel(_("Total steps")));
    stepNumberSpin.setRange(1, 999);
    stepNumberSpin.setValue(8);
    hbox->addWidget(&stepNumberSpin);
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    deformationEnergyTermCheck.setText(_("Laplacian deformation energy term"));
    deformationEnergyTermCheck.setChecked(true);
    hbox->addWidget(&deformationEnergyTermCheck);
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel(_("Max Laplacian difference toward the original position")));
    maxLaplacianDiffSpin.setDecimals(4);
    maxLaplacianDiffSpin.setRange(0.0, 0.1);
    maxLaplacianDiffSpin.setValue(0.001);
    hbox->addWidget(&maxLaplacianDiffSpin);
    hbox->addStretch();
    vbox->addLayout(hbox);
    
    hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel(_("Joint damping weight")));
    jointDampingWeightSpin.setDecimals(6);
    jointDampingWeightSpin.setRange(0.000001, 1.0);
    jointDampingWeightSpin.setValue(0.02);
    hbox->addWidget(&jointDampingWeightSpin);
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel(_("Acceleration term weight")));
    accTermWeightSpin.setDecimals(10);
    accTermWeightSpin.setRange(0.0, 1.0);
    accTermWeightSpin.setValue(1.0e-8);
    hbox->addWidget(&accTermWeightSpin);
    hbox->addStretch();
    vbox->addLayout(hbox);
    
    hbox = new QHBoxLayout;
    accTermOnlyLastCheck.setText(_("Apply acceleration term only for the last step with partial velocity constraints"));
    accTermOnlyLastCheck.setChecked(true);
    hbox->addWidget(&accTermOnlyLastCheck);
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    velConstraintCheck.setText(_("Velocity constraints"));
    velConstraintCheck.setChecked(true);
    hbox->addWidget(&velConstraintCheck);
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    hbox->addSpacing(20);
    hbox->addWidget(new QLabel(_("Initial steps for shrinking velocity limits")));
    shrinkStepNumberSpin.setRange(1, 999);
    shrinkStepNumberSpin.setValue(5);
    hbox->addWidget(&shrinkStepNumberSpin);
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    hbox->addSpacing(20);
    hbox->addWidget(new QLabel(_("Final steps for keeping previous velocity constraints")));
    velConstraintKeepingStepSpin.setRange(0, 999);
    velConstraintKeepingStepSpin.setValue(2);
    hbox->addWidget(&velConstraintKeepingStepSpin);
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    globalJointLimitShrinkModeCheck.setText(_("Shrink joint limit values globally"));
    globalJointLimitShrinkModeCheck.setChecked(false);
    hbox->addWidget(&globalJointLimitShrinkModeCheck);
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    fixFeetCheck.setText(_("Fix feet positions"));
    fixFeetCheck.setChecked(true);
    hbox->addWidget(&fixFeetCheck);
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    onlyCurrentPoseCheck.setText(_("Only the current pose"));
    onlyCurrentPoseCheck.setChecked(false);
    hbox->addWidget(&onlyCurrentPoseCheck);
    hbox->addStretch();
    vbox->addLayout(hbox);

    PushButton* applyButton = new PushButton(_("&Apply"));
    applyButton->setDefault(true);
    QDialogButtonBox* buttonBox = new QDialogButtonBox(this);
    buttonBox->addButton(applyButton, QDialogButtonBox::AcceptRole);
    applyButton->sigClicked().connect([this]() { accept(); });
    vbox->addWidget(buttonBox);
}


void JointLimitterDialog::onAccepted()
{
    switch(solverTypeCombo.currentIndex()){
    case 0:
        applyBodyIMeshSolver();
        break;
    default:
        break;
    }
}


void JointLimitterDialog::applyBodyIMeshSolver()
{
    auto meshItems = RootItem::instance()->selectedItems<BodyIMeshItem>();

    if(!meshItems.empty()){
        
        BodyIMeshSolver solver(mvout(true));
        solver.enableHalfwayMotionRecording(true);

        //solver.setFixedRootMode(!fixFeetCheck.isChecked());
        //solver.setFixedRootMode(true);
        solver.setFixedRootMode(false);
        solver.setAccTermWeight(accTermWeightSpin.value());
        solver.setJointDampingWeight(jointDampingWeightSpin.value());
        solver.setMaxLaplacianDiffToOrgPosition(maxLaplacianDiffSpin.value());

        for(int i=0; i < meshItems.size(); ++i){

            BodyIMeshItem* meshItem = meshItems.get(i);
            meshItem->updateMesh(!onlyCurrentPoseCheck.isChecked());
            solver.setInteractionMesh(meshItem->mesh());

            if(fixFeetCheck.isChecked()){
                for(int j=0; j < meshItem->numBodyItems(); ++j){
                    LeggedBodyHelperPtr legged = getLeggedBodyHelper(meshItem->bodyItem(j)->body());
                    if(legged->isValid()){
                        for(int k=0; k < legged->numFeet(); ++k){
                            solver.addHardConstraintToKeepOriginalLinkPosition(j, legged->footLink(k)->index());
                        }
                    }
                }
            }

            //if(solver.applyFrameLocally(stepNumberSpin.value())){
            solver.setNumSteps(stepNumberSpin.value());
            solver.setNumStepsOfShrinkingVelocityLimits(shrinkStepNumberSpin.value());
            solver.setNumStepsOfKeepingVelocityConstraints(velConstraintKeepingStepSpin.value());
            if(solver.apply()){
                for(int j=0; j < meshItem->numBodyItems(); ++j){
                    BodyItem* bodyItem = meshItem->bodyItem(j);
                    for(int k=0; k < solver.numHalfwayMotions(); ++k){
                        BodyMotionItemPtr motionItem = new BodyMotionItem(solver.halfwayMotion(j, k));
                        motionItem->setName(format(_("Halfway-{0}"), k));
                        motionItem->setTemporary(true);
                        bodyItem->addChildItem(motionItem);
                    }
                }
            }
        }
    }
}


bool JointLimitterDialog::store(Archive& archive)
{
    archive.write("solver_type", solverTypeCombo.currentOrgText().toStdString(), DOUBLE_QUOTED);
    archive.write("num_steps", stepNumberSpin.value());
    archive.write("num_shrink_steps", shrinkStepNumberSpin.value());
    archive.write("enable_deformation_energy_term", deformationEnergyTermCheck.isChecked());
    archive.write("max_laplacian_diff", maxLaplacianDiffSpin.value());
    archive.write("joint_damping_weight", jointDampingWeightSpin.value());
    archive.write("acceleration_term_weight", accTermWeightSpin.value());
    archive.write("enable_acceleration_term_only_in_last_step", accTermOnlyLastCheck.isChecked());
    archive.write("enable_velocity_constraints", velConstraintCheck.isChecked());
    archive.write("shrink_joint_limits_globally", globalJointLimitShrinkModeCheck.isChecked());
    archive.write("fix_feet", fixFeetCheck.isChecked());
    archive.write("current_pose_only", onlyCurrentPoseCheck.isChecked());
    return true;
}


void JointLimitterDialog::restore(const Archive& archive)
{
    string solverType;
    if(archive.read({ "solver_type", "solverType" }, solverType)){
        solverTypeCombo.findOrgText(solverType, true);
    }
    stepNumberSpin.setValue(
        archive.get({ "num_steps", "numSteps" }, stepNumberSpin.value()));
    
    shrinkStepNumberSpin.setValue(
        archive.get({ "num_shrink_steps", "numShrinkSteps" }, shrinkStepNumberSpin.value()));
    
    deformationEnergyTermCheck.setChecked(
        archive.get({ "enable_deformation_energy_term", "deformationEnergyTerm" },
                    deformationEnergyTermCheck.isChecked()));
    
    maxLaplacianDiffSpin.setValue(
        archive.get({ "max_laplacian_diff", "maxLaplacianDiff" }, maxLaplacianDiffSpin.value()));

    jointDampingWeightSpin.setValue(
        archive.get({ "joint_damping_weight", "jointDampingWeight" }, jointDampingWeightSpin.value()));
    
    accTermWeightSpin.setValue(
        archive.get({ "acceleration_term_weight", "accTermWeight" }, accTermWeightSpin.value()));
    
    accTermOnlyLastCheck.setChecked(
        archive.get({ "enable_acceleration_term_only_in_last_step", "accTermOnlyForLastStep" },
                    accTermOnlyLastCheck.isChecked()));
    
    velConstraintCheck.setChecked(
        archive.get({ "enable_velocity_constraints", "velocityConstraints" },
                    velConstraintCheck.isChecked()));
    
    globalJointLimitShrinkModeCheck.setChecked(
        archive.get({ "shrink_joint_limits_globally", "globalJointLimitShrinkMode", },
                    globalJointLimitShrinkModeCheck.isChecked()));

    fixFeetCheck.setChecked(
        archive.get({ "fix_feet", "fixFeet" }, fixFeetCheck.isChecked()));
    
    onlyCurrentPoseCheck.setChecked(
        archive.get({ "current_pose_only", "onlyCurrentPose" }, onlyCurrentPoseCheck.isChecked()));
}


InterpolationTestDialog::InterpolationTestDialog()
    : os(MessageView::instance()->cout())
{
    setWindowTitle(_("Interpolation test using a body interaction mesh"));

    QVBoxLayout* vbox = new QVBoxLayout;
    setLayout(vbox);

    QHBoxLayout* hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel(_("Steps")));
    stepNumberSpin.setRange(1, 999);
    stepNumberSpin.setValue(10);
    hbox->addWidget(&stepNumberSpin);
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    interpolationCheck.setText(_("Do interpolation in the Laplacian coordinate"));
    interpolationCheck.setChecked(true);
    hbox->addWidget(&interpolationCheck);
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    useQuadraticCheck.setText(_("Use quadratic form"));
    useQuadraticCheck.setChecked(false);
    hbox->addWidget(&useQuadraticCheck);
    hbox->addStretch();
    vbox->addLayout(hbox);
    
    PushButton* applyButton = new PushButton(_("&Apply"));
    applyButton->setDefault(true);
    QDialogButtonBox* buttonBox = new QDialogButtonBox(this);
    buttonBox->addButton(applyButton, QDialogButtonBox::AcceptRole);
    applyButton->sigClicked().connect([this](){ accept(); });
    vbox->addWidget(buttonBox);
}


void InterpolationTestDialog::onAccepted()
{
    auto bodyItems = RootItem::instance()->selectedItems<BodyItem>();

    if(!bodyItems.empty()){
        
        BodyIMeshSolver solver;

        BodyIMeshPtr mesh;
        for(int i=0; i < bodyItems.size(); ++i){
            BodyIMeshItem* item = bodyItems[i]->findOwnerItem<BodyIMeshItem>();
            if(!item || (mesh && item->mesh() != mesh)){
                mesh.reset();
                break;
            }
            mesh = item->mesh();
        }
        if(!mesh){
            mesh = std::make_shared<BodyIMesh>();
        }
        solver.setInteractionMesh(mesh);

        //solver.setFixedRootMode(true);
        solver.setFixedRootMode(false);

        for(int i=0; i < bodyItems.size(); ++i){
            BodyItem* bodyItem = bodyItems.get(i);
            BodyState initial;
            BodyState final;
            bodyItem->getInitialState(initial);
            final.storeStateOfBody(bodyItem->body());
            BodyMotionItemPtr motionItem = new BodyMotionItem;
            motionItem->setName(_("Interpolation Test"));
            bodyItem->addChildItem(motionItem);
            solver.setInterpolationTestPosePair(bodyItem->body(), initial, final, motionItem->motion());
        }

        solver.doInterpolationTest(
            stepNumberSpin.value(), interpolationCheck.isChecked(), useQuadraticCheck.isChecked());
    }
}


bool InterpolationTestDialog::store(Archive& archive)
{
    archive.write("num_steps", stepNumberSpin.value());
    archive.write("interpolation", interpolationCheck.isChecked());
    return true;
}


void InterpolationTestDialog::restore(const Archive& archive)
{
    stepNumberSpin.setValue(archive.get("num_steps", stepNumberSpin.value()));
    interpolationCheck.setChecked(archive.get("interpolation", interpolationCheck.isChecked()));
}
