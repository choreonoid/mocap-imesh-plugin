#include "MarkerIMeshItem.h"
#include "BodyIMeshItem.h"
#include <cnoid/Plugin>

using namespace std;
using namespace cnoid;

namespace {
  
class MocapIMeshPlugin : public Plugin
{
public:
    MocapIMeshPlugin();
    virtual bool initialize() override;
};

}


MocapIMeshPlugin::MocapIMeshPlugin()
    : Plugin("MocapIMesh")
{
    require("Mocap");
}


bool MocapIMeshPlugin::initialize()
{
    MarkerIMeshItem::initialize(this);
    BodyIMeshItem::initialize(this);
    return true;
}


CNOID_IMPLEMENT_PLUGIN_ENTRY(MocapIMeshPlugin)
