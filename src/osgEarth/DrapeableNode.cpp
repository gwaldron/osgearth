/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */

#include <osgEarth/DrapeableNode>
#include <osgEarth/DrapingCullSet>
#include <osgEarth/CullingUtils>
#include <osgEarth/Utils>

#include <osgDB/ObjectWrapper>

#define LC "[DrapeableNode] "

using namespace osgEarth;


DrapeableNode::DrapeableNode() :
_drapingEnabled( true )
{
    // Unfortunetly, there's no way to return a correct bounding sphere for
    // the node since the draping will move it to the ground. The bounds
    // check has to be done by the Draping Camera at cull time. Therefore we
    // have to ensure that this node makes it into the draping cull set so it
    // can be frustum-culled at the proper time.
    setCullingActive( !_drapingEnabled );
}

DrapeableNode::DrapeableNode(const DrapeableNode& rhs, const osg::CopyOp& copy) :
osg::Group(rhs, copy),
_drapingEnabled(rhs._drapingEnabled)
{
    //nop
}

void
DrapeableNode::setDrapingEnabled(bool value)
{
    if ( value != _drapingEnabled )
    {
        _drapingEnabled = value;
        setCullingActive( !_drapingEnabled );
    }
}

void
DrapeableNode::traverse(osg::NodeVisitor& nv)
{
    if (_drapingEnabled && nv.getVisitorType() == nv.CULL_VISITOR)
    {
        osgUtil::CullVisitor* cv = Culling::asCullVisitor(nv);
        if (cv->getCurrentRenderBin()->getName() != "OE_EMPTY_RENDER_BIN")
        {
            std::shared_ptr<DrapingManager> dm;
            if (ObjectStorage::get(&nv, dm))
            {
                DrapingCullSet& cullSet = dm->get(cv->getCurrentCamera());
                cullSet.push(this, cv->getNodePath(), nv.getFrameStamp());
            }
        }
    }
    else
    {
        osg::Group::traverse(nv);
    }
}

//...........................................................................

#undef  LC
#define LC "[DrapeableNode Serializer] "


namespace osgEarth { namespace Serializers { namespace DrapeableNode
{
    REGISTER_OBJECT_WRAPPER(
        DrapeableNode,
        new osgEarth::DrapeableNode,
        osgEarth::DrapeableNode,
        "osg::Object osg::Node osg::Group osgEarth::DrapeableNode")
    {
        ADD_BOOL_SERIALIZER(DrapingEnabled, true);
    }
} } }
