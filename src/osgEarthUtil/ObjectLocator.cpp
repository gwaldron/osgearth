#include <osgEarthUtil/ObjectLocator>

using namespace osgEarth;
using namespace osgEarth::Util;

#define LC "[ObjectLocator] "

/***************************************************************************/

ObjectLocator::ObjectLocator(const osgEarth::SpatialReference* mapSRS) :
_mapSRS( mapSRS ),
_componentsToInherit( COMP_ALL ),
_timestamp( 0.0 ),
_isEmpty( true )
{
    if ( !_mapSRS.valid() )
        OE_WARN << LC << "Illegal: cannot create an ObjectLocator with a NULL SRS." << std::endl;
}

ObjectLocator::ObjectLocator(ObjectLocator* parentLoc, unsigned int inheritMask ) :
_timestamp( 0.0 ),
_isEmpty( false )
{
    setParentLocator( parentLoc, inheritMask );
}

bool
ObjectLocator::isEmpty() const
{
    return _parentLoc.valid() ? _parentLoc->isEmpty() : _isEmpty;
}

static void
getHPRFromQuat(const osg::Quat& q, double& h, double& p, double& r)
{
    osg::Matrixd rot(q);
    p = asin(rot(1,2));
    if( osg::equivalent(osg::absolute(p), osg::PI_2) )
    {
        r = 0.0;
        h = atan2( rot(0,1), rot(0,0) );
    }
    else
    {
        r = atan2( rot(0,2), rot(2,2) );
        h = atan2( rot(1,0), rot(1,1) );
    }
    h = osg::RadiansToDegrees(h);
    p = osg::RadiansToDegrees(p);
    r = osg::RadiansToDegrees(r);
}


void
ObjectLocator::setParentLocator( ObjectLocator* newParent, unsigned int inheritMask )
{
    if ( newParent == this )
    {
        OE_WARN << LC << "Illegal state: Locator cannot self-parent" << std::endl;
        return;
    }

    if ( newParent == 0L )
    {
        // when clearing the parent locator, reset the XYZ and HPR to the absolute
        // values previously derived from the parent.
        osg::Matrixd mat;
        if ( getWorldMatrix( mat ) )
        {
            _xyz = mat.getTrans();
            getHPRFromQuat( mat.getRotate(), _hpr[0], _hpr[1], _hpr[2] );
            _isEmpty = false;
        }
        else
        {
            _isEmpty = true;
        }
    }
    else
    {
        _isEmpty = false;
    }

    _parentLoc = newParent;
    _componentsToInherit = inheritMask;

    if ( newParent )
        _mapSRS = newParent->getSRS();

    if ( !_mapSRS.valid() )
        OE_WARN << LC << "Illegal: cannot create an ObjectLocator with a NULL SRS." << std::endl;

    dirty();
}

void
ObjectLocator::setPosition( const osg::Vec3d& xyz )
{
    _xyz = xyz;
    _isEmpty = false;
    dirty();
}

void
ObjectLocator::setOrientation( const osg::Vec3d& hpr )
{
    _hpr = hpr;
    _isEmpty = false;
    dirty();
}

bool 
ObjectLocator::getWorldMatrix( osg::Matrixd& output, unsigned int inherit ) const
{
    bool outputOK = true;

    if ( isEmpty() )
    {
        outputOK = false;
    }
    else if ( _mapSRS.valid() )
    {
        // first figure out the orientation
        osg::Quat azim_q;
        if ( inherit & COMP_HEADING )
            azim_q = osg::Quat( osg::DegreesToRadians(_hpr[0]), osg::Vec3d(0,0,1) );

        osg::Quat pitch_q;
        if ( inherit & COMP_PITCH )
            pitch_q = osg::Quat( -osg::DegreesToRadians(_hpr[1]), osg::Vec3d(1,0,0) );

        osg::Quat roll_q;
        if ( inherit & COMP_ROLL )
            roll_q = osg::Quat( osg::DegreesToRadians(_hpr[2]), osg::Vec3d(0,1,0) );

        osg::Matrix rot = osg::Matrixd( azim_q * pitch_q * roll_q );
        osg::Quat localRot = osg::Matrixd::inverse(rot).getRotate();


        if ( !_parentLoc.valid() )
        {
            // top-level locator is in absolute map coordinates.
            osg::Matrixd local2World;

            if ( inherit & COMP_POSITION )
            {
                _mapSRS->getEllipsoid()->computeLocalToWorldTransformFromLatLongHeight(
                    osg::DegreesToRadians( _xyz[1] ), osg::DegreesToRadians( _xyz[0] ), _xyz[2],
                    local2World );
            }
            output =
                osg::Matrixd::rotate( localRot ) * 
                local2World;        
        }
        else // if ( parentLoc_.valid() )
        {
            osg::Matrixd parentMat;
            if ( _parentLoc->getWorldMatrix( parentMat, _componentsToInherit ) )
            {
                // offset locator is relative to the parent, and the pos/ori are in platform-local coords.
                output = 
                    osg::Matrixd::translate( _xyz ) * 
                    osg::Matrixd::rotate( localRot ) *
                    parentMat;
            }
            else
            {
                outputOK = false;
            }
        }
    }
    else
    {
        outputOK = false;
        OE_WARN << LC << "Illegal state: No map SRS." << std::endl;
    }

    return outputOK;
}

bool
ObjectLocator::inSyncWith( int exRev ) const
{
    return _parentLoc.valid() ? _parentLoc->inSyncWith( exRev ) :
        osgEarth::Revisioned<osg::Referenced>::inSyncWith( exRev );
}

/***************************************************************************/

// Binds the update traversal to the udpate() method
struct LocatorUpdateCallback : public osg::NodeCallback
{
    void operator()( osg::Node* node, osg::NodeVisitor* nv ) // override
    {
        static_cast<ObjectLocatorNode*>( node )->update();
        traverse( node, nv );
    }
};

ObjectLocatorNode::ObjectLocatorNode()
{
    //nop
}

ObjectLocatorNode::ObjectLocatorNode( const ObjectLocatorNode& rhs, const osg::CopyOp& op ) :
osg::MatrixTransform( rhs, op ),
_matrixRevision( rhs._matrixRevision ),
_locator( rhs._locator.get() )
{
    setLocator( _locator.get() ); // to update the trav count
}

ObjectLocatorNode::ObjectLocatorNode( ObjectLocator* locator ) :
_matrixRevision( -1 )
{
    setLocator( locator );
}

void
ObjectLocatorNode::setLocator( ObjectLocator* locator )
{
    _locator = locator;
    _matrixRevision = -1;
    setUpdateCallback( _locator.valid() ? new LocatorUpdateCallback() : 0L );
}

void
ObjectLocatorNode::update()
{
    if ( _locator.valid() && _locator->outOfSyncWith( _matrixRevision ) )
    {
        osg::Matrix mat;
        if ( _locator->getWorldMatrix( mat ) )
        {
            this->setMatrix( mat );
            this->dirtyBound();
            _locator->sync( _matrixRevision );
        }
    }
}
