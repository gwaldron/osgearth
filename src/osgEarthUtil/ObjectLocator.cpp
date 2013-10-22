#include <osgEarthUtil/ObjectLocator>

using namespace osgEarth;
using namespace osgEarth::Util;

#define LC "[ObjectLocator] "

/***************************************************************************/

namespace
{
    void getHPRFromQuat(const osg::Quat& q, double& h, double& p, double& r)
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
}

/***************************************************************************/

ObjectLocator::ObjectLocator(const osgEarth::Map* map) :
_map                ( map ),
_componentsToInherit( COMP_ALL ),
_timestamp          ( 0.0 ),
_isEmpty            ( true ),
_rotOrder           ( HPR )
{
    if ( !_map.valid() )
        OE_WARN << LC << "Illegal: cannot create an ObjectLocator with a NULL Map." << std::endl;
}

ObjectLocator::ObjectLocator(ObjectLocator* parentLoc, unsigned int inheritMask ) :
_timestamp( 0.0 ),
_isEmpty  ( false ),
_rotOrder ( HPR )
{
    setParentLocator( parentLoc, inheritMask );
    _map = parentLoc->_map.get();
}

bool
ObjectLocator::isEmpty() const
{
    return _parentLoc.valid() ? _parentLoc->isEmpty() : _isEmpty;
}

void
ObjectLocator::setParentLocator( ObjectLocator* newParent, unsigned int inheritMask )
{
    if ( newParent == this )
    {
        OE_WARN << LC << "Illegal state, locator cannot be its own parent." << std::endl;
        return;
    }

    _parentLoc = newParent;
    _componentsToInherit = inheritMask;

    if ( newParent )
    {
        _map = newParent->_map.get();
    }

    if ( !_map.valid() )
    {
        OE_WARN << "Illegal state, cannot create a Locator with a NULL srs" << std::endl;
    }

    dirty();
}

void
ObjectLocator::setPosition( const osg::Vec3d& pos )
{
    _pos = pos;
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
ObjectLocator::getLocatorPosition( osg::Vec3d& output ) const
{
    if ( !isValid() )
        return false;

    output = _pos;

    if ( _parentLoc.valid() && (_componentsToInherit & COMP_POSITION) != 0 )
    {
        osg::Vec3d parentPos;
        _parentLoc->getLocatorPosition( parentPos );
        output += parentPos;
    }

    return true;
}

bool
ObjectLocator::getPositionMatrix( osg::Matrixd& output ) const
{
    osg::Vec3d pos;
    if ( !getLocatorPosition(pos) )
        return false;

    if ( _map->isGeocentric() )
    {
        _map->getProfile()->getSRS()->getEllipsoid()->computeLocalToWorldTransformFromLatLongHeight(
            osg::DegreesToRadians(pos.y()),
            osg::DegreesToRadians(pos.x()),
            pos.z(),
            output );
    }
    else
    {
        output.makeTranslate(pos);
    }

    return true;
}

bool
ObjectLocator::getLocatorOrientation( osg::Vec3d& output ) const
{
    if ( !isValid() )
        return false;

    output = _hpr;

    if ( _parentLoc.valid() && (_componentsToInherit & COMP_ORIENTATION) != 0 )
    {
        osg::Vec3d parentHPR;
        _parentLoc->getLocatorOrientation( parentHPR );
        output += parentHPR;
    }

    return true;
}

bool
ObjectLocator::getOrientationMatrix( osg::Matrixd& output, unsigned inherit ) const
{
    if ( !isValid() )
        return false;

    if ( (inherit & COMP_ORIENTATION) == 0 )
        return true;

    if ( _hpr[0] != 0.0 || _hpr[1] != 0.0 || _hpr[2] != 0.0 )
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

        // these look backwards, but it's actually a fast way to avoid inverting a matrix
        if ( _rotOrder == HPR )
            output.set( roll_q.conj() * pitch_q.conj() * azim_q.conj() );
        else if ( _rotOrder == RPH )
            output.set( azim_q.conj() * pitch_q.conj() * roll_q.conj() );

    }

    if ( _parentLoc.valid() && (_componentsToInherit * COMP_ORIENTATION) != 0 )
    {
        osg::Matrixd parentRot;
        if ( _parentLoc->getOrientationMatrix(parentRot, _componentsToInherit) )
            output = output * parentRot;
    }

    return true;
}

bool
ObjectLocator::getLocatorMatrix( osg::Matrixd& output, unsigned comps ) const
{
    bool ok = true;
    osg::Matrixd pos, rot;

    if ( comps & COMP_POSITION )
        if ( !getPositionMatrix(pos) )
            ok = false;

    if ( comps & COMP_ORIENTATION )
        if ( !getOrientationMatrix(rot, comps) )
            ok = false;

    output = rot * pos;
    return ok;
}

bool
ObjectLocator::inSyncWith( int exRev ) const
{
    return _parentLoc.valid() ? _parentLoc->inSyncWith( exRev ) :
        osgEarth::Revisioned::inSyncWith( exRev );
}

/***************************************************************************/

ObjectLocatorNode::ObjectLocatorNode()
{
    setNumChildrenRequiringUpdateTraversal( 1 );
}

ObjectLocatorNode::ObjectLocatorNode( const ObjectLocatorNode& rhs, const osg::CopyOp& op ) :
osg::MatrixTransform( rhs, op ),
_matrixRevision( rhs._matrixRevision ),
_locator( rhs._locator.get() )
{
    setNumChildrenRequiringUpdateTraversal( 1 );
    setLocator( _locator.get() ); // to update the trav count
}

ObjectLocatorNode::ObjectLocatorNode( ObjectLocator* locator ) :
_matrixRevision( -1 )
{
    setNumChildrenRequiringUpdateTraversal( 1 );
    setLocator( locator );
}

ObjectLocatorNode::ObjectLocatorNode( const Map* map ) :
_matrixRevision( -1 )
{
    setNumChildrenRequiringUpdateTraversal( 1 );
    setLocator( new ObjectLocator(map) );
}

void
ObjectLocatorNode::setLocator( ObjectLocator* locator )
{
    _locator = locator;
    _matrixRevision = -1;    
}


void
ObjectLocatorNode::traverse(osg::NodeVisitor &nv)
{
    if (nv.getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR)
    {
        update();
    }
    osg::MatrixTransform::traverse( nv );
}

void
ObjectLocatorNode::update()
{
    if ( _locator.valid() && _locator->outOfSyncWith( _matrixRevision ) )
    {
        osg::Matrix mat;
        if ( _locator->getLocatorMatrix( mat ) )
        {
            this->setMatrix( mat );
            this->dirtyBound();
            _locator->sync( _matrixRevision );
        }
    }
}
