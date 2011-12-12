/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2010 Pelican Mapping
* http://osgearth.org
*
* osgEarth is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/

#include <osgEarthAnnotation/OrthoNode>
#include <osgEarthAnnotation/AnnotationUtils>
#include <osgEarthAnnotation/Decluttering>
#include <osgEarthSymbology/Color>
#include <osgEarth/ThreadingUtils>
#include <osgEarth/Utils>
#include <osgText/Text>

using namespace osgEarth;
using namespace osgEarth::Annotation;

OrthoNode::OrthoNode(const SpatialReference* mapSRS,
                     const osg::Vec3d&       pos ) :
_mapSRS        ( mapSRS ),
_horizonCulling( false )
{
    init();

    if ( mapSRS )
    {
        setHorizonCulling( true );
    }

    setPosition( pos );
}

OrthoNode::OrthoNode() :
_mapSRS        ( 0L ),
_horizonCulling( false )
{
    init();
}

void
OrthoNode::init()
{
    _autoxform = new osg::AutoTransform();
    _autoxform->setAutoRotateMode( osg::AutoTransform::ROTATE_TO_SCREEN );
    _autoxform->setAutoScaleToScreen( true );
    _autoxform->setCullingActive( false ); // for the first pass
    this->addChild( _autoxform.get() );

    _matxform = new osg::MatrixTransform();
    this->addChild( _matxform.get() );

    this->setSingleChildOn( 0 );

    this->getOrCreateStateSet()->setMode( GL_LIGHTING, 0 );
}

void
OrthoNode::traverse( osg::NodeVisitor& nv )
{
    osgUtil::CullVisitor* cv = 0L;

    if ( nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR )
    {
        cv = static_cast<osgUtil::CullVisitor*>( &nv );

        // make sure that we're NOT using the AutoTransform if this node is in the decluttering bin;
        // the decluttering bin automatically manages screen space transformation.
        bool declutter = cv->getCurrentRenderBin()->getName() == OSGEARTH_DECLUTTER_BIN;
        if ( declutter && getValue(0) == 1 )
        {
            this->setSingleChildOn( 1 );
        }
        else if ( !declutter && getValue(0) == 0 )
        {
            this->setSingleChildOn( 0 );
        }

        // turn off small feature culling
        cv->setSmallFeatureCullingPixelSize(0.0f);
    }

    for( unsigned pos = 0; pos < _children.size(); ++pos )
    {
        if ( _values[pos] )
            _children[pos]->accept( nv );
    }

    if ( cv )
    {
        this->setCullingActive( true );
    }
}

void
OrthoNode::attach( osg::Node* child )
{
    _autoxform->addChild( child );
    _matxform->addChild( child );
}

bool
OrthoNode::setPosition( const osg::Vec3d& pos )
{
    return setPosition( pos, 0L );
}

bool
OrthoNode::setPosition( const osg::Vec3d& pos, const SpatialReference* posSRS )
{
    // first transform the point to the map's SRS:
    osg::Vec3d mapPos = pos;
    if ( posSRS && _mapSRS.valid() && !posSRS->transform(pos, _mapSRS.get(), mapPos) )
        return false;

    CullNodeByHorizon* culler = dynamic_cast<CullNodeByHorizon*>(this->getCullCallback());

    // update the transform:
    if ( !_mapSRS.valid() )
    {
        _autoxform->setPosition( mapPos );
        _matxform->setMatrix( osg::Matrix::translate(pos) );
    }
    else
    {
        osg::Matrixd local2world;
        _mapSRS->createLocal2World( mapPos, local2world );
        _autoxform->setPosition( local2world.getTrans() );
        _matxform->setMatrix( local2world );
        
        if ( culler )
            culler->_world = local2world.getTrans();
    }

    return true;
}

osg::Vec3d
OrthoNode::getPosition() const
{
    return getPosition( 0L );
}

osg::Vec3d
OrthoNode::getPosition( const SpatialReference* srs ) const
{
    osg::Vec3d world = _autoxform->getPosition();
    if ( _mapSRS.valid() )
    {
        osg::Vec3d output = world;
        if ( _mapSRS->isGeographic() )
            _mapSRS->transformFromECEF( world, output );
    
        if ( srs )
            _mapSRS->transform( output, srs, output );

        return output;
    }
    else
    {
        return world;
    }
}

void
OrthoNode::setHorizonCulling( bool value )
{
    if ( _horizonCulling != value && _mapSRS.valid() )
    {
        _horizonCulling = value;

        if ( _horizonCulling )
        {
            osg::Vec3d world = _autoxform->getPosition();
            this->setCullCallback( new CullNodeByHorizon(world, _mapSRS->getEllipsoid()) );
        }
        else
        {
            this->removeCullCallback( this->getCullCallback() );
        }
    }
}
