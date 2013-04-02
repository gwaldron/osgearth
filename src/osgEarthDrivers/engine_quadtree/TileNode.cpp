/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2013 Pelican Mapping
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
#include "TileNode"
#include "TerrainNode"

#include <osg/ClusterCullingCallback>
#include <osg/NodeCallback>
#include <osg/NodeVisitor>

using namespace osgEarth_engine_quadtree;
using namespace osgEarth;
using namespace OpenThreads;

#define LC "[TileNode] "


//----------------------------------------------------------------------------

TileNode::TileNode( const TileKey& key, GeoLocator* keyLocator ) :
_key              ( key ),
_locator          ( keyLocator ),
_publicStateSet   ( 0L )
{
    this->setName( key.str() );

    _born = new osg::Uniform(osg::Uniform::FLOAT, "oe_birthTime");
    _born->set( -1.0f );
    this->getOrCreateStateSet()->addUniform( _born );
}


TileNode::~TileNode()
{
    //nop
}


void
TileNode::setTileModel( TileModel* model )
{
    _model = model;
    _publicStateSet = 0L;
}


bool
TileNode::compile( TileModelCompiler* compiler, bool releaseModel )
{
    if ( !_model.valid() )
        return false;

    osg::Node* node = 0L;
    _publicStateSet = 0L;

    if ( !compiler->compile( _model.get(), node, _publicStateSet ) )
        return false;

    this->removeChildren( 0, this->getNumChildren() );
    this->addChild( node );

    // release the memory associated with the tile model.
    if ( releaseModel )
        _model = 0L;

    return true;
}

void
TileNode::traverse( osg::NodeVisitor& nv )
{
    // TODO: not sure we need this.
    if ( nv.getVisitorType()==osg::NodeVisitor::CULL_VISITOR )
    {
        osg::ClusterCullingCallback* ccc = dynamic_cast<osg::ClusterCullingCallback*>(getCullCallback());
        if (ccc)
        {
            if (ccc->cull(&nv,0,static_cast<osg::State *>(0))) return;
        }

        float bt;
        _born->get( bt );
        if ( bt < 0.0f )
            _born->set( nv.getFrameStamp() ? (float)nv.getFrameStamp()->getReferenceTime() : 0.0f );
    }

    osg::Group::traverse( nv );
}
