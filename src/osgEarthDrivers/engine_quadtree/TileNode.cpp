/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2012 Pelican Mapping
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

#include <osgEarth/Registry>
#include <osgEarth/Locators>
#include <osgEarth/Map>
#include <osgEarth/NodeUtils>

#include <osg/ClusterCullingCallback>
#include <osg/NodeCallback>
#include <osg/NodeVisitor>
#include <osg/Node>
#include <osg/Texture2D>
#include <osgGA/EventVisitor>

#include <OpenThreads/ScopedLock>


using namespace osgEarth;
using namespace OpenThreads;

#define LC "[TileNode] "


//----------------------------------------------------------------------------

TileNode::TileNode( const TileKey& key, GeoLocator* keyLocator, bool quickReleaseGLObjects ) :
_key                  ( key ),
_locator              ( keyLocator )
{
    this->setName( key.str() );
}


TileNode::~TileNode()
{
    //nop
}


void
TileNode::setTileModel( TileModel* model )
{
    _model = model;
}


void
TileNode::compile( TileModelCompiler* compiler )
{
    osg::Node* node = compiler->compile( _model.get() );
    if ( node )
    {
        this->removeChildren( 0, this->getNumChildren() );
        this->addChild( node );
    }
}


osg::BoundingSphere
TileNode::computeBound() const
{
    //Overriden computeBound that takes into account the vertical scale.
    //OE_NOTICE << "TileNode::computeBound verticalScale = " << _verticalScale << std::endl;

    osg::BoundingSphere bs;

    if ( !_model.valid() )
        return bs;

    //if (_elevationLayer.valid())
    if ( _model->_elevationData.getHFLayer() )
    {
        osgTerrain::HeightFieldLayer* hflayer = _model->_elevationData.getHFLayer();

        if ( !hflayer->getLocator() )
            return bs;

        //if (!_elevationLayer->getLocator()) return bs;

        osg::BoundingBox bb;
        unsigned int numColumns = hflayer->getNumColumns();
        unsigned int numRows = hflayer->getNumRows();
        for(unsigned int r=0;r<numRows;++r)
        {
            for(unsigned int c=0;c<numColumns;++c)
            {
                float value = 0.0f;
                bool validValue = hflayer->getValidValue(c,r, value);
                if (validValue) 
                {
                    //Multiply by the vertical scale.
                    value *= _verticalScale;
                    osg::Vec3d ndc, v;
                    ndc.x() = ((double)c)/(double)(numColumns-1), 
                        ndc.y() = ((double)r)/(double)(numRows-1);
                    ndc.z() = value;

                    if (hflayer->getLocator()->convertLocalToModel(ndc, v))
                    {
                        bb.expandBy(v);
                    }
                }
            }
        }
        bs.expandBy(bb);

    }

    else
    {
        for(TileModel::ColorDataByUID::const_iterator i = _model->_colorData.begin(); i != _model->_colorData.end(); ++i )
        {
            bs.expandBy( i->second.computeBound() ); 
        }
    }

    return bs;
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
    }

    osg::Group::traverse( nv );
}
