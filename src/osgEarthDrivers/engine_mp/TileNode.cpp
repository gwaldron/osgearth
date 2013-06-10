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

#include <osg/ClusterCullingCallback>
#include <osg/NodeCallback>
#include <osg/NodeVisitor>
#include <osg/Uniform>

using namespace osgEarth_engine_mp;
using namespace osgEarth;
using namespace OpenThreads;

#define LC "[TileNode] "


//----------------------------------------------------------------------------

TileNode::TileNode( const TileKey& key, const TileModel* model ) :
_key  ( key ),
_model( model )
{
    this->setName( key.str() );

    osg::StateSet* stateset = getOrCreateStateSet();

    // TileKey uniform.
    _keyUniform = new osg::Uniform(osg::Uniform::FLOAT_VEC4, "oe_tile_key");
    _keyUniform->setDataVariance( osg::Object::STATIC );
    _keyUniform->set( osg::Vec4f(0,0,0,0) );
    stateset->addUniform( _keyUniform );

    // born-on date uniform.
    _bornUniform = new osg::Uniform(osg::Uniform::FLOAT, "oe_tile_birthtime");
    _bornUniform->set( -1.0f );
    stateset->addUniform( _bornUniform );

#if 0
    // Texture matrix uniform that transforms texture coordinates from the 
    // parent tile into the quadrant of this tile.
    _tileParentMatrixUniform = new osg::Uniform(osg::Uniform::FLOAT_MAT4, "oe_tile_parent_matrix");
    unsigned q = key.getQuadrant();
    osg::Matrixf scaleBias;
    model->createScaleBiasMatrix(scaleBias);
    scaleBias(0,0) = 0.5f;
    scaleBias(1,1) = 0.5f;
    scaleBias(3,0) = (float)(key.getTileX() & 0x1) * 0.5f;
    scaleBias(3,1) = (float)(1 - key.getTileY() & 0x1) * 0.5f;
    _tileParentMatrixUniform->set( scaleBias );
    stateset->addUniform( _tileParentMatrixUniform );
#endif
}


osg::BoundingSphere
TileNode::computeBound() const
{
    osg::BoundingSphere bs = osg::MatrixTransform::computeBound();
    
    unsigned tw, th;
    _key.getProfile()->getNumTiles(_key.getLOD(), tw, th);

    // swap the Y index.
    _keyUniform->set( osg::Vec4f(
        _key.getTileX(),
        th-_key.getTileY()-1.0,
        _key.getLOD(),
        bs.radius()) );

    return bs;
}


void
TileNode::traverse( osg::NodeVisitor& nv )
{
    // TODO: not sure we need this.
    if ( nv.getVisitorType() == nv.CULL_VISITOR )
    {
        osg::ClusterCullingCallback* ccc = dynamic_cast<osg::ClusterCullingCallback*>(getCullCallback());
        if (ccc)
        {
            if (ccc->cull(&nv,0,static_cast<osg::State *>(0))) return;
        }

        // set the birth time if not already set.
        const osg::FrameStamp* fs = nv.getFrameStamp();

        float bt;
        _bornUniform->get( bt );
        if ( bt < 0.0f )
        {
            _bornUniform->set( nv.getFrameStamp() ? (float)nv.getFrameStamp()->getReferenceTime() : 0.0f );
        }
    }

    osg::MatrixTransform::traverse( nv );
}


void
TileNode::releaseGLObjects(osg::State* state) const
{
    if ( _model.valid() )
        _model->releaseGLObjects( state );
}
