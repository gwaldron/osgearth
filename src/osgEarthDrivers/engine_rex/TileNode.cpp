/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2014 Pelican Mapping
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
#include <osgEarth/VirtualProgram>
#include <osgEarth/ShaderGenerator>
#include <osgEarth/DrawInstanced>
#include <osgEarth/Registry>
#include <osgEarth/CullingUtils>
#include <osgEarth/ImageUtils>
#include <osgUtil/Optimizer>

using namespace osgEarth::Drivers::RexTerrainEngine;
using namespace osgEarth;
using namespace OpenThreads;

#define LC "[TileNode] "

TileNode::TileNode() :
_model( 0L )
{
    //NOP
}

TileNode::TileNode(const TerrainTileModel* model) :
_model             ( model ),
_lastTraversalFrame( 0 ),
_dirty             ( false ),
_outOfDate         ( false )
{
    // model required.
    if ( !model )
    {
        OE_WARN << LC << "Illegal: Created a tile node with no model\n";
        return;
    }

    this->setName( _model->getKey().str() );

    // revisions are initially in sync:
    if ( model )
    {
        _mapRevision = model->getRevision();

        if ( model->requiresUpdateTraverse() )
        {
            this->setNumChildrenRequiringUpdateTraversal(1);
        }
    }
}


void
TileNode::setLastTraversalFrame(unsigned frame)
{
    _lastTraversalFrame = frame;
}

#define OE_TEST OE_DEBUG
void
TileNode::notifyOfArrival(TileNode* that)
{
    OE_TEST << LC << this->getKey().str()
        << " was waiting on "
        << that->getKey().str() << " and it arrived.\n";
        
    osg::Texture* thisTex = this->getModel()->getNormalTexture();
    osg::Texture* thatTex = that->getModel()->getNormalTexture();
    if ( !thisTex || !thatTex ) {
        OE_TEST << LC << "bailed on " << getKey().str() << " - null normal texture\n";
        return;
    }

    osg::RefMatrixf* thisTexMat = this->getModel()->getNormalTextureMatrix();
    osg::RefMatrixf* thatTexMat = that->getModel()->getNormalTextureMatrix();
    if ( !thisTexMat || !thatTexMat || !thisTexMat->isIdentity() || !thatTexMat->isIdentity() ) {
        OE_TEST << LC << "bailed on " << getKey().str() << " - null texmat\n";
        return;
    }

    osg::Image* thisImage = thisTex->getImage(0);
    osg::Image* thatImage = thatTex->getImage(0);
    if ( !thisImage || !thatImage ) {
        OE_TEST << LC << "bailed on " << getKey().str() << " - null image\n";
        return;
    }

    int width = thisImage->s();
    int height = thisImage->t();
    if ( width != thatImage->s() || height != thatImage->t() ) {
        OE_TEST << LC << "bailed on " << getKey().str() << " - mismatched sizes\n";
        return;
    }

    //if (_model->_normalData.isFallbackData()) {
    //    OE_TEST << LC << "bailed on " << getKey().str() << " - fallback data\n";
    //    return;
    //}

    // Just copy the neighbor's edge normals over to our texture.
    // Averaging them would be more accurate, but then we'd have to
    // re-generate each texture multiple times instead of just once.
    // Besides, there's almost no visual difference anyway.
    ImageUtils::PixelReader readThat(thatImage);
    ImageUtils::PixelWriter writeThis(thisImage);

    bool thisDirty = false;

    if ( that->getKey() == getKey().createNeighborKey(1,0) )
    {
        // neighbor is to the east:
        for(int t=1; t<height; ++t) // start at 1 to skip the corner piece
        {
            writeThis(readThat(0,t), width-1, t);
        }
        thisDirty = true;
    }

    else if ( that->getKey() == getKey().createNeighborKey(0,1) )
    {
        // neighbor is to the south:
        for(int s=0; s<width-1; ++s) // -1 because of the corner piece
        {
            writeThis(readThat(s, height-1), s, 0);
        }
        thisDirty = true;
    }

    else if ( that->getKey() == getKey().createNeighborKey(1,1) )
    {
        // the corner
        writeThis(readThat(0,height-1), width-1, 0);
        thisDirty = true;
    }

    else
    {
        OE_INFO << LC << "Unhandled notify\n";
    }

    if ( thisDirty )
    {
        // so heavy handed. Wish we could just write the row
        // or column that changed.
        thisImage->dirty();
    }
}
