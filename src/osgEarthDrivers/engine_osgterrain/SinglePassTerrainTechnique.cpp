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
#include "SinglePassTerrainTechnique"
#include "TerrainNode"
#include "Tile"

#include <osgEarth/Cube>
#include <osgEarth/ImageUtils>

#include <osg/Point>
#include <osg/Program>
#include <osg/io_utils>
#include <osg/StateSet>
#include <osg/Program>
#include <osg/Math>
#include <osg/Timer>
#include <osg/Version>
#include <osgUtil/DelaunayTriangulator>
#include <osgUtil/Tessellator>
#include <osgUtil/SmoothingVisitor>

#include <osgEarthSymbology/Geometry>
#include <osgEarthSymbology/MeshConsolidator>

#include <sstream>

using namespace osgEarth_engine_osgterrain;
using namespace osgEarth;
using namespace osgEarth::Symbology;
using namespace OpenThreads;

#define LC "[SinglePassTechnique] "

#define MATCH_TOLERANCE 0.000001

// --------------------------------------------------------------------------

namespace
{
  struct MaskRecord
  {
    osg::ref_ptr<osg::Vec3dArray> _boundary;
    osg::Vec3d _ndcMin, _ndcMax;
    osg::Geometry* _geom;
    osg::ref_ptr<osg::Vec3Array> _internal;

    MaskRecord(osg::Vec3dArray* boundary, osg::Vec3d& ndcMin, osg::Vec3d& ndcMax, osg::Geometry* geom) : _boundary(boundary), _ndcMin(ndcMin), _ndcMax(ndcMax), _geom(geom) { _internal = new osg::Vec3Array(); }
  };

  typedef std::vector<MaskRecord> MaskRecordVector;
}

// --------------------------------------------------------------------------

SinglePassTerrainTechnique::SinglePassTerrainTechnique( TextureCompositor* compositor ) :
CustomTerrainTechnique(),
_verticalScaleOverride(1.0f),
_atomicCallOnce(0),
_initCount(0),
_pendingFullUpdate( false ),
_pendingGeometryUpdate(false),
_optimizeTriangleOrientation(true),
_texCompositor( compositor ),
_frontGeodeInstalled( false ),
_debug( false ),
_compileMutex( Mutex::MUTEX_RECURSIVE ),
_clearDataAfterCompile( true )
{
    setThreadSafeRefUnref(true);
}

SinglePassTerrainTechnique::SinglePassTerrainTechnique(const SinglePassTerrainTechnique& rhs, const osg::CopyOp& copyop):
CustomTerrainTechnique( rhs, copyop ),
_verticalScaleOverride( rhs._verticalScaleOverride ),
_atomicCallOnce( 0 ),
_initCount( 0 ),
_pendingFullUpdate( false ),
_pendingGeometryUpdate( false ),
_optimizeTriangleOrientation( rhs._optimizeTriangleOrientation ),
_texCompositor( rhs._texCompositor.get() ),
_frontGeodeInstalled( rhs._frontGeodeInstalled ),
_debug( rhs._debug ),
_parentTile( rhs._parentTile ),
_compileMutex( Mutex::MUTEX_RECURSIVE ),
_clearDataAfterCompile( rhs._clearDataAfterCompile )
{
    //NOP
}

SinglePassTerrainTechnique::~SinglePassTerrainTechnique()
{
    //nop
}

void
SinglePassTerrainTechnique::setVerticalScaleOverride( float value )
{
    _verticalScaleOverride = value;
}

float
SinglePassTerrainTechnique::getVerticalScaleOverride() const 
{
    return _verticalScaleOverride;
}

void
SinglePassTerrainTechnique::setOptimizeTriangleOrientation(bool optimizeTriangleOrientation)
{
    _optimizeTriangleOrientation = optimizeTriangleOrientation;
}

bool
SinglePassTerrainTechnique::getOptimizeTriangleOrientation() const
{
    return _optimizeTriangleOrientation;
}

void 
SinglePassTerrainTechnique::init()
{
    compile( TileUpdate(TileUpdate::UPDATE_ALL), 0L );
    applyTileUpdates();

    if (_clearDataAfterCompile)
    {        
        _tile->clear();
    }
}

void
SinglePassTerrainTechnique::compile( const TileUpdate& update, ProgressCallback* progress )
{
    // safety check
    if ( !_tile ) 
    {
        OE_WARN << LC << "Illegal; terrain tile is null" << std::endl;
        return;
    }

    // only legal to call this once and only once.
    // lame. i know. but it's friday
    if ( _atomicCallOnce.OR(0x01) != 0 )
    {
        //OE_WARN << LC << "Tried to call more than once and was locked out" << std::endl;
        return;
    }

    //if ( _debug )
    //{
    //    OE_NOTICE << LC << "compile() " << std::endl;
    //}

    // serialize access to the compilation procedure.
    OpenThreads::ScopedLock<Mutex> exclusiveLock( _compileMutex );
    
    // make a frame to use during compilation.
    TileFrame tilef( _tile );

    // establish the master tile locator if this is the first compilation:
    if ( !_masterLocator.valid() || !_transform.valid() )
    {
        _masterLocator = static_cast<GeoLocator*>( tilef._locator.get() );
        _masterLocator->convertLocalToModel( osg::Vec3(.5,.5,0), _centerModel );

        _transform = new osg::MatrixTransform( osg::Matrix::translate(_centerModel) );
        // this is a placeholder so that we can always just call setChild(0) later.
        _transform->addChild( new osg::Group );
    }

    // see whether a full update is required.
    bool partialUpdateOK = _texCompositor->supportsLayerUpdate() && _frontGeodeInstalled;

    // handle image layer addition or update:
    if (partialUpdateOK && 
        ( update.getAction() == TileUpdate::ADD_IMAGE_LAYER || update.getAction() == TileUpdate::UPDATE_IMAGE_LAYER ))
    {
        prepareImageLayerUpdate( update.getLayerUID(), tilef );

        // conditionally regenerate the texture coordinates for this layer.
        // TODO: optimize this with a method that ONLY regenerates the texture coordinates.
        if ( !_texCompositor->requiresUnitTextureSpace() )
        {
            osg::ref_ptr<osg::StateSet> stateSet = _backNode.valid() ? _backNode->getStateSet() : 0L;
            _backNode = createGeometry( tilef );
            _backNode->setStateSet( stateSet.get() );

            _pendingGeometryUpdate = true;
        }
    }

    else if (partialUpdateOK && update.getAction() == TileUpdate::MOVE_IMAGE_LAYER )
    {
        //nop - layer re-ordering happens entirely in the texture compositor.
    }

    //TODO: we should not need to check supportsLayerUpdate here, but it is not working properly in
    // multitexture mode (white tiles show up). Need to investigate and fix.
    else if ( partialUpdateOK && update.getAction() == TileUpdate::UPDATE_ELEVATION )
    {
        osg::ref_ptr<osg::StateSet> stateSet = _backNode.valid() ? _backNode->getStateSet() : 0L;
        _backNode = createGeometry( tilef );
        _backNode->setStateSet( stateSet.get() );

        _pendingGeometryUpdate = true;
    }

    else // all other update types
    {
        // give the engine a chance to bail out before generating geometry
        if ( progress && progress->isCanceled() )
        {
            _backNode = 0L;
            return;
        }
    
        // create the geometry and texture coordinates for this tile in a new buffer
        _backNode = createGeometry( tilef );
        if ( !_backNode.valid() )
        {
            OE_WARN << LC << "createGeometry returned NULL" << std::endl;
            return;
        }

        // give the engine a chance to bail out before building the texture stateset:
        if ( progress && progress->isCanceled() )
        {
            _backNode = 0L;
            return;
        }

        // create the stateset for this tile, which contains all the texture information.
        osg::StateSet* stateSet = createStateSet( tilef );
        if ( stateSet )
        {
            _backNode->setStateSet( stateSet );
        }

        // give the engine a chance to bail out before swapping buffers
        if ( progress && progress->isCanceled() )
        {
            _backNode = 0L;
            return;
        }
       
        _initCount++;
        if ( _initCount > 1 )
            OE_WARN << LC << "Tile was fully build " << _initCount << " times" << std::endl;

        if ( _backNode.valid() && !_backNode->getStateSet() )
            OE_WARN << LC << "ILLEGAL! no stateset in BackNode!!" << std::endl;

        _pendingFullUpdate = true;
    }
}

// from the UPDATE traversal thread:
bool
SinglePassTerrainTechnique::applyTileUpdates()
{
    bool applied = false;

    // serialize access to the compilation mechanism.
    OpenThreads::ScopedLock<Mutex> exclusiveLock( _compileMutex );

    // process a pending buffer swap:
    if ( _pendingFullUpdate )
    {
        if ( _backNode->getStateSet() == 0L )
            OE_WARN << LC << "ILLEGAL: backGeode has no stateset" << std::endl;

        _transform->setChild( 0, _backNode.get() );
        _frontGeodeInstalled = true;
        _backNode = 0L;
        _pendingFullUpdate = false;
        _pendingGeometryUpdate = false;
        applied = true;
    }  

    else
    {
        // process any pending LIVE geometry updates:
        if ( _pendingGeometryUpdate )
        {
            osg::Group* frontNode = getFrontNode();

            if (frontNode)
            {
                if (frontNode->getNumChildren() != _backNode->getNumChildren())
                {
                    OE_WARN << "Error:  Front and back nodes do not have equal number of children" << std::endl;
                    return false;
                }

                if ( _texCompositor->requiresUnitTextureSpace() )
                {
                    for (unsigned int i = 0; i < _backNode->getNumChildren(); ++i)
                    {
                        osg::Geode* frontGeode = dynamic_cast< osg::Geode* > (frontNode->getChild(i));
                        osg::Geode* backGeode = dynamic_cast< osg::Geode* > (_backNode->getChild(i));
                        if (!frontGeode || !backGeode)
                        {
                            OE_WARN << "Error:  Children must be osg::Geodes" << std::endl;
                        }

                        // in "unit-texture-space" mode, we can take the shortcut of just updating
                        // the geometry VBOs. The texture coordinates never change.
                        for( unsigned int j=0; j< backGeode->getNumDrawables(); ++j )
                        {
                            osg::Geometry* backGeom = static_cast<osg::Geometry*>( backGeode->getDrawable(j) );
                            osg::Vec3Array* backVerts = static_cast<osg::Vec3Array*>( backGeom->getVertexArray() );

                            osg::Geometry* frontGeom = static_cast<osg::Geometry*>( frontGeode->getDrawable(j) );
                            osg::Vec3Array* frontVerts = static_cast<osg::Vec3Array*>( frontGeom->getVertexArray() );

                            if ( backVerts->size() == frontVerts->size() )
                            {
                                // simple VBO update:
                                std::copy( backVerts->begin(), backVerts->end(), frontVerts->begin() );
                                frontVerts->dirty();

                                osg::Vec3Array* backNormals = static_cast<osg::Vec3Array*>( backGeom->getNormalArray() );
                                if ( backNormals )
                                {
                                    osg::Vec3Array* frontNormals = static_cast<osg::Vec3Array*>( frontGeom->getNormalArray() );
                                    std::copy( backNormals->begin(), backNormals->end(), frontNormals->begin() );
                                    frontNormals->dirty();
                                }

                                osg::Vec2Array* backTexCoords = static_cast<osg::Vec2Array*>( backGeom->getTexCoordArray(0) );
                                if ( backTexCoords )
                                {
                                    osg::Vec2Array* frontTexCoords = static_cast<osg::Vec2Array*>( frontGeom->getTexCoordArray(0) );
                                    std::copy( backTexCoords->begin(), backTexCoords->end(), frontTexCoords->begin() );
                                    frontTexCoords->dirty();
                                }
                            }
                            else
                            {
                                frontGeom->setVertexArray( backVerts );
                                if ( backVerts->getVertexBufferObject() )
                                    backVerts->getVertexBufferObject()->setUsage(GL_STATIC_DRAW_ARB);

                                frontGeom->setTexCoordArray( 0, backGeom->getTexCoordArray( 0 ) ); // TODO: un-hard-code
                                if ( backGeom->getNormalArray() )
                                    frontGeom->setNormalArray( backGeom->getNormalArray() );
                            }
                        }
                    }
                }
                else
                {

                    for (unsigned int i = 0; i < _backNode->getNumChildren(); ++i)
                    {
                        osg::Geode* frontGeode = dynamic_cast< osg::Geode* > (frontNode->getChild(i));
                        osg::Geode* backGeode = dynamic_cast< osg::Geode* > (_backNode->getChild(i));
                        if (!frontGeode || !backGeode)
                        {
                            OE_WARN << "Error:  Children must be osg::Geodes" << std::endl;
                        }

                        // copy the drawables from the back buffer to the front buffer. By doing this,
                        // we don't touch the front geode's stateset (which contains the textures) and
                        // therefore they don't get re-applied.
                        for( unsigned int j=0; j<backGeode->getNumDrawables(); ++j )
                        {
                            frontGeode->setDrawable( j, backGeode->getDrawable( j ) );
                        }
                    }
                }
            }

            _pendingGeometryUpdate = false;
            _backNode = 0L;
            applied = true;
        }

        // process any pending LIVE per-layer updates:
        osg::StateSet* parentStateSet = 0;

        if ( !_pendingImageLayerUpdates.empty() )
        {
            parentStateSet = getParentStateSet();
        }

        while( _pendingImageLayerUpdates.size() > 0 )
        {
            const ImageLayerUpdate& update = _pendingImageLayerUpdates.front();

            osg::ref_ptr< osg::Group > front = getFrontNode();
            if (front.valid())
            {
                _texCompositor->applyLayerUpdate(
                    front->getStateSet(),
                    update._layerUID,
                    update._image,
                    _tileKey,
                    update._isRealData ? parentStateSet : 0L );
            }

            _pendingImageLayerUpdates.pop();
            applied = true;

        }
    }

    if ( _debug )
    {
        OE_NOTICE << "applyTileUpdates()" << std::endl;
    }

    return applied;
}

void
SinglePassTerrainTechnique::prepareImageLayerUpdate( UID layerUID, const TileFrame& tilef )
{
    CustomColorLayer layer;
    if ( tilef.getCustomColorLayer( layerUID, layer ) )
    {
        GeoImage geoImage, secondaryImage;

        if ( createGeoImage( layer, geoImage ) )
        {
            ImageLayerUpdate update;
            
            update._image = _texCompositor->prepareImage( geoImage, _tileExtent );
            update._layerUID = layerUID;
            update._isRealData = !layer.isFallbackData();

            if ( update._image.valid() )
                _pendingImageLayerUpdates.push( update );
        }

    }
}

bool
SinglePassTerrainTechnique::createGeoImage( const CustomColorLayer& colorLayer,
                                            GeoImage& image) const
{
    osg::ref_ptr<const GeoLocator> layerLocator = dynamic_cast<const GeoLocator*>( colorLayer.getLocator() );
    if ( layerLocator.valid() )
    {
        if ( layerLocator->getCoordinateSystemType() == osgTerrain::Locator::GEOCENTRIC )
            layerLocator = layerLocator->getGeographicFromGeocentric();

        const GeoExtent& imageExtent = layerLocator->getDataExtent();
        image = GeoImage( colorLayer.getImage(), imageExtent );
        return true;
    }
    return false;
}

osg::StateSet*
SinglePassTerrainTechnique::getActiveStateSet() const
{
    OpenThreads::ScopedLock<Mutex> exclusiveLock( const_cast<SinglePassTerrainTechnique*>(this)->_compileMutex );

    osg::StateSet* result = 0L;
    osg::Node* front = getFrontNode();
    if ( front ) 
        result = front->getStateSet();
    if ( !result && _backNode.valid() )
        result = _backNode->getStateSet();

    return result;
}

osg::StateSet*
SinglePassTerrainTechnique::getParentStateSet() const
{
    osg::StateSet* parentStateSet = 0;
    osg::ref_ptr<Tile> parentTile_safe = _parentTile.get();
    if ( parentTile_safe.valid() )
    {
        return static_cast<SinglePassTerrainTechnique*>(_parentTile->getTerrainTechnique())->getActiveStateSet();
    }
    else return 0L;
}

osg::StateSet*
SinglePassTerrainTechnique::createStateSet( const TileFrame& tilef )
{
    // establish the tile extent. we will calculate texture coordinate offset/scale based on this
    if ( !_tileExtent.isValid() )
    {
        osg::ref_ptr<GeoLocator> tileLocator = dynamic_cast<GeoLocator*>( tilef._locator.get() ); // _terrainTile->getLocator() );
        if ( tileLocator.valid() )
        {
            if ( tileLocator->getCoordinateSystemType() == osgTerrain::Locator::GEOCENTRIC )
                tileLocator = tileLocator->getGeographicFromGeocentric();

            _tileExtent = tileLocator->getDataExtent();
        }
        _tileKey = tilef._tileKey;
    }

    osg::StateSet* stateSet = new osg::StateSet();
    osg::StateSet* parentStateSet = getParentStateSet();

    for( ColorLayersByUID::const_iterator i = tilef._colorLayers.begin(); i != tilef._colorLayers.end(); ++i )
    {
        const CustomColorLayer& colorLayer = i->second;

        bool isRealData = !colorLayer.isFallbackData();

        GeoImage image;
        if ( createGeoImage( colorLayer, image ) )
        {
            image = _texCompositor->prepareImage( image, _tileExtent );

            _texCompositor->applyLayerUpdate( 
                stateSet, 
                colorLayer.getUID(), 
                image, 
                _tileKey, 
                isRealData ? parentStateSet : 0L );
        }
    }

    return stateSet;
}

void
SinglePassTerrainTechnique::calculateSampling( unsigned int& out_rows, unsigned int& out_cols, double& out_i, double& out_j )
{            
    osgTerrain::Layer* elevationLayer = _tile->getElevationLayer();

    out_rows = elevationLayer->getNumRows();
    out_cols = elevationLayer->getNumColumns();
    out_i = 1.0;
    out_j = 1.0;

    osg::ref_ptr< TerrainNode > terrain = _tile->getTerrain();

    float sampleRatio = terrain.valid() ? terrain->getSampleRatio() : 1.0f;
    if ( sampleRatio != 1.0f )
    {
        unsigned int originalNumColumns = out_cols;
        unsigned int originalNumRows = out_rows;

        out_cols = osg::maximum((unsigned int) (float(originalNumColumns)*sqrtf(sampleRatio)), 4u);
        out_rows = osg::maximum((unsigned int) (float(originalNumRows)*sqrtf(sampleRatio)),4u);

        out_i = double(originalNumColumns-1)/double(out_cols-1);
        out_j = double(originalNumRows-1)/double(out_rows-1);
    }
}

namespace
{
    struct GeoLocatorComp
    {
        bool operator()( const GeoLocator* lhs, const GeoLocator* rhs ) const
        {
            return rhs && lhs && lhs->isEquivalentTo( *rhs );
        }
    };

    typedef std::pair< const GeoLocator*, osg::Vec2Array* > LocatorTexCoordPair;

    struct LocatorToTexCoordTable : public std::list<LocatorTexCoordPair> {
        osg::Vec2Array* find( const GeoLocator* key ) const {
            for( const_iterator i = begin(); i != end(); ++i ) {
                if ( i->first->isEquivalentTo( *key ) )
                    return i->second;
            }
            return 0L;
        }
    };
    
    struct RenderLayer {
        CustomColorLayer _layer;
        osg::ref_ptr<const GeoLocator> _locator;
        osg::ref_ptr<osg::Vec2Array> _texCoords;
        osg::ref_ptr<osg::Vec2Array> _skirtTexCoords;
        osg::ref_ptr<osg::Vec2Array> _stitchTexCoords;
        osg::ref_ptr<osg::Vec2Array> _stitchSkirtTexCoords;
        bool _ownsTexCoords;
        RenderLayer() : _ownsTexCoords(false) { }
    };

    typedef std::vector< RenderLayer > RenderLayerVector;
}

osg::Group*
SinglePassTerrainTechnique::createGeometry( const TileFrame& tilef )
{
    osg::ref_ptr<GeoLocator> masterTextureLocator = _masterLocator.get();
    //GeoLocator* geoMasterLocator = dynamic_cast<GeoLocator*>(_masterLocator.get());

	bool isCube = dynamic_cast<CubeFaceLocator*>(_masterLocator.get()) != NULL;

    // If we have a geocentric locator, get a geographic version of it to avoid converting
    // to/from geocentric when computing texture coordinats
    if (!isCube && /*geoMasterLocator && */ _masterLocator->getCoordinateSystemType() == osgTerrain::Locator::GEOCENTRIC)
    {
        masterTextureLocator = masterTextureLocator->getGeographicFromGeocentric();
    }

    osg::Group* group = new osg::Group;
    
    osgTerrain::Layer* elevationLayer = _tile->getElevationLayer();

    // setting the geometry to DYNAMIC means its draw will not overlap the next frame's update/cull
    // traversal - which could access the buffer without a mutex

    osg::Geometry* surface = new osg::Geometry();

    surface->setThreadSafeRefUnref(true); // TODO: probably unnecessary.
    surface->setDataVariance( osg::Object::DYNAMIC );
    surface->setUseDisplayList(false);
    surface->setUseVertexBufferObjects(true);

    osg::Geode* surfaceGeode = new osg::Geode();
    surfaceGeode->addDrawable( surface );

    osg::Geometry* skirt = new osg::Geometry();
    skirt->setThreadSafeRefUnref(true); // TODO: probably unnecessary.
    skirt->setDataVariance( osg::Object::DYNAMIC );
    skirt->setUseDisplayList(false);
    skirt->setUseVertexBufferObjects(true);
    
    osg::Geode* skirtGeode = new osg::Geode;
    skirtGeode->addDrawable( skirt );

    osg::ref_ptr< TerrainNode > terrain = _tile->getTerrain();
    if ( terrain.valid() )
    {
        //Set the node masks of the surface and skirts if they are set.
        const TerrainOptions& opts = terrain->getTileFactory()->getTerrainOptions();
        surfaceGeode->setNodeMask( *opts.primaryTraversalMask() );
        skirtGeode->setNodeMask( *opts.secondaryTraversalMask() );
    }
    
    group->addChild( skirtGeode );
    group->addChild( surfaceGeode );    



	osg::ref_ptr<GeoLocator> geoLocator = _masterLocator;
	// Avoid coordinates conversion when GEOCENTRIC, so get a GEOGRAPHIC version of Locator 
	if (_masterLocator->getCoordinateSystemType() == osgTerrain::Locator::GEOCENTRIC) {
		geoLocator = _masterLocator->getGeographicFromGeocentric();
	}

	float scaleHeight = 
		_verticalScaleOverride != 1.0? _verticalScaleOverride :
		terrain.valid() ? terrain->getVerticalScale() :
		1.0f;

    MaskRecordVector masks;
    for (MaskLayerVector::const_iterator it = tilef._masks.begin(); it != tilef._masks.end(); ++it)
    {
	  // When displaying Plate Carre, Heights have to be converted from meters to degrees.
	  // This is also true for mask feature
	  // TODO: adjust this calculation based on the actual EllipsoidModel.
	  float scale = scaleHeight;
	  if (_masterLocator->getCoordinateSystemType() == osgEarth::GeoLocator::GEOGRAPHIC)
		scale = scaleHeight / 111319.0f;

	  // TODO: Get the map SRS if possible instead of masterLocator's one
	  osg::Vec3dArray* boundary = (*it)->getOrCreateBoundary(scale, _masterLocator->getDataExtent().getSRS());

      if ( boundary )
      {
          osg::Vec3d min, max;
          min = max = boundary->front();

          for (osg::Vec3dArray::iterator it = boundary->begin(); it != boundary->end(); ++it)
          {
            if (it->x() < min.x())
              min.x() = it->x();

            if (it->y() < min.y())
              min.y() = it->y();

            if (it->x() > max.x())
              max.x() = it->x();

            if (it->y() > max.y())
              max.y() = it->y();
          }

          osg::Vec3d min_ndc, max_ndc;
          geoLocator->convertModelToLocal(min, min_ndc);
          geoLocator->convertModelToLocal(max, max_ndc);

          bool x_match = ((min_ndc.x() >= 0.0 && max_ndc.x() <= 1.0) ||
                          (min_ndc.x() <= 0.0 && max_ndc.x() > 0.0) ||
                          (min_ndc.x() < 1.0 && max_ndc.x() >= 1.0));

          bool y_match = ((min_ndc.y() >= 0.0 && max_ndc.y() <= 1.0) ||
                          (min_ndc.y() <= 0.0 && max_ndc.y() > 0.0) ||
                          (min_ndc.y() < 1.0 && max_ndc.y() >= 1.0));

          if (x_match && y_match)
          {
            osg::Geometry* mask_geom = new osg::Geometry();
            mask_geom->setThreadSafeRefUnref(true);
            mask_geom->setDataVariance( osg::Object::DYNAMIC );
            mask_geom->setUseDisplayList(false);
            mask_geom->setUseVertexBufferObjects(true);
            //mask_geom->getOrCreateStateSet()->setAttribute(new osg::Point( 5.0f ), osg::StateAttribute::ON);
            surfaceGeode->addDrawable(mask_geom);

            masks.push_back(MaskRecord(boundary, min_ndc, max_ndc, mask_geom));
          }
       }
    }

    osg::Geometry* stitching_skirts = 0L;
    osg::Vec3Array* ss_verts = 0L;
    if (masks.size() > 0)
    {
      stitching_skirts = new osg::Geometry();
      stitching_skirts->setThreadSafeRefUnref(true);
      stitching_skirts->setDataVariance( osg::Object::DYNAMIC );
      stitching_skirts->setUseDisplayList(false);
      stitching_skirts->setUseVertexBufferObjects(true);
      //stitching_skirts->getOrCreateStateSet()->setAttribute(new osg::Point( 5.0f ), osg::StateAttribute::ON);
      surfaceGeode->addDrawable( stitching_skirts);

      ss_verts = new osg::Vec3Array();
      stitching_skirts->setVertexArray(ss_verts);

      if ( ss_verts->getVertexBufferObject() )
          ss_verts->getVertexBufferObject()->setUsage(GL_STATIC_DRAW_ARB);
    }


    unsigned int numRows = 20;
    unsigned int numColumns = 20;
    
    if (elevationLayer)
    {
        numColumns = elevationLayer->getNumColumns();
        numRows = elevationLayer->getNumRows();
    }
    
    double i_sampleFactor, j_sampleFactor;
    calculateSampling( numColumns, numRows, i_sampleFactor, j_sampleFactor );
    
    float skirtHeight = 0.0f;
    osgTerrain::HeightFieldLayer* hfl = dynamic_cast<osgTerrain::HeightFieldLayer*>(elevationLayer);
    if (hfl && hfl->getHeightField()) 
    {
        skirtHeight = hfl->getHeightField()->getSkirtHeight();
    }
    
    bool createSkirt = skirtHeight != 0.0f;
  
    unsigned int numVerticesInSurface = numColumns*numRows;
    unsigned int numVerticesInSkirt = createSkirt ? (2 * (numColumns*2 + numRows*2 - 4)) : 0;
    //unsigned int numVertices = numVerticesInBody+numVerticesInSkirt;

    // allocate and assign vertices
    osg::ref_ptr<osg::Vec3Array> surfaceVerts = new osg::Vec3Array;
    surfaceVerts->reserve( numVerticesInSurface );
    surface->setVertexArray( surfaceVerts.get() );

    if ( surfaceVerts->getVertexBufferObject() )
        surfaceVerts->getVertexBufferObject()->setUsage(GL_STATIC_DRAW_ARB);

    // allocate and assign normals
    osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array();
    normals->reserve(numVerticesInSurface);
    surface->setNormalArray(normals.get());
    surface->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);

    // skirt texture coordinates, if applicable:
    osg::Vec2Array* unifiedSkirtTexCoords = 0L;

    // stitching skirt texture coordinates, if applicable:
    osg::Vec2Array* unifiedStitchSkirtTexCoords = 0L;

    // allocate and assign texture coordinates
    osg::Vec2Array* unifiedSurfaceTexCoords = 0L;

    //int numColorLayers = _tile->getNumColorLayers();
    RenderLayerVector renderLayers;

    if ( _texCompositor->requiresUnitTextureSpace() )
    {
        // for a unified unit texture space, just make a single texture coordinate array.
        unifiedSurfaceTexCoords = new osg::Vec2Array();
        unifiedSurfaceTexCoords->reserve( numVerticesInSurface );
        surface->setTexCoordArray( 0, unifiedSurfaceTexCoords );
        if (createSkirt)
        {
            unifiedSkirtTexCoords = new osg::Vec2Array();
            unifiedSkirtTexCoords->reserve( numVerticesInSkirt );        
            skirt->setTexCoordArray( 0, unifiedSkirtTexCoords );
        }

        if (masks.size() > 0)
        {
            unifiedStitchSkirtTexCoords = new osg::Vec2Array();
            //unifiedStitchSkirtTexCoords->reserve( ? );        
            stitching_skirts->setTexCoordArray( 0, unifiedStitchSkirtTexCoords );
        }
    }

    else // if ( !_texCompositor->requiresUnitTextureSpace() )
    {
        LocatorToTexCoordTable locatorToTexCoordTable;
        renderLayers.reserve( tilef._colorLayers.size() );

        // build a list of "render layers", in slot order, sharing texture coordinate
        // arrays wherever possible.
        for( ColorLayersByUID::const_iterator i = tilef._colorLayers.begin(); i != tilef._colorLayers.end(); ++i )
        {
            const CustomColorLayer& colorLayer = i->second;
            RenderLayer r;
            r._layer = colorLayer;

            const GeoLocator* locator = dynamic_cast<const GeoLocator*>( r._layer.getLocator() );
            if ( locator )
            {
                r._texCoords = locatorToTexCoordTable.find( locator );
                if ( !r._texCoords.valid() )
                {
                    r._texCoords = new osg::Vec2Array();
                    r._texCoords->reserve( numVerticesInSurface );
                    r._ownsTexCoords = true;
                    locatorToTexCoordTable.push_back( LocatorTexCoordPair(locator, r._texCoords.get()) );
                }

                r._skirtTexCoords = new osg::Vec2Array();
                r._skirtTexCoords->reserve( numVerticesInSkirt );

                if ( masks.size() > 0 )
                {
                    r._stitchTexCoords = new osg::Vec2Array();
                    r._stitchSkirtTexCoords = new osg::Vec2Array();
                }

                r._locator = locator;
                if ( locator->getCoordinateSystemType() == osgTerrain::Locator::GEOCENTRIC )
                {
                    const GeoLocator* geo = dynamic_cast<const GeoLocator*>(locator);
                    if ( geo )
                        r._locator = geo->getGeographicFromGeocentric();
                }

                _texCompositor->assignTexCoordArray( surface, colorLayer.getUID(), r._texCoords.get() );
                _texCompositor->assignTexCoordArray( skirt, colorLayer.getUID(), r._skirtTexCoords.get() );

                for (MaskRecordVector::iterator mr = masks.begin(); mr != masks.end(); ++mr)
                    _texCompositor->assignTexCoordArray( (*mr)._geom, colorLayer.getUID(), r._stitchTexCoords.get() );

                if (stitching_skirts)
                    _texCompositor->assignTexCoordArray( stitching_skirts, colorLayer.getUID(), r._stitchSkirtTexCoords.get() );

                //surface->setTexCoordArray( renderLayers.size(), r._texCoords );
                renderLayers.push_back( r );
            }
            else
            {
                OE_WARN << LC << "Found a Locator, but it wasn't a GeoLocator." << std::endl;
            }
        }
    }

    osg::ref_ptr<osg::FloatArray> elevations = new osg::FloatArray;
    if (elevations.valid()) elevations->reserve(numVerticesInSurface);        

    // allocate and assign color
    osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array(1);
    (*colors)[0].set(1.0f,1.0f,1.0f,1.0f);
    
    surface->setColorArray(colors.get());
    surface->setColorBinding(osg::Geometry::BIND_OVERALL);

    typedef std::vector<int> Indices;
    Indices indices(numVerticesInSurface, -1);    

    // populate vertex and tex coord arrays    
    unsigned int i, j; //, k=0;
    for(j=0; j<numRows; ++j)
    {
        for(i=0; i<numColumns; ++i) // ++k)
        {
            unsigned int iv = j*numColumns + i;
            osg::Vec3d ndc( ((double)i)/(double)(numColumns-1), ((double)j)/(double)(numRows-1), 0.0);
     
            bool validValue = true;
            
            unsigned int i_equiv = i_sampleFactor==1.0 ? i : (unsigned int) (double(i)*i_sampleFactor);

            unsigned int j_equiv = j_sampleFactor==1.0 ? j : (unsigned int) (double(j)*j_sampleFactor);

            if (elevationLayer)
            {
                float value = 0.0f;
                validValue = elevationLayer->getValidValue(i_equiv,j_equiv, value);
                ndc.z() = value*scaleHeight;
            }

            //Invalidate if point falls within mask bounding box
            if (validValue && masks.size() > 0)
            {
              for (MaskRecordVector::iterator mr = masks.begin(); mr != masks.end(); ++mr)
              {
                if(ndc.x() >= (*mr)._ndcMin.x() && ndc.x() <= (*mr)._ndcMax.x() &&
                   ndc.y() >= (*mr)._ndcMin.y() && ndc.y() <= (*mr)._ndcMax.y())
                {
                  validValue = false;
                  indices[iv] = -2;

                  (*mr)._internal->push_back(ndc);

                  break;
                }
              }
            }
            
            if (validValue)
            {
                //indices[iv] = k;
                indices[iv] = surfaceVerts->size();
            
                osg::Vec3d model;
                _masterLocator->convertLocalToModel(ndc, model);

                //(*surfaceVerts)[k] = model - centerModel;
                (*surfaceVerts).push_back(model - _centerModel);

                if ( _texCompositor->requiresUnitTextureSpace() )
                {
                    // the unified unit texture space requires a single, untransformed unit coord [0..1]
                    (*unifiedSurfaceTexCoords).push_back( osg::Vec2( ndc.x(), ndc.y() ) );
                }
                else
                {
                    // the separate texture space requires separate transformed texcoords for each layer.
                    for( RenderLayerVector::const_iterator r = renderLayers.begin(); r != renderLayers.end(); ++r )
                    {
                        if ( r->_ownsTexCoords )
                        {
                            //if ( r->_locator.get() != _masterLocator.get() )
                            //if ( r->_locator->isEr->_locator != masterTextureLocator _masterLocator.get() )
                            if ( !r->_locator->isEquivalentTo( *masterTextureLocator.get() ) )
                            {
                                osg::Vec3d color_ndc;
                                osgTerrain::Locator::convertLocalCoordBetween( *masterTextureLocator.get(), ndc, *r->_locator.get(), color_ndc );
                                r->_texCoords->push_back( osg::Vec2( color_ndc.x(), color_ndc.y() ) );
                            }
                            else
                            {
                                r->_texCoords->push_back( osg::Vec2( ndc.x(), ndc.y() ) );
                            }
                        }
                    }
                }

                if (elevations.valid())
                {
                    (*elevations).push_back(ndc.z());
                }

                // compute the local normal
                osg::Vec3d ndc_one = ndc; ndc_one.z() += 1.0;
                osg::Vec3d model_one;
                _masterLocator->convertLocalToModel(ndc_one, model_one);
                model_one = model_one - model;
                model_one.normalize();    

                //(*normals)[k] = model_one;
                (*normals).push_back(model_one);
            }
        }
    }


    for (MaskRecordVector::iterator mr = masks.begin(); mr != masks.end(); ++mr)
    {
      int min_i = (int)floor((*mr)._ndcMin.x() * (double)(numColumns-1));
      if (min_i < 0) min_i = 0;
      if (min_i >= (int)numColumns) min_i = numColumns - 1;

      int min_j = (int)floor((*mr)._ndcMin.y() * (double)(numRows-1));
      if (min_j < 0) min_j = 0;
      if (min_j >= (int)numColumns) min_j = numColumns - 1;

      int max_i = (int)ceil((*mr)._ndcMax.x() * (double)(numColumns-1));
      if (max_i < 0) max_i = 0;
      if (max_i >= (int)numColumns) max_i = numColumns - 1;

      int max_j = (int)ceil((*mr)._ndcMax.y() * (double)(numRows-1));
      if (max_j < 0) max_j = 0;
      if (max_j >= (int)numColumns) max_j = numColumns - 1;

      if (min_i >= 0 && max_i >= 0 && min_j >= 0 && max_j >= 0)
      {
        int num_i = max_i - min_i + 1;
        int num_j = max_j - min_j + 1;

        osg::ref_ptr<osgEarth::Symbology::Polygon> maskSkirtPoly = new osgEarth::Symbology::Polygon();
        maskSkirtPoly->resize(num_i * 2 + num_j * 2 - 4);
        for (int i = 0; i < num_i; i++)
        {
          //int index = indices[min_j*numColumns + i + min_i];
          {
            osg::Vec3d ndc( ((double)(i + min_i))/(double)(numColumns-1), ((double)min_j)/(double)(numRows-1), 0.0);

            if (elevationLayer)
            {
              unsigned int i_equiv = i_sampleFactor==1.0 ? i + min_i : (unsigned int) (double(i + min_i)*i_sampleFactor);
              unsigned int j_equiv = j_sampleFactor==1.0 ? min_j : (unsigned int) (double(min_j)*j_sampleFactor);

              float value = 0.0f;
              if (elevationLayer->getValidValue(i_equiv,j_equiv, value))
                ndc.z() = value*scaleHeight;
            }

            (*maskSkirtPoly)[i] = ndc;
          }

          //index = indices[max_j*numColumns + i + min_i];
          {
            osg::Vec3d ndc( ((double)(i + min_i))/(double)(numColumns-1), ((double)max_j)/(double)(numRows-1), 0.0);

            if (elevationLayer)
            {
              unsigned int i_equiv = i_sampleFactor==1.0 ? i + min_i : (unsigned int) (double(i + min_i)*i_sampleFactor);
              unsigned int j_equiv = j_sampleFactor==1.0 ? max_j : (unsigned int) (double(max_j)*j_sampleFactor);

              float value = 0.0f;
              if (elevationLayer->getValidValue(i_equiv,j_equiv, value))
                ndc.z() = value*scaleHeight;
            }

            (*maskSkirtPoly)[i + (2 * num_i + num_j - 3) - 2 * i] = ndc;
          }
        }
        for (int j = 0; j < num_j - 2; j++)
        {
          //int index = indices[(min_j + j + 1)*numColumns + max_i];
          {
            osg::Vec3d ndc( ((double)max_i)/(double)(numColumns-1), ((double)(min_j + j + 1))/(double)(numRows-1), 0.0);

            if (elevationLayer)
            {
              unsigned int i_equiv = i_sampleFactor==1.0 ? max_i : (unsigned int) (double(max_i)*i_sampleFactor);
              unsigned int j_equiv = j_sampleFactor==1.0 ? min_j + j + 1 : (unsigned int) (double(min_j + j + 1)*j_sampleFactor);

              float value = 0.0f;
              if (elevationLayer->getValidValue(i_equiv,j_equiv, value))
                ndc.z() = value*scaleHeight;
            }

            (*maskSkirtPoly)[j + num_i] = ndc;
          }

          //index = indices[(min_j + j + 1)*numColumns + min_i];
          {
            osg::Vec3d ndc( ((double)min_i)/(double)(numColumns-1), ((double)(min_j + j + 1))/(double)(numRows-1), 0.0);

            if (elevationLayer)
            {
              unsigned int i_equiv = i_sampleFactor==1.0 ? min_i : (unsigned int) (double(min_i)*i_sampleFactor);
              unsigned int j_equiv = j_sampleFactor==1.0 ? min_j + j + 1 : (unsigned int) (double(min_j + j + 1)*j_sampleFactor);

              float value = 0.0f;
              if (elevationLayer->getValidValue(i_equiv,j_equiv, value))
                ndc.z() = value*scaleHeight;
            }

            (*maskSkirtPoly)[j + (2 * num_i + 2 * num_j - 5) - 2 * j] = ndc;
          }
        }

        //Create local polygon representing mask
        osg::ref_ptr<osgEarth::Symbology::Polygon> maskPoly = new osgEarth::Symbology::Polygon();
        for (osg::Vec3dArray::iterator it = (*mr)._boundary->begin(); it != (*mr)._boundary->end(); ++it)
        {
          osg::Vec3d local;
          geoLocator->convertModelToLocal(*it, local);
          maskPoly->push_back(local);
        }

#if 0

        //Do a diff on the polygons to get the actual mask skirt
        osg::ref_ptr<osgEarth::Symbology::Geometry> outPoly;
        maskSkirtPoly->difference(maskPoly.get(), outPoly);

        osg::Vec3Array* outVerts = new osg::Vec3Array();
        osg::Geometry* stitch_geom = (*mr)._geom;

        stitch_geom->setVertexArray(outVerts);

        bool multiParent = false;
        if (outPoly.valid())
          multiParent = outPoly->getType() == osgEarth::Symbology::Geometry::TYPE_MULTI;
        
        std::vector<int> skirtIndices;

        osgEarth::Symbology::GeometryIterator i( outPoly.get(), false );
        while( i.hasMore() )
        {
          osgEarth::Symbology::Geometry* part = i.next();
          if (!part)
            continue;

          if (part->getType() == osgEarth::Symbology::Geometry::TYPE_POLYGON)
          {
            osg::Vec3Array* partVerts = part->toVec3Array();
            outVerts->insert(outVerts->end(), partVerts->begin(), partVerts->end());
            stitch_geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POLYGON, outVerts->size() - partVerts->size(), partVerts->size()));
            skirtIndices.push_back(outVerts->size());

            if (!multiParent)
            {
              osg::ref_ptr<osgEarth::Symbology::Polygon> holePoly = static_cast<osgEarth::Symbology::Polygon*>(outPoly.get());
              if (holePoly)
              {
                osgEarth::Symbology::RingCollection holes = holePoly->getHoles();
                
                for (osgEarth::Symbology::RingCollection::iterator hit = holes.begin(); hit != holes.end(); ++hit)
                {
                  (*hit)->rewind(osgEarth::Symbology::Ring::ORIENTATION_CCW);
                  outVerts->insert(outVerts->end(), (*hit)->begin(), (*hit)->end());
                  stitch_geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POLYGON, outVerts->size() - (*hit)->size(), (*hit)->size()));
                  skirtIndices.push_back(outVerts->size());
                }
              }
            }
          }
        }

        if (stitch_geom->getNumPrimitiveSets() > 0)
        {
          // Tessellate mask skirt
          osg::ref_ptr<osgUtil::Tessellator> tscx=new osgUtil::Tessellator;
          tscx->setTessellationType(osgUtil::Tessellator::TESS_TYPE_GEOMETRY);
          tscx->setBoundaryOnly(false);
          tscx->setWindingType(osgUtil::Tessellator::TESS_WINDING_ODD);
          tscx->retessellatePolygons(*stitch_geom);

          // Assign normals to the stitching polygon: -gw
          osg::Vec3Array* sgVerts = dynamic_cast<osg::Vec3Array*>(stitch_geom->getVertexArray());
          osg::Vec3Array* sgNormals = new osg::Vec3Array(sgVerts->size());
          stitch_geom->setNormalArray( sgNormals );
          stitch_geom->setNormalBinding( osg::Geometry::BIND_PER_VERTEX );
          
          // calculate the normal and convert to model space.
          for( unsigned v=0; v<sgVerts->size(); ++v )
          {
              const osg::Vec3& vert = (*sgVerts)[v];
              osg::Vec3d local_one(vert);
              osg::Vec3d model;
              _masterLocator->convertLocalToModel( local_one, model );
              local_one.z() += 1.0;
              osg::Vec3d model_one;
              _masterLocator->convertLocalToModel( local_one, model_one );
              model_one = model_one - model;
              model_one.normalize();
              (*sgNormals)[v] = model_one;
          }

          //Initialize tex coords
          osg::Vec2Array* unifiedStitchTexCoords = 0L;
          if (_texCompositor->requiresUnitTextureSpace())
          {
            unifiedStitchTexCoords = new osg::Vec2Array();
            unifiedStitchTexCoords->reserve(outVerts->size());
            stitch_geom->setTexCoordArray(0, unifiedStitchTexCoords);
          }
          else if ( renderLayers.size() > 0 )
          {
            for (unsigned int i = 0; i < renderLayers.size(); ++i)
            {
              renderLayers[i]._stitchTexCoords->reserve(outVerts->size());
            }
          }


          //Retrieve z values for mask skirt verts
          std::vector<int> isZSet;
          for (osg::Vec3Array::iterator it = outVerts->begin(); it != outVerts->end(); ++it)
          {
            int zSet = 0;

            //Look for verts that belong to the original mask skirt polygon
            for (osgEarth::Symbology::Polygon::iterator mit = maskSkirtPoly->begin(); mit != maskSkirtPoly->end(); ++mit)
            {
              if (osg::absolute((*mit).x() - (*it).x()) < MATCH_TOLERANCE && osg::absolute((*mit).y() - (*it).y()) < MATCH_TOLERANCE)
              {
                (*it).z() = (*mit).z();
                zSet += 1;
                break;
              }
            }

            //Look for verts that belong to the mask polygon
            for (osgEarth::Symbology::Polygon::iterator mit = maskPoly->begin(); mit != maskPoly->end(); ++mit)
            {
              if (osg::absolute((*mit).x() - (*it).x()) < MATCH_TOLERANCE && osg::absolute((*mit).y() - (*it).y()) < MATCH_TOLERANCE)
              {
                (*it).z() = (*mit).z();
                zSet += 2;
                break;
              }
            }

            isZSet.push_back(zSet);
          }

          //Any mask skirt verts that are still unset are newly created verts where the skirt
          //meets the mask. Find the mask segment the point lies along and calculate the
          //appropriate z value for the point.
          //
          //Now that all the z values are set, convert each vert into model coords.
          //
          //Also, while we are iterating through the verts, set up tex coords.
          int count = 0;
          for (osg::Vec3Array::iterator it = outVerts->begin(); it != outVerts->end(); ++it)
          {
            //If the z-value was set from a mask vertex there is no need to change it.  If
            //it was set from a vertex from the stitching polygon it may need overriden if
            //the vertex lies along a mask edge.  Or if it is unset, it will need to be set.
            if (isZSet[count] < 2)
            {
              osg::Vec3d p2 = *it;
              double closestZ = 0.0;
              double closestRatio = DBL_MAX;
              for (osgEarth::Symbology::Polygon::iterator mit = maskPoly->begin(); mit != maskPoly->end(); ++mit)
              {
                osg::Vec3d p1 = *mit;
                osgEarth::Symbology::Polygon::iterator mitEnd = maskPoly->end();
                osg::Vec3d p3 = (mit == (--mitEnd)) ? maskPoly->front() : (*(mit + 1));

                //Truncated vales to compensate for accuracy issues
                double p1x = ((int)(p1.x() * 1000000)) / 1000000.0L;
                double p3x = ((int)(p3.x() * 1000000)) / 1000000.0L;
                double p2x = ((int)(p2.x() * 1000000)) / 1000000.0L;

                double p1y = ((int)(p1.y() * 1000000)) / 1000000.0L;
                double p3y = ((int)(p3.y() * 1000000)) / 1000000.0L;
                double p2y = ((int)(p2.y() * 1000000)) / 1000000.0L;

                if ((p1x < p3x ? p2x >= p1x && p2x <= p3x : p2x >= p3x && p2x <= p1x) &&
                    (p1y < p3y ? p2y >= p1y && p2y <= p3y : p2y >= p3y && p2y <= p1y))
                {
                  double l1 =(osg::Vec2d(p2.x(), p2.y()) - osg::Vec2d(p1.x(), p1.y())).length();
                  double lt = (osg::Vec2d(p3.x(), p3.y()) - osg::Vec2d(p1.x(), p1.y())).length();
                  double zmag = p3.z() - p1.z();

                  double foundZ = (l1 / lt) * zmag + p1.z();

                  double mRatio = 1.0;
                  if (osg::absolute(p1x - p3x) < MATCH_TOLERANCE)
                  {
                    if (osg::absolute(p1x-p2x) < MATCH_TOLERANCE)
                      mRatio = 0.0;
                  }
                  else
                  {
                    double m1 = p1x == p2x ? 0.0 : (p2y - p1y) / (p2x - p1x);
                    double m2 = p1x == p3x ? 0.0 : (p3y - p1y) / (p3x - p1x);
                    mRatio = m2 == 0.0 ? m1 : osg::absolute(1.0L - m1 / m2);
                  }

                  if (mRatio < 0.01)
                  {
                    (*it).z() = foundZ;
                    isZSet[count] = 2;
                    break;
                  }
                  else if (mRatio < closestRatio)
                  {
                    closestRatio = mRatio;
                    closestZ = foundZ;
                  }
                }
              }

              if (!isZSet[count] && closestRatio < DBL_MAX)
              {
                (*it).z() = closestZ;
                isZSet[count] = 2;
              }
            }

            if (!isZSet[count])
              OE_WARN << LC << "Z-value not set for stitching polygon vertex" << std::endl;

            count++;

            //Convert to model coords
            osg::Vec3d model;
            _masterLocator->convertLocalToModel(*it, model);
            model = model - _centerModel;
            (*it).set(model.x(), model.y(), model.z());

            //Setup tex coords
            osg::Vec3d ndc;
            _masterLocator->convertModelToLocal(*it + _centerModel, ndc);

            if (_texCompositor->requiresUnitTextureSpace())
            {
              unifiedStitchTexCoords->push_back(osg::Vec2(ndc.x(), ndc.y()));
            }
            else if (renderLayers.size() > 0)
            {
              for (unsigned int i = 0; i < renderLayers.size(); ++i)
              {
                if (!renderLayers[i]._locator->isEquivalentTo(*masterTextureLocator.get()))
                {
                  osg::Vec3d color_ndc;
                  osgTerrain::Locator::convertLocalCoordBetween(*masterTextureLocator.get(), ndc, *renderLayers[i]._locator.get(), color_ndc);
                  renderLayers[i]._stitchTexCoords->push_back(osg::Vec2(color_ndc.x(), color_ndc.y()));
                }
                else
                {
                  renderLayers[i]._stitchTexCoords->push_back(osg::Vec2(ndc.x(), ndc.y()));
                }
              }
            }
          }
 
          //Create stitching skirts
          if (createSkirt && skirtIndices.size() > 0)
          {
            ss_verts->reserve(ss_verts->size() + outVerts->size() * 4 + skirtIndices.size() * 2);

            //Add a primative set for each continuous skirt strip
            for (int p=0; p < skirtIndices.size(); p++)
            {
              int cursor = ss_verts->size();

              int outStart = p == 0 ? 0 : skirtIndices[p-1];
              for (int i=outStart; i < skirtIndices[p]; i++)
              {
                ss_verts->push_back((*outVerts)[i]);
                ss_verts->push_back((*outVerts)[i] - (*sgNormals)[i] * skirtHeight);

                if ( _texCompositor->requiresUnitTextureSpace() )
                {
                    unifiedStitchSkirtTexCoords->push_back( (*unifiedStitchTexCoords)[i] );
                    unifiedStitchSkirtTexCoords->push_back( (*unifiedStitchTexCoords)[i] );
                }
                else if ( renderLayers.size() > 0 )
                {
                    for (unsigned int r = 0; r < renderLayers.size(); ++r)
                    {
                        const osg::Vec2& tc = (*renderLayers[r]._stitchTexCoords.get())[i];
                        renderLayers[r]._stitchSkirtTexCoords->push_back( tc );
                        renderLayers[r]._stitchSkirtTexCoords->push_back( tc );
                    }
                }
              }

              //Add the first vert again to complete the loop
              ss_verts->push_back((*outVerts)[outStart]);
              ss_verts->push_back((*outVerts)[outStart] - (*sgNormals)[outStart] * skirtHeight);

              if ( _texCompositor->requiresUnitTextureSpace() )
              {
                  unifiedStitchSkirtTexCoords->push_back( (*unifiedStitchTexCoords)[outStart] );
                  unifiedStitchSkirtTexCoords->push_back( (*unifiedStitchTexCoords)[outStart] );
              }
              else if ( renderLayers.size() > 0 )
              {
                  for (unsigned int r = 0; r < renderLayers.size(); ++r)
                  {
                      const osg::Vec2& tc = (*renderLayers[r]._stitchTexCoords.get())[outStart];
                      renderLayers[r]._stitchSkirtTexCoords->push_back( tc );
                      renderLayers[r]._stitchSkirtTexCoords->push_back( tc );
                  }
              }

              //Now go back the opposite direction to create a skirt facing the other direction
              for (int i=skirtIndices[p] - 1; i >= outStart; i--)
              {
                ss_verts->push_back((*outVerts)[i]);
                ss_verts->push_back((*outVerts)[i] - (*sgNormals)[i] * skirtHeight);

                if ( _texCompositor->requiresUnitTextureSpace() )
                {
                    unifiedStitchSkirtTexCoords->push_back( (*unifiedStitchTexCoords)[i] );
                    unifiedStitchSkirtTexCoords->push_back( (*unifiedStitchTexCoords)[i] );
                }
                else if ( renderLayers.size() > 0 )
                {
                    for (unsigned int r = 0; r < renderLayers.size(); ++r)
                    {
                        const osg::Vec2& tc = (*renderLayers[r]._stitchTexCoords.get())[i];
                        renderLayers[r]._stitchSkirtTexCoords->push_back( tc );
                        renderLayers[r]._stitchSkirtTexCoords->push_back( tc );
                    }
                }
              }

              stitching_skirts->addPrimitiveSet(new osg::DrawArrays( GL_TRIANGLE_STRIP, cursor, ss_verts->size() - cursor));
            }
          }
        }

#else
        // Use delaunay triangulation for stitching as an alternative to the method above

        // Add the outter stitching bounds to the collection of vertices to be used for triangulation
        (*mr)._internal->insert((*mr)._internal->end(), maskSkirtPoly->begin(), maskSkirtPoly->end());

        osg::ref_ptr<osgUtil::DelaunayTriangulator> trig=new osgUtil::DelaunayTriangulator();
        trig->setInputPointArray((*mr)._internal.get());


        // Add mask bounds as a triangulation constraint
        osg::ref_ptr<osgUtil::DelaunayConstraint> dc=new osgUtil::DelaunayConstraint;

        osg::Vec3Array* maskConstraint = new osg::Vec3Array();
        dc->setVertexArray(maskConstraint);

        if ( maskConstraint->getVertexBufferObject() )
            maskConstraint->getVertexBufferObject()->setUsage(GL_STATIC_DRAW_ARB);

        //Crop the mask to the stitching poly (for case where mask crosses tile edge)
        osg::ref_ptr<osgEarth::Symbology::Geometry> maskCrop;
        maskPoly->crop(maskSkirtPoly.get(), maskCrop);
        
        osgEarth::Symbology::GeometryIterator i( maskCrop.get(), false );
        while( i.hasMore() )
        {
          osgEarth::Symbology::Geometry* part = i.next();
          if (!part)
            continue;

          if (part->getType() == osgEarth::Symbology::Geometry::TYPE_POLYGON)
          {
            osg::Vec3Array* partVerts = part->toVec3Array();
            maskConstraint->insert(maskConstraint->end(), partVerts->begin(), partVerts->end());
            dc->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_LOOP, maskConstraint->size() - partVerts->size(), partVerts->size()));
          }
        }

        // Cropping strips z-values so need reassign
        std::vector<int> isZSet;
        for (osg::Vec3Array::iterator it = maskConstraint->begin(); it != maskConstraint->end(); ++it)
        {
          int zSet = 0;

          //Look for verts that belong to the original mask skirt polygon
          for (osgEarth::Symbology::Polygon::iterator mit = maskSkirtPoly->begin(); mit != maskSkirtPoly->end(); ++mit)
          {
            if (osg::absolute((*mit).x() - (*it).x()) < MATCH_TOLERANCE && osg::absolute((*mit).y() - (*it).y()) < MATCH_TOLERANCE)
            {
              (*it).z() = (*mit).z();
              zSet += 1;
              break;
            }
          }

          //Look for verts that belong to the mask polygon
          for (osgEarth::Symbology::Polygon::iterator mit = maskPoly->begin(); mit != maskPoly->end(); ++mit)
          {
            if (osg::absolute((*mit).x() - (*it).x()) < MATCH_TOLERANCE && osg::absolute((*mit).y() - (*it).y()) < MATCH_TOLERANCE)
            {
              (*it).z() = (*mit).z();
              zSet += 2;
              break;
            }
          }

          isZSet.push_back(zSet);
        }

        //Any mask skirt verts that are still unset are newly created verts where the skirt
        //meets the mask. Find the mask segment the point lies along and calculate the
        //appropriate z value for the point.
        int count = 0;
        for (osg::Vec3Array::iterator it = maskConstraint->begin(); it != maskConstraint->end(); ++it)
        {
          //If the z-value was set from a mask vertex there is no need to change it.  If
          //it was set from a vertex from the stitching polygon it may need overriden if
          //the vertex lies along a mask edge.  Or if it is unset, it will need to be set.
          //if (isZSet[count] < 2)
          if (!isZSet[count])
          {
            osg::Vec3d p2 = *it;
            double closestZ = 0.0;
            double closestRatio = DBL_MAX;
            for (osgEarth::Symbology::Polygon::iterator mit = maskPoly->begin(); mit != maskPoly->end(); ++mit)
            {
              osg::Vec3d p1 = *mit;
              osg::Vec3d p3 = mit == --maskPoly->end() ? maskPoly->front() : (*(mit + 1));

              //Truncated vales to compensate for accuracy issues
              double p1x = ((int)(p1.x() * 1000000)) / 1000000.0L;
              double p3x = ((int)(p3.x() * 1000000)) / 1000000.0L;
              double p2x = ((int)(p2.x() * 1000000)) / 1000000.0L;

              double p1y = ((int)(p1.y() * 1000000)) / 1000000.0L;
              double p3y = ((int)(p3.y() * 1000000)) / 1000000.0L;
              double p2y = ((int)(p2.y() * 1000000)) / 1000000.0L;

              if ((p1x < p3x ? p2x >= p1x && p2x <= p3x : p2x >= p3x && p2x <= p1x) &&
                  (p1y < p3y ? p2y >= p1y && p2y <= p3y : p2y >= p3y && p2y <= p1y))
              {
                double l1 =(osg::Vec2d(p2.x(), p2.y()) - osg::Vec2d(p1.x(), p1.y())).length();
                double lt = (osg::Vec2d(p3.x(), p3.y()) - osg::Vec2d(p1.x(), p1.y())).length();
                double zmag = p3.z() - p1.z();

                double foundZ = (l1 / lt) * zmag + p1.z();

                double mRatio = 1.0;
                if (osg::absolute(p1x - p3x) < MATCH_TOLERANCE)
                {
                  if (osg::absolute(p1x-p2x) < MATCH_TOLERANCE)
                    mRatio = 0.0;
                }
                else
                {
                  double m1 = p1x == p2x ? 0.0 : (p2y - p1y) / (p2x - p1x);
                  double m2 = p1x == p3x ? 0.0 : (p3y - p1y) / (p3x - p1x);
                  mRatio = m2 == 0.0 ? m1 : osg::absolute(1.0L - m1 / m2);
                }

                if (mRatio < 0.01)
                {
                  (*it).z() = foundZ;
                  isZSet[count] = 2;
                  break;
                }
                else if (mRatio < closestRatio)
                {
                  closestRatio = mRatio;
                  closestZ = foundZ;
                }
              }
            }

            if (!isZSet[count] && closestRatio < DBL_MAX)
            {
              (*it).z() = closestZ;
              isZSet[count] = 2;
            }
          }

          if (!isZSet[count])
            OE_WARN << LC << "Z-value not set for mask constraint vertex" << std::endl;

          count++;
        }

        trig->addInputConstraint(dc.get());


        // Create array to hold vertex normals
        osg::Vec3Array *norms=new osg::Vec3Array;
        trig->setOutputNormalArray(norms);
        

        // Triangulate vertices and remove triangles that lie within the contraint loop
        trig->triangulate();
        trig->removeInternalTriangles(dc.get());


        // Set up new arrays to hold final vertices and normals
        osg::Geometry* stitch_geom = (*mr)._geom;
        osg::Vec3Array* stitch_verts = new osg::Vec3Array();
        stitch_verts->reserve(trig->getInputPointArray()->size());
        stitch_geom->setVertexArray(stitch_verts);
        if ( stitch_verts->getVertexBufferObject() )
            stitch_verts->getVertexBufferObject()->setUsage(GL_STATIC_DRAW_ARB);
        osg::Vec3Array* stitch_norms = new osg::Vec3Array(trig->getInputPointArray()->size());
        stitch_geom->setNormalArray( stitch_norms );
        stitch_geom->setNormalBinding( osg::Geometry::BIND_PER_VERTEX );


        //Initialize tex coords
        osg::Vec2Array* unifiedStitchTexCoords = 0L;
        if (_texCompositor->requiresUnitTextureSpace())
        {
          unifiedStitchTexCoords = new osg::Vec2Array();
          unifiedStitchTexCoords->reserve(trig->getInputPointArray()->size());
          stitch_geom->setTexCoordArray(0, unifiedStitchTexCoords);
        }
        else if ( renderLayers.size() > 0 )
        {
          for (unsigned int i = 0; i < renderLayers.size(); ++i)
          {
            renderLayers[i]._stitchTexCoords->reserve(trig->getInputPointArray()->size());
          }
        }


        // Iterate through point to convert to model coords, calculate normals, and set up tex coords
        int norm_i = -1;
        for (osg::Vec3Array::iterator it = trig->getInputPointArray()->begin(); it != trig->getInputPointArray()->end(); ++it)
        {
          // get model coords
          osg::Vec3d model;
          _masterLocator->convertLocalToModel(*it, model);
          model = model - _centerModel;

          stitch_verts->push_back(model);

          // calc normals
          osg::Vec3d local_one(*it);
          local_one.z() += 1.0;
          osg::Vec3d model_one;
          _masterLocator->convertLocalToModel( local_one, model_one );
          model_one = model_one - model;
          model_one.normalize();
          (*stitch_norms)[++norm_i] = model_one;

          // set up text coords
          if (_texCompositor->requiresUnitTextureSpace())
          {
            unifiedStitchTexCoords->push_back(osg::Vec2((*it).x(), (*it).y()));
          }
          else if (renderLayers.size() > 0)
          {
            for (unsigned int i = 0; i < renderLayers.size(); ++i)
            {
              if (!renderLayers[i]._locator->isEquivalentTo(*masterTextureLocator.get()))
              {
                osg::Vec3d color_ndc;
                osgTerrain::Locator::convertLocalCoordBetween(*masterTextureLocator.get(), (*it), *renderLayers[i]._locator.get(), color_ndc);
                renderLayers[i]._stitchTexCoords->push_back(osg::Vec2(color_ndc.x(), color_ndc.y()));
              }
              else
              {
                renderLayers[i]._stitchTexCoords->push_back(osg::Vec2((*it).x(), (*it).y()));
              }
            }
          }
        }


        // Get triangles from triangulator and add as primative set to the geometry
        stitch_geom->addPrimitiveSet(trig->getTriangles());
        //stitch_geom->setNormalArray(trig->getOutputNormalArray());
        //stitch_geom->setNormalBinding(osg::Geometry::BIND_PER_PRIMITIVE);
#endif
      }
    }

    // populate primitive sets
    bool swapOrientation = !(_masterLocator->orientationOpenGL());

    osg::ref_ptr<osg::DrawElementsUShort> elements = new osg::DrawElementsUShort(GL_TRIANGLES);
    elements->reserve((numRows-1) * (numColumns-1) * 6);

    surface->addPrimitiveSet(elements.get());
    
    osg::ref_ptr<osg::Vec3Array> skirtVectors = new osg::Vec3Array( *normals );

          if (!normals)
        createSkirt = false;
    
    // New separated skirts.
    if ( createSkirt )
    {        
        // build the verts first:
        osg::Vec3Array* skirtVerts = new osg::Vec3Array();
        osg::Vec3Array* skirtNormals = new osg::Vec3Array();

        skirtVerts->reserve( numVerticesInSkirt );
        skirtNormals->reserve( numVerticesInSkirt );
        
        Indices skirtBreaks;
        skirtBreaks.push_back(0);

        // bottom:
        for( unsigned int c=0; c<numColumns-1; ++c )
        {
            int orig_i = indices[c];

            //int offset = 0;
            //while (orig_i < 0 && offset < numRows - 1)
            //  orig_i = indices[c + ++offset * numColumns];

            if (orig_i < 0)
            {
              if (skirtBreaks.back() != skirtVerts->size())
                skirtBreaks.push_back(skirtVerts->size());
            }
            else
            {
              skirtVerts->push_back( (*surfaceVerts)[orig_i] );
              skirtVerts->push_back( (*surfaceVerts)[orig_i] - ((*skirtVectors)[orig_i])*skirtHeight );
              skirtNormals->push_back( (*normals)[orig_i] );             
              skirtNormals->push_back( (*normals)[orig_i] );             


              if ( _texCompositor->requiresUnitTextureSpace() )
              {
                  unifiedSkirtTexCoords->push_back( (*unifiedSurfaceTexCoords)[orig_i] );
                  unifiedSkirtTexCoords->push_back( (*unifiedSurfaceTexCoords)[orig_i] );
              }
              else if ( renderLayers.size() > 0 )
              {
                  for (unsigned int i = 0; i < renderLayers.size(); ++i)
                  {
                      const osg::Vec2& tc = (*renderLayers[i]._texCoords.get())[orig_i];
                      renderLayers[i]._skirtTexCoords->push_back( tc );
                      renderLayers[i]._skirtTexCoords->push_back( tc );
                  }
              }
            }
        }

        // right:
        for( unsigned int r=0; r<numRows-1; ++r )
        {
            int orig_i = indices[r*numColumns+(numColumns-1)];
            if (orig_i < 0)
            {
              if (skirtBreaks.back() != skirtVerts->size())
                skirtBreaks.push_back(skirtVerts->size());
            }
            else
            {
              skirtVerts->push_back( (*surfaceVerts)[orig_i] );
              skirtVerts->push_back( (*surfaceVerts)[orig_i] - ((*skirtVectors)[orig_i])*skirtHeight );
              skirtNormals->push_back( (*normals)[orig_i] );             
              skirtNormals->push_back( (*normals)[orig_i] );             

              if ( _texCompositor->requiresUnitTextureSpace() )
              {
                  unifiedSkirtTexCoords->push_back( (*unifiedSurfaceTexCoords)[orig_i] );
                  unifiedSkirtTexCoords->push_back( (*unifiedSurfaceTexCoords)[orig_i] );
              }
              else if ( renderLayers.size() > 0 )
              {
                  for (unsigned int i = 0; i < renderLayers.size(); ++i)
                  {
                      const osg::Vec2& tc = (*renderLayers[i]._texCoords.get())[orig_i];
                      renderLayers[i]._skirtTexCoords->push_back( tc );
                      renderLayers[i]._skirtTexCoords->push_back( tc );
                  }
              }
            }
        }

        // top:
        for( int c=numColumns-1; c>0; --c )
        {
            int orig_i = indices[(numRows-1)*numColumns+c];
            if (orig_i < 0)
            {
              if (skirtBreaks.back() != skirtVerts->size())
                skirtBreaks.push_back(skirtVerts->size());
            }
            else
            {
              skirtVerts->push_back( (*surfaceVerts)[orig_i] );
              skirtVerts->push_back( (*surfaceVerts)[orig_i] - ((*skirtVectors)[orig_i])*skirtHeight );
              skirtNormals->push_back( (*normals)[orig_i] );             
              skirtNormals->push_back( (*normals)[orig_i] );             

              if ( _texCompositor->requiresUnitTextureSpace() )
              {
                  unifiedSkirtTexCoords->push_back( (*unifiedSurfaceTexCoords)[orig_i] );
                  unifiedSkirtTexCoords->push_back( (*unifiedSurfaceTexCoords)[orig_i] );
              }
              else if ( renderLayers.size() > 0 )
              {
                  for (unsigned int i = 0; i < renderLayers.size(); ++i)
                  {
                      const osg::Vec2& tc = (*renderLayers[i]._texCoords.get())[orig_i];
                      renderLayers[i]._skirtTexCoords->push_back( tc );
                      renderLayers[i]._skirtTexCoords->push_back( tc );
                  }
              }
            }
        }

        // left:
        for( int r=numRows-1; r>=0; --r )
        {
            int orig_i = indices[r*numColumns];
            if (orig_i < 0)
            {
              if (skirtBreaks.back() != skirtVerts->size())
                skirtBreaks.push_back(skirtVerts->size());
            }
            else
            {
              skirtVerts->push_back( (*surfaceVerts)[orig_i] );
              skirtVerts->push_back( (*surfaceVerts)[orig_i] - ((*skirtVectors)[orig_i])*skirtHeight );              
              skirtNormals->push_back( (*normals)[orig_i] );             
              skirtNormals->push_back( (*normals)[orig_i] );             

              if ( _texCompositor->requiresUnitTextureSpace() )
              {
                  unifiedSkirtTexCoords->push_back( (*unifiedSurfaceTexCoords)[orig_i] );
                  unifiedSkirtTexCoords->push_back( (*unifiedSurfaceTexCoords)[orig_i] );
              }
              else if ( renderLayers.size() > 0 )
              {
                  for (unsigned int i = 0; i < renderLayers.size(); ++i)
                  {
                      const osg::Vec2& tc = (*renderLayers[i]._texCoords.get())[orig_i];
                      renderLayers[i]._skirtTexCoords->push_back( tc );
                      renderLayers[i]._skirtTexCoords->push_back( tc );
                  }
              }
            }
        }

        skirt->setVertexArray( skirtVerts );
        if ( skirtVerts->getVertexBufferObject() )
            skirtVerts->getVertexBufferObject()->setUsage(GL_STATIC_DRAW_ARB);

        skirt->setNormalArray( skirtNormals );
        skirt->setNormalBinding( osg::Geometry::BIND_PER_VERTEX );

        //Add a primative set for each continuous skirt strip
        skirtBreaks.push_back(skirtVerts->size());
        for (int p=1; p < (int)skirtBreaks.size(); p++)
          skirt->addPrimitiveSet( new osg::DrawArrays( GL_TRIANGLE_STRIP, skirtBreaks[p-1], skirtBreaks[p] - skirtBreaks[p-1] ) );
    }
    
    bool recalcNormals = elevationLayer != NULL;

    //Clear out the normals
    if (recalcNormals)
    {
        osg::Vec3Array::iterator nitr;
        for(nitr = normals->begin();
            nitr!=normals->end();
            ++nitr)
        {
            nitr->set(0.0f,0.0f,0.0f);
        }
    }
    
    for(j=0; j<numRows-1; ++j)
    {
        for(i=0; i<numColumns-1; ++i)
        {
            int i00;
            int i01;
            if (swapOrientation)
            {
                i01 = j*numColumns + i;
                i00 = i01+numColumns;
            }
            else
            {
                i00 = j*numColumns + i;
                i01 = i00+numColumns;
            }

            int i10 = i00+1;
            int i11 = i01+1;

            // remap indices to final vertex positions
            i00 = indices[i00];
            i01 = indices[i01];
            i10 = indices[i10];
            i11 = indices[i11];
            
            unsigned int numValid = 0;
            if (i00>=0) ++numValid;
            if (i01>=0) ++numValid;
            if (i10>=0) ++numValid;
            if (i11>=0) ++numValid;
            
            if (numValid==4)
            {

				bool VALID = true;
				for (MaskRecordVector::iterator mr = masks.begin(); mr != masks.end(); ++mr) {
					float min_i = (*mr)._ndcMin.x() * (double)(numColumns-1);
					float min_j = (*mr)._ndcMin.y() * (double)(numRows-1);
					float max_i = (*mr)._ndcMax.x() * (double)(numColumns-1);
					float max_j = (*mr)._ndcMax.y() * (double)(numRows-1);

					// We test if mask is completely in square
					if(i+1 >= min_i && i <= max_i && j+1 >= min_j && j <= max_j) {
						VALID = false;
						break;
					}
				}

				if (VALID) {
					float e00 = (*elevations)[i00];
					float e10 = (*elevations)[i10];
					float e01 = (*elevations)[i01];
					float e11 = (*elevations)[i11];

                osg::Vec3f &v00 = (*surfaceVerts)[i00];
                osg::Vec3f &v10 = (*surfaceVerts)[i10];
                osg::Vec3f &v01 = (*surfaceVerts)[i01];
                osg::Vec3f &v11 = (*surfaceVerts)[i11];

                if (!_optimizeTriangleOrientation || (e00-e11)<fabsf(e01-e10))
                {
                    elements->push_back(i01);
                    elements->push_back(i00);
                    elements->push_back(i11);

                    elements->push_back(i00);
                    elements->push_back(i10);
                    elements->push_back(i11);

                    if (recalcNormals)
                    {                        
                        osg::Vec3 normal1 = (v00-v01) ^ (v11-v01);
                        (*normals)[i01] += normal1;
                        (*normals)[i00] += normal1;
                        (*normals)[i11] += normal1;

                        osg::Vec3 normal2 = (v10-v00)^(v11-v00);
                        (*normals)[i00] += normal2;
                        (*normals)[i10] += normal2;
                        (*normals)[i11] += normal2;
                    }
                }
                else
                {
                    elements->push_back(i01);
                    elements->push_back(i00);
                    elements->push_back(i10);

                    elements->push_back(i01);
                    elements->push_back(i10);
                    elements->push_back(i11);

                    if (recalcNormals)
                    {                       
                        osg::Vec3 normal1 = (v00-v01) ^ (v10-v01);
                        (*normals)[i01] += normal1;
                        (*normals)[i00] += normal1;
                        (*normals)[i10] += normal1;

                        osg::Vec3 normal2 = (v10-v01)^(v11-v01);
                        (*normals)[i01] += normal2;
                        (*normals)[i10] += normal2;
                        (*normals)[i11] += normal2;
                    }
                }
				}
            }
            // As skirtPoly is filling the mask bbox, we don't need to create isolated triangle
			/*else if (numValid==3)
            {
                int validIndices[3];
                int indexPtr = 0;
                if (i00>=0)
                {
                    elements->push_back(i00);
                    validIndices[indexPtr++] = i00;
                }

                if (i01>=0)
                {
                    elements->push_back(i01);
                    validIndices[indexPtr++] = i01;
                }

                if (i11>=0)
                {
                    elements->push_back(i11);
                    validIndices[indexPtr++] = i11;
                }

                if (i10>=0)
                {
                    elements->push_back(i10);
                    validIndices[indexPtr++] = i10;
                }

                if (recalcNormals)
                {
                    osg::Vec3f &v1 = (*surfaceVerts)[validIndices[0]];
                    osg::Vec3f &v2 = (*surfaceVerts)[validIndices[1]];
                    osg::Vec3f &v3 = (*surfaceVerts)[validIndices[2]];
                    osg::Vec3f normal = (v2 - v1) ^ (v3 - v1);
                    (*normals)[validIndices[0]] += normal;
                    (*normals)[validIndices[1]] += normal;
                    (*normals)[validIndices[2]] += normal;
                }
            } */
        }
    }

    //Normalize the normals
    if (recalcNormals)
    {
        osg::Vec3Array::iterator nitr;
        for(nitr = normals->begin();
            nitr!=normals->end();
            ++nitr)
        {
            nitr->normalize();
        }
    }

  

    MeshConsolidator::convertToTriangles( *surface );

    if ( skirt )
        MeshConsolidator::convertToTriangles( *skirt );

    for (MaskRecordVector::iterator mr = masks.begin(); mr != masks.end(); ++mr)
        MeshConsolidator::convertToTriangles( *((*mr)._geom) );
    
   
    if (osgDB::Registry::instance()->getBuildKdTreesHint()==osgDB::ReaderWriter::Options::BUILD_KDTREES &&
        osgDB::Registry::instance()->getKdTreeBuilder())
    {            
        osg::ref_ptr<osg::KdTreeBuilder> builder = osgDB::Registry::instance()->getKdTreeBuilder()->clone();
        group->accept(*builder);
    }

    return group;
}

void
SinglePassTerrainTechnique::traverse(osg::NodeVisitor& nv)
{
    if ( !_tile )
        return;

    if ( _transform.valid() )
    {
        _transform->accept( nv );
    }
}

void
SinglePassTerrainTechnique::releaseGLObjects(osg::State* state) const
{
    SinglePassTerrainTechnique* ncThis = const_cast<SinglePassTerrainTechnique*>(this);

    Threading::ScopedWriteLock lock( static_cast<Tile*>(ncThis->_tile)->getTileLayersMutex() );

    if ( _transform.valid() )
    {
        _transform->releaseGLObjects( state );
    }

    if ( _backNode.valid() )
    {
        _backNode->releaseGLObjects(state);
        ncThis->_backNode = 0L;
    }
}
