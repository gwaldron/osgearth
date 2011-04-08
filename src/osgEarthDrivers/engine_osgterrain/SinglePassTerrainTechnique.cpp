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
#include "SinglePassTerrainTechnique"
#include "CustomTerrain"

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
#include <osgUtil/Tessellator>

#include <osgEarthSymbology/Geometry>

#include <sstream>

using namespace osgEarth;
using namespace OpenThreads;

#define LC "[SinglePassTechnique] "

// OSG 2.9.8 changed the osgTerrain API...
#if OSG_VERSION_GREATER_OR_EQUAL(2,9,8)
#   define USE_NEW_OSGTERRAIN_298_API 1 
#endif

#define MATCH_TOLERANCE 0.001

// --------------------------------------------------------------------------

SinglePassTerrainTechnique::SinglePassTerrainTechnique( TextureCompositor* compositor ) :
_verticalScaleOverride(1.0f),
_initCount(0),
_pendingFullUpdate( false ),
_pendingGeometryUpdate(false),
_lastUpdate( TileUpdate::UPDATE_ALL ),
_optimizeTriangleOrientation(true),
_texCompositor( compositor ),
_frontGeodeInstalled( false )
{
    this->setThreadSafeRefUnref(true);
}

SinglePassTerrainTechnique::SinglePassTerrainTechnique(const SinglePassTerrainTechnique& rhs, const osg::CopyOp& copyop):
CustomTerrainTechnique( rhs, copyop ),
_verticalScaleOverride( rhs._verticalScaleOverride ),
_initCount( 0 ),
_pendingFullUpdate( false ),
_pendingGeometryUpdate( false ),
_lastUpdate( rhs._lastUpdate ),
_optimizeTriangleOrientation( rhs._optimizeTriangleOrientation ),
_texCompositor( rhs._texCompositor.get() ),
_frontGeodeInstalled( rhs._frontGeodeInstalled )
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
#ifdef USE_NEW_OSGTERRAIN_298_API
SinglePassTerrainTechnique::init(int dirtyMask, bool assumeMultiThreaded)
#else
SinglePassTerrainTechnique::init()
#endif
{
    compile( TileUpdate(TileUpdate::UPDATE_ALL), 0L );

    //_pendingFullUpdate = true;
    applyTileUpdates();
}

void
SinglePassTerrainTechnique::compile( const TileUpdate& update, ProgressCallback* progress )
{
    // safety check
    if ( !_terrainTile ) 
    {
        OE_WARN << LC << "Illegal; terrain tile is null" << std::endl;
        return;
    }

    // serialize access to the compilation procedure.
    OpenThreads::ScopedLock<Mutex> exclusiveLock( _compileMutex );

    // make a frame to use during compilation.
    CustomTileFrame tilef( static_cast<CustomTile*>(_terrainTile) );

    _lastUpdate = update;

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
            osg::ref_ptr<osg::StateSet> stateSet = _backGeode.valid() ? _backGeode->getStateSet() : 0L;
            _backGeode = createGeometry( tilef );
            _backGeode->setStateSet( stateSet.get() );

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
        osg::ref_ptr<osg::StateSet> stateSet = _backGeode.valid() ? _backGeode->getStateSet() : 0L;
        _backGeode = createGeometry( tilef );
        _backGeode->setStateSet( stateSet.get() );

        _pendingGeometryUpdate = true;
    }

    else // all other update types
    {
        // give the engine a chance to bail out before generating geometry
        if ( progress && progress->isCanceled() )
        {
            _backGeode = 0L;
            return;
        }
    
        // create the geometry and texture coordinates for this tile in a new buffer
        _backGeode = createGeometry( tilef );
        if ( !_backGeode.valid() )
        {
            OE_WARN << LC << "createGeometry returned NULL" << std::endl;
            return;
        }

        // give the engine a chance to bail out before building the texture stateset:
        if ( progress && progress->isCanceled() )
        {
            _backGeode = 0L;
            return;
        }

        // create the stateset for this tile, which contains all the texture information.
        osg::StateSet* stateSet = createStateSet( tilef );
        if ( stateSet )
        {
            _backGeode->setStateSet( stateSet );
        }

        // give the engine a chance to bail out before swapping buffers
        if ( progress && progress->isCanceled() )
        {
            _backGeode = 0L;
            return;
        }
       
        _initCount++;
        //if ( _initCount > 1 )
        //    OE_WARN << LC << "Tile was fully build " << _initCount << " times" << std::endl;

        if ( _backGeode.valid() && !_backGeode->getStateSet() )
            OE_WARN << LC << "ILLEGAL! no stateset in BackGeode!!" << std::endl;

        _pendingFullUpdate = true;
    }
    
#ifdef USE_NEW_OSGTERRAIN_298_API
    // In the updated API, the technique is now responsible for clearing the dirty flag.
    // It used to be the tile that cleared it.
    _terrainTile->setDirtyMask(0);
#endif
}

// from the UPDATE traversal thread:
bool
SinglePassTerrainTechnique::applyTileUpdates()
{
    bool applied = false;

    //Threading::ScopedReadLock lock( getMutex() );

    // serialize access to the compilation mechanism.
    OpenThreads::ScopedLock<Mutex> exclusiveLock( _compileMutex );

    // process a pending buffer swap:
    if ( _pendingFullUpdate )
    {
        if ( _backGeode->getStateSet() == 0L )
            OE_WARN << LC << "ILLEGAL: backGeode has no stateset" << std::endl;

        _transform->setChild( 0, _backGeode.get() );
        _frontGeodeInstalled = true;
        _backGeode = 0L;
        _pendingFullUpdate = false;
        _pendingGeometryUpdate = false;
        applied = true;
    }

    else
    {
        // process any pending LIVE geometry updates:
        if ( _pendingGeometryUpdate )
        {
            osg::Geode* frontGeode = getFrontGeode();
            
            if ( _texCompositor->requiresUnitTextureSpace() )
            {
                // in "unit-texture-space" mode, we can take the shortcut of just updating
                // the geometry VBOs. The texture coordinates never change.
                for( unsigned int i=0; i<_backGeode->getNumDrawables(); ++i )
                {
                    osg::Geometry* backGeom = static_cast<osg::Geometry*>( _backGeode->getDrawable(i) );
                    osg::Vec3Array* backVerts = static_cast<osg::Vec3Array*>( backGeom->getVertexArray() );

                    osg::Geometry* frontGeom = static_cast<osg::Geometry*>( frontGeode->getDrawable(i) );
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
                        frontGeom->setTexCoordArray( 0, backGeom->getTexCoordArray( 0 ) ); // TODO: un-hard-code
                        if ( backGeom->getNormalArray() )
                            frontGeom->setNormalArray( backGeom->getNormalArray() );
                    }
                }
            }
            else
            {
                // copy the drawables from the back buffer to the front buffer. By doing this,
                // we don't touch the front geode's stateset (which contains the textures) and
                // therefore they don't get re-applied.
                for( unsigned int i=0; i<_backGeode->getNumDrawables(); ++i )
                {
                    frontGeode->setDrawable( i, _backGeode->getDrawable( i ) );
                }
            }

            _pendingGeometryUpdate = false;
            _backGeode = 0L;
            applied = true;
        }

        // process any pending LIVE per-layer updates:
        osg::StateSet* parentStateSet = 0;
        if (! _pendingImageLayerUpdates.empty())
            parentStateSet = getParentStateSet();
        while( _pendingImageLayerUpdates.size() > 0 )
        {
            const ImageLayerUpdate& update = _pendingImageLayerUpdates.front();

            _texCompositor->applyLayerUpdate(
                getFrontGeode()->getStateSet(),
                update._layerUID,
                update._image,
                _tileKey,
                parentStateSet);

            _pendingImageLayerUpdates.pop();
            applied = true;
        }
    }

    return applied;
}

void
SinglePassTerrainTechnique::prepareImageLayerUpdate( UID layerUID, const CustomTileFrame& tilef )
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
        image = GeoImage( colorLayer.getImage(), imageExtent ); //const_cast<osg::Image*>(colorLayer.getImage()), imageExtent );
        return true;
    }
    return false;
}

osg::StateSet* SinglePassTerrainTechnique::getParentStateSet() const
{
    osg::StateSet* parentStateSet = 0;
    if (_terrainTile->getTerrain())
    {
        TileKey parentKey = _tileKey.createParentKey();
        CustomTile* parentTile
            = dynamic_cast<CustomTile*>(_terrainTile->getTerrain()->getTile(parentKey.getTileId()));
        if (parentTile)
        {
            SinglePassTerrainTechnique* parentTechnique
                = dynamic_cast<SinglePassTerrainTechnique*>(parentTile->getTerrainTechnique());
            if (parentTechnique)
            {
                if (parentTechnique->_backGeode.valid())
                    parentStateSet = parentTechnique->_backGeode->getStateSet();
                else if (parentTechnique->_transform.valid())
                    parentStateSet = parentTechnique->getFrontGeode()->getStateSet();
            }
        }
    }
    return parentStateSet;
}

osg::StateSet*
SinglePassTerrainTechnique::createStateSet( const CustomTileFrame& tilef )
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
        GeoImage image;
        if ( createGeoImage( colorLayer, image ) )
        {
            image = _texCompositor->prepareImage( image, _tileExtent );
            _texCompositor->applyLayerUpdate( stateSet, colorLayer.getUID(), image, _tileKey, parentStateSet );
        }
    }

    return stateSet;
}

void
SinglePassTerrainTechnique::calculateSampling( unsigned int& out_rows, unsigned int& out_cols, double& out_i, double& out_j )
{            
    osgTerrain::Layer* elevationLayer = _terrainTile->getElevationLayer();

    out_rows = elevationLayer->getNumRows();
    out_cols = elevationLayer->getNumColumns();
    out_i = 1.0;
    out_j = 1.0;

    float sampleRatio = _terrainTile->getTerrain() ? _terrainTile->getTerrain()->getSampleRatio() : 1.0f;
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
        osg::Vec2Array* _texCoords;
        osg::Vec2Array* _skirtTexCoords;
        bool _ownsTexCoords;
        RenderLayer() : _texCoords(0L), _ownsTexCoords(false) { }
    };

    typedef std::vector< RenderLayer > RenderLayerVector;
}

osg::Geode*
SinglePassTerrainTechnique::createGeometry( const CustomTileFrame& tilef )
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
    
    osgTerrain::Layer* elevationLayer = _terrainTile->getElevationLayer();

    // fire up a brand new geode.
    osg::Geode* geode = new osg::Geode();
    geode->setThreadSafeRefUnref(true);
    
    // setting the geometry to DYNAMIC means its draw will not overlap the next frame's update/cull
    // traversal - which could access the buffer without a mutex

    osg::Geometry* surface = new osg::Geometry();
    surface->setThreadSafeRefUnref(true); // TODO: probably unnecessary.
    surface->setDataVariance( osg::Object::DYNAMIC );
    geode->addDrawable( surface );

    osg::Geometry* skirt = new osg::Geometry();
    skirt->setThreadSafeRefUnref(true); // TODO: probably unnecessary.
    skirt->setDataVariance( osg::Object::DYNAMIC );
    geode->addDrawable( skirt );

    osg::Geometry* mask_skirt = new osg::Geometry();
    mask_skirt->setThreadSafeRefUnref(true);
    mask_skirt->setDataVariance( osg::Object::DYNAMIC );
    mask_skirt->getOrCreateStateSet()->setAttribute(new osg::Point( 5.0f ), osg::StateAttribute::ON);
    geode->addDrawable( mask_skirt);
        
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

    // allocate and assign normals
    osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array();
    normals->reserve(numVerticesInSurface);
    surface->setNormalArray(normals.get());
    surface->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);

    // skirt texture coordinates, if applicable:
    osg::Vec2Array* unifiedSkirtTexCoords = 0L;

    // allocate and assign texture coordinates
    osg::Vec2Array* unifiedSurfaceTexCoords = 0L;

    //int numColorLayers = _terrainTile->getNumColorLayers();
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
                if ( !r._texCoords )
                {
                    r._texCoords = new osg::Vec2Array();
                    r._texCoords->reserve( numVerticesInSurface );
                    r._ownsTexCoords = true;
                    locatorToTexCoordTable.push_back( LocatorTexCoordPair(locator, r._texCoords) );
                }

                r._skirtTexCoords = new osg::Vec2Array();
                r._skirtTexCoords->reserve( numVerticesInSkirt );

                r._locator = locator;
                if ( locator->getCoordinateSystemType() == osgTerrain::Locator::GEOCENTRIC )
                {
                    const GeoLocator* geo = dynamic_cast<const GeoLocator*>(locator);
                    if ( geo )
                        r._locator = geo->getGeographicFromGeocentric();
                }

                _texCompositor->assignTexCoordArray( surface, colorLayer.getUID(), r._texCoords );
                _texCompositor->assignTexCoordArray( skirt, colorLayer.getUID(), r._skirtTexCoords );
                //surface->setTexCoordArray( renderLayers.size(), r._texCoords );
                renderLayers.push_back( r );
            }
            else
            {
                OE_WARN << LC << "Found a Locator, but it wasn't a GeoLocator." << std::endl;
            }
        }
    }

    float scaleHeight = 
        _verticalScaleOverride != 1.0? _verticalScaleOverride :
        _terrainTile->getTerrain() ? _terrainTile->getTerrain()->getVerticalScale() :
        1.0f;

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

    osgEarth::GeoLocator* geoLocator = _masterLocator->getGeographicFromGeocentric();
    osg::Vec3dArray* mask = tilef._mask.valid() ? tilef._mask.get() : 0L;

    //Find the mask bounds
    osg::BoundingBoxd maskBB;
    //int i_max, i_min, j_max, j_min;
    if (mask)
    {
      osg::Vec3d min, max;
      min = max = mask->front();

      for (osg::Vec3dArray::iterator it = mask->begin(); it != mask->end(); ++it)
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

      min.z() = DBL_MIN;
      max.z() = DBL_MAX;
      maskBB.set(min, max);

      //min.z() = mask->front().z();
      //max.z() = mask->front().z();
      //osg::Vec3d localMin, localMax;
      //geoLocator->convertModelToLocal(min, localMin);
      //geoLocator->convertModelToLocal(max, localMax);

      //int max_x = max.x() * 

      //std::cout << std::endl << "localMin: " << localMin << std::endl << "localMax: " << localMax << std::endl;
    }

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
            if (validValue && mask)
            {
              osg::Vec3d world;
              geoLocator->convertLocalToModel(ndc, world);

              validValue = !maskBB.contains(world);
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
            else
            {
                indices[iv] = -1;
            }
        }
    }


    if (mask)
    {
      //Find mask skirt vertices
      int min_i = -1, max_i = -1, min_j = -1, max_j = -1;
      for(j=0; j<numRows; ++j)
      {
          for(i=0; i<numColumns; ++i)
          {
              unsigned int iv = j*numColumns + i;

              int n = 0;
              if (indices[iv] >= 0)
              {
                if (j > 0)
                {               
                  if (i > 0 && indices[iv - numColumns - 1] < 0)
                    n++;
                  
                  if (i < numColumns - 1 && indices[iv - numColumns + 1] < 0)
                    n++;
                }

                if (j < numRows - 1)
                {
                  if (i > 0 && indices[iv + numColumns - 1] < 0)
                    n++;
                  
                  if (i < numColumns - 1 && indices[iv + numColumns + 1] < 0)
                    n++;
                }

                if (n == 1)
                {
                  if (i != 0 && i != numColumns - 1 && j != 0 && j != numRows - 1)
                  {
                    if (indices[iv - 1] < 0)
                      n++;

                    if (indices[iv + 1] < 0)
                      n++;

                    if (indices[iv - numColumns] < 0 )
                      n++;

                    if (indices[iv + numColumns] < 0)
                      n++;
                  }
                }
                else
                {
                  //Test for special case where mask only intersects a single row
                  //or column along the edge of the tile.

                  if (i == 0 || i == numColumns - 1)
                  {
                    if (j > 0 && indices[iv - numColumns] < 0)
                      n++;

                    if (j < numRows - 1 && indices[iv + numColumns] < 0)
                      n++;
                  }
                  
                  if(j == 0 || j == numRows - 1)
                  {
                    if (i > 0 && indices[iv - 1] < 0)
                      n++;

                    if (i < numColumns - 1 && indices[iv + 1] < 0)
                      n++;
                  }
                }

                if (n == 1)
                {
                    if (min_i == -1)
                      min_i = i;
                    else if (min_i != i && max_i == -1)
                      max_i = i;

                    if (min_j == -1)
                      min_j = j;
                    else if (min_j != j && max_j == -1)
                      max_j = j;
                }
              }
              else
              {
                //Test for tile corners that fall within the mask bounds
                if ((i==0 && (j == 0 || j == numRows - 1)) ||
                    (i == numColumns - 1 && (j == 0 || j == numRows - 1)))
                {
                  if (min_i == -1)
                    min_i = i;
                  else if (min_i != i && max_i == -1)
                    max_i = i;

                  if (min_j == -1)
                    min_j = j;
                  else if (min_j != j && max_j == -1)
                    max_j = j;
                }
              }
          }
      }

      if (min_i >= 0 && max_i >= 0 && min_j >= 0 && max_j >= 0)
      {
        int num_i = max_i - min_i + 1;
        int num_j = max_j - min_j + 1;

        osg::ref_ptr<osgEarth::Symbology::Polygon> maskSkirtPoly = new osgEarth::Symbology::Polygon();
        maskSkirtPoly->resize(num_i * 2 + num_j * 2 - 4);
        for (int i = 0; i < num_i; i++)
        {
          int index = indices[min_j*numColumns + i + min_i];
          if (index == -1)
          {
            //unsigned int iv = j*numColumns + i;
            osg::Vec3d ndc( ((double)(i + min_i))/(double)(numColumns-1), ((double)min_j)/(double)(numRows-1), 0.0);

            if (elevationLayer)
            {
              unsigned int i_equiv = i_sampleFactor==1.0 ? i + min_i : (unsigned int) (double(i + min_i)*i_sampleFactor);
              unsigned int j_equiv = j_sampleFactor==1.0 ? min_j : (unsigned int) (double(min_j)*j_sampleFactor);

              float value = 0.0f;
              if (elevationLayer->getValidValue(i_equiv,j_equiv, value))
                ndc.z() = value*scaleHeight;
            }

            osg::Vec3d model;
            _masterLocator->convertLocalToModel(ndc, model);

            (*maskSkirtPoly)[i] = model - _centerModel;
          }
          else
          {
            (*maskSkirtPoly)[i] = (*surfaceVerts)[index];
          }

          index = indices[max_j*numColumns + i + min_i];
          if (index == -1)
          {
            //unsigned int iv = j*numColumns + i;
            osg::Vec3d ndc( ((double)(i + min_i))/(double)(numColumns-1), ((double)max_j)/(double)(numRows-1), 0.0);

            if (elevationLayer)
            {
              unsigned int i_equiv = i_sampleFactor==1.0 ? i + min_i : (unsigned int) (double(i + min_i)*i_sampleFactor);
              unsigned int j_equiv = j_sampleFactor==1.0 ? max_j : (unsigned int) (double(max_j)*j_sampleFactor);

              float value = 0.0f;
              if (elevationLayer->getValidValue(i_equiv,j_equiv, value))
                ndc.z() = value*scaleHeight;
            }

            osg::Vec3d model;
            _masterLocator->convertLocalToModel(ndc, model);

            (*maskSkirtPoly)[i + (2 * num_i + num_j - 3) - 2 * i] = model - _centerModel;
          }
          else
          {
            (*maskSkirtPoly)[i + (2 * num_i + num_j - 3) - 2 * i] = (*surfaceVerts)[index];
          }
        }
        for (int j = 0; j < num_j - 2; j++)
        {
          int index = indices[(min_j + j + 1)*numColumns + max_i];
          if (index == -1)
          {
            //unsigned int iv = j*numColumns + i;
            osg::Vec3d ndc( ((double)max_i)/(double)(numColumns-1), ((double)(min_j + j + 1))/(double)(numRows-1), 0.0);

            if (elevationLayer)
            {
              unsigned int i_equiv = i_sampleFactor==1.0 ? max_i : (unsigned int) (double(max_i)*i_sampleFactor);
              unsigned int j_equiv = j_sampleFactor==1.0 ? min_j + j + 1 : (unsigned int) (double(min_j + j + 1)*j_sampleFactor);

              float value = 0.0f;
              if (elevationLayer->getValidValue(i_equiv,j_equiv, value))
                ndc.z() = value*scaleHeight;
            }

            osg::Vec3d model;
            _masterLocator->convertLocalToModel(ndc, model);

            (*maskSkirtPoly)[j + num_i] = model - _centerModel;
          }
          else
          {
            (*maskSkirtPoly)[j + num_i] = (*surfaceVerts)[index];
          }

          index = indices[(min_j + j + 1)*numColumns + min_i];
          if (index == -1)
          {
            //unsigned int iv = j*numColumns + i;
            osg::Vec3d ndc( ((double)min_i)/(double)(numColumns-1), ((double)(min_j + j + 1))/(double)(numRows-1), 0.0);

            if (elevationLayer)
            {
              unsigned int i_equiv = i_sampleFactor==1.0 ? min_i : (unsigned int) (double(min_i)*i_sampleFactor);
              unsigned int j_equiv = j_sampleFactor==1.0 ? min_j + j + 1 : (unsigned int) (double(min_j + j + 1)*j_sampleFactor);

              float value = 0.0f;
              if (elevationLayer->getValidValue(i_equiv,j_equiv, value))
                ndc.z() = value*scaleHeight;
            }

            osg::Vec3d model;
            _masterLocator->convertLocalToModel(ndc, model);

            (*maskSkirtPoly)[j + (2 * num_i + 2 * num_j - 5) - 2 * j] = model - _centerModel;
          }
          else
          {
            (*maskSkirtPoly)[j + (2 * num_i + 2 * num_j - 5) - 2 * j] = (*surfaceVerts)[index];
          }
        }

        //Create local polygon representing mask
        osg::ref_ptr<osgEarth::Symbology::Polygon> maskPoly = new osgEarth::Symbology::Polygon();
        for (osg::Vec3dArray::iterator it = mask->begin(); it != mask->end(); ++it)
        {
          osg::Vec3d local;
          geoLocator->convertModelToLocal(*it, local);
          osg::Vec3d model;
          _masterLocator->convertLocalToModel(local, model);
          maskPoly->push_back(model - _centerModel);
        }

//Change the following two #if statements to see mask skirt polygons
//before clipping and adjusting
#if 1
        //Do a diff on the polygons to get the actual mask skirt
        osg::ref_ptr<osgEarth::Symbology::Geometry> outPoly;
        maskSkirtPoly->difference(maskPoly, outPoly);
#else
        osg::ref_ptr<osgEarth::Symbology::Geometry> outPoly = maskSkirtPoly;
#endif

        osg::Vec3Array* outVerts = new osg::Vec3Array();
        mask_skirt->setVertexArray(outVerts);
        
        osgEarth::Symbology::GeometryIterator i( outPoly );
        while( i.hasMore() )
        {
          osgEarth::Symbology::Geometry* part = i.next();
          if (!part)
            continue;

          if (part->getType() == osgEarth::Symbology::Geometry::TYPE_POLYGON)
          {
            osg::Vec3Array* partVerts = part->toVec3Array();
            outVerts->insert(outVerts->end(), partVerts->begin(), partVerts->end());
            mask_skirt->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POLYGON, outVerts->size() - partVerts->size(), partVerts->size()));
          }
        }

        if (mask_skirt->getNumPrimitiveSets() > 0)
        {
#if 1
          //Tessellate mask skirt
          osg::ref_ptr<osgUtil::Tessellator> tscx=new osgUtil::Tessellator;
          tscx->setTessellationType(osgUtil::Tessellator::TESS_TYPE_GEOMETRY);
          tscx->setBoundaryOnly(false);
          tscx->setWindingType(osgUtil::Tessellator::TESS_WINDING_ODD);
          tscx->retessellatePolygons(*mask_skirt);

          //Retrieve z values for mask skirt verts
          for (osg::Vec3Array::iterator it = outVerts->begin(); it != outVerts->end(); ++it)
          {
            //Look for verts that belong to the original mask skirt polygon
            for (osgEarth::Symbology::Polygon::iterator mit = maskSkirtPoly->begin(); mit != maskSkirtPoly->end(); ++mit)
            {
              if (osg::absolute((*mit).x() - (*it).x()) < MATCH_TOLERANCE && osg::absolute((*mit).y() - (*it).y()) < MATCH_TOLERANCE)
              {
                (*it).z() = (*mit).z();
                break;
              }
            }

            //Look for verts that belong to the mask polygon
            for (osgEarth::Symbology::Polygon::iterator mit = maskPoly->begin(); mit != maskPoly->end(); ++mit)
            {
              if ((*it).z() == 0 && osg::absolute((*mit).x() - (*it).x()) < MATCH_TOLERANCE && osg::absolute((*mit).y() - (*it).y()) < MATCH_TOLERANCE)
              {
                (*it).z() = (*mit).z();
                break;
              }
            }
          }

          //Any mask skirt verts that still have a z value of 0 are newly created verts where the
          //skirt meets the mask. Find the mask segment the point lies along and calculate the
          //appropriate z value for the point.
          for (osg::Vec3Array::iterator it = outVerts->begin(); it != outVerts->end(); ++it)
          {
            if ((*it).z() == 0.0)
            {
              osg::Vec3d p2 = *it;
              for (osgEarth::Symbology::Polygon::iterator mit = maskPoly->begin(); mit != maskPoly->end(); ++mit)
              {
                osg::Vec3d p1 = *mit;
                osg::Vec3d p3 = mit == --maskPoly->end() ? maskPoly->front() : (*(mit + 1));
                
                double m1 = (p2.y() - p1.y()) / (p2.x() - p1.x());
                double m2 = (p3.y() - p1.y()) / (p3.x() - p1.x());

                if (osg::absolute(m2 - m1) < MATCH_TOLERANCE)
                {
                  double l1 =(osg::Vec2d(p2.x(), p2.y()) - osg::Vec2d(p1.x(), p1.y())).length();
                  double lt = (osg::Vec2d(p3.x(), p3.y()) - osg::Vec2d(p1.x(), p1.y())).length();
                  double zmag = p3.z() - p1.z();

                  (*it).z() = (l1 / lt) * zmag + p1.z();

                  break;
                }
              }
            }
          }
#endif
        }
      }
    }

    // populate primitive sets
    bool swapOrientation = !(_masterLocator->orientationOpenGL());

    osg::ref_ptr<osg::DrawElementsUInt> elements = new osg::DrawElementsUInt(GL_TRIANGLES);
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
        skirtVerts->reserve( numVerticesInSkirt );
        
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

              if ( _texCompositor->requiresUnitTextureSpace() )
              {
                  unifiedSkirtTexCoords->push_back( (*unifiedSurfaceTexCoords)[orig_i] );
                  unifiedSkirtTexCoords->push_back( (*unifiedSurfaceTexCoords)[orig_i] );
              }
              else if ( renderLayers.size() > 0 )
              {
                  for (unsigned int i = 0; i < renderLayers.size(); ++i)
                  {
                      const osg::Vec2& tc = (*renderLayers[i]._texCoords)[orig_i];
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

              if ( _texCompositor->requiresUnitTextureSpace() )
              {
                  unifiedSkirtTexCoords->push_back( (*unifiedSurfaceTexCoords)[orig_i] );
                  unifiedSkirtTexCoords->push_back( (*unifiedSurfaceTexCoords)[orig_i] );
              }
              else if ( renderLayers.size() > 0 )
              {
                  for (unsigned int i = 0; i < renderLayers.size(); ++i)
                  {
                      const osg::Vec2& tc = (*renderLayers[i]._texCoords)[orig_i];
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

              if ( _texCompositor->requiresUnitTextureSpace() )
              {
                  unifiedSkirtTexCoords->push_back( (*unifiedSurfaceTexCoords)[orig_i] );
                  unifiedSkirtTexCoords->push_back( (*unifiedSurfaceTexCoords)[orig_i] );
              }
              else if ( renderLayers.size() > 0 )
              {
                  for (unsigned int i = 0; i < renderLayers.size(); ++i)
                  {
                      const osg::Vec2& tc = (*renderLayers[i]._texCoords)[orig_i];
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

              if ( _texCompositor->requiresUnitTextureSpace() )
              {
                  unifiedSkirtTexCoords->push_back( (*unifiedSurfaceTexCoords)[orig_i] );
                  unifiedSkirtTexCoords->push_back( (*unifiedSurfaceTexCoords)[orig_i] );
              }
              else if ( renderLayers.size() > 0 )
              {
                  for (unsigned int i = 0; i < renderLayers.size(); ++i)
                  {
                      const osg::Vec2& tc = (*renderLayers[i]._texCoords)[orig_i];
                      renderLayers[i]._skirtTexCoords->push_back( tc );
                      renderLayers[i]._skirtTexCoords->push_back( tc );
                  }
              }
            }
        }

        skirt->setVertexArray( skirtVerts );

        //Add a primative set for each continuous skirt strip
        skirtBreaks.push_back(skirtVerts->size());
        for (int p=1; p < skirtBreaks.size(); p++)
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
            else if (numValid==3)
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
            }            
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

    surface->setUseDisplayList(false);
    surface->setUseVertexBufferObjects(true);

    skirt->setUseDisplayList(false);
    skirt->setUseVertexBufferObjects(true);

    mask_skirt->setUseDisplayList(false);
    mask_skirt->setUseVertexBufferObjects(true);
    
    
    if (osgDB::Registry::instance()->getBuildKdTreesHint()==osgDB::ReaderWriter::Options::BUILD_KDTREES &&
        osgDB::Registry::instance()->getKdTreeBuilder())
    {            
        //osg::Timer_t before = osg::Timer::instance()->tick();
        //OE_NOTICE<<"osgTerrain::GeometryTechnique::build kd tree"<<std::endl;
        osg::ref_ptr<osg::KdTreeBuilder> builder = osgDB::Registry::instance()->getKdTreeBuilder()->clone();
        geode->accept(*builder);
        //osg::Timer_t after = osg::Timer::instance()->tick();
        //OE_NOTICE<<"KdTree build time "<<osg::Timer::instance()->delta_m(before, after)<<std::endl;
    }

    return geode;
}

void
SinglePassTerrainTechnique::traverse(osg::NodeVisitor& nv)
{
    if ( !_terrainTile )
        return;

    // if app traversal update the frame count.
    if ( nv.getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR )
    {
#if OSG_MIN_VERSION_REQUIRED(2,9,8)
        if (_terrainTile->getDirty()) _terrainTile->init(~0x0,true);
#else
        if (_terrainTile->getDirty()) _terrainTile->init();
#endif

        _terrainTile->osg::Group::traverse( nv );    

        // traverse the actual geometry in the tile. this is especially 
        // important for geometry with ImageSequences and other things
        // that require an update traversal.
        if ( _transform.valid() )
            _transform->accept( nv );

        return;
    }

    else if (nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR)
    {
        if ( _transform.valid() )
            _transform->accept( nv );
        return;
    }

    // the code from here on accounts for user traversals (intersections, etc)

    //TODO: evaluate this and see if we can get rid of it.

    if ( _terrainTile->getDirty() ) 
    {
#if OSG_MIN_VERSION_REQUIRED(2,9,8)
        _terrainTile->init(~0x0, true);
#else
        _terrainTile->init();
#endif
    }

    if ( _transform.valid() )
        _transform->accept( nv );
}

void
SinglePassTerrainTechnique::releaseGLObjects(osg::State* state) const
{
    SinglePassTerrainTechnique* ncThis = const_cast<SinglePassTerrainTechnique*>(this);

    Threading::ScopedWriteLock lock( 
        static_cast<CustomTile*>( ncThis->_terrainTile )->getTileLayersMutex() );

    if ( _transform.valid() )
        _transform->releaseGLObjects( state );

    if ( _backGeode.valid() )
    {
        _backGeode->releaseGLObjects(state);
        ncThis->_backGeode = 0L;
    }
}
