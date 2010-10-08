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

#include <osg/Program>
#include <osg/io_utils>
#include <osg/StateSet>
#include <osg/Program>
#include <osg/Math>
#include <osg/Timer>
#include <osg/Version>

#include <sstream>

using namespace osgEarth;
using namespace OpenThreads;


#define LC "[SinglePassTechnique] "

// OSG 2.9.8 changed the osgTerrain API...
#if OSG_VERSION_GREATER_OR_EQUAL(2,9,8)
#   define USE_NEW_OSGTERRAIN_298_API 1 
#endif

// --------------------------------------------------------------------------

SinglePassTerrainTechnique::SinglePassTerrainTechnique( TextureCompositor* compositor ) :
_texCompositor( compositor ),
_verticalScaleOverride(1.0f),
_fullUpdatePending( false ),
_initCount(0),
_optimizeTriangleOrientation(true),
_attachedProgram(false),
_pendingImageLayerUpdateIndex(-1),
_pendingGeometryUpdate(false)
{
    this->setThreadSafeRefUnref(true);

    // create a texture compositor.
    if ( !_texCompositor.valid() )
        _texCompositor = new TextureCompositor();

    // do this here so that we can use the program in the prototype
    _compositeProgram = _texCompositor->getProgram();
}

SinglePassTerrainTechnique::SinglePassTerrainTechnique(const SinglePassTerrainTechnique& gt,const osg::CopyOp& copyop):
CustomTerrainTechnique(gt,copyop),
_masterLocator( gt._masterLocator ),
_centerModel( gt._centerModel ),
_verticalScaleOverride( gt._verticalScaleOverride ),
_fullUpdatePending( gt._fullUpdatePending ),
_initCount( gt._initCount ),
_optimizeTriangleOrientation(gt._optimizeTriangleOrientation),
_compositeProgram(gt._compositeProgram),
_texCompositor(gt._texCompositor),
_pendingImageLayerUpdateIndex(gt._pendingImageLayerUpdateIndex),
_pendingGeometryUpdate(gt._pendingGeometryUpdate)
{
    _attachedProgram = false;
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

    _fullUpdatePending = true;
    applyTileUpdates();
}

void
SinglePassTerrainTechnique::compile( const TileUpdate& update, ProgressCallback* progress )
{
    // lock changes to the layers while we're compiling them
    Threading::ScopedReadLock lock( getMutex() );

    // we cannot run this method is there is a swap currently pending!
    if ( _fullUpdatePending )
    {
        OE_INFO << "illegal; cannot init() with a pending update!" << std::endl;
        return;
    }
    
    // safety check
    if (!_terrainTile) 
        return;

    // establish the master tile locator from the customtile key
    if ( !_masterLocator.valid() || !_transform.valid() )
    {
        CustomTile* tile = static_cast<CustomTile*>( _terrainTile );
        _masterLocator = tile->getLocator();
        
        osg::Vec3d centerNDC( 0.5, 0.5, 0 );
        _masterLocator->convertLocalToModel( osg::Vec3(.5,.5,0), _centerModel );
        _transform = new osg::MatrixTransform( osg::Matrix::translate(_centerModel) );

        // this is a placeholder so that we can always just call setChild(0) later.
        _transform->addChild( new osg::Group );
    }

    // attach the shader program required by the active texture compositor, if applicable
    if ( !_attachedProgram && _compositeProgram.valid() )
    {
        _terrainTile->getOrCreateStateSet()->setAttributeAndModes( _compositeProgram.get(), osg::StateAttribute::ON );
        _attachedProgram = true;
    }

    if ( update.getAction() == TileUpdate::UPDATE_IMAGE_LAYER && _texCompositor->supportsLayerUpdate() )
    {
        _pendingImageLayerUpdate = prepareImageLayerUpdate( update.getIndex() );
        _pendingImageLayerUpdateIndex = _pendingImageLayerUpdate.valid() ? update.getIndex() : -1;
    }

    else if ( update.getAction() == TileUpdate::UPDATE_ELEVATION )
    {
        createGeometry();
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
        createGeometry();

        // give the engine a chance to bail out before building the texture stateset:
        if ( progress && progress->isCanceled() )
        {
            _backGeode = 0L;
            return;
        }

        // create the stateset for this tile, which contains all the texture information.
        osg::StateSet* stateSet = createStateSet();
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
    }
    
#ifdef USE_NEW_OSGTERRAIN_298_API
    // In the updated API, the technique is now responsible for clearing the dirty flag.
    // It used to be the tile that cleared it.
    _terrainTile->setDirtyMask(0);
#endif
}

bool
SinglePassTerrainTechnique::applyTileUpdates()
{
    bool swapped = false;

    Threading::ScopedReadLock lock( getMutex() );

    // process a pending buffer swap:
    if ( _fullUpdatePending )
    {
        _transform->setChild( 0, _backGeode.get() );
        _backGeode = 0L;
        swapped = true;
    }
    _fullUpdatePending = false;

    // process any pending LIVE per-layer updates:
    if ( _pendingImageLayerUpdate.valid() && _pendingImageLayerUpdateIndex >= 0 )
    {
        _texCompositor->applyLayerUpdate(
            getFrontGeode()->getStateSet(),
            _pendingImageLayerUpdateIndex,
            _pendingImageLayerUpdate,
            _tileExtent );

        _pendingImageLayerUpdate = GeoImage::INVALID;
        _pendingImageLayerUpdateIndex = -1;
    }

    // process any pending LIVE geometry updates:
    if ( _pendingGeometryUpdate )
    {
        osg::Geode* frontGeode = getFrontGeode();

        // copy the drawables from the back buffer to the front buffer. By doing this,
        // we don't touch the front geode's stateset (which contains the textures) and
        // therefore they don't get re-applied.
        for( int i=0; i<_backGeode->getNumDrawables(); ++i )
        {
            frontGeode->setDrawable( i, _backGeode->getDrawable( i ) );
        }
        
        _pendingGeometryUpdate = false;
        _backGeode = 0L;
    }

    return swapped;
}

Threading::ReadWriteMutex&
SinglePassTerrainTechnique::getMutex()
{
    return static_cast<CustomTile*>(_terrainTile)->getTileLayersMutex();
}

GeoImage
SinglePassTerrainTechnique::prepareImageLayerUpdate( int layerNum )
{
    GeoImage geoImage = createGeoImage( _terrainTile->getColorLayer(layerNum) );
    if ( geoImage.valid() )
    {
        return _texCompositor->prepareLayerUpdate( geoImage, _tileExtent );
    }
    return GeoImage::INVALID;
}

GeoImage
SinglePassTerrainTechnique::createGeoImage( osgTerrain::Layer* colorLayer ) const
{
    osgTerrain::ImageLayer* imageLayer = dynamic_cast<osgTerrain::ImageLayer*>( colorLayer );
    if ( imageLayer )
    {            
        // record the proper texture offset/scale for this layer. this accounts for subregions that
        // are used when referencing lower LODs.
        osg::ref_ptr<GeoLocator> layerLocator = dynamic_cast<GeoLocator*>( imageLayer->getLocator() );
        if ( layerLocator )
        {
            if ( layerLocator->getCoordinateSystemType() == osgTerrain::Locator::GEOCENTRIC )
                layerLocator = layerLocator->getGeographicFromGeocentric();

            const GeoExtent& layerExtent = layerLocator->getDataExtent();
            return GeoImage( imageLayer->getImage(), layerExtent );
        }
    }
    return GeoImage::INVALID;
}

osg::StateSet*
SinglePassTerrainTechnique::createStateSet()
{
    // Composite all the image layer images into a single 2D texture array.

    // establish the tile extent. we will calculate texture coordinate offset/scale based on this
    if ( !_tileExtent.isValid() )
    {
        osg::ref_ptr<GeoLocator> tileLocator = dynamic_cast<GeoLocator*>( _terrainTile->getLocator() );
        if ( tileLocator.valid() )
        {
            if ( tileLocator->getCoordinateSystemType() == osgTerrain::Locator::GEOCENTRIC )
                tileLocator = tileLocator->getGeographicFromGeocentric();

            _tileExtent = tileLocator->getDataExtent();
        }
    }

    // find each image layer and create a region entry for it
    GeoImageVector imageStack;
    unsigned int numColorLayers = _terrainTile->getNumColorLayers();

    for( unsigned int layerNum=0; layerNum < numColorLayers; ++layerNum )
    {
        GeoImage geoImage = createGeoImage( _terrainTile->getColorLayer( layerNum ) );
        if ( geoImage.valid() )
        {
            imageStack.push_back( geoImage );
        }
    }

    osg::StateSet* texStateSet = _texCompositor->createStateSet( imageStack, _tileExtent );
    return texStateSet;
}

void
SinglePassTerrainTechnique::calculateSampling( int& out_rows, int& out_cols, double& out_i, double& out_j )
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

void
SinglePassTerrainTechnique::createGeometry()
{
    osg::ref_ptr< osgTerrain::Locator > masterTextureLocator = _masterLocator.get();
    GeoLocator* geoMasterLocator = dynamic_cast<GeoLocator*>(_masterLocator.get());

	bool isCube = dynamic_cast<CubeFaceLocator*>(_masterLocator.get()) != NULL;

    // If we have a geocentric locator, get a geographic version of it to avoid converting
    // to/from geocentric when computing texture coordinats
    if (!isCube && geoMasterLocator && _masterLocator->getCoordinateSystemType() == osgTerrain::Locator::GEOCENTRIC)
    {
        masterTextureLocator = geoMasterLocator->getGeographicFromGeocentric();
    }
    
    osgTerrain::Layer* elevationLayer = _terrainTile->getElevationLayer();

    // fire up a brand new geode.
    _backGeode = new osg::Geode();
    _backGeode->setThreadSafeRefUnref(true);
    
    // setting the geometry to DYNAMIC means its draw will not overlap the next frame's update/cull
    // traversal - which could access the buffer without a mutex

    osg::Geometry* surface = new osg::Geometry();
    surface->setThreadSafeRefUnref(true); // TODO: probably unnecessary.
    surface->setDataVariance( osg::Object::DYNAMIC );
    _backGeode->addDrawable( surface );

    osg::Geometry* skirt = new osg::Geometry();
    skirt->setThreadSafeRefUnref(true); // TODO: probably unnecessary.
    skirt->setDataVariance( osg::Object::DYNAMIC );
    _backGeode->addDrawable( skirt );
        
    int numRows = 20;
    int numColumns = 20;
    
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
    //osg::ref_ptr<osg::Vec3Array> surfaceVerts = new osg::Vec3Array( numVerticesInSurface );
    surface->setVertexArray( surfaceVerts.get() );

    // allocate and assign normals
    osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array(); // numVerticesInSurface );
    if (normals.valid()) normals->reserve(numVerticesInSurface);
    surface->setNormalArray(normals.get());
    surface->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);

    // allocate and assign texture coordinates
    osg::Vec2Array* unifiedSurfaceTexCoords = 0L;

    // used for per-layer texture coordinates:
    std::vector< osg::ref_ptr<osg::Vec2Array> > layerTexCoords;
    std::vector< osg::ref_ptr<osgTerrain::Locator> > layerLocators;
    
    if ( _texCompositor->requiresUnitTextureSpace() )
    {
        // for a unified unit texture space, just make a single texture coordinate array.
        unifiedSurfaceTexCoords = new osg::Vec2Array();
        unifiedSurfaceTexCoords->reserve( numVerticesInSurface );
        surface->setTexCoordArray( 0, unifiedSurfaceTexCoords );
    }
    else
    {
        // for a multitexture space, make a tex coord array per layer, each with its own locator.
        int numColorLayers = _terrainTile->getNumColorLayers();

        layerTexCoords.reserve( numColorLayers );
        layerLocators.reserve( numColorLayers );

        for( int i=0; i<numColorLayers; ++i )
        {
            osgTerrain::Layer* colorLayer = _terrainTile->getColorLayer( i );
            if (colorLayer)
            {
                osgTerrain::Locator* locator = colorLayer->getLocator();
                if ( !isCube && locator && locator->getCoordinateSystemType() == osgTerrain::Locator::GEOCENTRIC )
                {
                    GeoLocator* geo = dynamic_cast<GeoLocator*>(locator);
                    if ( geo )
                        locator = geo->getGeographicFromGeocentric();
                }
                layerLocators.push_back( locator ? locator : masterTextureLocator.get() );
                osg::Vec2Array* texCoords = new osg::Vec2Array();
                texCoords->reserve( numVerticesInSurface );
                layerTexCoords.push_back( texCoords );
                surface->setTexCoordArray( layerTexCoords.size()-1, texCoords );
            }
        }
    }

    // skirt texture coordinates, if applicable:
    osg::Vec2Array* skirtTexCoords = 0L;
    if ( createSkirt )
    {
        skirtTexCoords = new osg::Vec2Array();
        skirtTexCoords->reserve( numVerticesInSkirt );
        skirt->setTexCoordArray( 0, skirtTexCoords );
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
                // OE_INFO<<"i="<<i<<" j="<<j<<" z="<<value<<std::endl;
                ndc.z() = value*scaleHeight;
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
                    for( int layerNum = 0; layerNum < layerTexCoords.size(); ++layerNum )
                    {
                        osg::Vec2Array* texCoords = layerTexCoords[layerNum].get();
                        osgTerrain::Locator* layerLocator = layerLocators[layerNum].get();
                        if ( layerLocator != _masterLocator )
                        {
                            osg::Vec3d color_ndc;
                            osgTerrain::Locator::convertLocalCoordBetween( *masterTextureLocator.get(), ndc, *layerLocator, color_ndc );
                            texCoords->push_back( osg::Vec2(color_ndc.x(), color_ndc.y()) );
                        }
                        else
                        {
                            texCoords->push_back( osg::Vec2(ndc.x(), ndc.y()) );
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
        
        // bottom:
        for( int c=0; c<numColumns-1; ++c )
        {
            int orig_i = indices[c];
            skirtVerts->push_back( (*surfaceVerts)[orig_i] );
            skirtVerts->push_back( (*surfaceVerts)[orig_i] - ((*skirtVectors)[orig_i])*skirtHeight );

            if ( _texCompositor->requiresUnitTextureSpace() )
            {
                skirtTexCoords->push_back( (*unifiedSurfaceTexCoords)[orig_i] );
                skirtTexCoords->push_back( (*unifiedSurfaceTexCoords)[orig_i] );
            }
            else
            {
                for( int layerNum = 0; layerNum < layerTexCoords.size(); ++layerNum )
                {
                    skirtTexCoords->push_back( (*layerTexCoords[layerNum].get())[orig_i] );
                    skirtTexCoords->push_back( (*layerTexCoords[layerNum].get())[orig_i] );
                }
            }
        }

        // right:
        for( int r=0; r<numRows-1; ++r )
        {
            int orig_i = indices[r*numColumns+(numColumns-1)];
            skirtVerts->push_back( (*surfaceVerts)[orig_i] );
            skirtVerts->push_back( (*surfaceVerts)[orig_i] - ((*skirtVectors)[orig_i])*skirtHeight );

            if ( _texCompositor->requiresUnitTextureSpace() )
            {
                skirtTexCoords->push_back( (*unifiedSurfaceTexCoords)[orig_i] );
                skirtTexCoords->push_back( (*unifiedSurfaceTexCoords)[orig_i] );
            }
            else
            {
                for( int layerNum = 0; layerNum < layerTexCoords.size(); ++layerNum )
                {
                    skirtTexCoords->push_back( (*layerTexCoords[layerNum].get())[orig_i] );
                    skirtTexCoords->push_back( (*layerTexCoords[layerNum].get())[orig_i] );
                }
            }
        }

        // top:
        for( int c=numColumns-1; c>0; --c )
        {
            int orig_i = indices[(numRows-1)*numColumns+c];
            skirtVerts->push_back( (*surfaceVerts)[orig_i] );
            skirtVerts->push_back( (*surfaceVerts)[orig_i] - ((*skirtVectors)[orig_i])*skirtHeight );

            if ( _texCompositor->requiresUnitTextureSpace() )
            {
                skirtTexCoords->push_back( (*unifiedSurfaceTexCoords)[orig_i] );
                skirtTexCoords->push_back( (*unifiedSurfaceTexCoords)[orig_i] );
            }
            else
            {
                for( int layerNum = 0; layerNum < layerTexCoords.size(); ++layerNum )
                {
                    skirtTexCoords->push_back( (*layerTexCoords[layerNum].get())[orig_i] );
                    skirtTexCoords->push_back( (*layerTexCoords[layerNum].get())[orig_i] );
                }
            }
        }

        // left:
        for( int r=numRows-1; r>=0; --r )
        {
            int orig_i = indices[r*numColumns];
            skirtVerts->push_back( (*surfaceVerts)[orig_i] );
            skirtVerts->push_back( (*surfaceVerts)[orig_i] - ((*skirtVectors)[orig_i])*skirtHeight );

            if ( _texCompositor->requiresUnitTextureSpace() )
            {
                skirtTexCoords->push_back( (*unifiedSurfaceTexCoords)[orig_i] );
                skirtTexCoords->push_back( (*unifiedSurfaceTexCoords)[orig_i] );
            }
            else
            {
                for( int layerNum = 0; layerNum < layerTexCoords.size(); ++layerNum )
                {
                    skirtTexCoords->push_back( (*layerTexCoords[layerNum].get())[orig_i] );
                    skirtTexCoords->push_back( (*layerTexCoords[layerNum].get())[orig_i] );
                }
            }
        }

        skirt->setVertexArray( skirtVerts );
        skirt->addPrimitiveSet( new osg::DrawArrays( GL_TRIANGLE_STRIP, 0, skirtVerts->size() ) );
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
    
    
    if (osgDB::Registry::instance()->getBuildKdTreesHint()==osgDB::ReaderWriter::Options::BUILD_KDTREES &&
        osgDB::Registry::instance()->getKdTreeBuilder())
    {            
        //osg::Timer_t before = osg::Timer::instance()->tick();
        //OE_NOTICE<<"osgTerrain::GeometryTechnique::build kd tree"<<std::endl;
        osg::ref_ptr<osg::KdTreeBuilder> builder = osgDB::Registry::instance()->getKdTreeBuilder()->clone();
        _backGeode->accept(*builder);
        //osg::Timer_t after = osg::Timer::instance()->tick();
        //OE_NOTICE<<"KdTree build time "<<osg::Timer::instance()->delta_m(before, after)<<std::endl;
    }
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

        if ( _terrainTile )
            _terrainTile->osg::Group::traverse( nv );        
    }

    else if (nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR)
    {
        if ( _transform.valid() )
            _transform->accept( nv );

        return;
    }

    // the code from here on accounts for user traversals (intersections, etc)
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

    Threading::ScopedWriteLock lock( ncThis->getMutex() );

    if ( _transform.valid() )
        _transform->releaseGLObjects( state );

    if ( _backGeode.valid() )
    {
        _backGeode->releaseGLObjects(state);
        ncThis->_backGeode = 0L;
    }
}
