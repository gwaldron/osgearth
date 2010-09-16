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
#include "CompositingTerrainTechnique"
#include "CustomTerrain"

#include <osgTerrain/GeometryTechnique>
#include <osgTerrain/TerrainTile>
#include <osgTerrain/Terrain>
#include <osgEarth/MapLayer>
#include <osgEarth/Cube>
#include <osgEarth/ImageUtils>

#include <osg/Program>
#include <osg/Shader>
#include <osg/Uniform>

#include <osgUtil/SmoothingVisitor>
#include <osgDB/FileUtils>
#include <osg/io_utils>
#include <osg/StateSet>
#include <osg/Texture2DArray>
#include <osg/Texture3D>
#include <osg/Texture2D>
#include <osg/Texture1D>
#include <osg/TexEnvCombine>
#include <osg/Program>
#include <osg/Math>
#include <osg/Timer>
#include <osg/Version>

#include <osgText/Text>
#include <sstream>
#include <osg/Depth>

#include <osg/ImageSequence>

using namespace osgTerrain;
using namespace osgEarth;
using namespace OpenThreads;

#define LC "[CompositingTechnique] "
#define EXPLICIT_RELEASE_GL_OBJECTS 1

// OSG 2.9.8 changed the osgTerrain API...
#if OSG_VERSION_GREATER_OR_EQUAL(2,9,8)
#   define USE_NEW_OSGTERRAIN_298_API 1 
#endif

// --------------------------------------------------------------------------

CompositingTerrainTechnique::CompositingTerrainTechnique( Locator* masterLocator ) :
_masterLocator( masterLocator ),
_currentReadOnlyBuffer(1),
_currentWriteBuffer(0),
_verticalScaleOverride(1.0f),
_swapPending( false ),
_initCount(0),
_optimizeTriangleOrientation(true),
_attachedProgram(false)
{
    this->setThreadSafeRefUnref(true);

    // create a texture compositor.
    _texCompositor = new TextureCompositor();

    // do this here so that we can use the program in the prototype
    _compositeProgram = _texCompositor->getProgram();
}

CompositingTerrainTechnique::CompositingTerrainTechnique(const CompositingTerrainTechnique& gt,const osg::CopyOp& copyop):
CustomTerrainTechnique(gt,copyop),
_masterLocator( gt._masterLocator ),
_lastCenterModel( gt._lastCenterModel ),
_currentReadOnlyBuffer( gt._currentReadOnlyBuffer ),
_currentWriteBuffer( gt._currentWriteBuffer ),
_verticalScaleOverride( gt._verticalScaleOverride ),
_swapPending( gt._swapPending ),
_initCount( gt._initCount ),
_optimizeTriangleOrientation(gt._optimizeTriangleOrientation),
_compositeProgram(gt._compositeProgram),
_texCompositor(gt._texCompositor)
{
    _attachedProgram = false;
    _bufferData[0] = gt._bufferData[0];
    _bufferData[1] = gt._bufferData[1];
}

CompositingTerrainTechnique::~CompositingTerrainTechnique()
{
    //nop
}

void
CompositingTerrainTechnique::setVerticalScaleOverride( float value )
{
    _verticalScaleOverride = value;
}

float
CompositingTerrainTechnique::getVerticalScaleOverride() const 
{
    return _verticalScaleOverride;
}

void
CompositingTerrainTechnique::setOptimizeTriangleOrientation(bool optimizeTriangleOrientation)
{
    _optimizeTriangleOrientation = optimizeTriangleOrientation;
}

bool
CompositingTerrainTechnique::getOptimizeTriangleOrientation() const
{
    return _optimizeTriangleOrientation;
}


void
CompositingTerrainTechnique::clearBuffer( int b )
{
    _bufferData[b]._transform = 0L;
    _bufferData[b]._geode = 0L;
    _bufferData[b]._surface = 0L;
    _bufferData[b]._skirt = 0L;
}

void CompositingTerrainTechnique::swapBuffers()
{
    std::swap(_currentReadOnlyBuffer,_currentWriteBuffer);
    clearBuffer( _currentWriteBuffer );
}

void
#ifdef USE_NEW_OSGTERRAIN_298_API
CompositingTerrainTechnique::init(int dirtyMask, bool assumeMultiThreaded)
#else
CompositingTerrainTechnique::init()
#endif
{
    init( true, 0L );
}

void
CompositingTerrainTechnique::init( bool swapNow, ProgressCallback* progress )
{
    // lock changes to the layers while we're rendering them
    Threading::ScopedReadLock lock( getMutex() );

    _initCount++;
    //if ( _initCount > 1 ) 
    //{
    //    OE_INFO << "tile init = " << _initCount << std::endl;
    //}

    // we cannot run this method is there is a swap currently pending!
    if ( _swapPending )
    {
        //TODO: figure out WHY this is happening in sequential mode after
        // the osgTerrain edge-normal calculation update.
        // http://www.osgearth.org/ticket/140

        OE_INFO << "illegal; cannot init() with a pending swap!" << std::endl;
        return;
    }

    //OE_INFO<<"Doing GeometryTechnique::init()"<<std::endl;
    
    if (!_terrainTile) return;

    //GW: this does not appear to do anything, since 2 threads should not be calling init()
    // at the same time. 2010-06-28
    //OpenThreads::ScopedLock<OpenThreads::Mutex> wbLock(_writeBufferMutex);

    if ( !_attachedProgram && _compositeProgram.valid() )
    {
        _terrainTile->getOrCreateStateSet()->setAttributeAndModes( _compositeProgram.get(), osg::StateAttribute::ON );
        _attachedProgram = true;
    }

    BufferData& buffer = getWriteBuffer();
    
    Locator* masterLocator = computeMasterLocator();
    
    osg::Vec3d centerModel = computeCenterModel(masterLocator);

    if ( progress && progress->isCanceled() )
    {
        clearBuffer( _currentWriteBuffer );
        return;
    }
    
    // create the geometry and texture coordinates for this tile.
    generateGeometry( masterLocator, centerModel );

    // create the texture for this tile.
    generateCompositeTexture();

    if (buffer._transform.valid())
        buffer._transform->setThreadSafeRefUnref(true);

    if ( progress && progress->isCanceled() )
    {
        clearBuffer( _currentWriteBuffer );
        return;
    }

    if ( swapNow )
        swapBuffers();

    _swapPending = !swapNow;
    
#ifdef USE_NEW_OSGTERRAIN_298_API
    // In the updated API, the technique is now responsible for clearing the dirty flag.
    // It used to be the tile that cleared it.
    _terrainTile->setDirtyMask(0);
#endif
}

bool
CompositingTerrainTechnique::swapIfNecessary()
{
    bool swapped = false;

    Threading::ScopedReadLock lock( getMutex() );
    if ( _swapPending )
    {        
        swapBuffers();
        swapped = true;
    }
    _swapPending = false;

    return swapped;
}

Locator*
CompositingTerrainTechnique::computeMasterLocator()
{
    if ( _masterLocator.valid() )
        return _masterLocator.get();

    osgTerrain::Layer* elevationLayer = _terrainTile->getElevationLayer();
    osgTerrain::Layer* colorLayer = _terrainTile->getColorLayer(0);

    Locator* elevationLocator = elevationLayer ? elevationLayer->getLocator() : 0;
    Locator* colorLocator = colorLayer ? colorLayer->getLocator() : 0;
    
    Locator* masterLocator = elevationLocator ? elevationLocator : colorLocator;
    if (!masterLocator)
    {
        OE_NOTICE<<"[osgEarth::CompositingTerrainTechnique] Problem, no locator found in any of the terrain layers"<<std::endl;
        return 0;
    }
    
    return masterLocator;
}

Threading::ReadWriteMutex&
CompositingTerrainTechnique::getMutex()
{
    return static_cast<CustomTile*>(_terrainTile)->getTileLayersMutex();
}

osg::Vec3d
CompositingTerrainTechnique::computeCenterModel(Locator* masterLocator)
{
    if (!masterLocator) return osg::Vec3d(0.0,0.0,0.0);

    BufferData& buffer = getWriteBuffer();
    
    osgTerrain::Layer* elevationLayer = _terrainTile->getElevationLayer();
    osgTerrain::Layer* colorLayer = _terrainTile->getColorLayer(0);

    Locator* elevationLocator = elevationLayer ? elevationLayer->getLocator() : 0;
    Locator* colorLocator = colorLayer ? colorLayer->getLocator() : 0;
    
    if (!elevationLocator) elevationLocator = masterLocator;
    if (!colorLocator) colorLocator = masterLocator;

    osg::Vec3d bottomLeftNDC(DBL_MAX, DBL_MAX, 0.0);
    osg::Vec3d topRightNDC(-DBL_MAX, -DBL_MAX, 0.0);
    
    if (elevationLayer)
    {
        if (elevationLocator!= masterLocator)
        {
            masterLocator->computeLocalBounds(*elevationLocator, bottomLeftNDC, topRightNDC);
        }
        else
        {
            bottomLeftNDC.x() = osg::minimum(bottomLeftNDC.x(), 0.0);
            bottomLeftNDC.y() = osg::minimum(bottomLeftNDC.y(), 0.0);
            topRightNDC.x() = osg::maximum(topRightNDC.x(), 1.0);
            topRightNDC.y() = osg::maximum(topRightNDC.y(), 1.0);
        }
    }

    if (colorLayer)
    {
        if (colorLocator!= masterLocator)
        {
            masterLocator->computeLocalBounds(*colorLocator, bottomLeftNDC, topRightNDC);
        }
        else
        {
            bottomLeftNDC.x() = osg::minimum(bottomLeftNDC.x(), 0.0);
            bottomLeftNDC.y() = osg::minimum(bottomLeftNDC.y(), 0.0);
            topRightNDC.x() = osg::maximum(topRightNDC.x(), 1.0);
            topRightNDC.y() = osg::maximum(topRightNDC.y(), 1.0);
        }
    }

//    OE_INFO<<"[osgEarth::CompositingTerrainTechnique] bottomLeftNDC = "<<bottomLeftNDC<<std::endl;
//    OE_INFO<<"[osgEarth::CompositingTerrainTechnique] topRightNDC = "<<topRightNDC<<std::endl;

    buffer._transform = new osg::MatrixTransform;

    osg::Vec3d centerNDC = (bottomLeftNDC + topRightNDC)*0.5;
    osg::Vec3d centerModel = (bottomLeftNDC + topRightNDC)*0.5;
    masterLocator->convertLocalToModel(centerNDC, centerModel);
    
    buffer._transform->setMatrix(osg::Matrix::translate(centerModel));
    
    _lastCenterModel = centerModel;
    return centerModel;
}

bool
CompositingTerrainTechnique::generateCompositeTexture()
{
    BufferData& buffer = getWriteBuffer();
    osg::StateSet* stateSet = buffer._geode->getOrCreateStateSet();

    // Composite all the image layer images into a single composite image.
    //
    // NOTE!
    // This should work if images are different sizes, BUT it will NOT work if they use
    // different locators. In other words, this will only work if the texture coordinate
    // pair (u,v) is the SAME across all image layers for a given vertex. That's because
    // GLSL will only support one tex-coord pair per texture unit, and we are doing the
    // compositing so we only need to use one texture unit.

    // establish the tile extent. we will calculate texture coordinate offset/scale based on this
    GeoExtent tileExtent;

    osg::ref_ptr<GeoLocator> tileLocator = dynamic_cast<GeoLocator*>( _terrainTile->getLocator() );
    if ( tileLocator.valid() )
    {
        if ( tileLocator->getCoordinateSystemType() == osgTerrain::Locator::GEOCENTRIC )
            tileLocator = tileLocator->getGeographicFromGeocentric();

        tileExtent = tileLocator->getDataExtent();
    }

    // find each image layer and create a region entry for it
    GeoImageList imageStack;
    unsigned int numColorLayers = _terrainTile->getNumColorLayers();

    for( unsigned int layerNum=0; layerNum < numColorLayers; ++layerNum )
    {
        // TODO: consider contour layers..
        osgTerrain::ImageLayer* imageLayer = dynamic_cast<osgTerrain::ImageLayer*>( _terrainTile->getColorLayer( layerNum ) );
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

                imageStack.push_back( new GeoImage( imageLayer->getImage(), layerExtent ) );
            }
        }
    }

    osg::StateSet* texStateSet = _texCompositor->createStateSet( imageStack, tileExtent );
    if ( texStateSet )
    {
        buffer._geode->setStateSet( texStateSet );
    }

    return true;
}

void
CompositingTerrainTechnique::calculateSampling( int& out_rows, int& out_cols, double& out_i, double& out_j )
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
CompositingTerrainTechnique::generateGeometry(Locator* masterLocator, const osg::Vec3d& centerModel)
{
    osg::ref_ptr< Locator > masterTextureLocator = masterLocator;
    GeoLocator* geoMasterLocator = dynamic_cast<GeoLocator*>(masterLocator);

	bool isCube = dynamic_cast<CubeFaceLocator*>(masterLocator) != NULL;

    //If we have a geocentric locator, get a geographic version of it to avoid converting
    //to/from geocentric when computing texture coordinats
    if (!isCube && geoMasterLocator && masterLocator->getCoordinateSystemType() == osgTerrain::Locator::GEOCENTRIC)
    {
        masterTextureLocator = geoMasterLocator->getGeographicFromGeocentric();
    }

    //osg::Timer_t before = osg::Timer::instance()->tick();
    BufferData& buffer = getWriteBuffer();
    
    osgTerrain::Layer* elevationLayer = _terrainTile->getElevationLayer();

    buffer._geode = new osg::Geode();
    buffer._geode->setThreadSafeRefUnref(true);

    buffer._transform->addChild( buffer._geode.get() );
    buffer._transform->setThreadSafeRefUnref(true);
    
    buffer._surface = new osg::Geometry;
    buffer._skirt = new osg::Geometry;

    // setting the geometry to DYNAMIC means its draw will not overlap the next frame's update/cull
    // traversal - which could access the buffer without a mutex
    buffer._surface->setDataVariance( osg::Object::DYNAMIC );
    buffer._skirt->setDataVariance( osg::Object::DYNAMIC );

    buffer._geode->addDrawable(buffer._surface.get());
    buffer._geode->addDrawable(buffer._skirt.get());
        
    osg::Geometry* surface = buffer._surface.get();
    surface->setThreadSafeRefUnref(true);

    osg::Geometry* skirt = buffer._skirt.get();
    skirt->setThreadSafeRefUnref(true);

    int numRows = 20;
    int numColumns = 20;
    
    if (elevationLayer)
    {
        numColumns = elevationLayer->getNumColumns();
        numRows = elevationLayer->getNumRows();
    }
    
    double i_sampleFactor, j_sampleFactor;
    calculateSampling( numColumns, numRows, i_sampleFactor, j_sampleFactor );

//    bool treatBoundariesToValidDataAsDefaultValue = _terrainTile->getTreatBoundariesToValidDataAsDefaultValue();
//    OE_INFO<<"[osgEarth::CompositingTerrainTechnique] TreatBoundariesToValidDataAsDefaultValue="<<treatBoundariesToValidDataAsDefaultValue<<std::endl;
    
    float skirtHeight = 0.0f;
    HeightFieldLayer* hfl = dynamic_cast<HeightFieldLayer*>(elevationLayer);
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
    osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array;
    if (normals.valid()) normals->reserve(numVerticesInSurface);
    surface->setNormalArray(normals.get());
    surface->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);

    // allocate and assign texture coordinates
    osg::Vec2Array* surfaceTexCoords = new osg::Vec2Array();
    surfaceTexCoords->reserve( numVerticesInSurface );
    surface->setTexCoordArray( 0, surfaceTexCoords );

    // skirt texture coordinates, if applicable:
    osg::Vec2Array* skirtTexCoords = 0L;
    if ( createSkirt )
    {
        skirtTexCoords = new osg::Vec2Array();
        skirtTexCoords->reserve( numVerticesInSkirt );
        skirt->setTexCoordArray( 0, skirtTexCoords );
    }

    //float minHeight = 0.0;
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
    unsigned int i, j;

    //osg::Timer_t populateBefore = osg::Timer::instance()->tick();
    for(j=0; j<numRows; ++j)
    {
        for(i=0; i<numColumns; ++i)
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
                indices[iv] = surfaceVerts->size();
            
                osg::Vec3d model;
                masterLocator->convertLocalToModel(ndc, model);

                (*surfaceVerts).push_back(model - centerModel);

                (*surfaceTexCoords).push_back( osg::Vec2( ndc.x(), ndc.y() ) );

                if (elevations.valid())
                {
                    (*elevations).push_back(ndc.z());
                }

                // compute the local normal
                osg::Vec3d ndc_one = ndc; ndc_one.z() += 1.0;
                osg::Vec3d model_one;
                masterLocator->convertLocalToModel(ndc_one, model_one);
                model_one = model_one - model;
                model_one.normalize();            
                (*normals).push_back(model_one);
            }
            else
            {
                indices[iv] = -1;
            }
        }
    }
    //osg::Timer_t populateAfter = osg::Timer::instance()->tick();

    //OE_NOTICE << "  PopulateTime " << osg::Timer::instance()->delta_m(populateBefore, populateAfter) << std::endl;
    
    // populate primitive sets
//    bool optimizeOrientations = elevations!=0;
    bool swapOrientation = !(masterLocator->orientationOpenGL());
    

    //osg::Timer_t genPrimBefore = osg::Timer::instance()->tick();
    osg::ref_ptr<osg::DrawElementsUInt> elements = new osg::DrawElementsUInt(GL_TRIANGLES);
    elements->reserve((numRows-1) * (numColumns-1) * 6);

    surface->addPrimitiveSet(elements.get());

    //osg::Timer_t genPrimAfter = osg::Timer::instance()->tick();
    //OE_NOTICE << "  genPrimTime " << osg::Timer::instance()->delta_m(genPrimBefore, genPrimAfter) << std::endl;
    
    osg::ref_ptr<osg::Vec3Array> skirtVectors = new osg::Vec3Array((*normals));
    
    if (!normals) createSkirt = false;


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
            skirtTexCoords->push_back( (*surfaceTexCoords)[orig_i] );
            skirtTexCoords->push_back( (*surfaceTexCoords)[orig_i] );
        }

        // right:
        for( int r=0; r<numRows-1; ++r )
        {
            int orig_i = indices[r*numColumns+(numColumns-1)];
            skirtVerts->push_back( (*surfaceVerts)[orig_i] );
            skirtVerts->push_back( (*surfaceVerts)[orig_i] - ((*skirtVectors)[orig_i])*skirtHeight );
            skirtTexCoords->push_back( (*surfaceTexCoords)[orig_i] );
            skirtTexCoords->push_back( (*surfaceTexCoords)[orig_i] );
        }

        // top:
        for( int c=numColumns-1; c>0; --c )
        {
            int orig_i = indices[(numRows-1)*numColumns+c];
            skirtVerts->push_back( (*surfaceVerts)[orig_i] );
            skirtVerts->push_back( (*surfaceVerts)[orig_i] - ((*skirtVectors)[orig_i])*skirtHeight );
            skirtTexCoords->push_back( (*surfaceTexCoords)[orig_i] );
            skirtTexCoords->push_back( (*surfaceTexCoords)[orig_i] );
        }

        // left:
        for( int r=numRows-1; r>=0; --r )
        {
            int orig_i = indices[r*numColumns];
            skirtVerts->push_back( (*surfaceVerts)[orig_i] );
            skirtVerts->push_back( (*surfaceVerts)[orig_i] - ((*skirtVectors)[orig_i])*skirtHeight );
            skirtTexCoords->push_back( (*surfaceTexCoords)[orig_i] );
            skirtTexCoords->push_back( (*surfaceTexCoords)[orig_i] );
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

    //osg::Timer_t skirtAfter = osg::Timer::instance()->tick();
    //OE_NOTICE << "  skirtTime " << osg::Timer::instance()->delta_m(skirtBefore, skirtAfter) << std::endl;

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
        buffer._geode->accept(*builder);
        //osg::Timer_t after = osg::Timer::instance()->tick();
        //OE_NOTICE<<"KdTree build time "<<osg::Timer::instance()->delta_m(before, after)<<std::endl;
    }
}

void
CompositingTerrainTechnique::update(osgUtil::UpdateVisitor* uv)
{
    if (_terrainTile) _terrainTile->osg::Group::traverse(*uv);
}


void
CompositingTerrainTechnique::cull(osgUtil::CullVisitor* cv)
{
    BufferData& buffer = getReadOnlyBuffer();

#if 0
    if (buffer._terrainTile) buffer._terrainTile->osg::Group::traverse(*cv);
#else
    if (buffer._transform.valid())
    {
        buffer._transform->accept(*cv);
    }
#endif    
}


void
CompositingTerrainTechnique::traverse(osg::NodeVisitor& nv)
{
    if (!_terrainTile) return;

    // if app traversal update the frame count.
    if (nv.getVisitorType()==osg::NodeVisitor::UPDATE_VISITOR)
    {
#if OSG_MIN_VERSION_REQUIRED(2,9,8)
        if (_terrainTile->getDirty()) _terrainTile->init(~0x0,true);
#else
        if (_terrainTile->getDirty()) _terrainTile->init();
#endif

        osgUtil::UpdateVisitor* uv = dynamic_cast<osgUtil::UpdateVisitor*>(&nv);
        if (uv)
        {
            update(uv);
            //return;
        }        
        
    }
    else if (nv.getVisitorType()==osg::NodeVisitor::CULL_VISITOR)
    {
        osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(&nv);
        if (cv)
        {
            cull(cv);
            return;
        }
        else
        {
            OE_WARN << "[ETT] CULL_VISITOR not a osgUtil::CullVisitor" << std::endl;
        }
    }

    // the code from here on accounts for user traversals (intersections, etc)
    if (_terrainTile->getDirty()) 
    {
        //OE_INFO<<"******* Doing init ***********"<<std::endl;
#if OSG_MIN_VERSION_REQUIRED(2,9,8)
        _terrainTile->init(~0x0, true);
#else
        _terrainTile->init();
#endif
    }

    BufferData& buffer = getReadOnlyBuffer();
    if (buffer._transform.valid()) buffer._transform->accept(nv);
}


void CompositingTerrainTechnique::cleanSceneGraph()
{
}

void
CompositingTerrainTechnique::releaseGLObjects(osg::State* state) const
{
    CompositingTerrainTechnique* ncThis = const_cast<CompositingTerrainTechnique*>(this);

    Threading::ScopedWriteLock lock( ncThis->getMutex() );

    if (_bufferData[0]._transform.valid())
    {
        _bufferData[0]._transform->releaseGLObjects(state);
    }
    if (_bufferData[1]._transform.valid())
    {
        _bufferData[1]._transform->releaseGLObjects(state);   
    }
}
