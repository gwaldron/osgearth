/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2009 Pelican Ventures, Inc.
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

#include <osgTerrain/GeometryTechnique>
#include <osgTerrain/TerrainTile>
#include <osgTerrain/Terrain>
#include <osgEarth/VersionedTerrain>
#include <osgEarth/MapLayer>
#include <osgEarth/Cube>
#include <osgEarth/ImageUtils>

#include <osg/Program>
#include <osg/Shader>
#include <osg/Uniform>

#include <osgEarth/CompositingTerrainTechnique>

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

#define LC "[osgEarth::CompositingTechnique] "
#define EXPLICIT_RELEASE_GL_OBJECTS 1

// OSG 2.9.8 changed the osgTerrain API...
#if OSG_VERSION_GREATER_OR_EQUAL(2,9,8)
#   define USE_NEW_OSGTERRAIN_298_API 1 
#endif

// --------------------------------------------------------------------------

struct SwitchedGroup : public osg::Group
{
    SwitchedGroup( int mask, bool positive )
        : _mask(mask), _positive(positive) { }

    void traverse( osg::NodeVisitor& nv )
    {
        bool match = nv.getTraversalMask() == _mask;
        //OE_INFO << std::hex << "mask=" << nv.getTraversalMask() << ",match=" << match << ", trav=" << (match==_positive) << std::endl;
        if ( match == _positive )
        {
            osg::Group::traverse( nv );
        }
    }

    int _mask;
    bool _positive;
};

// --------------------------------------------------------------------------

CompositingTerrainTechnique::LayerTexRegion::LayerTexRegion() :
_px(0), _py(0),
_pw(256), _ph(256),
_tx(0.0f), _ty(0.0f),
_tw(1.0f), _th(1.0f),
_xoffset(0.0f), _yoffset(0.0f),
_xscale(1.0f), _yscale(1.0f)
{
    //nop
}

// --------------------------------------------------------------------------

CompositingTerrainTechnique::CompositingTerrainTechnique( Locator* masterLocator ) :
TerrainTechnique(),
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

    // do this here so that we can use the program in the prototype
    initShaders();
}

CompositingTerrainTechnique::CompositingTerrainTechnique(const CompositingTerrainTechnique& gt,const osg::CopyOp& copyop):
TerrainTechnique(gt,copyop),
_masterLocator( gt._masterLocator ),
_lastCenterModel( gt._lastCenterModel ),
_currentReadOnlyBuffer( gt._currentReadOnlyBuffer ),
_currentWriteBuffer( gt._currentWriteBuffer ),
_verticalScaleOverride( gt._verticalScaleOverride ),
_swapPending( gt._swapPending ),
_initCount( gt._initCount ),
_optimizeTriangleOrientation(gt._optimizeTriangleOrientation),
_layerTexRegions(gt._layerTexRegions),
_compositeProgram(gt._compositeProgram)
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
    _bufferData[b]._simpleGeode = 0L;
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

    if ( !_attachedProgram )
    {
        _terrainTile->getOrCreateStateSet()->setAttributeAndModes( _compositeProgram.get(), 1 );
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

    generateComposite( _compositeImage, _layerTexRegions );
    
    generateGeometry( masterLocator, centerModel );
    
    //applyColorLayers();
    applyTexture();
    applyTransparency();

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

Locator* CompositingTerrainTechnique::computeMasterLocator()
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
    return static_cast<VersionedTile*>(_terrainTile)->getTileLayersMutex();
}

osg::Vec3d CompositingTerrainTechnique::computeCenterModel(Locator* masterLocator)
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
CompositingTerrainTechnique::generateComposite(osg::ref_ptr<osg::Image>& out_image,
                                               LayerTexRegionList&       out_regions )
{
    // Composite all the image layer images into a single composite image.
    //
    // NOTE!
    // This should work if images are different sizes, BUT it will NOT work if they use
    // different locators. In other words, this will only work if the texture coordinate
    // pair (u,v) is the SAME across all image layers for a given vertex. That's because
    // GLSL will only support one tex-coord pair per texture unit, and we are doing the
    // compositing so we only need to use one texture unit.

    // clear out the old
    out_image = 0L;
    out_regions.clear();

    int cx=0, cy=0;
    int maxRowHeight=0;
    int maxLegalWidth=1024, maxLegalHeight=1024; // hard coded for the moment..just testing

    int totalWidth=0, totalHeight=0;

    std::vector<osgTerrain::ImageLayer*> _imageLayers;

    GeoExtent tileExtent;

    // establish the tile locator. we will calculate texture coordinate offset/scale based on this
    osg::ref_ptr<GeoLocator> tileLocator = dynamic_cast<GeoLocator*>( _terrainTile->getLocator() );
    if ( tileLocator.valid() )
    {
        if ( tileLocator->getCoordinateSystemType() == osgTerrain::Locator::GEOCENTRIC )
            tileLocator = tileLocator->getGeographicFromGeocentric();

        tileExtent = tileLocator->getDataExtent();
    }

    // find each image layer and create a region entry for it
    GeoLocator* masterLocator = 0L;
    unsigned int numColorLayers = _terrainTile->getNumColorLayers();
    for( unsigned int layerNum=0; layerNum < numColorLayers; ++layerNum )
    {
        // TODO: consider contour layers..
        osgTerrain::ImageLayer* imageLayer = dynamic_cast<osgTerrain::ImageLayer*>( _terrainTile->getColorLayer( layerNum ) );
        if ( imageLayer )
        {
            // save these for later:
            _imageLayers.push_back( imageLayer );

            osg::Image* image = imageLayer->getImage();

            LayerTexRegion region;

            if ( cx + image->s() <= maxLegalWidth )
            {
                // append this tile to the current row
                region._px = cx;
                region._py = cy;
                region._pw = image->s(); 
                region._ph = image->t();
                if ( maxRowHeight < region._ph )
                    maxRowHeight = region._ph;
            }
            else
            {
                // ran out of width; start a new row
                cx = 0;
                cy += maxRowHeight;
                maxRowHeight = 0.0;
                region._px = cx;
                region._py = cy;
                region._pw = image->s();
                region._ph = image->t();
            }
            cx += region._pw;

            
            osg::ref_ptr<GeoLocator> layerLocator = dynamic_cast<GeoLocator*>( imageLayer->getLocator() );
            if ( layerLocator.valid() )
            {
                if ( layerLocator->getCoordinateSystemType() == osgTerrain::Locator::GEOCENTRIC )
                    layerLocator = layerLocator->getGeographicFromGeocentric();

                const GeoExtent& layerExtent = layerLocator->getDataExtent();

                region._xoffset = (tileExtent.xMin() - layerExtent.xMin()) / layerExtent.width();
                region._yoffset = (tileExtent.yMin() - layerExtent.yMin()) / layerExtent.height();

                region._xscale = tileExtent.width() / layerExtent.width();
                region._yscale = tileExtent.height() / layerExtent.height();
            }
            
            out_regions.push_back( region );

            totalWidth = osg::maximum( totalWidth, cx );
            totalHeight = osg::maximum( totalHeight, cy + maxRowHeight );
        }
    }

    // now, calculate the size of the composite image and allocate it.
    // TODO: account for different image pixel formats by converting everything to RGBA
    out_image = new osg::Image();
    out_image->allocateImage( totalWidth, totalHeight, 1, GL_RGBA, GL_UNSIGNED_BYTE );

    // build the uniforms.
    osg::StateSet* stateSet = _terrainTile->getOrCreateStateSet();
    
    // The uniform array contains 8 floats for each region:
    //   tx, ty : origin texture coordinates in the composite-image space
    //   tw, th : width and height in composite-image space
    //   xoff, yoff : x- and y- offsets within texture space
    //   xsca, ysca : x- and y- scale factors within texture space

    osg::Uniform* texInfoArray = new osg::Uniform( osg::Uniform::FLOAT, "osgearth_region", out_regions.size() * 8 );
    int p=0;
    for( unsigned int i=0; i<out_regions.size(); ++i )
    {
        LayerTexRegion& region = out_regions[i];
        osgTerrain::ImageLayer* layer = _imageLayers[i];

        // copy the image into the composite:
        ImageUtils::copyAsSubImage( layer->getImage(), out_image.get(), region._px, region._py );

        // next calculate the texture space extents and store those in uniforms.
        // (GW: there is no actual reason to store these in the region structure)
        region._tx = (float)region._px/(float)totalWidth;
        region._ty = (float)region._py/(float)totalHeight;
        region._tw = (float)region._pw/(float)totalWidth;
        region._th = (float)region._ph/(float)totalHeight;

        texInfoArray->setElement( p++, region._tx );
        texInfoArray->setElement( p++, region._ty );
        texInfoArray->setElement( p++, region._tw );
        texInfoArray->setElement( p++, region._th );
        texInfoArray->setElement( p++, region._xoffset );
        texInfoArray->setElement( p++, region._yoffset );
        texInfoArray->setElement( p++, region._xscale );
        texInfoArray->setElement( p++, region._yscale );
    }

    stateSet->addUniform( texInfoArray );
    stateSet->getOrCreateUniform( "osgearth_region_count", osg::Uniform::INT )->set( (int)out_regions.size() );

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

    buffer._simpleGeode = new osg::Geode();
    buffer._simpleGeode->setThreadSafeRefUnref(true);

    if(buffer._transform.valid())
    {
        osg::Group* geodeGroup = new SwitchedGroup( 0xf0f0f0f0, false );
        geodeGroup->setDataVariance( osg::Object::DYNAMIC );
        geodeGroup->addChild( buffer._geode.get() );
        buffer._transform->addChild( geodeGroup ); //buffer._geode.get());

        osg::Group* simpleGroup = new SwitchedGroup( 0xf0f0f0f0, true );
        simpleGroup->setDataVariance( osg::Object::DYNAMIC );
        simpleGroup->addChild( buffer._simpleGeode.get() );
        buffer._transform->addChild( simpleGroup ); //buffer._simpleGeode.get());

        buffer._transform->setThreadSafeRefUnref(true);
    }
    
    buffer._surface = new osg::Geometry;
    buffer._skirt = new osg::Geometry;

    // setting the geometry to DYNAMIC means its draw will not overlap the next frame's update/cull
    // traversal - which could access the buffer without a mutex
    buffer._surface->setDataVariance( osg::Object::DYNAMIC );
    buffer._skirt->setDataVariance( osg::Object::DYNAMIC );

    buffer._geode->addDrawable(buffer._surface.get());
    buffer._geode->addDrawable(buffer._skirt.get());

    buffer._simpleGeode->addDrawable(buffer._surface.get());
        
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
    

    //float minHeight = 0.0;
    float scaleHeight = 
        _verticalScaleOverride != 1.0? _verticalScaleOverride :
        _terrainTile->getTerrain() ? _terrainTile->getTerrain()->getVerticalScale() :
        1.0f;

    // allocate and assign tex coords
    struct TexCoordData {
        osg::ref_ptr<osg::Vec2Array> _surface;
        osg::ref_ptr<osg::Vec2Array> _skirt;
        osg::ref_ptr<Locator>        _locator;
        int                          _layerNum;
    };
    typedef std::map<Layer*, TexCoordData> LayerToTexCoordMap;
    //typedef std::pair< osg::ref_ptr<osg::Vec2Array>, osg::ref_ptr<Locator> > TexCoordLocatorPair;
    //typedef std::map< Layer*, TexCoordLocatorPair > LayerToTexCoordMap;

    LayerToTexCoordMap layerToTexCoordMap;
    for(unsigned int layerNum=0; layerNum<_terrainTile->getNumColorLayers(); ++layerNum)
    {
        osgTerrain::Layer* colorLayer = _terrainTile->getColorLayer(layerNum);
        if (colorLayer)
        {
            LayerToTexCoordMap::iterator itr = layerToTexCoordMap.find(colorLayer);
            if (itr!=layerToTexCoordMap.end())
            {
                surface->setTexCoordArray(layerNum, itr->second._surface.get()); //second.first.get());
            }
            else
            {

                Locator* locator = colorLayer->getLocator();
                
				TexCoordData& tclp = layerToTexCoordMap[colorLayer];
                
                tclp._layerNum = layerNum;

                tclp._surface = new osg::Vec2Array;
                tclp._surface->reserve( numVerticesInSurface );
                tclp._skirt = new osg::Vec2Array;
                tclp._skirt->reserve( numVerticesInSkirt );

                if (!isCube && locator && locator->getCoordinateSystemType() == Locator::GEOCENTRIC)
                {
                    GeoLocator* geo = dynamic_cast<GeoLocator*>(locator);
                    if (geo)
                    {
                        locator = geo->getGeographicFromGeocentric();
                    }
                }
                tclp._locator = locator ? locator : masterTextureLocator.get();

                surface->setTexCoordArray(layerNum, tclp._surface.get());
                skirt->setTexCoordArray(layerNum, tclp._skirt.get());
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

                for(LayerToTexCoordMap::iterator itr = layerToTexCoordMap.begin();
                    itr != layerToTexCoordMap.end();
                    ++itr)
                {
                    TexCoordData& tcdata = itr->second;
                    
                    osg::Vec2Array*         texcoords (tcdata._surface.get()); //second.first.get());
                    osgTerrain::ImageLayer* imageLayer(dynamic_cast<osgTerrain::ImageLayer*>(itr->first));

                    if (imageLayer != NULL)
                    {
#if 0
                      osgTerrain::Locator* colorLocator(tcdata._locator.get()); //.second);

                      if (colorLocator != masterLocator)
                      {
                          osg::Vec3d color_ndc;

                          Locator::convertLocalCoordBetween(*masterTextureLocator.get(), ndc, *colorLocator, color_ndc);
                          (*texcoords).push_back( osg::Vec2( color_ndc.x(), color_ndc.y() ) );
                          //osg::Vec2 compTexCoord = _layerTexRegions[tcdata._layerNum].transform( color_ndc.x(), color_ndc.y() );
                          //(*texcoords).push_back( compTexCoord );
                      }
                      else
#endif
                      {
                          (*texcoords).push_back( osg::Vec2( ndc.x(), ndc.y() ) );
                          //osg::Vec2 compTexCoord = _layerTexRegions[tcdata._layerNum].transform( ndc.x(), ndc.y() );
                          //(*texcoords).push_back( compTexCoord );
                      }
                    }
                    else
                    {
                      osgTerrain::ContourLayer* contourLayer(dynamic_cast<osgTerrain::ContourLayer*>(itr->first));

                      if (contourLayer != NULL)
                      {
                        osg::TransferFunction1D const*const transferFunction = contourLayer->getTransferFunction();
                        osg::Vec3d                          color_ndc;
                        osgTerrain::Locator*                colorLocator(tcdata._locator.get()); //.second);
                        
                        if (colorLocator != masterTextureLocator)
                          Locator::convertLocalCoordBetween(*masterTextureLocator.get(),ndc,*colorLocator,color_ndc);
                        else
                          color_ndc = ndc;
                        color_ndc[2] /= scaleHeight;
                        if (transferFunction != NULL)
                        {
                          float const difference = transferFunction->getMaximum()-transferFunction->getMinimum();
                          
                          //TODO
                          if (difference != 0)
                            (*texcoords).push_back(osg::Vec2((color_ndc[2]-transferFunction->getMinimum())/difference,0.0f));
                          else
                            (*texcoords).push_back(osg::Vec2(0.0f,0.0f));
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


#if 1

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

            for(LayerToTexCoordMap::iterator i = layerToTexCoordMap.begin(); i != layerToTexCoordMap.end(); ++i ) {
                i->second._skirt->push_back( (*i->second._surface.get())[orig_i] );
                i->second._skirt->push_back( (*i->second._surface.get())[orig_i] );
            }
        }

        // right:
        for( int r=0; r<numRows-1; ++r )
        {
            int orig_i = indices[r*numColumns+(numColumns-1)];
            skirtVerts->push_back( (*surfaceVerts)[orig_i] );
            skirtVerts->push_back( (*surfaceVerts)[orig_i] - ((*skirtVectors)[orig_i])*skirtHeight );

            for(LayerToTexCoordMap::iterator i = layerToTexCoordMap.begin(); i != layerToTexCoordMap.end(); ++i ) {
                i->second._skirt->push_back( (*i->second._surface.get())[orig_i] );
                i->second._skirt->push_back( (*i->second._surface.get())[orig_i] );
            }
        }

        // top:
        for( int c=numColumns-1; c>0; --c )
        {
            int orig_i = indices[(numRows-1)*numColumns+c];
            skirtVerts->push_back( (*surfaceVerts)[orig_i] );
            skirtVerts->push_back( (*surfaceVerts)[orig_i] - ((*skirtVectors)[orig_i])*skirtHeight );

            for(LayerToTexCoordMap::iterator i = layerToTexCoordMap.begin(); i != layerToTexCoordMap.end(); ++i ) {
                i->second._skirt->push_back( (*i->second._surface.get())[orig_i] );
                i->second._skirt->push_back( (*i->second._surface.get())[orig_i] );
            }
        }

        // left:
        for( int r=numRows-1; r>=0; --r )
        {
            int orig_i = indices[r*numColumns];
            skirtVerts->push_back( (*surfaceVerts)[orig_i] );
            skirtVerts->push_back( (*surfaceVerts)[orig_i] - ((*skirtVectors)[orig_i])*skirtHeight );

            for(LayerToTexCoordMap::iterator i = layerToTexCoordMap.begin(); i != layerToTexCoordMap.end(); ++i ) {
                i->second._skirt->push_back( (*i->second._surface.get())[orig_i] );
                i->second._skirt->push_back( (*i->second._surface.get())[orig_i] );
            }
        }

        skirt->setVertexArray( skirtVerts );
        skirt->addPrimitiveSet( new osg::DrawArrays( GL_TRIANGLE_STRIP, 0, skirtVerts->size() ) );
    }



#else


    //osg::Timer_t skirtBefore = osg::Timer::instance()->tick();
    if (createSkirt)
    {
        osg::ref_ptr<osg::DrawElementsUShort> skirtDrawElements = new osg::DrawElementsUShort(GL_QUAD_STRIP);

        // create bottom skirt vertices
        int r,c;
        r=0;
        for(c=0;c<static_cast<int>(numColumns);++c)
        {
            int orig_i = indices[(r)*numColumns+c]; // index of original vertex of grid
            if (orig_i>=0)
            {
                unsigned int new_i = surfaceVerts->size(); // index of new index of added skirt point
                osg::Vec3 new_v = (*surfaceVerts)[orig_i] - ((*skirtVectors)[orig_i])*skirtHeight;
                (*surfaceVerts).push_back(new_v);
                if (normals.valid()) (*normals).push_back((*normals)[orig_i]);

                for(LayerToTexCoordMap::iterator itr = layerToTexCoordMap.begin();
                    itr != layerToTexCoordMap.end();
                    ++itr)
                {
                    itr->second._surface->push_back((*itr->second._surface.get())[orig_i]);
                }
                
                skirtDrawElements->push_back(orig_i);
                skirtDrawElements->push_back(new_i);
            }
            else
            {
                if (!skirtDrawElements->empty())
                {
                    surface->addPrimitiveSet(skirtDrawElements.get());
                    skirtDrawElements = new osg::DrawElementsUShort(GL_QUAD_STRIP);
                }
                
            }
        }

        if (!skirtDrawElements->empty())
        {
            surface->addPrimitiveSet(skirtDrawElements.get());
            skirtDrawElements = new osg::DrawElementsUShort(GL_QUAD_STRIP);
        }

        // create right skirt vertices
        c=numColumns-1;
        for(r=0;r<static_cast<int>(numRows);++r)
        {
            int orig_i = indices[(r)*numColumns+c]; // index of original vertex of grid
            if (orig_i>=0)
            {
                unsigned int new_i = surfaceVerts->size(); // index of new index of added skirt point
                osg::Vec3 new_v = (*surfaceVerts)[orig_i] - ((*skirtVectors)[orig_i])*skirtHeight;
                (*surfaceVerts).push_back(new_v);
                if (normals.valid()) (*normals).push_back((*normals)[orig_i]);
                for(LayerToTexCoordMap::iterator itr = layerToTexCoordMap.begin();
                    itr != layerToTexCoordMap.end();
                    ++itr)
                {
                    itr->second._surface->push_back((*itr->second._surface.get())[orig_i]);
                    //itr->second.first->push_back((*itr->second.first)[orig_i]);
                }
                
                skirtDrawElements->push_back(orig_i);
                skirtDrawElements->push_back(new_i);
            }
            else
            {
                if (!skirtDrawElements->empty())
                {
                    surface->addPrimitiveSet(skirtDrawElements.get());
                    skirtDrawElements = new osg::DrawElementsUShort(GL_QUAD_STRIP);
                }
                
            }
        }

        if (!skirtDrawElements->empty())
        {
            surface->addPrimitiveSet(skirtDrawElements.get());
            skirtDrawElements = new osg::DrawElementsUShort(GL_QUAD_STRIP);
        }

        // create top skirt vertices
        r=numRows-1;
        for(c=numColumns-1;c>=0;--c)
        {
            int orig_i = indices[(r)*numColumns+c]; // index of original vertex of grid
            if (orig_i>=0)
            {
                unsigned int new_i = surfaceVerts->size(); // index of new index of added skirt point
                osg::Vec3 new_v = (*surfaceVerts)[orig_i] - ((*skirtVectors)[orig_i])*skirtHeight;
                (*surfaceVerts).push_back(new_v);
                if (normals.valid()) (*normals).push_back((*normals)[orig_i]);
                for(LayerToTexCoordMap::iterator itr = layerToTexCoordMap.begin();
                    itr != layerToTexCoordMap.end();
                    ++itr)
                {
                    itr->second._surface->push_back((*itr->second._surface.get())[orig_i]);
                    //itr->second.first->push_back((*itr->second.first)[orig_i]);
                }
                
                skirtDrawElements->push_back(orig_i);
                skirtDrawElements->push_back(new_i);
            }
            else
            {
                if (!skirtDrawElements->empty())
                {
                    surface->addPrimitiveSet(skirtDrawElements.get());
                    skirtDrawElements = new osg::DrawElementsUShort(GL_QUAD_STRIP);
                }
                
            }
        }

        if (!skirtDrawElements->empty())
        {
            surface->addPrimitiveSet(skirtDrawElements.get());
            skirtDrawElements = new osg::DrawElementsUShort(GL_QUAD_STRIP);
        }

        // create left skirt vertices
        c=0;
        for(r=numRows-1;r>=0;--r)
        {
            int orig_i = indices[(r)*numColumns+c]; // index of original vertex of grid
            if (orig_i>=0)
            {
                unsigned int new_i = surfaceVerts->size(); // index of new index of added skirt point
                osg::Vec3 new_v = (*surfaceVerts)[orig_i] - ((*skirtVectors)[orig_i])*skirtHeight;
                (*surfaceVerts).push_back(new_v);
                if (normals.valid()) (*normals).push_back((*normals)[orig_i]);
                for(LayerToTexCoordMap::iterator itr = layerToTexCoordMap.begin();
                    itr != layerToTexCoordMap.end();
                    ++itr)
                {
                    itr->second._surface->push_back((*itr->second._surface.get())[orig_i]);
                    //itr->second.first->push_back((*itr->second.first)[orig_i]);
                }
                
                skirtDrawElements->push_back(orig_i);
                skirtDrawElements->push_back(new_i);
            }
            else
            {
                if (!skirtDrawElements->empty())
                {
                    surface->addPrimitiveSet(skirtDrawElements.get());
                    skirtDrawElements = new osg::DrawElementsUShort(GL_QUAD_STRIP);
                }
                
            }
        }

        if (!skirtDrawElements->empty())
        {
            surface->addPrimitiveSet(skirtDrawElements.get());
            skirtDrawElements = new osg::DrawElementsUShort(GL_QUAD_STRIP);
        }
    }


#endif // skirts


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


    //geometry->setUseDisplayList(false);
    surface->setUseVertexBufferObjects(true);
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

    

    //DEBUGGING
#ifdef DEBUG_SHOW_TILEKEY_LABELS

    static osgText::Font* s_font = osgText::readFontFile( "arialbd.ttf" );
    osgText::Text* text = new osgText::Text();
    text->setThreadSafeRefUnref( true );
    text->setDataVariance( osg::Object::DYNAMIC );
    text->setFont( s_font );
    std::stringstream buf;
    buf << "" << _terrainTile->getTileID().level << "(" <<_terrainTile->getTileID().x << "," << _terrainTile->getTileID().y << ")" << std::endl
        << "elev=" << static_cast<VersionedTile*>(_terrainTile)->getElevationLOD() << std::endl;

        buf << "imglod" << std::endl;
    for (unsigned int i = 0; i < _terrainTile->getNumColorLayers(); ++i)
    {
        TransparentLayer* tl = dynamic_cast<TransparentLayer*>( _terrainTile->getColorLayer(i) );
        if (tl)
        {
            buf << tl->getId() << "=" << tl->getLevelOfDetail() << std::endl;
        }
    }

	std::string bufStr;
	bufStr = buf.str();
    text->setText( bufStr );
    //text->setFont( s_font );
    //text->setFont( osgText::readFontFile( "arialbd.ttf" ) );
    text->setCharacterSizeMode( osgText::Text::SCREEN_COORDS );
    text->setCharacterSize( 32 );
    text->setColor( osg::Vec4f(1,1,1,1) );
    text->setBackdropType( osgText::Text::OUTLINE );
    text->setAutoRotateToScreen(true);
    text->getOrCreateStateSet()->setAttributeAndModes( new osg::Depth(osg::Depth::ALWAYS), osg::StateAttribute::ON );
    
    buffer._geode->addDrawable( text );
#endif

    //osg::Timer_t after = osg::Timer::instance()->tick();
    //osg::notify( osg::NOTICE ) << "generateGeometryTime " << osg::Timer::instance()->delta_m(before, after) << std::endl;
}

void
CompositingTerrainTechnique::applyTexture()
{
    BufferData& buffer = getWriteBuffer();

    osg::StateSet* stateSet = buffer._geode->getOrCreateStateSet();
    osg::Texture* texture = new osg::Texture2D( _compositeImage.get() );

    texture->setMaxAnisotropy(16.0f);
    texture->setResizeNonPowerOfTwoHint(false);

    texture->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR );
    texture->setFilter( osg::Texture::MAG_FILTER, osg::Texture::LINEAR );

    texture->setWrap(osg::Texture::WRAP_S,osg::Texture::CLAMP_TO_EDGE);
    texture->setWrap(osg::Texture::WRAP_T,osg::Texture::CLAMP_TO_EDGE);
    texture->setWrap(osg::Texture::WRAP_R,osg::Texture::REPEAT);

    bool mipMapping = !(texture->getFilter(osg::Texture::MIN_FILTER)==osg::Texture::LINEAR || texture->getFilter(osg::Texture::MIN_FILTER)==osg::Texture::NEAREST);
    bool s_NotPowerOfTwo = _compositeImage->s()==0 || (_compositeImage->s() & (_compositeImage->s() - 1));
    bool t_NotPowerOfTwo = _compositeImage->t()==0 || (_compositeImage->t() & (_compositeImage->t() - 1));

    if (mipMapping && (s_NotPowerOfTwo || t_NotPowerOfTwo))
    {
        OE_DEBUG<<"[osgEarth::CompositingTerrainTechnique] Disabling mipmapping for non power of two tile size("
            <<_compositeImage->s()<<", "<<_compositeImage->t()<<")"<<std::endl;
        texture->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR);
    }

    stateSet->setTextureAttributeAndModes(0, texture, osg::StateAttribute::ON);

    // done with the composite image - can unref it to free the memory.
    _compositeImage = 0L;
}

#if 0
void CompositingTerrainTechnique::applyColorLayers()
{
    BufferData& buffer = getWriteBuffer();

    typedef std::map<osgTerrain::Layer*, osg::Texture*> LayerToTextureMap;
    LayerToTextureMap layerToTextureMap;
    
    for(unsigned int layerNum=0; layerNum<_terrainTile->getNumColorLayers(); ++layerNum)
    {
        osgTerrain::Layer* colorLayer = _terrainTile->getColorLayer(layerNum);
        if (!colorLayer) continue;

        osg::Image* image = colorLayer->getImage();
        if (!image) continue;

        osgTerrain::ImageLayer* imageLayer = dynamic_cast<osgTerrain::ImageLayer*>(colorLayer);
        osgTerrain::ContourLayer* contourLayer = dynamic_cast<osgTerrain::ContourLayer*>(colorLayer);
        if (imageLayer)
        {
            osg::StateSet* stateset = buffer._geode->getOrCreateStateSet();

            osg::Texture* texture = dynamic_cast<osg::Texture*>( layerToTextureMap[colorLayer] );

            //osg::Texture2D* texture2D = dynamic_cast<osg::Texture2D*>(layerToTextureMap[colorLayer]);
            if ( !texture ) // (!texture2D)
            {   
                osg::Texture2D* t2d = new osg::Texture2D();
                t2d->setImage( image );
                texture = t2d;

                // ImageSequence requires an update traversal, and the terrain tile will not 
                // automatically get one when it's under a terrain technique. So we need to
                // manually bump it here. (TBD: if the layer is removed we need to manually
                // decremenet this.)
                if ( dynamic_cast<osg::ImageSequence*>( image ) )
                {
                    //TODO: this is a totally un-threasd-safe hack!! fix it!!
                    static_cast<VersionedTile*>(_terrainTile)->adjustUpdateTraversalCount( 1 );
                }

                texture->setMaxAnisotropy(16.0f);
                texture->setResizeNonPowerOfTwoHint(false);

#if OSG_MIN_VERSION_REQUIRED(2,8,0)
                texture->setFilter(osg::Texture::MIN_FILTER, colorLayer->getMinFilter());
                texture->setFilter(osg::Texture::MAG_FILTER, colorLayer->getMagFilter());
#else
				texture2D->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR);
				texture2D->setFilter(osg::Texture::MAG_FILTER, colorLayer->getFilter()==Layer::LINEAR ? osg::Texture::LINEAR :  osg::Texture::NEAREST);
#endif
                
                texture->setWrap(osg::Texture::WRAP_S,osg::Texture::CLAMP_TO_EDGE);
                texture->setWrap(osg::Texture::WRAP_T,osg::Texture::CLAMP_TO_EDGE);
                texture->setWrap(osg::Texture::WRAP_R,osg::Texture::REPEAT);

                bool mipMapping = !(texture->getFilter(osg::Texture::MIN_FILTER)==osg::Texture::LINEAR || texture->getFilter(osg::Texture::MIN_FILTER)==osg::Texture::NEAREST);
                bool s_NotPowerOfTwo = image->s()==0 || (image->s() & (image->s() - 1));
                bool t_NotPowerOfTwo = image->t()==0 || (image->t() & (image->t() - 1));

                if (mipMapping && (s_NotPowerOfTwo || t_NotPowerOfTwo))
                {
                    OE_DEBUG<<"[osgEarth::CompositingTerrainTechnique] Disabling mipmapping for non power of two tile size("<<image->s()<<", "<<image->t()<<")"<<std::endl;
                    texture->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR);
                }

                layerToTextureMap[colorLayer] = texture;
                //texture = texture2D;

                // OE_NOTICE<<"Creating new ImageLayer texture "<<layerNum<<" image->s()="<<image->s()<<"  image->t()="<<image->t()<<std::endl;

            }
            else
            {
                // OE_NOTICE<<"Reusing ImageLayer texture "<<layerNum<<std::endl;
            }

            //stateset->setTextureAttributeAndModes(layerNum, texture2D, osg::StateAttribute::ON);
            stateset->setTextureAttributeAndModes(layerNum, texture, osg::StateAttribute::ON);
            
        }
        else if (contourLayer)
        {
            osg::StateSet* stateset = buffer._geode->getOrCreateStateSet();

            osg::Texture1D* texture1D = dynamic_cast<osg::Texture1D*>(layerToTextureMap[colorLayer]);
            if (!texture1D)
            {
                texture1D = new osg::Texture1D;
                texture1D->setImage(image);
                texture1D->setResizeNonPowerOfTwoHint(false);
#if OSG_MIN_VERSION_REQUIRED(2,8,0)
                texture1D->setFilter(osg::Texture::MIN_FILTER, osg::Texture::NEAREST);
                texture1D->setFilter(osg::Texture::MAG_FILTER, colorLayer->getMagFilter());
#else
				texture1D->setFilter(osg::Texture::MIN_FILTER, osg::Texture::NEAREST);
                texture1D->setFilter(osg::Texture::MAG_FILTER, colorLayer->getFilter()==Layer::LINEAR ? osg::Texture::LINEAR :  osg::Texture::NEAREST);
#endif
                layerToTextureMap[colorLayer] = texture1D;
            }
            
            stateset->setTextureAttributeAndModes(layerNum, texture1D, osg::StateAttribute::ON);

        }
    }

    // copy it over
    buffer._simpleGeode->setStateSet( buffer._geode->getStateSet() );
}
#endif

void CompositingTerrainTechnique::applyTransparency()
{
    BufferData& buffer = getWriteBuffer();
    
    bool containsTransparency = false;
    for(unsigned int i=0; i<_terrainTile->getNumColorLayers(); ++i)
    {
         osg::Image* image = (_terrainTile->getColorLayer(i)!=0) ? _terrainTile->getColorLayer(i)->getImage() : 0; 
        if (image)
        {
            containsTransparency = image->isImageTranslucent();
            break;
        }        
    }
    
    if (containsTransparency)
    {
        osg::StateSet* stateset = buffer._geode->getOrCreateStateSet();
        stateset->setMode(GL_BLEND, osg::StateAttribute::ON);
        stateset->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    }

}

void CompositingTerrainTechnique::smoothGeometry()
{
    BufferData& buffer = getWriteBuffer();

    //osg::Timer_t before = osg::Timer::instance()->tick();
    
    if (buffer._surface.valid())
    {
        osgUtil::SmoothingVisitor smoother;
        smoother.smooth(*buffer._surface);
    }

    //osg::Timer_t after = osg::Timer::instance()->tick();

    //OE_NOTICE << "Smooth time " << osg::Timer::instance()->delta_m(before, after) << std::endl;
}

void CompositingTerrainTechnique::update(osgUtil::UpdateVisitor* uv)
{
    if (_terrainTile) _terrainTile->osg::Group::traverse(*uv);
}


void CompositingTerrainTechnique::cull(osgUtil::CullVisitor* cv)
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


void CompositingTerrainTechnique::traverse(osg::NodeVisitor& nv)
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


static char source_vertMain[] =

"varying vec3 normal, lightDir, halfVector; \n"

"void main(void) \n"
"{ \n"
"    gl_TexCoord[0] = gl_MultiTexCoord0; \n"
"    gl_Position = ftransform(); \n"
"} \n";

static char source_fragMain[] =

"varying vec3 normal, lightDir, halfVector; \n"

"uniform float osgearth_region[256]; \n"
"uniform int   osgearth_region_count; \n"
"uniform sampler2D tex0; \n"

"uniform float osgearth_imagelayer_opacity[128]; \n"

"void main(void) \n"
"{ \n"
"    vec3 color = vec3(1,1,1); \n"
"    for(int i=0; i<osgearth_region_count; i++) \n"
"    { \n"
"        int r = 8*i; \n"
"        float tx   = osgearth_region[r];   \n"
"        float ty   = osgearth_region[r+1]; \n"
"        float tw   = osgearth_region[r+2]; \n"
"        float th   = osgearth_region[r+3]; \n"
"        float xoff = osgearth_region[r+4]; \n"
"        float yoff = osgearth_region[r+5]; \n"
"        float xsca = osgearth_region[r+6]; \n"
"        float ysca = osgearth_region[r+7]; \n"

"        float opac = osgearth_imagelayer_opacity[i]; \n"

"        float u = tx + ( xoff + xsca * gl_TexCoord[0].s ) * tw; \n"
"        float v = ty + ( yoff + ysca * gl_TexCoord[0].t ) * th; \n"

"        vec4 texel = texture2D( tex0, vec2(u,v) ); \n"
"        color = mix(color, texel.rgb, texel.a * opac); \n"
"    } \n"
"    gl_FragColor = vec4(color, 1); \n"
"} \n";


void
CompositingTerrainTechnique::initShaders()
{
    _compositeProgram = new osg::Program();

    osg::Shader* vert = new osg::Shader( osg::Shader::VERTEX, std::string(source_vertMain) );
    _compositeProgram->addShader( vert );

    osg::Shader* frag = new osg::Shader( osg::Shader::FRAGMENT, std::string(source_fragMain) );
    _compositeProgram->addShader( frag );

    OE_INFO << LC << "Initialized shaders." << std::endl;
}
