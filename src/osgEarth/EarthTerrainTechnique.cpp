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

#include <osgEarth/EarthTerrainTechnique>

#include <osgUtil/SmoothingVisitor>

#include <osgDB/FileUtils>

#include <osg/io_utils>
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

using namespace osgTerrain;
using namespace osgEarth;
using namespace OpenThreads;

#define NEW_COORD_CODE


EarthTerrainTechnique::EarthTerrainTechnique( Locator* masterLocator ) :
TerrainTechnique(),
_masterLocator( masterLocator ),
_currentReadOnlyBuffer(1),
_currentWriteBuffer(0),
_verticalScaleOverride(1.0f),
_swapPending( false )
{
    this->setThreadSafeRefUnref(true);
}

EarthTerrainTechnique::EarthTerrainTechnique(const EarthTerrainTechnique& gt,const osg::CopyOp& copyop):
TerrainTechnique(gt,copyop),
_masterLocator( gt._masterLocator ),
_lastCenterModel( gt._lastCenterModel ),
_currentReadOnlyBuffer( gt._currentReadOnlyBuffer ),
_currentWriteBuffer( gt._currentWriteBuffer ),
_verticalScaleOverride( gt._verticalScaleOverride ),
_swapPending( gt._swapPending )
{
    _bufferData[0] = gt._bufferData[0];
    _bufferData[1] = gt._bufferData[1];
}

EarthTerrainTechnique::~EarthTerrainTechnique()
{
}

void
EarthTerrainTechnique::setVerticalScaleOverride( float value )
{
    _verticalScaleOverride = value;
}

float
EarthTerrainTechnique::getVerticalScaleOverride() const 
{
    return _verticalScaleOverride;
}

void
EarthTerrainTechnique::clearBuffer( int b )
{
    _bufferData[b]._transform = 0L;
    _bufferData[b]._geode = 0L;
    _bufferData[b]._geometry = 0L;
}

void EarthTerrainTechnique::swapBuffers()
{
    std::swap(_currentReadOnlyBuffer,_currentWriteBuffer);
    clearBuffer( _currentWriteBuffer );
}

void
EarthTerrainTechnique::init()
{
    init( true, 0L );
}

void
EarthTerrainTechnique::init( bool swapNow, ProgressCallback* progress )
{
    // lock changes to the layers while we're rendering them
    ScopedReadLock lock( getMutex() );

    // we cannot run this method is there is a swap currently pending!
    if ( _swapPending )
    {
        osg::notify(osg::NOTICE) << "[osgEarth] illegal; cannot init() with a pending swap!" << std::endl;
        return;
    }

    //osg::notify(osg::INFO)<<"Doing GeometryTechnique::init()"<<std::endl;
    
    if (!_terrainTile) return;

    BufferData& buffer = getWriteBuffer();
    
    Locator* masterLocator = computeMasterLocator();
    
    osg::Vec3d centerModel = computeCenterModel(masterLocator);

    if ( progress && progress->isCanceled() )
    {
        clearBuffer( _currentWriteBuffer );
        return;
    }
    
    generateGeometry( masterLocator, centerModel );
    
    applyColorLayers();
    applyTransparency();
    
// don't call this here
//    smoothGeometry();

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
}

bool
EarthTerrainTechnique::swapIfNecessary()
{
    bool swapped = false;

    ScopedReadLock lock( getMutex() );
    if ( _swapPending )
    {
        swapBuffers();
        swapped = true;
    }
    _swapPending = false;

    return swapped;
}

Locator* EarthTerrainTechnique::computeMasterLocator()
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
        osg::notify(osg::NOTICE)<<"[osgEarth::EarthTerrainTechnique] Problem, no locator found in any of the terrain layers"<<std::endl;
        return 0;
    }
    
    return masterLocator;
}

ReadWriteMutex&
EarthTerrainTechnique::getMutex()
{
    return static_cast<VersionedTile*>(_terrainTile)->getTileLayersMutex();
}

osg::Vec3d EarthTerrainTechnique::computeCenterModel(Locator* masterLocator)
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

    osg::notify(osg::INFO)<<"[osgEarth::EarthTerrainTechnique] bottomLeftNDC = "<<bottomLeftNDC<<std::endl;
    osg::notify(osg::INFO)<<"[osgEarth::EarthTerrainTechnique] topRightNDC = "<<topRightNDC<<std::endl;

    buffer._transform = new osg::MatrixTransform;

    osg::Vec3d centerNDC = (bottomLeftNDC + topRightNDC)*0.5;
    osg::Vec3d centerModel = (bottomLeftNDC + topRightNDC)*0.5;
    masterLocator->convertLocalToModel(centerNDC, centerModel);
    
    buffer._transform->setMatrix(osg::Matrix::translate(centerModel));
    
    _lastCenterModel = centerModel;
    return centerModel;
}

void
EarthTerrainTechnique::calculateSampling( int& out_rows, int& out_cols, double& out_i, double& out_j )
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

void EarthTerrainTechnique::generateGeometry(Locator* masterLocator, const osg::Vec3d& centerModel)
{
    osg::ref_ptr< Locator > masterTextureLocator = masterLocator;
    GeoLocator* geoMasterLocator = dynamic_cast<GeoLocator*>(masterLocator);

    //If we have a geocentric locator, get a geographic version of it to avoid converting
    //to/from geocentric when computing texture coordinats
    if (geoMasterLocator && masterLocator->getCoordinateSystemType() == osgTerrain::Locator::GEOCENTRIC)
    {
        masterTextureLocator = geoMasterLocator->getGeographicFromGeocentric();
    }


    //osg::Timer_t before = osg::Timer::instance()->tick();
    BufferData& buffer = getWriteBuffer();
    
    osgTerrain::Layer* elevationLayer = _terrainTile->getElevationLayer();

    buffer._geode = new osg::Geode;
    buffer._geode->setThreadSafeRefUnref(true);
    if(buffer._transform.valid())
        buffer._transform->addChild(buffer._geode.get());
    
    buffer._geometry = new osg::Geometry;
    buffer._geode->addDrawable(buffer._geometry.get());
        
    osg::Geometry* geometry = buffer._geometry.get();
    geometry->setThreadSafeRefUnref(true);

    buffer._transform->setThreadSafeRefUnref(true);

    int numRows = 20;
    int numColumns = 20;
    
    if (elevationLayer)
    {
        numColumns = elevationLayer->getNumColumns();
        numRows = elevationLayer->getNumRows();
    }
    
    double i_sampleFactor, j_sampleFactor;
    calculateSampling( numColumns, numRows, i_sampleFactor, j_sampleFactor );

    bool treatBoundariesToValidDataAsDefaultValue = _terrainTile->getTreatBoundariesToValidDataAsDefaultValue();
    osg::notify(osg::INFO)<<"[osgEarth::EarthTerrainTechnique] TreatBoundariesToValidDataAsDefaultValue="<<treatBoundariesToValidDataAsDefaultValue<<std::endl;
    
    float skirtHeight = 0.0f;
    HeightFieldLayer* hfl = dynamic_cast<HeightFieldLayer*>(elevationLayer);
    if (hfl && hfl->getHeightField()) 
    {
        skirtHeight = hfl->getHeightField()->getSkirtHeight();
    }
    
    bool createSkirt = skirtHeight != 0.0f;
  
    unsigned int numVerticesInBody = numColumns*numRows;
    unsigned int numVerticesInSkirt = createSkirt ? numColumns*2 + numRows*2 - 4 : 0;
    unsigned int numVertices = numVerticesInBody+numVerticesInSkirt;

    // allocate and assign vertices
    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
    vertices->reserve(numVertices);
    geometry->setVertexArray(vertices.get());

    // allocate and assign normals
    osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array;
    if (normals.valid()) normals->reserve(numVertices);
    geometry->setNormalArray(normals.get());
    geometry->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);
    

    //float minHeight = 0.0;
    float scaleHeight = 
        _verticalScaleOverride != 1.0? _verticalScaleOverride :
        _terrainTile->getTerrain() ? _terrainTile->getTerrain()->getVerticalScale() :
        1.0f;

    // allocate and assign tex coords
    typedef std::pair< osg::ref_ptr<osg::Vec2Array>, osg::ref_ptr<Locator> > TexCoordLocatorPair;
    typedef std::map< Layer*, TexCoordLocatorPair > LayerToTexCoordMap;

    LayerToTexCoordMap layerToTexCoordMap;
    for(unsigned int layerNum=0; layerNum<_terrainTile->getNumColorLayers(); ++layerNum)
    {
        osgTerrain::Layer* colorLayer = _terrainTile->getColorLayer(layerNum);
        if (colorLayer)
        {
            LayerToTexCoordMap::iterator itr = layerToTexCoordMap.find(colorLayer);
            if (itr!=layerToTexCoordMap.end())
            {
                geometry->setTexCoordArray(layerNum, itr->second.first.get());
            }
            else
            {

                Locator* locator = colorLayer->getLocator();
                
				TexCoordLocatorPair& tclp = layerToTexCoordMap[colorLayer];
                tclp.first = new osg::Vec2Array;
                tclp.first->reserve(numVertices);
                if (locator && locator->getCoordinateSystemType() == Locator::GEOCENTRIC)
                {
                    GeoLocator* geo = dynamic_cast<GeoLocator*>(locator);
                    if (geo)
                    {
                        locator = geo->getGeographicFromGeocentric();
                    }
                }
                tclp.second = locator ? locator : masterTextureLocator.get();
                geometry->setTexCoordArray(layerNum, tclp.first.get());
            }
        }
    }

    osg::ref_ptr<osg::FloatArray> elevations = new osg::FloatArray;
    if (elevations.valid()) elevations->reserve(numVertices);
        

    // allocate and assign color
    osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array(1);
    (*colors)[0].set(1.0f,1.0f,1.0f,1.0f);
    
    geometry->setColorArray(colors.get());
    geometry->setColorBinding(osg::Geometry::BIND_OVERALL);


    typedef std::vector<int> Indices;
    Indices indices(numVertices, -1);


    
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
                // osg::notify(osg::INFO)<<"i="<<i<<" j="<<j<<" z="<<value<<std::endl;
                ndc.z() = value*scaleHeight;
            }
            
            if (validValue)
            {
                indices[iv] = vertices->size();
            
                osg::Vec3d model;
                masterLocator->convertLocalToModel(ndc, model);

                (*vertices).push_back(model - centerModel);

                for(LayerToTexCoordMap::iterator itr = layerToTexCoordMap.begin();
                    itr != layerToTexCoordMap.end();
                    ++itr)
                {
                    osg::Vec2Array* texcoords = itr->second.first.get();
                    Locator* colorLocator = itr->second.second;
                    if (colorLocator != masterLocator)
                    {
                        osg::Vec3d color_ndc;
                        Locator::convertLocalCoordBetween(*masterTextureLocator.get(), ndc, *colorLocator, color_ndc);
                        (*texcoords).push_back(osg::Vec2(color_ndc.x(), color_ndc.y()));
                    }
                    else
                    {
                        (*texcoords).push_back(osg::Vec2(ndc.x(), ndc.y()));
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

    //osg::notify(osg::NOTICE) << "  PopulateTime " << osg::Timer::instance()->delta_m(populateBefore, populateAfter) << std::endl;
    
    // populate primitive sets
//    bool optimizeOrientations = elevations!=0;
    bool swapOrientation = !(masterLocator->orientationOpenGL());
    

    //osg::Timer_t genPrimBefore = osg::Timer::instance()->tick();
    osg::ref_ptr<osg::DrawElementsUInt> elements = new osg::DrawElementsUInt(GL_TRIANGLES);
    elements->reserve((numRows-1) * (numColumns-1) * 6);

    geometry->addPrimitiveSet(elements.get());

    //osg::Timer_t genPrimAfter = osg::Timer::instance()->tick();
    //osg::notify(osg::NOTICE) << "  genPrimTime " << osg::Timer::instance()->delta_m(genPrimBefore, genPrimAfter) << std::endl;
    
    osg::ref_ptr<osg::Vec3Array> skirtVectors = new osg::Vec3Array((*normals));
    
    if (!normals) createSkirt = false;

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
                unsigned int new_i = vertices->size(); // index of new index of added skirt point
                osg::Vec3 new_v = (*vertices)[orig_i] - ((*skirtVectors)[orig_i])*skirtHeight;
                (*vertices).push_back(new_v);
                if (normals.valid()) (*normals).push_back((*normals)[orig_i]);

                for(LayerToTexCoordMap::iterator itr = layerToTexCoordMap.begin();
                    itr != layerToTexCoordMap.end();
                    ++itr)
                {
                    itr->second.first->push_back((*itr->second.first)[orig_i]);
                }
                
                skirtDrawElements->push_back(orig_i);
                skirtDrawElements->push_back(new_i);
            }
            else
            {
                if (!skirtDrawElements->empty())
                {
                    geometry->addPrimitiveSet(skirtDrawElements.get());
                    skirtDrawElements = new osg::DrawElementsUShort(GL_QUAD_STRIP);
                }
                
            }
        }

        if (!skirtDrawElements->empty())
        {
            geometry->addPrimitiveSet(skirtDrawElements.get());
            skirtDrawElements = new osg::DrawElementsUShort(GL_QUAD_STRIP);
        }

        // create right skirt vertices
        c=numColumns-1;
        for(r=0;r<static_cast<int>(numRows);++r)
        {
            int orig_i = indices[(r)*numColumns+c]; // index of original vertex of grid
            if (orig_i>=0)
            {
                unsigned int new_i = vertices->size(); // index of new index of added skirt point
                osg::Vec3 new_v = (*vertices)[orig_i] - ((*skirtVectors)[orig_i])*skirtHeight;
                (*vertices).push_back(new_v);
                if (normals.valid()) (*normals).push_back((*normals)[orig_i]);
                for(LayerToTexCoordMap::iterator itr = layerToTexCoordMap.begin();
                    itr != layerToTexCoordMap.end();
                    ++itr)
                {
                    itr->second.first->push_back((*itr->second.first)[orig_i]);
                }
                
                skirtDrawElements->push_back(orig_i);
                skirtDrawElements->push_back(new_i);
            }
            else
            {
                if (!skirtDrawElements->empty())
                {
                    geometry->addPrimitiveSet(skirtDrawElements.get());
                    skirtDrawElements = new osg::DrawElementsUShort(GL_QUAD_STRIP);
                }
                
            }
        }

        if (!skirtDrawElements->empty())
        {
            geometry->addPrimitiveSet(skirtDrawElements.get());
            skirtDrawElements = new osg::DrawElementsUShort(GL_QUAD_STRIP);
        }

        // create top skirt vertices
        r=numRows-1;
        for(c=numColumns-1;c>=0;--c)
        {
            int orig_i = indices[(r)*numColumns+c]; // index of original vertex of grid
            if (orig_i>=0)
            {
                unsigned int new_i = vertices->size(); // index of new index of added skirt point
                osg::Vec3 new_v = (*vertices)[orig_i] - ((*skirtVectors)[orig_i])*skirtHeight;
                (*vertices).push_back(new_v);
                if (normals.valid()) (*normals).push_back((*normals)[orig_i]);
                for(LayerToTexCoordMap::iterator itr = layerToTexCoordMap.begin();
                    itr != layerToTexCoordMap.end();
                    ++itr)
                {
                    itr->second.first->push_back((*itr->second.first)[orig_i]);
                }
                
                skirtDrawElements->push_back(orig_i);
                skirtDrawElements->push_back(new_i);
            }
            else
            {
                if (!skirtDrawElements->empty())
                {
                    geometry->addPrimitiveSet(skirtDrawElements.get());
                    skirtDrawElements = new osg::DrawElementsUShort(GL_QUAD_STRIP);
                }
                
            }
        }

        if (!skirtDrawElements->empty())
        {
            geometry->addPrimitiveSet(skirtDrawElements.get());
            skirtDrawElements = new osg::DrawElementsUShort(GL_QUAD_STRIP);
        }

        // create left skirt vertices
        c=0;
        for(r=numRows-1;r>=0;--r)
        {
            int orig_i = indices[(r)*numColumns+c]; // index of original vertex of grid
            if (orig_i>=0)
            {
                unsigned int new_i = vertices->size(); // index of new index of added skirt point
                osg::Vec3 new_v = (*vertices)[orig_i] - ((*skirtVectors)[orig_i])*skirtHeight;
                (*vertices).push_back(new_v);
                if (normals.valid()) (*normals).push_back((*normals)[orig_i]);
                for(LayerToTexCoordMap::iterator itr = layerToTexCoordMap.begin();
                    itr != layerToTexCoordMap.end();
                    ++itr)
                {
                    itr->second.first->push_back((*itr->second.first)[orig_i]);
                }
                
                skirtDrawElements->push_back(orig_i);
                skirtDrawElements->push_back(new_i);
            }
            else
            {
                if (!skirtDrawElements->empty())
                {
                    geometry->addPrimitiveSet(skirtDrawElements.get());
                    skirtDrawElements = new osg::DrawElementsUShort(GL_QUAD_STRIP);
                }
                
            }
        }

        if (!skirtDrawElements->empty())
        {
            geometry->addPrimitiveSet(skirtDrawElements.get());
            skirtDrawElements = new osg::DrawElementsUShort(GL_QUAD_STRIP);
        }
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

                osg::Vec3f &v00 = (*vertices)[i00];
                osg::Vec3f &v10 = (*vertices)[i10];
                osg::Vec3f &v01 = (*vertices)[i01];
                osg::Vec3f &v11 = (*vertices)[i11];

                if (fabsf(e00-e11)<fabsf(e01-e10))
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
                    osg::Vec3f &v1 = (*vertices)[validIndices[0]];
                    osg::Vec3f &v2 = (*vertices)[validIndices[1]];
                    osg::Vec3f &v3 = (*vertices)[validIndices[2]];
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
    //osg::notify(osg::NOTICE) << "  skirtTime " << osg::Timer::instance()->delta_m(skirtBefore, skirtAfter) << std::endl;


    //geometry->setUseDisplayList(false);
    geometry->setUseVertexBufferObjects(true);
    
    
    if (osgDB::Registry::instance()->getBuildKdTreesHint()==osgDB::ReaderWriter::Options::BUILD_KDTREES &&
        osgDB::Registry::instance()->getKdTreeBuilder())
    {            
        //osg::Timer_t before = osg::Timer::instance()->tick();
        //osg::notify(osg::NOTICE)<<"osgTerrain::GeometryTechnique::build kd tree"<<std::endl;
        osg::ref_ptr<osg::KdTreeBuilder> builder = osgDB::Registry::instance()->getKdTreeBuilder()->clone();
        buffer._geode->accept(*builder);
        //osg::Timer_t after = osg::Timer::instance()->tick();
        //osg::notify(osg::NOTICE)<<"KdTree build time "<<osg::Timer::instance()->delta_m(before, after)<<std::endl;
    }

    

    //DEBUGGING
#if 0
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
        TransparentLayer* tl = static_cast<TransparentLayer*>( _terrainTile->getColorLayer(i) );
        buf << tl->getId() << "=" << tl->getLevelOfDetail() << std::endl;
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

void EarthTerrainTechnique::applyColorLayers()
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
                if ( image->r() < 2 )
                {
                    osg::Texture2D* t2d = new osg::Texture2D();
                    t2d->setImage( image );
                    texture = t2d;
                }
                else
                {
                    // NOTE: Texture2DArray is not supported in NVIDIA 7 series
                    //
                    //osg::Texture2DArray* tarray = new osg::Texture2DArray();
                    //tarray->setTextureSize( image->s(), image->t(), image->r() );
                    //for( int r=0; r<image->r(); ++r )
                    //{
                    //    osg::Image* rImage = new osg::Image();
                    //    rImage->allocateImage( image->s(), image->t(), 1, image->getPixelFormat(), image->getDataType(), image->getPacking() );
                    //    memcpy( rImage->data(), image->data(0,0,r), image->getImageSizeInBytes() );
                    //    tarray->setImage( r, rImage );
                    //}

                    osg::Texture3D* tarray = new osg::Texture3D();
                    tarray->setTextureSize( image->s(), image->t(), image->r() );
                    for( int r=0; r<image->r(); ++r )
                    {
                        osg::Image* rImage = new osg::Image();
                        rImage->allocateImage( image->s(), image->t(), 1, image->getPixelFormat(), image->getDataType(), image->getPacking() );
                        memcpy( rImage->data(), image->data(0,0,r), image->getImageSizeInBytes() );
                        tarray->setImage( r, rImage );
                    }

                    texture = tarray;
                }

                //osg::Texture2D* texture2D = new osg::Texture2D;
                //texture2D->setImage(image);
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

                bool mipMapping = !(texture->getFilter(osg::Texture::MIN_FILTER)==osg::Texture::LINEAR || texture->getFilter(osg::Texture::MIN_FILTER)==osg::Texture::NEAREST);
                bool s_NotPowerOfTwo = image->s()==0 || (image->s() & (image->s() - 1));
                bool t_NotPowerOfTwo = image->t()==0 || (image->t() & (image->t() - 1));

                if (mipMapping && (s_NotPowerOfTwo || t_NotPowerOfTwo))
                {
                    osg::notify(osg::INFO)<<"[osgEarth::EarthTerrainTechnique] Disabling mipmapping for non power of two tile size("<<image->s()<<", "<<image->t()<<")"<<std::endl;
                    texture->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR);
                }

                layerToTextureMap[colorLayer] = texture;
                //texture = texture2D;

                // osg::notify(osg::NOTICE)<<"Creating new ImageLayer texture "<<layerNum<<" image->s()="<<image->s()<<"  image->t()="<<image->t()<<std::endl;

            }
            else
            {
                // osg::notify(osg::NOTICE)<<"Reusing ImageLayer texture "<<layerNum<<std::endl;
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
}

void EarthTerrainTechnique::applyTransparency()
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

void EarthTerrainTechnique::smoothGeometry()
{
    BufferData& buffer = getWriteBuffer();

    //osg::Timer_t before = osg::Timer::instance()->tick();
    
    if (buffer._geometry.valid())
    {
        osgUtil::SmoothingVisitor smoother;
        smoother.smooth(*buffer._geometry);
    }

    //osg::Timer_t after = osg::Timer::instance()->tick();

    //osg::notify(osg::NOTICE) << "Smooth time " << osg::Timer::instance()->delta_m(before, after) << std::endl;
}

void EarthTerrainTechnique::update(osgUtil::UpdateVisitor* uv)
{
    if (_terrainTile) _terrainTile->osg::Group::traverse(*uv);
}


void EarthTerrainTechnique::cull(osgUtil::CullVisitor* cv)
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


void EarthTerrainTechnique::traverse(osg::NodeVisitor& nv)
{
    if (!_terrainTile) return;

    // if app traversal update the frame count.
    if (nv.getVisitorType()==osg::NodeVisitor::UPDATE_VISITOR)
    {
        if (_terrainTile->getDirty()) _terrainTile->init();

        osgUtil::UpdateVisitor* uv = dynamic_cast<osgUtil::UpdateVisitor*>(&nv);
        if (uv)
        {
            update(uv);
            return;
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
    }

    // the code from here on accounts for user traversals (intersections, etc)
    if (_terrainTile->getDirty()) 
    {
        //osg::notify(osg::INFO)<<"******* Doing init ***********"<<std::endl;
        _terrainTile->init();
    }

    BufferData& buffer = getReadOnlyBuffer();
    if (buffer._transform.valid()) buffer._transform->accept(nv);
}


void EarthTerrainTechnique::cleanSceneGraph()
{
}

void
EarthTerrainTechnique::releaseGLObjects(osg::State* state) const
{
    EarthTerrainTechnique* ncThis = const_cast<EarthTerrainTechnique*>(this);

    ScopedWriteLock lock( ncThis->getMutex() );

    if (_bufferData[0]._transform.valid())
    {
        _bufferData[0]._transform->releaseGLObjects(state);
    }
    if (_bufferData[1]._transform.valid())
    {
        _bufferData[1]._transform->releaseGLObjects(state);   
    }
}

//void
//EarthTerrainTechnique::releaseStaleGLObjects(osg::State* state) const
//{     
//    ScopedReadLock lock( const_cast<EarthTerrainTechnique*>(this)->getMutex() );
//
//    for( ObjectList::const_iterator i = _objsToRelease.begin(); i != _objsToRelease.end(); ++i )
//    {        
//        i->get()->releaseGLObjects( state );
//    }
//    const_cast<EarthTerrainTechnique*>(this)->_objsToRelease.clear();
//}
