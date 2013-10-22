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
#include "MultiPassTerrainTechnique"
#include "TerrainNode"
#include "TransparentLayer"
#include <osgEarth/ImageUtils>


#include <osgUtil/SmoothingVisitor>
#include <osgDB/FileUtils>
#include <osg/io_utils>
#include <osg/Texture2D>
#include <osg/Texture1D>
#include <osg/TexEnvCombine>
#include <osg/Program>
#include <osg/Math>
#include <osg/Timer>
#include <osg/Depth>
#include <osg/Version>

using namespace osgEarth_engine_osgterrain;
using namespace osgEarth;

#define NEW_COORD_CODE

//-------------------------------------------------------------------------

namespace
{
    // userdata structure so we can look up passes by layer-UID in the graph.
    struct LayerData : public osg::Referenced
    {
        LayerData( UID layerUID ) : _layerUID(layerUID) { }
        UID _layerUID;
    };

    static osg::Geode*
    s_findGeodeByUID( osg::Group* group, UID layerUID )
    {
        for( unsigned int i=0; i<group->getNumChildren(); ++i )
        {
            osg::Geode* geode = static_cast<osg::Geode*>( group->getChild(i) );
            LayerData* d = static_cast<LayerData*>( geode->getUserData() );
            if ( d && d->_layerUID == layerUID )
                return geode;
        }
        return 0L;
    }
}

//-------------------------------------------------------------------------

MultiPassTerrainTechnique::MultiPassTerrainTechnique( TextureCompositor* texCompositor ) :
TerrainTechnique(),
//osgTerrain::TerrainTechnique(),
_terrainTileInitialized(false),
_texCompositor( texCompositor )
{
    this->setThreadSafeRefUnref( true );
}

MultiPassTerrainTechnique::MultiPassTerrainTechnique(const MultiPassTerrainTechnique& mt,const osg::CopyOp& copyop) :
TerrainTechnique(mt,copyop)
{
    _terrainTileInitialized = mt._terrainTileInitialized;
    _texCompositor = mt._texCompositor.get();
}

MultiPassTerrainTechnique::~MultiPassTerrainTechnique()
{
}

#if 0
void
#if OSG_MIN_VERSION_REQUIRED(2,9,8)
MultiPassTerrainTechnique::init(int dirtyMask, bool assumeMultiThreaded)
#else
MultiPassTerrainTechnique::init()
#endif
#endif

void
MultiPassTerrainTechnique::init()
{
    OE_DEBUG<<"Doing MultiPassTerrainTechnique::init()"<<std::endl;
    
    if (!_tile) return;
   
    osgTerrain::Locator* masterLocator = computeMasterLocator();
    
    osg::Vec3d centerModel = computeCenterModel(masterLocator);
    
    generateGeometry(masterLocator, centerModel);   

    if (_transform.valid()) _transform->setThreadSafeReferenceCounting(true);
}

osgTerrain::Locator*
MultiPassTerrainTechnique::computeMasterLocator()
{
    osgTerrain::Layer* elevationLayer = _tile->getElevationLayer();
    osgTerrain::Locator* locator = elevationLayer ? elevationLayer->getLocator() : 0L;

    if ( !locator )
    {
        OE_NOTICE << "Problem, no locator found in any of the terrain layers"<<std::endl;
        return 0;
    }
    return locator;
}

osg::Vec3d
MultiPassTerrainTechnique::computeCenterModel(osgTerrain::Locator* masterLocator)
{
    if (!masterLocator) return osg::Vec3d(0.0,0.0,0.0);
  
    osgTerrain::Layer*   elevationLayer   = _tile->getElevationLayer();
    osgTerrain::Locator* elevationLocator = elevationLayer ? elevationLayer->getLocator() : 0;
    osgTerrain::Locator* colorLocator     = 0L;
    
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

    //if (colorLayer)
    //{
    //    if (colorLocator!= masterLocator)
    //    {
    //        masterLocator->computeLocalBounds(*colorLocator, bottomLeftNDC, topRightNDC);
    //    }
    //    else
    //    {
    //        bottomLeftNDC.x() = osg::minimum(bottomLeftNDC.x(), 0.0);
    //        bottomLeftNDC.y() = osg::minimum(bottomLeftNDC.y(), 0.0);
    //        topRightNDC.x() = osg::maximum(topRightNDC.x(), 1.0);
    //        topRightNDC.y() = osg::maximum(topRightNDC.y(), 1.0);
    //    }
    //}

    //OE_DEBUG<<"bottomLeftNDC = "<<bottomLeftNDC<<std::endl;
    //OE_DEBUG<<"topRightNDC = "<<topRightNDC<<std::endl;

    _transform = new osg::MatrixTransform;

    osg::Vec3d centerNDC = (bottomLeftNDC + topRightNDC)*0.5;
    osg::Vec3d centerModel = (bottomLeftNDC + topRightNDC)*0.5;
    masterLocator->convertLocalToModel(centerNDC, centerModel);
    
    _transform->setMatrix(osg::Matrix::translate(centerModel));
    
    return centerModel;
}

osg::Geometry*
MultiPassTerrainTechnique::createGeometryPrototype(osgTerrain::Locator* masterLocator, const osg::Vec3d& centerModel)
{  
    osgTerrain::Layer* elevationLayer = _tile->getElevationLayer();

    osg::Geometry* geometry = new osg::Geometry();
    geometry->setUseVertexBufferObjects(true);

	unsigned int numRows = 20;
    unsigned int numColumns = 20;
    
    if (elevationLayer)
    {
        numColumns = elevationLayer->getNumColumns();
        numRows = elevationLayer->getNumRows();
    }

    osg::ref_ptr< TerrainNode > terrain = _tile->getTerrain();
    
    float sampleRatio = terrain.valid() ? terrain->getSampleRatio() : 1.0f;
    
    double i_sampleFactor = 1.0;
    double j_sampleFactor = 1.0;

    // OE_NOTICE<<"Sample ratio="<<sampleRatio<<std::endl;

    if (sampleRatio!=1.0f)
    {
    
        unsigned int originalNumColumns = numColumns;
        unsigned int originalNumRows = numRows;
    
        numColumns = osg::maximum((unsigned int) (float(originalNumColumns)*sqrtf(sampleRatio)), 4u);
        numRows = osg::maximum((unsigned int) (float(originalNumRows)*sqrtf(sampleRatio)),4u);

        i_sampleFactor = double(originalNumColumns-1)/double(numColumns-1);
        j_sampleFactor = double(originalNumRows-1)/double(numRows-1);
    }
    
    

    //bool treatBoundariesToValidDataAsDefaultValue = _tile->getTreatBoundariesToValidDataAsDefaultValue();
    //OE_DEBUG<<"TreatBoundariesToValidDataAsDefaultValue="<<treatBoundariesToValidDataAsDefaultValue<<std::endl;
    
    float skirtHeight = 0.0f;
    osgTerrain::HeightFieldLayer* hfl = dynamic_cast<osgTerrain::HeightFieldLayer*>(elevationLayer);
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
    float scaleHeight = terrain.valid() ? terrain->getVerticalScale() : 1.0f;

    //Reserve space for the elevations
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
    for(j=0; j<numRows; ++j)
    {
        for(i=0; i<numColumns; ++i)
        {
            unsigned int iv = j*numColumns + i;
            osg::Vec3d ndc( ((double)i)/(double)(numColumns-1), ((double)j)/(double)(numRows-1), 0.0);
     
            bool validValue = true;
     
            
            unsigned int i_equiv = i_sampleFactor==1.0 ? i : (unsigned int) (double(i)*i_sampleFactor);
            unsigned int j_equiv = i_sampleFactor==1.0 ? j : (unsigned int) (double(j)*j_sampleFactor);
            
            if (elevationLayer)
            {
                float value = 0.0f;
                validValue = elevationLayer->getValidValue(i_equiv,j_equiv, value);
                // OE_INFO<<"i="<<i<<" j="<<j<<" z="<<value<<std::endl;
                ndc.z() = value*scaleHeight;
            }
            
            if (validValue)
            {
                indices[iv] = vertices->size();
            
                osg::Vec3d model;
                masterLocator->convertLocalToModel(ndc, model);

                (*vertices).push_back(model - centerModel);

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
    
    // populate primitive sets
//    bool optimizeOrientations = elevations!=0;
    bool swapOrientation = !(masterLocator->orientationOpenGL());
    
    osg::ref_ptr<osg::DrawElementsUShort> elements = new osg::DrawElementsUShort(GL_TRIANGLES);
    elements->reserve((numRows-1) * (numColumns-1) * 6);

    geometry->addPrimitiveSet(elements.get());

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

                if (fabsf(e00-e11)<fabsf(e01-e10))
                {
                    elements->push_back(i01);
                    elements->push_back(i00);
                    elements->push_back(i11);

                    elements->push_back(i00);
                    elements->push_back(i10);
                    elements->push_back(i11);
                }
                else
                {
                    elements->push_back(i01);
                    elements->push_back(i00);
                    elements->push_back(i10);

                    elements->push_back(i01);
                    elements->push_back(i10);
                    elements->push_back(i11);
                }
            }
            else if (numValid==3)
            {
                if (i00>=0) elements->push_back(i00);
                if (i01>=0) elements->push_back(i01);
                if (i11>=0) elements->push_back(i11);
                if (i10>=0) elements->push_back(i10);
            }
            
        }
    }
    
    osg::ref_ptr<osg::Vec3Array> skirtVectors = new osg::Vec3Array((*normals));
    
    if (elevationLayer)
    {
        osgUtil::SmoothingVisitor smoother;
        smoother.smooth(*geometry);
        
        normals = dynamic_cast<osg::Vec3Array*>(geometry->getNormalArray());
        
        if (!normals) createSkirt = false;
    }

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


    geometry->setUseDisplayList(false);
    geometry->setUseVertexBufferObjects(true);

	return geometry;
}

osg::Geode* MultiPassTerrainTechnique::createPass(unsigned int            order,
                                                  const CustomColorLayer* colorLayer,
                                                  osgTerrain::Locator*    masterLocator,
                                                  const osg::Vec3d&       centerModel,
                                                  osg::Geometry*          geometry)
{
	OE_DEBUG << "osgEarth::MultiPassTerrainTechnique createPass " << order << std::endl;
    unsigned int binNumber = 1000;
    binNumber += order;
   
    osgTerrain::Layer* elevationLayer = _tile->getElevationLayer();

    //Create a new geode to store the geometry
    osg::Geode* geode = new osg::Geode;
   
    //Set up the pass 
    geode->getOrCreateStateSet()->setRenderBinDetails(binNumber, "RenderBin");
    geode->addDrawable(geometry);

    unsigned int numRows = 20;
    unsigned int numColumns = 20;
    
    if (elevationLayer)
    {
        numColumns = elevationLayer->getNumColumns();
        numRows = elevationLayer->getNumRows();
    }

    osg::ref_ptr< TerrainNode> terrain = _tile->getTerrain();
    
    float sampleRatio = terrain.valid() ? terrain->getSampleRatio() : 1.0f;
    
    double i_sampleFactor = 1.0;
    double j_sampleFactor = 1.0;

    // OE_NOTICE<<"Sample ratio="<<sampleRatio<<std::endl;

    if (sampleRatio!=1.0f)
    {
    
        unsigned int originalNumColumns = numColumns;
        unsigned int originalNumRows = numRows;
    
        numColumns = osg::maximum((unsigned int) (float(originalNumColumns)*sqrtf(sampleRatio)), 4u);
        numRows = osg::maximum((unsigned int) (float(originalNumRows)*sqrtf(sampleRatio)),4u);

        i_sampleFactor = double(originalNumColumns-1)/double(numColumns-1);
        j_sampleFactor = double(originalNumRows-1)/double(numRows-1);
    }

    //bool treatBoundariesToValidDataAsDefaultValue = _terrainTile->getTreatBoundariesToValidDataAsDefaultValue();
    //OE_DEBUG<<"TreatBoundariesToValidDataAsDefaultValue="<<treatBoundariesToValidDataAsDefaultValue<<std::endl;
    
    float skirtHeight = 0.0f;
    osgTerrain::HeightFieldLayer* hfl = dynamic_cast<osgTerrain::HeightFieldLayer*>(elevationLayer);
    if (hfl && hfl->getHeightField()) 
    {
        skirtHeight = hfl->getHeightField()->getSkirtHeight();
    }
    
    bool createSkirt = skirtHeight != 0.0f;
  
    unsigned int numVerticesInBody = numColumns*numRows;
    unsigned int numVerticesInSkirt = createSkirt ? numColumns*2 + numRows*2 - 4 : 0;
    unsigned int numVertices = numVerticesInBody+numVerticesInSkirt;

    //float minHeight = 0.0;
    float scaleHeight = terrain.valid() ? terrain->getVerticalScale() : 1.0f;

    osg::ref_ptr<osg::Vec2Array> texCoords;

    const osgTerrain::Locator* locator = colorLayer ? colorLayer->getLocator() : 0L;
    if ( locator )
    {
        texCoords = new osg::Vec2Array;
        texCoords->reserve(numVertices);
        _texCompositor->assignTexCoordArray( geometry, colorLayer->getUID(), texCoords.get() );
    }

    const osgTerrain::Locator* colorLocator = locator ? locator : masterLocator;

    // allocate and assign color
    osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array(1);
    (*colors)[0].set(1.0f,1.0f,1.0f,1.0f);    
    geometry->setColorArray(colors.get());
    geometry->setColorBinding(osg::Geometry::BIND_OVERALL);

    typedef std::vector<int> Indices;
    Indices indices(numVertices, -1);
    
    // populate vertex and tex coord arrays
    unsigned int i, j;
	int vindex = 0;
    for(j=0; j<numRows; ++j)
    {
        for(i=0; i<numColumns; ++i)
        {
            unsigned int iv = j*numColumns + i;
            osg::Vec3d ndc( ((double)i)/(double)(numColumns-1), ((double)j)/(double)(numRows-1), 0.0);
     
            bool validValue = true;

            unsigned int i_equiv = i_sampleFactor==1.0 ? i : (unsigned int) (double(i)*i_sampleFactor);
            unsigned int j_equiv = i_sampleFactor==1.0 ? j : (unsigned int) (double(j)*j_sampleFactor);
            
            if (elevationLayer)
            {
                float value = 0.0f;
                validValue = elevationLayer->getValidValue(i_equiv,j_equiv, value);
                // OE_INFO<<"i="<<i<<" j="<<j<<" z="<<value<<std::endl;
                ndc.z() = value*scaleHeight;
            }
            
            if (validValue)
            {
                indices[iv] = vindex++;

				if (texCoords.valid())
				{
					osg::Vec3d model;
					masterLocator->convertLocalToModel(ndc, model);

					if (colorLocator != masterLocator)
					{
						osg::Vec3d color_ndc;
						osgTerrain::Locator::convertLocalCoordBetween(*masterLocator, ndc, *colorLocator, color_ndc);
						texCoords->push_back(osg::Vec2(color_ndc.x(), color_ndc.y()));
					}
					else
					{
						texCoords->push_back(osg::Vec2(ndc.x(), ndc.y()));
					}
				}
			}
            else
            {
                indices[iv] = -1;
            }
        }
    }
    
    // populate primitive sets
//    bool optimizeOrientations = elevations!=0;
//    bool swapOrientation = !(masterLocator->orientationOpenGL());    

    if (createSkirt)
    {
         // create bottom skirt vertices
        int r,c;
        r=0;
        for(c=0;c<static_cast<int>(numColumns);++c)
        {
            int orig_i = indices[(r)*numColumns+c]; // index of original vertex of grid
            if (orig_i>=0)
            {
                if (texCoords.valid()) texCoords->push_back((*texCoords)[orig_i]);               
            }
        }

        // create right skirt vertices
        c=numColumns-1;
        for(r=0;r<static_cast<int>(numRows);++r)
        {
            int orig_i = indices[(r)*numColumns+c]; // index of original vertex of grid
            if (orig_i>=0)
            {
                 if (texCoords.valid()) texCoords->push_back((*texCoords)[orig_i]);
            }
        }

        // create top skirt vertices
        r=numRows-1;
        for(c=numColumns-1;c>=0;--c)
        {
            int orig_i = indices[(r)*numColumns+c]; // index of original vertex of grid
            if (orig_i>=0)
            {
                if (texCoords.valid()) texCoords->push_back((*texCoords)[orig_i]);               
            }
        }

        // create left skirt vertices
        c=0;
        for(r=numRows-1;r>=0;--r)
        {
            int orig_i = indices[(r)*numColumns+c]; // index of original vertex of grid
            if (orig_i>=0)
            {
                if (texCoords.valid()) texCoords->push_back((*texCoords)[orig_i]);               
            }
        }
    }

    geometry->setUseDisplayList(false);
    geometry->setUseVertexBufferObjects(true);

	//TODO:  Should we do this for each geode?  
    if (osgDB::Registry::instance()->getBuildKdTreesHint()==osgDB::ReaderWriter::Options::BUILD_KDTREES &&
        osgDB::Registry::instance()->getKdTreeBuilder())
    {    
        //osg::Timer_t before = osg::Timer::instance()->tick();
        //OE_NOTICE<<"osgTerrain::GeometryTechnique::build kd tree"<<std::endl;
        osg::ref_ptr<osg::KdTreeBuilder> builder = osgDB::Registry::instance()->getKdTreeBuilder()->clone();
        //buffer._geode->accept(*builder);
        geode->accept(*builder);
        //osg::Timer_t after = osg::Timer::instance()->tick();
        //OE_NOTICE<<"KdTree build time "<<osg::Timer::instance()->delta_m(before, after)<<std::endl;
    }


    //Apply the appropriate color layer to the pass

    if (colorLayer)
    {
        const osg::Image* image = colorLayer->getImage();
        if (image)
        {
            //osgTerrain::ImageLayer* imageLayer = dynamic_cast<osgTerrain::ImageLayer*>(colorLayer);
            //osgTerrain::ContourLayer* contourLayer = dynamic_cast<osgTerrain::ContourLayer*>(colorLayer);
            //if (imageLayer)
            //{

                osg::StateSet* stateset = geode->getOrCreateStateSet();

                //Compress the image if it's not compressed and it is requested that we compress textures

                osg::Image* img = const_cast<osg::Image*>(image);

                osg::Texture2D* texture2D = new osg::Texture2D;
                texture2D->setImage( img );
                texture2D->setMaxAnisotropy(16.0f);
                texture2D->setResizeNonPowerOfTwoHint(false);

                texture2D->setFilter( osg::Texture::MAG_FILTER, *_texCompositor->getOptions().magFilter() );
                if (ImageUtils::isPowerOfTwo( img ) && !(!img->isMipmap() && ImageUtils::isCompressed(img)))
                {
                    texture2D->setFilter( osg::Texture::MIN_FILTER, *_texCompositor->getOptions().minFilter() );
                }
                else
                {
                    OE_DEBUG<<"[osgEarth::MultiPassTerrainTechnique] Disabling mipmapping for non power of two tile size("<<image->s()<<", "<<image->t()<<")"<<std::endl;
                    texture2D->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR );
                }

                texture2D->setWrap(osg::Texture::WRAP_S,osg::Texture::CLAMP_TO_EDGE);
                texture2D->setWrap(osg::Texture::WRAP_T,osg::Texture::CLAMP_TO_EDGE);
                stateset->setTextureAttributeAndModes(0, texture2D, osg::StateAttribute::ON);           
        }
        //    }
        //    else if (contourLayer)
        //    {
        //        osg::StateSet* stateset = geode->getOrCreateStateSet();

        //        osg::Texture1D* texture1D = new osg::Texture1D;
        //        texture1D->setImage(image);
        //        texture1D->setResizeNonPowerOfTwoHint(false);
        //        texture1D->setFilter(osg::Texture::MIN_FILTER, osg::Texture::NEAREST);
        //        texture1D->setFilter(osg::Texture::MAG_FILTER, colorLayer->getMagFilter());            
        //        stateset->setTextureAttributeAndModes(0, texture1D, osg::StateAttribute::ON);
        //    }
        //}
    }

    //geode->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
    geode->getOrCreateStateSet()->setAttributeAndModes(new osg::Depth(osg::Depth::LEQUAL), osg::StateAttribute::ON);

    return geode;
}

void MultiPassTerrainTechnique::generateGeometry(osgTerrain::Locator* masterLocator, const osg::Vec3d& centerModel)
{
    _passes = new osg::Group;
    if (_transform.valid())
    {
        _transform->removeChildren( 0, _transform->getNumChildren() );
        _transform->addChild(_passes.get());
    }

    typedef std::map<int, osg::ref_ptr<osg::Geode> > OrderedGeodes;
    OrderedGeodes order;

	osg::ref_ptr<osg::Geometry> prototype = createGeometryPrototype( masterLocator, centerModel );

    // take a thread-safe snapshot of the layer stack:
    TileFrame tilef( _tile );

    if ( tilef._colorLayers.size() == 0 )
    {
        // if there's no data, just make a placeholder pass
		osg::Geode* geode = createPass(0, 0L, masterLocator, centerModel, prototype.get());
		_passes->addChild( geode );
	}
    else
    {
        int defaultLayerOrder = 0;

        // create a pass for each layer, and then add them in the proper order:
        for( ColorLayersByUID::const_iterator i = tilef._colorLayers.begin(); i != tilef._colorLayers.end(); ++i )
        {
            const CustomColorLayer& layer = i->second;
            osg::Geometry* passGeom = new osg::Geometry( *prototype.get() );
            int index = _texCompositor->getRenderOrder( layer.getUID() );
            if ( index < 0 ) // true on first-time initialization
                index = defaultLayerOrder++;
            osg::Geode* geode = createPass( index, &layer, masterLocator, centerModel, passGeom );
            order[index] = geode;

            // record the UID in the geode for lookup later:
            geode->setUserData( new LayerData( layer.getUID() ) );
        }            

        for( OrderedGeodes::const_iterator j = order.begin(); j != order.end(); ++j )
        {
            _passes->addChild( j->second.get() );
        }
    }

    osg::StateSet* stateset = _transform->getOrCreateStateSet();
    stateset->setMode(GL_BLEND, osg::StateAttribute::ON);
    stateset->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
}

void MultiPassTerrainTechnique::traverse(osg::NodeVisitor& nv)
{
    if (!_tile) return;

    // initialize the terrain tile on startup
    if (_tile->getDirty() && !_terrainTileInitialized) 
    {
        _tile->init();
        _terrainTileInitialized = true;

#if 0
#if OSG_MIN_VERSION_REQUIRED(2,9,8)
        _terrainTile->init(~0x0, true);
#else
        _terrainTile->init();
#endif

        _terrainTileInitialized = true;
#endif
    }
    
    if ( nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR )
    {
        updateTransparency();
    }

    // traverse the dynamically-generated geometry.
    if (_transform.valid()) 
        _transform->accept(nv);
}

void MultiPassTerrainTechnique::updateTransparency()
{	
    if ( _passes.valid() )
    {
        ColorLayersByUID colorLayers;
        _tile->getCustomColorLayers( colorLayers );

        for( ColorLayersByUID::const_iterator i = colorLayers.begin(); i != colorLayers.end(); ++i )
        {
            const CustomColorLayer& colorLayer = i->second;

            float opacity = colorLayer.getMapLayer()->getOpacity();
            osg::Geode* geode = s_findGeodeByUID( _passes.get(), colorLayer.getUID() );
			if (geode)
			{
				osg::Geometry* geometry = geode->getDrawable(0)->asGeometry();
				osg::Vec4Array* colors = static_cast<osg::Vec4Array*>(geometry->getColorArray());
                if ( (*colors)[0].a() != opacity )
                {
				    (*colors)[0] = osg::Vec4(1.0f, 1.0f, 1.0f, opacity);
                    colors->dirty();
                }

				if (colorLayer.getMapLayer()->getVisible())
				{
					geode->setNodeMask(0xffffffff);
				}
				else
				{
					geode->setNodeMask(0x0);
				}
			}
		}
	}
}

void MultiPassTerrainTechnique::releaseGLObjects(osg::State* state) const
{
    if (_transform.valid()) _transform->releaseGLObjects( state );
}
