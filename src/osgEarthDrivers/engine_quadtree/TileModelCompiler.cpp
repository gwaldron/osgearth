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
#include "TileModelCompiler"

#include <osgEarth/Locators>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/HeightFieldUtils>
#include <osgEarth/TextureCompositor>
#include <osgEarthSymbology/Geometry>
#include <osgEarthSymbology/MeshConsolidator>

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/MatrixTransform>
#include <osgUtil/DelaunayTriangulator>

using namespace osgEarth_engine_quadtree;
using namespace osgEarth;
using namespace osgEarth::Drivers;
using namespace osgEarth::Symbology;

#define LC "[TileModelCompiler] "

//------------------------------------------------------------------------

osg::ref_ptr<osg::Vec2Array>&
CompilerCache::TexCoordArrayCache::get(const osg::Vec4d& mat,
                                       unsigned          cols,
                                       unsigned          rows)
{
    for( iterator i = begin(); i != end(); ++i )
    {
        CompilerCache::TexCoordTableKey& key = i->first;
        if ( key._mat == mat && key._cols == cols && key._rows == rows )
        {
            return i->second;
        }
    }
    
    CompilerCache::TexCoordTableKey newKey;
    newKey._mat     = mat;
    newKey._cols    = cols;
    newKey._rows    = rows;
    this->push_back( std::make_pair(newKey, (osg::Vec2Array*)0L) );
    return this->back().second;
}

//------------------------------------------------------------------------


#define MATCH_TOLERANCE 0.000001

namespace
{    
    // Data for a single renderable color layer
    struct RenderLayer
    {
        TileModel::ColorData           _layer;
        osg::ref_ptr<const GeoLocator> _locator;
        osg::ref_ptr<osg::Vec2Array>   _texCoords;
        osg::ref_ptr<osg::Vec2Array>   _skirtTexCoords;
        osg::ref_ptr<osg::Vec2Array>   _stitchTexCoords;
        osg::ref_ptr<osg::Vec2Array>   _stitchSkirtTexCoords;
        bool _ownsTexCoords;
        bool _ownsSkirtTexCoords;
        RenderLayer() : _ownsTexCoords(false), _ownsSkirtTexCoords(false) { }
    };

    typedef std::vector< RenderLayer > RenderLayerVector;


    // Record that stores the data for a single masking region.
    struct MaskRecord
    {
        osg::ref_ptr<osg::Vec3dArray> _boundary;
        osg::Vec3d                    _ndcMin, _ndcMax;
        osg::Geometry*                _geom;
        osg::ref_ptr<osg::Vec3Array>  _internal;

        MaskRecord(osg::Vec3dArray* boundary, osg::Vec3d& ndcMin, osg::Vec3d& ndcMax, osg::Geometry* geom) 
            : _boundary(boundary), _ndcMin(ndcMin), _ndcMax(ndcMax), _geom(geom), _internal(new osg::Vec3Array()) { }
    };

    typedef std::vector<MaskRecord> MaskRecordVector;


    typedef std::vector<int> Indices;


    struct Data
    {
        Data(const TileModel* in_model, const MaskLayerVector& in_maskLayers)
            : model     ( in_model ), 
              maskLayers( in_maskLayers )
        {
            surfaceGeode     = 0L;
            surface          = 0L;
            skirtGeode       = 0L;
            skirt            = 0L;
            stitching_skirts = 0L;
            ss_verts         = 0L;
            scaleHeight      = 1.0f;
            createSkirt      = false;
            i_sampleFactor   = 1.0f;
            j_sampleFactor   = 1.0f;
            unifiedSkirtTexCoords       = 0L;
            unifiedStitchSkirtTexCoords = 0L;
            unifiedSurfaceTexCoords     = 0L;
            useVBOs = !Registry::capabilities().preferDisplayListsForStaticGeometry();
        }

        bool                     useVBOs;

        const TileModel*         model;                         // the tile's data model
        const MaskLayerVector&   maskLayers;                    // map-global masking layer set
        osg::ref_ptr<GeoLocator> geoLocator;                    // tile locator adjusted to geocentric
        osg::Vec3d               centerModel;                   // tile center in model (world) coords

        RenderLayerVector        renderLayers;

        // surface data:
        osg::Geode*                   surfaceGeode;
        osg::Geometry*                surface;
        osg::Vec3Array*               surfaceVerts;
        osg::Vec3Array*               normals;
        osg::Vec4Array*               surfaceElevData;
        osg::FloatArray*              surfaceMorphData;
        unsigned                      numVerticesInSurface;
        osg::Vec2Array*               unifiedSurfaceTexCoords;
        osg::ref_ptr<osg::FloatArray> elevations;
        Indices                       indices;
        osg::BoundingSphere           surfaceBound;

        // skirt data:
        osg::Geode*              skirtGeode;
        osg::Geometry*           skirt;
        unsigned                 numVerticesInSkirt;
        osg::Vec2Array*          unifiedSkirtTexCoords;
        //double                   skirtHeight;
        bool                     createSkirt;

        // sampling grid parameters:
        unsigned                 numRows;
        unsigned                 numCols;
        double                   i_sampleFactor;
        double                   j_sampleFactor;
        double                   scaleHeight;
        unsigned                 originalNumRows;
        unsigned                 originalNumCols;
        
        // for masking/stitching:
        MaskRecordVector         maskRecords;
        osg::Geometry*           stitching_skirts;
        osg::Vec3Array*          ss_verts;
        osg::Vec2Array*          unifiedStitchSkirtTexCoords;
    };



    /**
     * Set up the masking records for this build. Here we check all the map's mask layer
     * boundary geometries and find any that intersect the current tile. For an intersection
     * we create a MaskRecord that we'll use later in the process.
     */
    void setupMaskRecords( Data& d )
    {
        // TODO: Set up the boundary sets globally in the TileModelCompiler instead of
        // generating the boundaries every time for every tile.

        for (MaskLayerVector::const_iterator it = d.maskLayers.begin(); it != d.maskLayers.end(); ++it)
        {
          // When displaying Plate Carre, Heights have to be converted from meters to degrees.
          // This is also true for mask feature
          // TODO: adjust this calculation based on the actual EllipsoidModel.
          float scale = d.scaleHeight;
          if (d.model->_tileLocator->getCoordinateSystemType() == osgEarth::GeoLocator::GEOGRAPHIC)
          {
            scale = d.scaleHeight / 111319.0f;
          }

          // TODO: no need to do this for every tile right?
          osg::Vec3dArray* boundary = (*it)->getOrCreateBoundary(
              scale, 
              d.model->_tileLocator->getDataExtent().getSRS() );

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
              d.geoLocator->modelToUnit(min, min_ndc);
              d.geoLocator->modelToUnit(max, max_ndc);

              bool x_match = ((min_ndc.x() >= 0.0 && max_ndc.x() <= 1.0) ||
                              (min_ndc.x() <= 0.0 && max_ndc.x() > 0.0) ||
                              (min_ndc.x() < 1.0 && max_ndc.x() >= 1.0));

              bool y_match = ((min_ndc.y() >= 0.0 && max_ndc.y() <= 1.0) ||
                              (min_ndc.y() <= 0.0 && max_ndc.y() > 0.0) ||
                              (min_ndc.y() < 1.0 && max_ndc.y() >= 1.0));

              if (x_match && y_match)
              {
                osg::Geometry* mask_geom = new osg::Geometry();
                mask_geom->setUseVertexBufferObjects(d.useVBOs);
                d.surfaceGeode->addDrawable(mask_geom);
                d.maskRecords.push_back( MaskRecord(boundary, min_ndc, max_ndc, mask_geom) );
              }
           }
        }

        if (d.maskRecords.size() > 0)
        {
          d.stitching_skirts = new osg::Geometry();
          d.stitching_skirts->setUseVertexBufferObjects(d.useVBOs);
          d.surfaceGeode->addDrawable( d.stitching_skirts );

          d.ss_verts = new osg::Vec3Array();
          d.stitching_skirts->setVertexArray(d.ss_verts);

          if ( d.ss_verts->getVertexBufferObject() )
              d.ss_verts->getVertexBufferObject()->setUsage(GL_STATIC_DRAW_ARB);
        }
    }


    /**
     * Calculates the sample rate and allocates all the vertex, normal, and color
     * arrays for the tile.
     */
    void setupGeometryAttributes( Data& d, double sampleRatio )
    {
        d.numRows = 8;
        d.numCols = 8;
        d.originalNumRows = 8;
        d.originalNumCols = 8;        

        // read the row/column count and skirt size from the model:
        osgTerrain::HeightFieldLayer* hflayer = d.model->_elevationData.getHFLayer();
        if (hflayer)
        {
            d.numCols = hflayer->getNumColumns();
            d.numRows = hflayer->getNumRows();         
            d.originalNumCols = d.numCols;
            d.originalNumRows = d.numRows;
        }

        // calculate the elevation sampling factors that we'll use to step though
        // the tile's NDC space.
        d.i_sampleFactor = 1.0f;
        d.j_sampleFactor = 1.0f;

        if ( sampleRatio != 1.0f )
        {            
            d.numCols = osg::maximum((unsigned int) (float(d.originalNumCols)*sqrtf(sampleRatio)), 4u);
            d.numRows = osg::maximum((unsigned int) (float(d.originalNumRows)*sqrtf(sampleRatio)), 4u);

            d.i_sampleFactor = double(d.originalNumCols-1)/double(d.numCols-1);
            d.j_sampleFactor = double(d.originalNumRows-1)/double(d.numRows-1);
        }


        // calculate the total number of verts:
        d.numVerticesInSurface = d.numCols * d.numRows;
        d.numVerticesInSkirt   = d.createSkirt ? (2 * (d.numCols*2 + d.numRows*2 - 4)) : 0;

        // allocate and assign vertices
        d.surfaceVerts = new osg::Vec3Array();
        d.surfaceVerts->reserve( d.numVerticesInSurface );
        d.surface->setVertexArray( d.surfaceVerts );

        if ( d.surfaceVerts->getVertexBufferObject() )
            d.surfaceVerts->getVertexBufferObject()->setUsage(GL_STATIC_DRAW_ARB);

        // allocate and assign normals
        d.normals = new osg::Vec3Array();
        d.normals->reserve( d.numVerticesInSurface );
        d.surface->setNormalArray( d.normals );
        d.surface->setNormalBinding( osg::Geometry::BIND_PER_VERTEX );

        // allocate and assign color
        osg::Vec4Array* colors = new osg::Vec4Array(1);
        (*colors)[0].set(1.0f,1.0f,1.0f,1.0f);
        d.surface->setColorArray( colors );
        d.surface->setColorBinding( osg::Geometry::BIND_OVERALL );

        // elevation attribution
        // for each vertex, a vec4 containing a unit extrusion vector in [0..2] and the raw elevation in [3]
        d.surfaceElevData = new osg::Vec4Array();
        d.surfaceElevData->reserve( d.numVerticesInSurface );
        d.surface->setVertexAttribArray( osg::Drawable::ATTRIBUTE_6, d.surfaceElevData );
        d.surface->setVertexAttribBinding( osg::Drawable::ATTRIBUTE_6, osg::Geometry::BIND_PER_VERTEX );
        d.surface->setVertexAttribNormalize( osg::Drawable::ATTRIBUTE_6, false );

        d.surfaceMorphData = new osg::FloatArray();
        d.surfaceMorphData->reserve( d.numVerticesInSurface );
        d.surface->setVertexAttribArray( osg::Drawable::ATTRIBUTE_7, d.surfaceMorphData );
        d.surface->setVertexAttribBinding( osg::Drawable::ATTRIBUTE_7, osg::Geometry::BIND_PER_VERTEX );
        d.surface->setVertexAttribNormalize( osg::Drawable::ATTRIBUTE_7, false );

        // temporary data structures for triangulation support
        d.elevations = new osg::FloatArray();
        d.elevations->reserve( d.numVerticesInSurface );
        d.indices.resize( d.numVerticesInSurface, -1 );
    }


    /**
     * Collects a list of color layers to render and assigns each one a texture image
     * unit and a texture coordinate array. Color layers with the same locator setup
     * will share a texture coordinate array.
     *
     * TODO: introduce support in the TextureCompositor to not only share the texture
     * coordinate array, but also the binding attribute slot.
     */
    void setupTextureAttributes( Data& d, TextureCompositor* compositor, CompilerCache& cache )
    {
        if ( compositor->requiresUnitTextureSpace() )
        {
            // for a unified unit texture space, just make a single texture coordinate array.
            // this happens (for example) for a texture-array compositor where all textures
            // share the same unit texture coordinate space.

            d.unifiedSurfaceTexCoords = new osg::Vec2Array();
            d.unifiedSurfaceTexCoords->reserve( d.numVerticesInSurface );
            d.surface->setTexCoordArray( 0, d.unifiedSurfaceTexCoords );

            if ( d.createSkirt )
            {
                d.unifiedSkirtTexCoords = new osg::Vec2Array();
                d.unifiedSkirtTexCoords->reserve( d.numVerticesInSkirt );
                d.skirt->setTexCoordArray( 0, d.unifiedSkirtTexCoords );
            }

            if (d.maskRecords.size() > 0)
            {
                d.unifiedStitchSkirtTexCoords = new osg::Vec2Array();
                //unifiedStitchSkirtTexCoords->reserve( ? );
                d.stitching_skirts->setTexCoordArray( 0, d.unifiedStitchSkirtTexCoords );
            }
        }

        else // if ( !_texCompositor->requiresUnitTextureSpace() )
        {
            // normal texture coordinate sharing. Any color entries that have the same
            // color Locator will share a texcoord array, saving on memory.

            // TODO: not only should we also share the vertex attribute slot (as mentioned
            // above) but we should also share the texture coord arrays across ALL tiles,
            // not just within a single tile. Perhaps a TerrainModel-wide cache.

            //LocatorToTexCoordTable locatorToTexCoordTable;
            //TileModelCompiler::LocatorToTexCoordTable texCoordArrayCache;

            d.renderLayers.reserve( d.model->_colorData.size() );

            // build a list of "render layers", in rendering order, sharing texture coordinate
            // arrays wherever possible.
            for( TileModel::ColorDataByUID::const_iterator i = d.model->_colorData.begin(); i != d.model->_colorData.end(); ++i )
            {
                const TileModel::ColorData& colorLayer = i->second;
                RenderLayer r;
                r._layer = colorLayer;

                const GeoLocator* locator = dynamic_cast<const GeoLocator*>( r._layer.getLocator() );
                if ( locator )
                {
                    // if we have no mask records, we can use the texture coordinate array cache.
                    if ( d.maskRecords.size() == 0 )
                    {
                        const GeoExtent& locex = locator->getDataExtent();
                        const GeoExtent& keyex = d.model->_tileKey.getExtent();

                        osg::Vec4d mat;
                        mat[0] = (keyex.xMin() - locex.xMin())/locex.width();
                        mat[1] = (keyex.yMin() - locex.yMin())/locex.height();
                        mat[2] = (keyex.width() / locex.width());
                        mat[3] = (keyex.height() / locex.height());

                        //OE_DEBUG << "key=" << d.model->_tileKey.str() << ": off=[" <<mat[0]<< ", " <<mat[1] << "] scale=["
                        //    << mat[2]<< ", " << mat[3] << "]" << std::endl;

                        osg::ref_ptr<osg::Vec2Array>& surfaceTexCoords = cache._surfaceTexCoordArrays.get( mat, d.numCols, d.numRows );
                        if ( !surfaceTexCoords.valid() )
                        {
                            // Note: anything in the cache must have its own VBO. No sharing!
                            surfaceTexCoords = new osg::Vec2Array();
                            surfaceTexCoords->setVertexBufferObject( new osg::VertexBufferObject() );
                            surfaceTexCoords->reserve( d.numVerticesInSurface );
                            r._ownsTexCoords = true;
                        }
                        r._texCoords = surfaceTexCoords.get();

                        osg::ref_ptr<osg::Vec2Array>& skirtTexCoords = cache._skirtTexCoordArrays.get( mat, d.numCols, d.numRows );
                        if ( !skirtTexCoords.valid() )
                        {
                            // Note: anything in the cache must have its own VBO. No sharing!
                            skirtTexCoords = new osg::Vec2Array();
                            skirtTexCoords->setVertexBufferObject( new osg::VertexBufferObject() );
                            skirtTexCoords->reserve( d.numVerticesInSkirt );
                            r._ownsSkirtTexCoords = true;
                        }
                        r._skirtTexCoords = skirtTexCoords.get();
                    }

                    else // if ( d.maskRecords.size() > 0 )
                    {
                        // cannot use the tex coord array cache if there are masking records.
                        r._texCoords = new osg::Vec2Array();
                        r._texCoords->reserve( d.numVerticesInSurface );
                        r._ownsTexCoords = true;

                        r._skirtTexCoords = new osg::Vec2Array();
                        r._skirtTexCoords->reserve( d.numVerticesInSkirt );
                        r._ownsSkirtTexCoords = true;
                    
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

                    d.renderLayers.push_back( r );

                    // Note that we don't actually assign the tex coord arrays to the geometry yet.
                    // That must wait until the end. See the comments in assignTextureArrays()
                    // to understand why.
                }
                else
                {
                    OE_WARN << LC << "Found a Locator, but it wasn't a GeoLocator." << std::endl;
                }
            }
        }
    }


    /**
     * Assigns any texture coordinate arrays to the corresponding geometry.
     *
     * NOTE: This has to happen AFTER all other arrays have been assigned to the geometry.
     * Here is why:
     *
     * This compiler attempts to share texture coordinate arrays between tiles where possible.
     * When you call Geometry::setVertexArray (or other similar functions) OSG attempts to find
     * an existing VBO that it can use to attach to the array. If a shared texture coordinate
     * array is in the Geometry, and OSG will find its VBO and attempt to use it so store the
     * buffer data for the new array. In doing do it will be modifying a VBO that is potentially
     * in use by another thread, causing a crash.
     *
     * By assigning the shared arrays last, we prevent this from happening.
     */
    void assignTextureArrays( Data& d, TextureCompositor* compositor )
    {
        for( RenderLayerVector::const_iterator r = d.renderLayers.begin(); r != d.renderLayers.end(); ++r )
        {
            compositor->assignTexCoordArray( d.surface, r->_layer.getUID(), r->_texCoords.get() );
            compositor->assignTexCoordArray( d.skirt,   r->_layer.getUID(), r->_skirtTexCoords.get() );

            // If we have mask stitching geometries, those need tex coords too:
            for ( MaskRecordVector::iterator mr = d.maskRecords.begin(); mr != d.maskRecords.end(); ++mr )
            {
                compositor->assignTexCoordArray( (*mr)._geom, r->_layer.getUID(), r->_stitchTexCoords.get() );
            }

            if (d.stitching_skirts)
            {
                compositor->assignTexCoordArray( d.stitching_skirts, r->_layer.getUID(), r->_stitchSkirtTexCoords.get() );
            }
        }
    }


    /**
     * Iterate over the sampling grid and calculate the vertex positions and normals
     * for each sampling point.
     */
    void createSurfaceGeometry( Data& d, TextureCompositor* compositor )
    {
        d.surfaceBound.init();

        osgTerrain::HeightFieldLayer* elevationLayer = d.model->_elevationData.getHFLayer();

        // populate vertex and tex coord arrays    
        for(unsigned j=0; j < d.numRows; ++j)
        {
            for(unsigned i=0; i < d.numCols; ++i)
            {
                unsigned int iv = j*d.numCols + i;
                osg::Vec3d ndc( ((double)i)/(double)(d.numCols-1), ((double)j)/(double)(d.numRows-1), 0.0);

                bool validValue = true;

                // use the sampling factor to determine the lookup index:
                unsigned i_equiv = d.i_sampleFactor==1.0 ? i : (unsigned) (double(i)*d.i_sampleFactor);
                unsigned j_equiv = d.j_sampleFactor==1.0 ? j : (unsigned) (double(j)*d.j_sampleFactor);

                // raw height:
                float heightValue = 0.0f;

                if ( elevationLayer )
                {
                    validValue = elevationLayer->getValidValue(i_equiv,j_equiv, heightValue);
                    ndc.z() = heightValue; //*scaleHeight; // scaling will be done in the shader
                }

                // First check whether the sampling point falls within a mask's bounding box.
                // If so, skip the sampling and mark it as a mask location
                if ( validValue && d.maskRecords.size() > 0 )
                {
                    for (MaskRecordVector::iterator mr = d.maskRecords.begin(); mr != d.maskRecords.end(); ++mr)
                    {
                        if(ndc.x() >= (*mr)._ndcMin.x() && ndc.x() <= (*mr)._ndcMax.x() &&
                            ndc.y() >= (*mr)._ndcMin.y() && ndc.y() <= (*mr)._ndcMax.y())
                        {
                            validValue = false;
                            d.indices[iv] = -2;

                            (*mr)._internal->push_back(ndc);

                            break;
                        }
                    }
                }
                
                if ( validValue )
                {
                    d.indices[iv] = d.surfaceVerts->size();

                    osg::Vec3d model;
                    d.model->_tileLocator->unitToModel( ndc, model );

                    (*d.surfaceVerts).push_back(model - d.centerModel);

                    // grow the bounding sphere:
                    d.surfaceBound.expandBy( (*d.surfaceVerts).back() );


                    if ( compositor->requiresUnitTextureSpace() )
                    {
                        // the unified unit texture space requires a single, untransformed unit coord [0..1]
                        (*d.unifiedSurfaceTexCoords).push_back( osg::Vec2( ndc.x(), ndc.y() ) );
                    }
                    else
                    {
                        // the separate texture space requires separate transformed texcoords for each layer.
                        for( RenderLayerVector::const_iterator r = d.renderLayers.begin(); r != d.renderLayers.end(); ++r )
                        {
                            if ( r->_ownsTexCoords )
                            {
                                if ( !r->_locator->isEquivalentTo( *d.geoLocator.get() ) )
                                {
                                    osg::Vec3d color_ndc;
                                    osgTerrain::Locator::convertLocalCoordBetween( *d.geoLocator.get(), ndc, *r->_locator.get(), color_ndc );
                                    r->_texCoords->push_back( osg::Vec2( color_ndc.x(), color_ndc.y() ) );
                                }
                                else
                                {
                                    r->_texCoords->push_back( osg::Vec2( ndc.x(), ndc.y() ) );
                                }
                            }
                        }
                    }

                    // record the raw elevation value in our float array for later
                    (*d.elevations).push_back(ndc.z());

                    // compute the local normal
                    osg::Vec3d ndc_one = ndc; ndc_one.z() += 1.0;
                    osg::Vec3d model_one;
                    //_masterLocator->convertLocalToModel(ndc_one, model_one);
                    d.model->_tileLocator->unitToModel(ndc_one, model_one);
                    model_one = model_one - model;
                    model_one.normalize();    

                    //(*normals)[k] = model_one;
                    (*d.normals).push_back(model_one);

                    // store the unit extrusion vector and the raw height value.
                    (*d.surfaceElevData).push_back( osg::Vec4f(model_one.x(), model_one.y(), model_one.z(), heightValue) );

                    float oldHeightValue = heightValue;
                    if ( i>0 && j>0 && i<d.numCols-1 && j<d.numRows-1 && (i%2 == 1 || j%2 == 1) )
                    {
                        float h;
                        if ( HeightFieldUtils::getInterpolatedHeight(
                            elevationLayer->getHeightField(),
                            i_equiv,
                            j_equiv,
                            h ) )
                        {
                            oldHeightValue = h;
                        }
                    }

                    d.surfaceMorphData->push_back( oldHeightValue );
                }
            }
        }
    }


    /**
     * If there are masking records, calculate the vertices to bound the masked area
     * and the internal verticies to populate it. Then build a triangulation of the
     * area inside the masking bounding box and add this to the surface geode.
     */
    void createMaskGeometry( Data& d, TextureCompositor* compositor )
    {
        osgTerrain::HeightFieldLayer* elevationLayer = d.model->_elevationData.getHFLayer();

        for (MaskRecordVector::iterator mr = d.maskRecords.begin(); mr != d.maskRecords.end(); ++mr)
        {
            int min_i = (int)floor((*mr)._ndcMin.x() * (double)(d.numCols-1));
            if (min_i < 0) min_i = 0;
            if (min_i >= (int)d.numCols) min_i = d.numCols - 1;

            int min_j = (int)floor((*mr)._ndcMin.y() * (double)(d.numRows-1));
            if (min_j < 0) min_j = 0;
            if (min_j >= (int)d.numCols) min_j = d.numCols - 1;

            int max_i = (int)ceil((*mr)._ndcMax.x() * (double)(d.numCols-1));
            if (max_i < 0) max_i = 0;
            if (max_i >= (int)d.numCols) max_i = d.numCols - 1;

            int max_j = (int)ceil((*mr)._ndcMax.y() * (double)(d.numRows-1));
            if (max_j < 0) max_j = 0;
            if (max_j >= (int)d.numCols) max_j = d.numCols - 1;

            if (min_i >= 0 && max_i >= 0 && min_j >= 0 && max_j >= 0)
            {
                int num_i = max_i - min_i + 1;
                int num_j = max_j - min_j + 1;

                osg::ref_ptr<Polygon> maskSkirtPoly = new Polygon();
                maskSkirtPoly->resize(num_i * 2 + num_j * 2 - 4);

                for (int i = 0; i < num_i; i++)
                {
                    //int index = indices[min_j*numColumns + i + min_i];
                    {
                        osg::Vec3d ndc( ((double)(i + min_i))/(double)(d.numCols-1), ((double)min_j)/(double)(d.numRows-1), 0.0);

                        if (elevationLayer)
                        {
                            unsigned i_equiv = d.i_sampleFactor==1.0 ? i + min_i : (unsigned) (double(i + min_i)*d.i_sampleFactor);
                            unsigned j_equiv = d.j_sampleFactor==1.0 ? min_j : (unsigned) (double(min_j)*d.j_sampleFactor);

                            float value = 0.0f;
                            if (elevationLayer->getValidValue(i_equiv,j_equiv, value))
                                ndc.z() = value*d.scaleHeight;
                        }

                        (*maskSkirtPoly)[i] = ndc;
                    }

                    //index = indices[max_j*numColumns + i + min_i];
                    {
                        osg::Vec3d ndc( ((double)(i + min_i))/(double)(d.numCols-1), ((double)max_j)/(double)(d.numRows-1), 0.0);

                        if (elevationLayer)
                        {
                            unsigned i_equiv = d.i_sampleFactor==1.0 ? i + min_i : (unsigned) (double(i + min_i)*d.i_sampleFactor);
                            unsigned j_equiv = d.j_sampleFactor==1.0 ? max_j : (unsigned) (double(max_j)*d.j_sampleFactor);

                            float value = 0.0f;
                            if (elevationLayer->getValidValue(i_equiv,j_equiv, value))
                                ndc.z() = value*d.scaleHeight;
                        }

                        (*maskSkirtPoly)[i + (2 * num_i + num_j - 3) - 2 * i] = ndc;
                    }
                }
                for (int j = 0; j < num_j - 2; j++)
                {
                    //int index = indices[(min_j + j + 1)*numColumns + max_i];
                    {
                        osg::Vec3d ndc( ((double)max_i)/(double)(d.numCols-1), ((double)(min_j + j + 1))/(double)(d.numRows-1), 0.0);

                        if (elevationLayer)
                        {
                            unsigned int i_equiv = d.i_sampleFactor==1.0 ? max_i : (unsigned int) (double(max_i)*d.i_sampleFactor);
                            unsigned int j_equiv = d.j_sampleFactor==1.0 ? min_j + j + 1 : (unsigned int) (double(min_j + j + 1)*d.j_sampleFactor);

                            float value = 0.0f;
                            if (elevationLayer->getValidValue(i_equiv,j_equiv, value))
                                ndc.z() = value*d.scaleHeight;
                        }

                        (*maskSkirtPoly)[j + num_i] = ndc;
                    }

                    //index = indices[(min_j + j + 1)*numColumns + min_i];
                    {
                        osg::Vec3d ndc( ((double)min_i)/(double)(d.numCols-1), ((double)(min_j + j + 1))/(double)(d.numRows-1), 0.0);

                        if (elevationLayer)
                        {
                            unsigned int i_equiv = d.i_sampleFactor==1.0 ? min_i : (unsigned int) (double(min_i)*d.i_sampleFactor);
                            unsigned int j_equiv = d.j_sampleFactor==1.0 ? min_j + j + 1 : (unsigned int) (double(min_j + j + 1)*d.j_sampleFactor);

                            float value = 0.0f;
                            if (elevationLayer->getValidValue(i_equiv,j_equiv, value))
                                ndc.z() = value*d.scaleHeight;
                        }

                        (*maskSkirtPoly)[j + (2 * num_i + 2 * num_j - 5) - 2 * j] = ndc;
                    }
                }

                //Create local polygon representing mask
                osg::ref_ptr<Polygon> maskPoly = new Polygon();
                for (osg::Vec3dArray::iterator it = (*mr)._boundary->begin(); it != (*mr)._boundary->end(); ++it)
                {
                    osg::Vec3d local;
                    d.geoLocator->convertModelToLocal(*it, local);
                    maskPoly->push_back(local);
                }


                // Use delaunay triangulation for stitching:

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
                osg::ref_ptr<Geometry> maskCrop;
                maskPoly->crop(maskSkirtPoly.get(), maskCrop);

                GeometryIterator i( maskCrop.get(), false );
                while( i.hasMore() )
                {
                    Geometry* part = i.next();
                    if (!part)
                        continue;

                    if (part->getType() == Geometry::TYPE_POLYGON)
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
                    for (Polygon::iterator mit = maskSkirtPoly->begin(); mit != maskSkirtPoly->end(); ++mit)
                    {
                        if (osg::absolute((*mit).x() - (*it).x()) < MATCH_TOLERANCE && osg::absolute((*mit).y() - (*it).y()) < MATCH_TOLERANCE)
                        {
                            (*it).z() = (*mit).z();
                            zSet += 1;
                            break;
                        }
                    }

                    //Look for verts that belong to the mask polygon
                    for (Polygon::iterator mit = maskPoly->begin(); mit != maskPoly->end(); ++mit)
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
                        for (Polygon::iterator mit = maskPoly->begin(); mit != maskPoly->end(); ++mit)
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
                if (compositor->requiresUnitTextureSpace())
                {
                    unifiedStitchTexCoords = new osg::Vec2Array();
                    unifiedStitchTexCoords->reserve(trig->getInputPointArray()->size());
                    stitch_geom->setTexCoordArray(0, unifiedStitchTexCoords);
                }
                else if ( d.renderLayers.size() > 0 )
                {
                    for (unsigned int i = 0; i < d.renderLayers.size(); ++i)
                    {
                        d.renderLayers[i]._stitchTexCoords->reserve(trig->getInputPointArray()->size());
                    }
                }

                // Iterate through point to convert to model coords, calculate normals, and set up tex coords
                int norm_i = -1;
                for (osg::Vec3Array::iterator it = trig->getInputPointArray()->begin(); it != trig->getInputPointArray()->end(); ++it)
                {
                    // get model coords
                    osg::Vec3d model;
                    d.model->_tileLocator->convertLocalToModel(*it, model);
                    model = model - d.centerModel;

                    stitch_verts->push_back(model);

                    // calc normals
                    osg::Vec3d local_one(*it);
                    local_one.z() += 1.0;
                    osg::Vec3d model_one;
                    d.model->_tileLocator->convertLocalToModel( local_one, model_one );
                    model_one = model_one - model;
                    model_one.normalize();
                    (*stitch_norms)[++norm_i] = model_one;

                    // set up text coords
                    if (compositor->requiresUnitTextureSpace())
                    {
                        unifiedStitchTexCoords->push_back(osg::Vec2((*it).x(), (*it).y()));
                    }
                    else if (d.renderLayers.size() > 0)
                    {
                        for (unsigned int i = 0; i < d.renderLayers.size(); ++i)
                        {
                            if (!d.renderLayers[i]._locator->isEquivalentTo( *d.geoLocator.get() )) //*masterTextureLocator.get()))
                            {
                                osg::Vec3d color_ndc;
                                osgTerrain::Locator::convertLocalCoordBetween(*d.geoLocator.get(), (*it), *d.renderLayers[i]._locator.get(), color_ndc);
                                //osgTerrain::Locator::convertLocalCoordBetween(*masterTextureLocator.get(), (*it), *renderLayers[i]._locator.get(), color_ndc);
                                d.renderLayers[i]._stitchTexCoords->push_back(osg::Vec2(color_ndc.x(), color_ndc.y()));
                            }
                            else
                            {
                                d.renderLayers[i]._stitchTexCoords->push_back(osg::Vec2((*it).x(), (*it).y()));
                            }
                        }
                    }
                }


                // Get triangles from triangulator and add as primative set to the geometry
                stitch_geom->addPrimitiveSet(trig->getTriangles());
            }
        }
    }


    /**
     * Build the geometry for the tile "skirts" -- this the vertical geometry around the
     * tile edges that hides the gap effect caused when you render two adjacent tiles at
     * different LODs.
     */
    void createSkirtGeometry( Data& d, TextureCompositor* compositor, double skirtRatio )
    {
        // surface normals will double as our skirt extrusion vectors
        osg::Vec3Array* skirtVectors = d.normals;

        // find the skirt height
        double skirtHeight = d.surfaceBound.radius() * skirtRatio;

        // build the verts first:
        osg::Vec3Array* skirtVerts = new osg::Vec3Array();
        osg::Vec3Array* skirtNormals = new osg::Vec3Array();
        osg::Vec4Array* skirtElevData = new osg::Vec4Array();
        osg::FloatArray* skirtMorphData = new osg::FloatArray();

        skirtVerts->reserve( d.numVerticesInSkirt );
        skirtNormals->reserve( d.numVerticesInSkirt );
        skirtElevData->reserve( d.numVerticesInSkirt );
        skirtMorphData->reserve( d.numVerticesInSkirt );

        Indices skirtBreaks;
        skirtBreaks.reserve( d.numVerticesInSkirt );
        skirtBreaks.push_back(0);

        // bottom:
        for( unsigned int c=0; c<d.numCols-1; ++c )
        {
            int orig_i = d.indices[c];

            if (orig_i < 0)
            {
                if (skirtBreaks.back() != skirtVerts->size())
                    skirtBreaks.push_back(skirtVerts->size());
            }
            else
            {
                const osg::Vec3f& surfaceVert = (*d.surfaceVerts)[orig_i];
                skirtVerts->push_back( surfaceVert );
                skirtVerts->push_back( surfaceVert - ((*skirtVectors)[orig_i])*skirtHeight );

                const osg::Vec3f& surfaceNormal = (*d.normals)[orig_i];
                skirtNormals->push_back( surfaceNormal );
                skirtNormals->push_back( surfaceNormal );

                const osg::Vec4f& elevData = (*d.surfaceElevData)[orig_i];
                skirtElevData->push_back( elevData );
                skirtElevData->push_back( elevData - osg::Vec4f(0,0,0,skirtHeight) );
                const float& morphData = (*d.surfaceMorphData)[orig_i];
                skirtMorphData->push_back( morphData );
                skirtMorphData->push_back( morphData - skirtHeight );

                if ( compositor->requiresUnitTextureSpace() )
                {
                    d.unifiedSkirtTexCoords->push_back( (*d.unifiedSurfaceTexCoords)[orig_i] );
                    d.unifiedSkirtTexCoords->push_back( (*d.unifiedSurfaceTexCoords)[orig_i] );
                }
                else if ( d.renderLayers.size() > 0 )
                {
                    for (unsigned int i = 0; i < d.renderLayers.size(); ++i)
                    {
                        if ( d.renderLayers[i]._ownsSkirtTexCoords )
                        {
                            const osg::Vec2& tc = (*d.renderLayers[i]._texCoords.get())[orig_i];
                            d.renderLayers[i]._skirtTexCoords->push_back( tc );
                            d.renderLayers[i]._skirtTexCoords->push_back( tc );
                        }
                    }
                }
            }
        }

        // right:
        for( unsigned int r=0; r<d.numRows-1; ++r )
        {
            int orig_i = d.indices[r*d.numCols+(d.numCols-1)];
            if (orig_i < 0)
            {
                if (skirtBreaks.back() != skirtVerts->size())
                    skirtBreaks.push_back(skirtVerts->size());
            }
            else
            {
                const osg::Vec3f& surfaceVert = (*d.surfaceVerts)[orig_i];
                skirtVerts->push_back( surfaceVert );
                skirtVerts->push_back( surfaceVert - ((*skirtVectors)[orig_i])*skirtHeight );

                const osg::Vec3f& surfaceNormal = (*d.normals)[orig_i];
                skirtNormals->push_back( surfaceNormal );
                skirtNormals->push_back( surfaceNormal );

                const osg::Vec4f& elevData = (*d.surfaceElevData)[orig_i];
                skirtElevData->push_back( elevData );
                skirtElevData->push_back( elevData - osg::Vec4f(0,0,0,skirtHeight) );
                const float& morphData = (*d.surfaceMorphData)[orig_i];
                skirtMorphData->push_back( morphData );
                skirtMorphData->push_back( morphData - skirtHeight );

                if ( compositor->requiresUnitTextureSpace() )
                {
                    d.unifiedSkirtTexCoords->push_back( (*d.unifiedSurfaceTexCoords)[orig_i] );
                    d.unifiedSkirtTexCoords->push_back( (*d.unifiedSurfaceTexCoords)[orig_i] );
                }
                else if ( d.renderLayers.size() > 0 )
                {
                    for (unsigned int i = 0; i < d.renderLayers.size(); ++i)
                    {
                        if ( d.renderLayers[i]._ownsSkirtTexCoords )
                        {
                            const osg::Vec2& tc = (*d.renderLayers[i]._texCoords.get())[orig_i];
                            d.renderLayers[i]._skirtTexCoords->push_back( tc );
                            d.renderLayers[i]._skirtTexCoords->push_back( tc );
                        }
                    }
                }
            }
        }

        // top:
        for( int c=d.numCols-1; c>0; --c )
        {
            int orig_i = d.indices[(d.numRows-1)*d.numCols+c];
            if (orig_i < 0)
            {
                if (skirtBreaks.back() != skirtVerts->size())
                    skirtBreaks.push_back(skirtVerts->size());
            }
            else
            {
                const osg::Vec3f& surfaceVert = (*d.surfaceVerts)[orig_i];
                skirtVerts->push_back( surfaceVert );
                skirtVerts->push_back( surfaceVert - ((*skirtVectors)[orig_i])*skirtHeight );

                const osg::Vec3f& surfaceNormal = (*d.normals)[orig_i];
                skirtNormals->push_back( surfaceNormal );
                skirtNormals->push_back( surfaceNormal );

                const osg::Vec4f& elevData = (*d.surfaceElevData)[orig_i];
                skirtElevData->push_back( elevData );
                skirtElevData->push_back( elevData - osg::Vec4f(0,0,0,skirtHeight) );
                const float& morphData = (*d.surfaceMorphData)[orig_i];
                skirtMorphData->push_back( morphData );
                skirtMorphData->push_back( morphData - skirtHeight );

                if ( compositor->requiresUnitTextureSpace() )
                {
                    d.unifiedSkirtTexCoords->push_back( (*d.unifiedSurfaceTexCoords)[orig_i] );
                    d.unifiedSkirtTexCoords->push_back( (*d.unifiedSurfaceTexCoords)[orig_i] );
                }
                else if ( d.renderLayers.size() > 0 )
                {
                    for (unsigned int i = 0; i < d.renderLayers.size(); ++i)
                    {
                        if ( d.renderLayers[i]._ownsSkirtTexCoords )
                        {
                            const osg::Vec2& tc = (*d.renderLayers[i]._texCoords.get())[orig_i];
                            d.renderLayers[i]._skirtTexCoords->push_back( tc );
                            d.renderLayers[i]._skirtTexCoords->push_back( tc );
                        }
                    }
                }
            }
        }

        // left:
        for( int r=d.numRows-1; r>=0; --r )
        {
            int orig_i = d.indices[r*d.numCols];
            if (orig_i < 0)
            {
                if (skirtBreaks.back() != skirtVerts->size())
                    skirtBreaks.push_back(skirtVerts->size());
            }
            else
            {
                const osg::Vec3f& surfaceVert = (*d.surfaceVerts)[orig_i];
                skirtVerts->push_back( surfaceVert );
                skirtVerts->push_back( surfaceVert - ((*skirtVectors)[orig_i])*skirtHeight );

                const osg::Vec3f& surfaceNormal = (*d.normals)[orig_i];
                skirtNormals->push_back( surfaceNormal );
                skirtNormals->push_back( surfaceNormal );

                const osg::Vec4f& elevData = (*d.surfaceElevData)[orig_i];
                skirtElevData->push_back( elevData );
                skirtElevData->push_back( elevData - osg::Vec4f(0,0,0,skirtHeight) );
                const float& morphData = (*d.surfaceMorphData)[orig_i];
                skirtMorphData->push_back( morphData );
                skirtMorphData->push_back( morphData - skirtHeight );

                if ( compositor->requiresUnitTextureSpace() )
                {
                    d.unifiedSkirtTexCoords->push_back( (*d.unifiedSurfaceTexCoords)[orig_i] );
                    d.unifiedSkirtTexCoords->push_back( (*d.unifiedSurfaceTexCoords)[orig_i] );
                }
                else if ( d.renderLayers.size() > 0 )
                {
                    for (unsigned int i = 0; i < d.renderLayers.size(); ++i)
                    {
                        if ( d.renderLayers[i]._ownsSkirtTexCoords )
                        {
                            const osg::Vec2& tc = (*d.renderLayers[i]._texCoords.get())[orig_i];
                            d.renderLayers[i]._skirtTexCoords->push_back( tc );
                            d.renderLayers[i]._skirtTexCoords->push_back( tc );
                        }
                    }
                }
            }
        }

        d.skirt->setVertexArray( skirtVerts );
        if ( skirtVerts->getVertexBufferObject() )
            skirtVerts->getVertexBufferObject()->setUsage(GL_STATIC_DRAW_ARB);

        d.skirt->setNormalArray( skirtNormals );
        d.skirt->setNormalBinding( osg::Geometry::BIND_PER_VERTEX );

        d.skirt->setVertexAttribArray    (osg::Drawable::ATTRIBUTE_6, skirtElevData );
        d.skirt->setVertexAttribBinding  (osg::Drawable::ATTRIBUTE_6, osg::Geometry::BIND_PER_VERTEX);
        d.skirt->setVertexAttribNormalize(osg::Drawable::ATTRIBUTE_6, false);

        d.skirt->setVertexAttribArray    (osg::Drawable::ATTRIBUTE_7, skirtMorphData );
        d.skirt->setVertexAttribBinding  (osg::Drawable::ATTRIBUTE_7, osg::Geometry::BIND_PER_VERTEX);
        d.skirt->setVertexAttribNormalize(osg::Drawable::ATTRIBUTE_7, false);


        // GW: not sure why this break stuff is here...?
#if 0
        //Add a primative set for each continuous skirt strip
        skirtBreaks.push_back(skirtVerts->size());
        for (int p=1; p < (int)skirtBreaks.size(); p++)
            d.skirt->addPrimitiveSet( new osg::DrawArrays( GL_TRIANGLE_STRIP, skirtBreaks[p-1], skirtBreaks[p] - skirtBreaks[p-1] ) );
#else
        d.skirt->addPrimitiveSet( new osg::DrawArrays(GL_TRIANGLE_STRIP, 0, skirtVerts->size()) );
#endif
    }



    /**
     * Builds triangles for the surface geometry, and recalculates the surface normals
     * to be optimized for slope.
     */
    void tessellateSurfaceGeometry( Data& d, bool optimizeTriangleOrientation, bool normalizeEdges )
    {    
        bool swapOrientation = !(d.model->_tileLocator->orientationOpenGL());
        bool recalcNormals   = d.model->_elevationData.getHFLayer() != 0L;

        osg::DrawElements* elements;

        if ( d.surfaceVerts->size() < 0xFF )
            elements = new osg::DrawElementsUByte(GL_TRIANGLES);
        else if ( d.surfaceVerts->size() < 0xFFFF )
            elements = new osg::DrawElementsUShort(GL_TRIANGLES);
        else
            elements = new osg::DrawElementsUShort(GL_TRIANGLES);

        //DrawElementsUShort* elements = new osg::DrawElementsUShort(GL_TRIANGLES);
        elements->reserveElements((d.numRows-1) * (d.numCols-1) * 6);
        d.surface->addPrimitiveSet( elements );

        if ( recalcNormals )
        {
            // first clear out all the normals:
            for( osg::Vec3Array::iterator nitr = d.normals->begin(); nitr != d.normals->end(); ++nitr )
            {
                nitr->set( 0.0f, 0.0f, 0.0f );
            }
        }

        for(unsigned j=0; j<d.numRows-1; ++j)
        {
            for(unsigned i=0; i<d.numCols-1; ++i)
            {
                int i00;
                int i01;
                if (swapOrientation)
                {
                    i01 = j*d.numCols + i;
                    i00 = i01+d.numCols;
                }
                else
                {
                    i00 = j*d.numCols + i;
                    i01 = i00+d.numCols;
                }

                int i10 = i00+1;
                int i11 = i01+1;

                // remap indices to final vertex positions
                i00 = d.indices[i00];
                i01 = d.indices[i01];
                i10 = d.indices[i10];
                i11 = d.indices[i11];

                unsigned int numValid = 0;
                if (i00>=0) ++numValid;
                if (i01>=0) ++numValid;
                if (i10>=0) ++numValid;
                if (i11>=0) ++numValid;                

                if (numValid==4)
                {
                    bool VALID = true;
                    for (MaskRecordVector::iterator mr = d.maskRecords.begin(); mr != d.maskRecords.end(); ++mr)
                    {
                        float min_i = (*mr)._ndcMin.x() * (double)(d.numCols-1);
                        float min_j = (*mr)._ndcMin.y() * (double)(d.numRows-1);
                        float max_i = (*mr)._ndcMax.x() * (double)(d.numCols-1);
                        float max_j = (*mr)._ndcMax.y() * (double)(d.numRows-1);

                        // We test if mask is completely in square
                        if(i+1 >= min_i && i <= max_i && j+1 >= min_j && j <= max_j)
                        {
                            VALID = false;
                            break;
                        }
                    }

                    if (VALID) {
                        float e00 = (*d.elevations)[i00];
                        float e10 = (*d.elevations)[i10];
                        float e01 = (*d.elevations)[i01];
                        float e11 = (*d.elevations)[i11];

                        osg::Vec3f& v00 = (*d.surfaceVerts)[i00];
                        osg::Vec3f& v10 = (*d.surfaceVerts)[i10];
                        osg::Vec3f& v01 = (*d.surfaceVerts)[i01];
                        osg::Vec3f& v11 = (*d.surfaceVerts)[i11];

                        if (!optimizeTriangleOrientation || (e00-e11)<fabsf(e01-e10))
                        {
                            elements->addElement(i01);
                            elements->addElement(i00);
                            elements->addElement(i11);

                            elements->addElement(i00);
                            elements->addElement(i10);
                            elements->addElement(i11);

                            if (recalcNormals)
                            {                        
                                osg::Vec3 normal1 = (v00-v01) ^ (v11-v01);
                                (*d.normals)[i01] += normal1;
                                (*d.normals)[i00] += normal1;
                                (*d.normals)[i11] += normal1;

                                osg::Vec3 normal2 = (v10-v00) ^ (v11-v00);
                                (*d.normals)[i00] += normal2;
                                (*d.normals)[i10] += normal2;
                                (*d.normals)[i11] += normal2;
                            }
                        }
                        else
                        {
                            elements->addElement(i01);
                            elements->addElement(i00);
                            elements->addElement(i10);

                            elements->addElement(i01);
                            elements->addElement(i10);
                            elements->addElement(i11);

                            if (recalcNormals)
                            {                       
                                osg::Vec3 normal1 = (v00-v01) ^ (v10-v01);
                                (*d.normals)[i01] += normal1;
                                (*d.normals)[i00] += normal1;
                                (*d.normals)[i10] += normal1;

                                osg::Vec3 normal2 = (v10-v01) ^ (v11-v01);
                                (*d.normals)[i01] += normal2;
                                (*d.normals)[i10] += normal2;
                                (*d.normals)[i11] += normal2;
                            }
                        }
                    }
                }
            }
        }        
        
        if (recalcNormals && normalizeEdges)
        {            
            OE_DEBUG << "Normalizing edges" << std::endl;
            //Compute the edge normals if we have neighbor data
            //Get all the neighbors
            osg::ref_ptr< osg::HeightField > w_neighbor  = d.model->_elevationData.getNeighbor( -1, 0 );
            osg::ref_ptr< osg::HeightField > e_neighbor  = d.model->_elevationData.getNeighbor( 1, 0 );            
            osg::ref_ptr< osg::HeightField > s_neighbor  = d.model->_elevationData.getNeighbor( 0, 1 );
            osg::ref_ptr< osg::HeightField > n_neighbor  = d.model->_elevationData.getNeighbor( 0, -1 );
            
            //Recalculate the west side
            if (w_neighbor.valid() && w_neighbor->getNumColumns() == d.originalNumCols && w_neighbor->getNumRows() == d.originalNumRows)            
            {                                     
                osg::ref_ptr< osg::Vec3Array > boundaryVerts = new osg::Vec3Array();
                boundaryVerts->reserve( 2 * d.numRows );

                std::vector< float > boundaryElevations;
                boundaryElevations.reserve( 2 * d.numRows );
                
                //Compute the verts for the west side
                for (int j = 0; j < (int)d.numRows; j++)
                {
                    for (int i = (int)d.numCols-2; i <= (int)d.numCols-1; i++)
                    {                          
                        osg::Vec3d ndc( (double)(i - static_cast<int>(d.numCols-1))/(double)(d.numCols-1), ((double)j)/(double)(d.numRows-1), 0.0);                                                                        

                        // use the sampling factor to determine the lookup index:
                        unsigned i_equiv = d.i_sampleFactor==1.0 ? i : (unsigned) (double(i)*d.i_sampleFactor);
                        unsigned j_equiv = d.j_sampleFactor==1.0 ? j : (unsigned) (double(j)*d.j_sampleFactor);

                        //TODO:  Should probably use an interpolated method here
                        float heightValue = w_neighbor->getHeight( i_equiv, j_equiv );
                        ndc.z() = heightValue;                        

                        osg::Vec3d model;
                        d.model->_tileLocator->unitToModel( ndc, model );
                        osg::Vec3d v = model - d.centerModel;
                        boundaryVerts->push_back( v );
                        boundaryElevations.push_back( heightValue );                        
                    }
                }   

                //The boundary verts are now populated, so go through and triangulate them add add the normals to the existing normal array
                for (int j = 0; j < (int)d.numRows-1; j++)
                {                    
                    int i00;
                    int i01;
                    int i = 0;
                    if (swapOrientation)
                    {
                        i01 = j*d.numCols + i;
                        i00 = i01+d.numCols;
                    }
                    else
                    {
                        i00 = j*d.numCols + i;
                        i01 = i00+d.numCols;
                    }



                    //remap indices to final vertex position
                    i00 = d.indices[i00];
                    i01 = d.indices[i01];

                    int baseIndex = 2 * j;
                    osg::Vec3f& v00 = (*boundaryVerts)[baseIndex    ];
                    osg::Vec3f& v10 = (*boundaryVerts)[baseIndex + 1];
                    osg::Vec3f& v01 = (*boundaryVerts)[baseIndex + 2];
                    osg::Vec3f& v11 = (*boundaryVerts)[baseIndex + 3];

                    float e00 = boundaryElevations[baseIndex];
                    float e10 = boundaryElevations[baseIndex + 1];
                    float e01 = boundaryElevations[baseIndex + 2];
                    float e11 = boundaryElevations[baseIndex + 3];

                   
                    if (!optimizeTriangleOrientation || (e00-e11)<fabsf(e01-e10))
                    {                            
                        osg::Vec3 normal1 = (v00-v01) ^ (v11-v01);
                        (*d.normals)[i01] += normal1;                        

                        osg::Vec3 normal2 = (v10-v00) ^ (v11-v00);
                        (*d.normals)[i00] += normal2;                        
                        (*d.normals)[i01] += normal2;                                                
                    }
                    else
                    {                            
                        osg::Vec3 normal1 = (v00-v01) ^ (v10-v01);
                        (*d.normals)[i00] += normal1;                                               

                        osg::Vec3 normal2 = (v10-v01) ^ (v11-v01);
                        (*d.normals)[i00] += normal2;                                               
                        (*d.normals)[i01] += normal2;                        
                    }
                }
            }

                        
            //Recalculate the east side
            if (e_neighbor.valid() && e_neighbor->getNumColumns() == d.originalNumCols && e_neighbor->getNumRows() == d.originalNumRows)            
            {                           
                osg::ref_ptr< osg::Vec3Array > boundaryVerts = new osg::Vec3Array();
                boundaryVerts->reserve( 2 * d.numRows );

                std::vector< float > boundaryElevations;
                boundaryElevations.reserve( 2 * d.numRows );

                //Compute the verts for the east side
                for (int j = 0; j < (int)d.numRows; j++)
                {
                    for (int i = 0; i <= 1; i++)
                    {                           
                        osg::Vec3d ndc( ((double)(d.numCols -1 + i))/(double)(d.numCols-1), ((double)j)/(double)(d.numRows-1), 0.0);

                        unsigned i_equiv = d.i_sampleFactor==1.0 ? i : (unsigned) (double(i)*d.i_sampleFactor);
                        unsigned j_equiv = d.j_sampleFactor==1.0 ? j : (unsigned) (double(j)*d.j_sampleFactor);
                        
                        //TODO:  Should probably use an interpolated method here
                        float heightValue = e_neighbor->getHeight( i_equiv, j_equiv );
                        ndc.z() = heightValue;                        

                        osg::Vec3d model;
                        d.model->_tileLocator->unitToModel( ndc, model );
                        osg::Vec3d v = model - d.centerModel;
                        boundaryVerts->push_back( v );
                        boundaryElevations.push_back( heightValue );                        
                    }
                }   

                //The boundary verts are now populated, so go through and triangulate them add add the normals to the existing normal array
                for (int j = 0; j < (int)d.numRows-1; j++)
                {                    
                    int i00;
                    int i01;
                    int i = d.numCols-1;
                    if (swapOrientation)
                    {
                        i01 = j*d.numCols + i;
                        i00 = i01+d.numCols;
                    }
                    else
                    {
                        i00 = j*d.numCols + i;
                        i01 = i00+d.numCols;
                    }

                    //remap indices to final vertex position
                    i00 = d.indices[i00];
                    i01 = d.indices[i01];

                    int baseIndex = 2 * j;
                    osg::Vec3f& v00 = (*boundaryVerts)[baseIndex    ];
                    osg::Vec3f& v10 = (*boundaryVerts)[baseIndex + 1];
                    osg::Vec3f& v01 = (*boundaryVerts)[baseIndex + 2];
                    osg::Vec3f& v11 = (*boundaryVerts)[baseIndex + 3];

                    float e00 = boundaryElevations[baseIndex];
                    float e10 = boundaryElevations[baseIndex + 1];
                    float e01 = boundaryElevations[baseIndex + 2];
                    float e11 = boundaryElevations[baseIndex + 3];

                   
                    if (!optimizeTriangleOrientation || (e00-e11)<fabsf(e01-e10))
                    {                            
                        osg::Vec3 normal1 = (v00-v01) ^ (v11-v01);                       
                        (*d.normals)[i00] += normal1;                        
                        (*d.normals)[i01] += normal1;

                        osg::Vec3 normal2 = (v10-v00) ^ (v11-v00);                        
                        (*d.normals)[i00] += normal2;                                                
                    }
                    else
                    {                            
                        osg::Vec3 normal1 = (v00-v01) ^ (v10-v01);
                        (*d.normals)[i00] += normal1;                        
                        (*d.normals)[i01] += normal1;                                                                        

                        osg::Vec3 normal2 = (v10-v01) ^ (v11-v01);
                        (*d.normals)[i01] += normal2;                        
                    }
                }
            }

            //Recalculate the north side
            if (n_neighbor.valid() && n_neighbor->getNumColumns() == d.originalNumCols && n_neighbor->getNumRows() == d.originalNumRows)            
            {                 
                osg::ref_ptr< osg::Vec3Array > boundaryVerts = new osg::Vec3Array();
                boundaryVerts->reserve( 2 * d.numCols );

                std::vector< float > boundaryElevations;
                boundaryElevations.reserve( 2 * d.numCols );

                //Compute the verts for the north side               
                for (int j = 0; j <= 1; j++)
                {
                    for (int i = 0; i < (int)d.numCols; i++)                    
                    {                           
                        osg::Vec3d ndc( (double)(i)/(double)(d.numCols-1), (double)(d.numRows -1 + j)/(double)(d.numRows-1), 0.0);
                        //osg::Vec3d ndc( (double)(i)/(double)(d.numCols-1), (double)(-static_cast<int>(j))/(double)(d.numRows-1), 0.0);                        

                        unsigned i_equiv = d.i_sampleFactor==1.0 ? i : (unsigned) (double(i)*d.i_sampleFactor);
                        unsigned j_equiv = d.j_sampleFactor==1.0 ? j : (unsigned) (double(j)*d.j_sampleFactor);
                        
                        //TODO:  Should probably use an interpolated method here
                        float heightValue = n_neighbor->getHeight( i_equiv, j_equiv );
                        ndc.z() = heightValue;                        

                        osg::Vec3d model;
                        d.model->_tileLocator->unitToModel( ndc, model );
                        osg::Vec3d v = model - d.centerModel;
                        boundaryVerts->push_back( v );
                        boundaryElevations.push_back( heightValue );                        
                    }
                }   

                //The boundary verts are now populated, so go through and triangulate them add add the normals to the existing normal array                
                for (int i = 0; i < (int)d.numCols-1; i++)
                {                    
                    int i00;                    
                    int j = d.numRows-1;
                    if (swapOrientation)
                    {         
                        int i01 = j * d.numCols + i;
                        i00 = i01+d.numCols;
                    }
                    else
                    {
                        i00 = j*d.numCols + i;                        
                    }

                    int i10 = i00+1;

                    //remap indices to final vertex position
                    i00 = d.indices[i00];
                    i10 = d.indices[i10];

                    int baseIndex = i;
                    osg::Vec3f& v00 = (*boundaryVerts)[baseIndex    ];
                    osg::Vec3f& v10 = (*boundaryVerts)[baseIndex + 1];
                    osg::Vec3f& v01 = (*boundaryVerts)[baseIndex + d.numCols];
                    osg::Vec3f& v11 = (*boundaryVerts)[baseIndex + d.numCols + 1];

                    float e00 = boundaryElevations[baseIndex];
                    float e10 = boundaryElevations[baseIndex + 1];
                    float e01 = boundaryElevations[baseIndex + d.numCols];
                    float e11 = boundaryElevations[baseIndex + d.numCols + 1];

                   
                    if (!optimizeTriangleOrientation || (e00-e11)<fabsf(e01-e10))
                    {                            
                        osg::Vec3 normal1 = (v00-v01) ^ (v11-v01);                       
                        (*d.normals)[i00] += normal1;                        
                        (*d.normals)[i10] += normal1;

                        osg::Vec3 normal2 = (v10-v00) ^ (v11-v00);                        
                        (*d.normals)[i10] += normal2;                                                
                    }
                    else
                    {                            
                        osg::Vec3 normal1 = (v00-v01) ^ (v10-v01);
                        (*d.normals)[i00] += normal1;                                                

                        osg::Vec3 normal2 = (v10-v01) ^ (v11-v01);
                        (*d.normals)[i00] += normal2;                                                
                        (*d.normals)[i10] += normal2;                        
                    }
                }
            }

            //Recalculate the south side
            if (s_neighbor.valid() && s_neighbor->getNumColumns() == d.originalNumCols && s_neighbor->getNumRows() == d.originalNumRows)            
            {                
                osg::ref_ptr< osg::Vec3Array > boundaryVerts = new osg::Vec3Array();
                boundaryVerts->reserve( 2 * d.numCols );

                std::vector< float > boundaryElevations;
                boundaryElevations.reserve( 2 * d.numCols );

                //Compute the verts for the south side               
                for (int j = (int)d.numRows-2; j <= (int)d.numRows-1; j++)
                {
                    for (int i = 0; i < (int)d.numCols; i++)                    
                    {                           
                        osg::Vec3d ndc( (double)(i)/(double)(d.numCols-1), (double)(j - static_cast<int>(d.numRows-1))/(double)(d.numRows-1), 0.0);                                                

                        unsigned i_equiv = d.i_sampleFactor==1.0 ? i : (unsigned) (double(i)*d.i_sampleFactor);
                        unsigned j_equiv = d.j_sampleFactor==1.0 ? j : (unsigned) (double(j)*d.j_sampleFactor);
                        
                        //TODO:  Should probably use an interpolated method here
                        float heightValue = s_neighbor->getHeight( i_equiv, j_equiv );                        
                        ndc.z() = heightValue;                        

                        osg::Vec3d model;
                        d.model->_tileLocator->unitToModel( ndc, model );
                        osg::Vec3d v = model - d.centerModel;
                        boundaryVerts->push_back( v );
                        boundaryElevations.push_back( heightValue );                        
                    }
                }   

                //The boundary verts are now populated, so go through and triangulate them add add the normals to the existing normal array                
                for (int i = 0; i < (int)d.numCols-1; i++)
                {                    
                    int i00;                    
                    int j = 0;


                    if (swapOrientation)
                    {                   
                        int i01 = j*d.numCols + i;
                        i00 = i01+d.numCols;                    
                    }
                    else
                    {
                        i00 = j*d.numCols + i;                        
                    }                    

                    int i10 = i00+1;

                    //remap indices to final vertex position
                    i00 = d.indices[i00];
                    i10 = d.indices[i10];

                    int baseIndex = i;
                    osg::Vec3f& v00 = (*boundaryVerts)[baseIndex    ];
                    osg::Vec3f& v10 = (*boundaryVerts)[baseIndex + 1];
                    osg::Vec3f& v01 = (*boundaryVerts)[baseIndex + d.numCols];
                    osg::Vec3f& v11 = (*boundaryVerts)[baseIndex + d.numCols + 1];

                    float e00 = boundaryElevations[baseIndex];
                    float e10 = boundaryElevations[baseIndex + 1];
                    float e01 = boundaryElevations[baseIndex + d.numCols];
                    float e11 = boundaryElevations[baseIndex + d.numCols + 1];

                   
                    if (!optimizeTriangleOrientation || (e00-e11)<fabsf(e01-e10))
                    {                            
                        osg::Vec3 normal1 = (v00-v01) ^ (v11-v01);                       
                        (*d.normals)[i00] += normal1;                                                

                        osg::Vec3 normal2 = (v10-v00) ^ (v11-v00);                        
                        (*d.normals)[i00] += normal2;                                                
                        (*d.normals)[i10] += normal2;                                                
                    }
                    else
                    {                            
                        osg::Vec3 normal1 = (v00-v01) ^ (v10-v01);
                        (*d.normals)[i00] += normal1;                                                
                        (*d.normals)[i10] += normal1;                                                

                        osg::Vec3 normal2 = (v10-v01) ^ (v11-v01);                        
                        (*d.normals)[i10] += normal2;                        
                    }
                }
            }            
        }

        if (recalcNormals)
        {
            for( osg::Vec3Array::iterator nitr = d.normals->begin(); nitr != d.normals->end(); ++nitr )
            {
                nitr->normalize();
            }       
        }
    }


    /**
     * Builds a state set for the tile's Geodes that encodes the color layers as textures
     * according to the setup of the compistor.
     */
    osg::StateSet* createStateSet( Data& d, TextureCompositor* compositor )
    {
        const GeoExtent& tileExtent = d.geoLocator->getDataExtent();

        osg::StateSet* stateSet = new osg::StateSet();

        // mark as DYNAMIC so we can access it from different threads.
        // TODO: need thing??
        // stateSet->setDataVariance( osg::Object::DYNAMIC );

        //TODO: implement this to support blending.
        osg::StateSet* parentStateSet = d.model->_parentStateSet.get();

        for( TileModel::ColorDataByUID::const_iterator i = d.model->_colorData.begin(); i != d.model->_colorData.end(); ++i )
        {
            const TileModel::ColorData& colorLayer = i->second;

            bool isRealData = !colorLayer.isFallbackData();

            osg::ref_ptr<const GeoLocator> layerGeoLocator = dynamic_cast<const GeoLocator*>( colorLayer.getLocator() );
            if ( layerGeoLocator.valid() )
            {
                if ( layerGeoLocator->getCoordinateSystemType() == osgTerrain::Locator::GEOCENTRIC )
                    layerGeoLocator = layerGeoLocator->getGeographicFromGeocentric();

                GeoImage sourceImage( colorLayer.getImage(), layerGeoLocator->getDataExtent() );

                GeoImage preparedImage = compositor->prepareImage( 
                    sourceImage, 
                    tileExtent );

                compositor->applyLayerUpdate( 
                    stateSet, 
                    colorLayer.getUID(), 
                    preparedImage, 
                    d.model->_tileKey,
                    isRealData ? parentStateSet : 0L );
            }
        }

        return stateSet;
    }



    struct CullByTraversalMask : public osg::Drawable::CullCallback
    {
        CullByTraversalMask( unsigned mask ) : _mask(mask) { }
        unsigned _mask;

        bool cull(osg::NodeVisitor* nv, osg::Drawable* drawable, osg::RenderInfo* renderInfo) const 
        {
            return ((unsigned)nv->getTraversalMask() & ((unsigned)nv->getNodeMaskOverride() | _mask)) == 0;
        }
    };
}

//------------------------------------------------------------------------

TileModelCompiler::TileModelCompiler(const MaskLayerVector&              masks,
                                     TextureCompositor*                  texCompositor,
                                     bool                                optimizeTriOrientation,
                                     const QuadTreeTerrainEngineOptions& options) :
_masks                 ( masks ),
_texCompositor         ( texCompositor ),
_optimizeTriOrientation( optimizeTriOrientation ),
_options               ( options )
{
    _cullByTraversalMask = new CullByTraversalMask(*options.secondaryTraversalMask());
}


bool
TileModelCompiler::compile(const TileModel* model,
                           osg::Node*&      out_node,
                           osg::StateSet*&  out_stateSet)
{
    // Working data for the build.
    Data d(model, _masks);

    d.scaleHeight = *_options.verticalScale();

    // build the transform matrix for this tile:
    model->_tileLocator->unitToModel( osg::Vec3(0.5f, 0.5f, 0.0), d.centerModel );
    osg::MatrixTransform* xform = new osg::MatrixTransform( osg::Matrix::translate(d.centerModel) );

    // A Geode/Geometry for the surface:
    d.surface = new osg::Geometry();
    d.surface->setUseVertexBufferObjects(d.useVBOs);
    d.surfaceGeode = new osg::Geode();
    d.surfaceGeode->addDrawable( d.surface );
    d.surfaceGeode->setNodeMask( *_options.primaryTraversalMask() );

    xform->addChild( d.surfaceGeode );

    // A Geode/Geometry for the skirt. This is good for traversal masking (e.g. shadows)
    // but bad since we're not combining the entire tile into a single geometry.
    // TODO: make this optional?
    d.createSkirt = (_options.heightFieldSkirtRatio().value() > 0.0);

    if ( d.createSkirt )
    {
        d.skirt = new osg::Geometry();
        d.skirt->setUseVertexBufferObjects(d.useVBOs);

        // slightly faster than a separate geode:
        d.skirt->setDataVariance( osg::Object::DYNAMIC ); // since we're using a custom cull callback
        d.skirt->setCullCallback( _cullByTraversalMask.get() );
        d.surfaceGeode->addDrawable( d.skirt );
    }


    // adjust the tile locator for geocentric mode:
    d.geoLocator = model->_tileLocator->getCoordinateSystemType() == GeoLocator::GEOCENTRIC ? 
        model->_tileLocator->getGeographicFromGeocentric() :
        model->_tileLocator.get();


    // Set up any geometry-cutting masks:
    if ( d.maskLayers.size() > 0 )
        setupMaskRecords( d );

    // allocate all the vertex, normal, and color arrays.
    double sampleRatio = *_options.heightFieldSampleRatio();
    if ( sampleRatio <= 0.0f )
        sampleRatio = osg::clampBetween( model->_tileKey.getLevelOfDetail()/20.0, 0.0625, 1.0 );

    setupGeometryAttributes( d, sampleRatio );

    // set up the list of layers to render and their shared arrays.
    setupTextureAttributes( d, _texCompositor.get(), _cache );

    // calculate the vertex and normals for the surface geometry.
    createSurfaceGeometry( d, _texCompositor.get() );

    // build geometry for the masked areas, if applicable
    if ( d.maskRecords.size() > 0 )
        createMaskGeometry( d, _texCompositor.get() );

    // build the skirts.
    if ( d.createSkirt )
        createSkirtGeometry( d, _texCompositor.get(), *_options.heightFieldSkirtRatio() );

    // tesselate the surface verts into triangles.
    tessellateSurfaceGeometry( d, _optimizeTriOrientation, *_options.normalizeEdges() );

    // assign our texture coordinate arrays to the geometry. This must happen LAST
    // since we're sharing arrays across tiles. Here is why:
    assignTextureArrays( d, _texCompositor.get() );

    // create the StateSet that will active texture composition.
    osg::StateSet* stateSet = createStateSet( d, _texCompositor.get() );

    if ( stateSet )
        xform->setStateSet( stateSet );


    // lastly, optimize the results.

    // unnecessary (I think) since tessellateSurfaceGeometry already makes an optimal
    // triangle set
    //if ( d.maskRecords.size() > 0 )
    //{
    //    MeshConsolidator::convertToTriangles( *d.surface );
    //}

    if ( d.skirt )
    {
        MeshConsolidator::convertToTriangles( *d.skirt );
    }

    for (MaskRecordVector::iterator mr = d.maskRecords.begin(); mr != d.maskRecords.end(); ++mr)
    {
        MeshConsolidator::convertToTriangles( *((*mr)._geom) );
    }
    
   
    if (osgDB::Registry::instance()->getBuildKdTreesHint()==osgDB::ReaderWriter::Options::BUILD_KDTREES &&
        osgDB::Registry::instance()->getKdTreeBuilder())
    {            
        osg::ref_ptr<osg::KdTreeBuilder> builder = osgDB::Registry::instance()->getKdTreeBuilder()->clone();
        xform->accept(*builder);
    }

    out_node     = xform;
    out_stateSet = stateSet;
    return true;
}
