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
#include "MaskGenerator"

#include <osgEarth/Locators>
#include <osgEarthSymbology/Geometry>

#include <osgUtil/DelaunayTriangulator>


using namespace osgEarth::Drivers::RexTerrainEngine;
using namespace osgEarth::Symbology;


#define LC "[MaskGenerator] "

#define MATCH_TOLERANCE 0.000001


MaskGenerator::MaskGenerator(const TileKey& key, unsigned tileSize, const MapFrame& mapFrame) :
_key( key ), _tileSize(tileSize)
{
    const osgEarth::MaskLayerVector maskLayers = mapFrame.terrainMaskLayers();
    for(MaskLayerVector::const_iterator it = maskLayers.begin();
        it != maskLayers.end(); 
        ++it)
    {
        MaskLayer* layer = it->get();
        if ( layer->getMinLevel() <= key.getLevelOfDetail() )
        {
            setupMaskRecord( mapFrame, layer->getOrCreateMaskBoundary( 1.0, key.getExtent().getSRS(), (ProgressCallback*)0L ) );
        }
    }
}

void
MaskGenerator::setupMaskRecord(const MapFrame& mapFrame, osg::Vec3dArray* boundary)
{
    osg::ref_ptr<osgEarth::GeoLocator> geoLocator = GeoLocator::createForKey(_key, mapFrame.getMapInfo());
    if (geoLocator->getCoordinateSystemType() == GeoLocator::GEOCENTRIC)
        geoLocator = geoLocator->getGeographicFromGeocentric();

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
        geoLocator->modelToUnit(min, min_ndc);
        geoLocator->modelToUnit(max, max_ndc);

        bool x_match = ((min_ndc.x() >= 0.0 && max_ndc.x() <= 1.0) ||
                        (min_ndc.x() <= 0.0 && max_ndc.x() > 0.0) ||
                        (min_ndc.x() < 1.0 && max_ndc.x() >= 1.0));

        bool y_match = ((min_ndc.y() >= 0.0 && max_ndc.y() <= 1.0) ||
                        (min_ndc.y() <= 0.0 && max_ndc.y() > 0.0) ||
                        (min_ndc.y() < 1.0 && max_ndc.y() >= 1.0));

        if (x_match && y_match)
        {
            if (_maskRecords.size() == 0)
            {
                _ndcMin = min_ndc;
                _ndcMax = max_ndc;
            }
            else
            {
                if (min_ndc.x() < _ndcMin.x()) _ndcMin.x() = min_ndc.x();
                if (min_ndc.y() < _ndcMin.y()) _ndcMin.y() = min_ndc.y();
                if (max_ndc.x() > _ndcMax.x()) _ndcMax.x() = max_ndc.x();
                if (max_ndc.y() > _ndcMax.y()) _ndcMax.y() = max_ndc.y();
            }

            _maskRecords.push_back( MaskRecord(boundary, min_ndc, max_ndc, 0L) );
        }
    }
}

osg::DrawElementsUInt*
MaskGenerator::createMaskPrimitives(const MapInfo& mapInfo, osg::Vec3Array* verts, osg::Vec3Array* texCoords, osg::Vec3Array* normals, osg::Vec3Array* neighbors)
{
    if (_maskRecords.size() <= 0)
      return 0L;

    osg::ref_ptr<osgEarth::GeoLocator> geoLocator = GeoLocator::createForKey(_key, mapInfo);
    if (geoLocator->getCoordinateSystemType() == GeoLocator::GEOCENTRIC)
        geoLocator = geoLocator->getGeographicFromGeocentric();

    GeoPoint centroid;
    _key.getExtent().getCentroid( centroid );

    osg::Matrix world2local, local2world;
    centroid.createWorldToLocal( world2local );
    local2world.invert( world2local );

    osg::ref_ptr<osgUtil::DelaunayTriangulator> trig=new osgUtil::DelaunayTriangulator();

    std::vector<osg::ref_ptr<osgUtil::DelaunayConstraint> > alldcs;

    osg::ref_ptr<osg::Vec3Array> coordsArray = new osg::Vec3Array;

    double minndcx = _maskRecords[0]._ndcMin.x();
    double minndcy = _maskRecords[0]._ndcMin.y();
    double maxndcx = _maskRecords[0]._ndcMax.x();
    double maxndcy = _maskRecords[0]._ndcMax.y();
    for (int mrs = 1; mrs < _maskRecords.size(); ++mrs)
    {
        if ( _maskRecords[mrs]._ndcMin.x()< minndcx)
        {
            minndcx = _maskRecords[mrs]._ndcMin.x();
        }
        if ( _maskRecords[mrs]._ndcMin.y()< minndcy)
        {
            minndcy = _maskRecords[mrs]._ndcMin.y();
        }
        if ( _maskRecords[mrs]._ndcMax.x()> maxndcx)
        {
            maxndcx = _maskRecords[mrs]._ndcMax.x();
        }
        if ( _maskRecords[mrs]._ndcMax.y()> maxndcy)
        {
            maxndcy = _maskRecords[mrs]._ndcMax.y();
        }			
    }

    int min_i = (int)floor(minndcx * (double)(_tileSize-1));
    if (min_i < 0) min_i = 0;
    if (min_i >= (int)_tileSize) min_i = _tileSize - 1;

    int min_j = (int)floor(minndcy * (double)(_tileSize-1));
    if (min_j < 0) min_j = 0;
    if (min_j >= (int)_tileSize) min_j = _tileSize - 1;

    int max_i = (int)ceil(maxndcx * (double)(_tileSize-1));
    if (max_i < 0) max_i = 0;
    if (max_i >= (int)_tileSize) max_i = _tileSize - 1;

    int max_j = (int)ceil(maxndcy * (double)(_tileSize-1));
    if (max_j < 0) max_j = 0;
    if (max_j >= (int)_tileSize) max_j = _tileSize - 1;

    if (min_i >= 0 && max_i >= 0 && min_j >= 0 && max_j >= 0)
    {
        int num_i = max_i - min_i + 1;
        int num_j = max_j - min_j + 1;

        osg::ref_ptr<Polygon> maskSkirtPoly = new Polygon();
        maskSkirtPoly->resize(num_i * 2 + num_j * 2 - 4);

        for (int i = 0; i < num_i; i++)
        {
            {
                osg::Vec3d ndc( ((double)(i + min_i))/(double)(_tileSize-1), ((double)min_j)/(double)(_tileSize-1), 0.0);
                (*maskSkirtPoly)[i] = ndc;
            }

            {
                osg::Vec3d ndc( ((double)(i + min_i))/(double)(_tileSize-1), ((double)max_j)/(double)(_tileSize-1), 0.0);
                (*maskSkirtPoly)[i + (2 * num_i + num_j - 3) - 2 * i] = ndc;
            }
        }
        for (int j = 0; j < num_j - 2; j++)
        {
            {
                osg::Vec3d ndc( ((double)max_i)/(double)(_tileSize-1), ((double)(min_j + j + 1))/(double)(_tileSize-1), 0.0);
                (*maskSkirtPoly)[j + num_i] = ndc;
            }

            {
                osg::Vec3d ndc( ((double)min_i)/(double)(_tileSize-1), ((double)(min_j + j + 1))/(double)(_tileSize-1), 0.0);
                (*maskSkirtPoly)[j + (2 * num_i + 2 * num_j - 5) - 2 * j] = ndc;
            }
        }

        for (int j = 0; j < num_j; j++)
        {
            for (int i = 0; i < num_i; i++)
            {
                {
                    osg::Vec3d ndc( ((double)(i + min_i))/(double)(_tileSize-1), ((double)(j+min_j))/(double)(_tileSize-1), 0.0);
                    coordsArray->push_back(ndc) ;
                }						
            }
        }


        //
        osg::ref_ptr<osg::Vec3dArray> boundaryVerts = new osg::Vec3dArray;

        // Use delaunay triangulation for stitching:
        for (MaskRecordVector::iterator mr = _maskRecords.begin();mr != _maskRecords.end();mr++)
        {
            //Create local polygon representing mask
            osg::ref_ptr<Polygon> maskPoly = new Polygon();
            for (osg::Vec3dArray::iterator it = (*mr)._boundary->begin(); it != (*mr)._boundary->end(); ++it)
            {
                osg::Vec3d local;
                geoLocator->convertModelToLocal(*it, local);
                maskPoly->push_back(local);
            }

            // Add mask bounds as a triangulation constraint
            osg::ref_ptr<osgUtil::DelaunayConstraint> newdc=new osgUtil::DelaunayConstraint;
            osg::Vec3Array* maskConstraint = new osg::Vec3Array();
            newdc->setVertexArray(maskConstraint);

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
                    osg::ref_ptr<osg::Vec3Array> partVerts = part->createVec3Array();
                    maskConstraint->insert(maskConstraint->end(), partVerts->begin(), partVerts->end());
                    newdc->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_LOOP, maskConstraint->size() - partVerts->size(), partVerts->size()));
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
                        //(*it).z() = (*mit).z();
                        zSet += 1;

                        // Remove duplicate point from coordsArray to avoid duplicate point warnings
                        osg::Vec3Array::iterator caIt;
                        for (caIt = coordsArray->begin(); caIt != coordsArray->end(); ++caIt)
                        {
                            if (osg::absolute((*caIt).x() - (*it).x()) < MATCH_TOLERANCE && osg::absolute((*caIt).y() - (*it).y()) < MATCH_TOLERANCE)
                                break;
                        }
                        if (caIt != coordsArray->end())
                            coordsArray->erase(caIt);

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

                        boundaryVerts->push_back((*it));
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

                                boundaryVerts->push_back((*it));
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

                        boundaryVerts->push_back((*it));
                    }
                }

                if (!isZSet[count])
                    OE_WARN << LC << "Z-value not set for mask constraint vertex" << std::endl;

                count++;
            }

            alldcs.push_back(newdc);
        }

        trig->setInputPointArray(coordsArray.get());

        for (int dcnum =0; dcnum < alldcs.size();dcnum++)
        {
            trig->addInputConstraint(alldcs[dcnum].get());
        }

        // Create array to hold vertex normals
        osg::Vec3Array *norms=new osg::Vec3Array;
        trig->setOutputNormalArray(norms);


        // Triangulate vertices and remove triangles that lie within the contraint loop
        trig->triangulate();
        for (int dcnum =0; dcnum < alldcs.size();dcnum++)
        {
            trig->removeInternalTriangles(alldcs[dcnum].get());
        }

        verts->reserve(verts->size() + trig->getInputPointArray()->size());
        texCoords->reserve(texCoords->size() + trig->getInputPointArray()->size());
        normals->reserve(normals->size() + trig->getInputPointArray()->size());
        if ( neighbors )
            neighbors->reserve(neighbors->size() + trig->getInputPointArray()->size()); 

        // Iterate through point to convert to model coords, calculate normals, and set up tex coords
        osg::ref_ptr<GeoLocator> locator = GeoLocator::createForKey( _key, mapInfo );

        //int norm_i = -1;
        unsigned vertsOffset = verts->size();

        for (osg::Vec3Array::iterator it = trig->getInputPointArray()->begin(); it != trig->getInputPointArray()->end(); ++it)
        {
            // check to see if point is a part of the original mask boundary
            bool isBoundary = false;
            for (osg::Vec3dArray::iterator bit = boundaryVerts->begin(); bit != boundaryVerts->end(); ++bit)
            {
                if (osg::absolute((*bit).x() - (*it).x()) < MATCH_TOLERANCE && osg::absolute((*bit).y() - (*it).y()) < MATCH_TOLERANCE)
                {
                    isBoundary = true;
                    break;
                }
            }

            // get model coords
            osg::Vec3d model;
            locator->unitToModel(osg::Vec3d(it->x(), it->y(), 0.0f), model);
            model = model * world2local;

            // calc normals
            osg::Vec3d modelPlusOne;
            locator->unitToModel(osg::Vec3d(it->x(), it->y(), 1.0f), modelPlusOne);
            osg::Vec3d normal = (modelPlusOne*world2local)-model;                
            normal.normalize();
            normals->push_back( normal );

            // set elevation if this is a point along the mask boundary
            if (isBoundary)
                model += normal*it->z();

            verts->push_back(model);

            // use same vert for neighbor to prevent morphing
            if ( neighbors )
                neighbors->push_back( model );  

            // set up text coords
            texCoords->push_back( osg::Vec3f(it->x(), it->y(), isBoundary ? MASK_MARKER_BOUNDARY : MASK_MARKER_SKIRT) );
        }

        // Get triangles from triangulator and add as primative set to the geometry
        osg::DrawElementsUInt* tris = trig->getTriangles();
        if ( tris && tris->getNumIndices() >= 3 )
        {
            osg::ref_ptr<osg::DrawElementsUInt> elems = new osg::DrawElementsUInt(tris->getMode());
            elems->reserve(tris->size());

            const osg::MixinVector<GLuint> ins = tris->asVector();
            for (osg::MixinVector<GLuint>::const_iterator it = ins.begin(); it != ins.end(); ++it)
            {
                elems->push_back((*it) + vertsOffset);
            }

            return elems.release();
        }
    }

    return 0L;
}

void
MaskGenerator::getMinMax(osg::Vec3d& min, osg::Vec3d& max)
{
    if (_maskRecords.size() > 0)
    {
        min.x() = _maskRecords[0]._ndcMin.x();
        min.y() = _maskRecords[0]._ndcMin.y();
        min.z() = _maskRecords[0]._ndcMin.z();

        max.x() = _maskRecords[0]._ndcMax.x();
        max.y() = _maskRecords[0]._ndcMax.y();
        max.z() = _maskRecords[0]._ndcMax.z();

        for (MaskRecordVector::const_iterator it = _maskRecords.begin(); it != _maskRecords.end(); ++it)
        {
            if (it->_ndcMin.x() < min.x()) min.x() = it->_ndcMin.x();
            if (it->_ndcMin.y() < min.y()) min.y() = it->_ndcMin.y();
            if (it->_ndcMin.z() < min.z()) min.z() = it->_ndcMin.z();

            if (it->_ndcMax.x() > max.x()) max.x() = it->_ndcMax.x();
            if (it->_ndcMax.y() > max.y()) max.y() = it->_ndcMax.y();
            if (it->_ndcMax.z() > max.z()) max.z() = it->_ndcMax.z();
        }
    }
}

float
MaskGenerator::getMarker(float nx, float ny) const
{
    float marker = 1.0f; // 1.0 == does not contain

    if (_maskRecords.size() > 0)
    {
        int min_i = (int)floor(_ndcMin.x() * (double)(_tileSize-1));
        int min_j = (int)floor(_ndcMin.y() * (double)(_tileSize-1));
        int max_i = (int)ceil(_ndcMax.x() * (double)(_tileSize-1));
        int max_j = (int)ceil(_ndcMax.y() * (double)(_tileSize-1));

        int i = nx * (double)(_tileSize-1);
        int j = ny * (double)(_tileSize-1);

        if (i > min_i && i < max_i && j > min_j && j < max_j)
        {
            marker = MASK_MARKER_DISCARD; // contained by mask
        }
        else if ((i == min_i && j >= min_j && j <= max_j) ||
                 (i == max_i && j >= min_j && j <= max_j) ||
                 (j == min_j && i >= min_i && i <= max_i) ||
                 (j == max_j && i >= min_i && i <= max_i))
        {
            marker = MASK_MARKER_SKIRT; // tile vert on outer mask skirt boundary
        }
    }

    return marker;
}