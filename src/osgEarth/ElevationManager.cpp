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

#include <osgEarth/ElevationManager>
#include <osgEarth/HeightFieldUtils>
#include <osgEarth/Compositing>
#include <osgEarth/Mercator>

using namespace osg;
using namespace osgEarth;


ElevationManager::ElevationManager():
_samplePolicy(FIRST_VALID)
{
}

TileSourceList&
ElevationManager::getElevationSources()
{
    return _elevationSources;
}

void
ElevationManager::setSamplePolicy(SamplePolicy samplePolicy)
{
    _samplePolicy = samplePolicy;
}

ElevationManager::SamplePolicy
ElevationManager::getSamplePolicy()
{
    return _samplePolicy;
}

static GeoHeightField*
createGeoHeightField(const TileKey* key, TileSource* source, bool fallback)
{
    osg::ref_ptr<const TileKey> hf_key = key;
    while (hf_key.valid())
    {
        if (source->isKeyValid(hf_key.get()))
        {
            osg::HeightField* hf = source->createHeightField(hf_key.get());
            if (hf)
            {
                //Modify the heightfield data so that is contains a standard value for NO_DATA
                osg::ref_ptr<CompositeValidValueOperator> ops = new CompositeValidValueOperator;
                ops->getOperators().push_back(new osgTerrain::NoDataValue(source->getNoDataValue()));
                ops->getOperators().push_back(new osgTerrain::ValidRange(source->getNoDataMinValue(), source->getNoDataMaxValue()));
                
                ReplaceInvalidDataOperator o;
                o.setReplaceWith(NO_DATA_VALUE);
                o.setValidDataOperator(ops.get());
                o(hf);

                return new GeoHeightField(hf, hf_key->getGeoExtent());
            }
        }
        if (!fallback) break;
        hf_key = hf_key->createParentKey();
    }
    return NULL;
}

osg::HeightField*
ElevationManager::createHeightField(const osgEarth::TileKey *key, unsigned int cols /*=0*/, unsigned int rows/*=0*/, bool fallback /*=false*/)
{
    //Collect the heightfields
    typedef std::vector< osg::ref_ptr<GeoHeightField> > HeightFields;
    HeightFields heightFields;

    std::vector<bool> foundElevation;
    foundElevation.reserve(_elevationSources.size());

    int sourceIndex = 0;
    for (TileSourceList::iterator itr = _elevationSources.begin(); itr != _elevationSources.end(); ++itr)
    {
        foundElevation.push_back(false);

        //If the profile is exactly the same, just grab the the key
        if (itr->get()->getProfile()->isEquivalentTo(key->getProfile()))
        {   
            GeoHeightField *hf = createGeoHeightField(key, itr->get(), false);
            if (hf)
            {
                heightFields.push_back(hf);
                foundElevation[sourceIndex] = true;
            }
        }
        else
        {
            //Determine the intersecting keys
            std::vector< osg::ref_ptr<const TileKey> > intersectingTiles;
            itr->get()->getProfile()->getIntersectingTiles(key, intersectingTiles);
            if (intersectingTiles.size() > 0)
            {
                for (unsigned int i = 0; i < intersectingTiles.size(); ++i)
                {
                    GeoHeightField *hf = createGeoHeightField(intersectingTiles[i].get(), itr->get(), false);
                    if (hf)
                    {
                        heightFields.push_back(hf);
                        foundElevation[sourceIndex] = true;
                    }
                }
            }
        }
        sourceIndex++;
    }

    //Count the number of valid sources
    unsigned int numValidSources = 0;
    for (unsigned int i = 0; i < foundElevation.size(); ++i)
    {
        if (foundElevation[i]) numValidSources ++;
    }

    osg::notify(osg::INFO) << "[osgEarth::ElevationManager] found " << numValidSources << " out of " << _elevationSources.size() << std::endl;

    //If we didn't find any heightfields and we weren't explicity asked to fall back, just return NULL to signal that subdivision should stop
    if (numValidSources == 0 && !fallback) return NULL;

    //We have either been asked to fallback to previous levels
    for (unsigned int i = 0; i < foundElevation.size(); ++i)
    {
        if (!foundElevation[i])
        {
            TileSource* source = _elevationSources[i].get();

            //If the profile is exactly the same, just grab the the key
            if (source->getProfile()->isEquivalentTo(key->getProfile()))
            {   
                GeoHeightField *hf = createGeoHeightField(key, source, true);
                if (hf)
                {
                    heightFields.push_back(hf);
                    osg::notify(osg::INFO) << "[osgEarth::ElevationManager] had to fall back on elevation source " << i << " for " << key->str() <<  std::endl;
                }
            }
            else
            {
                //Determine the intersecting keys
                std::vector< osg::ref_ptr<const TileKey> > intersectingTiles;
                source->getProfile()->getIntersectingTiles(key, intersectingTiles);
                if (intersectingTiles.size() > 0)
                {
                    for (unsigned int i = 0; i < intersectingTiles.size(); ++i)
                    {
                        GeoHeightField *hf = createGeoHeightField(intersectingTiles[i].get(), source, true);
                        if (hf)
                        {
                            heightFields.push_back(hf);
                            osg::notify(osg::INFO) << "[osgEarth::ElevationManager] had to fall back on elevation source " << i << " for " << key->str() <<  std::endl;
                        }
                    }
                }
            }
        }
    }
   

    osg::notify(osg::INFO) << "[osgEarth::ElevationManager] getHeightField found " << heightFields.size() << " tiles" << std::endl;

    osg::HeightField *hf = NULL;

    //If we dind't find any heightfields, return NULL
    if (!heightFields.empty())
    {
        //If no explicit size specified, create a heightfield with the highest resolution of the incoming heightfields
        unsigned int width = cols;
        unsigned int height = rows;

        if (width == 0 || height == 0)
        {
            width = 0;
            height = 0;

            for (HeightFields::iterator itr = heightFields.begin(); itr != heightFields.end(); ++itr)
            {
                if (itr->get()->getHeightField()->getNumColumns() > width) width = itr->get()->getHeightField()->getNumColumns();
                if (itr->get()->getHeightField()->getNumRows() > height) height = itr->get()->getHeightField()->getNumRows();
            }
        }

        hf = new osg::HeightField;
        hf->allocate(width, height);

        //Go ahead and set up the heightfield so we don't have to worry about it later
        double minx, miny, maxx, maxy;
        key->getGeoExtent().getBounds(minx, miny, maxx, maxy);
        hf->setOrigin( osg::Vec3d( minx, miny, 0.0 ) );
        double dx = (maxx - minx)/(double)(hf->getNumColumns()-1);
        double dy = (maxy - miny)/(double)(hf->getNumRows()-1);
        hf->setXInterval( dx );
        hf->setYInterval( dy );
        hf->setBorderWidth( 0 );

        //Create the new heightfield by sampling all of them.
        for (unsigned int c = 0; c < width; ++c)
        {
            double geoX = minx + (dx * (double)c);
            for (unsigned r = 0; r < height; ++r)
            {
                double geoY = miny + (dy * (double)r);

                bool hasValidData = false;

                //Collect elevations from all of the sources
                std::vector<float> elevations;
                for (HeightFields::iterator itr = heightFields.begin(); itr != heightFields.end(); ++itr)
                {
                    float elevation = 0.0f;
                    if (itr->get()->getElevation(key->getGeoExtent().getSRS(), geoX, geoY, BILINEAR, elevation))
                    {
                        if (elevation != NO_DATA_VALUE)
                        {
                            elevations.push_back(elevation);
                        }
                    }
                }

                float result = NO_DATA_VALUE;

                //The list of elevations only contains valid values
                if (elevations.size() > 0)
                {
                    if (_samplePolicy == FIRST_VALID)
                    {
                        result = elevations[0];
                    }
                    else if (_samplePolicy == HIGHEST)
                    {
                        result = -FLT_MAX;
                        for (unsigned int i = 0; i < elevations.size(); ++i)
                        {
                            if (result < elevations[i]) result = elevations[i];
                        }
                    }
                    else if (_samplePolicy == LOWEST)
                    {
                        result = FLT_MAX;
                        for (unsigned i = 0; i < elevations.size(); ++i)
                        {
                            if (result > elevations[i]) result = elevations[i];
                        }
                    }
                    else if (_samplePolicy = AVERAGE)
                    {
                        result = 0.0;
                        for (unsigned i = 0; i < elevations.size(); ++i)
                        {
                            result += elevations[i];
                        }
                        result /= (float)elevations.size();
                    }
                }
                hf->setHeight(c, r, result);
            }
        }
    }

    if (hf)
    {
        //Attempt to fill NoData holes if they exists
        //FillNoDataOperator  fndo;
        //fndo.setValidDataOperator(new osgTerrain::NoDataValue(NO_DATA_VALUE));
        //fndo(hf);
        ReplaceInvalidDataOperator o;
        o.setValidDataOperator(new osgTerrain::NoDataValue(NO_DATA_VALUE));
        o(hf);
    }


    return hf;
}
