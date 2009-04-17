#include <osgEarth/ElevationManager>
#include <osgEarth/HeightFieldUtils>
#include <osgEarth/Compositing>
#include <osgEarth/Mercator>

using namespace osg;
using namespace osgEarth;


ElevationManager::ElevationManager():
_samplePolicy(HIGHEST)
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

GeoHeightField* createGeoHeightField(const TileKey* key, TileSource* source, bool fallback)
{
    osg::ref_ptr<const TileKey> hf_key = key;
    while (hf_key.valid())
    {
        if (source->isKeyValid(hf_key.get()))
        {
            osg::HeightField* hf = source->createHeightField(hf_key.get());
            if (hf)
            {
                return new GeoHeightField(hf, hf_key->getGeoExtent());
            }
        }
        if (!fallback) break;
        hf_key = hf_key->createParentKey();
    }
    return NULL;
}

osg::HeightField*
ElevationManager::getHeightField(const osgEarth::TileKey *key, unsigned int cols /*=0*/, unsigned int rows/*=0*/, bool fallback /*=false*/)
{
    //Collect the heightfields
    std::vector<osg::ref_ptr<GeoHeightField>> heightFields;
    for (TileSourceList::iterator itr = _elevationSources.begin(); itr != _elevationSources.end(); ++itr)
    {
        //If the profile is exactly the same, just grab the the key
        if (itr->get()->getProfile()->isEquivalentTo(key->getProfile()))
        {   
            GeoHeightField *hf = createGeoHeightField(key, itr->get(), fallback);
            if (hf)
            {
                heightFields.push_back(hf);
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
                    GeoHeightField *hf = createGeoHeightField(intersectingTiles[i].get(), itr->get(), fallback);
                    if (hf)
                    {
                        heightFields.push_back(hf);
                    }
                }
            }
        }
    }

    //osg::notify(osg::NOTICE) << "getHeightField found " << heightFields.size() << " tiles" << std::endl;

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

            for (std::vector<osg::ref_ptr<GeoHeightField>>::iterator itr = heightFields.begin(); itr != heightFields.end(); ++itr)
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

                float result = 0.0f;

                std::vector<float> elevations;

                for (std::vector<osg::ref_ptr<GeoHeightField>>::iterator itr = heightFields.begin(); itr != heightFields.end(); ++itr)
                {
                    float elevation = 0.0f;
                    if (itr->get()->getElevation(key->getGeoExtent().getSRS(), geoX, geoY, BILINEAR, elevation))
                    {
                        elevations.push_back(elevation);
                        //osg::notify(osg::NOTICE) << "Got elevation " << elevation << std::endl;
                        //if (elevation > -100000) break;
                    }
                }

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
                if (result <= -result) result = 0.0f;
                hf->setHeight(c, r, result);
            }
        }
    }

    return hf;
}