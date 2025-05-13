/* osgEarth
 * Copyright 2008-2014 Pelican Mapping
 * MIT License
 */
#include "IntersectFeatureFilterOptions"

#include <osgEarth/Registry>
#include <osgEarth/ImageUtils>
#include <osgEarth/Progress>

#include <osgEarth/Filter>
#include <osgEarth/FeatureCursor>
#include <osgEarth/FeatureSource>
#include <osgEarth/FilterContext>

#include <osgEarth/Geometry>

#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>

#define LC "[Intersect FeatureFilter] "

using namespace osgEarth;
using namespace osgEarth::Drivers;



class IntersectFeatureFilter : public FeatureFilter, public IntersectFeatureFilterOptions
{
private:
    osg::ref_ptr< FeatureSource > _featureSource;
    osg::ref_ptr< const osgDB::Options > _readOptions;

public:
    IntersectFeatureFilter(const ConfigOptions& options)
        : FeatureFilter(), IntersectFeatureFilterOptions(options)
    {
    }

public: // FeatureFilter

    Status initialize(const osgDB::Options* readOptions)
    {
        _readOptions = readOptions;
        return Status::OK();
    }

    void addedToMap(const class Map* map) override
    {
        if (!_featureSource.valid())
        {
            auto status = featureSource().open(_readOptions.get());
            if (status.isOK())
            {
                featureSource().addedToMap(map);
                _featureSource = featureSource().getLayer();
            }
            else
            {
                OE_WARN << LC << status.message() << std::endl;
            }
        }
    }

    /**
     * Gets all the features that intersect the extent
     */
    void getFeatures(const GeoExtent& extent, FeatureList& features, ProgressCallback* progress)
    {
        GeoExtent localExtent = extent.transform( _featureSource->getFeatureProfile()->getSRS() );
        Query query;
        query.bounds() = localExtent.bounds();
        if (localExtent.intersects( _featureSource->getFeatureProfile()->getExtent()))
        {
            osg::ref_ptr< FeatureCursor > cursor = _featureSource->createFeatureCursor(query, {}, nullptr, progress);
            if (cursor)
            {
                cursor->fill( features );
            }
        }     
    }

    FilterContext push(FeatureList& input, FilterContext& context)
    {       
        if (_featureSource.valid())
        {
            osg::ref_ptr<ProgressCallback> progress = new ProgressCallback();

            // Get any features that intersect this query.
            FeatureList boundaries;
            getFeatures(context.extent().get(), boundaries, progress.get());
                        
            // The list of output features
            FeatureList output;

            if (boundaries.empty())
            {
                // No intersecting features.  If contains is false, then just the output to the input.
                if (contains() == false)
                {
                    output = input;
                }
            }
            else
            {
                // Transform the boundaries into the coordinate system of the features
                for (FeatureList::iterator itr = boundaries.begin(); itr != boundaries.end(); ++itr)
                {
                    itr->get()->transform( context.profile()->getSRS() );
                }

                for(FeatureList::const_iterator f = input.begin(); f != input.end(); ++f)
                {
                    Feature* feature = f->get();
                    if ( feature && feature->getGeometry() )
                    {
                        osg::Vec3d c = feature->getGeometry()->getBounds().center();

                        if ( contains() == true )
                        {
                            // coarsest:
                            if (_featureSource->getFeatureProfile()->getExtent().contains(GeoPoint(feature->getSRS(), c.x(), c.y())))
                            {
                                for (FeatureList::iterator itr = boundaries.begin(); itr != boundaries.end(); ++itr)
                                {
                                    Ring* ring = dynamic_cast< Ring*>(itr->get()->getGeometry());
                                    if (ring && ring->contains2D(c.x(), c.y()))
                                    {
                                        output.push_back( feature );
                                    }
                                }                        
                            }
                        }

                        else
                        {    
                            bool contained = false;

                            // coarsest:
                            if (_featureSource->getFeatureProfile()->getExtent().contains(GeoPoint(feature->getSRS(), c.x(), c.y())))
                            {
                                for (FeatureList::iterator itr = boundaries.begin(); itr != boundaries.end(); ++itr)
                                {
                                    Ring* ring = dynamic_cast< Ring*>(itr->get()->getGeometry());
                                    if (ring && ring->contains2D(c.x(), c.y()))
                                    {                             
                                        contained = true;
                                        break;
                                    }
                                }
                            }
                            if ( !contained )
                            {
                                output.push_back( feature );
                            }
                        }
                    }
                }
            }

            //OE_INFO << LC << "isect: " << input.size() << " in, " << output.size() << " out" << std::endl;

            input.swap(output);
        }

        return context;
    }
};


class IntersectFeatureFilterPlugin : public FeatureFilterDriver
{
public:
    IntersectFeatureFilterPlugin() : FeatureFilterDriver()
    {
        this->supportsExtension("osgearth_featurefilter_intersect", className() );
    }
    
    const char* className() const
    {
        return "IntersectFeatureFilterPlugin";
    }

    ReadResult readObject(const std::string& file_name, const Options* options) const
    {
        if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
            return ReadResult::FILE_NOT_HANDLED;

        return new IntersectFeatureFilter( getConfigOptions(options) );
    }
};

REGISTER_OSGPLUGIN(osgearth_featurefilter_intersect, IntersectFeatureFilterPlugin);

//OSGEARTH_REGISTER_SIMPLE_FEATUREFILTER(intersect, IntersectFeatureFilter);

