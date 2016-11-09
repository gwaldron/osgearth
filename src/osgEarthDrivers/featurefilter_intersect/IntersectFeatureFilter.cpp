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
#include "IntersectFeatureFilterOptions"

#include <osgEarthFeatures/Filter>

#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgEarth/Registry>
#include <osgEarth/ImageUtils>
#include <osgEarthFeatures/FeatureSource>
#include <osgEarthSymbology/Geometry>

#define LC "[Intersect FeatureFilter] "

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Drivers;
using namespace osgEarth::Symbology;



class IntersectFeatureFilter : public FeatureFilter, public IntersectFeatureFilterOptions
{
private:
    osg::ref_ptr< FeatureSource > _featureSource;

public:
    IntersectFeatureFilter(const ConfigOptions& options)
        : FeatureFilter(), IntersectFeatureFilterOptions(options)
    {
    }

public: // FeatureFilter

    Status initialize(const osgDB::Options* readOptions)
    {
        // Load the feature source containing the intersection geometry.
        _featureSource = FeatureSourceFactory::create( features().get() );
        if ( !_featureSource.valid() )
            return Status::Error(Status::ServiceUnavailable, Stringify()<< "Failed to create feature driver \"" << features()->getDriver() << "\"");

        const Status& s = _featureSource->open(readOptions);
        if (s.isError())
            return s;

        return Status::OK();
    }

    /**
     * Gets all the features that intersect the extent
     */
    void getFeatures(const GeoExtent& extent, FeatureList& features)
    {
        GeoExtent localExtent = extent.transform( _featureSource->getFeatureProfile()->getSRS() );
        Query query;
        query.bounds() = localExtent.bounds();
        if (localExtent.intersects( _featureSource->getFeatureProfile()->getExtent()))
        {
            osg::ref_ptr< FeatureCursor > cursor = _featureSource->createFeatureCursor( query );
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
            // Get any features that intersect this query.
            FeatureList boundaries;
            getFeatures(context.extent().get(), boundaries );
            
            
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
                        osg::Vec2d c = feature->getGeometry()->getBounds().center2d();

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

            OE_INFO << LC << "Allowed " << output.size() << " out of " << input.size() << " features\n";

            input = output;
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

