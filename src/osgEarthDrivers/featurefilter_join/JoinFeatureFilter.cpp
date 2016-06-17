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
#include "JoinFeatureFilterOptions"

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



class JoinFeatureFilter : public FeatureFilter, public JoinFeatureFilterOptions
{
private:
    osg::ref_ptr< FeatureSource > _featureSource;

public:
    JoinFeatureFilter(const ConfigOptions& options)
        : FeatureFilter(), JoinFeatureFilterOptions(options)
    {
    }

public: // FeatureFilter

    void initialize(const osgDB::Options* dbo)
    {
        // Load the feature source containing the intersection geometry.
        _featureSource = FeatureSourceFactory::create( features().get() );
        if ( !_featureSource.valid() )
        {
            OE_WARN << LC << "Failed to load the intersect feature source.\n";
            return;
        }

        _featureSource->initialize( dbo );

        if ( !_featureSource->getFeatureProfile() )
        {
            OE_WARN << LC << "Failed to establish the feature profile.\n";
            return;
        }
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

            if (!boundaries.empty())
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
                       
                        if (_featureSource->getFeatureProfile()->getExtent().contains(GeoPoint(feature->getSRS(), c.x(), c.y())))
                        {
                            for (FeatureList::iterator itr = boundaries.begin(); itr != boundaries.end(); ++itr)
                            {
                                // TODO:  Could use intersects here instead of contains2D to be more accurate.
                                Ring* ring = dynamic_cast< Ring*>(itr->get()->getGeometry());
                                if (ring && ring->contains2D(c.x(), c.y()))
                                {
                                    // Copy the attributes in the boundary to the feature
                                    for (AttributeTable::const_iterator attrItr = itr->get()->getAttrs().begin();
                                         attrItr != itr->get()->getAttrs().end();
                                         attrItr++)
                                    {
                                        feature->set( attrItr->first, attrItr->second );
                                    }
                                    break;
                                }
                            }                        
                        }
                    }
                }
            }
        }

        return context;
    }
};


class JoinFeatureFilterPlugin : public FeatureFilterDriver
{
public:
    JoinFeatureFilterPlugin() : FeatureFilterDriver()
    {
        this->supportsExtension("osgearth_featurefilter_join", className() );
    }
    
    const char* className()
    {
        return "JoinFeatureFilterPlugin";
    }

    ReadResult readObject(const std::string& file_name, const Options* options) const
    {
        if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
            return ReadResult::FILE_NOT_HANDLED;

        return new JoinFeatureFilter( getConfigOptions(options) );
    }
};

REGISTER_OSGPLUGIN(osgearth_featurefilter_join, JoinFeatureFilterPlugin);

//OSGEARTH_REGISTER_SIMPLE_FEATUREFILTER(intersect, IntersectFeatureFilter);

