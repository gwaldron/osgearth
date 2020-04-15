/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
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

#include <osgEarth/Filter>
#include <osgEarth/FeatureCursor>

#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgEarth/Registry>
#include <osgEarth/ImageUtils>
#include <osgEarth/FeatureSource>
#include <osgEarth/FilterContext>
#include <osgEarth/Geometry>

#define LC "[Intersect FeatureFilter] "

using namespace osgEarth;
using namespace osgEarth::Drivers;


/**
 * Spatial Join: This feature finds all the features in a secondary feature source
 * that intersect the input feature's extent, and adds their attributes to the
 * input feature.
 */
class JoinFeatureFilter : public FeatureFilter, public JoinFeatureFilterOptions
{
public:
    JoinFeatureFilter(const ConfigOptions& options)
        : FeatureFilter(), JoinFeatureFilterOptions(options)
    {
    }

public: // FeatureFilter

    Status initialize(const osgDB::Options* readOptions)
    {
        Status fsStatus = featureSource().open(readOptions);
        if (fsStatus.isError())
            return fsStatus;

        return Status::OK();
    }

    /**
     * Gets all the features that intersect the extent
     */
    void getFeatures(const GeoExtent& extent, FeatureList& features, ProgressCallback* progress)
    {
        FeatureSource* fs = featureSource().getLayer();
        if (!fs)
            return;

        //TODO: should this be Profile::transformAndClampExtent instead?
        GeoExtent localExtent = extent.transform( fs->getFeatureProfile()->getSRS() );
        Query query;
        query.bounds() = localExtent.bounds();
        if (localExtent.intersects( fs->getFeatureProfile()->getExtent()))
        {
            osg::ref_ptr< FeatureCursor > cursor = fs->createFeatureCursor( query, progress );
            if (cursor)
            {
                cursor->fill( features );
            }
        }     
    }

    FilterContext push(FeatureList& input, FilterContext& context)
    {
        if (featureSource().getLayer())
        {
            // Get any features that intersect this query.
            FeatureList boundaries;
            getFeatures(context.extent().get(), boundaries, 0L); // TODO: progress...

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
                       
                        if (featureSource().getLayer()->getFeatureProfile()->getExtent().contains(GeoPoint(feature->getSRS(), c.x(), c.y())))
                        {
                            for (FeatureList::iterator itr = boundaries.begin(); itr != boundaries.end(); ++itr)
                            {
                                //if (ring && ring->contains2D(c.x(), c.y()))
                                if (itr->get()->getGeometry()->intersects( feature->getGeometry() ) )
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
    
    const char* className() const
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


