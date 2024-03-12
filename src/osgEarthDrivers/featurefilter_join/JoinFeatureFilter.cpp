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
#include <osgEarth/Metrics>

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

        OE_PROFILING_ZONE;

        //TODO: should this be Profile::transformAndClampExtent instead?
        GeoExtent extentInFeatureSRS = extent.transform( fs->getFeatureProfile()->getSRS() );

        if (extentInFeatureSRS.intersects( fs->getFeatureProfile()->getExtent()))
        {
            Query query;
            query.bounds() = extentInFeatureSRS.bounds();

            osg::ref_ptr<FeatureCursor> cursor = fs->createFeatureCursor(query, {}, nullptr, progress);
            if (cursor)
            {
                cursor->fill( features );
            }
        }     
    }

    //! Joins the boundaries attributes into the features by doing a
    //! spatial intersection.
    void combine(FeatureList& boundaries, FeatureList& input, FilterContext& context) const
    {
        OE_PROFILING_ZONE;

        // Transform the boundaries into the coordinate system of the features
        // for fast intersection testing
        for (auto& boundary : boundaries)
        {
            boundary->transform(context.profile()->getSRS());
        }

        // For each feature, check for a spatial join:
        for (auto& feature : input)
        {
            if (feature.valid() && feature->getGeometry())
            {
                for (const auto& boundary : boundaries)
                {
                    if (boundary->getGeometry()->intersects(feature->getGeometry()))
                    {
                        // Copy the attributes from the boundary to the feature (and overwrite)
                        for (const auto& attr : boundary->getAttrs())
                        {
                            feature->set(attr.first, attr.second);
                        }

                        // upon success, don't check any more boundaries:
                        break;
                    }
                }
            }
        }
    }


    FilterContext push(FeatureList& input, FilterContext& context)
    {
        OE_PROFILING_ZONE;

        if (featureSource().getLayer())
        {
            // Get any features that intersect this query.
            FeatureList boundaries;
            getFeatures(context.extent().get(), boundaries, nullptr); // TODO: progress...

            if (!boundaries.empty())
            {
                combine(boundaries, input, context);
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


