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

#include <osgEarth/MaskSource>
#include <osgEarth/Registry>
#include <osgEarth/Map>
#include <osgEarthFeatures/TransformFilter>
#include <osgEarthFeatures/FeatureSource>

#include <osgDB/FileNameUtils>
#include <OpenThreads/Mutex>
#include <OpenThreads/ScopedLock>

#include "FeatureMaskOptions"

#define LC "[FeatureMaskDriver] "

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Drivers;

//------------------------------------------------------------------------

class FeatureMaskSource : public MaskSource
{
public:
    FeatureMaskSource( const MaskSourceOptions& options )
        : MaskSource( options ), _options( options ), _failed( false )
    {
        // the data source from which to pull features:
        if ( _options.featureSource().valid() )
        {
            _features = _options.featureSource().get();
        }
        else if ( _options.featureOptions().isSet() )
        {
            _features = FeatureSourceFactory::create( _options.featureOptions().value() );
            if ( !_features.valid() )
            {
                OE_WARN << "FeatureModelSource - no valid feature source provided" << std::endl;
            }
        }
    }

    const MaskSourceOptions& getOptions() const { return _options; }

    //override
    void initialize(const osgDB::Options* dbOptions)
    {
        MaskSource::initialize( dbOptions );

        if ( _features.valid() )
        {
            _features->initialize( dbOptions );
        } 
        else
        {
            OE_WARN << LC << "No FeatureSource; nothing will be rendered (" << getName() << ")" << std::endl;
        }
    }

    osg::Vec3dArray* createBoundary(const SpatialReference* srs, ProgressCallback* progress)
    {
        if ( _failed )
            return 0L;

        if ( _features.valid() )
        {
            if ( _features->getFeatureProfile() )
            {
                osg::ref_ptr<FeatureCursor> cursor = _features->createFeatureCursor();
                if ( cursor.valid() && cursor->hasMore() )
                {
                    Feature* f = cursor->nextFeature();
                    if ( f && f->getGeometry() )
                    {
                        // Init a filter to tranform feature in desired SRS 
                        if (!srs->isEquivalentTo(_features->getFeatureProfile()->getSRS()))
                        {
                            FilterContext cx;
                            cx.setProfile( new FeatureProfile(_features->getFeatureProfile()->getExtent()) );

                            TransformFilter xform( srs );
                            FeatureList featureList;
                            featureList.push_back(f);
                            cx = xform.push(featureList, cx);
                        }

                        return f->getGeometry()->toVec3dArray();
                    }
                }
            }
            else
            {
                OE_WARN << LC << "Failed to create boundary; feature source has no SRS" << std::endl;
                _failed = true;
            }
        }
        else
        {
            OE_WARN << LC << "Unable to create boundary; invalid feature source" << std::endl;
            _failed = true;
        }
        return 0L;
    }

private:
    bool _failed;
    const FeatureMaskOptions _options;
    osg::ref_ptr<FeatureSource> _features;
};

//------------------------------------------------------------------------

class FeatureMaskDriver : public MaskSourceDriver
{
public:
    FeatureMaskDriver()
    {
        supportsExtension( "osgearth_mask_feature", "osgEarth feature mask plugin" );
    }

    virtual const char* className()
    {
        return "osgEarth Feature Mask Plugin";
    }

    virtual ReadResult readObject(const std::string& file_name, const Options* options) const
    {
        if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
            return ReadResult::FILE_NOT_HANDLED;

        return new FeatureMaskSource( getMaskSourceOptions(options) );
    }
};

REGISTER_OSGPLUGIN(osgearth_mask_feature, FeatureMaskDriver)
