/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2010 Pelican Mapping
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
#include <osgEarthFeatures/FeatureSource>

#include <osgDB/FileNameUtils>
#include <OpenThreads/Mutex>
#include <OpenThreads/ScopedLock>

#include "FeatureMaskOptions"

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Drivers;

//------------------------------------------------------------------------

class FeatureMaskSource : public MaskSource
{
public:
    FeatureMaskSource( const MaskSourceOptions& options )
        : MaskSource( options ), _options( options )
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
    void initialize( const std::string& referenceURI, const osgEarth::Map* map )
    {
        MaskSource::initialize( referenceURI, map );
    }

    osg::Vec3dArray* createBoundary( ProgressCallback* progress )
    {
        if ( _features.valid() && _features->getFeatureProfile() )
        {
            osg::ref_ptr<FeatureCursor> cursor = _features->createFeatureCursor();
            if ( cursor )
            {
                if ( cursor->hasMore() )
                {
                    Feature* f = cursor->nextFeature();
                    if ( f && f->getGeometry() )
                    {
                        return f->getGeometry();
                    }
                }
            }
        }
        return 0L;
    }

private:
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
