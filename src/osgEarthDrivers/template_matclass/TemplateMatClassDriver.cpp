/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2016 Pelican Mapping
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

#include <osgEarthFeatures/FeatureTileSource>
#include <osgEarthFeatures/TransformFilter>
#include <osgEarthSymbology/Style>
#include <osgEarth/Registry>
#include <osgEarth/FileUtils>
#include <osgEarth/ImageLayer>

#include <osg/Notify>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>

#include "TemplateMatClassOptions"

#include <sstream>

#define LC "[MatClass] "

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Drivers;

/********************************************************************/

class TemplateMatClassTileSource : public TileSource
{
private:
    osg::ref_ptr<ImageLayer>     _imageLayer;
    osg::ref_ptr<FeatureSource>  _featureSource;
    osg::ref_ptr<osgDB::Options> _dbOptions;
    TemplateMatClassOptions      _options;

public:
    TemplateMatClassTileSource( const TileSourceOptions& options ) :
        TileSource( options ),
        _options( options )
    {
        //nop
    }

public: // TileSource interface

    // override
    // One-time initialization. This is called by osgEarth.
    Status initialize(const osgDB::Options* dbOptions)
    {
        _dbOptions = Registry::instance()->cloneOrCreateOptions(dbOptions);

        const Profile* profile = getProfile();
        if ( !profile )
        {
            profile = osgEarth::Registry::instance()->getGlobalGeodeticProfile();
            setProfile( profile );
        }

        // load the image layer:
        if ( _options.imageLayerOptions().isSet() )
        {
            _imageLayer = new ImageLayer( _options.imageLayerOptions().get() );
            _imageLayer->setTargetProfileHint( profile );
            // TODO: what about the cache?
        }

        // load the feature source:
        if ( _options.featureSourceOptions().isSet() )
        {
            _featureSource = FeatureSourceFactory::create( _options.featureSourceOptions().get() );
            _featureSource->open( _dbOptions.get() );
        }

        return Status::OK();
    }

    // Tells the layer not to cache data from this tile source.
    CachePolicy getCachePolicyHint(const Profile* profile) const 
    {
        return CachePolicy::NO_CACHE;
    }

    // override
    // Creates an image.
    osg::Image* createImage(const TileKey&    key,
                            ProgressCallback* progress )
    {
        if ( !_imageLayer.valid() || !_featureSource.valid() )
            return 0L;

        // fetch the image for this key:
        GeoImage image = _imageLayer->createImage(key, progress);
        if ( !image.valid() )
            return 0L;

        // fetch a set of features for this key. The features are in their
        // own SRS, so we need to transform:
        const SpatialReference* featureSRS = _featureSource->getFeatureProfile()->getSRS();
        GeoExtent extentInFeatureSRS = key.getExtent().transform( featureSRS );

        // assemble a spatial query. It helps if your features have a spatial index.
        Query query;
        query.bounds() = extentInFeatureSRS.bounds();
        //query.expression() = ... // SQL expression compatible with data source
        osg::ref_ptr<FeatureCursor> cursor = _featureSource->createFeatureCursor(query);

        // create a new image to return.
        osg::Image* output = new osg::Image();
        //output->allocateImage(128, 128, 1, GL_RGB, GL_UNSIGNED_BYTE);

        // do your magic here.

        return output;
    }
};


/**
 * Plugin entry point for the AGGLite feature rasterizer
 */
class TemplateMatClassDriver : public TileSourceDriver
{
    public:
        TemplateMatClassDriver() {}

        virtual const char* className() const
        {
            return "Template mat class driver";
        }
        
        virtual bool acceptsExtension(const std::string& extension) const
        {
            return
                osgDB::equalCaseInsensitive( extension, "osgearth_template_matclass" );
        }

        virtual ReadResult readObject(const std::string& file_name, const Options* options) const
        {
            std::string ext = osgDB::getFileExtension( file_name );
            if ( !acceptsExtension( ext ) )
            {
                return ReadResult::FILE_NOT_HANDLED;
            }

            return new TemplateMatClassTileSource( getTileSourceOptions(options) );
        }
};

REGISTER_OSGPLUGIN(osgearth_template_matclass, TemplateMatClassDriver)
