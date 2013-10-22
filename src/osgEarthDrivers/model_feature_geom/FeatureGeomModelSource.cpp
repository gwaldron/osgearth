/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2013 Pelican Mapping
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

#include <osgEarth/ModelSource>
#include <osgEarth/Registry>
#include <osgEarth/Map>
#include <osgEarthFeatures/GeometryCompiler>

#include <osg/Notify>
#include <osg/MatrixTransform>
#include <osgDB/FileNameUtils>

#include "FeatureGeomModelOptions"

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;
using namespace osgEarth::Drivers;

//------------------------------------------------------------------------

namespace
{
    //------------------------------------------------------------------------

    /** The model source implementation for feature_geom */
    class FeatureGeomModelSource : public FeatureModelSource
    {
    public:
        FeatureGeomModelSource( const ModelSourceOptions& options )
            : FeatureModelSource( options ),
              _options( options )
        {
            //nop
        }

        const FeatureGeomModelOptions& getOptions() const { return _options; }

    public: // FeatureModelSource

        void initialize( const osgDB::Options* dbOptions )
        {
            FeatureModelSource::initialize( dbOptions );
        }

        FeatureNodeFactory* createFeatureNodeFactory()
        {
            return new GeomFeatureNodeFactory( _options.compilerOptions() );
        }

    private:
        const FeatureGeomModelOptions _options;
    };
}

//------------------------------------------------------------------------

/** The plugin factory object */
class FeatureGeomModelSourceDriver : public ModelSourceDriver
{
public:
    FeatureGeomModelSourceDriver()
    {
        supportsExtension( "osgearth_model_feature_geom", "osgEarth feature geom plugin" );
    }

    virtual const char* className()
    {
        return "osgEarth Feature Geom Model Plugin";
    }

    virtual ReadResult readObject(const std::string& file_name, const Options* options) const
    {
        if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
            return ReadResult::FILE_NOT_HANDLED;

        return new FeatureGeomModelSource( getModelSourceOptions(options) );
    }
};

REGISTER_OSGPLUGIN(osgearth_model_feature_geom, FeatureGeomModelSourceDriver) 
