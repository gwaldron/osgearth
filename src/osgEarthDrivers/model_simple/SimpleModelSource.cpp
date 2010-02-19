/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2009 Pelican Ventures, Inc.
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

#include "SimpleModelOptions"
#include <osgEarth/ModelSource>
#include <osgEarth/Registry>
#include <osgEarth/Map>
#include <osgEarth/FileUtils>
#include <osg/Notify>
#include <osgDB/FileNameUtils>

using namespace osgEarth;
using namespace osgEarth::Drivers;

class SimpleModelSource : public ModelSource
{
public:
    SimpleModelSource( const PluginOptions* options ) : ModelSource( options )
    {
        _settings = dynamic_cast<const SimpleModelOptions*>( options );
        if ( !_settings.valid() )
            _settings = new SimpleModelOptions( options );
    }

    //override
    void initialize( const std::string& referenceURI, const osgEarth::Map* map )
    {
        ModelSource::initialize( referenceURI, map );

        _url = osgEarth::getFullPath( referenceURI, _settings->url().value() );
    }

    // override
    osg::Node* createNode( ProgressCallback* progress )
    {
        osg::ref_ptr<osg::Node> output;
        if ( HTTPClient::readNodeFile( _url, output, getOptions(), progress ) == HTTPClient::RESULT_OK )
        {
            osg::notify(osg::NOTICE) << "[osgEarth] Loaded OK: " << _url << std::endl;
        }
        else
        {
            osg::notify(osg::NOTICE) << "[osgEarth] FAILED to load " << _url << std::endl;
        }
        return output.release();
    }

private:
    std::string _url;
    osg::ref_ptr<const SimpleModelOptions> _settings;
};


class SimpleModelSourceFactory : public osgDB::ReaderWriter
{
public:
    SimpleModelSourceFactory()
    {
        supportsExtension( "osgearth_model_simple", "osgEarth simple model plugin" );
    }

    virtual const char* className()
    {
        return "osgEarth Simple Model Plugin";
    }

    SimpleModelSource* create( const PluginOptions* options )
    {
        return new SimpleModelSource( options );
    }

    virtual ReadResult readObject(const std::string& file_name, const Options* options) const
    {
        if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
            return ReadResult::FILE_NOT_HANDLED;

        SimpleModelSourceFactory* nonConstThis = const_cast<SimpleModelSourceFactory*>(this);
        return nonConstThis->create( static_cast<const PluginOptions*>(options) );
    }
};

REGISTER_OSGPLUGIN(osgearth_model_simple, SimpleModelSourceFactory) 
