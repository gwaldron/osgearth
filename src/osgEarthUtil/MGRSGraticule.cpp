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
#include <osgEarthUtil/MGRSGraticule>

#include <osgDB/FileNameUtils>
#include <osgDB/ReaderWriter>

#define LC "[MGRSGraticule] "

using namespace osgEarth;
using namespace osgEarth::Util;

#define MGRS_GRATICULE_EXTENSION "osgearthutil_mgrs_graticule"

//---------------------------------------------------------------------------

MGRSGraticuleOptions::MGRSGraticuleOptions( const Config& conf ) :
UTMGraticuleOptions( conf )
{
    mergeConfig( _conf );
}

void
MGRSGraticuleOptions::mergeConfig( const Config& conf )
{
    //todo
}

Config
MGRSGraticuleOptions::getConfig() const
{
    Config conf = ConfigOptions::newConfig();
    conf.key() = "mgrs_graticule";
    //todo
    return conf;
}

//---------------------------------------------------------------------------


MGRSGraticule::MGRSGraticule( MapNode* mapNode ) :
UTMGraticule( mapNode )
{
    //nop
}

MGRSGraticule::MGRSGraticule( MapNode* mapNode, const MGRSGraticuleOptions& options ) :
UTMGraticule( mapNode, options )
{
    _options = options;
}

osg::Group*
MGRSGraticule::buildGZDChildren( osg::Group* parent, const std::string& gzd )
{
    //todo
    return 0L;
}

osg::Node*
MGRSGraticule::buildSQIDTiles( const std::string& gzd )
{
    const GeoExtent& extent = _gzd[gzd];

    return 0L;
}


//---------------------------------------------------------------------------

namespace osgEarth { namespace Util
{
    // OSG Plugin for loading subsequent graticule levels
    class MGRSGraticuleFactory : public osgDB::ReaderWriter
    {
    public:
        virtual const char* className()
        {
            supportsExtension( MGRS_GRATICULE_EXTENSION, "osgEarth MGRS graticule" );
            return "osgEarth MGRS graticule LOD loader";
        }

        virtual bool acceptsExtension(const std::string& extension) const
        {
            return osgDB::equalCaseInsensitive(extension, MGRS_GRATICULE_EXTENSION);
        }

        virtual ReadResult readNode(const std::string& uri, const Options* options) const
        {        
            std::string ext = osgDB::getFileExtension( uri );
            if ( !acceptsExtension( ext ) )
                return ReadResult::FILE_NOT_HANDLED;

            // the graticule definition is formatted: LEVEL_ID.MARKER.EXTENSION
            std::string def = osgDB::getNameLessExtension( uri );
            
            std::string marker = osgDB::getFileExtension( def );
            def = osgDB::getNameLessExtension( def );

            char gzd[8];
            unsigned id;
            sscanf( def.c_str(), "%s_%d", gzd, &id );

            // look up the graticule referenced in the location name:
            MGRSGraticule* graticule = 0L;
            {
                Threading::ScopedMutexLock lock( UTMGraticule::s_graticuleMutex );
                UTMGraticule::UTMGraticuleRegistry::iterator i = UTMGraticule::s_graticuleRegistry.find(id);
                if ( i != UTMGraticule::s_graticuleRegistry.end() )
                    graticule = dynamic_cast<MGRSGraticule*>( i->second.get() );
            }

            osg::Node* result = graticule->buildSQIDTiles( std::string(gzd) );
            return result ? ReadResult(result) : ReadResult::ERROR_IN_READING_FILE;
        }
    };
    REGISTER_OSGPLUGIN(MGRS_GRATICULE_EXTENSION, MGRSGraticuleFactory)

} } // namespace osgEarth::Util


