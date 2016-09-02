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
#ifndef OSGEARTH_EXT_TERRAIN_SHADER_OPTIONS
#define OSGEARTH_EXT_TERRAIN_SHADER_OPTIONS 1

#include <osgEarth/Config>
#include <osgEarth/URI>
#include <vector>

namespace osgEarth { namespace TerrainShader
{
    using namespace osgEarth;

    /**
     * Options for applying an inline shader to the terrain.
     */
    class TerrainShaderOptions : public DriverConfigOptions // NO EXPORT; header only
    {
    public:
        struct Code
        {
            std::string   _source;
            optional<URI> _uri;
        };

        struct Sampler
        {
            std::string      _name;
            std::vector<URI> _uris;
        };

        struct Uniform
        {
            std::string     _name;
            optional<float> _value;
        };

        std::vector<Code>& code() { return _code; }
        const std::vector<Code>& code() const { return _code; }

        std::vector<Sampler>& samplers() { return _samplers; }
        const std::vector<Sampler>& samplers() const { return _samplers; }

        std::vector<Uniform>& uniforms() { return _uniforms; }
        const std::vector<Uniform>& uniforms() const { return _uniforms; }

        optional<std::string>& landCoverGroup() { return _landCoverGroup; }
        const optional<std::string>& landCoverGroup() const { return _landCoverGroup; }

        optional<unsigned>& landCoverLOD() { return _landCoverLOD; }
        const optional<unsigned>& landCoverLOD() const { return _landCoverLOD; }

    public:
        TerrainShaderOptions( const ConfigOptions& opt =ConfigOptions() ) : DriverConfigOptions( opt )
        {
            setDriver( "terrainshader" );
            _landCoverLOD.init(15);
            fromConfig( _conf );
        }

        virtual ~TerrainShaderOptions() { }

    public:
        Config getConfig() const
        {
            Config conf = DriverConfigOptions::getConfig();

            for(unsigned i=0; i<_code.size(); ++i) {
                Config c("code", _code[i]._source);
                c.addIfSet("url", _code[i]._uri);
                conf.add( c );
            }

            for(unsigned i=0; i<_samplers.size(); ++i) {
                Config c("sampler");
                c.add("name", _samplers[i]._name);
                if ( _samplers[i]._uris.size() > 1 ) {
                    Config urlarray("array");
                    c.add( urlarray );
                    for( std::vector<URI>::const_iterator j = _samplers[i]._uris.begin(); j != _samplers[i]._uris.end(); ++j) {
                        urlarray.add( j->getConfig() );
                    }
                }
                else if ( _samplers[i]._uris.size() == 1 ) {
                    c.add( _samplers[i]._uris.back().getConfig() );
                }
                conf.add( c );
            }

            for(unsigned i=0; i<_uniforms.size(); ++i) {
                Config c("uniform");
                c.add("name", _uniforms[i]._name);
                c.addIfSet("value", _uniforms[i]._value);
                conf.add( c );
            }

            //conf.addIfSet("land_cover", _landCover);
            conf.addIfSet("land_cover_group", _landCoverGroup);
            conf.addIfSet("land_cover_lod",   _landCoverLOD);
            return conf;
        }

    protected:
        void mergeConfig( const Config& conf ) {
            DriverConfigOptions::mergeConfig( conf );
            fromConfig( conf );
        }

    private:
        void fromConfig( const Config& conf ) {

            ConfigSet s = conf.children("code");
            for(ConfigSet::const_iterator i = s.begin(); i != s.end(); ++i) {
                _code.push_back(Code());
                _code.back()._source = i->value();
                i->getIfSet("url", _code.back()._uri);
            }

            s = conf.children("sampler");
            for(ConfigSet::const_iterator i = s.begin(); i != s.end(); ++i) {
                _samplers.push_back(Sampler());
                _samplers.back()._name = i->value("name");
                const Config* urlarray = i->find("array");
                if ( urlarray ) {
                    ConfigSet uris = urlarray->children("url");
                    for(ConfigSet::const_iterator j = uris.begin(); j != uris.end(); ++j) {
                        URI uri( j->value(), URIContext(conf.referrer()) );
                        _samplers.back()._uris.push_back( uri );
                    }
                }
                else {
                   optional<URI> uri;
                   i->getIfSet("url", uri);
                   if ( uri.isSet() )
                       _samplers.back()._uris.push_back( uri.get() );
                }
            }

            s = conf.children("uniform");
            for(ConfigSet::const_iterator i = s.begin(); i != s.end(); ++i) {
                _uniforms.push_back(Uniform());
                _uniforms.back()._name = i->value("name");
                i->getIfSet("value", _uniforms.back()._value);
            }
            
            //conf.getIfSet("land_cover", _landCover);
            conf.getIfSet("land_cover_group", _landCoverGroup);
            conf.getIfSet("land_cover_lod",   _landCoverLOD);
        }

        std::vector<Code>     _code;
        std::vector<Sampler>  _samplers;
        std::vector<Uniform>  _uniforms;
        optional<std::string> _landCoverGroup;
        optional<unsigned>    _landCoverLOD;
    };

} } // namespace osgEarth::TerrainShader

#endif // OSGEARTH_EXT_TERRAIN_SHADER_OPTIONS

