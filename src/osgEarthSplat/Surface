/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2019 Pelican Mapping
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
#ifndef OSGEARTH_PROCEDURAL_SURFACE
#define OSGEARTH_PROCEDURAL_SURFACE 1

#include "Export"
#include "Coverage"
#include "SplatCatalog"
#include <osgEarth/Common>
#include <osgEarth/Config>
#include <osgEarth/URI>
#include <osgEarth/LandCover>

namespace osgEarth {
    class Map;
}

namespace osgEarth { namespace Splat
{
    using namespace osgEarth;

    /**
     * Options pertaining to the terrain surface raster splatting.
     */
    class SurfaceOptions : public ConfigOptions
    {
    public:
        SurfaceOptions(const ConfigOptions& conf = ConfigOptions()) : ConfigOptions(conf) {
            fromConfig( _conf );
        }

        /** URI of the catalog file that describes the classification codes in the coverage data.
            Define either this OR biomesURI. */
        optional<URI>& catalogURI() { return _catalogURI; }
        const optional<URI>& catalogURI() const { return _catalogURI; }

    protected:
        optional<URI> _catalogURI;

    public:
        void fromConfig(const Config& conf) {
            conf.get("catalog", _catalogURI);
        }
        
        Config getConfig() const {
            Config conf = ConfigOptions::getConfig();
            //conf.key() = "surface";
            conf.set("catalog", _catalogURI);
            return conf;
        }

        virtual void mergeConfig( const Config& conf ) {
            ConfigOptions::mergeConfig( conf );
            fromConfig( conf );
        }
    };

    /**
     * Surface splatting configuration.
     */
    class OSGEARTHSPLAT_EXPORT Surface : public osg::Referenced
    {
    public:
        /** Construct and empty data model */
        Surface();

    public:
        /** Catalog that maps coverage codes to textures */
        void setCatalog(SplatCatalog* catalog) { _catalog = catalog; }
        SplatCatalog* getCatalog() const { return _catalog.get(); }

        /**
         * Loads textures for splatting and generates a sampling function.
         * Returns false if something goes wrong
         */
        bool loadTextures(const LandCoverDictionary* landCoverDict, const osgDB::Options* readOptions);

        /** Gets the texture definition creates by loadTextures */
        const SplatTextureDef& getTextureDef() const { return _textureDef; }

        osg::StateSet* getOrCreateStateSet();
        osg::StateSet* getStateSet() const { return _stateSet.get(); }

        void resizeGLObjectBuffers(unsigned maxSize);

        void releaseGLObjects(osg::State* state) const;

    protected:
        virtual ~Surface() { }

    protected:
        osg::ref_ptr<SplatCatalog> _catalog;
        SplatTextureDef            _textureDef;
        osg::ref_ptr<osg::StateSet> _stateSet;

        osg::Texture* createLUTBuffer(const LandCoverDictionary* lcd) const;

    public:

        /** Initialize this Surface from a config. */
        bool configure(const ConfigOptions& conf, const Map* map, const osgDB::Options* dbo);
    };

} } // namespace osgEarth::Splat

OSGEARTH_SPECIALIZE_CONFIG(osgEarth::Splat::SurfaceOptions);

#endif // OSGEARTH_PROCEDURAL_SURFACE
