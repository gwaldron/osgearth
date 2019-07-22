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
#ifndef OSGEARTH_PROCEDURAL_ZONE
#define OSGEARTH_PROCEDURAL_ZONE 1

#include "Export"
#include "Surface"
#include "GroundCover"
#include <osgEarth/TerrainEngineNode>
#include <osg/BoundingBox>
#include <osg/Polytope>
#include <osg/StateSet>
#include <osg/NodeCallback>
#include <vector>

namespace osgDB {
    class Options;
}

namespace osgEarth { namespace Splat
{
    using namespace osgEarth;


    //........................................................................

    /**
     * Serializable options data for a Zone object
     */
    class OSGEARTHSPLAT_EXPORT ZoneOptions : public ConfigOptions
    {
    public:
        ZoneOptions(const ConfigOptions& conf = ConfigOptions()) : ConfigOptions(conf) {
            fromConfig(_conf);
        }

        //! Name of this zone (readable)
        optional<std::string> name() { return _name; }
        const optional<std::string> name() const { return _name; }

        //! Boundary set for this zone (optional)
        std::vector<osg::BoundingBox>& boundaries() { return _boundaries; }
        const std::vector<osg::BoundingBox>& boundaries() const { return _boundaries; }

        //! Surface rendering options
        optional<SurfaceOptions>& surface() { return _surface; }
        const optional<SurfaceOptions>& surface() const { return _surface; }

        //! Ground cover rendering options        
        optional<GroundCoverOptions>& groundCover() { return _groundCover; }
        const optional<GroundCoverOptions>& groundCover() const { return _groundCover; }

    protected:
        optional<std::string> _name;
        std::vector<osg::BoundingBox> _boundaries;
        optional<SurfaceOptions> _surface;
        optional<GroundCoverOptions> _groundCover;

    public:
        void fromConfig(const Config& conf);
        Config getConfig() const;
    };

    typedef std::vector<ZoneOptions> ZoneOptionsVector;
    

    /**
     * A zone limits a particular surface or land cover layer to a set of
     * geographic boundaries.
     */
    class OSGEARTHSPLAT_EXPORT Zone : public osg::Referenced
    {
    public:
        struct Boundary
        {
            GeoExtent     extent;
            double        zmin, zmin2;
            double        zmax, zmax2;
            double        meanRadius2;
            osg::Polytope tope;
        };
        typedef std::vector<Boundary> Boundaries;

    public:
        Zone();

        Zone(const ZoneOptions& options);

        void setName(const std::string& name) { _name = name; }
        const std::string& getName() const { return _name; }

        Boundaries& getBoundaries() { return _boundaries; }
        const Boundaries& getBoundaries() const { return _boundaries; }

        void setSurface(Surface* surface) { _surface = surface; }
        Surface* getSurface() const { return _surface.get(); }

        void setGroundCover(GroundCover* groundCover) { _groundCover = groundCover; }
        GroundCover* getGroundCover() const { return _groundCover.get(); }

        bool contains(const osg::Vec3& points) const;

        void setUID(UID uid) { _uid = uid; }
        UID getUID() const { return _uid; }

        void resizeGLObjectBuffers(unsigned maxSize);
        void releaseGLObjects(osg::State* state) const;

    protected:
        virtual ~Zone() { }
        std::string                 _name;
        UID                         _uid;
        Boundaries                  _boundaries;
        osg::ref_ptr<Surface>       _surface;
        osg::ref_ptr<GroundCover>   _groundCover;
        const ZoneOptions           _options;

    public:
        bool configure(const Map* map, const osgDB::Options* readOptions);
    };

    typedef std::vector<osg::ref_ptr<Zone> > Zones;


    
    
    /**
     * Cull callback that will select the most appropriate Zone based on the camera position
     * Internal - no export
     */
    class OSGEARTHSPLAT_EXPORT ZoneSwitcher : public osg::NodeCallback
    {
    public:
        ZoneSwitcher(const Zones& zones) : _zones(zones) { }

    public: // NodeCallback
        void operator()(osg::Node* node, osg::NodeVisitor* nv);

    protected:
        Zones _zones;
    };

} } // namespace osgEarth::Splat

OSGEARTH_SPECIALIZE_CONFIG(osgEarth::Splat::ZoneOptions);

#endif // OSGEARTH_PROCEDURAL_ZONE
