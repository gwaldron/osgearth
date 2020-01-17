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
#include "Zone"
#include <osgEarth/TraversalData>
#include <osgUtil/CullVisitor>

#define LC "[Zone] "

using namespace osgEarth;
using namespace osgEarth::Splat;

Zone::Zone() :
_uid(0)
{
    //nop
}

Zone::Zone(const ZoneOptions& options) :
_options(options),
_uid(0)
{
    //nop
}

bool
Zone::configure(const Map* map, const osgDB::Options* readOptions)
{
    if ( _options.name().isSet() )
        setName( _options.name().get() );

    for(int i=0; i<_options.boundaries().size(); ++i)
    {
        const osg::BoundingBox& box = _options.boundaries()[i];
        _boundaries.push_back( Boundary() );
        Boundary& b = _boundaries.back();
        
        GeoExtent extent(
            SpatialReference::get("wgs84"),
            osg::clampBetween(static_cast<float>(box.xMin()), -180.0f, 180.0f),
            osg::clampBetween(static_cast<float>(box.yMin()),  -90.0f,  90.0f),
            osg::clampBetween(static_cast<float>(box.xMax()), -180.0f, 180.0f),
            osg::clampBetween(static_cast<float>(box.yMax()),  -90.0f,  90.0f));

        extent.createPolytope( b.tope );
        b.zmin2 = box.zMin() > -FLT_MAX ? box.zMin()*box.zMin() : box.zMin();
        b.zmax2 = box.zMax() <  FLT_MAX ? box.zMax()*box.zMax() : box.zMax();

        // this only needs to be very approximate.
        double meanRadius = extent.getSRS()->isGeographic() ?
            extent.getSRS()->getEllipsoid()->getRadiusEquator() : 0.0;
        b.meanRadius2 = meanRadius*meanRadius;
    }
    
    if ( _options.surface().isSet() )
    {
        _surface = new Surface();
    }

    if (_surface.valid())
    {
        if ( !_surface->configure(_options.surface().get(), map, readOptions) )
        {
            OE_WARN << LC << "Surface data is not properly configured; surface splatting disabled.\n";
            _surface = 0L;
        }
    }

    if( _options.groundCover().isSet() )
    {
        _groundCover = new GroundCover(_options.groundCover().get());
    }

    if (_groundCover.valid())
    {
        if (_groundCover->configure(readOptions))
        {
            OE_DEBUG << LC << "Configured land cover group \"" << _groundCover->getName() << "\"\n";
        }
        else
        {
            OE_WARN << LC << "Land cover group is improperly configured\n";
            return false;
        }
    }

    return true;
}

bool
Zone::contains(const osg::Vec3& point) const
{
    for(Boundaries::const_iterator b = _boundaries.begin(); b != _boundaries.end(); ++b)
    {
        if ( b->tope.empty() )
        {
            return true;
        }

        else if ( b->tope.contains(point) )
        {
            double hat2 = point.length2() - b->meanRadius2; // assumes round earth
            if ( hat2 >= b->zmin2 && hat2 <= b->zmax2 )
            {
                return true;
            }
        }
    }

    return false;
}

void
Zone::resizeGLObjectBuffers(unsigned maxSize)
{
    if (_groundCover.valid())
        _groundCover->resizeGLObjectBuffers(maxSize);

    if (_surface.valid())
        _surface->resizeGLObjectBuffers(maxSize);
}

void
Zone::releaseGLObjects(osg::State* state) const
{
    if (_groundCover.valid())
        _groundCover->releaseGLObjects(state);

    if (_surface.valid())
        _surface->releaseGLObjects(state);
}

//........................................................................

void
ZoneOptions::fromConfig(const Config& conf)
{
    conf.get("name", _name);
    const Config* boundaries = conf.child_ptr("boundaries");
    if ( boundaries ) {
        for(ConfigSet::const_iterator i = boundaries->children().begin(); i != boundaries->children().end(); ++i) {
            _boundaries.push_back(osg::BoundingBox(
                i->value("xmin", -FLT_MAX), i->value("ymin", -FLT_MAX), i->value("zmin", -FLT_MAX),
                i->value("xmax",  FLT_MAX), i->value("ymax",  FLT_MAX), i->value("zmax",  FLT_MAX)));
        }
    }
    conf.get( "surface",     _surface );
    conf.get( "groundcover", _groundCover);
}

Config
ZoneOptions::getConfig() const
{
    Config conf("zone");
    conf.set("name", _name);
    if ( _boundaries.size() > 0 ) {
        Config regions("boundaries");
        for(int i=0; i<_boundaries.size(); ++i) {
            Config region("boundary");
            if ( _boundaries[i].xMin() > -FLT_MAX ) region.set("xmin", _boundaries[i].xMin());
            if ( _boundaries[i].yMin() > -FLT_MAX ) region.set("ymin", _boundaries[i].yMin());
            if ( _boundaries[i].zMin() > -FLT_MAX ) region.set("zmin", _boundaries[i].zMin());
            if ( _boundaries[i].xMax() <  FLT_MAX ) region.set("xmax", _boundaries[i].xMax());
            if ( _boundaries[i].yMax() <  FLT_MAX ) region.set("ymax", _boundaries[i].yMax());
            if ( _boundaries[i].zMax() <  FLT_MAX ) region.set("zmax", _boundaries[i].zMax());
            regions.add(region);
        }
        conf.set(regions);
    }
    conf.set( "surface",     _surface );
    conf.set( "groundcover", _groundCover );
    return conf;
}

void
ZoneSwitcher::operator()(osg::Node* node, osg::NodeVisitor* nv)
{
#if 0
    osg::StateSet* stateset = 0L;
    
    Zone* finalZone = 0L;

    if ( _zones.size() > 0 )
    {
        osg::Vec3d vp = nv->getViewPoint();
        double z2 = vp.length2();

        unsigned zoneIndex = 0;
        unsigned finalZoneIndex = ~0;

        for(unsigned z=0; z<_zones.size() && !stateset; ++z)
        {
            if ( _zones[z]->contains(vp) )
            {
                stateset = _zones[z]->getStateSet();
                finalZoneIndex = zoneIndex;
                finalZone = _zones[z].get();
            }
            if ( _zones[z]->getGroundCover() )
            {
                zoneIndex++;
            }
        }

        if ( !stateset )
        {
            stateset = _zones[0]->getStateSet();
            finalZoneIndex = 0;
        }                
        
        // Relays the zone to the GroundCoverPatchLayer.
        //VisitorData::store(*nv, "oe.GroundCover.zoneIndex", new RefUID(finalZoneIndex));
        VisitorData::store(*nv, "oe.GroundCover.zone", finalZone );
    }

    if ( stateset )
        static_cast<osgUtil::CullVisitor*>(nv)->pushStateSet( stateset );

    traverse(node, nv);

    if ( stateset )
        static_cast<osgUtil::CullVisitor*>(nv)->popStateSet();

    if (finalZone)
    {
        VisitorData::remove(*nv, "oe.GroundCover.zone");
    }
#endif
}
