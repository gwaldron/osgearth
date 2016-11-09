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
#include "Zone"
#include <osgEarth/TraversalData>
#include <osgUtil/CullVisitor>

#define LC "[Zone] "

using namespace osgEarth;
using namespace osgEarth::Splat;

bool
Zone::configure(const ConfigOptions& options, const Map* map, const osgDB::Options* dbo)
{
    ZoneOptions in(options);

    if ( in.name().isSet() )
        setName( in.name().get() );

    for(int i=0; i<in.boundaries().size(); ++i)
    {
        const osg::BoundingBox& box = in.boundaries().at(i);
        _boundaries.push_back( Boundary() );
        Boundary& b = _boundaries.back();
        
        GeoExtent extent(
            map->getSRS()->getGeographicSRS(),
            osg::clampBetween(box.xMin(), -180.0f, 180.0f),
            osg::clampBetween(box.yMin(),  -90.0f,  90.0f),
            osg::clampBetween(box.xMax(), -180.0f, 180.0f),
            osg::clampBetween(box.yMax(),  -90.0f,  90.0f));

        extent.createPolytope( b.tope );
        b.zmin2 = box.zMin() > -FLT_MAX ? box.zMin()*box.zMin() : box.zMin();
        b.zmax2 = box.zMax() <  FLT_MAX ? box.zMax()*box.zMax() : box.zMax();

        // this only needs to be very approximate.
        double meanRadius = extent.getSRS()->isGeographic() ?
            extent.getSRS()->getEllipsoid()->getRadiusEquator() : 0.0;
        b.meanRadius2 = meanRadius*meanRadius;
    }
    
    if ( in.surface().isSet() )
    {
        _surface = new Surface();
        if ( !_surface->configure(in.surface().get(), map, dbo) )
        {
            OE_WARN << LC << "Surface data is not properly configured; surface splatting disabled.\n";
            _surface = 0L;
        }
    }

    if ( in.landCover().isSet() )
    {
        _landCover = new LandCover();
        if ( !_landCover->configure(in.landCover().get(), map, dbo) )
        {
            OE_WARN << LC << "Land cover is not properly configured; land cover disabled.\n";
            _landCover = 0L;
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

osg::StateSet*
Zone::getOrCreateStateSet() 
{
    if ( !_stateSet.valid() )
        _stateSet = new osg::StateSet();
    return _stateSet.get();
}

void
ZoneOptions::fromConfig(const Config& conf)
{
    conf.getIfSet("name", _name);
    const Config* boundaries = conf.child_ptr("boundaries");
    if ( boundaries ) {
        for(ConfigSet::const_iterator i = boundaries->children().begin(); i != boundaries->children().end(); ++i) {
            _boundaries.push_back(osg::BoundingBox(
                i->value("xmin", -FLT_MAX), i->value("ymin", -FLT_MAX), i->value("zmin", -FLT_MAX),
                i->value("xmax",  FLT_MAX), i->value("ymax",  FLT_MAX), i->value("zmax",  FLT_MAX)));
        }
    }
    conf.getObjIfSet( "surface",    _surface );
    conf.getObjIfSet( "land_cover", _landCover );
}

Config
ZoneOptions::getConfig() const
{
    Config conf("zone");
    conf.addIfSet("name", _name);
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
        conf.add(regions);
    }
    conf.updateObjIfSet( "surface",    _surface );
    conf.updateObjIfSet( "land_cover", _landCover );
    return conf;
}

void
ZoneSwitcher::operator()(osg::Node* node, osg::NodeVisitor* nv)
{
    osg::StateSet* stateset = 0L;

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
                finalZoneIndex      = zoneIndex;
            }
            if ( _zones[z]->getLandCover() )
            {
                zoneIndex++;
            }
        }

        if ( !stateset )
        {
            stateset = _zones[0]->getStateSet();
            finalZoneIndex = 0;
        }                
        
        // Relays the zone index to the Patch callback.
        VisitorData::store(*nv, "oe.LandCover.zoneIndex", new RefUID(finalZoneIndex));
    }

    if ( stateset )
        static_cast<osgUtil::CullVisitor*>(nv)->pushStateSet( stateset );

    traverse(node, nv);

    if ( stateset )
        static_cast<osgUtil::CullVisitor*>(nv)->popStateSet();
}
