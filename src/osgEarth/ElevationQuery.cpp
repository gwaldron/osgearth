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
#include <osgEarth/ElevationQuery>
#include <osgUtil/LineSegmentIntersector>
#include <osgEarth/Map>
#include <osgEarth/ElevationPool>
#include <osgUtil/IntersectionVisitor>
#include <osgSim/LineOfSight>

#define LC "[ElevationQuery] "

using namespace osgEarth;


ElevationQuery::ElevationQuery()
{
    reset();
}

ElevationQuery::ElevationQuery(const Map* map)
{
    setMap(map);
}

ElevationQuery::ElevationQuery(const MapFrame& mapFrame)
{
    setMapFrame(mapFrame);
}

void
ElevationQuery::setMap(const Map* map)
{
    _mapf.setMap(map);
    reset();
}

void
ElevationQuery::setMapFrame(const MapFrame& frame)
{
    _mapf = frame;
    reset();
}

void
ElevationQuery::reset()
{
    // set read callback for IntersectionVisitor
    _ivrc = new osgSim::DatabaseCacheReadCallback();

    // find terrain patch layers.
    gatherPatchLayers();

    // clear any active envelope
    _envelope = 0L;
}

void
ElevationQuery::sync()
{
    if ( _mapf.needsSync() )
    {
        _mapf.sync();
        reset();
    }
}

void
ElevationQuery::gatherPatchLayers()
{
    // cache a vector of terrain patch models.
    _patchLayers.clear();
    ModelLayerVector modelLayers;
    _mapf.getLayers(modelLayers);
    for(ModelLayerVector::const_iterator i = modelLayers.begin();
        i != modelLayers.end();
        ++i)
    {
        if ( i->get()->isTerrainPatch() )
            _patchLayers.push_back( i->get() );
    }
}


float
ElevationQuery::getElevation(const GeoPoint& point,
                             double          desiredResolution,
                             double*         out_actualResolution)
{
    float result = NO_DATA_VALUE;

    sync();
    if ( point.altitudeMode() == ALTMODE_ABSOLUTE )
    {
        getElevationImpl( point, result, desiredResolution, out_actualResolution );
    }
    else
    {
        GeoPoint point_abs( point.getSRS(), point.x(), point.y(), 0.0, ALTMODE_ABSOLUTE );
        getElevationImpl( point_abs, result, desiredResolution, out_actualResolution );
    }

    return result;
}

bool
ElevationQuery::getElevations(std::vector<osg::Vec3d>& points,
                              const SpatialReference*  pointsSRS,
                              bool                     ignoreZ,
                              double                   desiredResolution )
{
    sync();
    for( osg::Vec3dArray::iterator i = points.begin(); i != points.end(); ++i )
    {
        float elevation;
        double z = (*i).z();
        GeoPoint p(pointsSRS, *i, ALTMODE_ABSOLUTE);
        if ( getElevationImpl(p, elevation, desiredResolution, 0L))
        {
            if (elevation == NO_DATA_VALUE)
            {
                elevation = 0.0;
            }

            (*i).z() = ignoreZ ? elevation : elevation + z;
        }
    }
    return true;
}

bool
ElevationQuery::getElevations(const std::vector<osg::Vec3d>& points,
                              const SpatialReference*        pointsSRS,
                              std::vector<float>&            out_elevations,
                              double                         desiredResolution )
{
    sync();
    for( osg::Vec3dArray::const_iterator i = points.begin(); i != points.end(); ++i )
    {
        float elevation;
        GeoPoint p(pointsSRS, *i, ALTMODE_ABSOLUTE);

        if ( getElevationImpl(p, elevation, desiredResolution, 0L) )
        {
            out_elevations.push_back( elevation );
        }
        else
        {
            out_elevations.push_back( 0.0 );
        }
    }
    return true;
}

bool
ElevationQuery::getElevationImpl(const GeoPoint& point,
                                 float&          out_elevation,
                                 double          desiredResolution,
                                 double*         out_actualResolution)
{
    // assertion.
    if ( !point.isAbsolute() )
    {
        OE_WARN << LC << "Assertion failure; input must be absolute" << std::endl;
        return false;
    }

    osg::Timer_t begin = osg::Timer::instance()->tick();

    // first try the terrain patches.
    if ( _patchLayers.size() > 0 )
    {
        osgUtil::IntersectionVisitor iv;

        if ( _ivrc.valid() )
            iv.setReadCallback(_ivrc.get());

        for(std::vector<ModelLayer*>::iterator i = _patchLayers.begin(); i != _patchLayers.end(); ++i)
        {
            // find the scene graph for this layer:
            osg::Node* node = (*i)->getSceneGraph( _mapf.getUID() );
            if ( node )
            {
                // configure for intersection:
                osg::Vec3d surface;
                point.toWorld( surface );

                // trivial bounds check:
                if ( node->getBound().contains(surface) )
                {
                    osg::Vec3d nvector;
                    point.createWorldUpVector(nvector);

                    osg::Vec3d start( surface + nvector*5e5 );
                    osg::Vec3d end  ( surface - nvector*5e5 );

                    // first time through, set up the intersector on demand
                    if ( !_patchLayersLSI.valid() )
                    {
                        _patchLayersLSI = new osgUtil::LineSegmentIntersector(start, end);
                        _patchLayersLSI->setIntersectionLimit( _patchLayersLSI->LIMIT_NEAREST );
                    }
                    else
                    {
                        _patchLayersLSI->reset();
                        _patchLayersLSI->setStart( start );
                        _patchLayersLSI->setEnd  ( end );
                    }

                    // try it.
                    iv.setIntersector( _patchLayersLSI.get() );
                    node->accept( iv );

                    // check for a result!!
                    if ( _patchLayersLSI->containsIntersections() )
                    {
                        osg::Vec3d isect = _patchLayersLSI->getIntersections().begin()->getWorldIntersectPoint();

                        // transform back to input SRS:
                        GeoPoint output;
                        output.fromWorld( point.getSRS(), isect );
                        out_elevation = (float)output.z();
                        if ( out_actualResolution )
                            *out_actualResolution = 0.0;

                        return true;
                    }
                }
                else
                {
                    //OE_INFO << LC << "Trivial rejection (bounds check)" << std::endl;
                }
            }
        }
    }

    if ( _mapf.elevationLayers().empty() )
    {
        // this means there are no heightfields.
        out_elevation = NO_DATA_VALUE;
        return true;
    }

    // tile size (resolution of elevation tiles)
    unsigned tileSize = 257; // yes?

    // default LOD:
    unsigned lod = 23u;

    // attempt to map the requested resolution to an LOD:
    if (desiredResolution > 0.0)
    {
        int level = _mapf.getProfile()->getLevelOfDetailForHorizResolution(desiredResolution, tileSize);
        if ( level > 0 )
            lod = level;
    }

    // do we need a new ElevationEnvelope?
    if (!_envelope.valid() ||
        !point.getSRS()->isHorizEquivalentTo(_envelope->getSRS()) ||
        lod != _envelope->getLOD())
    {        
        _envelope = _mapf.getElevationPool()->createEnvelope(point.getSRS(), lod);
    }

    // sample the elevation, and if requested, the resolution as well:
    if (out_actualResolution)
    {
        std::pair<float, float> result = _envelope->getElevationAndResolution(point.x(), point.y());
        out_elevation = result.first;
        *out_actualResolution = result.second;
    }
    else
    {
        out_elevation = _envelope->getElevation(point.x(), point.y());
    }

    return out_elevation != NO_DATA_VALUE;
}
