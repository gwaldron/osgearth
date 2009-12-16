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
#include <osgEarthFeatures/FeatureGridder>
#include <osgEarthFeatures/Geometry>
#include <osg/Notify>

#ifdef OSGEARTH_HAVE_GEOS
#  include <osgEarthFeatures/GEOS>
#  include <geos/geom/Geometry.h>
#  include <geos/geom/GeometryFactory.h>
#  include <geos/operation/overlay/OverlayOp.h>
   using namespace geos;
   using namespace geos::operation;
#endif

using namespace osgEarth;
using namespace osgEarth::Features;

bool
FeatureGridder::isSupported()
{
#ifdef OSGEARTH_HAVE_GEOS
    static bool s_isSupported = true;
#else
    static bool s_isSupported = false;
#endif

    return s_isSupported;
}

FeatureGridder::FeatureGridder(const FeatureList& input,
                               const GeoExtent&   inputExtent,
                               double             cellWidth,
                               double             cellHeight) :
_input( input ),
_inputExtent( inputExtent ),
_cellWidth( cellWidth ),
_cellHeight( cellHeight ),
_cellsX( 0 ),
_cellsY( 0 )
{
    if ( !isSupported() )
    {
        osg::notify(osg::WARN) << "[osgEarth] FeatureGridder not supported - requires GEOS" << std::endl;
    }

#ifdef OSGEARTH_HAVE_GEOS

    if ( isSupported() && _cellWidth > 0 && _cellHeight > 0 )
    {
        _cellsX = (int)::ceil(_inputExtent.width() / _cellWidth);
        _cellsY = (int)::ceil(_inputExtent.height() / _cellHeight);
    }

    osg::notify(osg::NOTICE) << "[osgEarth] Grid cells = " << _cellsX << " x " << _cellsY << std::endl;

    // import geometry to geos:
    for( FeatureList::const_iterator i = input.begin(); i != input.end(); ++i )
    {
        geom::Geometry* geosGeom = GEOSUtils::importGeometry( i->get()->getGeometry() );
        // insert even if it's null
        _geosGeoms.push_back( geosGeom );
    }

#endif // OSGEARTH_HAVE_GEOS
}

FeatureGridder::~FeatureGridder()
{
    for( std::list<void*>::iterator i = _geosGeoms.begin(); i != _geosGeoms.end(); ++i )
    {
        geom::Geometry* geom = static_cast<geom::Geometry*>( *i );
        if ( geom )
            geom->getFactory()->destroyGeometry( geom );
    }
}

int
FeatureGridder::getNumCells() const
{
    return _cellsX * _cellsY;
}

bool
FeatureGridder::getCell( int i, FeatureList& output ) const
{
    bool success = false;

#ifdef OSGEARTH_HAVE_GEOS

    if ( i >= 0 && i < (_cellsX*_cellsY) )
    {
        int x = i % _cellsX;
        int y = i / _cellsX;

        double xmin = _inputExtent.xMin() + _cellWidth  * (double)x;
        double ymin = _inputExtent.yMin() + _cellHeight * (double)y;
        double xmax = osg::clampBelow( _inputExtent.xMin() + _cellWidth  * (double)(x+1), _inputExtent.xMax() );
        double ymax = osg::clampBelow( _inputExtent.yMin() + _cellHeight * (double)(y+1), _inputExtent.yMax() );

        GeoExtent cx( _inputExtent.getSRS(), xmin, ymin, xmax, ymax );

        geom::GeometryFactory* f = new geom::GeometryFactory();

        // create the intersection polygon:
        osg::ref_ptr<Polygon> poly = new Polygon( 4 );
        poly->push_back( osg::Vec3d( cx.xMin(), cx.yMin(), 0 ) );
        poly->push_back( osg::Vec3d( cx.xMax(), cx.yMin(), 0 ) );
        poly->push_back( osg::Vec3d( cx.xMax(), cx.yMax(), 0 ) );
        poly->push_back( osg::Vec3d( cx.xMin(), cx.yMax(), 0 ) );        
        geom::Geometry* cropGeom = GEOSUtils::importGeometry( poly.get() );

        // intersect it with each feature:
        int count =0;
        FeatureList::const_iterator f_i = _input.begin();
        std::list<void*>::const_iterator g_i = _geosGeoms.begin();
        for( ; f_i != _input.end(); ++f_i, ++g_i )
        {
            Feature* feature = f_i->get();
            geom::Geometry* inGeom = static_cast<geom::Geometry*>( *g_i );
            Geometry* featureGeom = 0L;
            if ( inGeom )
            {            
                geom::Geometry* outGeom = overlay::OverlayOp::overlayOp(
                    inGeom, cropGeom,
                    overlay::OverlayOp::opINTERSECTION );
                    
                if ( outGeom )
                {
                    featureGeom = GEOSUtils::exportGeometry( outGeom );
                    f->destroyGeometry( outGeom );
                    if ( featureGeom && featureGeom->isValid() )
                    {
                        Feature* newFeature = osg::clone<Feature>( feature, osg::CopyOp::DEEP_COPY_ALL );
                        newFeature->setGeometry( featureGeom );
                        output.push_back( newFeature );
                        count++;
                    }
                }
            }
        }

        // clean up
        f->destroyGeometry( cropGeom );
        delete f;

        osg::notify(osg::NOTICE)
            << "[osgEarth] Grid cell " << i << ": " << count << " features; extent=" << cx.toString() 
            << std::endl;
    }

#endif // OSGEARTH_HAVE_GEOS

    return success;
}

