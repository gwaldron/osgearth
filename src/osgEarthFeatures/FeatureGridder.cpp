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
#include <osgEarthFeatures/FeatureGridder>
#include <osgEarthSymbology/Geometry>
#include <osg/Notify>
#include <osg/Timer>

using namespace osgEarth;
using namespace osgEarth::Features;

/**************************************************************************/

#define PROP_CELL_SIZE         "cell_size"
#define PROP_CULLING_TECHNIQUE "culling_technique"
#define PROP_SPATIALIZE_GROUPS "spatialize_groups"
#define PROP_CLUSTER_CULLING   "cluster_culling"

GriddingPolicy::GriddingPolicy( const Config& conf ) :
_cellSize( DBL_MAX ),
_cullingTechnique( GriddingPolicy::CULL_BY_CENTROID ),
_spatializeGroups( true ),
_clusterCulling( false )
{
    fromConfig( conf );
}

Config
GriddingPolicy::getConfig() const 
{
    Config conf;

    conf.updateIfSet( PROP_CELL_SIZE, _cellSize );

    if ( _cullingTechnique.isSet() ) {
        if ( _cullingTechnique == CULL_BY_CROPPING )
            conf.update( PROP_CULLING_TECHNIQUE, "crop" );
        else if ( _cullingTechnique == CULL_BY_CENTROID )
            conf.update( PROP_CULLING_TECHNIQUE, "centroid" );
    }

    conf.updateIfSet( PROP_SPATIALIZE_GROUPS, _spatializeGroups );
    conf.updateIfSet( PROP_CLUSTER_CULLING, _clusterCulling );

    return conf;        
}

void
GriddingPolicy::fromConfig( const Config& conf )
{
    conf.getIfSet( PROP_CELL_SIZE, _cellSize );

    // read the culling technique
    if ( conf.hasValue( PROP_CULLING_TECHNIQUE ) ) {
        if ( conf.value( PROP_CULLING_TECHNIQUE ) == "crop" )
            _cullingTechnique = CULL_BY_CROPPING;
        else if ( conf.value( PROP_CULLING_TECHNIQUE ) == "centroid" )
            _cullingTechnique = CULL_BY_CENTROID;
    }

    // spatial optimization
    conf.getIfSet<bool>( PROP_SPATIALIZE_GROUPS, _spatializeGroups );

    // cluster culling
    conf.getIfSet<bool>( PROP_CLUSTER_CULLING, _clusterCulling );
}

/***************************************************************************/

FeatureGridder::FeatureGridder(const Bounds& inputBounds,
                               const GriddingPolicy& policy ) :
_inputBounds( inputBounds ),
_policy( policy )
{
    if ( _policy.enabled() ) //_policy.cellSize().isSet() && *_policy.cellSize() > 0.0 )
    {
        _cellsX = (int)::ceil(_inputBounds.width() / *_policy.cellSize() );
        _cellsY = (int)::ceil(_inputBounds.height() / *_policy.cellSize() );
    }
    else
    {
        _cellsX = 1;
        _cellsY = 1;
    }

    _cellsX = osg::clampAbove( _cellsX, 1 );
    _cellsY = osg::clampAbove( _cellsY, 1 );

#ifndef OSGEARTH_HAVE_GEOS

    if ( _policy.valid() && _policy->cullingTechnique().isSet() && _policy->cullingTechnique() == GriddingPolicy::CULL_BY_CROPPING )
    {
        OE_WARN 
            << "Warning: Gridding policy 'cull by cropping' requires GEOS. Falling back on 'cull by centroid'." 
            << std::endl;

        _policy->cullingTechnique() = GriddingPolicy::CULL_BY_CENTROID;
    }

#endif // !OSGEARTH_HAVE_GEOS
}

FeatureGridder::~FeatureGridder()
{
    //nop
}

int
FeatureGridder::getNumCells() const
{
    return _cellsX * _cellsY;
}

bool
FeatureGridder::getCellBounds( int i, Bounds& output ) const
{
    if ( i >= 0 && i < (_cellsX*_cellsY) )
    {
        int x = i % _cellsX;
        int y = i / _cellsX;

        double xmin = _inputBounds.xMin() + _policy.cellSize().value()  * (double)x;
        double ymin = _inputBounds.yMin() + _policy.cellSize().value() * (double)y;
        double xmax = osg::clampBelow( _inputBounds.xMin() + *_policy.cellSize() * (double)(x+1), _inputBounds.xMax() );
        double ymax = osg::clampBelow( _inputBounds.yMin() + *_policy.cellSize() * (double)(y+1), _inputBounds.yMax() );

        output = Bounds( xmin, ymin, xmax, ymax );
        return true;
    }
    return false;
}

bool
FeatureGridder::cullFeatureListToCell( int i, FeatureList& features ) const
{
    bool success = true;
    int inCount = features.size();

    Bounds b;
    if ( getCellBounds( i, b ) )
    {
        if ( _policy.cullingTechnique() == GriddingPolicy::CULL_BY_CENTROID )
        {
            for( FeatureList::iterator f_i = features.begin(); f_i != features.end();  )
            {
                bool keepFeature = false;

                Feature* feature = f_i->get();
                Symbology::Geometry* featureGeom = feature->getGeometry();
                if ( featureGeom )
                {
                    osg::Vec3d centroid = featureGeom->getBounds().center();
                    if ( b.contains( centroid.x(), centroid.y() ) )
                    {
                        keepFeature = true;
                    }
                }

                if ( keepFeature )
                    ++f_i;
                else
                    f_i = features.erase( f_i );
            }
        }

        else // CULL_BY_CROPPING (requires GEOS)
        {

#ifdef OSGEARTH_HAVE_GEOS

            // create the intersection polygon:
            osg::ref_ptr<Symbology::Polygon> poly = new Symbology::Polygon( 4 );
            poly->push_back( osg::Vec3d( b.xMin(), b.yMin(), 0 ));
            poly->push_back( osg::Vec3d( b.xMax(), b.yMin(), 0 ));
            poly->push_back( osg::Vec3d( b.xMax(), b.yMax(), 0 ));
            poly->push_back( osg::Vec3d( b.xMin(), b.yMax(), 0 ));

            for( FeatureList::iterator f_i = features.begin(); f_i != features.end();  )
            {
                bool keepFeature = false;

                Feature* feature = f_i->get();
                Symbology::Geometry* featureGeom = feature->getGeometry();
                if ( featureGeom )
                {
                    osg::ref_ptr<Symbology::Geometry> croppedGeometry;
                    if ( featureGeom->crop( poly.get(), croppedGeometry ) )
                    {
                        feature->setGeometry( croppedGeometry.get() );
                        keepFeature = true;
                    }                   
                }

                if ( keepFeature )
                    ++f_i;
                else
                    f_i = features.erase( f_i );
            }  

#endif // OSGEARTH_HAVE_GEOS

        }

    }

    OE_INFO
            << "Grid cell " << i << ": bounds="
            << b.xMin() << "," << b.yMin() << " => " << b.xMax() << "," << b.yMax()
            << "; in=" << inCount << "; out=" << features.size()
            << std::endl;

    return success;
}



//bool
//FeatureGridder::getCell( int i, FeatureList& output ) const
//{
//    bool success = true;
//
//#ifdef OSGEARTH_HAVE_GEOS
//
//    if ( i >= 0 && i < (_cellsX*_cellsY) )
//    {
//        int x = i % _cellsX;
//        int y = i / _cellsX;
//
//        double xmin = _inputBounds.xMin() + _cellWidth  * (double)x;
//        double ymin = _inputBounds.yMin() + _cellHeight * (double)y;
//        double xmax = osg::clampBelow( _inputBounds.xMin() + _cellWidth  * (double)(x+1), _inputBounds.xMax() );
//        double ymax = osg::clampBelow( _inputBounds.yMin() + _cellHeight * (double)(y+1), _inputBounds.yMax() );
//
//        Bounds cx( xmin, ymin, xmax, ymax );
//
//        geom::GeometryFactory* f = new geom::GeometryFactory();
//
//        // create the intersection polygon:
//        osg::ref_ptr<Polygon> poly = new Polygon( 4 );
//        poly->push_back( osg::Vec3d( xmin, ymin, 0 ) );
//        poly->push_back( osg::Vec3d( xmax, ymin, 0 ) );
//        poly->push_back( osg::Vec3d( xmax, ymax, 0 ) );
//        poly->push_back( osg::Vec3d( xmin, ymax, 0 ) );
//        geom::Geometry* cropGeom = GEOSUtils::importGeometry( poly.get() );
//
//        // intersect it with each feature:
//        int count =0;
//        FeatureList::const_iterator f_i = _input.begin();
//        std::list<void*>::const_iterator g_i = _geosGeoms.begin();
//        for( ; f_i != _input.end(); ++f_i, ++g_i )
//        {
//            Feature* feature = f_i->get();
//            geom::Geometry* inGeom = static_cast<geom::Geometry*>( *g_i );
//            Geometry* featureGeom = 0L;
//            if ( inGeom )
//            {            
//                geom::Geometry* outGeom = 0L;
//                try {
//                    outGeom = overlay::OverlayOp::overlayOp(
//                        inGeom, cropGeom,
//                        overlay::OverlayOp::opINTERSECTION );
//                }
//                catch( ... ) {
//                    outGeom = 0L;
//                    OE_NOTICE << "Feature gridder, GEOS overlay op exception, skipping feature" << std::endl;
//                }
//                    
//                if ( outGeom )
//                {
//                    featureGeom = GEOSUtils::exportGeometry( outGeom );
//                    f->destroyGeometry( outGeom );
//                    if ( featureGeom && featureGeom->isValid() )
//                    {
//                        Feature* newFeature = osg::clone<Feature>( feature, osg::CopyOp::DEEP_COPY_ALL );
//                        newFeature->setGeometry( featureGeom );
//                        output.push_back( newFeature );
//                        count++;
//                    }
//                }
//            }
//        }
//
//        // clean up
//        f->destroyGeometry( cropGeom );
//        delete f;
//
//        OE_NOTICE
//            << "Grid cell " << i << ": " << count << " features; bounds="
//            << xmin << "," << ymin << " => " << xmax << "," << ymax
//            << std::endl;
//    }
//
//#else // OSGEARTH_HAVE_GEOS
//
//    output = _input;
//
//#endif // OSGEARTH_HAVE_GEOS
//
//    return success;
//}
//
