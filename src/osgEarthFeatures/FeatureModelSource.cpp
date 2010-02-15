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
#include <osgEarthFeatures/FeatureModelSource>
#include <osgEarthFeatures/FeatureGridder>
#include <osgEarthFeatures/Styling>
#include <osgEarth/SpatialReference>
#include <osg/Notify>
#include <osg/Timer>
#include <osg/LOD>
#include <osg/ClusterCullingCallback>
#include <osgUtil/Optimizer>

using namespace osgEarth;
using namespace osgEarth::Features;

#define PROP_FEATURES      "features"
#define PROP_GEOMETRY_TYPE "geometry_type"
#define PROP_LIGHTING      "lighting"
#define PROP_GRIDDING      "gridding"


FeatureModelSource::FeatureModelSource( const PluginOptions* options ) :
ModelSource( options ),
_geomTypeOverride( Geometry::TYPE_UNKNOWN ),
_lit( false ),
_gridding( GriddingPolicy() )
{
    const Config& conf = options->config();

    // the data source from which to pull features:
    _features = FeatureSourceFactory::create( conf.child( PROP_FEATURES ) );
    if ( !_features.valid() )
    {
        osg::notify( osg::WARN ) << "[osgEarth] FeatureModelSource - no valid feature source provided" << std::endl;
    }

    // force a particular geometry type
    if ( conf.hasValue( PROP_GEOMETRY_TYPE ) )
    {
        // geometry type override: the config can ask that input geometry
        // be interpreted as a particular geometry type
        std::string gt = conf.value( PROP_GEOMETRY_TYPE );
        if ( gt == "line" || gt == "lines" || gt == "linestrip" )
            _geomTypeOverride = Geometry::TYPE_LINESTRING;
        else if ( gt == "point" || gt == "points" || gt == "pointset" )
            _geomTypeOverride = Geometry::TYPE_POINTSET;
        else if ( gt == "polygon" || gt == "polygons" )
            _geomTypeOverride = Geometry::TYPE_POLYGON;
    }

    // gridding policy
    if ( conf.hasChild( PROP_GRIDDING ) )
    {
        _gridding = GriddingPolicy( conf.child( PROP_GRIDDING ) );
    }

    // lighting
    if ( conf.hasValue( PROP_LIGHTING ) )
    {
        if ( conf.value( PROP_LIGHTING ) == "true" )
            _lit = true;
        else if ( conf.value( PROP_LIGHTING ) == "false" )
            _lit = false;
        else if ( conf.value( PROP_LIGHTING ) == "default" )
            _lit.unset();
    }
    else
    {
        _lit = false;
    }

    // load up the style catalog.
    StyleReader::readLayerStyles( this->getName(), conf, _styleCatalog );
}

void 
FeatureModelSource::initialize( const std::string& referenceURI, const osgEarth::Map* map )
{
    ModelSource::initialize( referenceURI, map );

    if ( _features.valid() )
        _features->initialize( referenceURI );

    _map = map;
}

osg::Node*
FeatureModelSource::createNode( ProgressCallback* progress )
{
    if ( !_features.valid() || !_features->getFeatureProfile() )
        return 0L;

    osg::Timer_t start = osg::Timer::instance()->tick();

    // implementation-specific data
    osg::ref_ptr<osg::Referenced> buildData = createBuildData();

    osg::Group* group = new osg::Group();

    // figure out which rule to use to style the geometry.
    //osg::notify(osg::NOTICE) << "checking for style layer named '" << this->getName() << "'" << std::endl;
    StyledLayer layer;
    bool hasStyledLayer = _styleCatalog.getNamedLayer( this->getName(), layer );

    if ( hasStyledLayer )
    {
        //osg::notify(osg::NOTICE) << "Styled layer def found" << std::endl;

        // The catalog contains style data for this source, so use it:
        for( StyleList::iterator i = layer.styles().begin(); i != layer.styles().end(); ++i )
        {
            const Style& style = *i;
            osg::Node* node = gridAndRenderFeaturesForStyle( style, buildData.get() );
            if ( node )
                group->addChild( node );
        }
    }
    else if ( _features->hasEmbeddedStyles() )
    {
        //osg::notify(osg::NOTICE) << "Using embedded style info" << std::endl;

        // Each feature has its own embedded style data, so use that:
        osg::ref_ptr<FeatureCursor> cursor = _features->createFeatureCursor( Query() );
        while( cursor->hasMore() )
        {
            Feature* feature = cursor->nextFeature();
            if ( feature )
            {
                FeatureList list;
                list.push_back( feature );
                // gridding is not supported for embedded styles.
                osg::Node* node = renderFeaturesForStyle( feature->style().get(), list, buildData.get() );
                if ( node )
                    group->addChild( node );
            }
        }
    }
    else
    {
        osg::notify(osg::NOTICE) << "[osgEarth] " << getName() << ": no styles found for '" << this->getName() << "'" << std::endl;

        // There is no style data, so use the default.
        osg::Node* node = gridAndRenderFeaturesForStyle( Style(), buildData.get() );
        if ( node )
            group->addChild( node );
    }

    // run the SpatializeGroups optimization pass on the result
    if ( _gridding.isSet() && _gridding->spatializeGroups() == true )
    {
        osg::notify(osg::NOTICE) << "[osgEarth] " << getName() << ": running spatial optimization" << std::endl;
        osgUtil::Optimizer optimizer;
        optimizer.optimize( group, osgUtil::Optimizer::SPATIALIZE_GROUPS );
    }

    // apply explicit lighting if necessary:
    if ( _lit.isSet() )
    {
        osg::StateSet* ss = group->getOrCreateStateSet();
        ss->setMode( GL_LIGHTING, _lit == true?
            osg::StateAttribute::ON | osg::StateAttribute::PROTECTED :
            osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED );
    }

    osg::Timer_t end = osg::Timer::instance()->tick();

    osg::notify(osg::NOTICE) << "[osgEarth] layer " << getName() << ", time to compile = " << 
        osg::Timer::instance()->delta_s( start, end ) << "s" << std::endl;

    return group;
}


osg::Group*
FeatureModelSource::gridAndRenderFeaturesForStyle(const Style& style,
                                                  osg::Referenced* data )
{
    osg::Group* styleGroup = 0L;

    // first we need the overall extent of the layer:
    const GeoExtent& extent = getFeatureSource()->getFeatureProfile()->getExtent();

    // next set up a gridder/cropper:
    FeatureGridder gridder( extent.bounds(), _gridding.get() );

    if ( gridder.getNumCells() > 1 )
    {
        osg::notify(osg::NOTICE)
            << "[osgEarth] " << getName() << ": grid cells = " << gridder.getNumCells()
            << std::endl;
    }

    // now query the feature source once for each grid cell extent:
    for( int cell=0; cell<gridder.getNumCells(); ++cell )
    {
        Bounds cellBounds;
        if ( gridder.getCellBounds( cell, cellBounds ) )
        {
            // incorporate the cell bounds into the query:
            Query query = style.query().value();
            query.bounds() = query.bounds().isSet()?
                query.bounds()->unionWith( cellBounds ) :
                cellBounds;

            // query the feature source:
            osg::ref_ptr<FeatureCursor> cursor = _features->createFeatureCursor( query );

            // now copy the resulting feature set into a list, converting the data
            // types along the way if a geometry override is in place:
            FeatureList cellFeatures;
            while( cursor->hasMore() )
            {
                Feature* feature = cursor->nextFeature();
                Geometry* geom = feature->getGeometry();
                if ( geom )
                {
                    // apply a type override if requested:
                    if ( _geomTypeOverride.isSet() && _geomTypeOverride.get() != geom->getComponentType() )
                    {
                        geom = geom->cloneAs( _geomTypeOverride.get() );
                        if ( geom )
                            feature->setGeometry( geom );
                    }
                }
                if ( geom )
                {
                    cellFeatures.push_back( feature );
                }
            }

            // cut the features so they fall completely within the cell. Note, we only need to 
            // do this is gridding is enabled.
            if ( gridder.getNumCells() > 1 )
            {
                gridder.cullFeatureListToCell( cell, cellFeatures );
            }

            if ( cellFeatures.size() > 0 )
            {
                // next ask the implementation to construct OSG geometry for the cell features.
                // note: just b/c render* returns NULL does not mean it didn't generate anything.
                // Some implementations might return a group node on the first pass and add children
                // to it on subsequent passes.
                osg::Node* createdNode = 0L;
                osg::Node* nodeToAdd = renderFeaturesForStyle( style, cellFeatures, data, &createdNode );
                if ( nodeToAdd )
                {
                    if ( !styleGroup )
                        styleGroup = new osg::Group();

                    styleGroup->addChild( nodeToAdd );
                }

                // if the method created a node, apply a cluter culler to it if neceesary:
                if ( createdNode )
                {
                    if ( _map->isGeocentric() && _gridding->clusterCulling() == true )
                    {
                        const SpatialReference* mapSRS = _map->getProfile()->getSRS()->getGeographicSRS();
                        GeoExtent cellExtent( extent.getSRS(), cellBounds );
                        GeoExtent mapCellExtent = cellExtent.transform( mapSRS );

                        double cx, cy;
                        mapCellExtent.getCentroid( cx, cy );

                        osg::Vec3d geoc_center;
                        mapSRS->getEllipsoid()->convertLatLongHeightToXYZ(
                            osg::DegreesToRadians( cy ), osg::DegreesToRadians( cx ), 0,
                            geoc_center.x(), geoc_center.y(), geoc_center.z() );

                        osg::Vec3d geoc_corner;
                        mapSRS->getEllipsoid()->convertLatLongHeightToXYZ(
                            osg::DegreesToRadians( mapCellExtent.yMin() ), osg::DegreesToRadians( mapCellExtent.xMin()), 0,
                            geoc_corner.x(), geoc_corner.y(), geoc_corner.z() );

                        osg::Vec3d normal = mapSRS->getEllipsoid()->computeLocalUpVector(
                            geoc_center.x(), geoc_center.y(), geoc_center.z() );

                        // the "deviation" determines how far below the tangent plane of the cell your
                        // camera has to be before culling occurs. 0.0 is at the plane; -1.0 is 90deg
                        // below the plane (which means never cull).
                        osg::Vec3d radialVector = geoc_corner - geoc_center;
                        double radius = radialVector.length();
                        radialVector.normalize();
                        double minDotProduct = radialVector * normal;                     

                        osg::ClusterCullingCallback* ccc = new osg::ClusterCullingCallback();
                        ccc->set( geoc_center, normal, minDotProduct, radius );

                        createdNode->setCullCallback( ccc );

                        osg::notify(osg::NOTICE)
                            << "[osgEarth] Cell: " << mapCellExtent.toString()
                            << ": centroid = " << cx << "," << cy
                            << "; normal = " << normal.x() << "," << normal.y() << "," << normal.z()
                            << "; dev = " << minDotProduct
                            << "; radius = " << radius
                            << std::endl;
                    }
                }
            }
        }
    }

    return styleGroup;
}
