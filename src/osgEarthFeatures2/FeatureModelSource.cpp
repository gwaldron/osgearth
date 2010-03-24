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
#include <osgEarthFeatures2/FeatureModelSource>
#include <osgEarthFeatures2/FeatureGridder>
#include <osgEarth/SpatialReference>
#include <osg/Notify>
#include <osg/Timer>
#include <osg/LOD>
#include <osg/ClusterCullingCallback>
#include <osgUtil/Optimizer>

using namespace osgEarth;
using namespace osgEarth::Features2;
using namespace osgEarth::Symbology;

/****************************************************************/

FeatureModelSourceOptions::FeatureModelSourceOptions( const PluginOptions* opt ) :
ModelSourceOptions( opt ),
_geomTypeOverride( Geometry::TYPE_UNKNOWN ),
_lit( true )
{
    if ( config().hasChild("features") )
        _featureOptions = new FeatureSourceOptions( new PluginOptions( config().child("features") ) );

    config().getObjIfSet( "styles", _styles );
    config().getObjIfSet( "gridding", _gridding );
    config().getIfSet( "lighting", _lit );

    std::string gt = config().value( "geometry_type" );
    if ( gt == "line" || gt == "lines" || gt == "linestring" )
        _geomTypeOverride = Geometry::TYPE_LINESTRING;
    else if ( gt == "point" || gt == "pointset" || gt == "points" )
        _geomTypeOverride = Geometry::TYPE_POINTSET;
    else if ( gt == "polygon" || gt == "polygons" )
        _geomTypeOverride = Geometry::TYPE_POLYGON;
    
    // load up the style catalog.
    //StyleReader::readLayerStyles( name(), config(), _styles );
}

Config
FeatureModelSourceOptions::toConfig() const
{
    Config conf = ModelSourceOptions::toConfig();

    conf.updateObjIfSet( "features", _featureOptions );
    conf.updateObjIfSet( "gridding", _gridding );
    conf.updateObjIfSet( "styles", _styles );
    conf.updateIfSet( "lighting", _lit );

    if ( _geomTypeOverride.isSet() ) {
        if ( _geomTypeOverride == Geometry::TYPE_LINESTRING )
            conf.update( "geometry_type", "line" );
        else if ( _geomTypeOverride == Geometry::TYPE_POINTSET )
            conf.update( "geometry_type", "point" );
        else if ( _geomTypeOverride == Geometry::TYPE_POLYGON )
            conf.update( "geometry_type", "polygon" );
    }

    return conf;
}

/****************************************************************/

FeatureModelSource::FeatureModelSource( const PluginOptions* options ) :
ModelSource( options )
{
    _options = dynamic_cast<const FeatureModelSourceOptions*>( options );
    if ( !_options )
        _options = new FeatureModelSourceOptions( options );

    // the data source from which to pull features:
    _features = FeatureSourceFactory::create( _options->featureOptions().get() );
    if ( !_features.valid() )
    {
        OE_WARN << "FeatureModelSource - no valid feature source provided" << std::endl;
    }
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

    const optional<StyleCatalog>& styles = _options->styles();
    //const StyleCatalog* styles = _options->styles().get();
    
    // figure out if and how to style the geometry.
    if ( _features->hasEmbeddedStyles() )
    {
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
    else if ( styles.isSet() )
    {
        if ( styles->selectors().size() > 0 )
        {
            for( StyleSelectorList::const_iterator i = styles->selectors().begin(); i != styles->selectors().end(); ++i )
            {
                const StyleSelector& sel = *i;
                Style* style;
                styles->getStyle( sel.getSelectedStyleName(), style );
                osg::Node* node = gridAndRenderFeaturesForStyle( style, sel.query().value(), buildData.get() );
                if ( node )
                    group->addChild( node );
            }
        }
        else
        {
            const Style* style = styles->getDefaultStyle();
            osg::Node* node = gridAndRenderFeaturesForStyle( style, Query(), buildData.get() );
            if ( node )
                group->addChild( node );
        }
    }
    else
    {
        osg::Node* node = gridAndRenderFeaturesForStyle( new Style, Query(), buildData.get() );
        if ( node )
            group->addChild( node );
    }

    // run the SpatializeGroups optimization pass on the result
    if ( _options->gridding().valid() && _options->gridding()->spatializeGroups() == true )
    {
        OE_NOTICE << getName() << ": running spatial optimization" << std::endl;
        osgUtil::Optimizer optimizer;
        optimizer.optimize( group, osgUtil::Optimizer::SPATIALIZE_GROUPS );
    }

    // apply explicit lighting if necessary:
    if ( _options->enableLighting().isSet() )
    {
        osg::StateSet* ss = group->getOrCreateStateSet();
        ss->setMode( GL_LIGHTING, _options->enableLighting() == true?
            osg::StateAttribute::ON | osg::StateAttribute::PROTECTED :
            osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED );
    }

    osg::Timer_t end = osg::Timer::instance()->tick();

    OE_INFO << "Layer " << getName() << ", time to compile styles = " << 
        osg::Timer::instance()->delta_s( start, end ) << "s" << std::endl;

    return group;
}


osg::Group*
FeatureModelSource::gridAndRenderFeaturesForStyle(const Style* style,
                                                  const Query& query,
                                                  osg::Referenced* data )
{
    osg::Group* styleGroup = 0L;

    // first we need the overall extent of the layer:
    const GeoExtent& extent = getFeatureSource()->getFeatureProfile()->getExtent();

    osg::ref_ptr<GriddingPolicy> gridding = 
        _options->gridding().valid() ? _options->gridding().get() :
        new GriddingPolicy();

    // next set up a gridder/cropper:
    FeatureGridder gridder( extent.bounds(), gridding );

    if ( gridder.getNumCells() > 1 )
    {
        OE_NOTICE
            << getName() << ": grid cells = " << gridder.getNumCells()
            << std::endl;
    }

    // now query the feature source once for each grid cell extent:
    for( int cell=0; cell<gridder.getNumCells(); ++cell )
    {
        Bounds cellBounds;
        if ( gridder.getCellBounds( cell, cellBounds ) )
        {
            // incorporate the cell bounds into the query:
            Query localQuery = query;
            localQuery.bounds() = query.bounds().isSet()?
                query.bounds()->unionWith( cellBounds ) :
                cellBounds;

            // query the feature source:
            osg::ref_ptr<FeatureCursor> cursor = _features->createFeatureCursor( localQuery );

            // now copy the resulting feature set into a list, converting the data
            // types along the way if a geometry override is in place:
            FeatureList cellFeatures2;
            while( cursor->hasMore() )
            {
                Feature* feature = cursor->nextFeature();
                Geometry* geom = feature->getGeometry();
                if ( geom )
                {
                    // apply a type override if requested:
                    if ( _options->geometryTypeOverride().isSet() && _options->geometryTypeOverride() != geom->getComponentType() )
                    {
                        geom = geom->cloneAs( _options->geometryTypeOverride().value() );
                        if ( geom )
                            feature->setGeometry( geom );
                    }
                }
                if ( geom )
                {
                    cellFeatures2.push_back( feature );
                }
            }

            // cut the features so they fall completely within the cell. Note, we only need to 
            // do this is gridding is enabled.
            if ( gridder.getNumCells() > 1 )
            {
                gridder.cullFeatureListToCell( cell, cellFeatures2 );
            }

            if ( cellFeatures2.size() > 0 )
            {
                // next ask the implementation to construct OSG geometry for the cell features.
                // note: just b/c render* returns NULL does not mean it didn't generate anything.
                // Some implementations might return a group node on the first pass and add children
                // to it on subsequent passes.
                osg::Node* createdNode = 0L;
                osg::Node* nodeToAdd = renderFeaturesForStyle( style, cellFeatures2, data, &createdNode );
                if ( nodeToAdd )
                {
                    if ( !styleGroup )
                        styleGroup = new osg::Group();

                    styleGroup->addChild( nodeToAdd );
                }

                // if the method created a node, apply a cluter culler to it if neceesary:
                if ( createdNode )
                {
                    if ( _map->isGeocentric() && gridding->clusterCulling() == true )
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

                        OE_NOTICE
                            << "Cell: " << mapCellExtent.toString()
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
