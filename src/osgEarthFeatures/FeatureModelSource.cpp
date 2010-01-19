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
#include <osg/Notify>
#include <osg/Timer>
#include <osg/LOD>
#include <osgUtil/Optimizer>

using namespace osgEarth;
using namespace osgEarth::Features;

#define PROP_CELL_SIZE         "cell_size"
#define PROP_CULLING_TECHNIQUE "culling_technique"
#define PROP_SPATIALIZE_GROUPS "spatialize_groups"

GriddingPolicy::GriddingPolicy() :
_cellSize( DBL_MAX ),
_cullingTechnique( GriddingPolicy::CULL_BY_CENTROID ),
_spatializeGroups( true )
{
    //nop
}

GriddingPolicy::GriddingPolicy( const Config& conf ) :
_cellSize( DBL_MAX ),
_cullingTechnique( GriddingPolicy::CULL_BY_CENTROID ),
_spatializeGroups( true )
{
    // read the cell size
    if ( conf.hasValue( PROP_CELL_SIZE ) )
        _cellSize = conf.value<double>( PROP_CELL_SIZE, _cellSize.defaultValue() );

    // read the culling technique
    if ( conf.value(PROP_CULLING_TECHNIQUE) == "crop" )
        _cullingTechnique = CULL_BY_CROPPING;
    else if ( conf.value(PROP_CULLING_TECHNIQUE) == "centroid" )
        _cullingTechnique = CULL_BY_CENTROID;

    // spatial optimization
    if ( conf.value(PROP_SPATIALIZE_GROUPS) == "true" )
        _spatializeGroups = true;
    else if ( conf.value(PROP_SPATIALIZE_GROUPS) == "false" )
        _spatializeGroups = false;
}

Config
GriddingPolicy::toConfig() const 
{
    Config conf;
    if ( _cellSize.isSet() )
        conf.add( PROP_CELL_SIZE, toString(_cellSize.value()) );
    if ( _cullingTechnique.isSet() ) {
        if ( _cullingTechnique == CULL_BY_CROPPING )
            conf.add( PROP_CULLING_TECHNIQUE, "crop" );
        else if ( _cullingTechnique == CULL_BY_CENTROID )
            conf.add( PROP_CULLING_TECHNIQUE, "centroid" );
    }
    if ( _spatializeGroups.isSet() ) {
        conf.add( PROP_SPATIALIZE_GROUPS, toString(_spatializeGroups.value()) );
    }
    return conf;        
}

/***************************************************************************/

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

osg::Node*
FeatureModelSource::createNode( ProgressCallback* progress )
{
    if ( !_features.valid() )
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


osg::Node*
FeatureModelSource::gridAndRenderFeaturesForStyle(const Style& style,
                                                  osg::Referenced* data )
{
    osg::Group* styleGroup = 0L;

    // first we need the overall extent of the layer:
    const GeoExtent& extent = getFeatureSource()->getFeatureProfile()->getExtent();

    // next set up a gridder/cropper:
    FeatureGridder gridder( extent.bounds(), _gridding.get() );

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
                // next ask the implementation to construct OSG geometry for the cell features:
                osg::Node* styleNode = renderFeaturesForStyle( style, cellFeatures, data );
                if ( styleNode )
                {
                    if ( !styleGroup )
                        styleGroup = new osg::Group();

                    styleGroup->addChild( styleNode );
                }
            }
        }
    }

    return styleGroup;
}
