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
#include <osg/Notify>

using namespace osgEarth;
using namespace osgEarth::Features;

#define PROP_FEATURES      "features"
#define PROP_GEOMETRY_TYPE "geometry_type"
#define PROP_LIGHTING      "lighting"

FeatureModelSource::FeatureModelSource( const PluginOptions* options ) :
ModelSource( options ),
_geomTypeOverride( Geometry::TYPE_UNKNOWN ),
_lit( false )
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

            osg::ref_ptr<FeatureCursor> cursor = _features->createCursor( style.query() );
            osg::Node* node = renderStyle( style, cursor.get(), buildData.get() );
            if ( node )
                group->addChild( node );
        }
    }
    else if ( _features->hasEmbeddedStyles() )
    {
        //osg::notify(osg::NOTICE) << "Using embedded style info" << std::endl;

        // Each feature has its own embedded style data, so use that:
        osg::ref_ptr<FeatureCursor> cursor = _features->createCursor( Query() );
        while( cursor->hasMore() )
        {
            Feature* feature = cursor->nextFeature();
            if ( feature )
            {
                FeatureList list;
                list.push_back( feature );
                osg::ref_ptr<FeatureCursor> tempCursor = new FeatureListCursor( list );
                osg::Node* node = renderStyle( feature->style().get(), tempCursor.get(), buildData.get() );
                if ( node )
                    group->addChild( node );
            }
        }
    }
    else
    {
        osg::notify(osg::NOTICE) << "[osgEarth] FeatureModelSource: No styles found for '" << this->getName() << "'" << std::endl;

        // There is no style data, so use the default.
        osg::ref_ptr<FeatureCursor> cursor = _features->createCursor( Query() );
        osg::Node* node = renderStyle( Style(), cursor.get(), buildData.get() );
        if ( node )
            group->addChild( node );
    }

    if ( _lit.isSet() )
    {
        osg::StateSet* ss = group->getOrCreateStateSet();
        ss->setMode( GL_LIGHTING, _lit == true?
            osg::StateAttribute::ON | osg::StateAttribute::PROTECTED :
            osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED );
    }

    return group;
}
