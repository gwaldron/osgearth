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
#include <osgEarthFeatures/FeatureNode>
#include <osgEarthFeatures/Session>

#define LC "[FeatureNode] "

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;

FeatureNode::FeatureNode( MapNode* mapNode, Feature* feature, bool draped, const GeometryCompilerOptions& options ) :
DrapeableNode( mapNode, draped ),
_options     ( options ),
_feature     ( feature )
{
    init();
}

void
FeatureNode::init()
{
    if ( _feature.valid() && _feature->getGeometry() )
    {
        GeometryCompiler compiler( _options );
        Session* session = new Session( _mapNode->getMap() );
        GeoExtent extent(_mapNode->getMap()->getProfile()->getSRS(), _feature->getGeometry()->getBounds());
        FeatureProfile* profile = new FeatureProfile(extent);
        FilterContext context( session, profile, extent );

        // Clone the Feature before rendering as the GeometryCompiler and it's filters can change the coordinates
        // of the geometry when performing localization or converting to geocentric.
        osg::ref_ptr< Feature > clone = new osgEarth::Features::Feature(*_feature.get(), osg::CopyOp::DEEP_COPY_ALL);        

        osg::Node* node;
        if ( _style.isSet() )
            node = compiler.compile( clone.get(), *_style, context );
        else
            node = compiler.compile( clone.get(), *clone->style(), context );

        setNode( node );
    }
}

void
FeatureNode::setFeature( Feature* feature )
{
    _feature = feature;
    init();
}

void
FeatureNode::setStyle( const Style& style )
{
    _style = style;
    init();
}
