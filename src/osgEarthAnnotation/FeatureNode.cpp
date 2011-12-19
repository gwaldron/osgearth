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

#include <osgEarthAnnotation/FeatureNode>
#include <osgEarthFeatures/GeometryCompiler>
#include <osgEarth/DrapeableNode>
#include <osgEarth/FindNode>
#include <osgEarth/Utils>

using namespace osgEarth;
using namespace osgEarth::Annotation;
using namespace osgEarth::Features;

FeatureNode::FeatureNode(MapNode* mapNode, 
                         Feature* feature, 
                         bool     draped,
                         const GeometryCompilerOptions& options ) :
AnnotationNode(),
_mapNode( mapNode ),
_feature( feature ),
_draped ( draped ),
_options( options )
{
    init();
}

void
FeatureNode::init()
{
    // if there's a draw state, clear it out first.
    this->clearAltDrawState();

    // if there is existing geometry, kill it
    this->removeChildren( 0, this->getNumChildren() );

    // build the new feature geometry
    if ( _feature.valid() && _feature->getGeometry() && _mapNode.valid() )
    {
        GeometryCompiler compiler( _options );
        Session* session = new Session( _mapNode->getMap() );
        GeoExtent extent(_mapNode->getMap()->getProfile()->getSRS(), _feature->getGeometry()->getBounds());
        osg::ref_ptr<FeatureProfile> profile = new FeatureProfile( extent );
        FilterContext context( session, profile.get(), extent );

        // Clone the Feature before rendering as the GeometryCompiler and it's filters can change the coordinates
        // of the geometry when performing localization or converting to geocentric.
        osg::ref_ptr< Feature > clone = new osgEarth::Features::Feature(*_feature.get(), osg::CopyOp::DEEP_COPY_ALL);        

        osg::Node* node = compiler.compile( clone.get(), *clone->style(), context );
        if ( node )
        {
            if ( _draped )
            {
                DrapeableNode* d = new DrapeableNode();
                d->setNode( node );
            }
            else
            {
                this->addChild( node );
            }
        }
    }
}

void
FeatureNode::setFeature( Feature* feature )
{
    _feature = feature;
    init();
}
