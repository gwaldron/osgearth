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
#include <osgEarthAnnotation/AnnotationRegistry>
#include <osgEarthAnnotation/AnnotationUtils>

#include <osgEarthFeatures/GeometryCompiler>
#include <osgEarthFeatures/GeometryUtils>
#include <osgEarthFeatures/MeshClamper>

#include <osgEarthSymbology/AltitudeSymbol>

#include <osgEarth/DrapeableNode>
#include <osgEarth/NodeUtils>
#include <osgEarth/Utils>
#include <osgEarth/Registry>

#include <osg/BoundingSphere>
#include <osg/Polytope>
#include <osg/Transform>

#define LC "[FeatureNode] "

using namespace osgEarth;
using namespace osgEarth::Annotation;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;


FeatureNode::FeatureNode(MapNode* mapNode, 
                         Feature* feature, 
                         bool     draped,
                         const GeometryCompilerOptions& options ) :
AnnotationNode( mapNode ),
_feature( feature ),
_draped ( draped ),
_options( options )
{
    init();
}

void
FeatureNode::init()
{
    // if there's a decoration, clear it out first.
    this->clearDecoration();
    _attachPoint = 0L;

    // if there is existing geometry, kill it
    this->removeChildren( 0, this->getNumChildren() );

    // build the new feature geometry
    {
        GeometryCompilerOptions options = _options;
        
        // have to disable compiler clamping if we're doing auto-clamping; especially
        // in terrain-relative mode because the auto-clamper will think the clamped
        // coords are the relative coords.
        bool autoClamping = supportsAutoClamping(*_feature->style());
        if ( autoClamping )
        {
            options.ignoreAltitudeSymbol() = true;
        }

        // prep the compiler:
        GeometryCompiler compiler( options );
        Session* session = new Session( _mapNode->getMap() );
        GeoExtent extent(_feature->getSRS(), _feature->getGeometry()->getBounds());
        osg::ref_ptr<FeatureProfile> profile = new FeatureProfile( extent );
        FilterContext context( session, profile.get(), extent );

        // Clone the Feature before rendering as the GeometryCompiler and it's filters can change the coordinates
        // of the geometry when performing localization or converting to geocentric.
        osg::ref_ptr< Feature > clone = new Feature(*_feature.get(), osg::CopyOp::DEEP_COPY_ALL);        

        osg::Node* node = compiler.compile( clone.get(), *clone->style(), context );
        if ( node )
        {
            if (_style.isSet() && 
                AnnotationUtils::styleRequiresAlphaBlending(*_style) &&
                _style->get<ExtrusionSymbol>() )
            {
                node = AnnotationUtils::installTwoPassAlpha( node );
            }

            _attachPoint = new osg::Group();
            _attachPoint->addChild( node );

            if ( _draped )
            {
                DrapeableNode* d = new DrapeableNode(_mapNode.get());
                d->addChild( _attachPoint );
                this->addChild( d );
            }
            else
            {
                this->addChild( _attachPoint );
            }
        }

        // workaround until we can auto-clamp extruded/sub'd geometries.
        if ( autoClamping )
        {
            applyStyle( *_feature->style(), _draped );
            clampMesh( _mapNode->getTerrain()->getGraph() );
        }
    }
}

void
FeatureNode::setFeature( Feature* feature )
{
    _feature = feature;
    init();
}

osg::Group*
FeatureNode::getAttachPoint()
{
    if ( !_attachPoint )
        return 0L;

    // first try to find a transform to go under:
    osg::Group* xform = osgEarth::findTopMostNodeOfType<osg::Transform>(_attachPoint);
    if ( xform )
        return xform;

    // failing that, use the artificial attach group we created.
    return _attachPoint;
}

void
FeatureNode::reclamp( const TileKey& key, osg::Node* tile, const Terrain* )
{
    osg::Polytope p = _feature->getWorldBoundingPolytope();
    if ( p.contains( tile->getBound() ) )
    {
        clampMesh( tile );
    }
}

void
FeatureNode::clampMesh( osg::Node* terrainModel )
{
    double scale  = 1.0;
    double offset = 0.0;
    bool   relative = false;

    if (_altitude.valid())
    {
        NumericExpression scale(_altitude->verticalScale().value());
        NumericExpression offset(_altitude->verticalOffset().value());
        scale = _feature->eval(scale);
        offset = _feature->eval(offset);
        relative = _altitude->clamping() == AltitudeSymbol::CLAMP_RELATIVE_TO_TERRAIN;
    }

    MeshClamper clamper( terrainModel, _mapNode->getMapSRS(), _mapNode->isGeocentric(), relative, scale, offset );
    this->accept( clamper );

    this->dirtyBound();
}


//-------------------------------------------------------------------

OSGEARTH_REGISTER_ANNOTATION( feature, osgEarth::Annotation::FeatureNode );


FeatureNode::FeatureNode(MapNode*      mapNode,
                         const Config& conf) :
AnnotationNode( mapNode )
{
    osg::ref_ptr<const SpatialReference> srs;
    osg::ref_ptr<Geometry> geom;

    if ( conf.hasChild("geometry") )
    {
        Config geomconf = conf.child("geometry");
        srs = SpatialReference::create( geomconf.value("srs"), geomconf.value("vdatum") );
        geom = GeometryUtils::geometryFromWKT( geomconf.value() );
    }

    conf.getObjIfSet( "style", _style );

    if ( srs.valid() && geom.valid() )
    {
        _draped = conf.value<bool>("draped",false);
        setFeature( new Feature(geom.get(), srs.get(), *_style) );
    }
}

Config
FeatureNode::getConfig() const
{
    Config conf("feature");

    if ( _feature.valid() && _feature->getGeometry() )
    {
        std::string wkt = GeometryUtils::geometryToWKT( _feature->getGeometry() );
        std::string srs = _feature->getSRS() ? _feature->getSRS()->getHorizInitString() : "";
        std::string vsrs = _feature->getSRS() ? _feature->getSRS()->getVertInitString() : "";

        Config geomConf("geometry");
        geomConf.value() = wkt;
        if ( !srs.empty() ) geomConf.set("srs", srs);
        if ( !vsrs.empty() ) geomConf.set("vdatum", vsrs);
        conf.add(geomConf);

        conf.addObjIfSet( "style", _style );
    }

    return conf;
}
