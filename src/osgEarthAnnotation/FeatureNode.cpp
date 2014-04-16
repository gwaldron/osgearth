/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2013 Pelican Mapping
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

#include <osgEarth/ClampableNode>
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
_feature      ( feature ),
_draped       ( draped ),
_options      ( options )
{
    init();
}

FeatureNode::FeatureNode(MapNode* mapNode,
                         Feature* feature,
                         const GeometryCompilerOptions& options ) :
AnnotationNode( mapNode ),
_feature      ( feature ),
_draped       ( false ),
_options      ( options )
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

    if ( !getMapNode() )
        return;

    if ( !_feature.valid() )
        return;

    // compilation options.
    GeometryCompilerOptions options = _options;
    
    // figure out what kind of altitude manipulation we need to perform.
    AnnotationUtils::AltitudePolicy ap;
    AnnotationUtils::getAltitudePolicy( *_feature->style(), ap );

    // If we're doing auto-clamping on the CPU, shut off compiler map clamping
    // clamping since it would be redundant.
    // TODO: I think this is OBE now that we have "scene" clamping technique..
    if ( ap.sceneClamping )
    {
        options.ignoreAltitudeSymbol() = true;
    }

    // prep the compiler:
    GeometryCompiler compiler( options );
    Session* session = new Session( getMapNode()->getMap() );
    GeoExtent extent(_feature->getSRS(), _feature->getGeometry()->getBounds());
    osg::ref_ptr<FeatureProfile> profile = new FeatureProfile( extent );
    FilterContext context( session, profile.get(), extent );

    // Clone the Feature before rendering as the GeometryCompiler and it's filters can change the coordinates
    // of the geometry when performing localization or converting to geocentric.
    osg::ref_ptr< Feature > clone = new Feature(*_feature.get(), osg::CopyOp::DEEP_COPY_ALL);

    osg::Node* node = compiler.compile( clone.get(), *clone->style(), context );
    if ( node )
    {
        if ( _feature->style().isSet() &&
            AnnotationUtils::styleRequiresAlphaBlending( *_feature->style() ) &&
            _feature->style()->get<ExtrusionSymbol>() )
        {
            node = AnnotationUtils::installTwoPassAlpha( node );
        }

        //OE_NOTICE << GeometryUtils::geometryToGeoJSON( _feature->getGeometry() ) << std::endl;

        _attachPoint = new osg::Group();
        _attachPoint->addChild( node );

        // Draped (projected) geometry
        if ( ap.draping )
        {
            DrapeableNode* d = new DrapeableNode( getMapNode() );
            d->addChild( _attachPoint );
            this->addChild( d );
        }

        // GPU-clamped geometry
        else if ( ap.gpuClamping )
        {
            ClampableNode* clampable = new ClampableNode( getMapNode() );
            clampable->addChild( _attachPoint );
            this->addChild( clampable );

            const RenderSymbol* render = _feature->style()->get<RenderSymbol>();
            if ( render && render->depthOffset().isSet() )
            {
                clampable->setDepthOffsetOptions( *render->depthOffset() );
            }
        }

        else 
        {
            this->addChild( _attachPoint );

            // CPU-clamped geometry?
            if ( ap.sceneClamping )
            {
                // save for later when we need to reclamp the mesh on the CPU
                _altitude = _feature->style()->get<AltitudeSymbol>();

                // The polytope will ensure we only clamp to intersecting tiles:
                _feature->getWorldBoundingPolytope( getMapNode()->getMapSRS(), _featurePolytope );

                // activate the terrain callback:
                setCPUAutoClamping( true );

                // set default lighting based on whether we are extruding:
                setLightingIfNotSet( _feature->style()->has<ExtrusionSymbol>() );

                // do an initial clamp to get started.
                clampMesh( getMapNode()->getTerrain()->getGraph() );
            } 

            applyGeneralSymbology( *_feature->style() );
        }
    }
}

void
FeatureNode::setMapNode( MapNode* mapNode )
{
    if ( getMapNode() != mapNode )
    {
        AnnotationNode::setMapNode( mapNode );
        init();
    }
}

void
FeatureNode::setFeature( Feature* feature )
{
    _feature = feature;
    init();
}

const Style& FeatureNode::getStyle() const
{
    if ( _feature.valid() )
    {
        return *_feature->style();
    }
    return AnnotationNode::getStyle();
}

void
FeatureNode::setStyle(const Style& style)
{
    if ( _feature.valid() )
    {
        _feature->style() = style;
        init();
    }
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


// This will be called by AnnotationNode when a new terrain tile comes in.
void
FeatureNode::reclamp( const TileKey& key, osg::Node* tile, const Terrain* terrain )
{
    if ( _featurePolytope.contains( tile->getBound() ) )
    {
        clampMesh( tile );
    }
}

void
FeatureNode::clampMesh( osg::Node* terrainModel )
{
    if ( getMapNode() )
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

        MeshClamper clamper( terrainModel, getMapNode()->getMapSRS(), getMapNode()->isGeocentric(), relative, scale, offset );
        getAttachPoint()->accept( clamper );

        this->dirtyBound();
    }
}


//-------------------------------------------------------------------

OSGEARTH_REGISTER_ANNOTATION( feature, osgEarth::Annotation::FeatureNode );


FeatureNode::FeatureNode(MapNode*              mapNode,
                         const Config&         conf,
                         const osgDB::Options* dbOptions ) :
AnnotationNode( mapNode, conf )
{
    osg::ref_ptr<Geometry> geom;
    if ( conf.hasChild("geometry") )
    {
        Config geomconf = conf.child("geometry");
        geom = GeometryUtils::geometryFromWKT( geomconf.value() );
        if ( !geom.valid() )
            OE_WARN << LC << "Config is missing required 'geometry' element" << std::endl;
    }
    
    osg::ref_ptr<const SpatialReference> srs;
    srs = SpatialReference::create( conf.value("srs"), conf.value("vdatum") );
    if ( !srs.valid() )
        OE_WARN << LC << "Config is missing required 'srs' element" << std::endl;

    optional<GeoInterpolation> geoInterp;

    Style style;
    conf.getObjIfSet( "style", style );

    if ( srs.valid() && geom.valid() )
    {
        //_draped = conf.value<bool>("draped",false);
        Feature* feature = new Feature(geom.get(), srs.get(), style);

        conf.getIfSet( "geointerp", "greatcircle", feature->geoInterp(), GEOINTERP_GREAT_CIRCLE );
        conf.getIfSet( "geointerp", "rhumbline",   feature->geoInterp(), GEOINTERP_RHUMB_LINE );

        setFeature( feature );
    }
}

Config
FeatureNode::getConfig() const
{
    Config conf("feature");

    if ( _feature.valid() && _feature->getGeometry() )
    {
        conf.set("name", getName());

        Config geomConf("geometry");
        geomConf.value() = GeometryUtils::geometryToWKT( _feature->getGeometry() );
        conf.add(geomConf);

        std::string srs = _feature->getSRS() ? _feature->getSRS()->getHorizInitString() : "";
        if ( !srs.empty() ) conf.set("srs", srs);

        std::string vsrs = _feature->getSRS() ? _feature->getSRS()->getVertInitString() : "";
        if ( !vsrs.empty() ) conf.set("vdatum", vsrs);

        if ( _feature->geoInterp().isSet() )
            conf.set("geointerp", _feature->geoInterp() == GEOINTERP_GREAT_CIRCLE? "greatcircle" : "rhumbline");

        conf.addObjIfSet( "style", _feature->style() );
    }

    return conf;
}
