/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2015 Pelican Mapping
* http://osgearth.org
*
* osgEarth is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
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
#include <osgEarth/CullingUtils>

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
                         const Style& style,
                         const GeometryCompilerOptions& options ) :
AnnotationNode( mapNode ),
_style        ( style ),
_options      ( options ),
_needsRebuild (true),
_clusterCulling(true),
_clusterCullingCallback(0)
{
    if (_style.empty() && feature->style().isSet())
    {
        _style = *feature->style();
    }

    _features.push_back( feature );
    build();
}

FeatureNode::FeatureNode(MapNode* mapNode, 
                         FeatureList& features,
                         const Style& style,
                         const GeometryCompilerOptions& options):
AnnotationNode( mapNode ),
_style        ( style ),
_options      ( options ),
_needsRebuild (true),
_clusterCulling(true),
_clusterCullingCallback(0)
{
    _features.insert( _features.end(), features.begin(), features.end() );
    build();
}

bool
FeatureNode::getClusterCulling() const
{
    return _clusterCulling;
}

void
FeatureNode::setClusterCulling( bool clusterCulling)
{
    if (_clusterCulling != clusterCulling)
    {
        _clusterCulling = clusterCulling;
        updateClusterCulling();
    }
}


void
FeatureNode::build()
{
    // if there's a decoration, clear it out first.
    this->clearDecoration();
    _attachPoint = 0L;

    // if there is existing geometry, kill it
    this->removeChildren( 0, this->getNumChildren() );

    if ( !getMapNode() )
        return;

    if ( _features.empty() )
        return;

    const Style &style = getStyle();

    // compilation options.
    GeometryCompilerOptions options = _options;
    
    // figure out what kind of altitude manipulation we need to perform.
    AnnotationUtils::AltitudePolicy ap;
    AnnotationUtils::getAltitudePolicy( style, ap );

    // If we're doing auto-clamping on the CPU, shut off compiler map clamping
    // clamping since it would be redundant.
    // TODO: I think this is OBE now that we have "scene" clamping technique..
    if ( ap.sceneClamping )
    {
        options.ignoreAltitudeSymbol() = true;
    }

    osg::Node* node = _compiled.get();
    if (_needsRebuild || !_compiled.valid() )
    {
        // Clone the Features before rendering as the GeometryCompiler and it's filters can change the coordinates
        // of the geometry when performing localization or converting to geocentric.
        _extent = GeoExtent::INVALID;

        FeatureList clone;
        for(FeatureList::iterator itr = _features.begin(); itr != _features.end(); ++itr)
        {
            Feature* feature = new Feature( *itr->get(), osg::CopyOp::DEEP_COPY_ALL);
            GeoExtent featureExtent(feature->getSRS(), feature->getGeometry()->getBounds());

            if (_extent.isInvalid())
            {
                _extent = featureExtent;
            }
            else
            {
                _extent.expandToInclude( featureExtent );
            }
            clone.push_back( feature );
        }

        // prep the compiler:
        GeometryCompiler compiler( options );
        Session* session = new Session( getMapNode()->getMap() );

        FilterContext context( session, new FeatureProfile( _extent ), _extent );

        _compiled = compiler.compile( clone, style, context );
        node = _compiled.get();
        _needsRebuild = false;

        // Compute the world bounds
        osg::BoundingSphered bounds;
        for( FeatureList::iterator itr = _features.begin(); itr != _features.end(); ++itr)
        {
            osg::BoundingSphered bs;
            itr->get()->getWorldBound(getMapNode()->getMapSRS(), bs);
            bounds.expandBy(bs);
        }
        // The polytope will ensure we only clamp to intersecting tiles:
        Feature::getWorldBoundingPolytope(bounds, getMapNode()->getMapSRS(), _featurePolytope);

    }

    if ( node )
    {
        if ( AnnotationUtils::styleRequiresAlphaBlending( style ) &&
             getStyle().get<ExtrusionSymbol>() )
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

            const RenderSymbol* render = style.get<RenderSymbol>();
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
                _altitude = style.get<AltitudeSymbol>();

                // activate the terrain callback:
                setCPUAutoClamping( true );

                // set default lighting based on whether we are extruding:
                setLightingIfNotSet( style.has<ExtrusionSymbol>() );

                // do an initial clamp to get started.
                clampMesh( getMapNode()->getTerrain()->getGraph() );
            } 

            applyGeneralSymbology( style );
        }
    }

    updateClusterCulling();
}

void
FeatureNode::setMapNode( MapNode* mapNode )
{
    if ( getMapNode() != mapNode )
    {
        AnnotationNode::setMapNode( mapNode );
        _needsRebuild = true;
        build();
    }
}

const Style& FeatureNode::getStyle() const
{
    return _style;
}

void
FeatureNode::setStyle(const Style& style)
{
    // Try to compare the styles and see if we can get away with not compiling the geometry again.
    Style a = _style;
    Style b = style;
   
    // If the only thing that has changed is the AltitudeSymbol, we don't need to worry about rebuilding the entire geometry again.
    a.remove<AltitudeSymbol>();
    b.remove<AltitudeSymbol>();
    if (a.getConfig().toJSON() == b.getConfig().toJSON())
    {
        _needsRebuild = false;
    }
    else
    {
        _needsRebuild = true;
    }
    _style = style;
    build();
}

Feature* FeatureNode::getFeature()
{
    if (_features.size() == 1)
    {
        return _features.front();
    }
    return 0;
}

void FeatureNode::setFeature(Feature* feature)
{
    _features.clear();
    if (feature)
    {
        _features.push_back( feature );
    }
    _needsRebuild = true;
    build();
}

void FeatureNode::init()
{
    _needsRebuild = true;
    build();
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
            NumericExpression scaleExpr(_altitude->verticalScale().value());
            NumericExpression offsetExpr(_altitude->verticalOffset().value());
            scale = scaleExpr.eval();
            offset = offsetExpr.eval();
            relative = _altitude->clamping() == AltitudeSymbol::CLAMP_RELATIVE_TO_TERRAIN;
        }

        MeshClamper clamper( terrainModel, getMapNode()->getMapSRS(), getMapNode()->isGeocentric(), relative, scale, offset );
        getAttachPoint()->accept( clamper );

        this->dirtyBound();
    }
}

void
FeatureNode::updateClusterCulling()
{
    // install a cluster culler.
    if ( getMapNode()->isGeocentric() && _clusterCulling && !_clusterCullingCallback)
    {
        const GeoExtent& ccExtent = _extent;
        if ( ccExtent.isValid() )
        {
            // if the extent is more than 90 degrees, bail
            GeoExtent geodeticExtent = ccExtent.transform( ccExtent.getSRS()->getGeographicSRS() );
            if ( geodeticExtent.width() < 90.0 && geodeticExtent.height() < 90.0 )
            {
                // get the geocentric tile center:
                osg::Vec3d tileCenter;
                ccExtent.getCentroid( tileCenter.x(), tileCenter.y() );

                osg::Vec3d centerECEF;
                ccExtent.getSRS()->transform( tileCenter, getMapNode()->getMapSRS()->getECEF(), centerECEF );
                _clusterCullingCallback = ClusterCullingFactory::create2( this, centerECEF );
                if ( _clusterCullingCallback )
                    this->addCullCallback( _clusterCullingCallback );
            }
        }
    }
    else if (!_clusterCulling && _clusterCullingCallback)
    {
        this->removeCullCallback( _clusterCullingCallback );
        _clusterCullingCallback = 0;
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

    conf.getObjIfSet( "style", _style );

    if ( srs.valid() && geom.valid() )
    {
        Feature* feature = new Feature(geom.get(), srs.get() );

        conf.getIfSet( "geointerp", "greatcircle", feature->geoInterp(), GEOINTERP_GREAT_CIRCLE );
        conf.getIfSet( "geointerp", "rhumbline",   feature->geoInterp(), GEOINTERP_RHUMB_LINE );

        _features.push_back( feature );
        build();
    }
}

Config
FeatureNode::getConfig() const
{
    
    Config conf("feature");

    if ( !_features.empty() )
    {
        // Write out a single feature for now.

        Feature* feature = _features.begin()->get();

        conf.set("name", getName());

        Config geomConf("geometry");
        geomConf.value() = GeometryUtils::geometryToWKT( feature->getGeometry() );
        conf.add(geomConf);

        std::string srs = feature->getSRS() ? feature->getSRS()->getHorizInitString() : "";
        if ( !srs.empty() ) conf.set("srs", srs);

        std::string vsrs = feature->getSRS() ? feature->getSRS()->getVertInitString() : "";
        if ( !vsrs.empty() ) conf.set("vdatum", vsrs);

        if ( feature->geoInterp().isSet() )
            conf.set("geointerp", feature->geoInterp() == GEOINTERP_GREAT_CIRCLE? "greatcircle" : "rhumbline");
    }

    if (!_style.empty() )
    {
        conf.addObj( "style", _style );
    }

    return conf;
}
