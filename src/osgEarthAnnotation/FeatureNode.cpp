/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2019 Pelican Mapping
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
#include <osgEarthFeatures/FilterContext>

#include <osgEarthSymbology/AltitudeSymbol>

#include <osgEarth/ClampableNode>
#include <osgEarth/DrapeableNode>
#include <osgEarth/NodeUtils>
#include <osgEarth/Utils>
#include <osgEarth/Registry>
#include <osgEarth/CullingUtils>
#include <osgEarth/GeometryClamper>
#include <osgEarth/TerrainEngineNode>

#include <osg/BoundingSphere>
#include <osg/Polytope>
#include <osg/Transform>

#define LC "[FeatureNode] "

using namespace osgEarth;
using namespace osgEarth::Annotation;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;

FeatureNode::FeatureNode(Feature* feature,
                         const Style& in_style,
                         const GeometryCompilerOptions& options,
                         StyleSheet* styleSheet) :
AnnotationNode(),
_options           ( options ),
_needsRebuild      ( true ),
_styleSheet        ( styleSheet ),
_clampDirty        (false),
_index             ( 0 )
{
    _features.push_back( feature );

    Style style = in_style;
    if (style.empty() && feature->style().isSet())
    {
        style = *feature->style();
    }

    setStyle( style );
}

FeatureNode::FeatureNode(const FeatureList& features,
                         const Style& style,
                         const GeometryCompilerOptions& options,
                         StyleSheet* styleSheet):
AnnotationNode(),
_options        ( options ),
_needsRebuild   ( true ),
_styleSheet     ( styleSheet ),
_clampDirty     ( false ),
_index          ( 0 )
{
    _features.insert( _features.end(), features.begin(), features.end() );
    setStyle( style );
}

void
FeatureNode::build()
{
    if ( !_clampCallback.valid() )
        _clampCallback = new ClampCallback(this);

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
    if ( ap.sceneClamping )
    {
        options.ignoreAltitudeSymbol() = true;
    }

    _clamperData.clear();

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
        Session* session = new Session( getMapNode()->getMap(), _styleSheet.get() );

        FilterContext context( session, new FeatureProfile( _extent ), _extent, _index);

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

        _attachPoint = new osg::Group();
        _attachPoint->addChild( node );

        // Draped (projected) geometry
        if ( ap.draping )
        {
            DrapeableNode* d = new DrapeableNode();
            d->addChild( _attachPoint );
            this->addChild( d );
        }

        // GPU-clamped geometry
        else if ( ap.gpuClamping )
        {
            ClampableNode* clampable = new ClampableNode();
            clampable->addChild( _attachPoint );
            this->addChild( clampable );
        }

        else
        {
            this->addChild( _attachPoint );

            // set default lighting based on whether we are extruding:
            setDefaultLighting( style.has<ExtrusionSymbol>() );
        }

        applyRenderSymbology(style);

        if ( getMapNode()->getTerrain() )
        {
            if ( ap.sceneClamping )
            {
                // Need dynamic data variance since scene clamping will change the verts
                SetDataVarianceVisitor sdv(osg::Object::DYNAMIC);
                this->accept(sdv);

                getMapNode()->getTerrain()->addTerrainCallback(_clampCallback.get());
                clamp(getMapNode()->getTerrain()->getGraph(), getMapNode()->getTerrain());
            }
            else
            {
                getMapNode()->getTerrain()->removeTerrainCallback( _clampCallback.get() );
            }
        }
    }
}

void
FeatureNode::setMapNode( MapNode* mapNode )
{
    if ( getMapNode() != mapNode )
    {
        if (_clampCallback.valid() && getMapNode() && getMapNode()->getTerrain())
            getMapNode()->getTerrain()->removeTerrainCallback( _clampCallback.get() );

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
    _style = style;

    AnnotationNode::setStyle( style );

    _needsRebuild = true;
    build();
}

StyleSheet* FeatureNode::getStyleSheet() const
{
    return _styleSheet.get();
}

void FeatureNode::setStyleSheet(StyleSheet* styleSheet)
{
    _styleSheet = styleSheet;
}

FeatureIndexBuilder* FeatureNode::getIndex()
{
    return _index;
}

void FeatureNode::setIndex(FeatureIndexBuilder* index)
{
    if (_index != index)
    {
        _index = index;
        _needsRebuild = true;
        build();
    }
}

Feature* FeatureNode::getFeature()
{
    if (_features.size() == 1)
    {
        return _features.front().get();
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

void FeatureNode::dirty()
{
    _needsRebuild = true;
    build();
}

// This will be called by AnnotationNode when a new terrain tile comes in.
void
FeatureNode::onTileAdded(const TileKey&          key,
                         osg::Node*              graph,
                         TerrainCallbackContext& context)
{
    if (!_clampDirty)
    {
        bool needsClamp;

        if (key.valid())
        {
            osg::Polytope tope;
            key.getExtent().createPolytope(tope);
            needsClamp = tope.contains(this->getBound());
        }
        else
        {
            // without a valid tilekey we don't know the extent of the change,
            // so clamping is required.
            needsClamp = true;
        }

        if (needsClamp)
        {
            _clampDirty = true;
            ADJUST_UPDATE_TRAV_COUNT(this, +1);
            //clamp(graph, context.getTerrain());
        }
    }
}

void
FeatureNode::clamp(osg::Node* graph, const Terrain* terrain)
{
    if ( terrain && graph )
    {
        const AltitudeSymbol* alt = getStyle().get<AltitudeSymbol>();
        if (alt && alt->technique() != alt->TECHNIQUE_SCENE)
            return;

        bool relative = alt && alt->clamping() == alt->CLAMP_RELATIVE_TO_TERRAIN && alt->technique() == alt->TECHNIQUE_SCENE;
        float offset = alt ? alt->verticalOffset()->eval() : 0.0f;

        GeometryClamper clamper(_clamperData);
        clamper.setTerrainPatch( graph );
        clamper.setTerrainSRS( terrain->getSRS() );
        clamper.setUseVertexZ( relative );
        clamper.setOffset( offset );

        this->accept( clamper );
    }
}

void
FeatureNode::traverse(osg::NodeVisitor& nv)
{
    if (nv.getVisitorType() == nv.UPDATE_VISITOR && _clampDirty)
    {
        if (getMapNode())
        {
            osg::ref_ptr<Terrain> terrain = getMapNode()->getTerrain();
            if (terrain.valid())
                clamp(terrain->getGraph(), terrain.get());

            ADJUST_UPDATE_TRAV_COUNT(this, -1);
            _clampDirty = false;
        }
    }
    AnnotationNode::traverse(nv);
}


//-------------------------------------------------------------------

OSGEARTH_REGISTER_ANNOTATION( feature, osgEarth::Annotation::FeatureNode );


FeatureNode::FeatureNode(const Config&         conf,
                         const osgDB::Options* readOptions ) :
AnnotationNode(conf, readOptions),
_clampDirty(false),
_index(0)
{
    osg::ref_ptr<Geometry> geom;
    if ( conf.hasChild("geometry") )
    {
        Config geomconf = conf.child("geometry");
        geom = GeometryUtils::geometryFromWKT( geomconf.value() );
        if ( !geom.valid() )
            OE_WARN << LC << "Config (" << conf.value("name") << ") is missing valid 'geometry' element" << std::endl;
    }

    osg::ref_ptr<const SpatialReference> srs;
    srs = SpatialReference::create( conf.value("srs"), conf.value("vdatum") );
    if ( !srs.valid() )
        OE_WARN << LC << "Config is missing valid 'srs' element" << std::endl;

    optional<GeoInterpolation> geoInterp;

    conf.get( "style", _style );

    //FeatureNode::setMapNode( mapNode );

    if ( srs.valid() && geom.valid() )
    {
        Feature* feature = new Feature(geom.get(), srs.get() );

        conf.get( "geointerp", "greatcircle", feature->geoInterp(), GEOINTERP_GREAT_CIRCLE );
        conf.get( "geointerp", "rhumbline",   feature->geoInterp(), GEOINTERP_RHUMB_LINE );

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
        geomConf.setValue(GeometryUtils::geometryToWKT( feature->getGeometry() ));
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
        conf.set( "style", _style );
    }

    return conf;
}
