/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2016 Pelican Mapping
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

#include <osgEarthAnnotation/LocalGeometryNode>
#include <osgEarthAnnotation/AnnotationRegistry>
#include <osgEarthAnnotation/AnnotationUtils>
#include <osgEarthFeatures/GeometryCompiler>
#include <osgEarthFeatures/GeometryUtils>
#include <osgEarth/GeometryClamper>
#include <osgEarth/Utils>

#define LC "[GeometryNode] "

using namespace osgEarth;
using namespace osgEarth::Annotation;
using namespace osgEarth::Features;


LocalGeometryNode::LocalGeometryNode() :
GeoPositionNode(),
_clampRelative(false)
{
    //nop - unused
}

LocalGeometryNode::LocalGeometryNode(MapNode* mapNode) :
GeoPositionNode(),
_clampRelative(false)
{
    LocalGeometryNode::setMapNode( mapNode );
    init( 0L );
}

LocalGeometryNode::LocalGeometryNode(MapNode*     mapNode,
                                     Geometry*    geom,
                                     const Style& style) :
GeoPositionNode(),
_geom    ( geom ),
_style   ( style ),
_clampRelative(false)
{
    LocalGeometryNode::setMapNode( mapNode );
    init( 0L );
}


LocalGeometryNode::LocalGeometryNode(MapNode*     mapNode,
                                     osg::Node*   node,
                                     const Style& style) :
GeoPositionNode(),
_node    ( node ),
_style   ( style ),
_clampRelative(false)
{
    LocalGeometryNode::setMapNode( mapNode );
    init( 0L );
}

void
LocalGeometryNode::setMapNode(MapNode* mapNode)
{
    if ( mapNode != getMapNode() )
    {
        GeoPositionNode::setMapNode( mapNode );
        init(0L);
    }
}

void
LocalGeometryNode::initNode()
{
    osgEarth::clearChildren( getPositionAttitudeTransform() );

    if ( _node.valid() )
    {
        _node = AnnotationUtils::installOverlayParent( _node.get(), _style );

        getPositionAttitudeTransform()->addChild( _node.get() );

        applyRenderSymbology( getStyle() );

        setLightingIfNotSet( getStyle().has<ExtrusionSymbol>() );
    }
}


void
LocalGeometryNode::initGeometry(const osgDB::Options* dbOptions)
{
    osgEarth::clearChildren( getPositionAttitudeTransform() );

    if ( _geom.valid() )
    {
        osg::ref_ptr<Session> session;
        if ( getMapNode() )
            session = new Session(getMapNode()->getMap(), 0L, 0L, dbOptions);
        
        GeometryCompiler gc;
        osg::ref_ptr<osg::Node> node = gc.compile( _geom.get(), getStyle(), FilterContext(session) );
        if ( node.valid() )
        {
            node = AnnotationUtils::installOverlayParent( node.get(), getStyle() );

            getPositionAttitudeTransform()->addChild( node.get() );

            applyRenderSymbology( getStyle() );

            applyAltitudeSymbology( getStyle() );
        }
    }
}


void 
LocalGeometryNode::init(const osgDB::Options* options)
{    
    if ( _node.valid() )
    {
        initNode();
    }
    else
    {
        initGeometry( options );
    }
}


void
LocalGeometryNode::setStyle( const Style& style )
{
    _style = style;
    init( 0L );
}


void
LocalGeometryNode::setNode( osg::Node* node )
{
    _node = node;
    _geom = 0L;
    initNode();
}


void
LocalGeometryNode::setGeometry( Geometry* geom )
{
    _geom = geom;
    _node = 0L;
    initGeometry(0L);
}

void
LocalGeometryNode::applyAltitudeSymbology(const Style& style)
{
    // deal with scene clamping symbology.
    if ( getMapNode() )
    {
        if ( _clampCallback.valid() )
        {
            getMapNode()->getTerrain()->removeTerrainCallback( _clampCallback.get() );
            _clampCallback = 0L;
        }

        const AltitudeSymbol* alt = style.get<AltitudeSymbol>();
        if ( alt && alt->technique() == alt->TECHNIQUE_SCENE )
        {
            if ( alt->binding() == alt->BINDING_CENTROID )
            {
                // centroid scene clamping? let GeoTransform do its thing.
                getGeoTransform()->setAutoRecomputeHeights( true );
            }

            else if ( alt->binding() == alt->BINDING_VERTEX )
            {
                // per vertex clamping? disable the GeoTransform's clamping and take over.
                getGeoTransform()->setAutoRecomputeHeights( false );

                if ( !_clampCallback.valid() )
                {
                    _clampCallback = new ClampCallback(this);
                }

                if ( alt->clamping() == alt->CLAMP_TO_TERRAIN )
                {
                    _clampRelative = false;
                    getMapNode()->getTerrain()->addTerrainCallback( _clampCallback.get() );
                }

                else if ( alt->clamping() == alt->CLAMP_RELATIVE_TO_TERRAIN )
                {
                    _clampRelative = true;
                    getMapNode()->getTerrain()->addTerrainCallback( _clampCallback.get() );
                }
            }
        }
    }
}

void
LocalGeometryNode::onTileAdded(const TileKey&          key, 
                               osg::Node*              patch, 
                               TerrainCallbackContext& context)
{
    // if key and data intersect then
    if ( _boundingPT.contains(patch->getBound()) )
    {    
        clampToScene( patch, context.getTerrain() );
        this->dirtyBound();
    }
}

void
LocalGeometryNode::clampToScene(osg::Node* patch, const Terrain* terrain)
{
    GeometryClamper clamper;

    clamper.setTerrainPatch( patch );
    clamper.setTerrainSRS( terrain ? terrain->getSRS() : 0L );
    clamper.setPreserveZ( _clampRelative );
    clamper.setOffset( getPosition().alt() );

    this->accept( clamper );
}

osg::BoundingSphere
LocalGeometryNode::computeBound() const
{
    osg::BoundingSphere bs = AnnotationNode::computeBound();
    
    // NOTE: this is the same code found in Feature.cpp. Consolidate?

    _boundingPT.clear();

    // add planes for the four sides of the BS. Normals point inwards.
    _boundingPT.add( osg::Plane(osg::Vec3d( 1, 0,0), osg::Vec3d(-bs.radius(),0,0)) );
    _boundingPT.add( osg::Plane(osg::Vec3d(-1, 0,0), osg::Vec3d( bs.radius(),0,0)) );
    _boundingPT.add( osg::Plane(osg::Vec3d( 0, 1,0), osg::Vec3d(0, -bs.radius(),0)) );
    _boundingPT.add( osg::Plane(osg::Vec3d( 0,-1,0), osg::Vec3d(0,  bs.radius(),0)) );

    const SpatialReference* srs = getPosition().getSRS();
    if ( srs )
    {
        // for a projected feature, we're done. For a geocentric one, transform the polytope
        // into world (ECEF) space.
        if ( srs->isGeographic() && !srs->isPlateCarre() )
        {
            const osg::EllipsoidModel* e = srs->getEllipsoid();

            // add a bottom cap, unless the bounds are sufficiently large.
            double minRad = std::min(e->getRadiusPolar(), e->getRadiusEquator());
            double maxRad = std::max(e->getRadiusPolar(), e->getRadiusEquator());
            double zeroOffset = bs.center().length();
            if ( zeroOffset > minRad * 0.1 )
            {
                _boundingPT.add( osg::Plane(osg::Vec3d(0,0,1), osg::Vec3d(0,0,-maxRad+zeroOffset)) );
            }
        }

        // transform the clipping planes ito ECEF space
        GeoPoint refPoint;
        refPoint.fromWorld( srs, bs.center() );

        osg::Matrix local2world;
        refPoint.createLocalToWorld( local2world );

        _boundingPT.transform( local2world );
    }

    return bs;
}

void
LocalGeometryNode::dirty()
{
    GeoPositionNode::dirty();

    // re-clamp the geometry if necessary.
    if ( _clampCallback.valid() && getMapNode() )
    {
        clampToScene( getMapNode()->getTerrain()->getGraph(), getMapNode()->getTerrain() );
    }
}

//-------------------------------------------------------------------

OSGEARTH_REGISTER_ANNOTATION( local_geometry, osgEarth::Annotation::LocalGeometryNode );


LocalGeometryNode::LocalGeometryNode(MapNode*              mapNode,
                                     const Config&         conf,
                                     const osgDB::Options* dbOptions) :
GeoPositionNode( mapNode, conf ),
_clampRelative(false)
{
    if ( conf.hasChild("geometry") )
    {
        Config geomconf = conf.child("geometry");
        _geom = GeometryUtils::geometryFromWKT( geomconf.value() );
    }

    conf.getObjIfSet( "style", _style );
    init( dbOptions );
}

Config
LocalGeometryNode::getConfig() const
{
    Config conf = GeoPositionNode::getConfig();
    conf.key() = "local_geometry";

    if ( !_style.empty() )
        conf.addObj( "style", _style );

    if ( _geom.valid() )
    {
        conf.add( Config("geometry", GeometryUtils::geometryToWKT(_geom.get())) );
    }

    return conf;
}
