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

#include <osgEarthAnnotation/LocalGeometryNode>
#include <osgEarthAnnotation/AnnotationRegistry>
#include <osgEarthAnnotation/AnnotationUtils>
#include <osgEarthFeatures/GeometryCompiler>
#include <osgEarthFeatures/GeometryUtils>
#include <osgEarthFeatures/FilterContext>
#include <osgEarth/GeometryClamper>
#include <osgEarth/Utils>
#include <osgEarth/NodeUtils>

#define LC "[GeometryNode] "

using namespace osgEarth;
using namespace osgEarth::Annotation;
using namespace osgEarth::Features;


LocalGeometryNode::LocalGeometryNode() :
GeoPositionNode()
{
    construct();
}

LocalGeometryNode::LocalGeometryNode(Geometry*    geom,
                                     const Style& style) :
GeoPositionNode()
{
    construct();
    setStyle(style);
    setGeometry(geom);
}

void
LocalGeometryNode::construct()
{
    _geom = 0L;
    _clampInUpdateTraversal = false;
    _perVertexClampingEnabled = false;
}

void
LocalGeometryNode::setMapNode(MapNode* mapNode)
{
    if ( mapNode != getMapNode() )
    {
        GeoPositionNode::setMapNode( mapNode );
        compileGeometry();
    }
}

void
LocalGeometryNode::compileGeometry()
{
    // clear out existing geometry first
    if (_node.valid())
    {
        getPositionAttitudeTransform()->removeChild(_node.get());
    }

    // any old clamping data is out of date, so clear it
    _clamperData.clear();
    _perVertexClampingEnabled = false;
    
    if ( _geom.valid() )
    {
        osg::ref_ptr<Session> session;
        if ( getMapNode() )
        {
            session = new Session(getMapNode()->getMap(), 0L);
        }

        AltitudeSymbol* alt = _style.get<AltitudeSymbol>();

        GeometryCompilerOptions options;
        if (alt == NULL ||
            alt->technique().isSet() == false ||
            alt->technique().isSetTo(alt->TECHNIQUE_SCENE))
        {        
            options.ignoreAltitudeSymbol() = true;
        }

        GeometryCompiler gc(options);

        _node = gc.compile( _geom.get(), getStyle(), FilterContext(session.get()) );
        if ( _node.valid() )
        {
            // deal with draping or gpu-clamping settings
            _node = AnnotationUtils::installOverlayParent( _node.get(), getStyle() );

            // install the new geometry under the geotransforms
            getPositionAttitudeTransform()->addChild( _node.get() );

            // re-assess support for per vertex clamping
            togglePerVertexClamping();

            // apply current style
            setDefaultLighting( getStyle().has<ExtrusionSymbol>() );
            applyRenderSymbology( getStyle() );
        }
    }
}

void
LocalGeometryNode::setStyle( const Style& style )
{
    _style = style;
    compileGeometry();
}

void
LocalGeometryNode::setGeometry( Geometry* geom )
{
    _geom = geom;
    compileGeometry();
}

// GeoPositionNode override
void
LocalGeometryNode::setPosition(const GeoPoint& pos)
{
    GeoPositionNode::setPosition(pos);

    // detect a position change because it will require a re-clamp
    // if per-vertex clamping is enabled.
    bool posXYchanged = false;

    if (_lastPosition.x() != getPosition().x() ||
        _lastPosition.y() != getPosition().y())
    {
        posXYchanged = true;
    }

    _lastPosition = getPosition();

    // since the altitude mode may have changed, assess whether
    // to toggle the per-vertex clamping
    togglePerVertexClamping();

    if (posXYchanged)
    {
        reclamp();
    }
}

void
LocalGeometryNode::togglePerVertexClamping()
{
    const AltitudeSymbol* alt = _style.get<AltitudeSymbol>();
    bool needPVC =
        alt &&
        alt->binding() == alt->BINDING_VERTEX &&
        (alt->technique().isSet() == false || alt->technique() == alt->TECHNIQUE_SCENE) &&
        getPosition().altitudeMode() == ALTMODE_RELATIVE;

    if (needPVC && !_perVertexClampingEnabled)
    {    
        osg::ref_ptr<Terrain> terrain = getGeoTransform()->getTerrain();
        if (terrain.valid())
        {
            if (!_clampCallback.valid())
                _clampCallback = new ClampCallback(this);

            if (_clampCallback->referenceCount() == 1)
                terrain->addTerrainCallback(_clampCallback.get());

            // all drawables must be dynamic since we are altering the verts
            SetDataVarianceVisitor sdv(osg::Object::DYNAMIC);
            this->accept(sdv);

            _perVertexClampingEnabled = true;

            reclamp();
        }
    }

    else if (!needPVC && _perVertexClampingEnabled)
    {
        osg::ref_ptr<Terrain> terrain = getGeoTransform()->getTerrain();
        if (terrain.valid())
        {
            if (_clampCallback.valid())
                terrain->removeTerrainCallback(_clampCallback.get());

            // revert to original vertex array if necessary:
            GeometryClamper clamper(_clamperData);
            clamper.setRevert(true);
            this->accept(clamper);

            _perVertexClampingEnabled = false;
        }
    }
}

void
LocalGeometryNode::onTileAdded(const TileKey&          key, 
                               osg::Node*              graph, 
                               TerrainCallbackContext& context)
{
    // If we are already set to clamp, ignore this
    if (_clampInUpdateTraversal)
        return;

    bool needsClamp;

    // Does the tile key's polytope intersect the world bounds or this object?
    // (taking getParent(0) gives the world-tranformed bounds vs. local bounds)
    if (key.valid())
    {
        osg::Polytope tope;
        key.getExtent().createPolytope(tope);
        needsClamp = tope.contains(getBound());
    }
    else
    {
        // with no key, must clamp no matter what
        needsClamp = true;
    }

    if (needsClamp)
    {
        _clampInUpdateTraversal = true;
        ADJUST_UPDATE_TRAV_COUNT(this, +1);

        OE_DEBUG << LC << "LGN: clamp requested b/c of key " << key.str() << std::endl;
    }
}

void
LocalGeometryNode::reclamp()
{
    if (_perVertexClampingEnabled)
    {
        osg::ref_ptr<Terrain> terrain = getGeoTransform()->getTerrain();
        if (terrain.valid())
        {
            clamp(terrain->getGraph(), terrain.get());
        }
    }
}

void
LocalGeometryNode::clamp(osg::Node* graph, const Terrain* terrain)
{
    if (terrain && graph)
    {
        GeometryClamper clamper(_clamperData);

        // The data to clamp to
        clamper.setTerrainPatch( graph );
        clamper.setTerrainSRS( terrain ? terrain->getSRS() : 0L );

        // Since the GeometryClamper will use the matrix stack to
        // resolve vertex locations, and that matrix stack will incorporate
        // the GeoTransform's altitude, we need to compensate by adding the
        // altitude back in as an offset.
        clamper.setOffset(getPosition().alt());

        this->accept( clamper );
        
        OE_DEBUG << LC << "LGN: clamped.\n";
    }
}

void
LocalGeometryNode::traverse(osg::NodeVisitor& nv)
{
    if (nv.getVisitorType() == nv.UPDATE_VISITOR && _clampInUpdateTraversal)
    {
        reclamp();

        _clampInUpdateTraversal = false;
        ADJUST_UPDATE_TRAV_COUNT(this, -1);
    }
    GeoPositionNode::traverse(nv);
}

//-------------------------------------------------------------------

OSGEARTH_REGISTER_ANNOTATION( local_geometry, osgEarth::Annotation::LocalGeometryNode );


LocalGeometryNode::LocalGeometryNode(const Config&         conf,
                                     const osgDB::Options* options) :
GeoPositionNode(conf, options)
{
    construct();

    conf.get( "style", _style );

    if ( conf.hasChild("geometry") )
    {
        Config geomconf = conf.child("geometry");
        osg::ref_ptr<Geometry> geom = GeometryUtils::geometryFromWKT( geomconf.value() );
        setGeometry(geom.get());
    }
}

Config
LocalGeometryNode::getConfig() const
{
    Config conf = GeoPositionNode::getConfig();
    conf.key() = "local_geometry";

    if ( !_style.empty() )
        conf.set( "style", _style );

    if ( _geom.valid() )
        conf.set( Config("geometry", GeometryUtils::geometryToWKT(_geom.get())) );

    return conf;
}
