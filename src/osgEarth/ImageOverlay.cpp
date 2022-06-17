/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2020 Pelican Mapping
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
#include <osgEarth/ImageOverlay>
#include <osgEarth/AnnotationRegistry>
#include <osgEarth/MeshSubdivider>
#include <osgEarth/GeometryUtils>
#include <osgEarth/MapNode>
#include <osgEarth/NodeUtils>
#include <osgEarth/ImageUtils>
#include <osgEarth/DrapeableNode>
#include <osgEarth/VirtualProgram>
#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/Texture2D>
#include <osg/io_utils>
#include <algorithm>

#define LC "[ImageOverlay] "

using namespace osgEarth;

//---------------------------------------------------------------------------

namespace
{
    void clampLatitude(osg::Vec2d& l)
    {
        l.y() = osg::clampBetween( l.y(), -90.0, 90.0);
    }    

    static Distance default_geometryResolution(5.0, Units::DEGREES);

    const char* imageVS =
        "out vec2 oe_ImageOverlay_texcoord; \n"
        "void oe_ImageOverlay_VS(inout vec4 vertex) { \n"
        "    oe_ImageOverlay_texcoord = gl_MultiTexCoord0.st; \n"
        "} \n";

    const char* imageFS =
        "in vec2 oe_ImageOverlay_texcoord; \n"
        "uniform sampler2D oe_ImageOverlay_tex; \n"
        "uniform float oe_ImageOverlay_alpha; \n"
        "void oe_ImageOverlay_FS(inout vec4 color) { \n"
        "    color = texture(oe_ImageOverlay_tex, oe_ImageOverlay_texcoord);\n"
        "    color.a *= oe_ImageOverlay_alpha; \n"
        "} \n";

}

//---------------------------------------------------------------------------

OSGEARTH_REGISTER_ANNOTATION( imageoverlay, osgEarth::ImageOverlay );

osg::ref_ptr<VirtualProgram> ImageOverlay::_program;

ImageOverlay::ImageOverlay(const Config& conf, const osgDB::Options* readOptions) :
AnnotationNode(conf, readOptions),
_lowerLeft    (10, 10),
_lowerRight   (20, 10),
_upperRight   (20, 20),
_upperLeft    (10, 20),
_dirty        (false),
_alpha        (1.0f),
_minFilter    (osg::Texture::LINEAR_MIPMAP_LINEAR),
_magFilter    (osg::Texture::LINEAR),
_texture      (0),
_geometryResolution(default_geometryResolution),
_draped(true)
{
    construct();

    conf.get( "url",   _imageURI );
    if ( _imageURI.isSet() )
    {
        setImage( _imageURI->getImage(readOptions) );
    }

    optional<float> tmpAlpha;
    if (conf.get( "alpha", tmpAlpha))
        setAlpha( *tmpAlpha );
    
    osg::ref_ptr<Geometry> geom;
    if ( conf.hasChild("geometry") )
    {
        Config geomconf = conf.child("geometry");
        geom = GeometryUtils::geometryFromWKT( geomconf.value() );
        
        if ( !geom.valid() || geom->size() < 4 )
        {
            OE_WARN << LC << "Config is missing required 'geometry' element, or not enough points (need 4)" << std::endl;
        }
        else
        {
            _lowerLeft.set ( (*geom)[0].x(), (*geom)[0].y() );
            _lowerRight.set( (*geom)[1].x(), (*geom)[1].y() );
            _upperRight.set( (*geom)[2].x(), (*geom)[2].y() );
            _upperLeft.set ( (*geom)[3].x(), (*geom)[3].y() );
        }
    }


    //Load the filter settings
    conf.get("mag_filter","LINEAR",                _magFilter,osg::Texture::LINEAR);
    conf.get("mag_filter","LINEAR_MIPMAP_LINEAR",  _magFilter,osg::Texture::LINEAR_MIPMAP_LINEAR);
    conf.get("mag_filter","LINEAR_MIPMAP_NEAREST", _magFilter,osg::Texture::LINEAR_MIPMAP_NEAREST);
    conf.get("mag_filter","NEAREST",               _magFilter,osg::Texture::NEAREST);
    conf.get("mag_filter","NEAREST_MIPMAP_LINEAR", _magFilter,osg::Texture::NEAREST_MIPMAP_LINEAR);
    conf.get("mag_filter","NEAREST_MIPMAP_NEAREST",_magFilter,osg::Texture::NEAREST_MIPMAP_NEAREST);
    conf.get("min_filter","LINEAR",                _minFilter,osg::Texture::LINEAR);
    conf.get("min_filter","LINEAR_MIPMAP_LINEAR",  _minFilter,osg::Texture::LINEAR_MIPMAP_LINEAR);
    conf.get("min_filter","LINEAR_MIPMAP_NEAREST", _minFilter,osg::Texture::LINEAR_MIPMAP_NEAREST);
    conf.get("min_filter","NEAREST",               _minFilter,osg::Texture::NEAREST);
    conf.get("min_filter","NEAREST_MIPMAP_LINEAR", _minFilter,osg::Texture::NEAREST_MIPMAP_LINEAR);
    conf.get("min_filter","NEAREST_MIPMAP_NEAREST",_minFilter,osg::Texture::NEAREST_MIPMAP_NEAREST);

    conf.get("draped", _draped);
    if (_draped.isSet())
        setDraped(*_draped);

    if (conf.hasValue("geometry_resolution"))
    {
        float value; Units units;
        if (Units::parse(conf.value("geometry_resolution"), value, units, Units::DEGREES))
            _geometryResolution.set(value, units);
    }

    compile();
    //ImageOverlay::setMapNode( mapNode );
}

Config
ImageOverlay::getConfig() const
{
    Config conf("imageoverlay");
    conf.set("name",  getName());

    if ( _imageURI.isSet() )
    {
        conf.set("url", _imageURI );
    }
    else if ( _image.valid() && !_image->getFileName().empty() )
    {
        optional<URI> temp;
        temp = URI(_image->getFileName());
        conf.set("url", temp);
    }

    conf.set("alpha", _alpha);

    osg::ref_ptr<Geometry> g = new Polygon();
    g->push_back( osg::Vec3d(_lowerLeft.x(),  _lowerLeft.y(), 0) );
    g->push_back( osg::Vec3d(_lowerRight.x(), _lowerRight.y(), 0) );
    g->push_back( osg::Vec3d(_upperRight.x(), _upperRight.y(), 0) );
    g->push_back( osg::Vec3d(_upperLeft.x(),  _upperLeft.y(),  0) );

    Config geomConf("geometry");
    geomConf.setValue(GeometryUtils::geometryToWKT( g.get() ));
    conf.add( geomConf );

    //Save the filter settings
	conf.set("mag_filter","LINEAR",                _magFilter,osg::Texture::LINEAR);
    conf.set("mag_filter","LINEAR_MIPMAP_LINEAR",  _magFilter,osg::Texture::LINEAR_MIPMAP_LINEAR);
    conf.set("mag_filter","LINEAR_MIPMAP_NEAREST", _magFilter,osg::Texture::LINEAR_MIPMAP_NEAREST);
    conf.set("mag_filter","NEAREST",               _magFilter,osg::Texture::NEAREST);
    conf.set("mag_filter","NEAREST_MIPMAP_LINEAR", _magFilter,osg::Texture::NEAREST_MIPMAP_LINEAR);
    conf.set("mag_filter","NEAREST_MIPMAP_NEAREST",_magFilter,osg::Texture::NEAREST_MIPMAP_NEAREST);
    conf.set("min_filter","LINEAR",                _minFilter,osg::Texture::LINEAR);
    conf.set("min_filter","LINEAR_MIPMAP_LINEAR",  _minFilter,osg::Texture::LINEAR_MIPMAP_LINEAR);
    conf.set("min_filter","LINEAR_MIPMAP_NEAREST", _minFilter,osg::Texture::LINEAR_MIPMAP_NEAREST);
    conf.set("min_filter","NEAREST",               _minFilter,osg::Texture::NEAREST);
    conf.set("min_filter","NEAREST_MIPMAP_LINEAR", _minFilter,osg::Texture::NEAREST_MIPMAP_LINEAR);
    conf.set("min_filter","NEAREST_MIPMAP_NEAREST",_minFilter,osg::Texture::NEAREST_MIPMAP_NEAREST);

    conf.set("draped", _draped);

    if (_geometryResolution != default_geometryResolution)
    {
        conf.set("geometry_resolution", _geometryResolution.asParseableString());
    }

    return conf;
}

//---------------------------------------------------------------------------


ImageOverlay::ImageOverlay(MapNode* mapNode, osg::Image* image) :
AnnotationNode(),
_lowerLeft    (10, 10),
_lowerRight   (20, 10),
_upperRight   (20, 20),
_upperLeft    (10, 20),
_image        (image),
_dirty        (false),
_alpha        (1.0f),
_minFilter    (osg::Texture::LINEAR_MIPMAP_LINEAR),
_magFilter    (osg::Texture::LINEAR),
_texture      (0),
_geometryResolution(default_geometryResolution),
_draped(true)
{        
    construct();

    ImageOverlay::setMapNode(mapNode);

    compile();
}

void
ImageOverlay::construct()
{
    _updateScheduled = false;

    // place the geometry under a drapeable node so it will project onto the terrain    
    DrapeableNode* d = new DrapeableNode();
    d->setDrapingEnabled(*_draped);
    addChild( d );
    
    if (!_program.valid())
    {
        static Threading::Mutex mutex(OE_MUTEX_NAME);
        mutex.lock();
        if (_program.valid() == false)
        {
            _program = new VirtualProgram;
            _program->setInheritShaders(true);
            _program->setFunction("oe_ImageOverlay_VS", imageVS, VirtualProgram::LOCATION_VERTEX_MODEL);
            _program->setFunction("oe_ImageOverlay_FS", imageFS, VirtualProgram::LOCATION_FRAGMENT_COLORING);
        }
        mutex.unlock();
    }

    _root = new osg::Group();
    osg::StateSet *ss = _root->getOrCreateStateSet();
    ss->setAttributeAndModes(_program.get(), osg::StateAttribute::ON);
    ss->addUniform(new osg::Uniform("oe_ImageOverlay_tex", 0));
    ss->addUniform(new osg::Uniform("oe_ImageOverlay_alpha", *_alpha));
    ss->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
    d->addChild( _root );

    ADJUST_EVENT_TRAV_COUNT(this, 1);
}

void
ImageOverlay::compile()
{
    Threading::ScopedMutexLock lock(_mutex);

    if (_root->getNumChildren() > 0)
    {
        _root->removeChildren(0, _root->getNumChildren());
    }

    if ( getMapNode() )
    {                        
        _root->addChild(createNode());
        _dirty = false;
        // image overlay is unlit by default.
        setDefaultLighting(false);
    }
}

void
ImageOverlay::setMapNode( MapNode* mapNode )
{
    if ( getMapNode() != mapNode )
    {
        AnnotationNode::setMapNode( mapNode );
        compile();
    }
}

bool
ImageOverlay::getDraped() const
{
    return static_cast< const DrapeableNode *>( getChild(0))->getDrapingEnabled();
}

void
ImageOverlay::setDraped( bool draped )
{
    _draped = draped;
    static_cast< DrapeableNode *>( getChild(0))->setDrapingEnabled( draped );
}

osg::Image*
ImageOverlay::getImage() const
{
    return _image.get();
}

void ImageOverlay::setImage( osg::Image* image )
{
    if (_image != image)
    {
        _image = image;
        dirty();        
    }
}

osg::Texture::FilterMode
    ImageOverlay::getMinFilter() const
{
    return *_minFilter;
}

void
ImageOverlay::setMinFilter( osg::Texture::FilterMode filter )
{
    _minFilter = filter;
    updateFilters();
}

osg::Texture::FilterMode
    ImageOverlay::getMagFilter() const
{
    return *_magFilter;
}

void
ImageOverlay::setMagFilter( osg::Texture::FilterMode filter )
{
    _magFilter = filter; 
    updateFilters();
}


// Bilinear interpolate a 2d position from texture coordinates
namespace
{
    osg::Vec2d bilinearInterpolate(double s, double t, const osg::Vec2d& lowerLeft, const osg::Vec2d& lowerRight, const osg::Vec2d& upperLeft, const osg::Vec2d& upperRight)
    {
        s = osg::clampBetween(s, 0.0, 1.0);
        t = osg::clampBetween(t, 0.0, 1.0);
        osg::Vec2d r1 = lowerLeft * (1.0 - s) + lowerRight * s;
        osg::Vec2d r2 = upperLeft * (1.0 - s) + upperRight * s;
        osg::Vec2d result = r1 * (1.0 - t) + r2 * t;
        return result;
    }
}

osg::Node* ImageOverlay::createNode()
{
    const SpatialReference* mapSRS = getMapNode()->getMapSRS();
    auto geoSRS = mapSRS->getGeodeticSRS();

    osg::MatrixTransform* transform = new osg::MatrixTransform;

    osg::Geometry* geometry = new osg::Geometry();
    geometry->setName(typeid(*this).name());
    geometry->setUseVertexBufferObjects(true);

    transform->addChild(geometry);


    double targetDegrees = _geometryResolution.as(Units::DEGREES);

    osgEarth::Bounds bounds;

    double minX = osg::minimum(_lowerLeft.x(), osg::minimum(_lowerRight.x(), osg::minimum(_upperLeft.x(), _upperRight.x())));
    double minY = osg::minimum(_lowerLeft.y(), osg::minimum(_lowerRight.y(), osg::minimum(_upperLeft.y(), _upperRight.y())));
    double maxX = osg::maximum(_lowerLeft.x(), osg::maximum(_lowerRight.x(), osg::maximum(_upperLeft.x(), _upperRight.x())));
    double maxY = osg::maximum(_lowerLeft.y(), osg::maximum(_lowerRight.y(), osg::maximum(_upperLeft.y(), _upperRight.y())));

    int numCols = osg::maximum(2, (int)((maxX - minX) / targetDegrees) + 1);
    int numRows = osg::maximum(2, (int)((maxY - minY) / targetDegrees) + 1);

    float dx = 1.0 / (float)(numCols - 1);
    float dy = 1.0 / (float)(numRows - 1);

    osg::Vec3Array* verts = new osg::Vec3Array();
    verts->reserve(numCols * numRows);
    geometry->setVertexArray(verts);

    bool flip = false;
    if (_image.valid())
    {
        //Create the texture
        _texture = new osg::Texture2D(_image.get());
        _texture->setWrap(_texture->WRAP_S, _texture->CLAMP_TO_EDGE);
        _texture->setWrap(_texture->WRAP_T, _texture->CLAMP_TO_EDGE);
        _texture->setResizeNonPowerOfTwoHint(false);
        updateFilters();
        transform->getOrCreateStateSet()->setTextureAttributeAndModes(0, _texture, osg::StateAttribute::ON);
        flip = _image->getOrigin() == osg::Image::TOP_LEFT;
    }

    osg::Vec2Array* texcoords = new osg::Vec2Array();
    texcoords->reserve(numCols * numRows);
    geometry->setTexCoordArray(0, texcoords);

    osg::Vec3d anchor;
    bool anchorSet = false;

    for (unsigned int r = 0; r < numRows; ++r)
    {
        float t = (float)r * dy;
        for (unsigned int c = 0; c < numCols; ++c)
        {
            float s = (float)c * dx;
            osg::Vec2d coord = bilinearInterpolate(s, t, _lowerLeft, _lowerRight, _upperLeft, _upperRight);
            
            osg::Vec3d map, world;
            geoSRS->transform(osg::Vec3d(coord.x(), coord.y(), 0.0), mapSRS, map);
            mapSRS->transformToWorld(map, world);

            if (!anchorSet)
            {
                anchor = world;
                anchorSet = true;
            }
            verts->push_back(world - anchor);
            texcoords->push_back(osg::Vec2(s, flip ? 1.0 - t : t));
        }
    }

    transform->setMatrix(osg::Matrixd::translate(anchor));

    // Generate the triangles
    osg::DrawElementsUInt* de = new osg::DrawElementsUInt(GL_TRIANGLES);
    geometry->addPrimitiveSet(de);

    for (unsigned int r = 0; r < numRows - 1; ++r)
    {
        for (unsigned int c = 0; c < numCols - 1; ++c)
        {
            unsigned int ll = r * numCols + c;
            unsigned int lr = ll + 1;
            unsigned int ul = ll + numCols;
            unsigned int ur = ul + 1;
            de->push_back(ll);  de->push_back(lr); de->push_back(ul);
            de->push_back(lr);  de->push_back(ur); de->push_back(ul);
        }
    }




    return transform;
}

void
ImageOverlay::updateFilters()
{
    if (_texture)
    {
        _texture->setFilter(osg::Texture::MAG_FILTER, *_magFilter);

        
        if (ImageUtils::isPowerOfTwo( _image.get() ) && !(!_image->isMipmap() && ImageUtils::isCompressed(_image.get())))
        {
            _texture->setFilter(osg::Texture::MIN_FILTER, *_minFilter);
        }
        else
        {
            if (*_minFilter == osg::Texture2D::NEAREST_MIPMAP_LINEAR || 
                *_minFilter == osg::Texture2D::NEAREST_MIPMAP_NEAREST)
            {
                _texture->setFilter(osg::Texture::MIN_FILTER, osg::Texture2D::NEAREST);
            }
            else if (*_minFilter == osg::Texture2D::LINEAR_MIPMAP_LINEAR || 
                     *_minFilter == osg::Texture2D::LINEAR_MIPMAP_NEAREST)
            {
                _texture->setFilter(osg::Texture::MIN_FILTER, osg::Texture2D::LINEAR);
            }
            else
            {
                _texture->setFilter(osg::Texture::MIN_FILTER, *_minFilter);
            }
        }        
        
        _texture->setFilter(osg::Texture::MAG_FILTER, *_magFilter);

    }
}

float
ImageOverlay::getAlpha() const
{
    return *_alpha;
}

void
ImageOverlay::setAlpha(float alpha)
{
    if (*_alpha != alpha)
    {
        _alpha = osg::clampBetween(alpha, 0.0f, 1.0f);
        _root->getOrCreateStateSet()->getOrCreateUniform("oe_ImageOverlay_alpha", osg::Uniform::FLOAT)->set(*_alpha);
    }
}

void
ImageOverlay::clampLatitudes()
{
    clampLatitude( _lowerLeft );
    clampLatitude( _lowerRight );
    clampLatitude( _upperLeft );
    clampLatitude( _upperRight );
}


osg::Vec2d
ImageOverlay::getCenter() const
{
    return (_lowerLeft + _lowerRight + _upperRight + _upperLeft) / 4.0;
}

void
ImageOverlay::setCenter(double lon_deg, double lat_deg)
{
    osg::Vec2d center = getCenter();
    osg::Vec2d newCenter(lon_deg, lat_deg);
    osg::Vec2d offset =  newCenter - center;
    setCorners(_lowerLeft += offset, _lowerRight += offset,
               _upperLeft += offset, _upperRight += offset);    
}

void
ImageOverlay::setNorth(double value_deg)
{
    _upperRight.y() = value_deg;
    _upperLeft.y()  = value_deg;
    clampLatitudes();
    dirty();
}

void
ImageOverlay::setSouth(double value_deg)
{
    _lowerRight.y() = value_deg;
    _lowerLeft.y() = value_deg;
    clampLatitudes();
    dirty();
}

void
ImageOverlay::setEast(double value_deg)
{
    while (value_deg < _upperLeft.x())
        value_deg += 360.0;

    _upperRight.x() = value_deg;
    _lowerRight.x() = value_deg;
    dirty();
}

void
ImageOverlay::setWest(double value_deg)
{
    while (value_deg > _upperRight.x())
        value_deg -= 360.0;

    _lowerLeft.x() = value_deg;
    _upperLeft.x() = value_deg;
    dirty();
}

void
ImageOverlay::setCorners(const osg::Vec2d& lowerLeft, const osg::Vec2d& lowerRight, 
        const osg::Vec2d& upperLeft, const osg::Vec2d& upperRight)
{
    _lowerLeft = lowerLeft;
    _lowerRight = lowerRight;
    _upperLeft = upperLeft;
    _upperRight = upperRight;    
    clampLatitudes();
    
    dirty();
}

osgEarth::Bounds
ImageOverlay::getBounds() const
{
    osgEarth::Bounds bounds;
    bounds.expandBy(_lowerLeft.x(), _lowerLeft.y(), bounds.zMin());
    bounds.expandBy(_lowerRight.x(), _lowerRight.y(), bounds.zMin());
    bounds.expandBy(_upperLeft.x(), _upperLeft.y(), bounds.zMin());
    bounds.expandBy(_upperRight.x(), _upperRight.y(), bounds.zMin());
    return bounds;
}

void ImageOverlay::setBounds(const osgEarth::Bounds &b)
{
    if (getMapNode())
    {
        GeoExtent e(getMapNode()->getMapSRS(), b);

        setCorners(
            osg::Vec2d(e.xMin(), e.yMin()),
            osg::Vec2d(e.xMax(), e.yMin()),
            osg::Vec2d(e.xMin(), e.yMax()),
            osg::Vec2d(e.xMax(), e.yMax()));
    }
    else
    {
        double w = b.xMax() - b.xMin();
        setCorners(
            osg::Vec2d(b.xMin(), b.yMin()),
            osg::Vec2d(b.xMin() + w, b.yMin()),
            osg::Vec2d(b.xMin(), b.yMax()), 
            osg::Vec2d(b.xMin() + w, b.yMax()));
    }
}

void
ImageOverlay::setBoundsAndRotation(const osgEarth::Bounds& b, const Angular& rot)
{
    double rot_rad = rot.as(Units::RADIANS);

    if ( osg::equivalent( rot_rad, 0.0 ) )    
    {
        setBounds(b);
    }
    else
    {
        double w = b.xMax() - b.xMin();
        osg::Vec3d ll(b.xMin(), b.yMin(), 0);
        osg::Vec3d ul(b.xMin(), b.yMax(), 0);
        osg::Vec3d ur(b.xMin() + w, b.yMax(), 0);
        osg::Vec3d lr(b.xMin() + w, b.yMin(), 0);

        osg::Vec3d center = b.center();
        // Rotate around the center point
        osg::Matrixd transform = osg::Matrixd::translate(-center) * osg::Matrixd::rotate(rot_rad, osg::Vec3d(0, 0, 1)) * osg::Matrixd::translate(center);

        ll = ll * transform;
        ul = ul * transform;
        ur = ur * transform;;
        lr = lr * transform;;
        setCorners(
            osg::Vec2d(ll.x(), ll.y()),
            osg::Vec2d(lr.x(), lr.y()),
            osg::Vec2d(ul.x(), ul.y()),
            osg::Vec2d(ur.x(), ur.y()));
    }
}

void
ImageOverlay::setLowerLeft(double lon_deg, double lat_deg)
{
    _lowerLeft = osg::Vec2d(lon_deg, lat_deg);
    clampLatitudes();
    dirty();    
}

void
ImageOverlay::setLowerRight(double lon_deg, double lat_deg)
{
    _lowerRight = osg::Vec2d(lon_deg, lat_deg);
    clampLatitudes();
    dirty();    
}

void
ImageOverlay::setUpperRight(double lon_deg, double lat_deg)
{
    _upperRight = osg::Vec2d(lon_deg, lat_deg);
    clampLatitudes();
    dirty();
}

void
ImageOverlay::setUpperLeft(double lon_deg, double lat_deg)
{
    _upperLeft = osg::Vec2d(lon_deg, lat_deg);
    clampLatitudes();
    dirty();
}

osg::Vec2d
ImageOverlay::getControlPoint(ControlPoint controlPoint)
{
    switch (controlPoint)
    {
    case CONTROLPOINT_CENTER:
        return getCenter();
    case CONTROLPOINT_UPPER_LEFT:
        return getUpperLeft();
    case CONTROLPOINT_LOWER_LEFT:
        return getLowerLeft();
    case CONTROLPOINT_UPPER_RIGHT:
        return getUpperRight();
    case CONTROLPOINT_LOWER_RIGHT:
        return getLowerRight();
    default:
        return getCenter();
    }       
}

void
ImageOverlay::setControlPoint(ControlPoint controlPoint, double lon_deg, double lat_deg,  bool singleVert)
{
    switch (controlPoint)
    {
    case CONTROLPOINT_CENTER:
        return setCenter(lon_deg, lat_deg);
        break;
    case CONTROLPOINT_UPPER_LEFT:
        if (singleVert)
        {
            setUpperLeft(lon_deg, lat_deg);
        }
        else
        {
            setNorth(lat_deg);
            setWest(lon_deg);
        }
        break;
    case CONTROLPOINT_LOWER_LEFT:
        if (singleVert)
        {
            setLowerLeft(lon_deg, lat_deg);
        }
        else
        {
            setSouth(lat_deg);
            setWest(lon_deg);
        }
        break;
    case CONTROLPOINT_UPPER_RIGHT:
        if (singleVert)
        {
            setUpperRight(lon_deg, lat_deg);
        }
        else
        {
            setNorth( lat_deg);
            setEast( lon_deg );
        }
        break;
    case CONTROLPOINT_LOWER_RIGHT:
        if (singleVert)
        {
            setLowerRight(lon_deg, lat_deg);
        }
        else
        {
            setSouth( lat_deg );
            setEast( lon_deg );
        }
        break;
    }
}

void
ImageOverlay::traverse(osg::NodeVisitor &nv)
{
    if (nv.getVisitorType() == nv.EVENT_VISITOR)
    {
        if (_dirty == true && _updateScheduled == false)
        {
            _updateScheduled = true;
            ADJUST_UPDATE_TRAV_COUNT(this, +1);
        }
    }

    else if (nv.getVisitorType() == nv.UPDATE_VISITOR)
    {
        if (_dirty)
        {
            compile();
        }

        if (_updateScheduled)
        {
            _updateScheduled = false;
            ADJUST_UPDATE_TRAV_COUNT(this, -1);
        }
    }

    AnnotationNode::traverse(nv);
}

void ImageOverlay::dirty()
{
    {
        Threading::ScopedMutexLock lock(_mutex);
        _dirty = true;
    }

    for( CallbackList::iterator i = _callbacks.begin(); i != _callbacks.end(); i++ )
    {
        i->get()->onOverlayChanged();
    }
}

void 
ImageOverlay::addCallback( ImageOverlayCallback* cb )
{
    if ( cb )
        this->_callbacks.push_back( cb );
}

void 
ImageOverlay::removeCallback( ImageOverlayCallback* cb )
{
    CallbackList::iterator i = std::find( _callbacks.begin(), _callbacks.end(), cb);
    if (i != _callbacks.end())
    {
        _callbacks.erase( i );
    }    
}
