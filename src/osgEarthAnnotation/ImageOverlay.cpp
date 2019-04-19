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
#include <osgEarthAnnotation/ImageOverlay>
#include <osgEarthAnnotation/AnnotationRegistry>
#include <osgEarthSymbology/MeshSubdivider>
#include <osgEarthFeatures/GeometryUtils>
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
using namespace osgEarth::Annotation;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;


//---------------------------------------------------------------------------

namespace
{
    void clampLatitude(osg::Vec2d& l)
    {
        l.y() = osg::clampBetween( l.y(), -90.0, 90.0);
    }    

    static Distance default_geometryResolution(5.0, Units::DEGREES);

    const char* imageVS =
        "#version " GLSL_VERSION_STR "\n"
        "out vec2 oe_ImageOverlay_texcoord; \n"
        "void oe_ImageOverlay_VS(inout vec4 vertex) { \n"
        "    oe_ImageOverlay_texcoord = gl_MultiTexCoord0.st; \n"
        "} \n";

    const char* imageFS =
        "#version " GLSL_VERSION_STR "\n"
        "in vec2 oe_ImageOverlay_texcoord; \n"
        "uniform sampler2D oe_ImageOverlay_tex; \n"
        "uniform float oe_ImageOverlay_alpha; \n"
        "void oe_ImageOverlay_FS(inout vec4 color) { \n"
        "    color = texture(oe_ImageOverlay_tex, oe_ImageOverlay_texcoord);\n"
        "    color.a *= oe_ImageOverlay_alpha; \n"
        "} \n";

}

//---------------------------------------------------------------------------

OSGEARTH_REGISTER_ANNOTATION( imageoverlay, osgEarth::Annotation::ImageOverlay );

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

    conf.get( "alpha", _alpha );
    
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
        static Threading::Mutex mutex;
        mutex.lock();
        if (_program.valid() == false)
        {
            _program = new VirtualProgram;
            _program->setInheritShaders(true);
            _program->setFunction("oe_ImageOverlay_VS", imageVS, ShaderComp::LOCATION_VERTEX_MODEL);
            _program->setFunction("oe_ImageOverlay_FS", imageFS, ShaderComp::LOCATION_FRAGMENT_COLORING);
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
    OpenThreads::ScopedLock< OpenThreads::Mutex > lock(_mutex);

    if (_root->getNumChildren() > 0)
    {
        _root->removeChildren(0, _root->getNumChildren());
    }

    if ( getMapNode() )
    {                
        const SpatialReference* mapSRS = getMapNode()->getMapSRS();

        osg::ref_ptr<Feature> f = new Feature( new Polygon(), mapSRS->getGeodeticSRS() );
        Geometry* g = f->getGeometry();
        g->push_back( osg::Vec3d(_lowerLeft.x(),  _lowerLeft.y(), 0) );
        g->push_back( osg::Vec3d(_lowerRight.x(), _lowerRight.y(), 0) );
        g->push_back( osg::Vec3d(_upperRight.x(), _upperRight.y(), 0) );
        g->push_back( osg::Vec3d(_upperLeft.x(),  _upperLeft.y(),  0) );

        osgEarth::Bounds bounds = getBounds();

        FeatureList features;
        if (!mapSRS->isGeographic())        
        {
            f->splitAcrossDateLine(features);
        }
        // The width of the image overlay is >= 180 degrees so split it into two chunks of < 180 degrees
        // so the MeshSubdivider will work.
        else if (bounds.width() > 180.0)
        {
            Bounds boundsA(bounds.xMin(), bounds.yMin(), bounds.xMin() + 180.0, bounds.yMax());
            Bounds boundsB(bounds.xMin() + 180.0, bounds.yMin(), bounds.xMax(), bounds.yMax());
            
            osg::ref_ptr< Geometry > geomA;
            if (f->getGeometry()->crop(boundsA, geomA))
            {
                osg::ref_ptr< Feature > croppedFeature = new Feature(*f);
                // Make sure the feature is wound correctly.
                geomA->rewind(osgEarth::Symbology::Geometry::ORIENTATION_CCW);
                croppedFeature->setGeometry(geomA.get());
                features.push_back(croppedFeature);
            }
            osg::ref_ptr< Geometry > geomB;
            if (f->getGeometry()->crop(boundsB, geomB))
            {
                osg::ref_ptr< Feature > croppedFeature = new Feature(*f);
                // Make sure the feature is wound correctly.
                geomA->rewind(osgEarth::Symbology::Geometry::ORIENTATION_CCW);
                croppedFeature->setGeometry(geomB.get());
                features.push_back(croppedFeature);
            }
        }
        else
        {
            features.push_back( f );
        }

        for (FeatureList::iterator itr = features.begin(); itr != features.end(); ++itr)
        {
            _root->addChild(createNode(itr->get(), features.size() > 1));
        }

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

osg::Node* ImageOverlay::createNode(Feature* feature, bool split)
{    
    const SpatialReference* mapSRS = getMapNode()->getMapSRS();

    osg::MatrixTransform* transform = new osg::MatrixTransform;
    
    //osg::Geode* geode = new osg::Geode;
    //transform->addChild(geode);

    osg::Geometry* geometry = new osg::Geometry();     
    geometry->setUseVertexBufferObjects(true);
    transform->addChild(geometry);

    // next, convert to world coords and create the geometry:
    osg::Vec3Array* verts = new osg::Vec3Array();
    verts->reserve(4);
    osg::Vec3d anchor;
    for( Geometry::iterator i = feature->getGeometry()->begin(); i != feature->getGeometry()->end(); ++i )
    {        
        osg::Vec3d map, world;        
        feature->getSRS()->transform( *i, mapSRS, map);
        mapSRS->transformToWorld( map, world );
        if (i == feature->getGeometry()->begin())
        {
            anchor = world;
        }
        verts->push_back( world - anchor );
    }    

    transform->setMatrix( osg::Matrixd::translate( anchor ) );

    geometry->setVertexArray( verts );
    if ( verts->getVertexBufferObject() )
        verts->getVertexBufferObject()->setUsage(GL_STATIC_DRAW_ARB);

    GLushort tris[6] = { 0, 1, 2,
        0, 2, 3
    };        
    geometry->addPrimitiveSet(new osg::DrawElementsUShort( GL_TRIANGLES, 6, tris ) );

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
        flip = _image->getOrigin()==osg::Image::TOP_LEFT;
    }

    osg::Vec2Array* texcoords = new osg::Vec2Array(4);



    if (split)
    {
        // If the feature has been split across the antimerdian we have to figure out new texture coordinates, we can't just just use the corners.
        // This code is limited in that it only works with rectangular images though, so overlays that are non axis aligned and split across the antimerdian could look wrong
        double width = _upperRight.x() - _lowerLeft.x();
        double height = _upperRight.y() - _lowerLeft.y();

        for (unsigned int i = 0; i < feature->getGeometry()->size(); ++i)
        {
            osg::Vec3d v = (*feature->getGeometry())[i];

            if (v.x() < _lowerLeft.x())
            {
                v.x() += 360.0;
            }
            if (v.x() > _upperRight.x())
            {
                v.x() -= 360.0;
            }

            float s = (v.x() - _lowerLeft.x()) / width;
            float t = (v.y() - _lowerLeft.y()) / height;
            (*texcoords)[i].set(s, flip ? 1.0f - t : t);
        }
    }
    else
    {
        (*texcoords)[0].set(0.0f, flip ? 1.0 : 0.0f);
        (*texcoords)[1].set(1.0f, flip ? 1.0 : 0.0f);
        (*texcoords)[2].set(1.0f, flip ? 0.0 : 1.0f);
        (*texcoords)[3].set(0.0f, flip ? 0.0 : 1.0f);
    }
    geometry->setTexCoordArray(0, texcoords);    

    //Only run the MeshSubdivider on geocentric maps
    if (getMapNode()->getMap()->isGeocentric())
    {
        MeshSubdivider ms(osg::Matrixd::inverse(transform->getMatrix()), transform->getMatrix());
        ms.run(*geometry, _geometryResolution.as(Units::RADIANS), GEOINTERP_RHUMB_LINE);
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
    _upperRight.x() = value_deg;
    _lowerRight.x() = value_deg;
    dirty();
}

void
ImageOverlay::setWest(double value_deg)
{
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
    bounds.expandBy(_lowerLeft.x(), _lowerLeft.y());
    bounds.expandBy(_lowerRight.x(), _lowerRight.y());
    bounds.expandBy(_upperLeft.x(), _upperLeft.y());
    bounds.expandBy(_upperRight.x(), _upperRight.y());
    return bounds;
}

void ImageOverlay::setBounds(const osgEarth::Bounds &extent)
{
    setCorners(osg::Vec2d(extent.xMin(), extent.yMin()), osg::Vec2d(extent.xMax(), extent.yMin()),
               osg::Vec2d(extent.xMin(), extent.yMax()), osg::Vec2d(extent.xMax(), extent.yMax()));
}

void
ImageOverlay::setBoundsAndRotation(const osgEarth::Bounds& b, const Angular& rot)
{
    double rot_rad = rot.as(Units::RADIANS);

    if ( osg::equivalent( rot_rad, 0.0 ) )
    {
        setBounds( b );
    }
    else
    {
        osg::Vec3d ll( b.xMin(), b.yMin(), 0 );
        osg::Vec3d ul( b.xMin(), b.yMax(), 0 );
        osg::Vec3d ur( b.xMax(), b.yMax(), 0 );
        osg::Vec3d lr( b.xMax(), b.yMin(), 0 );

        double sinR = sin(-rot_rad), cosR = cos(-rot_rad);

        osg::Vec3d c( 0.5*(b.xMax()+b.xMin()), 0.5*(b.yMax()+b.yMin()), 0);

        // there must be a better way, but my internet is down so i can't look it up with now..

        osg::ref_ptr<const SpatialReference> srs = SpatialReference::create("wgs84");
        osg::ref_ptr<const SpatialReference> utm = srs->createUTMFromLonLat( c.x(), c.y() );

        osg::Vec3d ll_utm, ul_utm, ur_utm, lr_utm, c_utm;
        
        srs->transform( ll, utm.get(), ll_utm );
        srs->transform( ul, utm.get(), ul_utm );
        srs->transform( ur, utm.get(), ur_utm );
        srs->transform( lr, utm.get(), lr_utm );
        srs->transform( c,  utm.get(), c_utm  );

        osg::Vec3d llp( cosR*(ll_utm.x()-c_utm.x()) - sinR*(ll_utm.y()-c_utm.y()), sinR*(ll_utm.x()-c_utm.x()) + cosR*(ll_utm.y()-c_utm.y()), 0 );
        osg::Vec3d ulp( cosR*(ul_utm.x()-c_utm.x()) - sinR*(ul_utm.y()-c_utm.y()), sinR*(ul_utm.x()-c_utm.x()) + cosR*(ul_utm.y()-c_utm.y()), 0 );
        osg::Vec3d urp( cosR*(ur_utm.x()-c_utm.x()) - sinR*(ur_utm.y()-c_utm.y()), sinR*(ur_utm.x()-c_utm.x()) + cosR*(ur_utm.y()-c_utm.y()), 0 );
        osg::Vec3d lrp( cosR*(lr_utm.x()-c_utm.x()) - sinR*(lr_utm.y()-c_utm.y()), sinR*(lr_utm.x()-c_utm.x()) + cosR*(lr_utm.y()-c_utm.y()), 0 );    

        utm->transform( (llp+c_utm), srs.get(), ll );
        utm->transform( (ulp+c_utm), srs.get(), ul );
        utm->transform( (urp+c_utm), srs.get(), ur );
        utm->transform( (lrp+c_utm), srs.get(), lr );

        setCorners( 
            osg::Vec2d(ll.x(), ll.y()), 
            osg::Vec2d(lr.x(), lr.y()),
            osg::Vec2d(ul.x(), ul.y()),
            osg::Vec2d(ur.x(), ur.y()) );
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
        OpenThreads::ScopedLock< OpenThreads::Mutex > lock(_mutex);
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
