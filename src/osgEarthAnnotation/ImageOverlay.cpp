/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2014 Pelican Mapping
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
#include <osgEarthAnnotation/ImageOverlay>
#include <osgEarthAnnotation/AnnotationRegistry>
#include <osgEarthSymbology/MeshSubdivider>
#include <osgEarthFeatures/GeometryUtils>
#include <osgEarthFeatures/MeshClamper>
#include <osgEarthFeatures/Feature>
#include <osgEarth/MapNode>
#include <osgEarth/NodeUtils>
#include <osgEarth/ImageUtils>
#include <osgEarth/DrapeableNode>
#include <osgEarth/VirtualProgram>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/ShaderGenerator>
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
}

//---------------------------------------------------------------------------

OSGEARTH_REGISTER_ANNOTATION( imageoverlay, osgEarth::Annotation::ImageOverlay );

ImageOverlay::ImageOverlay(MapNode* mapNode, const Config& conf, const osgDB::Options* dbOptions) :
AnnotationNode(mapNode, conf),
_lowerLeft    (10, 10),
_lowerRight   (20, 10),
_upperRight   (20, 20),
_upperLeft    (10, 20),
_dirty        (false),
_alpha        (1.0f),
_minFilter    (osg::Texture::LINEAR_MIPMAP_LINEAR),
_magFilter    (osg::Texture::LINEAR),
_texture      (0)
{
    conf.getIfSet( "url",   _imageURI );
    if ( _imageURI.isSet() )
    {
        setImage( _imageURI->getImage(dbOptions) );
    }

    conf.getIfSet( "alpha", _alpha );
    
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
    conf.getIfSet("mag_filter","LINEAR",                _magFilter,osg::Texture::LINEAR);
    conf.getIfSet("mag_filter","LINEAR_MIPMAP_LINEAR",  _magFilter,osg::Texture::LINEAR_MIPMAP_LINEAR);
    conf.getIfSet("mag_filter","LINEAR_MIPMAP_NEAREST", _magFilter,osg::Texture::LINEAR_MIPMAP_NEAREST);
    conf.getIfSet("mag_filter","NEAREST",               _magFilter,osg::Texture::NEAREST);
    conf.getIfSet("mag_filter","NEAREST_MIPMAP_LINEAR", _magFilter,osg::Texture::NEAREST_MIPMAP_LINEAR);
    conf.getIfSet("mag_filter","NEAREST_MIPMAP_NEAREST",_magFilter,osg::Texture::NEAREST_MIPMAP_NEAREST);
    conf.getIfSet("min_filter","LINEAR",                _minFilter,osg::Texture::LINEAR);
    conf.getIfSet("min_filter","LINEAR_MIPMAP_LINEAR",  _minFilter,osg::Texture::LINEAR_MIPMAP_LINEAR);
    conf.getIfSet("min_filter","LINEAR_MIPMAP_NEAREST", _minFilter,osg::Texture::LINEAR_MIPMAP_NEAREST);
    conf.getIfSet("min_filter","NEAREST",               _minFilter,osg::Texture::NEAREST);
    conf.getIfSet("min_filter","NEAREST_MIPMAP_LINEAR", _minFilter,osg::Texture::NEAREST_MIPMAP_LINEAR);
    conf.getIfSet("min_filter","NEAREST_MIPMAP_NEAREST",_minFilter,osg::Texture::NEAREST_MIPMAP_NEAREST);

    postCTOR();
}

Config
ImageOverlay::getConfig() const
{
    Config conf("imageoverlay");
    conf.set("name",  getName());

    if ( _imageURI.isSet() )
    {
        conf.addIfSet("url", _imageURI );
    }
    else if ( _image.valid() && !_image->getFileName().empty() )
    {
        optional<URI> temp;
        temp = URI(_image->getFileName());
        conf.addIfSet("url", temp);
    }

    conf.addIfSet("alpha", _alpha);

    osg::ref_ptr<Geometry> g = new Polygon();
    g->push_back( osg::Vec3d(_lowerLeft.x(),  _lowerLeft.y(), 0) );
    g->push_back( osg::Vec3d(_lowerRight.x(), _lowerRight.y(), 0) );
    g->push_back( osg::Vec3d(_upperRight.x(), _upperRight.y(), 0) );
    g->push_back( osg::Vec3d(_upperLeft.x(),  _upperLeft.y(),  0) );

    Config geomConf("geometry");
    geomConf.value() = GeometryUtils::geometryToWKT( g.get() );
    conf.add( geomConf );

    //Save the filter settings
	conf.updateIfSet("mag_filter","LINEAR",                _magFilter,osg::Texture::LINEAR);
    conf.updateIfSet("mag_filter","LINEAR_MIPMAP_LINEAR",  _magFilter,osg::Texture::LINEAR_MIPMAP_LINEAR);
    conf.updateIfSet("mag_filter","LINEAR_MIPMAP_NEAREST", _magFilter,osg::Texture::LINEAR_MIPMAP_NEAREST);
    conf.updateIfSet("mag_filter","NEAREST",               _magFilter,osg::Texture::NEAREST);
    conf.updateIfSet("mag_filter","NEAREST_MIPMAP_LINEAR", _magFilter,osg::Texture::NEAREST_MIPMAP_LINEAR);
    conf.updateIfSet("mag_filter","NEAREST_MIPMAP_NEAREST",_magFilter,osg::Texture::NEAREST_MIPMAP_NEAREST);
    conf.updateIfSet("min_filter","LINEAR",                _minFilter,osg::Texture::LINEAR);
    conf.updateIfSet("min_filter","LINEAR_MIPMAP_LINEAR",  _minFilter,osg::Texture::LINEAR_MIPMAP_LINEAR);
    conf.updateIfSet("min_filter","LINEAR_MIPMAP_NEAREST", _minFilter,osg::Texture::LINEAR_MIPMAP_NEAREST);
    conf.updateIfSet("min_filter","NEAREST",               _minFilter,osg::Texture::NEAREST);
    conf.updateIfSet("min_filter","NEAREST_MIPMAP_LINEAR", _minFilter,osg::Texture::NEAREST_MIPMAP_LINEAR);
    conf.updateIfSet("min_filter","NEAREST_MIPMAP_NEAREST",_minFilter,osg::Texture::NEAREST_MIPMAP_NEAREST);

    return conf;
}

//---------------------------------------------------------------------------


ImageOverlay::ImageOverlay(MapNode* mapNode, osg::Image* image) :
AnnotationNode(mapNode),
_lowerLeft    (10, 10),
_lowerRight   (20, 10),
_upperRight   (20, 20),
_upperLeft    (10, 20),
_image        (image),
_dirty        (false),
_alpha        (1.0f),
_minFilter    (osg::Texture::LINEAR_MIPMAP_LINEAR),
_magFilter    (osg::Texture::LINEAR),
_texture      (0)
{        
    postCTOR();
}

void
ImageOverlay::postCTOR()
{
    _geode = new osg::Geode;

    _transform = new osg::MatrixTransform;
    _transform->addChild( _geode );

    // place the geometry under a drapeable node so it will project onto the terrain    
    DrapeableNode* d = new DrapeableNode( getMapNode() );
    addChild( d );

    d->addChild( _transform );

    init();

    ADJUST_UPDATE_TRAV_COUNT( this, 1 );
}

void
ImageOverlay::init()
{
    OpenThreads::ScopedLock< OpenThreads::Mutex > lock(_mutex);

    _geode->removeDrawables(0, _geode->getNumDrawables() );

    if ( getMapNode() )
    {
        double height = 0;
        osg::Geometry* geometry = new osg::Geometry();
        geometry->setUseVertexBufferObjects(true);

        const osg::EllipsoidModel* ellipsoid = getMapNode()->getMapSRS()->getEllipsoid();

        const SpatialReference* mapSRS = getMapNode()->getMapSRS();

        // calculate a bounding polytope in world space (for mesh clamping):
        osg::ref_ptr<Feature> f = new Feature( new Polygon(), mapSRS->getGeodeticSRS() );
        Geometry* g = f->getGeometry();
        g->push_back( osg::Vec3d(_lowerLeft.x(),  _lowerLeft.y(), 0) );
        g->push_back( osg::Vec3d(_lowerRight.x(), _lowerRight.y(), 0) );
        g->push_back( osg::Vec3d(_upperRight.x(), _upperRight.y(), 0) );
        g->push_back( osg::Vec3d(_upperLeft.x(),  _upperLeft.y(),  0) );
        //_boundingPolytope = f->getWorldBoundingPolytope();
        f->getWorldBoundingPolytope( getMapNode()->getMapSRS(), _boundingPolytope );

        // next, convert to world coords and create the geometry:
        osg::Vec3Array* verts = new osg::Vec3Array();
        verts->reserve(4);
        osg::Vec3d anchor;
        for( Geometry::iterator i = g->begin(); i != g->end(); ++i )
        {        
            osg::Vec3d map, world;        
            f->getSRS()->transform( *i, mapSRS, map);
            mapSRS->transformToWorld( map, world );
            if (i == g->begin())
            {
                anchor = world;
            }
            verts->push_back( world - anchor );
        }
        
        _transform->setMatrix( osg::Matrixd::translate( anchor ) );



        geometry->setVertexArray( verts );
        if ( verts->getVertexBufferObject() )
            verts->getVertexBufferObject()->setUsage(GL_STATIC_DRAW_ARB);

        osg::Vec4Array* colors = new osg::Vec4Array(1);
        (*colors)[0] = osg::Vec4(1,1,1,*_alpha);

        geometry->setColorArray( colors );
        geometry->setColorBinding( osg::Geometry::BIND_OVERALL );

         GLushort tris[6] = { 0, 1, 2,
                            0, 2, 3
                          };        
        geometry->addPrimitiveSet(new osg::DrawElementsUShort( GL_TRIANGLES, 6, tris ) );

        bool flip = false;
        if (_image.valid())
        {
            //Create the texture
            _texture = new osg::Texture2D(_image.get());        
            _texture->setResizeNonPowerOfTwoHint(false);
            updateFilters();
            _geode->getOrCreateStateSet()->setTextureAttributeAndModes(0, _texture, osg::StateAttribute::ON);    
            flip = _image->getOrigin()==osg::Image::TOP_LEFT;
        }

        osg::Vec2Array* texcoords = new osg::Vec2Array(4);
        (*texcoords)[0].set(0.0f,flip ? 1.0 : 0.0f);
        (*texcoords)[1].set(1.0f,flip ? 1.0 : 0.0f);
        (*texcoords)[2].set(1.0f,flip ? 0.0 : 1.0f);
        (*texcoords)[3].set(0.0f,flip ? 0.0 : 1.0f);
        geometry->setTexCoordArray(0, texcoords);

         
        //Only run the MeshSubdivider on geocentric maps
        if (getMapNode()->getMap()->isGeocentric())
        {
            MeshSubdivider ms(osg::Matrixd::inverse(_transform->getMatrix()), _transform->getMatrix());
            ms.run(*geometry, osg::DegreesToRadians(5.0), GEOINTERP_RHUMB_LINE);
        }

        _geode->addDrawable( geometry );

        _geometry = geometry;

        _dirty = false;
        
        // Set the annotation up for auto-clamping. We always need to auto-clamp a draped image
        // so that the mesh roughly conforms with the surface, otherwise the draping routine
        // might clip it.
        Style style;
        style.getOrCreate<AltitudeSymbol>()->clamping() = AltitudeSymbol::CLAMP_RELATIVE_TO_TERRAIN;
        applyStyle( style );
        setLightingIfNotSet( false );
        clampMesh( getMapNode()->getTerrain()->getGraph() );

        if ( Registry::capabilities().supportsGLSL() )
        {
            //OE_WARN << LC << "ShaderGen RUNNING" << std::endl;
            Registry::shaderGenerator().run( _geode, "osgEarth.ImageOverlay" );
        }
    }
}

void
ImageOverlay::setMapNode( MapNode* mapNode )
{
    if ( getMapNode() != mapNode )
    {
        AnnotationNode::setMapNode( mapNode );
        init();
    }
}

bool
ImageOverlay::getDraped() const
{
    return static_cast< const DrapeableNode *>( getChild(0))->getDraped();
}

void
ImageOverlay::setDraped( bool draped )
{
    static_cast< DrapeableNode *>( getChild(0))->setDraped( draped );
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

void
ImageOverlay::updateFilters()
{
    if (_texture)
    {
        _texture->setFilter(osg::Texture::MAG_FILTER, *_magFilter);

        
        if (ImageUtils::isPowerOfTwo( _image ) && !(!_image->isMipmap() && ImageUtils::isCompressed(_image)))
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
        dirty();
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
    if (nv.getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR && _dirty)
    {
        init();        
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


void
ImageOverlay::reclamp( const TileKey& key, osg::Node* tile, const Terrain* )
{
    if ( _boundingPolytope.contains( tile->getBound() ) ) // intersects, actually
    {
        clampMesh( tile );
        OE_DEBUG << LC << "Clamped overlay mesh, tile radius = " << tile->getBound().radius() << std::endl;
    }
}

void
ImageOverlay::clampMesh( osg::Node* terrainModel )
{
    double scale  = 1.0;
    double offset = 0.0;
    bool   relative = false;

    if (_altitude.valid())
    {
        if ( _altitude->verticalScale().isSet() )
            scale = _altitude->verticalScale()->eval();

        if ( _altitude->verticalOffset().isSet() )
            offset = _altitude->verticalOffset()->eval();

        relative = _altitude->clamping() == AltitudeSymbol::CLAMP_RELATIVE_TO_TERRAIN;
    }

    MeshClamper clamper( terrainModel, getMapNode()->getMapSRS(), getMapNode()->isGeocentric(), relative, scale, offset );
    this->accept( clamper );

    this->dirtyBound();
}
