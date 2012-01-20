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
#include <osgEarthAnnotation/ImageOverlay>
#include <osgEarthSymbology/MeshSubdivider>
#include <osgEarth/FindNode>
#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/Texture2D>
#include <osgEarth/FindNode>
#include <osg/io_utils>
#include <algorithm>

using namespace osgEarth;
using namespace osgEarth::Annotation;
using namespace osgEarth::Symbology;

/***************************************************************************/

void clampLatitude(osg::Vec2d& l)
{
    l.y() = osg::clampBetween( l.y(), -90.0, 90.0);
}

ImageOverlay::ImageOverlay(MapNode* mapNode, osg::Image* image):
DrapeableNode(mapNode, true),
_lowerLeft(10,10),
_lowerRight(20, 10),
_upperRight(20,20),
_upperLeft(10, 20),
_image(image),
_dirty(false),
_alpha(1.0f)
{        
    postCTOR();
}

void
ImageOverlay::postCTOR()
{
    _geode = new osg::Geode;

    addChild( _geode );

    init();    
    ADJUST_UPDATE_TRAV_COUNT( this, 1 );
}

void
ImageOverlay::init()
{
    OpenThreads::ScopedLock< OpenThreads::Mutex > lock(_mutex);    

    double height = 0;
    osg::Geometry* geometry = new osg::Geometry();
    osg::Vec3d ll;
    const osg::EllipsoidModel* ellipsoid = getMapNode()->getMapSRS()->getEllipsoid();
    ellipsoid->convertLatLongHeightToXYZ(osg::DegreesToRadians(_lowerLeft.y()), osg::DegreesToRadians(_lowerLeft.x()), height, ll.x(), ll.y(), ll.z());

    osg::Vec3d lr;
    ellipsoid->convertLatLongHeightToXYZ(osg::DegreesToRadians(_lowerRight.y()), osg::DegreesToRadians(_lowerRight.x()), height, lr.x(), lr.y(), lr.z());

    osg::Vec3d ur;
    ellipsoid->convertLatLongHeightToXYZ(osg::DegreesToRadians(_upperRight.y()), osg::DegreesToRadians(_upperRight.x()), height, ur.x(), ur.y(), ur.z());

    osg::Vec3d ul;
    ellipsoid->convertLatLongHeightToXYZ(osg::DegreesToRadians(_upperLeft.y()), osg::DegreesToRadians(_upperLeft.x()), height, ul.x(), ul.y(), ul.z());


    osg::Vec3Array* verts = new osg::Vec3Array(4);
    (*verts)[0] = ll;
    (*verts)[1] = lr;
    (*verts)[2] = ur;
    (*verts)[3] = ul;
    
    geometry->setVertexArray( verts );

    osg::Vec4Array* colors = new osg::Vec4Array(1);
    (*colors)[0] = osg::Vec4(1,1,1,_alpha);

    geometry->setColorArray( colors );
    geometry->setColorBinding( osg::Geometry::BIND_OVERALL );

     GLuint tris[6] = { 0, 1, 2,
                        0, 2, 3
                      };        
    geometry->addPrimitiveSet(new osg::DrawElementsUInt( GL_TRIANGLES, 6, tris ) );

    bool flip = false;
    if (_image.valid())
    {
        //Create the texture
        osg::Texture2D* texture = new osg::Texture2D(_image.get());
        texture->setResizeNonPowerOfTwoHint(false);
        _geode->getOrCreateStateSet()->setTextureAttributeAndModes(0, texture, osg::StateAttribute::ON);    
        flip = _image->getOrigin()==osg::Image::TOP_LEFT;
    }

    osg::Vec2Array* texcoords = new osg::Vec2Array(4);
    (*texcoords)[0].set(0.0f,flip ? 1.0 : 0.0f);
    (*texcoords)[1].set(1.0f,flip ? 1.0 : 0.0f);
    (*texcoords)[2].set(1.0f,flip ? 0.0 : 1.0f);
    (*texcoords)[3].set(0.0f,flip ? 0.0 : 1.0f);
    geometry->setTexCoordArray(0, texcoords);
        
    MeshSubdivider ms;
    ms.run(*geometry, osg::DegreesToRadians(5.0), GEOINTERP_RHUMB_LINE);

    _geode->removeDrawables(0, _geode->getNumDrawables() );

    _geode->addDrawable( geometry );

    _geometry = geometry;

    _dirty = false;
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

float
ImageOverlay::getAlpha() const
{
    return _alpha;
}

void
ImageOverlay::setAlpha(float alpha)
{
    if (_alpha != alpha)
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
        osg::Vec2d ll( b.xMin(), b.yMin() );
        osg::Vec2d ul( b.xMin(), b.yMax() );
        osg::Vec2d ur( b.xMax(), b.yMax() );
        osg::Vec2d lr( b.xMax(), b.yMin() );

        double sinR = sin(-rot_rad), cosR = cos(-rot_rad);

        osg::Vec2d c( 0.5*(b.xMax()+b.xMin()), 0.5*(b.yMax()+b.yMin()) );

        // there must be a better way, but my internet is down so i can't look it up with now..

        osg::ref_ptr<SpatialReference> srs = SpatialReference::create("wgs84");
        osg::ref_ptr<SpatialReference> utm = srs->createUTMFromLongitude( c.x() );

        osg::Vec2d ll_utm, ul_utm, ur_utm, lr_utm, c_utm;
        srs->transform2D( ll.x(), ll.y(), utm.get(), ll_utm.x(), ll_utm.y() );
        srs->transform2D( ul.x(), ul.y(), utm.get(), ul_utm.x(), ul_utm.y() );
        srs->transform2D( ur.x(), ur.y(), utm.get(), ur_utm.x(), ur_utm.y() );
        srs->transform2D( lr.x(), lr.y(), utm.get(), lr_utm.x(), lr_utm.y() );
        srs->transform2D( c.x(),  c.y(),  utm.get(), c_utm.x(),  c_utm.y()  );

        osg::Vec2d llp( cosR*(ll_utm.x()-c_utm.x()) - sinR*(ll_utm.y()-c_utm.y()), sinR*(ll_utm.x()-c_utm.x()) + cosR*(ll_utm.y()-c_utm.y()) );
        osg::Vec2d ulp( cosR*(ul_utm.x()-c_utm.x()) - sinR*(ul_utm.y()-c_utm.y()), sinR*(ul_utm.x()-c_utm.x()) + cosR*(ul_utm.y()-c_utm.y()) );
        osg::Vec2d urp( cosR*(ur_utm.x()-c_utm.x()) - sinR*(ur_utm.y()-c_utm.y()), sinR*(ur_utm.x()-c_utm.x()) + cosR*(ur_utm.y()-c_utm.y()) );
        osg::Vec2d lrp( cosR*(lr_utm.x()-c_utm.x()) - sinR*(lr_utm.y()-c_utm.y()), sinR*(lr_utm.x()-c_utm.x()) + cosR*(lr_utm.y()-c_utm.y()) );    

        utm->transform2D( (llp+c_utm).x(), (llp+c_utm).y(), srs.get(), ll.x(), ll.y() );
        utm->transform2D( (ulp+c_utm).x(), (ulp+c_utm).y(), srs.get(), ul.x(), ul.y() );
        utm->transform2D( (urp+c_utm).x(), (urp+c_utm).y(), srs.get(), ur.x(), ur.y() );
        utm->transform2D( (lrp+c_utm).x(), (lrp+c_utm).y(), srs.get(), lr.x(), lr.y() );

        setCorners( ll, lr, ul, ur );
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
    DrapeableNode::traverse(nv);
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