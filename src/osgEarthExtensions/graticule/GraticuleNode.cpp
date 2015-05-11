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
#include "GraticuleNode"

using namespace osgEarth;
using namespace osgEarth::Graticule;

#define LC "[GraticuleNode] "


GraticuleNode::GraticuleNode(MapNode* mapNode, GraticuleTerrainEffect* effect, const GraticuleOptions& options):
_mapNode(mapNode),
    _effect(effect),
    _resolution(options.maxResolution().get()),
    _maxResolution(options.maxResolution().get()),
    _options(options),
    _lat(0.0),
    _lon(0.0),
    _viewExtent(osgEarth::SpatialReference::create("wgs84"), -180, -90, 180, 90),
    _visible(true)
{
    setNumChildrenRequiringUpdateTraversal(1);

    // Initialize the formatter
    _formatter = new LatLongFormatter(osgEarth::Util::LatLongFormatter::FORMAT_DEGREES_MINUTES_SECONDS_TERSE, LatLongFormatter::USE_SYMBOLS |LatLongFormatter::USE_PREFIXES);

    // Initialize the resolution uniform
    _resolutionUniform = mapNode->getTerrainEngine()->getTerrainStateSet()->getOrCreateUniform(GraticuleOptions::resolutionUniformName(), osg::Uniform::FLOAT);
    _resolutionUniform->set((float)_resolution);

    initLabelPool();
}

GraticuleNode::~GraticuleNode()
{
    //nop
}

bool GraticuleNode::getVisible() const
{
    return _visible;
}

void GraticuleNode::setVisible(bool visible)
{
    if (_visible != visible)
    {
        _visible = visible;
        if (_visible)
        {
            setNodeMask(~0u);
            _mapNode->getTerrainEngine()->addEffect(_effect.get());
            // We need to re-initilize the uniform b/c the uniform may have been removed when the effect was removed.
            _resolutionUniform = _mapNode->getTerrainEngine()->getTerrainStateSet()->getOrCreateUniform(GraticuleOptions::resolutionUniformName(), osg::Uniform::FLOAT);
            _resolutionUniform->set((float)_resolution);
        }
        else
        {
            setNodeMask(0);
            _mapNode->getTerrainEngine()->removeEffect(_effect.get());
        }
    }
}

void GraticuleNode::initLabelPool()
{
    const osgEarth::SpatialReference* srs = osgEarth::SpatialReference::create("wgs84");

    Style style;
    TextSymbol* text = style.getOrCreateSymbol<TextSymbol>();
    text->alignment() = TextSymbol::ALIGN_CENTER_CENTER;
    text->fill()->color() = _options.labelColor().get();
    AltitudeSymbol* alt = style.getOrCreateSymbol<AltitudeSymbol>();
    alt->clamping() = AltitudeSymbol::CLAMP_TO_TERRAIN;

    unsigned int labelPoolSize = 8 * _options.gridLines().get();
    for (unsigned int i = 0; i < labelPoolSize; i++)
    {
        GeoPoint pt(srs, 0,0,0);
        LabelNode* label = new LabelNode(_mapNode.get(), pt, "0,0");
        label->setDynamic(true);
        label->setStyle(style);
        _labelPool.push_back(label);
        addChild(label);
    }
}

void GraticuleNode::updateLabels()
{
    const osgEarth::SpatialReference* srs = osgEarth::SpatialReference::create("wgs84");

    std::vector< GeoExtent > extents;
    if (_viewExtent.crossesAntimeridian())
    {
        GeoExtent first, second;
        _viewExtent.splitAcrossAntimeridian(first, second);
        extents.push_back(first);
        extents.push_back(second);
    }
    else
    {
        extents.push_back( _viewExtent );
    }

    double resDegrees = _resolution * 180.0;
    // We want half the resolution so the labels don't appear as often as the grid lines
    resDegrees *= 2.0;

    
    // Hide all the labels
    for (unsigned int i = 0; i < _labelPool.size(); i++)
    {
        _labelPool[i]->setNodeMask(0);
    }




    
    unsigned int labelIndex = 0;


    for (unsigned int extentIndex = 0; extentIndex < extents.size(); extentIndex++)
    {
        GeoExtent extent = extents[extentIndex];

        int minLonIndex = floor(((extent.xMin() + 180.0)/resDegrees));
        int maxLonIndex = ceil(((extent.xMax() + 180.0)/resDegrees));

        int minLatIndex = floor(((extent.yMin() + 90)/resDegrees));
        int maxLatIndex = ceil(((extent.yMax() + 90)/resDegrees));

        // Generate horizontal labels
        for (unsigned int i = minLonIndex; i <= maxLonIndex; i++)
        {
            GeoPoint point(srs, -180.0 + (double)i * resDegrees, _lat, 0, ALTMODE_ABSOLUTE);
            LabelNode* label = _labelPool[labelIndex++];

            label->setNodeMask(~0u);
            label->setPosition(point);
            std::string text = getText( point, false);
            label->setText( text );
            if (labelIndex == _labelPool.size() - 1)
            {
                return;
            }
        }



        // Generate the vertical labels
        for (unsigned int i = minLatIndex; i <= maxLatIndex; i++)
        {
            GeoPoint point(srs, _lon, -90.0 + (double)i * resDegrees, 0, ALTMODE_ABSOLUTE);
            LabelNode* label = _labelPool[labelIndex++];
            label->setNodeMask(~0u);
            label->setPosition(point);
            std::string text = getText( point, true);
            label->setText( text );
            if (labelIndex == _labelPool.size() - 1)
            {
                return;
            }
        }
    }
}

void GraticuleNode::traverse(osg::NodeVisitor& nv)
{
    if (nv.getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR)
    {
        updateLabels();
    }
    else if (nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR)
    {
        osgUtil::CullVisitor* cv = static_cast<osgUtil::CullVisitor*>(&nv);

        osg::Vec3d vp = cv->getViewPoint();


        GeoPoint eyeGeo;
        eyeGeo.fromWorld( _mapNode->getMapSRS(), vp );
        _lon = eyeGeo.x();
        _lat = eyeGeo.y();

        osg::Viewport* viewport = cv->getCurrentCamera()->getViewport();

        float centerX = viewport->x() + viewport->width() / 2.0;
        float centerY = viewport->y() + viewport->height() / 2.0;

        if (_mapNode->getTerrain()->getWorldCoordsUnderMouse(cv->getCurrentCamera()->getView(), centerX, centerY, _focalPoint))
        {
            GeoPoint focalGeo;
            focalGeo.fromWorld( _mapNode->getMapSRS(), _focalPoint );
            _lon = focalGeo.x();
            _lat = focalGeo.y();
        }

    
        double targetResolution = (_viewExtent.height() / 180.0) / _options.gridLines().get();

        double resolution = _maxResolution;
        while (resolution  > targetResolution)
        {
            resolution /= 2.0;
        }
        

        // Trippy
        //resolution = targetResolution;

        _viewExtent = getViewExtent( cv->getCurrentCamera() );

        if (_resolution != resolution)
        {
            setResolution(resolution);
        }
    }
    osg::Group::traverse(nv);
}

double GraticuleNode::getResolution() const
{
    return _resolution;
}

void GraticuleNode::setResolution(double resolution)
{
    if (_resolution != resolution)
    {
        _resolution = resolution;
        _resolutionUniform->set((float)_resolution);
    }
}

std::string GraticuleNode::getText(const GeoPoint& location, bool lat)
{ 
    double value = lat ? location.y() : location.x();
    return _formatter->format(value, lat);
}

osgEarth::GeoExtent GraticuleNode::getViewExtent(osg::Camera* camera)
{
    // Get the corners of all points on the view frustum.  Mostly modified from osgthirdpersonview
    osg::Matrixd proj = camera->getProjectionMatrix();
    osg::Matrixd mv = camera->getViewMatrix();
    osg::Matrixd invmv = osg::Matrixd::inverse( mv );

    double nearPlane = proj(3,2) / (proj(2,2)-1.0);
    double farPlane = proj(3,2) / (1.0+proj(2,2));

    // Get the sides of the near plane.
    double nLeft = nearPlane * (proj(2,0)-1.0) / proj(0,0);
    double nRight = nearPlane * (1.0+proj(2,0)) / proj(0,0);
    double nTop = nearPlane * (1.0+proj(2,1)) / proj(1,1);
    double nBottom = nearPlane * (proj(2,1)-1.0) / proj(1,1);

    // Get the sides of the far plane.
    double fLeft = farPlane * (proj(2,0)-1.0) / proj(0,0);
    double fRight = farPlane * (1.0+proj(2,0)) / proj(0,0);
    double fTop = farPlane * (1.0+proj(2,1)) / proj(1,1);
    double fBottom = farPlane * (proj(2,1)-1.0) / proj(1,1);

    double dist = farPlane - nearPlane;

    std::vector< osg::Vec3d > verts;
    verts.reserve(9);


    // Include origin?
    //verts.push_back(osg::Vec3d(0., 0., 0. ));
    verts.push_back(osg::Vec3d( nLeft, nBottom, -nearPlane ));
    verts.push_back(osg::Vec3d( nRight, nBottom, -nearPlane ));
    verts.push_back(osg::Vec3d( nRight, nTop, -nearPlane ));
    verts.push_back(osg::Vec3d( nLeft, nTop, -nearPlane ));
    verts.push_back(osg::Vec3d( fLeft, fBottom, -farPlane ));
    verts.push_back(osg::Vec3d( fRight, fBottom, -farPlane ));
    verts.push_back(osg::Vec3d( fRight, fTop, -farPlane ));
    verts.push_back(osg::Vec3d( fLeft, fTop, -farPlane ));

    const osgEarth::SpatialReference* srs = osgEarth::SpatialReference::create("epsg:4326");

    // Compute the bounding sphere of the frustum.
    osg::BoundingSphered bs;
    for (unsigned int i = 0; i < verts.size(); i++)
    {
        osg::Vec3d world = verts[i] * invmv;
        bs.expandBy( world );
    }

    // Get the center of the bounding sphere
    osgEarth::GeoPoint center;
    center.fromWorld(srs, bs.center());

    double radiusDegrees = bs.radius() /= 111000.0;
    double minLon = center.x() - radiusDegrees;
    double minLat = osg::clampAbove(center.y() - radiusDegrees, -90.0);
    double maxLon = center.x() + radiusDegrees;
    double maxLat = osg::clampBelow(center.y() + radiusDegrees, 90.0);

    osgEarth::GeoExtent extent(srs, minLon, minLat, maxLon, maxLat);
    extent.normalize();

    return extent;
}