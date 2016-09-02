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
#include "GraticuleNode"
#include <osgEarth/TerrainEngineNode>

using namespace osgEarth;
using namespace osgEarth::Util;

#define LC "[GraticuleNode] "


GraticuleNode::GraticuleNode(MapNode* mapNode, const GraticuleOptions& options):
_mapNode(mapNode),
    _resolution(10.0/180.0),
    _options(options),
    _lat(0.0),
    _lon(0.0),
    _viewExtent(osgEarth::SpatialReference::create("wgs84"), -180, -90, 180, 90),
    _visible(true),
    _metersPerPixel(0.0)
{
    setNumChildrenRequiringUpdateTraversal(1);

    // Read the resolutions from the config.
    if (options.resolutions().isSet())
    {
        StringTokenizer tok(" ");
        StringVector tokens;
        tok.tokenize(*options.resolutions(), tokens);
        for (unsigned int i = 0; i < tokens.size(); i++)
        {
            double r = as<double>(tokens[i], -1.0);
            if (r > 0) 
            {
                _resolutions.push_back( r );
            }
        }
    }

    if (_resolutions.empty())
    {
        // Initialize the resolutions
        _resolutions.push_back( 10.0 );
        _resolutions.push_back( 5.0 );
        _resolutions.push_back( 2.5 );
        _resolutions.push_back( 1.0 );
        _resolutions.push_back( 0.5 );
        _resolutions.push_back( 0.25 );
        _resolutions.push_back( 0.125 );
        _resolutions.push_back( 0.0625 );
        _resolutions.push_back( 0.03125 );

    }

    // Divide all the resolutions by 180 so they match up with the terrain effects concept of resolutions
    for (unsigned int i = 0; i < _resolutions.size(); i++)
    {
        _resolutions[i] /= 180.0;
    }

    // Create the effect and add it to the MapNode.
    _effect = new GraticuleTerrainEffect( options, 0 );
    _mapNode->getTerrainEngine()->addEffect( _effect );

    // Initialize the formatter
    _formatter = new LatLongFormatter(osgEarth::Util::LatLongFormatter::FORMAT_DEGREES_MINUTES_SECONDS_TERSE, LatLongFormatter::USE_SYMBOLS |LatLongFormatter::USE_PREFIXES);

    // Initialize the resolution uniform
    _resolutionUniform = mapNode->getTerrainEngine()->getSurfaceStateSet()->getOrCreateUniform(GraticuleOptions::resolutionUniformName(), osg::Uniform::FLOAT);
    _resolutionUniform->set((float)_resolution);

    initLabelPool();
}

GraticuleNode::~GraticuleNode()
{
    osg::ref_ptr< MapNode > mapNode = _mapNode.get();
    if ( mapNode.valid() )
    {
        mapNode->getTerrainEngine()->removeEffect( _effect );
    }
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
            _resolutionUniform = _mapNode->getTerrainEngine()->getSurfaceStateSet()->getOrCreateUniform(GraticuleOptions::resolutionUniformName(), osg::Uniform::FLOAT);
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

std::vector< double >& GraticuleNode::getResolutions()
{
    return _resolutions;
}

const osg::Vec2f& GraticuleNode::getCenterOffset() const
{
    return _centerOffset;
}


void GraticuleNode::setCenterOffset(const osg::Vec2f& offset)
{
    if (_centerOffset != offset)
    {
        _centerOffset = offset;
        updateLabels();
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

    // Approximate offset in degrees
    double degOffset = _metersPerPixel / 111000.0;
     
    unsigned int labelIndex = 0;


    for (unsigned int extentIndex = 0; extentIndex < extents.size(); extentIndex++)
    {
        GeoExtent extent = extents[extentIndex];

        int minLonIndex = floor(((extent.xMin() + 180.0)/resDegrees));
        int maxLonIndex = ceil(((extent.xMax() + 180.0)/resDegrees));

        int minLatIndex = floor(((extent.yMin() + 90)/resDegrees));
        int maxLatIndex = ceil(((extent.yMax() + 90)/resDegrees));

        // Generate horizontal labels
        for (int i = minLonIndex; i <= maxLonIndex; i++)
        {
            GeoPoint point(srs, -180.0 + (double)i * resDegrees, _lat + (_centerOffset.y() * degOffset), 0, ALTMODE_ABSOLUTE);
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
        for (int i = minLatIndex; i <= maxLatIndex; i++)
        {
            GeoPoint point(srs, _lon + (_centerOffset.x() * degOffset), -90.0 + (double)i * resDegrees, 0, ALTMODE_ABSOLUTE);
            // Skip drawing labels at the poles
            if (osg::equivalent(osg::absolute( point.y()), 90.0, 0.1))
            {
                continue;
            }
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

        osg::Matrixd viewMatrix = *cv->getModelViewMatrix();

        // Only update if the view matrix has changed.
        if (viewMatrix != _viewMatrix)
        {            
            GeoPoint eyeGeo;
            eyeGeo.fromWorld( _mapNode->getMapSRS(), vp );
            _lon = eyeGeo.x();
            _lat = eyeGeo.y();            

            osg::Viewport* viewport = cv->getViewport();

            float centerX = viewport->x() + viewport->width() / 2.0;
            float centerY = viewport->y() + viewport->height() / 2.0;

            float offsetCenterX = centerX;
            float offsetCenterY = centerY;

            bool hitValid = false;

            // Try the center of the screen.
            if(_mapNode->getTerrain()->getWorldCoordsUnderMouse(cv->getCurrentCamera()->getView(), centerX, centerY, _focalPoint))
            {
                hitValid = true;
            }

            if (hitValid)
            {
                GeoPoint focalGeo;
                focalGeo.fromWorld( _mapNode->getMapSRS(), _focalPoint );
                _lon = focalGeo.x();
                _lat = focalGeo.y();
                // We only store the previous view matrix if we actually got a hit.  Otherwise we still need to update.
                _viewMatrix = viewMatrix;
            }


            double targetResolution = (_viewExtent.height() / 180.0) / _options.gridLines().get();

            double resolution = _resolutions[0];
            for (unsigned int i = 0; i < _resolutions.size(); i++)
            {
                resolution = _resolutions[i];
                if (resolution <= targetResolution)
                {
                    break;
                }
            }

            // Trippy
            //resolution = targetResolution;

            _viewExtent = getViewExtent( cv );

            // Try to compute an approximate meters to pixel value at this view.
            double fovy, aspectRatio, zNear, zFar;
            cv->getProjectionMatrix()->getPerspective(fovy, aspectRatio, zNear, zFar);
            double dist = osg::clampAbove(eyeGeo.z(), 1.0);
            double halfWidth = osg::absolute( tan(osg::DegreesToRadians(fovy/2.0)) * dist );
            _metersPerPixel = (2.0 * halfWidth) / (double)viewport->height();

            if (_resolution != resolution)
            {
                setResolution(resolution);
            }
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

osgEarth::GeoExtent GraticuleNode::getViewExtent(osgUtil::CullVisitor* cullVisitor)
{
    // Get the corners of all points on the view frustum.  Mostly modified from osgthirdpersonview
    osg::Matrixd proj = *cullVisitor->getProjectionMatrix();
    osg::Matrixd mv = *cullVisitor->getModelViewMatrix();
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
    
    // Try to clamp the maximum radius so far out views don't go wacky.
    radiusDegrees = osg::minimum(radiusDegrees, 90.0);

    double minLon = center.x() - radiusDegrees;
    double minLat = osg::clampAbove(center.y() - radiusDegrees, -90.0);
    double maxLon = center.x() + radiusDegrees;
    double maxLat = osg::clampBelow(center.y() + radiusDegrees, 90.0);

    osgEarth::GeoExtent extent(srs, minLon, minLat, maxLon, maxLat);
    extent.normalize();

    return extent;
}