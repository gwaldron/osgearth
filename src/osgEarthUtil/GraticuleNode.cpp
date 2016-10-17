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
#include <osgEarth/VirtualProgram>

using namespace osgEarth;
using namespace osgEarth::Util;

#define LC "[GraticuleNode] "

namespace
{
    struct GraticuleTerrainCallback : public osg::NodeCallback
    {
        osg::observer_ptr<GraticuleNode> _g;
        GraticuleTerrainCallback(GraticuleNode* g) : _g(g) { }
        void operator()(osg::Node* node, osg::NodeVisitor* nv)
        {
            bool traversed = false;
            osg::ref_ptr<GraticuleNode> grat;
            if (_g.lock(grat))
            {
                osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(nv);
                if (cv)
                {
                    osg::StateSet* stateset = grat->getStateSet(cv);
                    if (stateset)
                        cv->pushStateSet(stateset);

                    traverse(node, nv);
                    traversed = true;

                    if (stateset)
                        cv->popStateSet();
                }
            }

            if (!traversed)
                traverse(node, nv);
        }
    };

    const char* textFadeFS =
        "#version 330\n"
        "uniform mat4 osg_ViewMatrixInverse;\n"
        "void oe_graticule_text_frag(inout vec4 color) { \n"
        "    const float maxHAE = 4000.0;\n"
        "    vec3 eye = osg_ViewMatrixInverse[3].xyz;\n"
        "    float hae = length(eye) - 6378137.0;\n"
        "    float alpha = clamp(hae/maxHAE, 0.0, 1.0); \n"
        "    color.a *= alpha;\n"
        "}\n";
}


GraticuleNode::GraticuleNode(MapNode* mapNode, const GraticuleOptions& options):
_mapNode(mapNode),
    _defaultResolution(10.0/180.0),
    _options(options),
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

    _callback = new GraticuleTerrainCallback(this);
    _mapNode->getTerrainEngine()->addCullCallback(_callback.get());

    // Initialize the formatter
    _formatter = new LatLongFormatter(osgEarth::Util::LatLongFormatter::FORMAT_DEGREES_MINUTES_SECONDS_TERSE, LatLongFormatter::USE_SYMBOLS |LatLongFormatter::USE_PREFIXES);
}

GraticuleNode::~GraticuleNode()
{
    osg::ref_ptr< MapNode > mapNode = _mapNode.get();
    if ( mapNode.valid() )
    {
        mapNode->getTerrainEngine()->removeEffect( _effect );
        mapNode->getTerrainEngine()->removeCullCallback( _callback.get() );
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
        }
        else
        {
            setNodeMask(0);
            _mapNode->getTerrainEngine()->removeEffect(_effect.get());
        }
    }
}

void GraticuleNode::initLabelPool(CameraData& cdata)
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
        cdata._labelPool.push_back(label);
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
    
    
    Threading::ScopedMutexLock lock(_cameraDataMapMutex);
    for (CameraDataMap::iterator i = _cameraDataMap.begin(); i != _cameraDataMap.end(); ++i)
    {
        CameraData& cdata = i->second;

        std::vector< GeoExtent > extents;
        if (cdata._viewExtent.crossesAntimeridian())
        {
            GeoExtent first, second;
            cdata._viewExtent.splitAcrossAntimeridian(first, second);
            extents.push_back(first);
            extents.push_back(second);
        }
        else
        {
            extents.push_back( cdata._viewExtent );
        }

        double resDegrees = cdata._resolution * 180.0;
        // We want half the resolution so the labels don't appear as often as the grid lines
        resDegrees *= 2.0;

    
        // Hide all the labels
        for (unsigned int i = 0; i < cdata._labelPool.size(); i++)
        {
            cdata._labelPool[i]->setNodeMask(0);
        }

        // Approximate offset in degrees
        double degOffset = _metersPerPixel / 111000.0;
     
        unsigned int labelIndex = 0;


        bool done = false;
        for (unsigned int extentIndex = 0; extentIndex < extents.size() && !done; extentIndex++)
        {
            GeoExtent extent = extents[extentIndex];

            int minLonIndex = floor(((extent.xMin() + 180.0)/resDegrees));
            int maxLonIndex = ceil(((extent.xMax() + 180.0)/resDegrees));

            int minLatIndex = floor(((extent.yMin() + 90)/resDegrees));
            int maxLatIndex = ceil(((extent.yMax() + 90)/resDegrees));

            // Generate horizontal labels
            for (int i = minLonIndex; i <= maxLonIndex && !done; i++)
            {
                GeoPoint point(srs, -180.0 + (double)i * resDegrees, cdata._lat + (_centerOffset.y() * degOffset), 0, ALTMODE_ABSOLUTE);
                LabelNode* label = cdata._labelPool[labelIndex++];

                label->setNodeMask(~0u);
                label->setPosition(point);
                std::string text = getText( point, false);
                label->setText( text );
                if (labelIndex == cdata._labelPool.size() - 1)
                {
                    done = true;
                }
            }

            // Generate the vertical labels
            for (int i = minLatIndex; i <= maxLatIndex && !done; i++)
            {
                GeoPoint point(srs, cdata._lon + (_centerOffset.x() * degOffset), -90.0 + (double)i * resDegrees, 0, ALTMODE_ABSOLUTE);
                // Skip drawing labels at the poles
                if (osg::equivalent(osg::absolute( point.y()), 90.0, 0.1))
                {
                    continue;
                }
                LabelNode* label = cdata._labelPool[labelIndex++];
                label->setNodeMask(~0u);
                label->setPosition(point);
                std::string text = getText( point, true);
                label->setText( text );
                if (labelIndex == cdata._labelPool.size() - 1)
                {
                    done = true;
                }
            }
        }
    }
}

GraticuleNode::CameraData&
GraticuleNode::getCameraData(osg::Camera* cam)
{
    Threading::ScopedMutexLock lock(_cameraDataMapMutex);
    CameraData& cdata = _cameraDataMap[cam];

    // New camera data? Initialize:
    if (cdata._labelPool.empty())
    {
        cdata._stateset = new osg::StateSet();
        cdata._resolution = _defaultResolution;
        cdata._resolutionUniform = cdata._stateset->getOrCreateUniform(GraticuleOptions::resolutionUniformName(), osg::Uniform::FLOAT);
        cdata._resolutionUniform->set(cdata._resolution);
        cdata._viewExtent = GeoExtent(osgEarth::SpatialReference::create("wgs84"), -180, -90, 180, 90);
        cdata._lat = 0.0;
        cdata._lon = 0.0;
        initLabelPool(cdata);

        cdata._labelStateset = new osg::StateSet();
        VirtualProgram* vp = VirtualProgram::getOrCreate(cdata._labelStateset.get());
        vp->setFunction("oe_graticule_text_frag", textFadeFS, ShaderComp::LOCATION_FRAGMENT_COLORING);
    }

    return cdata;
}

osg::StateSet*
GraticuleNode::getStateSet(osgUtil::CullVisitor* cv)
{
    CameraData& cdata = getCameraData(cv->getCurrentCamera());
    return cdata._stateset.get();
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

        osg::Matrixd viewMatrix = *cv->getModelViewMatrix();

        osg::Vec3d vp = cv->getViewPoint();

        CameraData& cdata = getCameraData(cv->getCurrentCamera());
        
        // Only update if the view matrix has changed.
        if (viewMatrix != cdata._lastViewMatrix)
        {            
            GeoPoint eyeGeo;
            eyeGeo.fromWorld( _mapNode->getMapSRS(), vp );
            cdata._lon = eyeGeo.x();
            cdata._lat = eyeGeo.y();            

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
                cdata._lon = focalGeo.x();
                cdata._lat = focalGeo.y();
                // We only store the previous view matrix if we actually got a hit.  Otherwise we still need to update.
                cdata._lastViewMatrix = viewMatrix;
            }

            // Get the view extent.
            cdata._viewExtent = getViewExtent( cv );

            double targetResolution = (cdata._viewExtent.height() / 180.0) / _options.gridLines().get();

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

            // Try to compute an approximate meters to pixel value at this view.
            double fovy, aspectRatio, zNear, zFar;
            cv->getProjectionMatrix()->getPerspective(fovy, aspectRatio, zNear, zFar);
            double dist = osg::clampAbove(eyeGeo.z(), 1.0);
            double halfWidth = osg::absolute( tan(osg::DegreesToRadians(fovy/2.0)) * dist );
            _metersPerPixel = (2.0 * halfWidth) / (double)viewport->height();

            if (cdata._resolution != resolution)
            {
                cdata._resolution = (float)resolution;
                cdata._resolutionUniform->set(cdata._resolution);
            }
        }

        // traverse the label pool for this camera.
        cv->pushStateSet(cdata._labelStateset.get());

        for(std::vector< osg::ref_ptr< LabelNode > >::iterator i = cdata._labelPool.begin();
            i != cdata._labelPool.end();
            ++i)
        {
            i->get()->accept(nv);
        }

        cv->popStateSet();
    }

    osg::Group::traverse(nv);
}

double GraticuleNode::getResolution() const
{
    return _defaultResolution;
}

void GraticuleNode::setResolution(double resolution)
{
    if (_defaultResolution != resolution)
    {
        _defaultResolution = resolution;
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

    // clamp the projection far plane so it's not on the other 
    // side of the globe
    osg::Vec3d eye = osg::Vec3d(0,0,0) * invmv;
    double f, a, zn, zf;
    proj.getPerspective(f,a,zn,zf);
    zf = std::min(zf, eye.length()-1000.0);
    proj.makePerspective(f, a, zn, zf);

    const osgEarth::SpatialReference* srs = osgEarth::SpatialReference::create("epsg:4326");
    
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

    double radiusDegrees = bs.radius() / 111000.0;
    
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