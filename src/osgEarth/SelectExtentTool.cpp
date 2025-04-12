/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */

#include <osgEarth/SelectExtentTool>
#include <osgEarth/GLUtils>

#define LC "[SelectExtentTool] "

using namespace osgEarth;
using namespace osgEarth::Contrib;

//#define SHOW_EXTENT 1


SelectExtentTool::SelectExtentTool(osgEarth::MapNode* mapNode)
{
    _root = new osg::Group();
    _mapNode = mapNode;
    if (_mapNode.valid())
    {
        _mapNode->addChild(_root.get());
    }
    rebuild();
}

void
SelectExtentTool::setEnabled(bool value)
{
    _enabled = value;
}

void
SelectExtentTool::rebuild()
{
    if ( _featureNode.valid() )
    {
        _root->removeChild( _featureNode.get() );
        _featureNode = 0L;
    }

    if ( !_mapNode.valid())
        return;

    _feature = new Feature(new Ring(), getMapNode()->getMapSRS());
    _feature->geoInterp() = GEOINTERP_RHUMB_LINE;

    // define a style for the selection graphics
    auto* ls = _feature->style().mutable_value().getOrCreate<LineSymbol>();
    ls->stroke().mutable_value().color() = Color::Yellow;
    ls->stroke().mutable_value().width() = Distance(3.0f, Units::PIXELS);
    ls->tessellationSize() = Distance(100, Units::KILOMETERS);

    auto* poly = _feature->style()->getOrCreate<PolygonSymbol>();
    poly->fill()->color() = Color(Color::Black, 0.15f);

    auto* alt = _feature->style()->getOrCreate<AltitudeSymbol>();
    alt->clamping() = alt->CLAMP_TO_TERRAIN;
    alt->technique() = alt->TECHNIQUE_DRAPE;

    _featureNode = new FeatureNode( _feature.get() );
    _featureNode->setMapNode(getMapNode());
    _root->addChild(_featureNode.get());

    GLUtils::setLighting(_featureNode->getOrCreateStateSet(), osg::StateAttribute::OFF);
}

bool
SelectExtentTool::handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
{    
    if ( ea.getHandled() || !_enabled )
    {
        return false;
    }
    
    if (ea.getEventType() == ea.PUSH)
    {
        if ((ea.getButton() & _mouseButtonMask) != 0 &&
            ((ea.getModKeyMask() & _modKeyMask)) != 0)
        {
            clear();
            _mouseDown = getMapNode()->getGeoPointUnderMouse(aa.asView(), ea.getX(), ea.getY(), _mouseDownPoint);
            return true;
        }
    }

    else if (
        (_mouseDown) && 
        (ea.getEventType() == ea.DRAG))
    {
        GeoPoint point;
        if (getMapNode()->getGeoPointUnderMouse(aa.asView(), ea.getX(), ea.getY(), point))
        {
            Bounds bounds(
                std::min(_mouseDownPoint.x(), point.x()),
                std::min(_mouseDownPoint.y(), point.y()),
                0.0,
                std::max(_mouseDownPoint.x(), point.x()),
                std::max(_mouseDownPoint.y(), point.y()),
                0.0);
            _extent = GeoExtent(_mapNode->getMapSRS(), bounds);
            updateFeature(_extent);
            return true;
        }
    }

    else if (
        (_mouseDown) &&
        (ea.getEventType() == ea.RELEASE))
    {
        _mouseDown = false;

        if (_featureVisible)
        {
            if (onSelect)
            {
                onSelect.fire(_extent);
            }
        }
        else
        {
            clear();
            if (onSelect)
            {
                onSelect.fire(GeoExtent::INVALID);
            }
        }
        return true;
    }
    return false;
}

void
SelectExtentTool::updateFeature(const GeoExtent& e)
{
    Ring* line = new Ring();    
    line->push_back(osg::Vec3d(e.xMin(), e.yMin(), 0));
    line->push_back(osg::Vec3d(e.xMax(), e.yMin(), 0));
    line->push_back(osg::Vec3d(e.xMax(), e.yMax(), 0));
    line->push_back(osg::Vec3d(e.xMin(), e.yMax(), 0));
    _feature->setGeometry(line);
    _featureNode->dirty();
    _featureVisible = true;
}

void SelectExtentTool::clear()
{
    _feature->getGeometry()->clear();
    _featureNode->dirty();
    _featureVisible = false;
}

void
SelectExtentTool::setMouseButtonMask(int value)
{
    _mouseButtonMask = value;
}

void
SelectExtentTool::setModKeyMask(int value)
{
    _modKeyMask = value;
}

Style&
SelectExtentTool::getStyle()
{
    return _feature->style().mutable_value();
}
