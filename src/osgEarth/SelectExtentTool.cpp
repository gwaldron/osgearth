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


SelectExtentTool::SelectExtentTool( osgEarth::MapNode* mapNode ):
_enabled(true),
_mouseDown(false),
_mouseButtonMask(osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON),
_modKeyMask(0)
{
    _root = new osg::Group();
    _mapNode = mapNode;
    if (_mapNode.valid())
    {
        _mapNode->addChild(_root.get());
    }
    rebuild();
}

SelectExtentTool::~SelectExtentTool()
{
    //nop
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

    // define a style for the line
    LineSymbol* ls = _feature->style().mutable_value().getOrCreate<LineSymbol>();
    ls->stroke().mutable_value().color() = Color::Yellow;
    ls->stroke().mutable_value().width() = Distance(3.0f, Units::PIXELS);
    ls->tessellationSize() = Distance(100, Units::KILOMETERS);

    RenderSymbol* render = _feature->style().mutable_value().getOrCreate<RenderSymbol>();
    render->depthTest() = false;

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
    
    if ((ea.getEventType() == ea.PUSH) &&
        (ea.getButton() & _mouseButtonMask) != 0 &&
        (ea.getModKeyMask() == _modKeyMask))
    {
        _mouseDown = getMapNode()->getGeoPointUnderMouse(aa.asView(), ea.getX(), ea.getY(), _mouseDownPoint);
        return true;
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
        if (_callback != nullptr)
        {
            _callback(_extent);
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
}

void SelectExtentTool::clear()
{
    _feature->getGeometry()->clear();
    _featureNode->dirty();
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

void
SelectExtentTool::setCallback(const Callback& value)
{
    _callback = value;
}

Style&
SelectExtentTool::getStyle()
{
    return _feature->style().mutable_value();
}
