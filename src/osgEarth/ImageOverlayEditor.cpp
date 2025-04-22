/* osgEarth
* Copyright 2025 Pelican Mapping
* MIT License
*/
#include <osgEarth/ImageOverlayEditor>

using namespace osgEarth;
using namespace osgEarth::Contrib;

/***************************************************************************/

namespace
{
    struct OverlayCallback : public ImageOverlay::ImageOverlayCallback
    {
        OverlayCallback(ImageOverlayEditor *editor)
            : _editor(editor)
        {
        }

        virtual void onOverlayChanged()
        {
            _editor->updateDraggers();
        }

        ImageOverlayEditor* _editor;    
    };
}

/***************************************************************************/




ImageOverlayEditor::ImageOverlayEditor(ImageOverlay* overlay, bool singleVert) :
osg::Group(),
_overlay        ( overlay ),
_singleVert     ( singleVert )
{   
    _overlayCallback = new OverlayCallback(this);
    _overlay->addCallback( _overlayCallback.get() );
    addDragger( ImageOverlay::CONTROLPOINT_CENTER );
    addDragger( ImageOverlay::CONTROLPOINT_LOWER_LEFT );
    addDragger( ImageOverlay::CONTROLPOINT_LOWER_RIGHT );
    addDragger( ImageOverlay::CONTROLPOINT_UPPER_LEFT );
    addDragger( ImageOverlay::CONTROLPOINT_UPPER_RIGHT );
}

ImageOverlayEditor::~ImageOverlayEditor()
{
    _overlay->removeCallback( _overlayCallback.get() );
}

void
ImageOverlayEditor::addDragger( ImageOverlay::ControlPoint controlPoint )
{    
    osg::Vec2d location = _overlay->getControlPoint( controlPoint );
    
    SphereDragger* dragger = new SphereDragger(_overlay->getMapNode());
    GeoPoint point( SpatialReference::get("epsg:4326"), location.x(), location.y() );
    dragger->setPosition( point );

    dragger->onPositionChanged([this, controlPoint](auto* sender, const osgEarth::GeoPoint& position)
        {
            //Convert to lat/lon
            GeoPoint p;
            position.transform(SpatialReference::create("epsg:4326"), p);
            _overlay->setControlPoint(controlPoint, p.x(), p.y(), _singleVert);
        });

    addChild(dragger);
    _draggers[ controlPoint ] = dragger;
}

void
ImageOverlayEditor::updateDraggers()
{
    for (ImageOverlayEditor::ControlPointDraggerMap::iterator itr = getDraggers().begin(); itr != getDraggers().end(); ++itr)
    {
        Dragger* dragger = itr->second.get();
        //Get the location of the control point
        osg::Vec2d location = getOverlay()->getControlPoint( itr->first );
        dragger->setPosition( GeoPoint( SpatialReference::create( "epsg:4326"), location.x(), location.y()), false );
    }
}
