#include <osgEarthUtil/ImageOverlayEditor>

#include <osg/Geode>
#include <osg/AutoTransform>
#include <osg/ShapeDrawable>
#include <osgViewer/Viewer>

using namespace osgEarth;
using namespace osgEarth::Util;

/***************************************************************************/


ImageOverlayEditor::ImageOverlayEditor(ImageOverlay* imageOverlay, osg::Group* editorGroup):
_imageOverlay(imageOverlay),
_editorGroup(editorGroup),
_dragging(false),
_moveVert(false)
{      
    //Build the handle
    osg::Sphere* sphere = new osg::Sphere(osg::Vec3(0,0,0), 5.0);
    osg::Geode* geode = new osg::Geode();
    osg::ShapeDrawable* sd = new osg::ShapeDrawable( sphere );
    sd->setColor(osg::Vec4(0,1,0,1));
    geode->addDrawable( sd );
    geode->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
    geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    geode->getOrCreateStateSet()->setRenderBinDetails( 99999, "RenderBin" );
    osg::AutoTransform* at = new osg::AutoTransform;
    at->setAutoScaleToScreen( true );
    at->setAutoRotateMode( osg::AutoTransform::AutoRotateMode::ROTATE_TO_CAMERA );
    at->addChild( geode );
    //_handle = geode;
    _handle = at;

    _lowerLeftHandle = new osg::MatrixTransform;
    _lowerLeftHandle->setName("LOWER_LEFT");
    _lowerLeftHandle->addChild( _handle );

    _lowerRightHandle = new osg::MatrixTransform;
    _lowerRightHandle->setName("LOWER_RIGHT");
    _lowerRightHandle->addChild( _handle );

    _upperRightHandle = new osg::MatrixTransform;
    _upperRightHandle->setName("UPPER_RIGHT");
    _upperRightHandle->addChild( _handle );

    _upperLeftHandle = new osg::MatrixTransform;
    _upperLeftHandle->setName("UPPER_LEFT");
    _upperLeftHandle->addChild( _handle );

    _centerHandle = new osg::MatrixTransform;
    _centerHandle->setName("CENTER");
    _centerHandle->addChild( _handle );

    _editor = new osg::Group;
    _editor->addChild( _lowerLeftHandle );
    _editor->addChild( _lowerRightHandle );
    _editor->addChild( _upperRightHandle );
    _editor->addChild( _upperLeftHandle );
    _editor->addChild( _centerHandle );

    if (_editorGroup.valid())
    {
        _editorGroup->addChild( _editor );
    }

    updateEditor();
}

ImageOverlayEditor::~ImageOverlayEditor()
{
    if (_editorGroup.valid())
    {
        _editorGroup->removeChild( _editor );
    }
}

osg::Matrixd
ImageOverlayEditor::getTransform(const osg::Vec2d& loc)
{
    osg::Matrixd matrix;
    _imageOverlay->getEllipsoid()->computeLocalToWorldTransformFromLatLongHeight(osg::DegreesToRadians(loc.y()), osg::DegreesToRadians(loc.x()), 0, matrix);
    return matrix;
}

void
ImageOverlayEditor::updateEditor()
{
    _lowerLeftHandle->setMatrix(getTransform(_imageOverlay->getLowerLeft()));
    _lowerRightHandle->setMatrix(getTransform(_imageOverlay->getLowerRight()));
    _upperLeftHandle->setMatrix(getTransform(_imageOverlay->getUpperLeft()));
    _upperRightHandle->setMatrix(getTransform(_imageOverlay->getUpperRight()));  
    _centerHandle->setMatrix(getTransform(_imageOverlay->getCenter()));
}

ImageOverlayEditor::EditPoint
ImageOverlayEditor::getEditPoint(const osg::NodePath &nodePath)
{
    for (osg::NodePath::const_iterator itr = nodePath.begin(); itr != nodePath.end(); ++itr)
    {
        if ((*itr)->getName() == "LOWER_LEFT") return EDITPOINT_LOWER_LEFT;
        if ((*itr)->getName() == "LOWER_RIGHT") return EDITPOINT_LOWER_RIGHT;
        if ((*itr)->getName() == "UPPER_LEFT") return EDITPOINT_UPPER_LEFT;
        if ((*itr)->getName() == "UPPER_RIGHT") return EDITPOINT_UPPER_RIGHT;
        if ((*itr)->getName() == "CENTER") return EDITPOINT_CENTER;
    }
    return EDITPOINT_NONE;
}





bool ImageOverlayEditor::handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
{
    osgViewer::Viewer* viewer = dynamic_cast<osgViewer::Viewer*>(&aa);
    if (!viewer) return false;

    if ( ea.getEventType() == osgGA::GUIEventAdapter::PUSH && ea.getButton() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON )
    {                        
        // use window coordinates
        // remap the mouse x,y into viewport coordinates.
        osg::Viewport* viewport = viewer->getCamera()->getViewport();
        double mx = viewport->x() + (int)((double )viewport->width()*(ea.getXnormalized()*0.5+0.5));
        double my = viewport->y() + (int)((double )viewport->height()*(ea.getYnormalized()*0.5+0.5));

        // half width, height.
        double w = 5.0f;
        double h = 5.0f;
        osgUtil::PolytopeIntersector* picker = new osgUtil::PolytopeIntersector( osgUtil::Intersector::WINDOW, mx-w, my-h, mx+w, my+h );
        osgUtil::IntersectionVisitor iv(picker);

        viewer->getCamera()->accept(iv);

        if (picker->containsIntersections())
        {
            osgUtil::PolytopeIntersector::Intersection intersection = picker->getFirstIntersection();                                
            osg::NodePath& nodePath = intersection.nodePath;
            _editPoint = getEditPoint(nodePath);
            if (_editPoint != EDITPOINT_NONE)
            {                    
                _dragging = true;
                //_moveVert = ((ea.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_LEFT_CTRL) == osgGA::GUIEventAdapter::MODKEY_LEFT_CTRL);
            }
        }

    }
    else if ( ea.getEventType() == osgGA::GUIEventAdapter::RELEASE && ea.getButton() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON )
    {
        _dragging = false;
    }
    else if ( ea.getEventType() == osgGA::GUIEventAdapter::DRAG)
    {         
        if (_dragging)
        {
            osgViewer::View* view = static_cast<osgViewer::View*>(aa.asView());            
            osgUtil::LineSegmentIntersector::Intersections results;            
            if ( view->computeIntersections( ea.getX(), ea.getY(), results ) )
            {                
                // find the first hit under the mouse:
                osgUtil::LineSegmentIntersector::Intersection first = *(results.begin());
                osg::Vec3d point = first.getWorldIntersectPoint();

                double lat_rad, lon_rad, height;                
                _imageOverlay->getEllipsoid()->convertXYZToLatLongHeight( point.x(), point.y(), point.z(), lat_rad, lon_rad, height );

                switch (_editPoint)
                {
                case EDITPOINT_LOWER_LEFT:
                    if (_moveVert)
                    {
                        _imageOverlay->setLowerLeft(osg::RadiansToDegrees(lon_rad), osg::RadiansToDegrees(lat_rad));                
                    }
                    else
                    {
                        _imageOverlay->setSouth(osg::RadiansToDegrees(lat_rad));
                        _imageOverlay->setWest(osg::RadiansToDegrees(lon_rad));
                    }
                    break;
                case EDITPOINT_LOWER_RIGHT:
                    if (_moveVert)
                    {
                        _imageOverlay->setLowerRight(osg::RadiansToDegrees(lon_rad), osg::RadiansToDegrees(lat_rad));                
                    }
                    else
                    {
                        _imageOverlay->setSouth(osg::RadiansToDegrees(lat_rad));
                        _imageOverlay->setEast(osg::RadiansToDegrees(lon_rad));
                    }
                    break;
                case EDITPOINT_UPPER_LEFT:
                    if (_moveVert)
                    {
                        _imageOverlay->setUpperLeft(osg::RadiansToDegrees(lon_rad), osg::RadiansToDegrees(lat_rad));                
                    }
                    else
                    {
                        _imageOverlay->setNorth(osg::RadiansToDegrees(lat_rad));
                        _imageOverlay->setWest(osg::RadiansToDegrees(lon_rad));
                    }
                    break;
                case EDITPOINT_UPPER_RIGHT:
                    if (_moveVert)
                    {
                        _imageOverlay->setUpperRight(osg::RadiansToDegrees(lon_rad), osg::RadiansToDegrees(lat_rad));                
                    }
                    else
                    {
                        _imageOverlay->setNorth(osg::RadiansToDegrees(lat_rad));
                        _imageOverlay->setEast(osg::RadiansToDegrees(lon_rad));
                    }
                    break;
                case EDITPOINT_CENTER:
                    _imageOverlay->setCenter(osg::RadiansToDegrees(lon_rad), osg::RadiansToDegrees(lat_rad));                
                    break;

                }

                updateEditor();
                return true;
            }
        }
    }
    return false;
}
