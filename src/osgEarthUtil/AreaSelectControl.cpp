#include <osgEarthUtil/AreaSelectControl>


//#include <GeographicLib/Geodesic.hpp>



#include <osgEarthSymbology/Geometry>
#include <osgEarthSymbology/Style>
#include <osgEarthUtil/Controls>
#include <osgEarthUtil/ObjectPlacer>
#include <iostream>

#include <sstream>
using namespace osgEarth::Util;
using namespace osgEarth;
using namespace std;

AreaSelectControl::AreaSelectControl(osgViewer::View* viewer, osgEarth::MapNode* mapNode) : m_view(viewer), m_mapNode(mapNode)
{
	reset(); 
}

void AreaSelectControl::reset(void)
{
	m_manip = (osgEarth::Util::EarthManipulator*)m_view->getCameraManipulator();
	m_currentState = new AreaSelectControlStates::Start(this);
	clearDrawing();
	for (int c = 0; c < 4; c++)
	{
		m_fourPoints[c] = osg::Vec3d(0.0, 0.0, 0.0);


	}

}

AreaSelectControl::~AreaSelectControl(void)
{
	clearDrawing();
}

bool AreaSelectControl::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	return (m_currentState->handle(ea, aa));
}

int sortCoordinates( const void * a , const void * b)
{
	osg::Vec3d* first = (osg::Vec3d*)a;
	osg::Vec3d* second = (osg::Vec3d*)b;

	if(first->y() < second->y() || first->x() < second->x())
		return -1;

	if(first->y() > second->y() || first->x() > second->x())
		return 1;

	return 0;

}

void AreaSelectControl::drawRectangle(void)
{

	osgEarth::Features::Feature* feature;
	osgEarth::Features::FeatureNode* featureNode;

	m_fourMapPoints[0] = m_firstPoint;
	m_fourMapPoints[2] = m_secondPoint;

	osg::Vec3d screenCoordinates[4];

	osg::Camera* camera =m_view->getCamera();

	m_mapNode->getMap()->mapPointToWorldPoint(m_fourMapPoints[0], screenCoordinates[0]);
	screenCoordinates[0] = screenCoordinates[0] * camera->getViewMatrix() * camera->getProjectionMatrix() * camera->getViewport()->computeWindowMatrix();
	m_mapNode->getMap()->mapPointToWorldPoint(m_fourMapPoints[2], screenCoordinates[2]);
	screenCoordinates[2] = screenCoordinates[2] * camera->getViewMatrix() * camera->getProjectionMatrix()* camera->getViewport()->computeWindowMatrix();


	screenCoordinates[1] = osg::Vec3d(screenCoordinates[2].x(), screenCoordinates[0].y(), 0);
	screenCoordinates[3] = osg::Vec3d(screenCoordinates[0].x(), screenCoordinates[2].y(), 0);



	qsort(screenCoordinates, 4, sizeof(osg::Vec3d), sortCoordinates);

	osg::Vec3d tempPoint = screenCoordinates[3];
	screenCoordinates[3] = screenCoordinates[2];
	screenCoordinates[2] = tempPoint;

	for (int c = 0; c < 4; c++)
	{
		osgUtil::LineSegmentIntersector::Intersections results;

		if (!m_view->computeIntersections(screenCoordinates[c].x(), screenCoordinates[c].y(), results))
		{
			return;
		}

		osgUtil::LineSegmentIntersector::Intersection first = *(results.begin());
		osg::Vec3d point = first.getWorldIntersectPoint();

		m_mapNode->getMap()->worldPointToMapPoint(point, m_fourMapPoints[c]);
		m_fourPoints[c] = m_fourMapPoints[c];


	}

	clearDrawing();
	osgEarth::Symbology::Geometry* line = new osgEarth::Symbology::LineString();
	osgEarth::Symbology::Style lineStyle;
	lineStyle.getOrCreate<osgEarth::Symbology::LineSymbol>()->stroke()->color() = osg::Vec4f((71 / 255.0), 255, (10 / 255.0), .5);
	lineStyle.getOrCreate<osgEarth::Symbology::LineSymbol>()->stroke()->width() = 2.0f;
	lineStyle.getOrCreate<osgEarth::Symbology::LineSymbol>()->stroke()->stipple()= 0x00FF;

	feature = new osgEarth::Features::Feature(line, lineStyle);
	feature->geoInterp() = osgEarth::GEOINTERP_GREAT_CIRCLE;
	featureNode = new osgEarth::Features::FeatureNode(m_mapNode, feature, false);

	featureNode->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);

	feature->getGeometry()->push_back(m_fourMapPoints[0]);
	feature->getGeometry()->push_back(m_fourMapPoints[1]);
	feature->getGeometry()->push_back(m_fourMapPoints[2]);
	feature->getGeometry()->push_back(m_fourMapPoints[3]);
	feature->getGeometry()->push_back(m_fourMapPoints[0]);

	featureNode->setNodeMask(201);
	m_mapNode->addChild(featureNode);
	featureNode->setFeature(feature);

	m_FeatureNodeList.push_back(featureNode);

	osgEarth::Symbology::Geometry* polygon = new osgEarth::Symbology::Polygon();
	osgEarth::Symbology::Style polygonStyle;
	polygonStyle.getOrCreate<osgEarth::Symbology::PolygonSymbol>()->fill()->color() = osg::Vec4f((175.0 / 255.0), 255, (10 / 255.0), .2);
	feature = new osgEarth::Features::Feature(polygon, polygonStyle);
	feature->geoInterp() = osgEarth::GEOINTERP_GREAT_CIRCLE;
	featureNode = new osgEarth::Features::FeatureNode(m_mapNode, feature, false);

	featureNode->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
	feature->getGeometry()->push_back(m_fourMapPoints[0]);
	feature->getGeometry()->push_back(m_fourMapPoints[1]);
	feature->getGeometry()->push_back(m_fourMapPoints[2]);
	feature->getGeometry()->push_back(m_fourMapPoints[3]);
	feature->getGeometry()->push_back(m_fourMapPoints[0]);

	featureNode->setNodeMask(202);
	m_mapNode->addChild(featureNode);
	featureNode->setFeature(feature);
	m_FeatureNodeList.push_back(featureNode);

	

	for(unsigned int c = 0; c < m_eventHandlers.size(); c++)
		m_eventHandlers.at(c)->drawingRectangle(getFourPoints());

	featureNode->setFeature(feature);

	m_FeatureNodeList.push_back(featureNode);
}

void AreaSelectControl::changed(void)
{
	for(unsigned int c = 0; c < m_eventHandlers.size(); c++)
		m_eventHandlers.at(c)->changed();
}
void AreaSelectControl::completed(void)
{
	for(unsigned int c = 0; c < m_eventHandlers.size(); c++)
		m_eventHandlers.at(c)->completed();
}

void AreaSelectControl::clearDrawing(void)
{
	for (unsigned int c = 0; c < m_FeatureNodeList.size(); c++)
	{
		m_mapNode->removeChild(m_FeatureNodeList[c]);
	}

	

	m_FeatureNodeList.clear();
}

using namespace AreaSelectControlStates;

State::State(AreaSelectControl* control) : m_control(control)
{
}

osg::Object* State::cloneType(void) const
{
	return (new Start(m_control));
}

osg::Object* State::clone(const osg::CopyOp&)
{
	return (new Start(m_control));
}

osg::Object* State::clone(const osg::CopyOp&) const
{
	return (new Start(m_control));
}

const char* State::libraryName(void)
{
	return ("State");
}

const char* State::libraryName(void) const
{
	return ("State");
}

const char* State::className(void) const
{
	return ("State");
}

Start::Start(AreaSelectControl* control) : State(control)
{
}

bool Start::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	m_control->clearDrawing();

	if ((ea.getEventType() == ea.PUSH) && (ea.getButton() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON))
	{
		m_control->clearDrawing();
		osgUtil::LineSegmentIntersector::Intersections results;
		osgViewer::View* view = static_cast<osgViewer::View*>(aa.asView());
		if (view->computeIntersections(ea.getX(), ea.getY(), results, 0x01))
		{
			// find the first hit under the mouse:
			osgUtil::LineSegmentIntersector::Intersection first = *(results.begin());
			osg::Vec3d point = first.getWorldIntersectPoint();

			// transform it to map coordinates:
			osg::Vec3d newPoint;
			m_control->getMapNode()->getMap()->worldPointToMapPoint(point, newPoint);
			m_control->setFirstPoint(newPoint);
			m_control->currentState() = new  FirstSelected(m_control);

			return (true);
		}
	}
	return (false);
}

FirstSelected::FirstSelected(AreaSelectControl* control) : State(control)
{
}

bool FirstSelected::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&)
{
	if (ea.getEventType() == ea.DRAG)
	{
		m_control->currentState() = new Dragging(m_control);
		return (true);
	}
	else if ((ea.getEventType() == ea.RELEASE) && (osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON))
	{
		m_control->currentState() = new WaitingForSecondSelection(m_control);
	}
	return (false);
}

Dragging::Dragging(AreaSelectControl* control) : State(control)
{
}

bool Dragging::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	bool handled = false;

	osgUtil::LineSegmentIntersector::Intersections results;
	osgViewer::View* view = static_cast<osgViewer::View*>(aa.asView());
	if ((ea.getEventType() == ea.DRAG) && view->computeIntersections(ea.getX(), ea.getY(), results, 0x01))
	{
		//double x = ea.getX();
		//double y = ea.getY();

		//QKeyEvent* event = 0;
		//if ((x /(double)(m_control->getViewerWidget()->width())) > 0.85)
		{
			//event = new QKeyEvent(QEvent::KeyPress, Qt::Key_Right, Qt::NoModifier);
			//QCoreApplication::postEvent(m_control->getViewerWidget(), event);
		}

		// find the first hit under the mouse:
		osgUtil::LineSegmentIntersector::Intersection first = *(results.begin());
		osg::Vec3d point = first.getWorldIntersectPoint();

		// transform it to map coordinates:
		osg::Vec3d newPoint;
		m_control->getMapNode()->getMap()->worldPointToMapPoint(point, newPoint);
		m_control->setSecondPoint(newPoint);

		m_control->drawRectangle();

		handled = true;
	}
	else if ((ea.getEventType() == ea.RELEASE) && (ea.getButton() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON))
	{
		m_control->currentState() = new Complete(m_control);
		handled = true;
	}

	return (handled);
}

Complete::Complete(AreaSelectControl* control) : State(control)
{
	control->completed();
}

bool Complete::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{

	m_control->currentState() = new Start(m_control);
	return (false);
}

WaitingForSecondSelection::WaitingForSecondSelection(AreaSelectControl* control) : State(control)
{
}

bool WaitingForSecondSelection::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	bool handled = false;

	osgUtil::LineSegmentIntersector::Intersections results;
	osgViewer::View* view = static_cast<osgViewer::View*>(aa.asView());
	if ((ea.getEventType() == ea.DRAG) && view->computeIntersections(ea.getX(), ea.getY(), results, 0x01))
	{
		// find the first hit under the mouse:
		osgUtil::LineSegmentIntersector::Intersection first = *(results.begin());
		osg::Vec3d point = first.getWorldIntersectPoint();

		// transform it to map coordinates:
		osg::Vec3d newPoint;
		m_control->getMapNode()->getMap()->worldPointToMapPoint(point, newPoint);
		m_control->setSecondPoint(newPoint);
		m_control->drawRectangle();
	}
	else if ((ea.getEventType() == ea.DOUBLECLICK) && (ea.getButton() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON))
	{
		m_control->currentState() = new Complete(m_control);
		handled = true;
	}

	return (handled);
}

bool AreaSelectControl::isNull(void)
{
	for (int c = 0; c < 4; c++)
	{
		if ((m_fourPoints[c].x() != 0.0) && (m_fourPoints[c].y() != 0.0))
		{
			return (false);
		}
	}

	return (true);
}


osg::Object* AreaSelectControlEventHandler::cloneType(void) const{ return NULL;}
osg::Object* AreaSelectControlEventHandler::clone(const osg::CopyOp&){return NULL;}
osg::Object* AreaSelectControlEventHandler::clone(const osg::CopyOp&) const {return NULL;}
const char* AreaSelectControlEventHandler::libraryName(void){return "AreaSelectControlEventHandler";}
const char* AreaSelectControlEventHandler::libraryName(void) const {return "AreaSelectControlEventHandler";}
const char* AreaSelectControlEventHandler::className(void) const {return "AreaSelectControlEventHandler";}