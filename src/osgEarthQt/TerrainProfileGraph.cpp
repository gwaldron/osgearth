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
#include <osgEarthQt/TerrainProfileGraph>
#include <osgEarthQt/Actions>
#include <osgEarthQt/DataManager>
#include <osgEarthQt/GuiActions>

#include <osgEarthUtil/TerrainProfile>
#include <osgEarthUtil/LatLongFormatter>

#include <QApplication>
#include <QClipboard>
#include <QMimeData>
#include <QGraphicsLineItem>
#include <QGraphicsRectItem>
#include <QGraphicsScene>
#include <QGraphicsSimpleTextItem>
#include <QGraphicsView>
#include <QLineF>
#include <QList>
#include <QMouseEvent>
#include <QPointF>
#include <QPolygonF>
#include <QRect>
#include <QResizeEvent>
#include <QToolTip>

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::QtGui;

//---------------------------------------------------------------------------
namespace
{
  class BoxedSimpleTextItem : public QGraphicsSimpleTextItem
  {
  public:
    BoxedSimpleTextItem(const QString & text, const QColor& background, QGraphicsItem * parent = 0)
      : QGraphicsSimpleTextItem(text, parent), _backgroundColor(background)
    {
    }

  protected:
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
    {
      painter->setPen(QPen(Qt::NoPen));
      painter->setBrush(QBrush(_backgroundColor));
      painter->drawRect(0, 0, sceneBoundingRect().width(), sceneBoundingRect().height());

      QGraphicsSimpleTextItem::paint(painter, option, widget);
    }

    QColor _backgroundColor;
  };



    struct TerrainGraphChangedShim : public TerrainProfileCalculator::ChangedCallback
    {
        TerrainGraphChangedShim( TerrainProfileGraph* graph ) : _graph( graph ) { }
        void onChanged(const TerrainProfileCalculator* ) { _graph->notifyTerrainGraphChanged(); }
        TerrainProfileGraph* _graph;
    };
}

//---------------------------------------------------------------------------

const int TerrainProfileGraph::FIELD_Z = 0;
const int TerrainProfileGraph::AXES_Z = 10;
const int TerrainProfileGraph::GRAPH_Z = 20;
const int TerrainProfileGraph::OVERLAY_Z = 30;


TerrainProfileGraph::TerrainProfileGraph(TerrainProfileCalculator* calculator, TerrainProfilePositionCallback* callback)
  : QGraphicsView(), _calculator(calculator), _positionCallback(callback), 
    _backgroundColor(128, 128, 128), _fieldColor(204, 204, 204), _axesColor(255, 255, 255),
    _graphColor(0, 128, 0), _graphFillColor(128, 255, 128, 192), _graphField(0, 0, 0, 0),
    _totalDistance(0.0), _graphMinY(0), _graphMaxY(0), _graphWidth(500), _graphHeight(309),
    _selecting(false)
{
  setMouseTracking(true);

  _graphFont.setStyleHint(QFont::SansSerif);
  _graphFont.setFamily(_graphFont.defaultFamily());
  _graphFont.setPixelSize( 10 );

  _linePen.setWidth(2);
  _linePen.setBrush(QBrush(_graphColor));

  _hoverPen.setWidth(1);
  _hoverPen.setBrush(QBrush(_graphColor));

  _axesPen.setWidth(1);
  _axesPen.setBrush(QBrush(_axesColor));

  _scene = new QGraphicsScene(0, 0, _graphWidth, _graphHeight);
  _scene->setBackgroundBrush(QBrush(_backgroundColor));
  setScene(_scene);

  redrawGraph();

  _graphChangedCallback = new TerrainGraphChangedShim(this);
  connect(this, SIGNAL(onNotifyTerrainGraphChanged()), this, SLOT(onTerrainGraphChanged()), Qt::QueuedConnection);
  //connect(_graphChangedCallback, SIGNAL(graphChanged()), this, SLOT(onGraphChanged()), Qt::QueuedConnection);
  if (_calculator.valid())
  {
      _calculator->addChangedCallback(_graphChangedCallback);
  }

  _coordinateFormatter = new osgEarth::Util::LatLongFormatter(
    osgEarth::Util::LatLongFormatter::FORMAT_DECIMAL_DEGREES,
    osgEarth::Util::LatLongFormatter::USE_COLONS);
}

TerrainProfileGraph::~TerrainProfileGraph()
{
    // removed: unnecessary now, since the callback is an observer list
  //if (_calculator.valid())
  //  _calculator->removeChangedCallback(_graphChangedCallback);
}

void TerrainProfileGraph::clear()
{
  _scene->clear();
  _graphLines.clear();
  _graphField.setCoords(0, 0, 0, 0);
  _hoverLine = 0L;
}

void TerrainProfileGraph::onCopyToClipboard()
{
  const osgEarth::Util::TerrainProfile profile = _calculator->getProfile();
  if (profile.getNumElevations() > 0)
  {
    const QLatin1String fieldSeparator(",");
    GeoPoint startPt = _calculator->getStart(ALTMODE_ABSOLUTE);
    GeoPoint endPt = _calculator->getEnd(ALTMODE_ABSOLUTE);
    QString profileInfo = QString("Start:,%1,%2\nEnd:,%3,%4\n")
      .arg(_coordinateFormatter->format(startPt).c_str()).arg(startPt.alt())
      .arg(_coordinateFormatter->format(endPt).c_str()).arg(endPt.alt());
    QString distanceInfo("Distance:");
    QString elevationInfo("Elevation:");
    for (unsigned int i = 0; i < profile.getNumElevations(); i++)
    {
      distanceInfo += fieldSeparator + QString::number(profile.getDistance(i));
      elevationInfo += fieldSeparator + QString::number(profile.getElevation(i));
    }
    profileInfo += distanceInfo + QString("\n") + elevationInfo;

    QImage graphImage(_graphWidth, _graphHeight, QImage::Format_RGB32);
    QPainter p;
    p.begin(&graphImage);
    _scene->render(&p);
    p.end();
    QMimeData* clipData = new QMimeData();
    clipData->setText(profileInfo);
    clipData->setImageData(graphImage);
    QApplication::clipboard()->setMimeData(clipData);
  }
}

void TerrainProfileGraph::setBackgroundColor(const QColor& color)
{
  _backgroundColor = color;
  _scene->setBackgroundBrush(QBrush(_backgroundColor));
  redrawGraph();
}

void TerrainProfileGraph::setFieldColor(const QColor& color)
{
  _fieldColor = color;
  redrawGraph();
}

void TerrainProfileGraph::setAxesColor(const QColor& color)
{
  _axesColor = color;
  _axesPen.setBrush(QBrush(_axesColor));
  redrawGraph();
}

void TerrainProfileGraph::setGraphColor(const QColor& color)
{
  _graphColor = color;
  _linePen.setBrush(QBrush(_graphColor));
  _hoverPen.setBrush(QBrush(_graphColor));
  redrawGraph();
}

void TerrainProfileGraph::setGraphFillColor(const QColor& color)
{
  _graphFillColor = color;
  redrawGraph();
}

void TerrainProfileGraph::setCoordinateFormatter(osgEarth::Util::Formatter* formatter)
{
  _coordinateFormatter = formatter;
}

void TerrainProfileGraph::resizeEvent(QResizeEvent* e)
{
  _graphWidth = e->size().width() - 20;
  _graphHeight = e->size().height() - 20;
  _scene->setSceneRect(0, 0, _graphWidth, _graphHeight);
  redrawGraph();

  QGraphicsView::resizeEvent(e);
}

void TerrainProfileGraph::mouseMoveEvent(QMouseEvent* e)
{
  drawHoverCursor(mapToScene(e->pos()));
  QGraphicsView::mouseMoveEvent(e);
}

void TerrainProfileGraph::mousePressEvent(QMouseEvent* e)
{
  _selectStart = mapToScene(e->pos()).x();
  _selecting = true;
}

void TerrainProfileGraph::mouseReleaseEvent(QMouseEvent* e)
{
  if (_selecting)
  {
    double selectEnd = mapToScene(e->pos()).x();

    double zoomStart = osg::minimum(_selectStart, selectEnd);
    double zoomEnd = osg::maximum(_selectStart, selectEnd);

    double startDistanceFactor = ((zoomStart - _graphField.x()) / (double)_graphField.width());
    double endDistanceFactor = ((zoomEnd - _graphField.x()) / (double)_graphField.width());

    osg::Vec3d worldStart, worldEnd;
    _calculator->getStart(ALTMODE_ABSOLUTE).toWorld(worldStart);
    _calculator->getEnd(ALTMODE_ABSOLUTE).toWorld(worldEnd);

    double newStartWorldX = (worldEnd.x() - worldStart.x()) * startDistanceFactor + worldStart.x();
    double newStartWorldY = (worldEnd.y() - worldStart.y()) * startDistanceFactor + worldStart.y();
    double newStartWorldZ = (worldEnd.z() - worldStart.z()) * startDistanceFactor + worldStart.z();

    GeoPoint newStart;
    newStart.fromWorld(_calculator->getStart().getSRS(), osg::Vec3d(newStartWorldX, newStartWorldY, newStartWorldZ));
    newStart.z() = 0.0;

    double newEndWorldX = (worldEnd.x() - worldStart.x()) * endDistanceFactor + worldStart.x();
    double newEndWorldY = (worldEnd.y() - worldStart.y()) * endDistanceFactor + worldStart.y();
    double newEndtWorldZ = (worldEnd.z() - worldStart.z()) * endDistanceFactor + worldStart.z();

    GeoPoint newEnd;
    newEnd.fromWorld(_calculator->getStart().getSRS(), osg::Vec3d(newEndWorldX, newEndWorldY, newEndtWorldZ));
    newEnd.z() = 0.0;

    if (osg::absolute(newEnd.x() - newStart.x()) > 0.001 || osg::absolute(newEnd.y() - newStart.y()) > 0.001)
    {
      _calculator->setStartEnd(newStart, newEnd);
    }
    else
    {
      _selecting = false;
      drawHoverCursor(mapToScene(e->pos()));
    }
  }

  _selecting = false;
}

void TerrainProfileGraph::redrawGraph()
{
  _scene->clear();
  _graphLines.clear();
  _graphField.setCoords(0, 0, 0, 0);
  _hoverLine = 0L;

  const osgEarth::Util::TerrainProfile profile = _calculator->getProfile();
  if (profile.getNumElevations() > 0)
  {
    double minElevation, maxElevation;
    profile.getElevationRanges( minElevation, maxElevation );
    _totalDistance = profile.getTotalDistance();

    int mag = (int)pow(10.0, (double)((int)log10(maxElevation - minElevation)));
    if( mag == 0 )
    {
        mag = 1;
    }
    _graphMinY = ((int)(minElevation / mag) - (minElevation < 0 ? 1 : 0)) * mag;
    _graphMaxY = ((int)(maxElevation / mag) + (maxElevation < 0 ? 0 : 1)) * mag;

    int graphRangeY = _graphMaxY - _graphMinY;
    double scale = (double)graphRangeY / 10.0;

    drawAxes(_graphMinY, _graphMaxY, scale, _totalDistance, _graphField);

    double lastX = _graphField.x();
    double lastY = (1.0 - ((profile.getElevation(0) - _graphMinY) / graphRangeY)) * _graphField.height() + _graphField.y();

    QPolygonF graphPoly;
    graphPoly << QPointF(_graphField.x(), _graphField.y() + _graphField.height());
    graphPoly << QPointF(lastX, lastY);

    for (unsigned int i = 0; i < profile.getNumElevations(); i++)
    {
      double distance = profile.getDistance( i );
      double elevation = profile.getElevation( i );

      double x = (distance / _totalDistance) * _graphField.width() + _graphField.x();
      double y = (1.0 - ((elevation - _graphMinY) / graphRangeY)) * _graphField.height() + _graphField.y();

      graphPoly << QPointF(x, y);

      QLineF line(lastX, lastY, x, y);
      _graphLines.push_back(line);
      _scene->addLine(line, _linePen)->setZValue(GRAPH_Z);

      lastX = x;
      lastY = y;
    }

    // Add gradient polygon beneath the graph line
    graphPoly << QPointF(_graphField.x() + _graphField.width(), _graphField.y() + _graphField.height());
    QLinearGradient polyGrad(0, 0, 0, (_graphField.y() + _graphField.height()) * 1.25);
    polyGrad.setColorAt(0, _graphFillColor);
    polyGrad.setColorAt(1, QColor(255, 255, 255, 0));
    polyGrad.setSpread(QGradient::PadSpread);
    _scene->addPolygon(graphPoly, QPen(Qt::NoPen), QBrush(polyGrad))->setZValue(GRAPH_Z - 1);
  }
}

void TerrainProfileGraph::drawAxes(double yMin, double yMax, double yScale, double xMax, QRect &out_field)
{
  QBrush axesBrush(_axesColor);

  // Create min/max text items
  QGraphicsSimpleTextItem* yMinText = new QGraphicsSimpleTextItem(QString::number(yMin));
  yMinText->setBrush(axesBrush);
  yMinText->setFont(_graphFont);

  QGraphicsSimpleTextItem* yMaxText = new QGraphicsSimpleTextItem(QString::number(yMax));
  yMaxText->setBrush(axesBrush);
  yMaxText->setFont(_graphFont);

  QGraphicsSimpleTextItem* xMaxText = new QGraphicsSimpleTextItem(QString::number(xMax));
  xMaxText->setBrush(axesBrush);
  xMaxText->setFont(_graphFont);


  // Calculate positioning offsets and set out_field to actual graph bounds
  double fontHalfHeight = yMinText->boundingRect().height() / 2.0;

  int textSpacing = 8;
  int xOffset = (int)osg::maximum(yMinText->boundingRect().width(), yMaxText->boundingRect().width()) + textSpacing;
  int yOffset = (int)xMaxText->boundingRect().height() + textSpacing;
  int xAxisY = _graphHeight - yOffset;

  out_field.setCoords(xOffset, (int)fontHalfHeight, _graphWidth, xAxisY);


  // Draw background rectangle
  _scene->addRect(out_field, QPen(Qt::NoPen), QBrush(_fieldColor))->setZValue(FIELD_Z);


  // Add min/max text items to the scene
  yMinText->setPos(xOffset - textSpacing - yMinText->boundingRect().width(), xAxisY - fontHalfHeight);
  yMinText->setZValue(AXES_Z);
  _scene->addItem(yMinText);

  yMaxText->setPos(xOffset - textSpacing - yMaxText->boundingRect().width(), 0);
  yMaxText->setZValue(AXES_Z);
  _scene->addItem(yMaxText);

  xMaxText->setPos(_graphWidth - xMaxText->boundingRect().width(), _graphHeight - xMaxText->boundingRect().height());
  xMaxText->setZValue(AXES_Z);
  _scene->addItem(xMaxText);


  // Draw the main axes and x-axis end cap
  _scene->addLine(xOffset, fontHalfHeight, xOffset, xAxisY + 5, _axesPen)->setZValue(AXES_Z);
  _scene->addLine(xOffset - 5, xAxisY, _graphWidth, xAxisY, _axesPen)->setZValue(AXES_Z);
  _scene->addLine(_graphWidth, xAxisY - 5, _graphWidth, xAxisY + 5, _axesPen)->setZValue(AXES_Z);

  // Draw horizontal graph lines
  double yGraphScale = (yScale / (yMax - yMin)) * out_field.height();
  double graphLineY = xAxisY - yGraphScale;
  for (double y = yMin + yScale; y <= yMax; y += yScale)
  {
    _scene->addLine(xOffset - 5, graphLineY, _graphWidth, graphLineY, _axesPen)->setZValue(AXES_Z);;

    if (y != yMax)
    {
      QGraphicsSimpleTextItem* yText = new QGraphicsSimpleTextItem(QString::number(y));
      yText->setBrush(axesBrush);
      yText->setFont(_graphFont);
      yText->setPos(xOffset - textSpacing - yText->boundingRect().width(), graphLineY - fontHalfHeight);
      yText->setZValue(AXES_Z);
      _scene->addItem(yText);
    }

    graphLineY -= yGraphScale;
  }
}

void TerrainProfileGraph::drawHoverCursor(const QPointF& position)
{
  if (_hoverLine)
  {
    _scene->removeItem(_hoverLine);
    delete _hoverLine;
    _hoverLine = 0L;
  }

  if (_graphField.width() < 2 || _graphField.height() < 2)
    return;

  double xPos = position.x() < _graphField.x() ? _graphField.x() : (position.x() > _graphField.x() + _graphField.width() ? _graphField.x() + _graphField.width() : position.x());

  QLineF vLine(xPos, _graphField.y(), xPos, _graphField.y() + _graphField.height());

  QPointF* intersect = new QPointF;
  bool foundIntersect = false;
  for (int i=0; i < _graphLines.count(); i++)
  {
    if (vLine.intersect(_graphLines[i], intersect) == QLineF::BoundedIntersection)
    {
      foundIntersect = true;
      break;
    }
  }

  if (foundIntersect)
  {
    // Draw the upper line segment.  Also serves as the parent item.
    _hoverLine = new QGraphicsLineItem(xPos, _graphField.y(), xPos, intersect->y() - 3);
    _hoverLine->setPen(_hoverPen);
    _hoverLine->setZValue(OVERLAY_Z);
    _scene->addItem(_hoverLine);

    // Draw the box around the intersect point
    QGraphicsRectItem* hoverBox = new QGraphicsRectItem(xPos - 3, intersect->y() - 3, 6, 6);
    hoverBox->setPen(_hoverPen);
    hoverBox->setBrush(Qt::NoBrush);
    hoverBox->setZValue(OVERLAY_Z);
    hoverBox->setParentItem(_hoverLine);

    // Draw the lower line segment
    QGraphicsLineItem* lowerLine = new QGraphicsLineItem(xPos, intersect->y() + 3, xPos, _graphField.y() + _graphField.height() + 5);
    lowerLine->setPen(_hoverPen);
    lowerLine->setZValue(OVERLAY_Z);
    lowerLine->setParentItem(_hoverLine);

    // Draw the text and background
    double y = (1.0 - ((intersect->y() - _graphField.y()) / _graphField.height())) * (_graphMaxY - _graphMinY) + _graphMinY;
    int textOffset = 10;

    QGraphicsSimpleTextItem* hoverText = new QGraphicsSimpleTextItem(QString::number(y) + tr("m"));
    hoverText->setBrush(QBrush(_axesColor));
    hoverText->setFont(_graphFont);
    hoverText->setZValue(OVERLAY_Z);

    if (intersect->x() + textOffset + hoverText->boundingRect().width() < _graphField.x() + _graphField.width())
      hoverText->setPos(intersect->x() + textOffset, intersect->y() - hoverText->boundingRect().height());
    else
      hoverText->setPos(intersect->x() - textOffset - hoverText->boundingRect().width(), intersect->y() - hoverText->boundingRect().height());

    QGraphicsRectItem* hoverTextBackground = new QGraphicsRectItem(hoverText->x() - 3, hoverText->y() - 1, 
                                                                   hoverText->boundingRect().width() + 6,
                                                                   hoverText->boundingRect().height() + 1);
    hoverTextBackground->setPen(_axesPen);
    hoverTextBackground->setBrush(QBrush(_graphColor));
    hoverTextBackground->setZValue(OVERLAY_Z);
    hoverTextBackground->setParentItem(_hoverLine);

    hoverText->setParentItem(_hoverLine);

    // Update callback
    if (_positionCallback.valid())
    {
      double distanceFactor = ((xPos - _graphField.x()) / (double)_graphField.width());

      osg::Vec3d worldStart, worldEnd;
      _calculator->getStart(ALTMODE_ABSOLUTE).toWorld(worldStart);
      _calculator->getEnd(ALTMODE_ABSOLUTE).toWorld(worldEnd);

      double worldX = (worldEnd.x() - worldStart.x()) * distanceFactor + worldStart.x();
      double worldY = (worldEnd.y() - worldStart.y()) * distanceFactor + worldStart.y();
      double worldZ = (worldEnd.z() - worldStart.z()) * distanceFactor + worldStart.z();

      GeoPoint mapPos;
      mapPos.fromWorld(_calculator->getStart().getSRS(), osg::Vec3d(worldX, worldY, worldZ));

      _positionCallback->updatePosition(mapPos.y(), mapPos.x(), hoverText->text().toStdString());
    }
  }
  else
  {
    // No intersect found so just draw the full line at xPos
    _hoverLine = new QGraphicsLineItem(xPos, _graphField.y(), xPos, _graphField.y() + _graphField.height() + 5);
    _hoverLine->setPen(_hoverPen);
    _hoverLine->setZValue(OVERLAY_Z);
    _scene->addItem(_hoverLine);
  }

  // Draw distance text
  double x = ((xPos - _graphField.x()) / _graphField.width()) * _totalDistance;

  BoxedSimpleTextItem* distanceText = new BoxedSimpleTextItem(QString::number(x / 1000.0, 'f', 2) + tr("km"), _backgroundColor);
  distanceText->setBrush(QBrush(_axesColor));
  distanceText->setFont(_graphFont);
  distanceText->setZValue(OVERLAY_Z);
  if(xPos - 2 - distanceText->boundingRect().width() > _graphField.x())
  {
      distanceText->setPos(xPos - 2 - distanceText->boundingRect().width(), _graphField.y() + _graphField.height() + 2);
  }
  else
  {
      distanceText->setPos(xPos + 2, _graphField.y() + _graphField.height() + 2);
  }
  distanceText->setParentItem(_hoverLine);

  // Draw selection box
  drawSelectionBox(xPos);

  delete intersect;
}

void TerrainProfileGraph::drawSelectionBox(double position)
{
  if (_selecting && _selectStart != position)
  {
    double selectionMin = osg::minimum(_selectStart, position);
    double selectionMax = osg::maximum(_selectStart, position);
    QGraphicsRectItem* selectionBox = new QGraphicsRectItem(selectionMin, _graphField.y(), selectionMax - selectionMin, _graphField.height());
    selectionBox->setPen(QPen(Qt::NoPen));
    selectionBox->setBrush(QBrush(QColor(0, 0, 0, 64)));
    selectionBox->setZValue(OVERLAY_Z);
    selectionBox->setParentItem(_hoverLine);
  }
}

void TerrainProfileGraph::onTerrainGraphChanged()
{
  redrawGraph();
}
