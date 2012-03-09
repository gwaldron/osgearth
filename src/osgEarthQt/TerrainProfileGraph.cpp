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
#include <osgEarthQt/TerrainProfileGraph>
#include <osgEarthQt/Actions>
#include <osgEarthQt/DataManager>
#include <osgEarthQt/GuiActions>

#include <osgEarth/Map>
#include <osgEarthUtil/TerrainProfile>

#include <QGraphicsRectItem>
#include <QGraphicsScene>
#include <QGraphicsSimpleTextItem>
#include <QGraphicsView>
#include <QMouseEvent>
#include <QPointF>
#include <QPolygonF>
#include <QRect>
#include <QResizeEvent>
#include <QToolTip>


using namespace osgEarth;
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
}

//---------------------------------------------------------------------------

const int TerrainProfileGraph::FIELD_Z = 0;
const int TerrainProfileGraph::AXES_Z = 10;
const int TerrainProfileGraph::GRAPH_Z = 20;
const int TerrainProfileGraph::OVERLAY_Z = 30;


TerrainProfileGraph::TerrainProfileGraph(osgEarth::Util::TerrainProfileCalculator* calculator)
  : QGraphicsView(), _calculator(calculator), _graphFont(tr("Helvetica,Verdana,Arial"), 10),
    _backgroundColor(128, 128, 128), _fieldColor(204, 204, 204), _axesColor(255, 255, 255), 
    _graphColor(0, 128, 0), _graphFillColor(128, 255, 128, 192), _graphField(0, 0, 0, 0),
    _totalDistance(0.0), _graphMinY(0), _graphMaxY(0), _graphWidth(500), _graphHeight(309)
{
  setMouseTracking(true);

  _graphFont.setStyleHint(QFont::SansSerif);

  _linePen.setWidth(3);
  _linePen.setBrush(QBrush(_graphColor));

  _axesPen.setWidth(1);
  _axesPen.setBrush(QBrush(_axesColor));

  _scene = new QGraphicsScene(0, 0, _graphWidth, _graphHeight);
  _scene->setBackgroundBrush(QBrush(_backgroundColor));
  setScene(_scene);
  
  redrawGraph();

  _graphChangedCallback = new GraphChangedCallback(/*this*/);
  connect(_graphChangedCallback, SIGNAL(graphChanged()), this, SLOT(onGraphChanged()), Qt::QueuedConnection);
  if (_calculator.valid())
    _calculator->addChangedCallback(_graphChangedCallback);
}

TerrainProfileGraph::~TerrainProfileGraph()
{
  if (_calculator.valid())
    _calculator->removeChangedCallback(_graphChangedCallback);
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
  if (_scene->items().count() > 0)
  {
    QPointF scenePoint = mapToScene(e->pos());
    if (_graphField.contains((int)scenePoint.x(), (int)scenePoint.y()))
    {
      double x = ((scenePoint.x() - _graphField.x()) / _graphField.width()) * _totalDistance;
      double y = (1.0 - ((scenePoint.y() - _graphField.y()) / _graphField.height())) * (_graphMaxY - _graphMinY) + _graphMinY;

      QPointF hoverPos(scenePoint.x() + 4.0, scenePoint.y() - 12.0);

      if (!_hoverTextItem)
      {
        _hoverTextItem = new BoxedSimpleTextItem(QString::number(x) + tr(", ") + QString::number(y), _backgroundColor);
        _hoverTextItem->setBrush(QBrush(_axesColor));
        _hoverTextItem->setFont(_graphFont);
        _hoverTextItem->setPos(hoverPos);
        _hoverTextItem->setZValue(OVERLAY_Z);
        _scene->addItem(_hoverTextItem);
      }
      else
      {
        _hoverTextItem->setText(QString::number(x) + tr(", ") + QString::number(y));
        _hoverTextItem->setPos(hoverPos);
      }

      if (_hoverTextItem->x() + _hoverTextItem->boundingRect().width() > _graphField.x() + _graphField.width())
        _hoverTextItem->setPos(scenePoint.x() - 4.0 - _hoverTextItem->boundingRect().width(), scenePoint.y() - 12.0);
    }
    else if (_hoverTextItem)
    {
      _scene->removeItem(_hoverTextItem);
      _hoverTextItem = 0L;
    }
  }

  QGraphicsView::mouseMoveEvent(e);
}

void TerrainProfileGraph::redrawGraph()
{
  _scene->clear();
  _hoverTextItem = 0L;

  const osgEarth::Util::TerrainProfile profile = _calculator->getProfile();
  if (profile.getNumElevations() > 0)
  {
    double minElevation, maxElevation;
    profile.getElevationRanges( minElevation, maxElevation );

    _totalDistance = profile.getTotalDistance();

    int mag = (int)pow(10.0, (double)((int)log10(maxElevation - minElevation)));
    _graphMinY = ((int)(minElevation / mag)) * mag;
    _graphMaxY = ((int)(maxElevation / mag) + 1) * mag;
    int graphRangeY = _graphMaxY - _graphMinY;
    double scale = (double)graphRangeY / 10.0;

    drawAxes(_graphMinY, _graphMaxY, scale, _totalDistance, _graphField);

    QPolygonF graphPoly;
    graphPoly << QPointF(_graphField.x(), _graphField.y() + _graphField.height());

    double lastX, lastY;
    for (unsigned int i = 0; i < profile.getNumElevations(); i++)
    {
      double distance = profile.getDistance( i );
      double elevation = profile.getElevation( i );

      double x = (distance / _totalDistance) * _graphField.width() + _graphField.x();
      double y = (1.0 - ((elevation - _graphMinY) / graphRangeY)) * _graphField.height() + _graphField.y();

      graphPoly << QPointF(x, y);

      if (i > 0)
        _scene->addLine(lastX, lastY, x, y, _linePen)->setZValue(GRAPH_Z);

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

  int textSpacing = 10;
  int xOffset = (int)osg::maximum(yMinText->boundingRect().width(), yMaxText->boundingRect().width()) + textSpacing;
  int yOffset = (int)xMaxText->boundingRect().height() + textSpacing;
  int xAxisY = _graphHeight - yOffset;

  out_field.setCoords(xOffset, fontHalfHeight, _graphWidth, xAxisY);


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

void TerrainProfileGraph::onGraphChanged()
{
  redrawGraph();
}