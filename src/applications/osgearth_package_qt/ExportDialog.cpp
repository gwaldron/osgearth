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

#include "ExportDialog"

#include <QFileDialog>

#include <osgEarth/CacheEstimator>

using namespace osgEarth;
using namespace PackageQt;


ExportDialog::ExportDialog(osgEarth::MapNode* mapNode, const std::string& dir, const osgEarth::Bounds& bounds):
_mapNode(mapNode),
_bounds(bounds)
{
  initUi(dir);
}

void ExportDialog::initUi(const std::string& dir)
{
	_ui.setupUi(this);

  _ui.errorLabel->setStyleSheet("color: red");

  _ui.exportPathEdit->setText(tr(dir.c_str()));

  if (_bounds.width() > 0 && _bounds.height() > 0)
  {
      std::stringstream ss;
      ss << "LL( " << _bounds.yMin() << ", " << _bounds.xMin() << " ) UR( " << _bounds.yMax() << ", " << _bounds.xMax() << " )";
      _ui.boundsLabel->setText(tr(ss.str().c_str()));
      _ui.boundsLabel->setEnabled(true);
      _ui.boundsCheckBox->setEnabled(true);
      _ui.boundsCheckBox->setChecked(true);
  }

  _ui.concurrencySpinBox->setValue(OpenThreads::GetNumberOfProcessors());

  updateEstimate();

  QObject::connect(_ui.exportPathBrowseButton, SIGNAL(clicked()), this, SLOT(showExportBrowse()));
  QObject::connect(_ui.earthFileCheckBox, SIGNAL(toggled(bool)), this, SLOT(updateEarthFilePathEdit()));
  QObject::connect(_ui.maxLevelCheckBox, SIGNAL(toggled(bool)), this, SLOT(updateMaxLevelSpinBox()));
  QObject::connect(_ui.okButton, SIGNAL(clicked()), this, SLOT(validateAndAccept()));
  QObject::connect(_ui.rbModeMP, SIGNAL(toggled(bool)), this, SLOT(updateMode(bool)));
  QObject::connect(_ui.rbModeMT, SIGNAL(toggled(bool)), this, SLOT(updateMode(bool)));
  QObject::connect(_ui.rbModeSingle, SIGNAL(toggled(bool)), this, SLOT(updateMode(bool)));
  QObject::connect(_ui.maxLevelSpinBox, SIGNAL(valueChanged(int)), this, SLOT(maxLevelChanged(int)));
  QObject::connect(_ui.concurrencySpinBox, SIGNAL(valueChanged(int)), this, SLOT(concurrencyChanged(int)));
}

void ExportDialog::showExportBrowse()
{
  QString dir = QFileDialog::getExistingDirectory(this, tr("Export Directory"),
    _ui.exportPathEdit->text().length() > 0 ? _ui.exportPathEdit->text() : QDir::homePath(),
    QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);

	if (!dir.isNull())
		_ui.exportPathEdit->setText(dir);
}

void ExportDialog::updateEarthFilePathEdit()
{
  _ui.earthFilePathEdit->setEnabled(_ui.earthFileCheckBox->isChecked());
}

void ExportDialog::updateMaxLevelSpinBox()
{
  _ui.maxLevelSpinBox->setEnabled(_ui.maxLevelCheckBox->isChecked());
  updateEstimate();
}

void ExportDialog::validateAndAccept()
{
  std::string errMsg = "ERROR: ";
  if (_ui.exportPathEdit->text().isEmpty())
  {
    errMsg += "Export path not set";
  }
  else if (_ui.earthFileCheckBox->isChecked() && _ui.earthFilePathEdit->text().isEmpty())
  {
    errMsg += "Earth file name not set";
  }
  else
  {
    accept();
    return;
  }

  _ui.errorLabel->setText(QString(errMsg.c_str()));
}

void ExportDialog::updateMode(bool checked)
{
    bool multi = _ui.rbModeMP->isChecked() || _ui.rbModeMT->isChecked();
    _ui.concurrencySpinBox->setEnabled(multi);
    updateEstimate();
}

void ExportDialog::maxLevelChanged(int value)
{    
    updateEstimate();
}

void ExportDialog::concurrencyChanged(int value)
{    
    updateEstimate();
}

void ExportDialog::updateEstimate()
{
    CacheEstimator est;    
    est.setProfile(_mapNode->getMap()->getProfile());
    if (useBounds() && _bounds.width() > 0 && _bounds.height() > 0)
    {
        est.addExtent(GeoExtent(_mapNode->getMapSRS(), _bounds));
    }    

    int maxLevel = 10;
    if (maxLevelEnabled())
    {
        maxLevel = getMaxLevel();        
    }
    else
    {
        // Determine the max level from the layers
        maxLevel = 0;
        for (unsigned int i = 0; i < _mapNode->getMap()->getNumImageLayers(); i++)
        {
            osgEarth::ImageLayer* layer = _mapNode->getMap()->getImageLayerAt(i);
            if (layer)
            {
                osgEarth::TileSource* ts = layer->getTileSource();
                if (ts)
                {
                    for (DataExtentList::iterator itr = ts->getDataExtents().begin(); itr != ts->getDataExtents().end(); itr++)
                    {
                        if (itr->maxLevel().isSet() && itr->maxLevel().value() > maxLevel)
                        {
                            maxLevel = itr->maxLevel().value();
                        }
                    }
                }
            }
        }

        for (unsigned int i = 0; i < _mapNode->getMap()->getNumElevationLayers(); i++)
        {
            osgEarth::ElevationLayer* layer = _mapNode->getMap()->getElevationLayerAt(i);
            if (layer)
            {
                osgEarth::TileSource* ts = layer->getTileSource();
                if (ts)
                {
                    for (DataExtentList::iterator itr = ts->getDataExtents().begin(); itr != ts->getDataExtents().end(); itr++)
                    {
                        if (itr->maxLevel().isSet() && itr->maxLevel().value() > maxLevel)
                        {
                            maxLevel = itr->maxLevel().value();
                        }
                    }
                }
            }
        }
    }

    est.setMaxLevel(maxLevel);    

    std::stringstream buf;
    double totalSeconds = est.getTotalTimeInSeconds();
    TMSExporter::ProcessingMode mode = getProcessingMode();
    // If we are using multiple threads or processes assume it will scale linearly.
    if (mode == TMSExporter::MODE_MULTIPROCESS || mode == TMSExporter::MODE_MULTITHREADED)
    {
        totalSeconds /= (double)getConcurrency();
    }

    // Adjust everything by the # of layers
    unsigned int numLayers = _mapNode->getMap()->getNumImageLayers() + _mapNode->getMap()->getNumElevationLayers();
    totalSeconds *= (double)numLayers;
    unsigned int numTiles = est.getNumTiles() * numLayers;
    double sizeMB = est.getSizeInMB() * (double)numLayers;

    std::string timeString = prettyPrintTime(totalSeconds);
    buf << "Estimate: Max level=" << maxLevel << "  " << numTiles << " tiles.  " << sizeMB << " MB.  " << timeString;
    _ui.estimateLabel->setText(QString::fromStdString(buf.str()));
}

