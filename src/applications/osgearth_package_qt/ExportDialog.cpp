/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2013 Pelican Mapping
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

using namespace PackageQt;


ExportDialog::ExportDialog(const std::string& dir, const std::string& boundsString)
{
  initUi(dir, boundsString);
}

void ExportDialog::initUi(const std::string& dir, const std::string& boundsString)
{
	_ui.setupUi(this);

  _ui.errorLabel->setStyleSheet("color: red");

  _ui.exportPathEdit->setText(tr(dir.c_str()));

  if (boundsString.length() > 0)
  {
    _ui.boundsLabel->setText(tr(boundsString.c_str()));
    _ui.boundsLabel->setEnabled(true);
    _ui.boundsCheckBox->setEnabled(true);
    _ui.boundsCheckBox->setChecked(true);
  }

  QObject::connect(_ui.exportPathBrowseButton, SIGNAL(clicked()), this, SLOT(showExportBrowse()));
  QObject::connect(_ui.earthFileCheckBox, SIGNAL(toggled(bool)), this, SLOT(updateEarthFilePathEdit()));
	QObject::connect(_ui.maxLevelCheckBox, SIGNAL(toggled(bool)), this, SLOT(updateMaxLevelSpinBox()));
//  QObject::connect(_ui.extensionCheckBox, SIGNAL(toggled(bool)), this, SLOT(updateExtensionComboBox()));
  QObject::connect(_ui.okButton, SIGNAL(clicked()), this, SLOT(validateAndAccept()));
}

void ExportDialog::showExportBrowse()
{
  QString dir = QFileDialog::getExistingDirectory(this, tr("Export Directory"),
    _ui.exportPathEdit->text().length() > 0 ? _ui.exportPathEdit->text() : QDir::homePath(),
    QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);

	if (!dir.isNull())
		_ui.exportPathEdit->setText(dir);
}

//void ExportDialog::showEarthFileBrowse()
//{
//  QString path = QFileDialog::getSaveFileName(this, tr("Earth File"),
//    _ui.earthFilePathEdit->text().length() > 0 ? _ui.earthFilePathEdit->text() : QDir::homePath() + QDir::separator() + "out.earth",
//    tr("Earth Files (*.earth)"));
//
//	if (!path.isNull())
//		_ui.earthFilePathEdit->setText(path);
//}

void ExportDialog::updateEarthFilePathEdit()
{
  _ui.earthFilePathEdit->setEnabled(_ui.earthFileCheckBox->isChecked());
}

void ExportDialog::updateMaxLevelSpinBox()
{
  _ui.maxLevelSpinBox->setEnabled(_ui.maxLevelCheckBox->isChecked());
}

//void ExportDialog::updateExtensionComboBox()
//{
//  _ui.extensionComboBox->setEnabled(_ui.extensionCheckBox->isChecked());
//}

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