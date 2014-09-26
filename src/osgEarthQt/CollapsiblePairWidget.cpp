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

#include <osgEarthQt/CollapsiblePairWidget>

#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QString>
#include <QSizePolicy>
#include <QVBoxLayout>
#include <QWidget>

using namespace osgEarth::QtGui;

const std::string CollapsiblePairWidget::DEFAULT_STYLESHEET = "#oeFrameContainer, #oeFrameContainer * { background-color: rgba(255, 255, 255, 100%) } #oeItemHeader, #oeItemHeader * { background-color: grey; color: white; }";

CollapsiblePairWidget::CollapsiblePairWidget()
  : _showIcon(":/images/plus.png"), _hideIcon(":/images/minus.png")
{
  initialize();
}

CollapsiblePairWidget::CollapsiblePairWidget(const QString& primaryTitle, QWidget* primaryWidget, const QString& secondaryTitle, QWidget* secondaryWidget)
  : _showIcon(":/images/plus.png"), _hideIcon(":/images/minus.png")
{
  initialize();
  
  _primaryTitle->setText(primaryTitle);

  if (primaryWidget)
    _primaryContainer->layout()->addWidget(primaryWidget);

  _secondaryTitle->setText(secondaryTitle);

  if (secondaryWidget)
  _secondaryContainer->layout()->addWidget(secondaryWidget);
}

void CollapsiblePairWidget::resetStyleSheet()
{
  setStyleSheet(tr(DEFAULT_STYLESHEET.c_str()));
}

void CollapsiblePairWidget::initialize()
{
  // object name for custom stylesheets
  setObjectName("oeFrameContainer");

  // create the main vertical layout
  QVBoxLayout* mainLayout = new QVBoxLayout;
	mainLayout->setSpacing(0);
	mainLayout->setContentsMargins(0, 0, 0, 0);
  setLayout(mainLayout);

  // create parent widget to hold the primary widget and header
  _primaryGroup = new QWidget;
  QVBoxLayout* primaryLayout = new QVBoxLayout;
  primaryLayout->setSpacing(0);
  primaryLayout->setContentsMargins(0, 0, 0, 0);
  _primaryGroup->setLayout(primaryLayout);

  // create primary header
  QFrame* primaryHeader = new QFrame;
  primaryHeader->setFrameStyle(QFrame::Box | QFrame::Plain);
  primaryHeader->setLineWidth(1);
  primaryHeader->setMaximumHeight(20);

  QHBoxLayout* primaryHeaderLayout = new QHBoxLayout;
  primaryHeaderLayout->setSpacing(4);
  primaryHeaderLayout->setContentsMargins(2, 2, 2, 2);
  primaryHeader->setLayout(primaryHeaderLayout);
  primaryHeader->setObjectName("oeItemHeader");

  _primaryTitle = new QLabel(tr("Primary"));
  primaryHeaderLayout->addWidget(_primaryTitle);
  primaryHeaderLayout->addStretch();

  _primaryHideButton = new QPushButton(_hideIcon, tr(""));
  _primaryHideButton->setFlat(true);
  _primaryHideButton->setMaximumSize(16, 16);
  primaryHeaderLayout->addWidget(_primaryHideButton);

  primaryLayout->addWidget(primaryHeader);

  // create primary widget container
  _primaryContainer = new QFrame;
  QVBoxLayout* primaryContainerLayout = new QVBoxLayout;
  primaryContainerLayout->setSpacing(0);
  primaryContainerLayout->setContentsMargins(0, 0, 0, 0);
  _primaryContainer->setLayout(primaryContainerLayout);
  _primaryContainer->setObjectName("oeFrameContainer");
  primaryLayout->addWidget(_primaryContainer);

  mainLayout->addWidget(_primaryGroup);

  // create parent widget to hold the secondary widget and header
  _secondaryGroup = new QWidget;
  _secondaryGroup->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Maximum);
  QVBoxLayout* secondaryGroupLayout = new QVBoxLayout;
  secondaryGroupLayout->setSpacing(0);
  secondaryGroupLayout->setContentsMargins(0, 0, 0, 0);
  _secondaryGroup->setLayout(secondaryGroupLayout);

  //create secondary header
  QFrame* secondaryHeader = new QFrame;
  secondaryHeader->setFrameStyle(QFrame::Box | QFrame::Plain);
  secondaryHeader->setLineWidth(1);
  secondaryHeader->setMaximumHeight(20);

  QHBoxLayout* secondaryHeaderLayout = new QHBoxLayout;
  secondaryHeaderLayout->setSpacing(4);
  secondaryHeaderLayout->setContentsMargins(2, 2, 2, 2);
  secondaryHeader->setLayout(secondaryHeaderLayout);
  secondaryHeader->setObjectName("oeItemHeader");

  _secondaryTitle = new QLabel(tr("Secondary"));
  secondaryHeaderLayout->addWidget(_secondaryTitle);
  secondaryHeaderLayout->addStretch();

  _secondaryHideButton = new QPushButton(_hideIcon, tr(""));
  _secondaryHideButton->setFlat(true);
  _secondaryHideButton->setMaximumSize(16, 16);
  secondaryHeaderLayout->addWidget(_secondaryHideButton);

  secondaryGroupLayout->addWidget(secondaryHeader);

  //create secondary widget container
  _secondaryContainer = new QFrame;
  QVBoxLayout* secondaryContainerLayout = new QVBoxLayout;
  secondaryContainerLayout->setSpacing(0);
  secondaryContainerLayout->setContentsMargins(0, 0, 0, 0);
  _secondaryContainer->setLayout(secondaryContainerLayout);
  _secondaryContainer->setObjectName("oeFrameContainer");
  secondaryGroupLayout->addWidget(_secondaryContainer);

  mainLayout->addWidget(_secondaryGroup);

  //create widget with a stretch child for layout purposes (not ideal)
  _stretchBox = new QWidget;
  QVBoxLayout* stretchLayout = new QVBoxLayout;
  _stretchBox->setLayout(stretchLayout);
  stretchLayout->addStretch();
  mainLayout->addWidget(_stretchBox);
  _stretchBox->setVisible(false);

  //connect show/hide button click events
  connect(_primaryHideButton, SIGNAL(clicked(bool)), this, SLOT(onPrimaryHideClicked(bool)));
  connect(_secondaryHideButton, SIGNAL(clicked(bool)), this, SLOT(onSecondaryHideClicked(bool)));

  resetStyleSheet();
}

void CollapsiblePairWidget::setPrimaryWidget(QWidget* widget)
{
  QLayout* primaryLayout = _primaryContainer->layout();

  QLayoutItem *child;
  while ((child = primaryLayout->takeAt(0)) != 0)
     delete child;

  primaryLayout->addWidget(widget);
}

void CollapsiblePairWidget::setSecondaryWidget(QWidget* widget)
{
  QLayout* secondaryLayout = _secondaryContainer->layout();

  QLayoutItem *child;
  while ((child = secondaryLayout->takeAt(0)) != 0)
     delete child;

  secondaryLayout->addWidget(widget);
}

void CollapsiblePairWidget::setPrimaryCollapsed(bool collapsed)
{
  _primaryHideButton->setIcon(collapsed ? _showIcon : _hideIcon);
  _primaryContainer->setHidden(collapsed);
  _stretchBox->setVisible(collapsed);
}

void CollapsiblePairWidget::setSecondaryCollapsed(bool collapsed)
{
  _secondaryHideButton->setIcon(collapsed ? _showIcon : _hideIcon);
  _secondaryContainer->setHidden(collapsed);
}

void CollapsiblePairWidget::onPrimaryHideClicked(bool checked)
{
  setPrimaryCollapsed(!_primaryContainer->isHidden());
}

void CollapsiblePairWidget::onSecondaryHideClicked(bool checked)
{
  setSecondaryCollapsed(!_secondaryContainer->isHidden());
}
