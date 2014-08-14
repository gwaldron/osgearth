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
#include <osgEarthQt/CaptureToClipboardOperation>
#include <osgEarthQt/ViewerWidget>

#include <QtGui>
#include <QApplication>
#include <QClipboard>

using namespace osgEarth;
using namespace osgEarth::QtGui;

void CaptureToClipboardOperation::operator()(const osg::Image& image, const unsigned int /*context_id*/)
{
  GLenum pixFormat = image.getPixelFormat();
  if(pixFormat != GL_RGB && pixFormat != GL_RGBA)
  {
    OE_WARN << "Unsupported pixel format (" << pixFormat << "), CaptureToClipboardOperation supports GL_RGB or GL_RGBA pixel formats" << std::endl;
    return;
  }

  int imageWidth = image.s();
  int imageHeight = image.t();
  int imageDepth = image.r();
  if(imageWidth > 0 && imageHeight > 0 && imageDepth > 0)
  {
    QImage clipImage(imageWidth, imageHeight, QImage::Format_ARGB32);

    QRgb clipPixColor;
    int clipImageRow = imageHeight - 1;
    for(int row = 0; row < imageHeight; ++row, --clipImageRow)
    {
      for(int col = 0; col < imageWidth; ++col)
      {
        const unsigned char* pixValue = image.data(col, row);
        switch(pixFormat)
        {
        case GL_RGB:
          clipPixColor = qRgb(*(pixValue + 0), *(pixValue + 1), *(pixValue + 2));
          break;
        case GL_RGBA:
          clipPixColor = qRgba(*(pixValue + 0), *(pixValue + 1), *(pixValue + 2), *(pixValue + 3));
          break;
        }
        clipImage.setPixel(col, clipImageRow, clipPixColor);
      }
    }

    QApplication::clipboard()->setImage(clipImage);
  }
}
