/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2020 Pelican Mapping
* http://osgearth.org
*
* osgEarth is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/

#include <osgEarth/ImageToHeightFieldConverter>
#include <osgEarth/GeoCommon>

// not needed for GL Core. Only for GL_R32F
#include <osg/Texture>

using namespace osgEarth;
using namespace osgEarth::Util;

namespace
{
    bool isNoData( float f )
    {
        return f == FLT_MAX || f == -FLT_MAX;
    }
}

ImageToHeightFieldConverter::ImageToHeightFieldConverter():
_replace_nodata( false ),
_nodata_value( 0.0f )
{
  //NOP
}

void
ImageToHeightFieldConverter::setRemoveNoDataValues( bool which, float f )
{
  _nodata_value = f;
  _replace_nodata = which;
}

osg::HeightField* ImageToHeightFieldConverter::convert(const osg::Image* image ) {
  if ( !image ) {
    return NULL;
  }

  osg::HeightField* hf;
  if ( image->getPixelSizeInBits() == 32 ) {
    hf = convert32( image );
  } else {
    hf = convert16( image );
  }

  // scan for and replace NODATA values. This algorithm is terrible but good enough for now
  if ( _replace_nodata )
  {
    for( unsigned int row=0; row < hf->getNumRows(); ++row )
    {
      for( unsigned int col=0; col < hf->getNumColumns(); ++col )
      {
        float val = hf->getHeight(col, row);
        if ( !isNoData( val ) ) {
          continue;
        }
        if ( col > 0 )
          val = hf->getHeight(col-1,row);
        else if ( col <= hf->getNumColumns()-1 )
          val = hf->getHeight(col+1,row);

        if ( isNoData( val ) )
        {
          if ( row > 0 )
            val = hf->getHeight(col, row-1);
          else if ( row < hf->getNumRows()-1 )
            val = hf->getHeight(col, row+1);
        }

        if ( isNoData( val ) )
        {
          val = _nodata_value;
        }

        hf->setHeight( col, row, val );
      }
    }
  }

  return hf;
}

osg::HeightField* ImageToHeightFieldConverter::convert16(const osg::Image* image ) const {
  if ( !image ) {
    return NULL;
  }

  osg::HeightField *hf = new osg::HeightField();
  hf->allocate( image->s(), image->t() );

  osg::FloatArray* floats = hf->getFloatArray();

  for( unsigned int i = 0; i < floats->size(); ++i ) {
      short v = *(short*)image->data(i);
      float h = (float)v;
      // Replace short nodata values with our float marker.
      if (v == -SHRT_MAX || v == SHRT_MAX)
      {
          h = NO_DATA_VALUE;
      }
      floats->at( i ) = h;
  }

  return hf;
}

osg::HeightField* ImageToHeightFieldConverter::convert32(const osg::Image* image ) const {
  if ( !image ) {
    return NULL;
  }

  osg::HeightField *hf = new osg::HeightField();
  hf->allocate( image->s(), image->t() );

  memcpy( &hf->getFloatArray()->front(), image->data(), sizeof(float) * hf->getFloatArray()->size() );

  return hf;
}


osg::HeightField*
ImageToHeightFieldConverter::convert(const osg::Image* image, float scaleFactor)
{
  if ( !image ) {
    return NULL;
  }

  osg::HeightField* hf = convert( image );

  // finally, apply the scale factor.
  for( osg::FloatArray::iterator i = hf->getFloatArray()->begin(); i != hf->getFloatArray()->end(); ++i )
  {
    (*i) *= scaleFactor;
  }

  return hf;
}

osg::Image*
ImageToHeightFieldConverter::convert(const osg::HeightField* hf, int pixelSize)
{
  if ( pixelSize == 32 ) {
    return convert32( hf );
  }

  return convert16( hf );
}

osg::Image* ImageToHeightFieldConverter::convert16(const osg::HeightField* hf ) const {
  if ( !hf ) {
    return NULL;
  }

  osg::Image* image = new osg::Image();
  image->allocateImage(hf->getNumColumns(), hf->getNumRows(), 1, GL_LUMINANCE, GL_SHORT);

  const osg::FloatArray* floats = hf->getFloatArray();

  for( unsigned int i = 0; i < floats->size(); ++i  ) {
      float h = floats->at( i );
      // Set NO_DATA_VALUE to a valid short value.
      if (h == NO_DATA_VALUE)
      {        
          h = -SHRT_MAX;
      }
      *(short*)image->data(i) = (short)h;
  }

  return image;
}

osg::Image* ImageToHeightFieldConverter::convertToR32F(const osg::HeightField* hf) const
{
    if (!hf) {
        return NULL;
    }

    osg::Image* image = new osg::Image();
    image->allocateImage(hf->getNumColumns(), hf->getNumRows(), 1, GL_RED, GL_FLOAT);
    image->setInternalTextureFormat(GL_R32F);
    memcpy(image->data(), &hf->getFloatArray()->front(), sizeof(float) * hf->getFloatArray()->size());

    return image;
}

osg::Image* ImageToHeightFieldConverter::convertToR16F(const osg::HeightField* hf) const
{
    if (!hf) {
        return NULL;
    }

    osg::Image* image = new osg::Image();
    image->allocateImage(hf->getNumColumns(), hf->getNumRows(), 1, GL_RED, GL_FLOAT);
    image->setInternalTextureFormat(GL_R16F);
    memcpy(image->data(), &hf->getFloatArray()->front(), sizeof(float) * hf->getFloatArray()->size());

    return image;
}

osg::Image* ImageToHeightFieldConverter::convert32(const osg::HeightField* hf) const {
  if ( !hf ) {
    return NULL;
  }

  osg::Image* image = new osg::Image();
  image->allocateImage(hf->getNumColumns(), hf->getNumRows(), 1, GL_LUMINANCE, GL_FLOAT );
  memcpy( image->data(), &hf->getFloatArray()->front(), sizeof(float) * hf->getFloatArray()->size() );

  return image;
}
