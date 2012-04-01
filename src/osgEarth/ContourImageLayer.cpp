/* -*-c++-*- */
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
#include "ContourImageLayer"

using namespace osgEarth;

ContourImageLayer::ContourImageLayer(Map* map, osgEarth::ImageLayerOptions const& options, osg::TransferFunction1D* transferFunction)
                  :ImageLayer(options), _map(map), _transferFunction(transferFunction)
{
	_runtimeOptions.profile()     = _map->getProfile()->toProfileOptions();
	_runtimeOptions.cachePolicy() = CachePolicy::NO_CACHE;
}
  
void ContourImageLayer::initTileSource()
{
	_tileSourceInitialized = true;
}
  
bool ContourImageLayer::isKeyValid( TileKey const& key ) const
{
	return key.getLevelOfDetail() <= _runtimeOptions.maxLevel().value();
}
  
bool ContourImageLayer::isCached( TileKey const& ) const
{
	return true;
}

GeoImage ContourImageLayer::createImage( TileKey const& key, ProgressCallback*, bool )
{
    if (_map.valid())
		if (_transferFunction.valid())
		{
			MapFrame                       mapFrame(_map.get());
			osg::ref_ptr<osg::HeightField> heightField;

			if ( mapFrame.getHeightField( key, true, heightField ) )
			{
				osg::FloatArray const* floats = heightField->getFloatArray();
				osg::Image*            image  = new osg::Image();

				// the contour is calculated as an 8 bit RGBA image
				image->allocateImage(heightField->getNumColumns(), heightField->getNumRows(), 1, GL_RGBA, GL_UNSIGNED_BYTE);
				if (image->isDataContiguous())
				{
					unsigned char* data = image->data();

					for ( unsigned int i=0; i<floats->size(); ++i )
					{
						osg::Vec4 color = _transferFunction->getColor((*floats)[i]);
						
						*data = static_cast<unsigned char>(color[0]*255.0f + 0.5f); ++data;
						*data = static_cast<unsigned char>(color[1]*255.0f + 0.5f); ++data;
						*data = static_cast<unsigned char>(color[2]*255.0f + 0.5f); ++data;
						*data = static_cast<unsigned char>(color[3]*255.0f + 0.5f); ++data;
					}
				}
				else
				{
					unsigned int indexFloats(0);
					
					for (unsigned int i=0; i<heightField->getNumRows(); ++i)
						for (unsigned int j=0; j<heightField->getNumColumns(); ++j)
						{
							osg::Vec4      color = _transferFunction->getColor((*floats)[indexFloats]);
							unsigned char* data  = image->data(j,i);

							*data = static_cast<unsigned char>(color[0]*255.0f + 0.5f); ++data;
							*data = static_cast<unsigned char>(color[1]*255.0f + 0.5f); ++data;
							*data = static_cast<unsigned char>(color[2]*255.0f + 0.5f); ++data;
							*data = static_cast<unsigned char>(color[3]*255.0f + 0.5f);
							++indexFloats;
						}
				}
				return GeoImage( image, key.getExtent() );
			}
			else
				OE_WARN << "[ContourImageLayer] No valid height field available -- cannot create image for contour" << std::endl;
		}
		else
			OE_WARN << "[ContourImageLayer] No valid transfer function available -- cannot create image for contour" << std::endl;
	else
		OE_WARN << "[ContourImageLayer] No valid map available -- cannot create image for contour" << std::endl;

    return GeoImage::INVALID;
}
  
