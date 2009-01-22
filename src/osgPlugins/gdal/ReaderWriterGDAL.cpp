/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2009 Pelican Ventures, Inc.
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

#include <osgEarth/MapConfig>
#include <osgEarth/Mercator>
#include <osgEarth/TileSource>
#include <osgEarth/FileUtils>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgDB/ImageOptions>
#include <sstream>
#include <stdlib.h>

#include <gdal_priv.h>
#include <gdalwarper.h>
#include <ogr_spatialref.h>

#include <OpenThreads/ScopedLock>
#include <OpenThreads/ReentrantMutex>

using namespace osgEarth;

#define PROPERTY_URL            "url"
#define PROPERTY_TILE_SIZE      "tile_size"
#define PROPERTY_MAP_CONFIG     "map_config"


static OpenThreads::ReentrantMutex s_mutex;


class GDALTileSource : public TileSource
{
public:
    GDALTileSource(const osgDB::ReaderWriter::Options* options):
      tile_size(256),
          _srcDS(NULL),
          _warpedDS(NULL)
      {
          static bool s_gdal_registered = false;
          if (!s_gdal_registered)
          {
              GDALAllRegister();
              s_gdal_registered = true;
          }

          if ( options->getPluginData( PROPERTY_URL ) )
              url = std::string( (const char*)options->getPluginData( PROPERTY_URL ) );

          if ( options->getPluginData( PROPERTY_TILE_SIZE ) )
              tile_size = as<int>( (const char*)options->getPluginData( PROPERTY_TILE_SIZE ), 256 );

          if (options->getPluginData( PROPERTY_MAP_CONFIG ))
                _mapConfig = (const MapConfig*)options->getPluginData( PROPERTY_MAP_CONFIG );

          if (url.empty())
          {
              osg::notify(osg::NOTICE) << "GDALTile: No URL specified " << std::endl;
              return;
          }

          std::string path = url;

          //Find the full path to the URL
          //If we have a relative path and the map file contains a server address, just concat the server path and the url together
          if (osgEarth::isRelativePath(path) && osgDB::containsServerAddress(_mapConfig->getFilename()))
          {
              path = osgDB::getFilePath(_mapConfig->getFilename()) + "/" + path;
          }

          //If the path doesn't contain a server address, get the full path to the file.
          if (!osgDB::containsServerAddress(path))
          {
              path = osgEarth::getFullPath(_mapConfig->getFilename(), path);
          }


          //Open the dataset
          _srcDS = (GDALDataset*)GDALOpen( path.c_str(), GA_ReadOnly );
          if (!_srcDS )
          {
              osg::notify(osg::NOTICE) << "Failed to open dataset " << path << std::endl;
              return;
          }

          //Create a spatial reference for the source.
          OGRSpatialReference srcRef;
          std::string src_wkt = _srcDS->GetProjectionRef();
          char* importString = strdup(_srcDS->GetProjectionRef());
          srcRef.importFromWkt(&importString);
          free(importString);

          //Create a spatial reference for Geodetic
          OGRSpatialReference gdRef;
          gdRef.importFromEPSG(4326);
          
          //Try to determine if the profile is Geodetic
          if (gdRef.IsSame(&srcRef))
          {
              _profile = TileGridProfile(TileGridProfile::GLOBAL_GEODETIC);
              osg::notify(osg::NOTICE) << url << " is global-geodetic " << std::endl;
          }

          if (_profile.profileType() == TileGridProfile::UNKNOWN)
          {
              bool isMercator = false;
              //Create a spatial reference for Spherical mercator
              OGRSpatialReference mercRef;
              mercRef.importFromEPSG(900913);
              isMercator = mercRef.IsSame(&srcRef) ? true : false;

              if (!isMercator)
              {
                  mercRef.importFromEPSG(41001);
                  isMercator = mercRef.IsSame(&srcRef) ? true : false;
              }

              if (!isMercator)
              {
                  mercRef.importFromEPSG(3785);
                  isMercator = mercRef.IsSame(&srcRef) ? true : false;
              }

              if (isMercator)
              {
                  _profile = TileGridProfile(TileGridProfile::GLOBAL_MERCATOR);
                  osg::notify(osg::NOTICE) << url << " is global-mercator" << std::endl;
              }
          }

          std::string t_srs;

          //See if we need to autowarp the file to geodetic or mercator
          TileGridProfile mapProfile(_mapConfig->getProfile());
          if (mapProfile.profileType() == TileGridProfile::GLOBAL_GEODETIC && _profile.profileType() != TileGridProfile::GLOBAL_GEODETIC)
          {
              char *wkt = NULL;
              gdRef.exportToWkt(&wkt);
              t_srs = wkt;
              OGRFree(wkt);
              osg::notify(osg::NOTICE) << "Warping " << url << " to global-geodetic " << std::endl;
              _profile = TileGridProfile(TileGridProfile::GLOBAL_GEODETIC);
          }
          if (mapProfile.profileType() == TileGridProfile::GLOBAL_MERCATOR && _profile.profileType() != TileGridProfile::GLOBAL_MERCATOR)
          {
              int epsgCodes[3] = {900913, 3785, 41001};
              bool gotProjection = false;
              OGRSpatialReference mercRef;
              for (int i = 0; i < 3; ++i)
              {
                  if (OGRERR_NONE == mercRef.importFromEPSG(epsgCodes[i]))
                  {
                      gotProjection = true;
                  }
              }

              if (gotProjection)
              {          
                  char *wkt = NULL;
                  mercRef.exportToWkt(&wkt);
                  t_srs = wkt;
                  OGRFree(wkt);
                  osg::notify(osg::NOTICE) << "Warping " << url << " to global-mercator " << std::endl;
                  _profile = TileGridProfile(TileGridProfile::GLOBAL_MERCATOR);
              }
              else
              {
                  osg::notify(osg::NOTICE) << "Could not import mercator projection from EPSG code" << std::endl;
              }
          }

          if (!t_srs.empty())
          {
            //Create a warped VRT going from the source projection to the destination projection.
            _warpedDS = (GDALDataset*)GDALAutoCreateWarpedVRT(_srcDS, src_wkt.c_str(), t_srs.c_str(), GRA_NearestNeighbour, 5.0, NULL);
          }
          else
          {
              _warpedDS = _srcDS;
          }
          
          //Get the geotransform
          _warpedDS->GetGeoTransform(_geotransform);

          //Compute the extents
          pixelToGeo(0.0, _warpedDS->GetRasterYSize(), _extentsMin.x(), _extentsMin.y());
          pixelToGeo(_warpedDS->GetRasterXSize(), 0.0, _extentsMax.x(), _extentsMax.y());

          if (_profile.profileType() == TileGridProfile::UNKNOWN)
          {
              _profile = TileGridProfile(_extentsMin.x(), _extentsMin.y(), _extentsMax.x(), _extentsMax.y(), _warpedDS->GetProjectionRef());
              osg::notify(osg::NOTICE) << url << " is projected" << std::endl;
          }

          //osg::notify(osg::NOTICE) << "Extents " << _extentsMin << ", " << _extentsMax << std::endl;
      }


      ~GDALTileSource()
      {
          if (_warpedDS != _srcDS)
          {
              delete _warpedDS;
          }
          //Close the datasets if it exists
          if (_srcDS) delete _srcDS;
      }

      /**
      * Finds a raster band based on color interpretation
      */
      static GDALRasterBand* findBand(GDALDataset *ds, GDALColorInterp colorInterp)
      {
          for (int i = 1; i <= ds->GetRasterCount(); ++i)
          {
              if (ds->GetRasterBand(i)->GetColorInterpretation() == colorInterp) return ds->GetRasterBand(i);
          }
          return 0;
      }

      void pixelToGeo(double x, double y, double &geoX, double &geoY)
      {
          geoX = _geotransform[0] + _geotransform[1] * x + _geotransform[2] * y;
          geoY = _geotransform[3] + _geotransform[4] * x + _geotransform[5] * y;
      }

      osg::Image* createImage( const TileKey* key )
      {
          OpenThreads::ScopedLock<OpenThreads::ReentrantMutex> lock(s_mutex);
          
          osg::ref_ptr<osg::Image> image;
          if (intersects(key))
          {
              //Get the extents of the tile
              double xmin, ymin, xmax, ymax;
              key->getNativeExtents(xmin, ymin, xmax, ymax);

              int target_width = tile_size;
              int target_height = tile_size;
              int tile_offset_left = 0;
              int tile_offset_top = 0;

              int off_x = int((xmin - _geotransform[0]) / _geotransform[1]);
              int off_y = int((ymax - _geotransform[3]) / _geotransform[5]);
              int width = int(((xmax - _geotransform[0]) / _geotransform[1]) - off_x);
              int height = int(((ymin - _geotransform[3]) / _geotransform[5]) - off_y);

              if (off_x + width > _warpedDS->GetRasterXSize())
              {
                  int oversize_right = off_x + width - _warpedDS->GetRasterXSize();
                  target_width = target_width - int(float(oversize_right) / width * target_width);
                  width = _warpedDS->GetRasterXSize() - off_x;
              }

              if (off_x < 0)
              {
                  int oversize_left = -off_x;
                  tile_offset_left = int(float(oversize_left) / width * target_width);
                  target_width = target_width - int(float(oversize_left) / width * target_width);
                  width = width + off_x;
                  off_x = 0;
              }

              if (off_y + height > _warpedDS->GetRasterYSize())
              {
                  int oversize_bottom = off_y + height - _warpedDS->GetRasterYSize();
                  target_height = target_height - osg::round(float(oversize_bottom) / height * target_height);
                  height = _warpedDS->GetRasterYSize() - off_y;
              }


              if (off_y < 0)
              {
                  int oversize_top = -off_y;
                  tile_offset_top = int(float(oversize_top) / height * target_height);
                  target_height = target_height - int(float(oversize_top) / height * target_height);
                  height = height + off_y;
                  off_y = 0;
              }

              GDALRasterBand* bandRed = findBand(_warpedDS, GCI_RedBand);
              GDALRasterBand* bandGreen = findBand(_warpedDS, GCI_GreenBand);
              GDALRasterBand* bandBlue = findBand(_warpedDS, GCI_BlueBand);
              GDALRasterBand* bandAlpha = findBand(_warpedDS, GCI_AlphaBand);

              if (bandRed && bandGreen && bandBlue)
              {
                  unsigned char *red = new unsigned char[target_width * target_height];
                  unsigned char *green = new unsigned char[target_width * target_height];
                  unsigned char *blue = new unsigned char[target_width * target_height];
                  unsigned char *alpha = new unsigned char[target_width * target_height];

                  //Initialize the alpha values to 255.
                  memset(alpha, 255, target_width * target_height);


                  bandRed->RasterIO(GF_Read, off_x, off_y, width, height, red, target_width, target_height, GDT_Byte, 0, 0);
                  bandGreen->RasterIO(GF_Read, off_x, off_y, width, height, green, target_width, target_height, GDT_Byte, 0, 0);
                  bandBlue->RasterIO(GF_Read, off_x, off_y, width, height, blue, target_width, target_height, GDT_Byte, 0, 0);

                  if (bandAlpha)
                  {
                      bandBlue->RasterIO(GF_Read, off_x, off_y, width, height, alpha, target_width, target_height, GDT_Byte, 0, 0);
                  }

                  image = new osg::Image;
                  image->allocateImage(tile_size, tile_size, 1, GL_RGBA, GL_UNSIGNED_BYTE);
                  memset(image->data(), 0, image->getImageSizeInBytes());

                  for (int src_row = 0, dst_row = tile_offset_top;
                      src_row < target_height;
                      src_row++, dst_row++)
                  {
                      for (int src_col = 0, dst_col = tile_offset_left;
                          src_col < target_width;
                          ++src_col, ++dst_col)
                      {
                          *(image->data(dst_col, dst_row) + 0) = red[src_col + src_row * target_width];
                          *(image->data(dst_col, dst_row) + 1) = green[src_col + src_row * target_width];
                          *(image->data(dst_col, dst_row) + 2) = blue[src_col + src_row * target_width];
                          *(image->data(dst_col, dst_row) + 3) = alpha[src_col + src_row * target_width];
                      }
                  }

                  image->flipVertical();

                  delete []red;
                  delete []green;
                  delete []blue;
                  delete []alpha;
              }
              else
              {
                  osg::notify(osg::NOTICE) << "Could not find red, green and blue bands in " << url << ".  Cannot create image. " << std::endl;
                  return NULL;
              }
          }

          //Create a transparent image if we don't have an image
          if (!image.valid())
          {
              image = new osg::Image();
              image->allocateImage(1,1,1, GL_RGBA, GL_UNSIGNED_BYTE);
              unsigned char *data = image->data(0,0);
              memset(data, 0, 4);
          }
          return image.release();
      }

      osg::HeightField* createHeightField( const TileKey* key )
      {
          OpenThreads::ScopedLock<OpenThreads::ReentrantMutex> lock(s_mutex);

          //Allocate the heightfield
          osg::ref_ptr<osg::HeightField> hf = new osg::HeightField;
          hf->allocate(tile_size, tile_size);
          for(unsigned int i=0; i<hf->getHeightList().size(); i++ ) hf->getHeightList()[i] = 0.0;

          if (intersects(key))
          {
              //Get the meter extents of the tile
              double xmin, ymin, xmax, ymax;
              key->getNativeExtents(xmin, ymin, xmax, ymax);

              int target_width = tile_size;
              int target_height = tile_size;
              int tile_offset_left = 0;
              int tile_offset_top = 0;

              int off_x = int((xmin - _geotransform[0]) / _geotransform[1]);
              int off_y = int((ymax - _geotransform[3]) / _geotransform[5]);
              int width = int(((xmax - _geotransform[0]) / _geotransform[1]) - off_x);
              int height = int(((ymin - _geotransform[3]) / _geotransform[5]) - off_y);

              if (off_x + width > _warpedDS->GetRasterXSize())
              {
                  int oversize_right = off_x + width - _warpedDS->GetRasterXSize();
                  target_width = target_width - int(float(oversize_right) / width * target_width);
                  width = _warpedDS->GetRasterXSize() - off_x;
              }

              if (off_x < 0)
              {
                  int oversize_left = -off_x;
                  tile_offset_left = int(float(oversize_left) / width * target_width);
                  target_width = target_width - int(float(oversize_left) / width * target_width);
                  width = width + off_x;
                  off_x = 0;
              }

              if (off_y + height > _warpedDS->GetRasterYSize())
              {
                  int oversize_bottom = off_y + height - _warpedDS->GetRasterYSize();
                  target_height = target_height - osg::round(float(oversize_bottom) / height * target_height);
                  height = _warpedDS->GetRasterYSize() - off_y;
              }


              if (off_y < 0)
              {
                  int oversize_top = -off_y;
                  tile_offset_top = int(float(oversize_top) / height * target_height);
                  target_height = target_height - int(float(oversize_top) / height * target_height);
                  height = height + off_y;
                  off_y = 0;
              }


              //Just read from the first band
              GDALRasterBand* band = _warpedDS->GetRasterBand(1);

              float *data = new float[target_width * target_height];

              band->RasterIO(GF_Read, off_x, off_y, width, height, data, target_width, target_height, GDT_Float32, 0, 0);

              for (int src_row = 0, dst_row = tile_offset_top;
                  src_row < target_height;
                  src_row++, dst_row++)
              {
                  for (int src_col = 0, dst_col = tile_offset_left;
                      src_col < target_width;
                      ++src_col, ++dst_col)
                  {
                      hf->setHeight(dst_col, dst_row, data[src_col + src_row * target_width]);
                  }
              }

              unsigned int copy_r = hf->getNumRows()-1;
              for(unsigned int r=0;r<copy_r;++r,--copy_r)
              {
                  for(unsigned int c=0;c<hf->getNumColumns();++c)
                  {
                      float temp = hf->getHeight(c,r);
                      hf->setHeight(c,r,hf->getHeight(c,copy_r));
                      hf->setHeight(c,copy_r,temp);
                  }
              }
              delete []data;
          }

          return hf.release();
      }

      virtual int getPixelsPerTile() const
      {
          return tile_size;
      }

      bool intersects(const TileKey* key)
      {
          //Get the native extents of the tile
          double xmin, ymin, xmax, ymax;
          key->getNativeExtents(xmin, ymin, xmax, ymax);

          return  osg::maximum(_extentsMin.x(), xmin) <= osg::minimum(_extentsMax.x(),xmax) &&
                  osg::maximum(_extentsMin.y(), ymin) <= osg::minimum(_extentsMax.y(),ymax);
      }




private:

    GDALDataset* _srcDS;
    GDALDataset* _warpedDS;
    double _geotransform[6];

    osg::Vec2d _extentsMin;
    osg::Vec2d _extentsMax;

    std::string url;
    int tile_size;

    const MapConfig* _mapConfig;
};


class ReaderWriterGDALTile : public osgDB::ReaderWriter
{
public:
    ReaderWriterGDALTile() {}

    virtual const char* className()
    {
        return "GDAL Tile Reader";
    }

    virtual bool acceptsExtension(const std::string& extension) const
    {
        return osgDB::equalCaseInsensitive( extension, "osgearth_gdal" );
    }

    virtual ReadResult readObject(const std::string& file_name, const Options* opt) const
    {
        if ( !acceptsExtension( osgDB::getFileExtension( file_name ) ) )
        {
            return ReadResult::FILE_NOT_HANDLED;
        }
        return new GDALTileSource(opt);
    }
};

REGISTER_OSGPLUGIN(osgearth_gdal, ReaderWriterGDALTile)
