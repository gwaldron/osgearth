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
      srcDS(NULL),
      warpedDS(NULL)
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
              mapConfig = (const MapConfig*)options->getPluginData( PROPERTY_MAP_CONFIG );

          if (url.empty())
          {
              osg::notify(osg::NOTICE) << "GDALTile: No URL specified " << std::endl;
              return;
          }

          std::string path = url;

          //Find the full path to the URL
          //If we have a relative path and the map file contains a server address, just concat the server path and the url together
          if (osgEarth::isRelativePath(path) && osgDB::containsServerAddress(mapConfig->getFilename()))
          {
              path = osgDB::getFilePath(mapConfig->getFilename()) + "/" + path;
          }

          //If the path doesn't contain a server address, get the full path to the file.
          if (!osgDB::containsServerAddress(path))
          {
              path = osgEarth::getFullPath(mapConfig->getFilename(), path);
          }


          //Open the dataset
          srcDS = (GDALDataset*)GDALOpen( path.c_str(), GA_ReadOnly );
          if ( !srcDS )
          {
              osg::notify(osg::NOTICE) << "Failed to open dataset " << path << std::endl;
              return;
          }

          //Create a spatial reference for the source.
          OGRSpatialReference srcRef;
          std::string src_wkt = srcDS->GetProjectionRef();
          char* projection_string = strdup(src_wkt.c_str());
          char* importString = projection_string;
          srcRef.importFromWkt(&importString);
          free(projection_string);

          //Create a spatial reference for Geodetic
          OGRSpatialReference gdRef;
          gdRef.importFromEPSG(4326);
          
          //Try to determine if the profile is Geodetic
          if (gdRef.IsSame(&srcRef))
          {
              profile = TileGridProfile(TileGridProfile::GLOBAL_GEODETIC);
              osg::notify(osg::NOTICE) << url << " is global-geodetic " << std::endl;
          }

          if (profile.getProfileType() == TileGridProfile::UNKNOWN)
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
                  mercRef.importFromEPSG(54004);
                  isMercator = mercRef.IsSame(&srcRef) ? true : false;
              }

              if (isMercator)
              {
                  profile = TileGridProfile(TileGridProfile::GLOBAL_MERCATOR);
                  osg::notify(osg::NOTICE) << url << " is global-mercator" << std::endl;
              }
          }

          std::string t_srs;

          //See if we need to autowarp the file to geodetic or mercator
          TileGridProfile mapProfile = mapConfig->getProfile();
          if (mapProfile.getProfileType() == TileGridProfile::GLOBAL_GEODETIC && profile.getProfileType() != TileGridProfile::GLOBAL_GEODETIC)
          {
              char *wkt = NULL;
              gdRef.exportToWkt(&wkt);
              t_srs = wkt;
              OGRFree(wkt);
              osg::notify(osg::NOTICE) << "Warping " << url << " to global-geodetic " << std::endl;
              profile = mapProfile;
          }
          if (mapProfile.getProfileType() == TileGridProfile::GLOBAL_MERCATOR && profile.getProfileType() != TileGridProfile::GLOBAL_MERCATOR)
          {
              int epsgCodes[3] = {900913, 54004, 41001};
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
                  profile = mapProfile;
              }
              else
              {
                  osg::notify(osg::NOTICE) << "Could not import mercator projection from EPSG code" << std::endl;
              }
          }
          //TODO:  We need a better way of determining if the SRS's of the profile match.
          else if ((mapProfile.getProfileType() != TileGridProfile::UNKNOWN) && (mapProfile.srs() == src_wkt))
          {
              //If the map profile's SRS actually match, just set our profile to the Map profile.
              profile = mapProfile;
          }

          if (!t_srs.empty())
          {
            //Create a warped VRT going from the source projection to the destination projection.
            warpedDS = (GDALDataset*)GDALAutoCreateWarpedVRT(srcDS, src_wkt.c_str(), t_srs.c_str(), GRA_NearestNeighbour, 5.0, NULL);
          }
          else
          {
              warpedDS = srcDS;
          }
          
          //Get the geotransform
          warpedDS->GetGeoTransform(geotransform);

          //Compute the extents
          pixelToGeo(0.0, warpedDS->GetRasterYSize(), extentsMin.x(), extentsMin.y());
          pixelToGeo(warpedDS->GetRasterXSize(), 0.0, extentsMax.x(), extentsMax.y());

          if (profile.getProfileType() == TileGridProfile::UNKNOWN)
          {
              profile = TileGridProfile(TileGridProfile::PROJECTED, extentsMin.x(), extentsMin.y(), extentsMax.x(), extentsMax.y(), warpedDS->GetProjectionRef());
              osg::notify(osg::NOTICE) << url << " is projected" << std::endl;
          }

          //osg::notify(osg::NOTICE) << "Extents " << _extentsMin << ", " << _extentsMax << std::endl;
      }


      ~GDALTileSource()
      {
          if (warpedDS != srcDS)
          {
              delete warpedDS;
          }
          //Close the datasets if it exists
          if (srcDS) delete srcDS;
      }

      const TileGridProfile& getProfile() const
      {
          return profile;
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
          geoX = geotransform[0] + geotransform[1] * x + geotransform[2] * y;
          geoY = geotransform[3] + geotransform[4] * x + geotransform[5] * y;
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

              int off_x = int((xmin - geotransform[0]) / geotransform[1]);
              int off_y = int((ymax - geotransform[3]) / geotransform[5]);
              int width = int(((xmax - geotransform[0]) / geotransform[1]) - off_x);
              int height = int(((ymin - geotransform[3]) / geotransform[5]) - off_y);

              if (off_x + width > warpedDS->GetRasterXSize())
              {
                  int oversize_right = off_x + width - warpedDS->GetRasterXSize();
                  target_width = target_width - int(float(oversize_right) / width * target_width);
                  width = warpedDS->GetRasterXSize() - off_x;
              }

              if (off_x < 0)
              {
                  int oversize_left = -off_x;
                  tile_offset_left = int(float(oversize_left) / width * target_width);
                  target_width = target_width - int(float(oversize_left) / width * target_width);
                  width = width + off_x;
                  off_x = 0;
              }

              if (off_y + height > warpedDS->GetRasterYSize())
              {
                  int oversize_bottom = off_y + height - warpedDS->GetRasterYSize();
                  target_height = target_height - osg::round(float(oversize_bottom) / height * target_height);
                  height = warpedDS->GetRasterYSize() - off_y;
              }


              if (off_y < 0)
              {
                  int oversize_top = -off_y;
                  tile_offset_top = int(float(oversize_top) / height * target_height);
                  target_height = target_height - int(float(oversize_top) / height * target_height);
                  height = height + off_y;
                  off_y = 0;
              }

              GDALRasterBand* bandRed = findBand(warpedDS, GCI_RedBand);
              GDALRasterBand* bandGreen = findBand(warpedDS, GCI_GreenBand);
              GDALRasterBand* bandBlue = findBand(warpedDS, GCI_BlueBand);
              GDALRasterBand* bandAlpha = findBand(warpedDS, GCI_AlphaBand);

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

      bool isValidValue(float v, GDALRasterBand* band)
      {
          float noDataValue = -32767.0f;
          int success;
          float value = band->GetNoDataValue(&success);
          if (success)
          {
              noDataValue = value;
          }

          float minValue = -32000.0f;

          if (noDataValue == v) return false;
          if (v < minValue) return false;
          return true;
      }

      float getInterpolatedValue(GDALRasterBand *band, double x, double y)
      {
          double invTransform[6];
          GDALInvGeoTransform(geotransform, invTransform);
          double r, c;
          GDALApplyGeoTransform(invTransform, x, y, &c, &r);

          int rowMin = osg::maximum((int)floor(r), 0);
          int rowMax = osg::maximum(osg::minimum((int)ceil(r), (int)(warpedDS->GetRasterYSize()-1)), 0);
          int colMin = osg::maximum((int)floor(c), 0);
          int colMax = osg::maximum(osg::minimum((int)ceil(c), (int)(warpedDS->GetRasterXSize()-1)), 0);

          if (rowMin > rowMax) rowMin = rowMax;
          if (colMin > colMax) colMin = colMax;

          float urHeight, llHeight, ulHeight, lrHeight;

          band->RasterIO(GF_Read, colMin, rowMin, 1, 1, &llHeight, 1, 1, GDT_Float32, 0, 0);
          band->RasterIO(GF_Read, colMin, rowMax, 1, 1, &ulHeight, 1, 1, GDT_Float32, 0, 0);
          band->RasterIO(GF_Read, colMax, rowMin, 1, 1, &lrHeight, 1, 1, GDT_Float32, 0, 0);
          band->RasterIO(GF_Read, colMax, rowMax, 1, 1, &urHeight, 1, 1, GDT_Float32, 0, 0);

          if (!isValidValue(urHeight, band)) urHeight = 0.0f;
          if (!isValidValue(llHeight, band)) llHeight = 0.0f;
          if (!isValidValue(ulHeight, band)) ulHeight = 0.0f;
          if (!isValidValue(lrHeight, band)) lrHeight = 0.0f;
          
          double x_rem = c - (int)c;
          double y_rem = r - (int)r;

          double w00 = (1.0 - y_rem) * (1.0 - x_rem) * (double)llHeight;
          double w01 = (1.0 - y_rem) * x_rem * (double)lrHeight;
          double w10 = y_rem * (1.0 - x_rem) * (double)ulHeight;
          double w11 = y_rem * x_rem * (double)urHeight;

          float result = (float)(w00 + w01 + w10 + w11);

          return result;
      }


      osg::HeightField* createHeightField( const TileKey* key )
      {
          OpenThreads::ScopedLock<OpenThreads::ReentrantMutex> lock(s_mutex);

          //Allocate the heightfield
          osg::ref_ptr<osg::HeightField> hf = new osg::HeightField;
          hf->allocate(tile_size, tile_size);

          if (intersects(key))
          {
              //Get the meter extents of the tile
              double xmin, ymin, xmax, ymax;
              key->getNativeExtents(xmin, ymin, xmax, ymax);

              //Just read from the first band
              GDALRasterBand* band = warpedDS->GetRasterBand(1);

              double dx = (xmax - xmin) / (tile_size-1);
              double dy = (ymax - ymin) / (tile_size-1);

              for (int c = 0; c < tile_size; ++c)
              {
                  double geoX = xmin + (dx * (double)c);
                  for (int r = 0; r < tile_size; ++r)
                  {
                      double geoY = ymin + (dy * (double)r);
                      float h = getInterpolatedValue(band, geoX, geoY);
                      hf->setHeight(c, r, h);
                  }
              }
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

          return  osg::maximum(extentsMin.x(), xmin) <= osg::minimum(extentsMax.x(), xmax) &&
                  osg::maximum(extentsMin.y(), ymin) <= osg::minimum(extentsMax.y(), ymax);
      }




private:

    GDALDataset* srcDS;
    GDALDataset* warpedDS;
    double       geotransform[6];

    osg::Vec2d extentsMin;
    osg::Vec2d extentsMax;

    std::string     url;
    int             tile_size;
    TileGridProfile profile;

    const MapConfig* mapConfig;
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
