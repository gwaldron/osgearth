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

#include <osgEarth/Mercator>
#include <osgEarth/TileSource>
#include <osgEarth/FileUtils>
#include <osgEarth/Registry>

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

//GDAL proxy is only available after GDAL 1.6
#if ((GDAL_VERSION_MAJOR >= 1) && (GDAL_VERSION_MINOR >= 6))
#include <gdal_proxy.h>
#endif

#include <cpl_string.h>

//GDAL VRT api is only available after 1.5.0
#if ((GDAL_VERSION_MAJOR >= 1) && (GDAL_VERSION_MINOR >= 5))
#include <gdal_vrt.h>
#endif

#include <OpenThreads/ScopedLock>
#include <OpenThreads/ReentrantMutex>

using namespace std;
using namespace osgEarth;

#define PROPERTY_URL            "url"
#define PROPERTY_TILE_SIZE      "tile_size"
#define PROPERTY_EXTENTSIONS    "extensions"
#define PROPERTY_INTERPOLATION  "interpolation"

static OpenThreads::ReentrantMutex s_mutex;

#define GEOTRSFRM_TOPLEFT_X            0
#define GEOTRSFRM_WE_RES               1
#define GEOTRSFRM_ROTATION_PARAM1      2
#define GEOTRSFRM_TOPLEFT_Y            3
#define GEOTRSFRM_ROTATION_PARAM2      4
#define GEOTRSFRM_NS_RES               5



typedef enum
{
    LOWEST_RESOLUTION,
    HIGHEST_RESOLUTION,
    AVERAGE_RESOLUTION
} ResolutionStrategy;

typedef struct
{
    int    isFileOK;
    int    nRasterXSize;
    int    nRasterYSize;
    double adfGeoTransform[6];
    int    nBlockXSize;
    int    nBlockYSize;
} DatasetProperty;

typedef struct
{
    GDALColorInterp        colorInterpretation;
    GDALDataType           dataType;
    GDALColorTableH        colorTable;
    int                    bHasNoData;
    double                 noDataValue;
} BandProperty;


static void
tokenize(const string& str,
         vector<string>& tokens,
         const string& delimiters = " ")
{
    // Skip delimiters at beginning.
    string::size_type lastPos = str.find_first_not_of(delimiters, 0);
    // Find first "non-delimiter".
    string::size_type pos     = str.find_first_of(delimiters, lastPos);

    while (string::npos != pos || string::npos != lastPos)
    {
        // Found a token, add it to the vector.
        tokens.push_back(str.substr(lastPos, pos - lastPos));
        // Skip delimiters.  Note the "not_of"
        lastPos = str.find_first_not_of(delimiters, pos);
        // Find next "non-delimiter"
        pos = str.find_first_of(delimiters, lastPos);
    }
}

static void
getFiles(const std::string &file, const std::vector<std::string> &exts, std::vector<std::string> &files)
{
    if (osgDB::fileType(file) == osgDB::DIRECTORY)
    {
        osgDB::DirectoryContents contents = osgDB::getDirectoryContents(file);
        for (osgDB::DirectoryContents::iterator itr = contents.begin(); itr != contents.end(); ++itr)
        {
            if (*itr == "." || *itr == "..") continue;
            std::string f = osgDB::concatPaths(file, *itr);
            getFiles(f, exts, files);
        }
    }
    else
    {
        bool fileValid = false;
        //If we have no _extensions specified, assume we should try everything
        if (exts.size() == 0)
        {
            fileValid = true;
        }
        else
        {
            //Only accept files with the given _extensions
            std::string ext = osgDB::getFileExtension(file);
            for (unsigned int i = 0; i < exts.size(); ++i)
            {
                if (osgDB::equalCaseInsensitive(ext, exts[i]))
                {
                    fileValid = true;
                    break;
                }
            }
        }
        
        if (fileValid)
        {
          files.push_back(osgDB::convertFileNameToNativeStyle(file));
        }
    }
}

//The VRT API is only available after GDAL 1.5.0
#if ((GDAL_VERSION_MAJOR >= 1) && (GDAL_VERSION_MINOR >= 5))
//Adapted from the gdalbuildvrt application
static GDALDatasetH
build_vrt(std::vector<std::string> &files, ResolutionStrategy resolutionStrategy)
{
    char* projectionRef = NULL;
    int nBands = 0;
    BandProperty* bandProperties = NULL;
    double minX = 0, minY = 0, maxX = 0, maxY = 0;
    int i,j;
    double we_res = 0;
    double ns_res = 0;
    int rasterXSize;
    int rasterYSize;
    int nCount = 0;
    int bFirst = TRUE;
    VRTDatasetH hVRTDS = NULL;

    int nInputFiles = files.size();

    DatasetProperty* psDatasetProperties =
            (DatasetProperty*) CPLMalloc(nInputFiles*sizeof(DatasetProperty));

    for(i=0;i<nInputFiles;i++)
    {
        const char* dsFileName = files[i].c_str();

        GDALTermProgress( 1.0 * (i+1) / nInputFiles, NULL, NULL);

        GDALDatasetH hDS = GDALOpen(dsFileName, GA_ReadOnly );
        psDatasetProperties[i].isFileOK = FALSE;

        if (hDS)
        {
            const char* proj = GDALGetProjectionRef(hDS);
            GDALGetGeoTransform(hDS, psDatasetProperties[i].adfGeoTransform);
            if (psDatasetProperties[i].adfGeoTransform[GEOTRSFRM_ROTATION_PARAM1] != 0 ||
                psDatasetProperties[i].adfGeoTransform[GEOTRSFRM_ROTATION_PARAM2] != 0)
            {
                fprintf( stderr, "GDAL Driver does not support rotated geo transforms. Skipping %s\n",
                             dsFileName);
                GDALClose(hDS);
                continue;
            }
            if (psDatasetProperties[i].adfGeoTransform[GEOTRSFRM_NS_RES] >= 0)
            {
                fprintf( stderr, "GDAL Driver does not support positive NS resolution. Skipping %s\n",
                             dsFileName);
                GDALClose(hDS);
                continue;
            }
            psDatasetProperties[i].nRasterXSize = GDALGetRasterXSize(hDS);
            psDatasetProperties[i].nRasterYSize = GDALGetRasterYSize(hDS);
            double product_minX = psDatasetProperties[i].adfGeoTransform[GEOTRSFRM_TOPLEFT_X];
            double product_maxY = psDatasetProperties[i].adfGeoTransform[GEOTRSFRM_TOPLEFT_Y];
            double product_maxX = product_minX +
                        GDALGetRasterXSize(hDS) * psDatasetProperties[i].adfGeoTransform[GEOTRSFRM_WE_RES];
            double product_minY = product_maxY +
                        GDALGetRasterYSize(hDS) * psDatasetProperties[i].adfGeoTransform[GEOTRSFRM_NS_RES];

            GDALGetBlockSize(GDALGetRasterBand( hDS, 1 ),
                             &psDatasetProperties[i].nBlockXSize,
                             &psDatasetProperties[i].nBlockYSize);

            if (bFirst)
            {
                if (proj)
                    projectionRef = CPLStrdup(proj);
                minX = product_minX;
                minY = product_minY;
                maxX = product_maxX;
                maxY = product_maxY;
                nBands = GDALGetRasterCount(hDS);
                bandProperties = (BandProperty*)CPLMalloc(nBands*sizeof(BandProperty));
                for(j=0;j<nBands;j++)
                {
                    GDALRasterBandH hRasterBand = GDALGetRasterBand( hDS, j+1 );
                    bandProperties[j].colorInterpretation = GDALGetRasterColorInterpretation(hRasterBand);
                    bandProperties[j].dataType = GDALGetRasterDataType(hRasterBand);
                    if (bandProperties[j].colorInterpretation == GCI_PaletteIndex)
                    {
                        bandProperties[j].colorTable = GDALGetRasterColorTable( hRasterBand );
                        if (bandProperties[j].colorTable)
                        {
                            bandProperties[j].colorTable = GDALCloneColorTable(bandProperties[j].colorTable);
                        }
                    }
                    else
                        bandProperties[j].colorTable = 0;
                    bandProperties[j].noDataValue = GDALGetRasterNoDataValue(hRasterBand, &bandProperties[j].bHasNoData);
                }
            }
            else
            {
                if ((proj != NULL && projectionRef == NULL) ||
                    (proj == NULL && projectionRef != NULL) ||
                    (proj != NULL && projectionRef != NULL && EQUAL(proj, projectionRef) == FALSE))
                {
                    fprintf( stderr, "gdalbuildvrt does not support heterogenous projection. Skipping %s\n",dsFileName);
                    GDALClose(hDS);
                    continue;
                }
                int _nBands = GDALGetRasterCount(hDS);
                if (nBands != _nBands)
                {
                    fprintf( stderr, "gdalbuildvrt does not support heterogenous band numbers. Skipping %s\n",
                             dsFileName);
                    GDALClose(hDS);
                    continue;
                }
                for(j=0;j<nBands;j++)
                {
                    GDALRasterBandH hRasterBand = GDALGetRasterBand( hDS, j+1 );
                    if (bandProperties[j].colorInterpretation != GDALGetRasterColorInterpretation(hRasterBand) ||
                        bandProperties[j].dataType != GDALGetRasterDataType(hRasterBand))
                    {
                        fprintf( stderr, "gdalbuildvrt does not support heterogenous band characteristics. Skipping %s\n",
                             dsFileName);
                        GDALClose(hDS);
                    }
                    if (bandProperties[j].colorTable)
                    {
                        GDALColorTableH colorTable = GDALGetRasterColorTable( hRasterBand );
                        if (colorTable == NULL ||
                            GDALGetColorEntryCount(colorTable) != GDALGetColorEntryCount(bandProperties[j].colorTable))
                        {
                            fprintf( stderr, "gdalbuildvrt does not support heterogenous band characteristics. Skipping %s\n",
                             dsFileName);
                            GDALClose(hDS);
                            break;
                        }
                        /* We should check that the palette are the same too ! */
                    }
                }
                if (j != nBands)
                    continue;
                if (product_minX < minX) minX = product_minX;
                if (product_minY < minY) minY = product_minY;
                if (product_maxX > maxX) maxX = product_maxX;
                if (product_maxY > maxY) maxY = product_maxY;
            }
            if (resolutionStrategy == AVERAGE_RESOLUTION)
            {
                we_res += psDatasetProperties[i].adfGeoTransform[GEOTRSFRM_WE_RES];
                ns_res += psDatasetProperties[i].adfGeoTransform[GEOTRSFRM_NS_RES];
            }
            else
            {
                if (bFirst)
                {
                    we_res = psDatasetProperties[i].adfGeoTransform[GEOTRSFRM_WE_RES];
                    ns_res = psDatasetProperties[i].adfGeoTransform[GEOTRSFRM_NS_RES];
                }
                else if (resolutionStrategy == HIGHEST_RESOLUTION)
                {
                    we_res = MIN(we_res, psDatasetProperties[i].adfGeoTransform[GEOTRSFRM_WE_RES]);
                    ns_res = MIN(ns_res, psDatasetProperties[i].adfGeoTransform[GEOTRSFRM_NS_RES]);
                }
                else
                {
                    we_res = MAX(we_res, psDatasetProperties[i].adfGeoTransform[GEOTRSFRM_WE_RES]);
                    ns_res = MAX(ns_res, psDatasetProperties[i].adfGeoTransform[GEOTRSFRM_NS_RES]);
                }
            }

            psDatasetProperties[i].isFileOK = 1;
            nCount ++;
            bFirst = FALSE;
            GDALClose(hDS);
        }
        else
        {
            fprintf( stderr, "Warning : can't open %s. Skipping it\n", dsFileName);
        }
    }
    
    if (nCount == 0)
        goto end;
    
    if (resolutionStrategy == AVERAGE_RESOLUTION)
    {
        we_res /= nCount;
        ns_res /= nCount;
    }
    
    rasterXSize = (int)(0.5 + (maxX - minX) / we_res);
    rasterYSize = (int)(0.5 + (maxY - minY) / -ns_res);
    
    hVRTDS = VRTCreate(rasterXSize, rasterYSize);
    
    if (projectionRef)
    {
        GDALSetProjection(hVRTDS, projectionRef);
    }

    double adfGeoTransform[6];
    adfGeoTransform[GEOTRSFRM_TOPLEFT_X] = minX;
    adfGeoTransform[GEOTRSFRM_WE_RES] = we_res;
    adfGeoTransform[GEOTRSFRM_ROTATION_PARAM1] = 0;
    adfGeoTransform[GEOTRSFRM_TOPLEFT_Y] = maxY;
    adfGeoTransform[GEOTRSFRM_ROTATION_PARAM2] = 0;
    adfGeoTransform[GEOTRSFRM_NS_RES] = ns_res;
    GDALSetGeoTransform(hVRTDS, adfGeoTransform);
    
    for(j=0;j<nBands;j++)
    {
        GDALRasterBandH hBand;
        GDALAddBand(hVRTDS, bandProperties[j].dataType, NULL);
        hBand = GDALGetRasterBand(hVRTDS, j+1);
        GDALSetRasterColorInterpretation(hBand, bandProperties[j].colorInterpretation);
        if (bandProperties[j].colorInterpretation == GCI_PaletteIndex)
        {
            GDALSetRasterColorTable(hBand, bandProperties[j].colorTable);
        }
        if (bandProperties[j].bHasNoData)
            GDALSetRasterNoDataValue(hBand, bandProperties[j].noDataValue);
    }

    for(i=0;i<nInputFiles;i++)
    {
        if (psDatasetProperties[i].isFileOK == 0)
            continue;
        const char* dsFileName = files[i].c_str();

        bool isProxy = true;

#if ((GDAL_VERSION_MAJOR >= 1) && (GDAL_VERSION_MINOR >= 6))
        //Use a proxy dataset if possible.  This helps with huge amount of files to keep the # of handles down
        GDALProxyPoolDatasetH hDS =
               GDALProxyPoolDatasetCreate(dsFileName,
                                         psDatasetProperties[i].nRasterXSize,
                                         psDatasetProperties[i].nRasterYSize,
                                         GA_ReadOnly, TRUE, projectionRef,
                                         psDatasetProperties[i].adfGeoTransform);

        for(j=0;j<nBands;j++)
        {
            GDALProxyPoolDatasetAddSrcBandDescription(hDS,
                                            bandProperties[j].dataType,
                                            psDatasetProperties[i].nBlockXSize,
                                            psDatasetProperties[i].nBlockYSize);
        }
        isProxy = true;
        osg::notify(osg::INFO) << "Using GDALProxyPoolDatasetH" << std::endl;
#else
        osg::notify(osg::INFO) << "Using GDALDataset, no proxy support enabled" << std::endl;
        //Just open the dataset
        GDALDatasetH hDS = (GDALDatasetH)GDALOpen(dsFileName, GA_ReadOnly);
        isProxy = false;
#endif
        int xoffset = (int)
                (0.5 + (psDatasetProperties[i].adfGeoTransform[GEOTRSFRM_TOPLEFT_X] - minX) / we_res);
        int yoffset = (int)
                (0.5 + (maxY - psDatasetProperties[i].adfGeoTransform[GEOTRSFRM_TOPLEFT_Y]) / -ns_res);
        int dest_width = (int)
                (0.5 + psDatasetProperties[i].nRasterXSize * psDatasetProperties[i].adfGeoTransform[GEOTRSFRM_WE_RES] / we_res);
        int dest_height = (int)
                (0.5 + psDatasetProperties[i].nRasterYSize * psDatasetProperties[i].adfGeoTransform[GEOTRSFRM_NS_RES] / ns_res);

        for(j=0;j<nBands;j++)
        {
            VRTSourcedRasterBandH hVRTBand = (VRTSourcedRasterBandH)GDALGetRasterBand(hVRTDS, j + 1);

            /* Place the raster band at the right position in the VRT */
            VRTAddSimpleSource(hVRTBand, GDALGetRasterBand((GDALDatasetH)hDS, j + 1),
                               0, 0,
                               psDatasetProperties[i].nRasterXSize,
                               psDatasetProperties[i].nRasterYSize,
                               xoffset, yoffset,
                               dest_width, dest_height, "near",
                               VRT_NODATA_UNSET);
        }
        //Only dereference if it is a proxy dataset
        if (isProxy)
        {
          GDALDereferenceDataset(hDS);
        }
    }
end:
    CPLFree(psDatasetProperties);
    for(j=0;j<nBands;j++)
    {
        GDALDestroyColorTable(bandProperties[j].colorTable);
    }
    CPLFree(bandProperties);
    CPLFree(projectionRef);
    return hVRTDS;
}
#endif


class GDALTileSource : public TileSource
{
public:
    GDALTileSource(const osgDB::ReaderWriter::Options* options) :
      TileSource( options ),
      _tile_size(256),
      _srcDS(NULL),
      _warpedDS(NULL)
    {
        static bool s_gdal_registered = false;
        if (!s_gdal_registered)
        {
            GDALAllRegister();
            s_gdal_registered = true;
        }

        std::string interpOption;

        if ( options )
        {
            if ( options->getPluginData( PROPERTY_URL ) )
                _url = std::string( (const char*)options->getPluginData( PROPERTY_URL ) );

            if ( options->getPluginData( PROPERTY_EXTENTSIONS ) )
                _extensions = std::string( (const char*)options->getPluginData( PROPERTY_EXTENTSIONS ) );

            if ( options->getPluginData( PROPERTY_TILE_SIZE ) )
                _tile_size = as<int>( (const char*)options->getPluginData( PROPERTY_TILE_SIZE ), 256 );

            if ( options->getPluginData( PROPERTY_INTERPOLATION ) )
                interpOption = std::string( (const char*)options->getPluginData( PROPERTY_INTERPOLATION ) );
        }

        if (interpOption.empty())
        {
            interpOption = "average";
        }

        if (interpOption == "nearest") _interpolation = NEAREST;
        else if (interpOption == "average") _interpolation = AVERAGE;
        else if (interpOption == "bilinear") _interpolation = BILINEAR;
        else _interpolation = AVERAGE;
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


    const Profile* createProfile( const Profile* mapProfile, const std::string& confPath )
    {   
        if (_url.empty())
        {
            osg::notify(osg::WARN) << "[osgEarth::GDAL] No URL or directory specified " << std::endl;
            return NULL;
        }

        std::string path = _url;

        //Find the full path to the URL
        //If we have a relative path and the map file contains a server address, just concat the server path and the _url together
        if (osgEarth::isRelativePath(path) && osgDB::containsServerAddress(confPath))
        {
            path = osgDB::getFilePath(confPath) + "/" + path;
        }

        //If the path doesn't contain a server address, get the full path to the file.
        if (!osgDB::containsServerAddress(path))
        {
            path = osgEarth::getFullPath(confPath, path);
        }


        std::vector<std::string> exts;
        tokenize(_extensions, exts, ";");
        for (unsigned int i = 0; i < exts.size(); ++i)
        {
            osg::notify(osg::INFO) << "[osgEarth::GDAL] Using Extension: " << exts[i] << std::endl;
        }
        std::vector<std::string> files;
        getFiles(path, exts, files);

        osg::notify(osg::NOTICE) << "[osgEarth::GDAL] GDAL Driver found " << files.size() << " files " << std::endl;
        for (unsigned int i = 0; i < files.size(); ++i)
        {
            osg::notify(osg::NOTICE) <<"  " << files[i] << std::endl;
        }

        //If we found more than one file, try to combine them into a single logical dataset
        if (files.size() > 1)
        {
#if ((GDAL_VERSION_MAJOR >= 1) && (GDAL_VERSION_MINOR >= 5))
            _srcDS = (GDALDataset*)build_vrt(files, HIGHEST_RESOLUTION);
            if (!_srcDS)
            {
                osg::notify(osg::WARN) << "[osgEarth::GDAL] Failed to build VRT from input datasets" << std::endl;
                return NULL;
            }
#else
            osg::notify(osg::NOTICE) << "GDAL Driver support for directories requires GDAL 1.5.0 or better" << std::endl;
#endif
        }

        //If we couldn't build a VRT, just try opening the file directly
        if ( !_srcDS )
        {
            //Open the dataset
            _srcDS = (GDALDataset*)GDALOpen( path.c_str(), GA_ReadOnly );
        }
        if ( !_srcDS )
        {
            osg::notify(osg::WARN) << "[osgEarth::GDAL] Failed to open dataset " << path << std::endl;
            return NULL;
        }

        const Profile* profile = NULL;

        // If the map already defines a profile, simply take it on.
        if ( mapProfile )
        {
            profile = mapProfile;
        }

        //Create a spatial reference for the source.
        osg::ref_ptr<SpatialReference> src_srs = SpatialReference::create( _srcDS->GetProjectionRef() );

        if ( !profile && src_srs->isGeographic() )
        {
            profile = osgEarth::Registry::instance()->getGlobalGeodeticProfile();
        }

        if ( !profile && src_srs->isMercator() )
        {
            profile = osgEarth::Registry::instance()->getGlobalMercatorProfile();
        }

        if ( profile && !profile->getSRS()->isEquivalentTo( src_srs ) )
        {
            _warpedDS = (GDALDataset*)GDALAutoCreateWarpedVRT(
                _srcDS,
                src_srs->getWKT().c_str(),
                profile->getSRS()->getWKT().c_str(),
                GRA_NearestNeighbour,
                5.0,
                NULL);

            //GDALAutoCreateWarpedVRT(srcDS, src_wkt.c_str(), t_srs.c_str(), GRA_NearestNeighbour, 5.0, NULL);
        }
        else
        {
            _warpedDS = _srcDS;
        }

        //Get the _geotransform
        _warpedDS->GetGeoTransform(_geotransform);

        //Compute the extents
        pixelToGeo(0.0, _warpedDS->GetRasterYSize(), _extentsMin.x(), _extentsMin.y());
        pixelToGeo(_warpedDS->GetRasterXSize(), 0.0, _extentsMax.x(), _extentsMax.y());

        if ( !profile )
        {
            profile = Profile::create( 
                _warpedDS->GetProjectionRef(),
                _extentsMin.x(), _extentsMin.y(), _extentsMax.x(), _extentsMax.y() );

            osg::notify(osg::INFO) << "[osgEarth::GDAL] " << _url << " is projected, SRS = " 
                << _warpedDS->GetProjectionRef() << std::endl;
        }

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
            key->getGeoExtent().getBounds(xmin, ymin, xmax, ymax);

            int target_width = _tile_size;
            int target_height = _tile_size;
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

            //osg::notify(osg::NOTICE) << "ReadWindow " << width << "x" << height << " DestWindow " << target_width << "x" << target_height << std::endl;

            //Return if parameters are out of range.
            if (width <= 0 || height <= 0 || target_width <= 0 || target_height <= 0) return 0;



            GDALRasterBand* bandRed = findBand(_warpedDS, GCI_RedBand);
            GDALRasterBand* bandGreen = findBand(_warpedDS, GCI_GreenBand);
            GDALRasterBand* bandBlue = findBand(_warpedDS, GCI_BlueBand);
            GDALRasterBand* bandAlpha = findBand(_warpedDS, GCI_AlphaBand);

            GDALRasterBand* bandGray = findBand(_warpedDS, GCI_GrayIndex);


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
                image->allocateImage(_tile_size, _tile_size, 1, GL_RGBA, GL_UNSIGNED_BYTE);
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
            else if (bandGray)
            {
                unsigned char *gray = new unsigned char[target_width * target_height];
                unsigned char *alpha = new unsigned char[target_width * target_height];

                //Initialize the alpha values to 255.
                memset(alpha, 255, target_width * target_height);

                bandGray->RasterIO(GF_Read, off_x, off_y, width, height, gray, target_width, target_height, GDT_Byte, 0, 0);

                if (bandAlpha)
                {
                    bandBlue->RasterIO(GF_Read, off_x, off_y, width, height, alpha, target_width, target_height, GDT_Byte, 0, 0);
                }

                image = new osg::Image;
                image->allocateImage(_tile_size, _tile_size, 1, GL_RGBA, GL_UNSIGNED_BYTE);
                memset(image->data(), 0, image->getImageSizeInBytes());

                for (int src_row = 0, dst_row = tile_offset_top;
                    src_row < target_height;
                    src_row++, dst_row++)
                {
                    for (int src_col = 0, dst_col = tile_offset_left;
                        src_col < target_width;
                        ++src_col, ++dst_col)
                    {
                        *(image->data(dst_col, dst_row) + 0) = gray[src_col + src_row * target_width];
                        *(image->data(dst_col, dst_row) + 1) = gray[src_col + src_row * target_width];
                        *(image->data(dst_col, dst_row) + 2) = gray[src_col + src_row * target_width];
                        *(image->data(dst_col, dst_row) + 3) = alpha[src_col + src_row * target_width];
                    }
                }

                image->flipVertical();

                delete []gray;
                delete []alpha;

            }
            else
            {
                osg::notify(osg::NOTICE) << "Could not find red, green and blue bands or gray bands in " << _url << ".  Cannot create image. " << std::endl;
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
        double offsetTransform[6];
        memcpy(offsetTransform, _geotransform, 6 * sizeof(double));
        //Offset the _geotransform by half a pixel
        offsetTransform[0] += 0.5 * _geotransform[1];
        offsetTransform[3] += 0.5 * _geotransform[5];

        double invTransform[6];
        GDALInvGeoTransform(offsetTransform, invTransform);
        double r, c;
        GDALApplyGeoTransform(invTransform, x, y, &c, &r);

        float result = 0.0f;

        //If the location is outside of the pixel values of the dataset, just return 0
        if (c < 0 || r < 0 || c > _warpedDS->GetRasterXSize()-1 || r > _warpedDS->GetRasterYSize()-1) return NO_DATA_VALUE;

        if (_interpolation == NEAREST)
        {
            band->RasterIO(GF_Read, (int)c, (int)r, 1, 1, &result, 1, 1, GDT_Float32, 0, 0);
        }
        else
        {
            int rowMin = osg::maximum((int)floor(r), 0);
            int rowMax = osg::maximum(osg::minimum((int)ceil(r), (int)(_warpedDS->GetRasterYSize()-1)), 0);
            int colMin = osg::maximum((int)floor(c), 0);
            int colMax = osg::maximum(osg::minimum((int)ceil(c), (int)(_warpedDS->GetRasterXSize()-1)), 0);

            if (rowMin > rowMax) rowMin = rowMax;
            if (colMin > colMax) colMin = colMax;

            float urHeight, llHeight, ulHeight, lrHeight;

            band->RasterIO(GF_Read, colMin, rowMin, 1, 1, &llHeight, 1, 1, GDT_Float32, 0, 0);
            band->RasterIO(GF_Read, colMin, rowMax, 1, 1, &ulHeight, 1, 1, GDT_Float32, 0, 0);
            band->RasterIO(GF_Read, colMax, rowMin, 1, 1, &lrHeight, 1, 1, GDT_Float32, 0, 0);
            band->RasterIO(GF_Read, colMax, rowMax, 1, 1, &urHeight, 1, 1, GDT_Float32, 0, 0);

            /*
            if (!isValidValue(urHeight, band)) urHeight = 0.0f;
            if (!isValidValue(llHeight, band)) llHeight = 0.0f;
            if (!isValidValue(ulHeight, band)) ulHeight = 0.0f;
            if (!isValidValue(lrHeight, band)) lrHeight = 0.0f;
            */
            if (!isValidValue(urHeight, band) || (!isValidValue(llHeight, band)) ||(!isValidValue(ulHeight, band)) || (!isValidValue(lrHeight, band)))
            {
                return NO_DATA_VALUE;
            }






            if (_interpolation == AVERAGE)
            {
                double x_rem = c - (int)c;
                double y_rem = r - (int)r;

                double w00 = (1.0 - y_rem) * (1.0 - x_rem) * (double)llHeight;
                double w01 = (1.0 - y_rem) * x_rem * (double)lrHeight;
                double w10 = y_rem * (1.0 - x_rem) * (double)ulHeight;
                double w11 = y_rem * x_rem * (double)urHeight;

                result = (float)(w00 + w01 + w10 + w11);
            }
            else if (_interpolation == BILINEAR)
            {
                //Check for exact value
                if ((colMax == colMin) && (rowMax == rowMin))
                {
                    //osg::notify(osg::NOTICE) << "Exact value" << std::endl;
                    result = llHeight;
                }
                else if (colMax == colMin)
                {
                    //osg::notify(osg::NOTICE) << "Vertically" << std::endl;
                    //Linear interpolate vertically
                    result = ((float)rowMax - r) * llHeight + (r - (float)rowMin) * ulHeight;
                }
                else if (rowMax == rowMin)
                {
                    //osg::notify(osg::NOTICE) << "Horizontally" << std::endl;
                    //Linear interpolate horizontally
                    result = ((float)colMax - c) * llHeight + (c - (float)colMin) * lrHeight;
                }
                else
                {
                    //osg::notify(osg::NOTICE) << "Bilinear" << std::endl;
                    //Bilinear interpolate
                    float r1 = ((float)colMax - c) * llHeight + (c - (float)colMin) * lrHeight;
                    float r2 = ((float)colMax - c) * ulHeight + (c - (float)colMin) * urHeight;

                    //osg::notify(osg::INFO) << "r1, r2 = " << r1 << " , " << r2 << std::endl;
                    result = ((float)rowMax - r) * r1 + (r - (float)rowMin) * r2;
                }
            }
        }

        return result;
    }


    osg::HeightField* createHeightField( const TileKey* key )
    {
        OpenThreads::ScopedLock<OpenThreads::ReentrantMutex> lock(s_mutex);

        //Allocate the heightfield
        osg::ref_ptr<osg::HeightField> hf = new osg::HeightField;
        hf->allocate(_tile_size, _tile_size);

        if (intersects(key))
        {
            //Get the meter extents of the tile
            double xmin, ymin, xmax, ymax;
            key->getGeoExtent().getBounds(xmin, ymin, xmax, ymax);

            //Just read from the first band
            GDALRasterBand* band = _warpedDS->GetRasterBand(1);

            double dx = (xmax - xmin) / (_tile_size-1);
            double dy = (ymax - ymin) / (_tile_size-1);

            for (int c = 0; c < _tile_size; ++c)
            {
                double geoX = xmin + (dx * (double)c);
                for (int r = 0; r < _tile_size; ++r)
                {
                    double geoY = ymin + (dy * (double)r);
                    float h = getInterpolatedValue(band, geoX, geoY);
                    hf->setHeight(c, r, h);
                }
            }
        }
        else
        {
            for (unsigned int i = 0; i < hf->getHeightList().size(); ++i) hf->getHeightList()[i] = NO_DATA_VALUE;
        }
        return hf.release();
    }

    virtual int getPixelsPerTile() const
    {
        return _tile_size;
    }

    bool intersects(const TileKey* key)
    {
        //Get the native extents of the tile
        double xmin, ymin, xmax, ymax;
        key->getGeoExtent().getBounds(xmin, ymin, xmax, ymax);

        return ! ( xmin >= _extentsMax.x() || xmax <= _extentsMin.x() || ymin >= _extentsMax.y() || ymax <= _extentsMin.y() );        
    }


private:

    GDALDataset* _srcDS;
    GDALDataset* _warpedDS;
    double       _geotransform[6];

    osg::Vec2d _extentsMin;
    osg::Vec2d _extentsMax;

    std::string     _url;
    int             _tile_size;
    std::string     _extensions;
    ElevationInterpolation   _interpolation;
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
