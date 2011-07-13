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

#include <osgEarth/TileSource>
#include <osgEarth/FileUtils>
#include <osgEarth/Registry>
#include <osgEarth/ImageUtils>

#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgDB/ImageOptions>

#include <sstream>
#include <stdlib.h>
#include <memory.h>

#include <gdal_priv.h>
#include <gdalwarper.h>
#include <ogr_spatialref.h>

#include "GDALOptions"

#define LC "[GDAL driver] "

// From easyrgb.com
float Hue_2_RGB( float v1, float v2, float vH )
{
   if ( vH < 0 ) vH += 1;
   if ( vH > 1 ) vH -= 1;
   if ( ( 6 * vH ) < 1 ) return ( v1 + ( v2 - v1 ) * 6 * vH );
   if ( ( 2 * vH ) < 1 ) return ( v2 );
   if ( ( 3 * vH ) < 2 ) return ( v1 + ( v2 - v1 ) * ( ( 2 / 3 ) - vH ) * 6 );
   return ( v1 );
}

#if (GDAL_VERSION_MAJOR > 1 || (GDAL_VERSION_MAJOR >= 1 && GDAL_VERSION_MINOR >= 5))
#  define GDAL_VERSION_1_5_OR_NEWER 1
#endif

#if (GDAL_VERSION_MAJOR > 1 || (GDAL_VERSION_MAJOR >= 1 && GDAL_VERSION_MINOR >= 6))
#  define GDAL_VERSION_1_6_OR_NEWER 1
#endif

#ifndef GDAL_VERSION_1_5_OR_NEWER
#  error "**** GDAL 1.5 or newer required ****"
#endif

//GDAL proxy is only available after GDAL 1.6
#if GDAL_VERSION_1_6_OR_NEWER
#  include <gdal_proxy.h>
#endif

#include <cpl_string.h>

//GDAL VRT api is only available after 1.5.0
#include <gdal_vrt.h>

using namespace std;
using namespace osgEarth;
using namespace osgEarth::Drivers;

//#define PROPERTY_URL            "url"
//#define PROPERTY_TILE_SIZE      "tile_size"
//#define PROPERTY_EXTENTSIONS    "extensions"
//#define PROPERTY_INTERPOLATION  "interpolation"
//#define PROPERTY_DEFAULT_TILE_SIZE "default_tile_size"

//static OpenThreads::ReentrantMutex s_mutex;


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

// "build_vrt()" is adapted from the gdalbuildvrt application. Following is
// the copyright notice from the source. The original code can be found at
// http://trac.osgeo.org/gdal/browser/trunk/gdal/apps/gdalbuildvrt.cpp

/******************************************************************************
 * Copyright (c) 2007, Even Rouault
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 ****************************************************************************/
static GDALDatasetH
build_vrt(std::vector<std::string> &files, ResolutionStrategy resolutionStrategy)
{
    GDAL_SCOPED_LOCK;

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
            if (!proj || strlen(proj) == 0)
            {                
                std::string prjLocation = osgDB::getNameLessExtension( std::string(dsFileName) ) + std::string(".prj");
                std::string wkt;
                if ( HTTPClient::readString( prjLocation, wkt ) == HTTPClient::RESULT_OK )
                {                    
                    proj = CPLStrdup( wkt.c_str() );
                }                
            }

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
                    /* Yes : as ns_res is negative, the highest resolution is the max value */
                    ns_res = MAX(ns_res, psDatasetProperties[i].adfGeoTransform[GEOTRSFRM_NS_RES]);
                }
                else
                {
                    we_res = MAX(we_res, psDatasetProperties[i].adfGeoTransform[GEOTRSFRM_WE_RES]);
                    /* Yes : as ns_res is negative, the lowest resolution is the min value */
                    ns_res = MIN(ns_res, psDatasetProperties[i].adfGeoTransform[GEOTRSFRM_NS_RES]);
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
        //OE_NOTICE << "Setting projection to " << projectionRef << std::endl;
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

#if GDAL_VERSION_1_6_OR_NEWER

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
        OE_DEBUG << LC << "Using GDALProxyPoolDatasetH" << std::endl;

#else // !GDAL_VERSION_1_6_OR_NEWER

        OE_DEBUG << LC << "Using GDALDataset, no proxy support enabled" << std::endl;
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


// This is simply the method GDALAutoCreateWarpedVRT() with the GDALSuggestedWarpOutput
// logic replaced with something that will work properly for polar projections.
// see: http://www.mail-archive.com/gdal-dev@lists.osgeo.org/msg01491.html
static
GDALDatasetH GDALAutoCreateWarpedVRTforPolarStereographic(
    GDALDatasetH hSrcDS,
    const char *pszSrcWKT,
    const char *pszDstWKT,
    GDALResampleAlg eResampleAlg,
    double dfMaxError,
    const GDALWarpOptions *psOptionsIn )
{
    GDALWarpOptions *psWO;
    int i;

    VALIDATE_POINTER1( hSrcDS, "GDALAutoCreateWarpedVRTForPolarStereographic", NULL );

    /* -------------------------------------------------------------------- */
    /*      Populate the warp options.                                      */
    /* -------------------------------------------------------------------- */
    if( psOptionsIn != NULL )
        psWO = GDALCloneWarpOptions( psOptionsIn );
    else
        psWO = GDALCreateWarpOptions();

    psWO->eResampleAlg = eResampleAlg;

    psWO->hSrcDS = hSrcDS;

    psWO->nBandCount = GDALGetRasterCount( hSrcDS );
    psWO->panSrcBands = (int *) CPLMalloc(sizeof(int) * psWO->nBandCount);
    psWO->panDstBands = (int *) CPLMalloc(sizeof(int) * psWO->nBandCount);

    for( i = 0; i < psWO->nBandCount; i++ )
    {
        psWO->panSrcBands[i] = i+1;
        psWO->panDstBands[i] = i+1;
    }

    /* TODO: should fill in no data where available */

    /* -------------------------------------------------------------------- */
    /*      Create the transformer.                                         */
    /* -------------------------------------------------------------------- */
    psWO->pfnTransformer = GDALGenImgProjTransform;
    psWO->pTransformerArg =
        GDALCreateGenImgProjTransformer( psWO->hSrcDS, pszSrcWKT,
        NULL, pszDstWKT,
        TRUE, 1.0, 0 );

    if( psWO->pTransformerArg == NULL )
    {
        GDALDestroyWarpOptions( psWO );
        return NULL;
    }

    /* -------------------------------------------------------------------- */
    /*      Figure out the desired output bounds and resolution.            */
    /* -------------------------------------------------------------------- */
    double adfDstGeoTransform[6];
    int    nDstPixels, nDstLines;
    CPLErr eErr;

    eErr =
        GDALSuggestedWarpOutput( hSrcDS, psWO->pfnTransformer,
            psWO->pTransformerArg,
            adfDstGeoTransform, &nDstPixels, &nDstLines );

    // override the suggestions:
    nDstPixels = GDALGetRasterXSize( hSrcDS ) * 4;
    nDstLines  = GDALGetRasterYSize( hSrcDS ) / 2;
    adfDstGeoTransform[0] = -180.0;
    adfDstGeoTransform[1] = 360.0/(double)nDstPixels;
    //adfDstGeoTransform[2] = 0.0;
    //adfDstGeoTransform[4] = 0.0;
    //adfDstGeoTransform[5] = (-90 -adfDstGeoTransform[3])/(double)nDstLines;

    /* -------------------------------------------------------------------- */
    /*      Update the transformer to include an output geotransform        */
    /*      back to pixel/line coordinates.                                 */
    /*                                                                      */
    /* -------------------------------------------------------------------- */
    GDALSetGenImgProjTransformerDstGeoTransform(
        psWO->pTransformerArg, adfDstGeoTransform );

    /* -------------------------------------------------------------------- */
    /*      Do we want to apply an approximating transformation?            */
    /* -------------------------------------------------------------------- */
    if( dfMaxError > 0.0 )
    {
        psWO->pTransformerArg =
            GDALCreateApproxTransformer( psWO->pfnTransformer,
            psWO->pTransformerArg,
            dfMaxError );
        psWO->pfnTransformer = GDALApproxTransform;
    }

    /* -------------------------------------------------------------------- */
    /*      Create the VRT file.                                            */
    /* -------------------------------------------------------------------- */
    GDALDatasetH hDstDS;

    hDstDS = GDALCreateWarpedVRT( hSrcDS, nDstPixels, nDstLines,
        adfDstGeoTransform, psWO );

    GDALDestroyWarpOptions( psWO );

    if( pszDstWKT != NULL )
        GDALSetProjection( hDstDS, pszDstWKT );
    else if( pszSrcWKT != NULL )
        GDALSetProjection( hDstDS, pszDstWKT );
    else if( GDALGetGCPCount( hSrcDS ) > 0 )
        GDALSetProjection( hDstDS, GDALGetGCPProjection( hSrcDS ) );
    else
        GDALSetProjection( hDstDS, GDALGetProjectionRef( hSrcDS ) );

    return hDstDS;
}


class GDALTileSource : public TileSource
{
public:
    GDALTileSource( const TileSourceOptions& options ) :
      TileSource( options ),
      _srcDS(NULL),
      _warpedDS(NULL),
      _options(options),
      _maxDataLevel(30)
    {    
    }

    virtual ~GDALTileSource()
    {             
        GDAL_SCOPED_LOCK;

        if (_warpedDS != _srcDS)
        {
            GDALClose( _warpedDS );
        }

        //Close the datasets if it exists
        if (_srcDS)
        {     
            GDALClose(_srcDS);
        }
    }


    void initialize( const std::string& referenceURI, const Profile* overrideProfile)
    {   
        GDAL_SCOPED_LOCK;

        if ( !_options.url().isSet() || _options.url()->empty() )
        {
            OE_WARN << LC << "No URL or directory specified " << std::endl;
            return;
        }

        std::string path = _options.url().value();

        //Find the full path to the URL
        //If we have a relative path and the map file contains a server address, just concat the server path and the _url together
        if (osgEarth::isRelativePath(path) && osgDB::containsServerAddress(referenceURI))
        {
            path = osgDB::getFilePath(referenceURI) + std::string("/") + path;
        }

        //If the path doesn't contain a server address, get the full path to the file.
        if (!osgDB::containsServerAddress(path))
        {
            path = osgEarth::getFullPath(referenceURI, path);
        }

        StringTokenizer izer( ";" );
        StringVector exts;
        izer.tokenize( *_options.extensions(), exts );

        //std::vector<std::string> exts;

        //tokenize( _options.extensions().value(), exts, ";");
        for (unsigned int i = 0; i < exts.size(); ++i)
        {
            OE_DEBUG << LC << "Using Extension: " << exts[i] << std::endl;
        }
        std::vector<std::string> files;
        getFiles(path, exts, files);

        OE_INFO << LC << "Driver found " << files.size() << " files:" << std::endl;
        for (unsigned int i = 0; i < files.size(); ++i)
        {
            OE_INFO << LC << "" << files[i] << std::endl;
        }

        if (files.empty())
        {
            OE_WARN << LC << "Could not find any valid files " << std::endl;
            return;
        }

        //If we found more than one file, try to combine them into a single logical dataset
        if (files.size() > 1)
        {
            _srcDS = (GDALDataset*)build_vrt(files, HIGHEST_RESOLUTION);
            if (!_srcDS)
            {
                OE_WARN << "[osgEarth::GDAL] Failed to build VRT from input datasets" << std::endl;
                return;
            }
        }
        else
        {
            //If we couldn't build a VRT, just try opening the file directly
            //Open the dataset
            _srcDS = (GDALDataset*)GDALOpen( files[0].c_str(), GA_ReadOnly );
            if ( !_srcDS )
            {
                OE_WARN << LC << "Failed to open dataset " << files[0] << std::endl;
                return;
            }
        }

        //Create a spatial reference for the source.
        const char* srcProj = _srcDS->GetProjectionRef();
        if ( srcProj != 0L && overrideProfile != 0L )
        {
            OE_WARN << LC << "WARNING, overriding profile of a source that already defines its own SRS (" 
                << this->getName() << ")" << std::endl;
        }

        osg::ref_ptr<const SpatialReference> src_srs;
        if ( overrideProfile )
        {
            src_srs = overrideProfile->getSRS();
        }
        else if ( srcProj )
        {
            src_srs = SpatialReference::create( srcProj );
        }
        
        // assert SRS is present
        if ( !src_srs.valid() )
        {
            // not found in the dataset; try loading a .prj file
            std::string prjLocation = osgDB::getNameLessExtension( path ) + std::string(".prj");
            std::string wkt;
            if ( HTTPClient::readString( prjLocation, wkt ) == HTTPClient::RESULT_OK )
            {
                src_srs = SpatialReference::create( wkt );
            }

            if ( !src_srs.valid() )
            {
                OE_WARN << LC << "Dataset has no spatial reference information: " << path << std::endl;
                return;
            }
        }

        const Profile* profile = NULL;

        if ( overrideProfile )
        {
            profile = overrideProfile;
        }

        if ( !profile && src_srs->isGeographic() )
        {
            profile = osgEarth::Registry::instance()->getGlobalGeodeticProfile();
        }

        //Note:  Can cause odd rendering artifacts if we have a dataset that is mercator that doesn't encompass the whole globe
        //       if we take on the global profile.
        /*
        if ( !profile && src_srs->isMercator() )
        {
            profile = osgEarth::Registry::instance()->getGlobalMercatorProfile();
        }*/

        std::string warpedSRSWKT;

        if ( profile && !profile->getSRS()->isEquivalentTo( src_srs.get() ) )
        {
            if ( profile->getSRS()->isGeographic() && (src_srs->isNorthPolar() || src_srs->isSouthPolar()) )
            {
                _warpedDS = (GDALDataset*)GDALAutoCreateWarpedVRTforPolarStereographic(
                    _srcDS,
                    src_srs->getWKT().c_str(),
                    profile->getSRS()->getWKT().c_str(),
                    GRA_NearestNeighbour,
                    5.0,
                    NULL);
            }
            else
            {
                _warpedDS = (GDALDataset*)GDALAutoCreateWarpedVRT(
                    _srcDS,
                    src_srs->getWKT().c_str(),
                    profile->getSRS()->getWKT().c_str(),
                    GRA_NearestNeighbour,
                    5.0,
                    NULL);
            }

            if ( _warpedDS )
            {
                warpedSRSWKT = _warpedDS->GetProjectionRef();
            }

            //GDALAutoCreateWarpedVRT(srcDS, src_wkt.c_str(), t_srs.c_str(), GRA_NearestNeighbour, 5.0, NULL);
        }
        else
        {
            _warpedDS = _srcDS;            
            warpedSRSWKT = src_srs->getWKT();
        }

        //Get the _geotransform
        if (overrideProfile)
        {        
            _geotransform[0] = overrideProfile->getExtent().xMin(); //Top left x
            _geotransform[1] = overrideProfile->getExtent().width() / (double)_warpedDS->GetRasterXSize();//pixel width
            _geotransform[2] = 0;

            _geotransform[3] = overrideProfile->getExtent().yMax(); //Top left y
            _geotransform[4] = 0;
            _geotransform[5] = -overrideProfile->getExtent().height() / (double)_warpedDS->GetRasterYSize();//pixel height

        }
        else
        {
            _warpedDS->GetGeoTransform(_geotransform);
        }

        GDALInvGeoTransform(_geotransform, _invtransform);


        //Compute the extents
        // polar needs a special case when combined with geographic
        if ( profile && profile->getSRS()->isGeographic() && (src_srs->isNorthPolar() || src_srs->isSouthPolar()) )
        {
            double ll_lon, ll_lat, ul_lon, ul_lat, ur_lon, ur_lat, lr_lon, lr_lat;

            pixelToGeo(0.0, 0.0, ul_lon, ul_lat );
            pixelToGeo(0.0, _warpedDS->GetRasterYSize(), ll_lon, ll_lat);
            pixelToGeo(_warpedDS->GetRasterXSize(), _warpedDS->GetRasterYSize(), lr_lon, lr_lat);
            pixelToGeo(_warpedDS->GetRasterXSize(), 0.0, ur_lon, ur_lat);

            _extentsMin.x() = osg::minimum( ll_lon, osg::minimum( ul_lon, osg::minimum( ur_lon, lr_lon ) ) );
            _extentsMax.x() = osg::maximum( ll_lon, osg::maximum( ul_lon, osg::maximum( ur_lon, lr_lon ) ) );
            
            if ( src_srs->isNorthPolar() )
            {
                _extentsMin.y() = osg::minimum( ll_lat, osg::minimum( ul_lat, osg::minimum( ur_lat, lr_lat ) ) );
                _extentsMax.y() = 90.0;
            }
            else
            {
                _extentsMin.y() = -90.0;
                _extentsMax.y() = osg::maximum( ll_lat, osg::maximum( ul_lat, osg::maximum( ur_lat, lr_lat ) ) );
            }
        }
        else
        {
            pixelToGeo(0.0, _warpedDS->GetRasterYSize(), _extentsMin.x(), _extentsMin.y());
            pixelToGeo(_warpedDS->GetRasterXSize(), 0.0, _extentsMax.x(), _extentsMax.y());
        }

        OE_INFO << LC << "Geo extents: " << _extentsMin.x() << ", " << _extentsMin.y() << " => " << _extentsMax.x() << ", " << _extentsMax.y() << std::endl;

        if ( !profile )
        {
            profile = Profile::create( 
                warpedSRSWKT,
                //_warpedDS->GetProjectionRef(),
                _extentsMin.x(), _extentsMin.y(), _extentsMax.x(), _extentsMax.y() );

            OE_INFO << LC << "" << path << " is projected, SRS = " 
                << warpedSRSWKT << std::endl;
                //<< _warpedDS->GetProjectionRef() << std::endl;
        }

        //Compute the min and max data levels
        double resolutionX = (_extentsMax.x() - _extentsMin.x()) / (double)_warpedDS->GetRasterXSize();
        double resolutionY = (_extentsMax.y() - _extentsMin.y()) / (double)_warpedDS->GetRasterYSize();

		double maxResolution = osg::minimum(resolutionX, resolutionY);

        OE_INFO << LC << "Resolution= " << resolutionX << "x" << resolutionY << " max=" << maxResolution << std::endl;

        if (_options.maxDataLevel().isSet())
        {
            _maxDataLevel = _options.maxDataLevel().value();
            OE_INFO << "Using override max data level " << _maxDataLevel << std::endl;
        }
        else
        {
            unsigned int max_level = 30;
            for (unsigned int i = 0; i < max_level; ++i)
            {
                _maxDataLevel = i;
                double w, h;
                profile->getTileDimensions(i, w, h);
                double resX = (w / (double)_options.tileSize().value() );
                double resY = (h / (double)_options.tileSize().value() );

                if (resX < maxResolution || resY < maxResolution)
                {
                    break;
                }
            }

            OE_INFO << LC << "Max Data Level: " << _maxDataLevel << std::endl;
        }

        // record the data extent in profile space:
        GeoExtent local_extent(
            SpatialReference::create( warpedSRSWKT ), //_warpedDS->GetProjectionRef() ),
            _extentsMin.x(), _extentsMin.y(), _extentsMax.x(), _extentsMax.y() );
        GeoExtent profile_extent = local_extent.transform( profile->getSRS() );

        getDataExtents().push_back( DataExtent(profile_extent, 0, _maxDataLevel) );
        
        OE_INFO << LC << "Data Extents: " << profile_extent.toString() << std::endl;

		//Set the profile
		setProfile( profile );
    }


    /**
    * Finds a raster band based on color interpretation 
    */
    static GDALRasterBand* findBand(GDALDataset *ds, GDALColorInterp colorInterp)
    {
        GDAL_SCOPED_LOCK;

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

    osg::Image* createImage( const TileKey& key,
                             ProgressCallback* progress)
    {
        if (key.getLevelOfDetail() > _maxDataLevel)
        {
            OE_DEBUG << LC << "" << getName() << ": Reached maximum data resolution key=" 
                << key.getLevelOfDetail() << " max=" << _maxDataLevel <<  std::endl;
            return NULL;
        }

        GDAL_SCOPED_LOCK;

        int tileSize = _options.tileSize().value();

        osg::ref_ptr<osg::Image> image;
        if (intersects(key)) //TODO: I think this test is OBE -gw
        {
            //Get the extents of the tile
            double xmin, ymin, xmax, ymax;
            key.getExtent().getBounds(xmin, ymin, xmax, ymax);

            int target_width = tileSize;
            int target_height = tileSize;
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
                target_height = target_height - (int)osg::round(float(oversize_bottom) / height * target_height);
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

            OE_DEBUG << LC << "ReadWindow " << width << "x" << height << " DestWindow " << target_width << "x" << target_height << std::endl;

            //Return if parameters are out of range.
            if (width <= 0 || height <= 0 || target_width <= 0 || target_height <= 0)
            {
                return 0;
            }



            GDALRasterBand* bandRed = findBand(_warpedDS, GCI_RedBand);
            GDALRasterBand* bandGreen = findBand(_warpedDS, GCI_GreenBand);
            GDALRasterBand* bandBlue = findBand(_warpedDS, GCI_BlueBand);
            GDALRasterBand* bandAlpha = findBand(_warpedDS, GCI_AlphaBand);

            GDALRasterBand* bandGray = findBand(_warpedDS, GCI_GrayIndex);

			GDALRasterBand* bandPalette = findBand(_warpedDS, GCI_PaletteIndex);

            //The pixel format is always RGBA to support transparency
            GLenum pixelFormat = GL_RGBA;


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
                    bandAlpha->RasterIO(GF_Read, off_x, off_y, width, height, alpha, target_width, target_height, GDT_Byte, 0, 0);
                }

                image = new osg::Image;
                image->allocateImage(tileSize, tileSize, 1, pixelFormat, GL_UNSIGNED_BYTE);
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
                    bandAlpha->RasterIO(GF_Read, off_x, off_y, width, height, alpha, target_width, target_height, GDT_Byte, 0, 0);
                }

                image = new osg::Image;
                image->allocateImage(tileSize, tileSize, 1, pixelFormat, GL_UNSIGNED_BYTE);
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
			else if (bandPalette)
			{
				unsigned char *palette = new unsigned char[target_width * target_height];

				bandPalette->RasterIO(GF_Read, off_x, off_y, width, height, palette, target_width, target_height, GDT_Byte, 0, 0);

				image = new osg::Image;
				image->allocateImage(tileSize, tileSize, 1, pixelFormat, GL_UNSIGNED_BYTE);
				memset(image->data(), 0, image->getImageSizeInBytes());

				for (int src_row = 0, dst_row = tile_offset_top;
					src_row < target_height;
					src_row++, dst_row++)
				{
					for (int src_col = 0, dst_col = tile_offset_left;
						src_col < target_width;
						++src_col, ++dst_col)
					{
						unsigned char r,g,b,a;
						const GDALColorEntry *colorEntry = bandPalette->GetColorTable()->GetColorEntry(palette[src_col + src_row * target_width]);
						GDALPaletteInterp interp = bandPalette->GetColorTable()->GetPaletteInterpretation();
						if (!colorEntry)
						{
							//FIXME: What to do here?

							//OE_INFO << "NO COLOR ENTRY FOR COLOR " << rawImageData[i] << std::endl;
							r = 255;
							g = 0;
							b = 0;
							a = 1;

						}
						else
						{
							if (interp == GPI_RGB)
							{
								r = colorEntry->c1;
								g = colorEntry->c2;
								b = colorEntry->c3;
								a = colorEntry->c4;
							}
							else if (interp == GPI_CMYK)
							{
								// from wikipedia.org
								short C = colorEntry->c1;
								short M = colorEntry->c2;
								short Y = colorEntry->c3;
								short K = colorEntry->c4;
								r = 255 - C*(255 - K) - K;
								g = 255 - M*(255 - K) - K;
								b = 255 - Y*(255 - K) - K;
								a = 255;
							}
							else if (interp == GPI_HLS)
							{
								// from easyrgb.com
								float H = colorEntry->c1;
								float S = colorEntry->c3;
								float L = colorEntry->c2;
								float R, G, B;
								if ( S == 0 )                       //HSL values = 0 - 1
								{
									R = L;                      //RGB results = 0 - 1 
									G = L;
									B = L;
								}
								else
								{
									float var_2, var_1;
									if ( L < 0.5 )
										var_2 = L * ( 1 + S );
									else
										var_2 = ( L + S ) - ( S * L );

									var_1 = 2 * L - var_2;

									R = Hue_2_RGB( var_1, var_2, H + ( 1 / 3 ) );
									G = Hue_2_RGB( var_1, var_2, H );
									B = Hue_2_RGB( var_1, var_2, H - ( 1 / 3 ) );                                
								} 
								r = static_cast<unsigned char>(R*255.0f);
								g = static_cast<unsigned char>(G*255.0f);
								b = static_cast<unsigned char>(B*255.0f);
								a = static_cast<unsigned char>(255.0f);
							}
							else if (interp == GPI_Gray)
							{
								r = static_cast<unsigned char>(colorEntry->c1*255.0f);
								g = static_cast<unsigned char>(colorEntry->c1*255.0f);
								b = static_cast<unsigned char>(colorEntry->c1*255.0f);
								a = static_cast<unsigned char>(255.0f);
							}

							*(image->data(dst_col, dst_row) + 0) = r;
							*(image->data(dst_col, dst_row) + 1) = g;
							*(image->data(dst_col, dst_row) + 2) = b;
							*(image->data(dst_col, dst_row) + 3) = a;
						}
					}
				}

				image->flipVertical();

				delete [] palette;
			}
            else
            {
                OE_WARN 
                    << LC << "Could not find red, green and blue bands or gray bands in "
                    << _options.url().value()
                    << ".  Cannot create image. " << std::endl;

                return NULL;
            }
        }

        // Moved this logic up into ImageLayer::createImageWrapper.
        ////Create a transparent image if we don't have an image
        //if (!image.valid())
        //{
        //    //OE_WARN << LC << "Illegal state-- should not get here" << std::endl;
        //    return ImageUtils::createEmptyImage();
        //}
        return image.release();
    }

    bool isValidValue(float v, GDALRasterBand* band)
    {
        GDAL_SCOPED_LOCK;

        float bandNoData = -32767.0f;
        int success;
        float value = band->GetNoDataValue(&success);
        if (success)
        {
            bandNoData = value;
        }

        //Check to see if the value is equal to the bands specified no data
        if (bandNoData == v) return false;
        //Check to see if the value is equal to the user specified nodata value
        if (getNoDataValue() == v) return false;        

        //Check to see if the user specified a custom min/max
        if (v < getNoDataMinValue()) return false;
        if (v > getNoDataMaxValue()) return false;

        //Check within a sensible range
        if (v < -32000) return false;
        if (v > 32000) return false;

        return true;
    }


    float getInterpolatedValue(GDALRasterBand *band, double x, double y)
    {
        double r, c;
        GDALApplyGeoTransform(_invtransform, x, y, &c, &r);

        //Account for slight rounding errors.  If we are right on the edge of the dataset, clamp to the edge
        double eps = 0.0001;
        if (osg::equivalent(c, 0, eps)) c = 0;
        if (osg::equivalent(r, 0, eps)) r = 0;
        if (osg::equivalent(c, (double)_warpedDS->GetRasterXSize(), eps)) c = _warpedDS->GetRasterXSize();
        if (osg::equivalent(r, (double)_warpedDS->GetRasterYSize(), eps)) r = _warpedDS->GetRasterYSize();

        //Apply half pixel offset
        r-= 0.5;
        c-= 0.5;

        //Account for the half pixel offset in the geotransform.  If the pixel value is -0.5 we are still technically in the dataset
        //since 0,0 is now the center of the pixel.  So, if are within a half pixel above or a half pixel below the dataset just use
        //the edge values
        if (c < 0 && c >= -0.5)
        {
            c = 0;
        }
        else if (c > _warpedDS->GetRasterXSize()-1 && c <= _warpedDS->GetRasterXSize()-0.5)
        {
            c = _warpedDS->GetRasterXSize()-1;
        }

        if (r < 0 && r >= -0.5)
        {
            r = 0;
        }
        else if (r > _warpedDS->GetRasterYSize()-1 && r <= _warpedDS->GetRasterYSize()-0.5)
        {
            r = _warpedDS->GetRasterYSize()-1;
        }

        float result = 0.0f;

        //If the location is outside of the pixel values of the dataset, just return 0
        if (c < 0 || r < 0 || c > _warpedDS->GetRasterXSize()-1 || r > _warpedDS->GetRasterYSize()-1)
            return NO_DATA_VALUE;

        if ( _options.interpolation() == INTERP_NEAREST )
        {
            band->RasterIO(GF_Read, (int)osg::round(c), (int)osg::round(r), 1, 1, &result, 1, 1, GDT_Float32, 0, 0);
            if (!isValidValue( result, band))
            {
                return NO_DATA_VALUE;
            }
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

            if ( _options.interpolation() == INTERP_AVERAGE )
            {
                double x_rem = c - (int)c;
                double y_rem = r - (int)r;

                double w00 = (1.0 - y_rem) * (1.0 - x_rem) * (double)llHeight;
                double w01 = (1.0 - y_rem) * x_rem * (double)lrHeight;
                double w10 = y_rem * (1.0 - x_rem) * (double)ulHeight;
                double w11 = y_rem * x_rem * (double)urHeight;

                result = (float)(w00 + w01 + w10 + w11);
            }
            else if ( _options.interpolation() == INTERP_BILINEAR )
            {
                //Check for exact value
                if ((colMax == colMin) && (rowMax == rowMin))
                {
                    //OE_NOTICE << "Exact value" << std::endl;
                    result = llHeight;
                }
                else if (colMax == colMin)
                {
                    //OE_NOTICE << "Vertically" << std::endl;
                    //Linear interpolate vertically
                    result = ((float)rowMax - r) * llHeight + (r - (float)rowMin) * ulHeight;
                }
                else if (rowMax == rowMin)
                {
                    //OE_NOTICE << "Horizontally" << std::endl;
                    //Linear interpolate horizontally
                    result = ((float)colMax - c) * llHeight + (c - (float)colMin) * lrHeight;
                }
                else
                {
                    //OE_NOTICE << "Bilinear" << std::endl;
                    //Bilinear interpolate
                    float r1 = ((float)colMax - c) * llHeight + (c - (float)colMin) * lrHeight;
                    float r2 = ((float)colMax - c) * ulHeight + (c - (float)colMin) * urHeight;

                    //OE_INFO << "r1, r2 = " << r1 << " , " << r2 << std::endl;
                    result = ((float)rowMax - r) * r1 + (r - (float)rowMin) * r2;
                }
            }
        }

        return result;
    }


    osg::HeightField* createHeightField( const TileKey& key,
                                         ProgressCallback* progress)
    {
        if (key.getLevelOfDetail() > _maxDataLevel)
        {
            //OE_NOTICE << "Reached maximum data resolution key=" << key.getLevelOfDetail() << " max=" << _maxDataLevel <<  std::endl;
            return NULL;
        }

        GDAL_SCOPED_LOCK;

        int tileSize = _options.tileSize().value();

        //Allocate the heightfield
        osg::ref_ptr<osg::HeightField> hf = new osg::HeightField;
        hf->allocate(tileSize, tileSize);

        if (intersects(key))
        {
            //Get the meter extents of the tile
            double xmin, ymin, xmax, ymax;
            key.getExtent().getBounds(xmin, ymin, xmax, ymax);

            //Just read from the first band
            GDALRasterBand* band = _warpedDS->GetRasterBand(1);

            double dx = (xmax - xmin) / (tileSize-1);
            double dy = (ymax - ymin) / (tileSize-1);

            for (int c = 0; c < tileSize; ++c)
            {
                double geoX = xmin + (dx * (double)c);
                for (int r = 0; r < tileSize; ++r)
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

    bool intersects(const TileKey& key)
    {
        //Get the native extents of the tile
        double xmin, ymin, xmax, ymax;
        key.getExtent().getBounds(xmin, ymin, xmax, ymax);

        return ! ( xmin >= _extentsMax.x() || xmax <= _extentsMin.x() || ymin >= _extentsMax.y() || ymax <= _extentsMin.y() );        
    }


private:

    GDALDataset* _srcDS;
    GDALDataset* _warpedDS;
    double       _geotransform[6];
    double       _invtransform[6];

    osg::Vec2d _extentsMin;
    osg::Vec2d _extentsMax;

    //std::string     _url;
    //int             _tile_size;
    //std::string     _extensions;
    //ElevationInterpolation   _interpolation;

    const GDALOptions _options;
    //osg::ref_ptr<const GDALOptions> _settings;

    unsigned int _maxDataLevel;
};


class ReaderWriterGDALTile : public TileSourceDriver
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
        return new GDALTileSource( getTileSourceOptions(opt) );
    }
};

REGISTER_OSGPLUGIN(osgearth_gdal, ReaderWriterGDALTile)
