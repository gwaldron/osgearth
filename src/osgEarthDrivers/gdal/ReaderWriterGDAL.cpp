/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2016 Pelican Mapping
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

#include <osgEarth/TileSource>
#include <osgEarth/FileUtils>
#include <osgEarth/Registry>
#include <osgEarth/ImageUtils>
#include <osgEarth/URI>
#include <osgEarth/HeightFieldUtils>

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

#define INDENT ""

// From easyrgb.com
float Hue_2_RGB( float v1, float v2, float vH )
{
   if ( vH < 0.0f ) vH += 1.0f;
   if ( vH > 1.0f ) vH -= 1.0f;
   if ( ( 6.0f * vH ) < 1.0f ) return ( v1 + ( v2 - v1 ) * 6.0f * vH );
   if ( ( 2.0f * vH ) < 1.0f ) return ( v2 );
   if ( ( 3.0f * vH ) < 2.0f ) return ( v1 + ( v2 - v1 ) * ( ( 2.0f / 3.0f ) - vH ) * 6.0f );
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
getFiles(const std::string &file, const std::vector<std::string> &exts, const std::vector<std::string> &blackExts, std::vector<std::string> &files)
{
    if (osgDB::fileType(file) == osgDB::DIRECTORY)
    {
        osgDB::DirectoryContents contents = osgDB::getDirectoryContents(file);
        for (osgDB::DirectoryContents::iterator itr = contents.begin(); itr != contents.end(); ++itr)
        {
            if (*itr == "." || *itr == "..") continue;
            std::string f = osgDB::concatPaths(file, *itr);
            getFiles(f, exts, blackExts, files);
        }
    }
    else
    {
		std::string ext = osgDB::getFileExtension(file);
        bool fileValid = false;
        //If we have no _extensions specified, assume we should try everything
        if (exts.size() == 0)
        {
            fileValid = true;
        }
        else
        {
            //Only accept files with the given _extensions
            for (unsigned int i = 0; i < exts.size(); ++i)
            {
                if (osgDB::equalCaseInsensitive(ext, exts[i]))
                {
                    fileValid = true;
                    break;
                }
            }
		}

		//Ignore any files that have blacklisted extensions
        for (unsigned int i = 0; i < blackExts.size(); ++i)
        {
			if (osgDB::equalCaseInsensitive(ext, blackExts[i]))
			{
				fileValid = false;
				break;
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
                ReadResult r = URI(prjLocation).readString();
                if ( r.succeeded() )
                {
                    proj = CPLStrdup( r.getString().c_str() );
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


/**
 * Gets the GeoExtent of the given filename.
 */
GeoExtent getGeoExtent(std::string& filename)
{
    GDALDataset* ds = (GDALDataset*)GDALOpen(filename.c_str(), GA_ReadOnly );
    if (!ds)
    {
        return GeoExtent::INVALID;
    }

    // Get the geotransforms
    double geotransform[6];
    ds->GetGeoTransform(geotransform);

    double minX, minY, maxX, maxY;

    GDALApplyGeoTransform(geotransform, 0.0, ds->GetRasterYSize(), &minX, &minY);
    GDALApplyGeoTransform(geotransform, ds->GetRasterXSize(), 0.0, &maxX, &maxY);

    std::string srsString = ds->GetProjectionRef();
    const SpatialReference* srs = SpatialReference::create(srsString);

    GDALClose(ds);

    GeoExtent ext(srs, minX, minY, maxX, maxY);
    return ext;
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

        // Close the _warpedDS dataset if :
        // - it exists
        // - and is different from _srcDS
        if (_warpedDS && (_warpedDS != _srcDS))
        {
            GDALClose( _warpedDS );
        }

        // Close the _srcDS dataset if :
        // - it exists
        // - and :
        //    -    is different from external dataset
        //    - or is equal to external dataset, but the tile source owns the external dataset
        if (_srcDS)
        {
            bool needClose = true;
            osg::ref_ptr<GDALOptions::ExternalDataset> pExternalDataset = _options.externalDataset();
            if (pExternalDataset != NULL)
            {
                if ( (pExternalDataset->dataset() == _srcDS) && (pExternalDataset->ownsDataset() == true) )
                {
                    needClose = false;
                }
            }

            if (needClose == true)
            {
                GDALClose(_srcDS);
            }
        }
    }


    Status initialize( const osgDB::Options* dbOptions )
    {
        GDAL_SCOPED_LOCK;

        _dbOptions = Registry::instance()->cloneOrCreateOptions( dbOptions );
        //if ( _dbOptions.valid() )
        //{
        //    // Set up a custom cache bin
        //    CacheManager* cacheManager = CacheManager::get(dbOptions);
        //    if (cacheManager && cacheManager->getCache())
        //    {
        //        std::string binId = Stringify() << std::hex << hashString(_options.getConfig().toJSON());
        //        _cacheBin = cacheManager->getCache()->addBin(binId);
        //    }
        //}

        // Is a valid external GDAL dataset specified ?
        bool useExternalDataset = false;
        osg::ref_ptr<GDALOptions::ExternalDataset> pExternalDataset = _options.externalDataset();
        if (pExternalDataset != NULL)
        {
            if (pExternalDataset->dataset() != NULL)
            {
                useExternalDataset = true;
            }
        }

        if (useExternalDataset == false &&
            (!_options.url().isSet() || _options.url()->empty()) &&
            (!_options.connection().isSet() || _options.connection()->empty()) )
        {
            return Status::Error(Status::ConfigurationError, "No URL, directory, or connection string specified" );
        }

        // source connection:
        std::string source;

        if ( _options.url().isSet() )
            source = _options.url()->full();
        else if ( _options.connection().isSet() )
            source = _options.connection().value();

        //URI uri = _options.url().value();

        if (useExternalDataset == false)
        {
            std::vector<std::string> files;

            if ( _options.url().isSet() )
            {
                // collect a list of files, filtering by extension if necessary
                StringTokenizer izer( ";" );
                StringVector exts;
                izer.tokenize( *_options.extensions(), exts );

				StringVector blackExts;
				izer.tokenize( *_options.blackExtensions(), blackExts );

                for (unsigned int i = 0; i < exts.size(); ++i)
                {
                    OE_DEBUG << LC << "Using Extension: " << exts[i] << std::endl;
                }

				for (unsigned int i = 0; i < blackExts.size(); ++i)
				{
					OE_DEBUG << LC << "Blacklisting Extension: " << blackExts[i] << std::endl;
				}

                getFiles(source, exts, blackExts, files);

                OE_INFO << LC << "Identified " << files.size() << " files:" << std::endl;
                for (unsigned int i = 0; i < files.size(); ++i)
                {
                    OE_INFO << LC << INDENT << files[i] << std::endl;
                }
            }
            else
            {
                // just add the connection string as the single source.
                files.push_back( source );
            }

            if (files.empty())
            {
                return Status::Error( Status::ResourceUnavailable, "Could not find any valid input." );
            }

            //If we found more than one file, try to combine them into a single logical dataset
            if (files.size() > 1)
            {
                std::string vrtKey = "combined.vrt";

                //Get the GDAL VRT driver
                GDALDriver* vrtDriver = (GDALDriver*)GDALGetDriverByName("VRT");

                //Try to load the VRT file from the cache so we don't have to build it each time.
                if (_cacheBin.valid())
                {
                    ReadResult result = _cacheBin->readString( vrtKey, 0L);
                    if (result.succeeded())
                    {
                        _srcDS = (GDALDataset*)GDALOpen(result.getString().c_str(), GA_ReadOnly );
                        if (_srcDS)
                        {
                            OE_INFO << LC << INDENT << "Read VRT from cache!" << std::endl;
                        }
                    }
                }

                //Build the dataset if we didn't already load it
                if (!_srcDS)
                {
                    //We couldn't get the VRT from the cache, so build it
                    osg::Timer_t startTime = osg::Timer::instance()->tick();
                    _srcDS = (GDALDataset*)build_vrt(files, HIGHEST_RESOLUTION);
                    osg::Timer_t endTime = osg::Timer::instance()->tick();
                    OE_INFO << LC << INDENT << "Built VRT in " << osg::Timer::instance()->delta_s(startTime, endTime) << " s" << std::endl;

                    if (_srcDS)
                    {
                        //Cache the VRT so we don't have to build it next time.
                        if (_cacheBin)
                        {
                            std::string vrtFile = getTempName( "", ".vrt");
                            OE_INFO << LC << INDENT << "Writing temp VRT to " << vrtFile << std::endl;

                            if (vrtDriver)
                            {
                                if ( vrtDriver->CreateCopy(vrtFile.c_str(), _srcDS, 0, 0, 0, 0 ) == NULL )
                                {
                                    OE_WARN << LC << INDENT << "Faile to create copy" << std::endl;
                                }

                                //We created the temp file, now read the contents back
                                std::ifstream input( vrtFile.c_str() );
                                if ( input.is_open() )
                                {
                                    input >> std::noskipws;
                                    std::stringstream buf;
                                    buf << input.rdbuf();
                                    std::string vrtContents = buf.str();
                                    osg::ref_ptr< StringObject > strObject = new StringObject( vrtContents );
                                    _cacheBin->write(vrtKey, strObject.get(), 0L);
                                }
                            }
                            if (osgDB::fileExists( vrtFile ) )
                            {
                                remove( vrtFile.c_str() );
                            }
                        }
                    }
                    else
                    {
                        return Status::Error( "Failed to build VRT from input datasets" );
                    }
                }
            }
            else
            {
                //If we couldn't build a VRT, just try opening the file directly
                //Open the dataset
                _srcDS = (GDALDataset*)GDALOpen( files[0].c_str(), GA_ReadOnly );

                if (_srcDS)
                {

                    char **subDatasets = _srcDS->GetMetadata( "SUBDATASETS");
                    int numSubDatasets = CSLCount( subDatasets );
                    //OE_NOTICE << "There are " << numSubDatasets << " in this file " << std::endl;

                    if (numSubDatasets > 0)
                    {
                        int subDataset = _options.subDataSet().isSet() ? *_options.subDataSet() : 1;
                        if (subDataset < 1 || subDataset > numSubDatasets) subDataset = 1;
                        std::stringstream buf;
                        buf << "SUBDATASET_" << subDataset << "_NAME";
                        char *pszSubdatasetName = CPLStrdup( CSLFetchNameValue( subDatasets, buf.str().c_str() ) );
                        GDALClose( _srcDS );
                        _srcDS = (GDALDataset*)GDALOpen( pszSubdatasetName, GA_ReadOnly ) ;
                        CPLFree( pszSubdatasetName );
                    }
                }

                if (!_srcDS)
                {
                    return Status::Error( Status::ResourceUnavailable, Stringify() << "Failed to open " << files[0] );
                }
            }
        }
        else
        {
            _srcDS = pExternalDataset->dataset();
        }


        //Get the "warp profile", which is the profile that this dataset should take on by creating a warping VRT.  This is
        //useful when you want to use multiple images of different projections in a composite image.
        osg::ref_ptr< const Profile > warpProfile;
        if (_options.warpProfile().isSet())
        {
            warpProfile = Profile::create( _options.warpProfile().value() );
        }

        if (warpProfile.valid())
        {
            OE_INFO << LC << INDENT << "Created warp profile " << warpProfile->toString() <<  std::endl;
        }




        //Create a spatial reference for the source.
        std::string srcProj = _srcDS->GetProjectionRef();

        // If the projection is empty and we have GCP's then use the GCP projection.
        if (srcProj.empty() && _srcDS->GetGCPCount() > 0)
        {
            srcProj = _srcDS->GetGCPProjection();
        }


        if ( !srcProj.empty() && getProfile() != 0L )
        {
            OE_WARN << LC << "Overriding profile of a source that already defines its own SRS ("
                << this->getName() << ")" << std::endl;
        }

        osg::ref_ptr<const SpatialReference> src_srs;
        if ( getProfile() )
        {
            src_srs = getProfile()->getSRS();
        }
        else if ( !srcProj.empty() )
        {
            src_srs = SpatialReference::create( srcProj );
            if ( !src_srs.valid() )
            {
                OE_DEBUG << LC << "Cannot create source SRS from its projection info: " << srcProj << std::endl;
            }
        }

        // assert SRS is present
        if ( !src_srs.valid() )
        {
            // not found in the dataset; try loading a .prj file
            std::string prjLocation = osgDB::getNameLessExtension(source) + std::string(".prj");

            ReadResult r = URI(prjLocation).readString( _dbOptions.get() );
            if ( r.succeeded() )
            {
                src_srs = SpatialReference::create( r.getString() );
            }

            if ( !src_srs.valid() )
            {
                return Status::Error( Status::ResourceUnavailable, Stringify()
                    << "Dataset has no spatial reference information (" << source << ")" );
            }
        }


        //Get the initial geotransform
        _srcDS->GetGeoTransform(_geotransform);

        bool hasGCP = _srcDS->GetGCPCount() > 0 && _srcDS->GetGCPProjection();
        bool isRotated = _geotransform[2] != 0.0 || _geotransform[4];
        if (hasGCP) OE_DEBUG << LC << source << " has GCP georeferencing" << std::endl;
        if (isRotated) OE_DEBUG << LC << source << " is rotated " << std::endl;
        bool requiresReprojection = hasGCP || isRotated;

        const Profile* profile = NULL;

        // The warp profile, if provided, takes precedence.
        if ( warpProfile )
        {
            profile = warpProfile;
            if ( profile )
            {
                OE_DEBUG << LC << INDENT << "Using warp Profile: " << profile->toString() <<  std::endl;
            }
        }

        // If we have an override profile, just take it.
        if ( getProfile() )
        {
            profile = getProfile();
            if ( profile )
            {
                OE_DEBUG << LC << INDENT << "Using override Profile: " << profile->toString() <<  std::endl;
            }
        }

        // If neither a warp nor override profile were provided, work out the profile from the source's own SRS.
        if ( !profile && src_srs->isGeographic() )
        {
            OE_DEBUG << LC << INDENT << "Creating Profile from source's geographic SRS: " << src_srs->getName() <<  std::endl;
            profile = Profile::create(src_srs.get(), -180.0, -90.0, 180.0, 90.0, 2u, 1u);
            //profile = osgEarth::Registry::instance()->getGlobalGeodeticProfile();
            if ( !profile )
            {
                return Status::Error( Status::ResourceUnavailable, Stringify()
                    << "Cannot create geographic Profile from dataset's spatial reference information: " << src_srs->getName() );
            }
        }

        std::string warpedSRSWKT;

        if ( requiresReprojection || (profile && !profile->getSRS()->isEquivalentTo( src_srs.get() )) )
        {
            if ( profile && profile->getSRS()->isGeographic() && (src_srs->isNorthPolar() || src_srs->isSouthPolar()) )
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
                std::string destWKT = profile ? profile->getSRS()->getWKT() : src_srs->getWKT();
                _warpedDS = (GDALDataset*)GDALAutoCreateWarpedVRT(
                    _srcDS,
                    src_srs->getWKT().c_str(),
                    destWKT.c_str(),
                    GRA_NearestNeighbour,
                    5.0,
                    0);
            }

            if ( _warpedDS )
            {
                warpedSRSWKT = _warpedDS->GetProjectionRef();
            }
        }
        else
        {
            _warpedDS = _srcDS;
            warpedSRSWKT = src_srs->getWKT();
        }

        if ( !_warpedDS )
        {
            return Status::Error( "Failed to create a warping VRT" );
        }

        //Get the _geotransform
        if ( getProfile() )
        {
            OE_DEBUG << LC << INDENT << "Get geotransform from Override Profile" <<  std::endl;
            _geotransform[0] =  getProfile()->getExtent().xMin(); //Top left x
            _geotransform[1] =  getProfile()->getExtent().width() / (double)_warpedDS->GetRasterXSize();//pixel width
            _geotransform[2] =  0;

            _geotransform[3] =  getProfile()->getExtent().yMax(); //Top left y
            _geotransform[4] =  0;
            _geotransform[5] = -getProfile()->getExtent().height() / (double)_warpedDS->GetRasterYSize();//pixel height

        }
        else
        {
            OE_DEBUG << LC << INDENT << "Get geotransform from warped dataset" <<  std::endl;
            _warpedDS->GetGeoTransform(_geotransform);
        }

        if ( GDALInvGeoTransform(_geotransform, _invtransform) == 0 )
        {
            OE_WARN << LC << INDENT << "_geotransform not invertible" << std::endl;
        }

        double minX, minY, maxX, maxY;


        //Compute the extents
        // polar needs a special case when combined with geographic
        if ( profile && profile->getSRS()->isGeographic() && (src_srs->isNorthPolar() || src_srs->isSouthPolar()) )
        {
            double ll_lon, ll_lat, ul_lon, ul_lat, ur_lon, ur_lat, lr_lon, lr_lat;

            pixelToGeo(0.0, 0.0, ul_lon, ul_lat );
            pixelToGeo(0.0, _warpedDS->GetRasterYSize() + 1, ll_lon, ll_lat);
            pixelToGeo(_warpedDS->GetRasterXSize(), _warpedDS->GetRasterYSize(), lr_lon, lr_lat);
            pixelToGeo(_warpedDS->GetRasterXSize(), 0.0, ur_lon, ur_lat);

            minX = osg::minimum( ll_lon, osg::minimum( ul_lon, osg::minimum( ur_lon, lr_lon ) ) );
            maxX = osg::maximum( ll_lon, osg::maximum( ul_lon, osg::maximum( ur_lon, lr_lon ) ) );

            if ( src_srs->isNorthPolar() )
            {
                minY = osg::minimum( ll_lat, osg::minimum( ul_lat, osg::minimum( ur_lat, lr_lat ) ) );
                maxY = 90.0;
            }
            else
            {
                minY = -90.0;
                maxY = osg::maximum( ll_lat, osg::maximum( ul_lat, osg::maximum( ur_lat, lr_lat ) ) );
            }
        }
        else
        {
            pixelToGeo(0.0, _warpedDS->GetRasterYSize(), minX, minY);
            pixelToGeo(_warpedDS->GetRasterXSize(), 0.0, maxX, maxY);
        }




        OE_DEBUG << LC << INDENT << "Geo extents: " << minX << ", " << minY << " -> " << maxX << ", " << maxY << std::endl;

        if ( !profile )
        {
            profile = Profile::create(
                warpedSRSWKT,
                minX, minY, maxX, maxY);

            if ( !profile )
            {
                return Status::Error( Stringify()
                    << "Cannot create projected Profile from dataset's warped spatial reference WKT: " << warpedSRSWKT );
            }

            OE_INFO << LC << INDENT << source << " is projected, SRS = "
                << warpedSRSWKT << std::endl;
                //<< _warpedDS->GetProjectionRef() << std::endl;
        }

        //Compute the min and max data levels
        double resolutionX = (maxX - minX) / (double)_warpedDS->GetRasterXSize();
        double resolutionY = (maxY - minY) / (double)_warpedDS->GetRasterYSize();

        double maxResolution = osg::minimum(resolutionX, resolutionY);

        OE_INFO << LC << INDENT << "Resolution= " << resolutionX << "x" << resolutionY << " max=" << maxResolution << std::endl;

        if (_options.maxDataLevelOverride().isSet())
        {
            _maxDataLevel = _options.maxDataLevelOverride().value();
            OE_INFO << LC << INDENT << _options.url().value().full() << " using override max data level " << _maxDataLevel << std::endl;
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

            OE_INFO << LC << INDENT << _options.url().value().full() << " max Data Level: " << _maxDataLevel << std::endl;
        }

        // If the input dataset is a VRT, then get the individual files in the dataset and use THEM for the DataExtents.
        // A VRT will create a potentially very large virtual dataset from sparse datasets, so using the extents from the underlying files
        // will allow osgEarth to only create tiles where there is actually data.
        DataExtentList dataExtents;
        if (strcmp(_warpedDS->GetDriver()->GetDescription(), "VRT") == 0)
        {
            char **papszFileList = _warpedDS->GetFileList();
            if (papszFileList != NULL)
            {
                for( int i = 0; papszFileList[i] != NULL; i++ )
                {
                    std::string file = papszFileList[i];
                    GeoExtent ext = getGeoExtent(file);
                    if (ext.isValid())
                    {
                        dataExtents.push_back(DataExtent(ext, 0, _maxDataLevel));
                    }
                }
            }
        }


        osg::ref_ptr< SpatialReference > srs = SpatialReference::create( warpedSRSWKT );
        // record the data extent in profile space:
        _bounds = Bounds(minX, minY, maxX, maxY);
        _extents = GeoExtent( srs, _bounds);
        GeoExtent profile_extent = _extents.transform( profile->getSRS() );

        if (dataExtents.empty())
        {
            // Use the extents of the whole file.
            getDataExtents().push_back( DataExtent(profile_extent, 0, _maxDataLevel) );
        }
        else
        {
            // Use the DataExtents from the subfiles of the VRT.
            getDataExtents().insert(getDataExtents().end(), dataExtents.begin(), dataExtents.end());
        }

        //Set the profile
        setProfile( profile );
        OE_DEBUG << LC << INDENT << "Set Profile to " << (profile ? profile->toString() : "NULL") <<  std::endl;

        return STATUS_OK;
    }


    /**
    * Finds a raster band based on color interpretation
    */
    static GDALRasterBand* findBandByColorInterp(GDALDataset *ds, GDALColorInterp colorInterp)
    {
        GDAL_SCOPED_LOCK;

        for (int i = 1; i <= ds->GetRasterCount(); ++i)
        {
            if (ds->GetRasterBand(i)->GetColorInterpretation() == colorInterp) return ds->GetRasterBand(i);
        }
        return 0;
    }

    static GDALRasterBand* findBandByDataType(GDALDataset *ds, GDALDataType dataType)
    {
        GDAL_SCOPED_LOCK;

        for (int i = 1; i <= ds->GetRasterCount(); ++i)
        {
            if (ds->GetRasterBand(i)->GetRasterDataType() == dataType) return ds->GetRasterBand(i);
        }
        return 0;
    }

    static void getPalleteIndexColor(GDALRasterBand* band, int index, osg::Vec4ub& color)
    {
        const GDALColorEntry *colorEntry = band->GetColorTable()->GetColorEntry( index );
        GDALPaletteInterp interp = band->GetColorTable()->GetPaletteInterpretation();
        if (!colorEntry)
        {
            //FIXME: What to do here?

            //OE_INFO << "NO COLOR ENTRY FOR COLOR " << rawImageData[i] << std::endl;
            color.r() = 255;
            color.g() = 0;
            color.b() = 0;
            color.a() = 1;

        }
        else
        {
            if (interp == GPI_RGB)
            {
                color.r() = colorEntry->c1;
                color.g() = colorEntry->c2;
                color.b() = colorEntry->c3;
                color.a() = colorEntry->c4;
            }
            else if (interp == GPI_CMYK)
            {
                // from wikipedia.org
                short C = colorEntry->c1;
                short M = colorEntry->c2;
                short Y = colorEntry->c3;
                short K = colorEntry->c4;
                color.r() = 255 - C*(255 - K) - K;
                color.g() = 255 - M*(255 - K) - K;
                color.b() = 255 - Y*(255 - K) - K;
                color.a() = 255;
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

                    R = Hue_2_RGB( var_1, var_2, H + ( 1.0f / 3.0f ) );
                    G = Hue_2_RGB( var_1, var_2, H );
                    B = Hue_2_RGB( var_1, var_2, H - ( 1.0f / 3.0f ) );
                }
                color.r() = static_cast<unsigned char>(R*255.0f);
                color.g() = static_cast<unsigned char>(G*255.0f);
                color.b() = static_cast<unsigned char>(B*255.0f);
                color.a() = static_cast<unsigned char>(255.0f);
            }
            else if (interp == GPI_Gray)
            {
                color.r() = static_cast<unsigned char>(colorEntry->c1*255.0f);
                color.g() = static_cast<unsigned char>(colorEntry->c1*255.0f);
                color.b() = static_cast<unsigned char>(colorEntry->c1*255.0f);
                color.a() = static_cast<unsigned char>(255.0f);
            }
        }
    }

    void pixelToGeo(double x, double y, double &geoX, double &geoY)
    {
        geoX = _geotransform[0] + _geotransform[1] * x + _geotransform[2] * y;
        geoY = _geotransform[3] + _geotransform[4] * x + _geotransform[5] * y;
    }

    void geoToPixel(double geoX, double geoY, double &x, double &y)
    {
        x = _invtransform[0] + _invtransform[1] * geoX + _invtransform[2] * geoY;
        y = _invtransform[3] + _invtransform[4] * geoX + _invtransform[5] * geoY;

         //Account for slight rounding errors.  If we are right on the edge of the dataset, clamp to the edge
        double eps = 0.0001;
        if (osg::equivalent(x, 0, eps)) x = 0;
        if (osg::equivalent(y, 0, eps)) y = 0;
        if (osg::equivalent(x, (double)_warpedDS->GetRasterXSize(), eps)) x = _warpedDS->GetRasterXSize();
        if (osg::equivalent(y, (double)_warpedDS->GetRasterYSize(), eps)) y = _warpedDS->GetRasterYSize();

    }

    osg::Image* createImage( const TileKey&        key,
        ProgressCallback*     progress)
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

        //Get the extents of the tile
        double xmin, ymin, xmax, ymax;
        key.getExtent().getBounds(xmin, ymin, xmax, ymax);

        // Compute the intersection of the incoming key with the data extents of the dataset
        osgEarth::GeoExtent intersection = key.getExtent().intersectionSameSRS( _extents );
        if (!intersection.isValid())
        {
            return 0;
        }

        double west = intersection.xMin();
        double east = intersection.xMax();
        double north = intersection.yMax();
        double south = intersection.yMin();

        // The extents and the intersection will be normalized between -180 and 180 longitude if they are geographic.
        // However, the georeferencing will expect the coordinates to be in the same longitude frame as the original dataset,
        // so the intersection bounds are adjusted here if necessary so that the values line up with the georeferencing.
        if (_extents.getSRS()->isGeographic())
        {
            // Shift the bounds to the right.
            if (east < _bounds.xMin())
            {
                while (east < _bounds.xMin())
                {
                    west += 360.0;
                    east += 360.0;
                }
            }
            // Shift the bounds to the left.
            else if (west > _bounds.xMax())
            {
                while (west > _bounds.xMax())
                {
                    west -= 360.0;
                    east -= 360.0;
                }
            }
        }
        
        // Determine the read window
        double src_min_x, src_min_y, src_max_x, src_max_y;
        // Get the pixel coordiantes of the intersection
        geoToPixel( west, intersection.yMax(), src_min_x, src_min_y);
        geoToPixel( east, intersection.yMin(), src_max_x, src_max_y);

        // Convert the doubles to integers.  We floor the mins and ceil the maximums to give the widest window possible.
        src_min_x = floor(src_min_x);
        src_min_y = floor(src_min_y);
        src_max_x = ceil(src_max_x);
        src_max_y = ceil(src_max_y);

        int off_x = (int)( src_min_x );
        int off_y = (int)( src_min_y );
        int width  = (int)(src_max_x - src_min_x);
        int height = (int)(src_max_y - src_min_y);


        int rasterWidth = _warpedDS->GetRasterXSize();
        int rasterHeight = _warpedDS->GetRasterYSize();
        if (off_x + width > rasterWidth || off_y + height > rasterHeight)
        {
            OE_WARN << LC << "Read window outside of bounds of dataset.  Source Dimensions=" << rasterWidth << "x" << rasterHeight << " Read Window=" << off_x << ", " << off_y << " " << width << "x" << height << std::endl;
        }

        // Determine the destination window

        // Compute the offsets in geo coordinates of the intersection from the TileKey
        double offset_left = intersection.xMin() - xmin;
        double offset_top = ymax - intersection.yMax();


        int target_width = (int)ceil((intersection.width() / key.getExtent().width())*(double)tileSize);
        int target_height = (int)ceil((intersection.height() / key.getExtent().height())*(double)tileSize);
        int tile_offset_left = (int)floor((offset_left / key.getExtent().width()) * (double)tileSize);
        int tile_offset_top = (int)floor((offset_top / key.getExtent().height()) * (double)tileSize);

        // Compute spacing
        double dx       = (xmax - xmin) / (tileSize-1);
        double dy       = (ymax - ymin) / (tileSize-1);

        OE_DEBUG << LC << "ReadWindow " << off_x << "," << off_y << " " << width << "x" << height << std::endl;
        OE_DEBUG << LC << "DestWindow " << tile_offset_left << "," << tile_offset_top << " " << target_width << "x" << target_height << std::endl;


        //Return if parameters are out of range.
        if (width <= 0 || height <= 0 || target_width <= 0 || target_height <= 0)
        {
            return 0;
        }



        GDALRasterBand* bandRed = findBandByColorInterp(_warpedDS, GCI_RedBand);
        GDALRasterBand* bandGreen = findBandByColorInterp(_warpedDS, GCI_GreenBand);
        GDALRasterBand* bandBlue = findBandByColorInterp(_warpedDS, GCI_BlueBand);
        GDALRasterBand* bandAlpha = findBandByColorInterp(_warpedDS, GCI_AlphaBand);

        GDALRasterBand* bandGray = findBandByColorInterp(_warpedDS, GCI_GrayIndex);

        GDALRasterBand* bandPalette = findBandByColorInterp(_warpedDS, GCI_PaletteIndex);

        if (!bandRed && !bandGreen && !bandBlue && !bandAlpha && !bandGray && !bandPalette)
        {
            OE_DEBUG << LC << "Could not determine bands based on color interpretation, using band count" << std::endl;
            //We couldn't find any valid bands based on the color interp, so just make an educated guess based on the number of bands in the file
            //RGB = 3 bands
            if (_warpedDS->GetRasterCount() == 3)
            {
                bandRed   = _warpedDS->GetRasterBand( 1 );
                bandGreen = _warpedDS->GetRasterBand( 2 );
                bandBlue  = _warpedDS->GetRasterBand( 3 );
            }
            //RGBA = 4 bands
            else if (_warpedDS->GetRasterCount() == 4)
            {
                bandRed   = _warpedDS->GetRasterBand( 1 );
                bandGreen = _warpedDS->GetRasterBand( 2 );
                bandBlue  = _warpedDS->GetRasterBand( 3 );
                bandAlpha = _warpedDS->GetRasterBand( 4 );
            }
            //Gray = 1 band
            else if (_warpedDS->GetRasterCount() == 1)
            {
                bandGray = _warpedDS->GetRasterBand( 1 );
            }
            //Gray + alpha = 2 bands
            else if (_warpedDS->GetRasterCount() == 2)
            {
                bandGray  = _warpedDS->GetRasterBand( 1 );
                bandAlpha = _warpedDS->GetRasterBand( 2 );
            }
        }



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

            image = new osg::Image;
            image->allocateImage(tileSize, tileSize, 1, pixelFormat, GL_UNSIGNED_BYTE);
            memset(image->data(), 0, image->getImageSizeInBytes());

            //Nearest interpolation just uses RasterIO to sample the imagery and should be very fast.
            if (!*_options.interpolateImagery() || _options.interpolation() == INTERP_NEAREST)
            {
                bandRed->RasterIO(GF_Read, off_x, off_y, width, height, red, target_width, target_height, GDT_Byte, 0, 0);
                bandGreen->RasterIO(GF_Read, off_x, off_y, width, height, green, target_width, target_height, GDT_Byte, 0, 0);
                bandBlue->RasterIO(GF_Read, off_x, off_y, width, height, blue, target_width, target_height, GDT_Byte, 0, 0);

                if (bandAlpha)
                {
                    bandAlpha->RasterIO(GF_Read, off_x, off_y, width, height, alpha, target_width, target_height, GDT_Byte, 0, 0);
                }

                for (int src_row = 0, dst_row = tile_offset_top;
                    src_row < target_height;
                    src_row++, dst_row++)
                {
                    for (int src_col = 0, dst_col = tile_offset_left;
                        src_col < target_width;
                        ++src_col, ++dst_col)
                    {
                        unsigned char r = red[src_col + src_row * target_width];
                        unsigned char g = green[src_col + src_row * target_width];
                        unsigned char b = blue[src_col + src_row * target_width];
                        unsigned char a = alpha[src_col + src_row * target_width];
                        *(image->data(dst_col, dst_row) + 0) = r;
                        *(image->data(dst_col, dst_row) + 1) = g;
                        *(image->data(dst_col, dst_row) + 2) = b;
                        if (!isValidValue( r, bandRed)    ||
                            !isValidValue( g, bandGreen)  ||
                            !isValidValue( b, bandBlue)   ||
                            (bandAlpha && !isValidValue( a, bandAlpha )))
                        {
                            a = 0.0f;
                        }
                        *(image->data(dst_col, dst_row) + 3) = a;
                    }
                }

                image->flipVertical();
            }
            else
            {
                //Sample each point exactly
                for (unsigned int c = 0; c < (unsigned int)tileSize; ++c)
                {
                    double geoX = xmin + (dx * (double)c);
                    for (unsigned int r = 0; r < (unsigned int)tileSize; ++r)
                    {
                        double geoY = ymin + (dy * (double)r);
                        *(image->data(c,r) + 0) = (unsigned char)getInterpolatedValue(bandRed,  geoX,geoY,false);
                        *(image->data(c,r) + 1) = (unsigned char)getInterpolatedValue(bandGreen,geoX,geoY,false);
                        *(image->data(c,r) + 2) = (unsigned char)getInterpolatedValue(bandBlue, geoX,geoY,false);
                        if (bandAlpha != NULL)
                            *(image->data(c,r) + 3) = (unsigned char)getInterpolatedValue(bandAlpha,geoX, geoY, false);
                        else
                            *(image->data(c,r) + 3) = 255;
                    }
                }
            }

            delete []red;
            delete []green;
            delete []blue;
            delete []alpha;
        }
        else if (bandGray)
        {
            if ( getOptions().coverage() == true )
            {
                GDALDataType gdalDataType = bandGray->GetRasterDataType();
                int          gdalSampleSize;
                GLenum       glDataType;
                GLint        internalFormat;

                switch(gdalDataType)
                {
                case GDT_Byte:
                    glDataType = GL_FLOAT;
                    gdalSampleSize = 1;
                    internalFormat = GL_LUMINANCE32F_ARB;
                    break;

                case GDT_UInt16:
                case GDT_Int16:
                    glDataType = GL_FLOAT;
                    gdalSampleSize = 2;
                    internalFormat = GL_LUMINANCE32F_ARB;
                    break;

                default:
                    glDataType = GL_FLOAT;
                    gdalSampleSize = 4;
                    internalFormat = GL_LUMINANCE32F_ARB;
                }

                // Create an un-normalized luminance image to hold coverage values.
                image = new osg::Image();
                image->allocateImage( tileSize, tileSize, 1, GL_LUMINANCE, glDataType );
                image->setInternalTextureFormat( internalFormat );
                ImageUtils::markAsUnNormalized( image, true );
                memset(image->data(), 0, image->getImageSizeInBytes());

                ImageUtils::PixelWriter write(image);

                // initialize all coverage texels to NODATA. -gw
                osg::Vec4 temp;
                temp.r() = NO_DATA_VALUE;

                for(int s=0; s<image->s(); ++s) {
                    for(int t=0; t<image->t(); ++t) {
                        write(temp, s, t);
                    }
                }

                // coverage data; one channel data that is not subject to interpolated values
                unsigned char* data = new unsigned char[target_width * target_height * gdalSampleSize];
                memset(data, 0, target_width * target_height * gdalSampleSize);


                int success;
                float nodata = bandGray->GetNoDataValue(&success);
                if ( !success )
                    nodata = getOptions().noDataValue().get();

                CPLErr err = bandGray->RasterIO(GF_Read, off_x, off_y, width, height, data, target_width, target_height, gdalDataType, 0, 0);
                if ( err == CE_None )
                {
                    // copy from data to image.
                    for (int src_row = 0, dst_row = tile_offset_top; src_row < target_height; src_row++, dst_row++)
                    {
                        for (int src_col = 0, dst_col = tile_offset_left; src_col < target_width; ++src_col, ++dst_col)
                        {
                            unsigned char* ptr = &data[(src_col + src_row*target_width)*gdalSampleSize];

                            float value =
                                gdalSampleSize == 1 ? (float)(*ptr) :
                                gdalSampleSize == 2 ? (float)*(unsigned short*)ptr :
                                gdalSampleSize == 4 ? *(float*)ptr :
                                NO_DATA_VALUE;

                            if ( !isValidValue_noLock(value, bandGray) )
                                value = NO_DATA_VALUE;

                            temp.r() = value;
                            write(temp, dst_col, dst_row);
                        }
                    }

                    // TODO: can we replace this by writing rows in reverse order? -gw
                    image->flipVertical();
                }
                else // err != CE_None
                {
                    OE_WARN << LC << "RasterIO failed.\n";
                    // TODO - handle error condition
                }

                delete [] data;
            }

            else // greyscale image (not a coverage)
            {
                unsigned char *gray = new unsigned char[target_width * target_height];
                unsigned char *alpha = new unsigned char[target_width * target_height];

                //Initialize the alpha values to 255.
                memset(alpha, 255, target_width * target_height);

                image = new osg::Image;
                image->allocateImage(tileSize, tileSize, 1, pixelFormat, GL_UNSIGNED_BYTE);
                memset(image->data(), 0, image->getImageSizeInBytes());


                if (!*_options.interpolateImagery() || _options.interpolation() == INTERP_NEAREST)
                {
                    bandGray->RasterIO(GF_Read, off_x, off_y, width, height, gray, target_width, target_height, GDT_Byte, 0, 0);

                    if (bandAlpha)
                    {
                        bandAlpha->RasterIO(GF_Read, off_x, off_y, width, height, alpha, target_width, target_height, GDT_Byte, 0, 0);
                    }

                    for (int src_row = 0, dst_row = tile_offset_top;
                        src_row < target_height;
                        src_row++, dst_row++)
                    {
                        for (int src_col = 0, dst_col = tile_offset_left;
                            src_col < target_width;
                            ++src_col, ++dst_col)
                        {
                            unsigned char g = gray[src_col + src_row * target_width];
                            unsigned char a = alpha[src_col + src_row * target_width];
                            *(image->data(dst_col, dst_row) + 0) = g;
                            *(image->data(dst_col, dst_row) + 1) = g;
                            *(image->data(dst_col, dst_row) + 2) = g;
                            if (!isValidValue( g, bandGray) ||
                                (bandAlpha && !isValidValue( a, bandAlpha)))
                            {
                                a = 0.0f;
                            }
                            *(image->data(dst_col, dst_row) + 3) = a;
                        }
                    }

                    image->flipVertical();
                }
                else
                {
                    for (int r = 0; r < tileSize; ++r)
                    {
                        double geoY   = ymin + (dy * (double)r);

                        for (int c = 0; c < tileSize; ++c)
                        {
                            double geoX = xmin + (dx * (double)c);
                            float  color = getInterpolatedValue(bandGray,geoX,geoY,false);

                            *(image->data(c,r) + 0) = (unsigned char)color;
                            *(image->data(c,r) + 1) = (unsigned char)color;
                            *(image->data(c,r) + 2) = (unsigned char)color;
                            if (bandAlpha != NULL)
                                *(image->data(c,r) + 3) = (unsigned char)getInterpolatedValue(bandAlpha,geoX,geoY,false);
                            else
                                *(image->data(c,r) + 3) = 255;
                        }
                    }
                }

                delete []gray;
                delete []alpha;
            }
        }
        else if (bandPalette)
        {
            //Pallete indexed imagery doesn't support interpolation currently and only uses nearest
            //b/c interpolating pallete indexes doesn't make sense.
            unsigned char *palette = new unsigned char[target_width * target_height];

            image = new osg::Image;

            if ( _options.coverage() == true )
            {
                image->allocateImage(tileSize, tileSize, 1, GL_LUMINANCE, GL_FLOAT);
                image->setInternalTextureFormat(GL_LUMINANCE32F_ARB);
                ImageUtils::markAsUnNormalized(image, true);

                // initialize all coverage texels to NODATA. -gw
                osg::Vec4 temp;
                temp.r() = NO_DATA_VALUE;
                ImageUtils::PixelWriter write(image);
                for(int s=0; s<image->s(); ++s) {
                    for(int t=0; t<image->t(); ++t) {
                        write(temp, s, t);
                    }
                }
            }
            else
            {
                image->allocateImage(tileSize, tileSize, 1, pixelFormat, GL_UNSIGNED_BYTE);
                memset(image->data(), 0, image->getImageSizeInBytes());
            }

            bandPalette->RasterIO(GF_Read, off_x, off_y, width, height, palette, target_width, target_height, GDT_Byte, 0, 0);

            ImageUtils::PixelWriter write(image);

            for (int src_row = 0, dst_row = tile_offset_top;
                src_row < target_height;
                src_row++, dst_row++)
            {
                for (int src_col = 0, dst_col = tile_offset_left;
                    src_col < target_width;
                    ++src_col, ++dst_col)
                {
                    unsigned char p = palette[src_col + src_row * target_width];

                    if ( _options.coverage() == true )
                    {
                        osg::Vec4 pixel;
                        if ( isValidValue(p, bandPalette) )
                            pixel.r() = (float)p;
                        else
                            pixel.r() = NO_DATA_VALUE;

                        write(pixel, dst_col, dst_row);
                    }
                    else
                    {
                        osg::Vec4ub color;
                        getPalleteIndexColor( bandPalette, p, color );
                        if (!isValidValue( p, bandPalette))
                        {
                            color.a() = 0.0f;
                        }

                        *(image->data(dst_col, dst_row) + 0) = color.r();
                        *(image->data(dst_col, dst_row) + 1) = color.g();
                        *(image->data(dst_col, dst_row) + 2) = color.b();
                        *(image->data(dst_col, dst_row) + 3) = color.a();
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
                << _options.url()->full()
                << ".  Cannot create image. " << std::endl;

            return NULL;
        }

        return image.release();
    }

    bool isValidValue_noLock(float v, GDALRasterBand* band)
    {
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
        if (v < getMinValidValue()) return false;
        if (v > getMaxValidValue()) return false;

        return true;
    }

    bool isValidValue(float v, GDALRasterBand* band)
    {
        GDAL_SCOPED_LOCK;
        return isValidValue_noLock( v, band );
    }


    float getInterpolatedValue(GDALRasterBand *band, double x, double y, bool applyOffset=true)
    {
        double r, c;
        geoToPixel( x, y, c, r );


        if (applyOffset)
        {
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
            if ((!isValidValue(urHeight, band)) || (!isValidValue(llHeight, band)) ||(!isValidValue(ulHeight, band)) || (!isValidValue(lrHeight, band)))
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


#if 1
    osg::HeightField* createHeightField( const TileKey&        key,
                                         ProgressCallback*     progress)
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

            // Try to find a FLOAT band
            GDALRasterBand* band = findBandByDataType(_warpedDS, GDT_Float32);
            if (band == NULL)
            {
                // Just get first band
                band = _warpedDS->GetRasterBand(1);
            }

            if (_options.interpolation() == INTERP_NEAREST)
            {
                double colMin, colMax;
                double rowMin, rowMax;
                geoToPixel( xmin, ymin, colMin, rowMax );
                geoToPixel( xmax, ymax, colMax, rowMin );
                std::vector<float> buffer(tileSize * tileSize, NO_DATA_VALUE);

                int iColMin = floor(colMin);
                int iColMax = ceil(colMax);
                int iRowMin = floor(rowMin);
                int iRowMax = ceil(rowMax);
                int iNumCols = iColMax - iColMin + 1;
                int iNumRows = iRowMax - iRowMin + 1;

                int iWinColMin = max(0, iColMin);
                int iWinColMax = min(_warpedDS->GetRasterXSize()-1, iColMax);
                int iWinRowMin = max(0, iRowMin);
                int iWinRowMax = min(_warpedDS->GetRasterYSize()-1, iRowMax);
                int iNumWinCols = iWinColMax - iWinColMin + 1;
                int iNumWinRows = iWinRowMax - iWinRowMin + 1;

                int iBufColMin = osg::round((iWinColMin - iColMin) / double(iNumCols - 1) * (tileSize - 1));
                int iBufColMax = osg::round((iWinColMax - iColMin) / double(iNumCols - 1) * (tileSize - 1));
                int iBufRowMin = osg::round((iWinRowMin - iRowMin) / double(iNumRows - 1) * (tileSize - 1));
                int iBufRowMax = osg::round((iWinRowMax - iRowMin) / double(iNumRows - 1) * (tileSize - 1));
                int iNumBufCols = iBufColMax - iBufColMin + 1;
                int iNumBufRows = iBufRowMax - iBufRowMin + 1;

                int startOffset = iBufRowMin * tileSize + iBufColMin;
                int lineSpace = tileSize * sizeof(float);

                band->RasterIO(GF_Read, iWinColMin, iWinRowMin, iNumWinCols, iNumWinRows, &buffer[startOffset], iNumBufCols, iNumBufRows, GDT_Float32, 0, lineSpace);

                for (int r = 0, ir = tileSize - 1; r < tileSize; ++r, --ir)
                {
                    for (int c = 0; c < tileSize; ++c)
                    {
                        hf->setHeight(c, ir, buffer[r * tileSize + c]);
                    }
                }
            }
            else
            {
                double dx = (xmax - xmin) / (tileSize-1);
                double dy = (ymax - ymin) / (tileSize-1);
                for (int r = 0; r < tileSize; ++r)
                {
                    double geoY = ymin + (dy * (double)r);
                    for (int c = 0; c < tileSize; ++c)
                    {
                        double geoX = xmin + (dx * (double)c);
                        float h = getInterpolatedValue(band, geoX, geoY);
                        hf->setHeight(c, r, h);
                    }
                }
            }
        }
        else
        {
            std::vector<float>& heightList = hf->getHeightList();
            std::fill(heightList.begin(), heightList.end(), NO_DATA_VALUE);
        }
        return hf.release();
    }

#else

    /**
     * Specialized version of GeoHeightField's getHeightAtLocation that just clamps values that are outside of the dataset
     * to be within the dataset (logic in HeightFieldUtils::getHeightAtLocation).
     * This is necessary when sampling datasets along the edges where data might actually not exist.
     * For example, take a worldwide elevation dataset with bounds -180, -90 to 180, 90 that is 200x100 pixels.
     * When you go to sample the western hemisphere you end up reading a heightfield of size 100x100.  The bounds of the actual data that was
     * read in heightfield form are actually 0.5,0.5 to 99.5, 99.5 b/c the elevation sample point is in the center of the pixels, not the entire pixel.
     * So the loop that attempts to resample the heightfield might be asking for elevation values at -180,-90 which is in pixel space 0,0.
     * In this version of the getHeightAtLocation function it will just return the value at 0.5, 0.5.
     */
    float getHeightAtLocation(const GeoHeightField& hf, double x, double y, ElevationInterpolation interp)
    {
        double xInterval = hf.getExtent().width()  / (double)(hf.getHeightField()->getNumColumns()-1);
        double yInterval = hf.getExtent().height() / (double)(hf.getHeightField()->getNumRows()-1);

        // sample the heightfield at the input coordinates:
        // (note: since it's sampling the HF, it will return an MSL height if applicable)
        float height = HeightFieldUtils::getHeightAtLocation(
            hf.getHeightField(),
            x, y,
            hf.getExtent().xMin(), hf.getExtent().yMin(),
            xInterval, yInterval,
            interp);

        return height;
    }

     osg::HeightField* createHeightField( const TileKey&        key,
                                         ProgressCallback*     progress)
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
        for (unsigned int i = 0; i < hf->getHeightList().size(); ++i) hf->getHeightList()[i] = NO_DATA_VALUE;

        if (intersects(key))
        {
            //Get the extents of the tile
            double xmin, ymin, xmax, ymax;
            key.getExtent().getBounds(xmin, ymin, xmax, ymax);

            // Compute the intersection of the incoming key with the data extents of the dataset
            osgEarth::GeoExtent intersection = key.getExtent().intersectionSameSRS( _extents );

            // Determine the read window
            double src_min_x, src_min_y, src_max_x, src_max_y;
            // Get the pixel coordinates of the intersection
            geoToPixel( intersection.xMin(), intersection.yMax(), src_min_x, src_min_y);
            geoToPixel( intersection.xMax(), intersection.yMin(), src_max_x, src_max_y);

            int rasterWidth = _warpedDS->GetRasterXSize();
            int rasterHeight = _warpedDS->GetRasterYSize();

            // Convert the doubles to integers.  We floor the mins and ceil the maximums to give the widest window possible.
            src_min_x = osg::round(src_min_x);
            src_min_y = osg::round(src_min_y);
            src_max_x = osg::round(src_max_x);
            src_max_y = osg::round(src_max_y);

            // We are now dealing with integer pixel values, so need to add 1 to get the width
            int width  = (int)(src_max_x - src_min_x) + 1;
            int height = (int)(src_max_y - src_min_y) + 1;

            // Don't read anything greater than a dimension of max_read_dimensions.  If the source window is really large it will use
            // GDAL's nearest neighbour sampling.  If the source window is < max_read_dimensions x max_read_dimensions then the exact source data is read and
            // resampled.  This make sure that once get into high enough resolution data the verts don't move around on you due to sampling.
            int max_read_dimensions = 256;

            // Width of the GDAL target buffer. Will be modified later.
            int target_width;
            int target_height;

            // If the source window is large, then just read a sample from it using GDALs nearest neighbour algorithm. We will then sample from the result.
            if(width > max_read_dimensions)
            {
                target_width = max_read_dimensions;
                // Figure out how many source pixels equate to half a read buffer cell.
                double dx = ((double)width) / ((double)(target_width - 1)) * 0.5;
                //Inflate the source read window by half a target buffer cell. There are two reasons for this:
                // 1) Later we will deflate our read window by the same amount, so if we don't do this here there will be an apparent gap between tiles.
                // 2) GDAL will start reading half way along the first target buffer cell and finish reading half way along the last.
                //     We want the the nearest neighbour to that point to be right on the tile boundary so that the boundary values are the same on neighbouring tiles.
                int x0 = (int)floor(src_min_x - dx);
                int x1 = (int)ceil(src_max_x + dx);

                bool limitx0 = false;
                bool limitx1 = false;

                // Check if the recalculated values are valid, or if they need to be limited to the edge of the data.
                if( x0 < 0 )
                {
                    x0 = 0;
                    limitx0 = true;
                }

                if( x1 > rasterWidth - 1 )
                {
                    x1 = rasterWidth - 1;
                    limitx1 = true;
                }

                // If one of the recalculated values was limited, then we need to recalculate the other so that half a cell offset will land on the edge of the grid.
                if( limitx0 && !limitx1 )
                {
                    // Recalculate x1
                    double dx_rev = (src_max_x - x0) / (((double)target_width) - 0.5);
                    x1 = (int)ceil(src_max_x + dx_rev);
                }

                if( limitx1 && !limitx0 )
                {
                    // Recalculate x0
                    double dx_rev = (x1 - src_min_x) / (((double)target_width) - 0.5);
                    x0 = (int)floor(src_min_x - dx_rev);
                }

                src_min_x = x0;
                src_max_x = x1;

                // Need to recalc width.
                width = (int)(src_max_x - src_min_x) + 1;
            }
            else
            {
                // Inflate the source window by one cell. Source pixels are read exactly, so don't need to worry about GDAL sub sampling.
                src_min_x = osg::maximum((int)src_min_x - 1, 0);
                src_max_x = osg::minimum((int)src_max_x + 1, rasterWidth - 1);
                // Need to recalc width.
                width = (int)(src_max_x - src_min_x) + 1;
                target_width = width;
            }

            if(height > max_read_dimensions)
            {
                target_height = max_read_dimensions;
                // Figure out how many source pixels equate to half a read buffer cell.
                double dy =  ((double)height) / ((double)(target_height - 1)) * 0.5;
                //Inflate the source read window by half a target buffer cell. There are two reasons for this:
                // 1) Later we will deflate our read window by the same amount, so if we don't do this here there will be an apparent gap between tiles.
                // 2) GDAL will start reading half way along the first target buffer cell and finish reading half way along the last.
                //     We want the the nearest neighbour to that point to be right on the tile boundary so that the boundary values are the same on neighbouring tiles.
                int y0 = (int)floor(src_min_y - dy);
                int y1 = (int)ceil(src_max_y + dy);

                bool limity0 = false;
                bool limity1 = false;

                // Check if the recalculated values are valid, or if they need to be limited to the edge of the data.
                if( y0 < 0 )
                {
                    y0 = 0;
                    limity0 = true;
                }

                if( y1 > rasterHeight - 1 )
                {
                    y1 = rasterHeight - 1;
                    limity1 = true;
                }

                // If one of the recalculated values was limited, then we need to recalculate the other so that half a cell offset will land on the edge of the grid.
                if( limity0 && !limity1 )
                {
                    // Recalculate y1
                    double dy_rev = (src_max_y - y0) / (((double)target_height) - 0.5);
                    y1 = (int)ceil(src_max_y + dy_rev);
                }

                if( limity1 && !limity0 )
                {
                    // Recalculate y0
                    double dy_rev = (y1 - src_min_y) / (((double)target_height) - 0.5);
                    y0 = (int)floor(src_min_y - dy_rev);
                }

                src_min_y = y0;
                src_max_y = y1;

                // Need to recalc height.
                height = (int)(src_max_y - src_min_y) + 1;
            }
            else
            {
                // Inflate the source window by one cell. Source pixels are read exactly, so don't need to worry about GDAL sub sampling.
                src_min_y = osg::maximum((int)src_min_y - 1, 0);
                src_max_y = osg::minimum((int)src_max_y + 1, rasterHeight - 1);
                // Need to recalc height.
                height = (int)(src_max_y - src_min_y) + 1;
                target_height = height;
            }

            OE_DEBUG << LC << "Reading key " << key.str() << "   " << xmin << ", " << ymin << ", " << xmax << ", " << ymax << ", " << std::endl;
            OE_DEBUG << LC << "ReadWindow " << src_min_x << "," << src_min_y  << "," << src_max_x << "," << src_max_y << " " << width << "x" << height << std::endl;
            OE_DEBUG << LC << "DestWindowSize " << target_width << "x" << target_height << std::endl;

            // Figure out the true pixel extents of what we read
            double read_min_x, read_min_y, read_max_x, read_max_y;
            pixelToGeo(src_min_x, src_min_y, read_min_x, read_max_y);
            // True extents extends to the far side of the last pixel and line.
            pixelToGeo(src_max_x + 1, src_max_y + 1, read_max_x, read_min_y);

            // We need to deflate the size of the extents by the width of 0.5 'target buffer' pixel to get the correct extents of the heightfield since it's
            // sampled at the center of the pixels and not the outside edges.
            double half_dx = ((read_max_x - read_min_x)/((double)target_width)) / 2.0;
            double half_dy = ((read_max_y - read_min_y)/((double)target_height)) / 2.0;
            read_min_x += half_dx;
            read_min_y += half_dy;
            read_max_x -= half_dx;
            read_max_y -= half_dy;


            OE_DEBUG << LC << "Read extents " << read_min_x << ", " << read_min_y << " to " << read_max_x << ", " << read_max_y << std::endl;

            // Try to find a FLOAT band
            GDALRasterBand* band = findBandByDataType(_warpedDS, GDT_Float32);
            if (band == NULL)
            {
                // Just get first band
                band = _warpedDS->GetRasterBand(1);
            }

            float *heights = new float[target_width * target_height];
            for (unsigned int i = 0; i < target_width * target_height; i++)
            {
                heights[i] = NO_DATA_VALUE;
            }
            band->RasterIO(GF_Read, src_min_x, src_min_y, width, height, heights, target_width, target_height, GDT_Float32, 0, 0);

            // Now create a GeoHeightField that we can sample from.  This heightfield only contains the portion that was actually read from the dataset
            osg::ref_ptr< osg::HeightField > readHF = new osg::HeightField();
            readHF->allocate( target_width, target_height );
            for (unsigned int c = 0; c < target_width; c++)
            {
                for (unsigned int r = 0; r < target_height; r++)
                {
                    unsigned inv_r = target_height - r -1;
                    float h = heights[r * target_width + c];
                    // Mark the value as nodata using the universal NO_DATA_VALUE marker.
                    if (!isValidValue( h, band ) )
                    {
                        h = NO_DATA_VALUE;
                    }

                    readHF->setHeight(c, inv_r, h );
                }
            }

            // Delete the heights array, it's been copied into readHF.
            delete[] heights;


            // Create a GeoHeightField so we can easily sample it.
            GeoHeightField readGeoHeightField(readHF, GeoExtent(this->getProfile()->getSRS(), read_min_x, read_min_y, read_max_x, read_max_y));


            // Iterate over the output heightfield and sample the data that was read into it.
            double dx = (xmax - xmin) / (tileSize-1);
            double dy = (ymax - ymin) / (tileSize-1);

            for (int c = 0; c < tileSize; ++c)
            {
                double geoX = xmin + (dx * (double)c);
                for (int r = 0; r < tileSize; ++r)
                {
                    double geoY = ymin + (dy * (double)r);

                    float h = NO_DATA_VALUE;
                    if (readGeoHeightField.getExtent().contains(geoX, geoY))
                    {
                        h = getHeightAtLocation( readGeoHeightField, geoX, geoY, *_options.interpolation() );
                    }
                    hf->setHeight(c, r, h);
                }
            }
        }
        return hf.release();
    }

#endif



    bool intersects(const TileKey& key)
    {
        return key.getExtent().intersects( _extents );
    }


private:

    GDALDataset* _srcDS;
    GDALDataset* _warpedDS;
    double       _geotransform[6];
    double       _invtransform[6];

    GeoExtent _extents;
    Bounds _bounds;

    const GDALOptions _options;

    osg::ref_ptr< CacheBin > _cacheBin;
    osg::ref_ptr< osgDB::Options > _dbOptions;

    unsigned int _maxDataLevel;
};


class ReaderWriterGDALTile : public TileSourceDriver
{
public:
    ReaderWriterGDALTile() {}

    virtual const char* className() const
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
