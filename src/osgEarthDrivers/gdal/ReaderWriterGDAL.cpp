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

#include <osgEarth/TileSource>
#include <osgEarth/FileUtils>
#include <osgEarth/Registry>
#include <osgEarth/ImageUtils>
#include <osgEarth/URI>

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

        Cache* cache = 0;

        _dbOptions = Registry::instance()->cloneOrCreateOptions( dbOptions );

        if ( _dbOptions.valid() )
        {
            // Set up a Custom caching bin for this TileSource
            cache = Cache::get( _dbOptions.get() );
            if ( cache )
            {
                Config optionsConf = _options.getConfig();

                std::string binId = Stringify() << std::hex << hashString(optionsConf.toJSON());                
                _cacheBin = cache->addBin( binId );

                if ( _cacheBin.valid() )
                {
                    _cacheBin->apply( _dbOptions.get() );
                }
            }
        }  

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
            return Status::Error( "No URL, directory, or connection string specified" );
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

                OE_INFO << LC << "Driver found " << files.size() << " files:" << std::endl;
                for (unsigned int i = 0; i < files.size(); ++i)
                {
                    OE_INFO << LC << "" << files[i] << std::endl;
                }
            }
            else
            {
                // just add the connection string as the single source.
                files.push_back( source );
            }

            if (files.empty())
            {
                return Status::Error( "Could not find any valid input." );
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
                    ReadResult result = _cacheBin->readString( vrtKey, 0 );
                    if (result.succeeded())
                    {                        
                        _srcDS = (GDALDataset*)GDALOpen(result.getString().c_str(), GA_ReadOnly );
                        if (_srcDS)
                        {
                            OE_INFO << LC << "Read VRT from cache!" << std::endl;
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
                    OE_INFO << LC << "Built VRT in " << osg::Timer::instance()->delta_s(startTime, endTime) << " s" << std::endl;

                    if (_srcDS)
                    {
                        //Cache the VRT so we don't have to build it next time.
                        if (_cacheBin)
                        {
                            std::string vrtFile = getTempName( "", ".vrt");
                            OE_INFO << "Writing temp VRT to " << vrtFile << std::endl;
                         
                            if (vrtDriver)
                            {                    
                                vrtDriver->CreateCopy(vrtFile.c_str(), _srcDS, 0, 0, 0, 0 );                                                        


                                //We created the temp file, now read the contents back                            
                                std::ifstream input( vrtFile.c_str() );
                                if ( input.is_open() )
                                {
                                    input >> std::noskipws;
                                    std::stringstream buf;
                                    buf << input.rdbuf();                                
                                    std::string vrtContents = buf.str();                                
                                    osg::ref_ptr< StringObject > strObject = new StringObject( vrtContents );
                                    _cacheBin->write( vrtKey, strObject.get() );
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
                    return Status::Error( Stringify() << "Failed to open dataset " << files[0] );
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
            OE_NOTICE << "Created warp profile " << warpProfile->toString() <<  std::endl;
        }




        //Create a spatial reference for the source.
        std::string srcProj = _srcDS->GetProjectionRef();

        
        if ( !srcProj.empty() && getProfile() != 0L )
        {
            OE_WARN << LC << "WARNING, overriding profile of a source that already defines its own SRS (" 
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
                return Status::Error( Stringify()
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

        if ( warpProfile )
        {
            profile = warpProfile;
        }

        // If we have an override profile, just take it.
        if ( getProfile() )
        {
            profile = getProfile();
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

        if ( requiresReprojection || (profile && !profile->getSRS()->isEquivalentTo( src_srs.get() )) )
        {
            std::string destWKT = profile ? profile->getSRS()->getWKT() : src_srs->getWKT();

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

        //Get the _geotransform
        if ( getProfile() )
        {
            _geotransform[0] =  getProfile()->getExtent().xMin(); //Top left x
            _geotransform[1] =  getProfile()->getExtent().width() / (double)_warpedDS->GetRasterXSize();//pixel width
            _geotransform[2] =  0;

            _geotransform[3] =  getProfile()->getExtent().yMax(); //Top left y
            _geotransform[4] =  0;
            _geotransform[5] = -getProfile()->getExtent().height() / (double)_warpedDS->GetRasterYSize();//pixel height

        }
        else
        {
            _warpedDS->GetGeoTransform(_geotransform);
        }

        GDALInvGeoTransform(_geotransform, _invtransform);

        double minX, minY, maxX, maxY;


        //Compute the extents
        // polar needs a special case when combined with geographic
        if ( profile && profile->getSRS()->isGeographic() && (src_srs->isNorthPolar() || src_srs->isSouthPolar()) )
        {
            double ll_lon, ll_lat, ul_lon, ul_lat, ur_lon, ur_lat, lr_lon, lr_lat;

            pixelToGeo(0.0, 0.0, ul_lon, ul_lat );
            pixelToGeo(0.0, _warpedDS->GetRasterYSize(), ll_lon, ll_lat);
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

        OE_DEBUG << LC << "Geo extents: " << minX << ", " << minY << " -> " << maxX << ", " << maxY << std::endl;

        if ( !profile )
        {
            profile = Profile::create( 
                warpedSRSWKT,
                minX, minY, maxX, maxY);

            OE_INFO << LC << "" << source << " is projected, SRS = " 
                << warpedSRSWKT << std::endl;
                //<< _warpedDS->GetProjectionRef() << std::endl;
        }

        //Compute the min and max data levels
        double resolutionX = (maxX - minX) / (double)_warpedDS->GetRasterXSize();
        double resolutionY = (maxY - minY) / (double)_warpedDS->GetRasterYSize();

        double maxResolution = osg::minimum(resolutionX, resolutionY);

        OE_INFO << LC << "Resolution= " << resolutionX << "x" << resolutionY << " max=" << maxResolution << std::endl;

        if (_options.maxDataLevel().isSet())
        {
            _maxDataLevel = _options.maxDataLevel().value();
            OE_INFO << _options.url().value().full() << " using override max data level " << _maxDataLevel << std::endl;
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

            OE_INFO << LC << _options.url().value().full() << " max Data Level: " << _maxDataLevel << std::endl;
        }

        osg::ref_ptr< SpatialReference > srs = SpatialReference::create( warpedSRSWKT );
        // record the data extent in profile space:
        _extents = GeoExtent( srs, minX, minY, maxX, maxY);
        GeoExtent profile_extent = _extents.transform( profile->getSRS() );

        getDataExtents().push_back( DataExtent(profile_extent, 0, _maxDataLevel) );

        //Set the profile
        setProfile( profile );

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

                    R = Hue_2_RGB( var_1, var_2, H + ( 1 / 3 ) );
                    G = Hue_2_RGB( var_1, var_2, H );
                    B = Hue_2_RGB( var_1, var_2, H - ( 1 / 3 ) );                                
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
        if (intersects(key)) //TODO: I think this test is OBE -gw
        {
            //Get the extents of the tile
            double xmin, ymin, xmax, ymax;
            key.getExtent().getBounds(xmin, ymin, xmax, ymax);

            // Compute the intersection of the incoming key with the data extents of the dataset
            osgEarth::GeoExtent intersection = key.getExtent().intersectionSameSRS( _extents );
            
            // Determine the read window
            double src_min_x, src_min_y, src_max_x, src_max_y;
            // Get the pixel coordiantes of the intersection
            geoToPixel( intersection.xMin(), intersection.yMax(), src_min_x, src_min_y);
            geoToPixel( intersection.xMax(), intersection.yMin(), src_max_x, src_max_y);   

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
                    for (int c = 0; c < tileSize; ++c) 
                    { 
                        double geoX = xmin + (dx * (double)c); 

                        for (int r = 0; r < tileSize; ++r) 
                        { 
                            double geoY   = ymin + (dy * (double)r); 
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
            else if (bandPalette)
            {
                //Pallete indexed imagery doesn't support interpolation currently and only uses nearest
                //b/c interpolating pallete indexes doesn't make sense.
                unsigned char *palette = new unsigned char[target_width * target_height];

                image = new osg::Image;
                image->allocateImage(tileSize, tileSize, 1, pixelFormat, GL_UNSIGNED_BYTE);
                memset(image->data(), 0, image->getImageSizeInBytes());

                bandPalette->RasterIO(GF_Read, off_x, off_y, width, height, palette, target_width, target_height, GDT_Byte, 0, 0);

                for (int src_row = 0, dst_row = tile_offset_top;
                    src_row < target_height;
                    src_row++, dst_row++)
                {
                    for (int src_col = 0, dst_col = tile_offset_left;
                        src_col < target_width;
                        ++src_col, ++dst_col)
                    {
                        
                        unsigned char p = palette[src_col + src_row * target_width];
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
        }
        else
        {
            OE_NOTICE << LC << key.str() << " does not intersect " << _options.url()->full() << std::endl;
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
        return key.getExtent().intersects( _extents );
    }


private:

    GDALDataset* _srcDS;
    GDALDataset* _warpedDS;
    double       _geotransform[6];
    double       _invtransform[6];

    GeoExtent _extents;

    const GDALOptions _options;

    osg::ref_ptr< CacheBin > _cacheBin;
    osg::ref_ptr< osgDB::Options > _dbOptions;

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
