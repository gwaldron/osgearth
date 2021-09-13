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
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */
#include <osgEarth/GDALDEM>
#include <osgEarth/Progress>
#include <osgEarth/ImageUtils>
#include <osgEarth/FileUtils>
#include <osgEarth/Registry>

#include <gdal_priv.h> // C++ API

#if GDAL_VERSION_NUM > 2004000
#define HAS_GDALDEM
#endif

#ifdef HAS_GDALDEM
#include <gdal_utils.h>
#endif


using namespace osgEarth;

#undef  LC
#define LC "[GDALDEM] "

//........................................................................

Config
GDALDEMLayer::Options::getConfig() const
{
    Config conf = ImageLayer::Options::getConfig();
    elevationLayer().set(conf, "layer");
    conf.set("processing", processing());
    conf.set("light_altitude", lightAltitude());
    conf.set("azimuth", azimuth());
    conf.set("combined", combined());
    conf.set("multidirectional", multidirectional());
    conf.set("alpha", alpha());
    conf.set("color_filename", color_filename());
    return conf;
}

void
GDALDEMLayer::Options::fromConfig(const Config& conf)
{
    processing().init("hillshade");
    elevationLayer().get(conf, "layer");
    conf.get("processing", processing());
    conf.get("light_altitude", lightAltitude());
    conf.get("azimuth", azimuth());
    conf.get("combined", combined());
    conf.get("multidirectional", multidirectional());
    conf.get("alpha", alpha());
    conf.get("color_filename", color_filename());
}

REGISTER_OSGEARTH_LAYER(GDALDEM, GDALDEMLayer);

void
GDALDEMLayer::setElevationLayer(ElevationLayer* elevationLayer)
{
    if (getElevationLayer() != elevationLayer)
    {
        bool wasOpen = isOpen();
        if (wasOpen) close();
        options().elevationLayer().setLayer(elevationLayer);
        if (wasOpen) open();
    }
}

ElevationLayer*
GDALDEMLayer::getElevationLayer() const
{
    return options().elevationLayer().getLayer();
}

void
GDALDEMLayer::setProcessing(const std::string& value)
{
    setOptionThatRequiresReopen(options().processing(), value);
}

const std::string&
GDALDEMLayer::getProcessing() const
{
    return options().processing().get();
}

void
GDALDEMLayer::setAzimuth(const float& value)
{
    setOptionThatRequiresReopen(options().azimuth(), value);
}

float
GDALDEMLayer::getAzimuth() const
{
    return options().azimuth().get();
}

void
GDALDEMLayer::setLightAltitude(const float& value)
{
    setOptionThatRequiresReopen(options().lightAltitude(), value);
}

float
GDALDEMLayer::getLightAltitude() const
{
    return options().lightAltitude().get();
}

void
GDALDEMLayer::setMultidirectional(const bool& value)
{
    setOptionThatRequiresReopen(options().multidirectional(), value);
}

bool
GDALDEMLayer::getMultidirectional() const
{
    return options().multidirectional().get();
}

void
GDALDEMLayer::setCombined(const bool& value)
{
    setOptionThatRequiresReopen(options().combined(), value);
}

bool
GDALDEMLayer::getCombined() const
{
    return options().combined().get();
}

void
GDALDEMLayer::setAlpha(const bool& value)
{
    setOptionThatRequiresReopen(options().alpha(), value);
}

bool
GDALDEMLayer::getAlpha() const
{
    return options().alpha().get();
}

void
GDALDEMLayer::setColorFilename(const URI& value)
{
  setOptionThatRequiresReopen(options().color_filename(), value);
}

const URI&
GDALDEMLayer::getColorFilename() const
{
  return options().color_filename().get();
}

void
GDALDEMLayer::init()
{
    ImageLayer::init();
}

void
GDALDEMLayer::addedToMap(const Map* map)
{
    options().elevationLayer().addedToMap(map);

    ElevationLayer* layer = options().elevationLayer().getLayer();
    if (!layer)
    {
        setStatus(Status::Error(Stringify() << "Failed to get elevaton layer"));
        return;
    }

    setProfile(layer->getProfile());
    setDataExtents(layer->getDataExtents());
}

void
GDALDEMLayer::removedFromMap(const Map* map)
{
    options().elevationLayer().removedFromMap(map);
}

Status
GDALDEMLayer::openImplementation()
{
    Status parent = ImageLayer::openImplementation();
    if (parent.isError())
        return parent;

    // If we're in cache-only mode, do not attempt to open the layer!
    if (getCacheSettings()->cachePolicy()->isCacheOnly())
        return Status::NoError;

    Status childStatus = options().elevationLayer().open(getReadOptions());
    if (childStatus.isError())
        return childStatus;

#ifdef HAS_GDALDEM

    //ElevationLayer* layer = options().elevationLayer().getLayer();
    /*
    if (!layer)
        return Status::ServiceUnavailable;
    setProfile(layer->getProfile());
    setDataExtents(layer->getDataExtents());
    */

    /*
    const Profile* profile = getProfile();
    if (!profile)
    {
        profile = Profile::create(Profile::GLOBAL_GEODETIC);
        setProfile(profile);
    }
    */

    return Status::NoError;

#else

    return Status(Status::AssertionFailure, "GDAL 2.4+ required");

#endif
}

Status
GDALDEMLayer::closeImplementation()
{
    getElevationLayer()->close();
    return Status::OK();
}

namespace
{
    GDALDataset*
        createMemDS(int width, int height, int numBands, GDALDataType dataType, double minX, double minY, double maxX, double maxY, const std::string &projection)
    {
        //Get the MEM driver
        GDALDriver* memDriver = (GDALDriver*)GDALGetDriverByName("MEM");
        if (!memDriver)
        {
            OE_NOTICE << "[osgEarth::GeoData] Could not get MEM driver" << std::endl;
        }

        //Create the in memory dataset.
        GDALDataset* ds = memDriver->Create("", width, height, numBands, dataType, 0);

        //Initialize the color interpretation
        if (numBands == 1)
        {
            ds->GetRasterBand(1)->SetColorInterpretation(GCI_GrayIndex);
        }
        else
        {
            ds->GetRasterBand(1)->SetColorInterpretation(GCI_RedBand);
            ds->GetRasterBand(2)->SetColorInterpretation(GCI_GreenBand);
            ds->GetRasterBand(3)->SetColorInterpretation(GCI_BlueBand);

            if (numBands == 4)
            {
                ds->GetRasterBand(4)->SetColorInterpretation(GCI_AlphaBand);
            }
        }

        //Initialize the geotransform
        double geotransform[6];
        double x_units_per_pixel = (maxX - minX) / (double)width;
        double y_units_per_pixel = (maxY - minY) / (double)height;
        geotransform[0] = minX;
        geotransform[1] = x_units_per_pixel;
        geotransform[2] = 0;
        geotransform[3] = maxY;
        geotransform[4] = 0;
        geotransform[5] = -y_units_per_pixel;
        ds->SetGeoTransform(geotransform);
        ds->SetProjection(projection.c_str());

        return ds;
    }

    osg::Image*
        createImageFromDataset(GDALDataset* ds)
    {
        // called internally -- GDAL lock not required

        int numBands = ds->GetRasterCount();
        if (numBands < 1)
            return 0L;

        GLenum dataType;
        int    sampleSize;
        GLint  internalFormat;

        switch (ds->GetRasterBand(1)->GetRasterDataType())
        {
        case GDT_Byte:
            dataType = GL_UNSIGNED_BYTE;
            sampleSize = 1;
            //internalFormat = GL_LUMINANCE8;
            internalFormat = GL_R8;
            break;
        case GDT_UInt16:
        case GDT_Int16:
            dataType = GL_UNSIGNED_SHORT;
            sampleSize = 2;
            //internalFormat = GL_LUMINANCE16;
            internalFormat = GL_R16;
            break;
        default:
            dataType = GL_FLOAT;
            sampleSize = 4;
            //internalFormat = GL_LUMINANCE32F_ARB;
            internalFormat = GL_R32F;
        }

        GLenum pixelFormat =
            numBands == 1 ? GL_RED :
            numBands == 2 ? GL_RG :
            numBands == 3 ? GL_RGB :
            GL_RGBA;

        int pixelBytes = sampleSize * numBands;

        //Allocate the image
        osg::Image *image = new osg::Image;
        image->allocateImage(ds->GetRasterXSize(), ds->GetRasterYSize(), 1, pixelFormat, dataType);

        CPLErr err = ds->RasterIO(
            GF_Read,
            0, 0,
            image->s(), image->t(),
            (void*)image->data(),
            image->s(), image->t(),
            ds->GetRasterBand(1)->GetRasterDataType(),
            numBands,
            NULL,
            pixelBytes,
            pixelBytes * image->s(),
            1);
        if (err != CE_None)
        {
            OE_WARN << LC << "RasterIO failed.\n";
        }

        return image;
    }

    GDALDataset*
    createDataSetFromHeightField(const osg::HeightField* hf, double minX, double minY, double maxX, double maxY, const std::string &projection)
    {
        GDALDataType gdalDataType = GDT_Float32;

        int numBands = 1;

        int pixelBytes = sizeof(float) * numBands;

        GDALDataset* srcDS = createMemDS(hf->getNumColumns(), hf->getNumRows(), numBands, gdalDataType, minX, minY, maxX, maxY, projection);

        if (srcDS)
        {
            CPLErr err = srcDS->RasterIO(
                GF_Write,
                0, 0,
                hf->getNumColumns(), hf->getNumRows(),
                (void*)hf->getFloatArray()->getDataPointer(),
                hf->getNumColumns(),
                hf->getNumRows(),
                gdalDataType,
                numBands,
                NULL,
                pixelBytes,
                pixelBytes * hf->getNumColumns(),
                1);

            if (err != CE_None)
            {
                OE_WARN << LC << "RasterIO failed.\n";
            }

            srcDS->FlushCache();
        }

        return srcDS;
    }
}

GeoImage
GDALDEMLayer::createImageImplementation(const TileKey& key, ProgressCallback* progress) const
{
#ifdef HAS_GDALDEM
    //GDAL_SCOPED_LOCK;

    ElevationLayer* layer = getElevationLayer();
    GeoHeightField heightField = layer->createHeightField(key, progress);
    if (heightField.valid())
    {
        osg::ref_ptr< osg::Image > image = 0;
        const osg::HeightField* hf = heightField.getHeightField();

        std::string tmpPath = getTempName(getTempPath(), ".tif");

        GDALDataset* srcDS = createDataSetFromHeightField(hf, key.getExtent().xMin(), key.getExtent().yMin(), key.getExtent().xMax(), key.getExtent().yMax(), key.getExtent().getSRS()->getWKT());
        int error = 0;
        std::string processing = options().processing().get();
        std::string color_filename = options().color_filename()->full();
        char **papsz = NULL;
        papsz = CSLAddString(papsz, "-compute_edges");

        if (options().azimuth().isSet())
        {
            papsz = CSLAddString(papsz, "-az");
            std::string arg = Stringify() << *options().azimuth();
            papsz = CSLAddString(papsz, arg.c_str());
        }

        if (options().lightAltitude().isSet())
        {
            papsz = CSLAddString(papsz, "-alt");
            std::string arg = Stringify() << *options().lightAltitude();
            papsz = CSLAddString(papsz, arg.c_str());
        }

        if (options().multidirectional().isSet())
        {
            papsz = CSLAddString(papsz, "-multidirectional");
        }

        if (options().combined().isSet())
        {
            papsz = CSLAddString(papsz, "-combined");
        }

        if (options().alpha().isSet())
        {
            papsz = CSLAddString(papsz, "-alpha");
        }

        GDALDEMProcessingOptions* psOptions = GDALDEMProcessingOptionsNew(papsz, NULL);

        const char* pszColorFilename = NULL;
        if (!color_filename.empty())
        {
            pszColorFilename = color_filename.c_str();
        }

        GDALDatasetH outputDS = GDALDEMProcessing(tmpPath.c_str(), srcDS, processing.c_str(), pszColorFilename, psOptions, &error);
        if (outputDS)
        {
            image = createImageFromDataset((GDALDataset*)outputDS);
            GDALClose(outputDS);
        }
        remove(tmpPath.c_str());
        delete srcDS;
        GDALDEMProcessingOptionsFree(psOptions);
        CSLDestroy(papsz);

        if (image.valid())
        {
            return GeoImage(image.get() , key.getExtent());
        }
    }

#endif // HAS_GDALDEM

    return GeoImage::INVALID;
}