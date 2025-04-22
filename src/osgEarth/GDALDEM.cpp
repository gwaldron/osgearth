/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include <osgEarth/GDALDEM>
#include <osgEarth/Progress>
#include <osgEarth/ImageUtils>
#include <osgEarth/Registry>
#include <osgDB/FileUtils>

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
    super::init();

    // disable caching by default since this is a derived layer
    layerHints().cachePolicy() = CachePolicy::NO_CACHE;
}

void
GDALDEMLayer::addedToMap(const Map* map)
{
    _map = map;
    setProfile(map->getProfile());
    super::addedToMap(map);
}

void
GDALDEMLayer::removedFromMap(const Map* map)
{
    _map = nullptr;

    super::removedFromMap(map);
}

Status
GDALDEMLayer::openImplementation()
{
    Status parent = ImageLayer::openImplementation();
    if (parent.isError())
        return parent;

    // if the colorramp file doesn't exist, create a default one.
    if (!osgDB::fileExists(options().color_filename()->full()))
    {
        const char* default_color_ramp = R"(
5000 220 220 220
4000 212 207 204
3000 212 193 179
2000 212 184 163
1000 212 201 180
600 169 192 166
200 134 184 159
50 120 172 149
1 114 164 141
0 66 135 245
-32768 0 0 255
)";
        _colorRampFilename = "/vsimem/osgearth_gdaldem_color_ramp.txt";
        auto handle = VSIFOpenL(_colorRampFilename.c_str(), "wb");
        VSIFWriteL(default_color_ramp, 1, strlen(default_color_ramp), handle);
        VSIFCloseL(handle);
    }
    else
    {
        _colorRampFilename = options().color_filename()->full();
    }

    return STATUS_OK;
}

Status
GDALDEMLayer::closeImplementation()
{
    options().elevationLayer().close();
    _elevationLayer = nullptr;
    _map = nullptr;
    return STATUS_OK;
}

namespace
{

    GDALDataset*
        createMemDS(int width, int height, int numBands, GDALDataType dataType, double minX, double minY, double maxX, double maxY, const std::string& projection)
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

    osg::Image* createImageFromDataset(GDALDataset* ds)
    {
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
        osg::ref_ptr<osg::Image> image = new osg::Image;
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

        // Convert the image to rgba8
        return ImageUtils::convertToRGBA8(image.get());
    }

    GDALDataset* createDataSetFromHeightField(const osg::HeightField* hf, double minX, double minY, double maxX, double maxY, const std::string& projection)
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

    osg::ref_ptr<const Map> map;
    if (!_map.lock(map))
        return {};

    osg::ref_ptr<ElevationTexture> tile;
    if (!map->getElevationPool()->getTile(key, false, tile, nullptr, progress))
        return {};

    osg::ref_ptr<const osg::HeightField> hf = tile->getHeightField();
    {
        osg::ref_ptr< osg::Image > image = 0;

        GDALDataset* srcDS = createDataSetFromHeightField(hf.get(), key.getExtent().xMin(), key.getExtent().yMin(), key.getExtent().xMax(), key.getExtent().yMax(), key.getExtent().getSRS()->getWKT());
        int error = 0;
        std::string processing = options().processing().get();
        char** papsz = NULL;
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
            std::string arg = std::to_string(options().lightAltitude().value());
            papsz = CSLAddString(papsz, arg.c_str());
        }

        if (options().multidirectional() == true)
        {
            papsz = CSLAddString(papsz, "-multidirectional");
        }

        if (options().combined() == true)
        {
            papsz = CSLAddString(papsz, "-combined");
        }

        if (options().alpha() == true)
        {
            papsz = CSLAddString(papsz, "-alpha");
        }

        GDALDEMProcessingOptions* psOptions = GDALDEMProcessingOptionsNew(papsz, NULL);

        // temporary in-memory file:
        static std::atomic_int s_tempNameGen = { 0 };
        std::string tmpPath = Stringify() << "/vsimem/" << std::this_thread::get_id() << std::to_string(s_tempNameGen++) << ".tif";

        const char* color_filename = processing == "color-relief" ? _colorRampFilename.c_str() : nullptr;

        GDALDatasetH outputDS = GDALDEMProcessing(tmpPath.c_str(), srcDS, processing.c_str(), color_filename, psOptions, &error);
        if (outputDS)
        {
            image = createImageFromDataset((GDALDataset*)outputDS);
            GDALClose(outputDS);
        }

        VSIUnlink(tmpPath.c_str());
        delete srcDS;
        GDALDEMProcessingOptionsFree(psOptions);
        CSLDestroy(papsz);

        if (image.valid())
        {
            // Make any NO_DATA_VALUE pixels transparent
            ImageUtils::PixelWriter writer(image.get());
            for (unsigned int r = 0; r < hf->getNumRows(); ++r)
            {
                for (unsigned int c = 0; c < hf->getNumColumns(); ++c)
                {
                    float h = hf->getHeight(c, r);
                    if (h == NO_DATA_VALUE)
                    {
                        writer(osg::Vec4(0, 0, 0, 0), c, r);
                    }
                }
            }
            return GeoImage(image.get(), key.getExtent());
        }
    }

#endif // HAS_GDALDEM

    return GeoImage::INVALID;
}