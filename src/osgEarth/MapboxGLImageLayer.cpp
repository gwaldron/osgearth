/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
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
#include <osgEarth/MapboxGLImageLayer>
#include <osgEarth/Session>

#include <osgEarth/FeatureImageLayer>
#include <osgEarth/ArcGISTilePackage>
#include <osgEarth/XYZFeatureSource>
#include <osgEarth/Registry>

#include <osgDB/WriteFile>

using namespace osgEarth;

#define LC "[MapboxGLImageLayer] " << getName() << ": "


REGISTER_OSGEARTH_LAYER(mapboxglimage, MapBoxGLImageLayer);
OE_LAYER_PROPERTY_IMPL(MapBoxGLImageLayer, URI, URL, url);

void getIfSet(const Json::Value& object, const std::string& member, optional<std::string>& value)
{
    if (object.isMember(member))
    {
        auto jsonValue = object.get(member, "");
        if (!jsonValue.isObject())
        {
            value = object.get(member, "").asString();
        }
    }
}

void getIfSet(const Json::Value& object, const std::string& member, optional<float>& value)
{
    if (object.isMember(member))
    {
        auto jsonValue = object.get(member, 0.0f);
        // Check for an object value  here as the line-width could be an array which is a series of "stops".  We don't support that yet.
        if (!jsonValue.isObject())
        {
            value = (float)object.get(member, 0.0f).asDouble();
        }
    }
}

void getIfSet(const Json::Value& object, const std::string& member, optional<double>& value)
{
    if (object.isMember(member) && object.isDouble())
    {
        auto jsonValue = object.get(member, 0.0f);
        // Check for an object value  here as the line-width could be an array which is a series of "stops".  We don't support that yet.
        if (!jsonValue.isObject())
        {
            value = object.get(member, 0.0).asDouble();
        }
    }
}

void getIfSet(const Json::Value& object, const std::string& member, optional<unsigned int>& value)
{
    if (object.isMember(member))
    {
        value = object.get(member, 0.0).asUInt();
    }
}

void getIfSet(const Json::Value& object, const std::string& member, optional<int>& value)
{
    if (object.isMember(member))
    {
        value = object.get(member, 0.0).asInt();
    }
}


/*************************/
MapBoxStyleSheet::Source::Source()
{
}

std::string& MapBoxStyleSheet::Source::attribution()
{
    return _attribution;
}

const std::string& MapBoxStyleSheet::Source::attribution() const
{
    return _attribution;
}

std::string& MapBoxStyleSheet::Source::url()
{
    return _url;
}

const std::string& MapBoxStyleSheet::Source::url() const
{
    return _url;
}

std::string& MapBoxStyleSheet::Source::type()
{
    return _type;
}

const std::string& MapBoxStyleSheet::Source::type() const
{
    return _type;
}

std::string& MapBoxStyleSheet::Source::name()
{
    return _name;
}

const std::string& MapBoxStyleSheet::Source::name() const
{
    return _name;
}

const std::vector< std::string >& MapBoxStyleSheet::Source::tiles() const
{
    return _tiles;
}

std::vector< std::string >& MapBoxStyleSheet::Source::tiles()
{
    return _tiles;
}

FeatureSource* MapBoxStyleSheet::Source::featureSource()
{
    return _featureSource.get();
}

const FeatureSource* MapBoxStyleSheet::Source::featureSource() const
{
    return _featureSource.get();
}

void MapBoxStyleSheet::Source::loadFeatureSource(const std::string& styleSheetURI)
{
    if (!_featureSource.valid())
    {
        URIContext context(styleSheetURI);

        if (name() == "esri")
        {
            URI uri(url(), context);

            osg::ref_ptr< VTPKFeatureSource > featureSource = new VTPKFeatureSource;
            featureSource->setURL(uri);
            featureSource->open();
            _featureSource = featureSource.get();
        }
        else if (type() == "vector")
        {
            if (!tiles().empty())
            {
                osg::ref_ptr< XYZFeatureSource > featureSource = new XYZFeatureSource;
                URI uri(tiles()[0], context);
                featureSource->setMinLevel(0);
                featureSource->setMaxLevel(14);
                featureSource->setURL(uri);
                featureSource->setFormat("pbf");
                featureSource->options().profile() = ProfileOptions("spherical-mercator");
                featureSource->open();
                _featureSource = featureSource.get();
            }
        }
    }
}


/*************************/
MapBoxStyleSheet::Paint::Paint() :
    _backgroundColor("#000000"),
    _backgroundOpacity(1.0f),
    _fillAntialias(true),
    _fillColor("#000000"),
    _fillOpacity(1.0f),

    _lineColor("#000000"),
    _lineWidth(1.0f),

    _visibility("visible")
{
}

optional<std::string>& MapBoxStyleSheet::Paint::backgroundColor()
{
    return _backgroundColor;
}

optional<float>& MapBoxStyleSheet::Paint::backgroundOpacity()
{
    return _backgroundOpacity;
}

optional<std::string>& MapBoxStyleSheet::Paint::fillColor()
{
    return _fillColor;
}

optional<std::string>& MapBoxStyleSheet::Paint::lineColor()
{
    return _lineColor;
}

optional<float>& MapBoxStyleSheet::Paint::lineWidth()
{
    return _lineWidth;
}

const optional<std::string>& MapBoxStyleSheet::Paint::backgroundColor() const
{
    return _backgroundColor;
}

const optional<float>& MapBoxStyleSheet::Paint::backgroundOpacity() const
{
    return _backgroundOpacity;
}

const optional<std::string>& MapBoxStyleSheet::Paint::fillColor() const
{
    return _fillColor;
}

const optional<std::string>& MapBoxStyleSheet::Paint::lineColor() const
{
    return _lineColor;
}

const optional<float>& MapBoxStyleSheet::Paint::lineWidth() const
{
    return _lineWidth;
}
/*************************/


MapBoxStyleSheet::Layer::Layer()
{
}

const std::string& MapBoxStyleSheet::Layer::id() const
{
    return _id;
}

std::string& MapBoxStyleSheet::Layer::id()
{
    return _id;
}

std::string& MapBoxStyleSheet::Layer::source()
{
    return _source;
}

const std::string& MapBoxStyleSheet::Layer::source() const
{
    return _source;
}

const std::string& MapBoxStyleSheet::Layer::sourceLayer() const
{
    return _sourceLayer;
}

std::string& MapBoxStyleSheet::Layer::sourceLayer()
{
    return _sourceLayer;
}

const std::string& MapBoxStyleSheet::Layer::type() const
{
    return _type;
}

std::string& MapBoxStyleSheet::Layer::type()
{
    return _type;
}

const unsigned int& MapBoxStyleSheet::Layer::minZoom() const
{
    return _minZoom;
}

unsigned int& MapBoxStyleSheet::Layer::minZoom()
{
    return _minZoom;
}


unsigned int& MapBoxStyleSheet::Layer::maxZoom()
{
    return _maxZoom;
}

const unsigned int& MapBoxStyleSheet::Layer::maxZoom() const
{
    return _maxZoom;
}

MapBoxStyleSheet::Paint& MapBoxStyleSheet::Layer::paint()
{
    return _paint;
}

const MapBoxStyleSheet::Paint& MapBoxStyleSheet::Layer::paint() const
{
    return _paint;
}

MapBoxStyleSheet::FilterExpression& MapBoxStyleSheet::Layer::filter()
{
    return _filter;
}

const MapBoxStyleSheet::FilterExpression& MapBoxStyleSheet::Layer::filter() const
{
    return _filter;
}
/*************************/


const std::string& MapBoxStyleSheet::version() const
{
    return _version;
}

const std::string& MapBoxStyleSheet::name() const
{
    return _name;
}

const std::string& MapBoxStyleSheet::sprite() const
{
    return _sprite;
}

const std::string& MapBoxStyleSheet::glyphs() const
{
    return _glyphs;
}

const std::vector< MapBoxStyleSheet::Layer >& MapBoxStyleSheet::layers() const
{
    return _layers;
}

std::vector< MapBoxStyleSheet::Layer >& MapBoxStyleSheet::layers()
{
    return _layers;
}

const std::vector< MapBoxStyleSheet::Source >& MapBoxStyleSheet::sources() const
{
    return _sources;
}

std::vector< MapBoxStyleSheet::Source >& MapBoxStyleSheet::sources()
{
    return _sources;
}

MapBoxStyleSheet::MapBoxStyleSheet()
{
}

MapBoxStyleSheet MapBoxStyleSheet::load(const URI& location, const osgDB::Options* options)
{
    MapBoxStyleSheet styleSheet;

    auto rr = location.readString(options);
    std::string data = rr.getString();

    Json::Reader reader;
    Json::Value root(Json::objectValue);
    if (!reader.parse(data, root, false))
    {
        return styleSheet;
    }

    styleSheet._version = root.get("version", "").asString();
    styleSheet._name = root.get("name", "").asString();
    styleSheet._sprite = root.get("sprite", "").asString();
    styleSheet._glyphs = root.get("glyphs", "").asString();

    if (root.isMember("sources"))
    {
        const Json::Value& s = root["sources"];
        for (Json::Value::const_iterator i = s.begin(); i != s.end(); ++i)
        {
            const Json::Value& sourceJson = (*i);

            Source source;
            source.name() = i.key().asString();

            if (sourceJson.isMember("url"))
            {
                source.url() = sourceJson.get("url", "").asString();
            }
            else if (sourceJson.isMember("tiles"))
            {
                const Json::Value& tilesJson = sourceJson["tiles"];
                for (Json::Value::const_iterator tileItr = tilesJson.begin(); tileItr != tilesJson.end(); ++tileItr)
                {
                    source.tiles().push_back((*tileItr).asString());
                }
            }
            source.attribution() = sourceJson.get("attribution", "").asString();
            source.type() = sourceJson.get("type", "").asString();
            source.loadFeatureSource(location.full());
            styleSheet._sources.push_back(source);
        }
    }

    if (root.isMember("layers"))
    {
        const Json::Value& s = root["layers"];
        for (Json::Value::const_iterator i = s.begin(); i != s.end(); ++i)
        {
            const Json::Value& layerJson = (*i);
            Layer layer;
            layer.id() = layerJson.get("id", "").asString();
            layer.source() = layerJson.get("source", "").asString();
            layer.sourceLayer() = layerJson.get("source-layer", "").asString();
            layer.type() = layerJson.get("type", "").asString();
            layer.minZoom() = layerJson.get("minzoom", 0).asUInt();
            layer.maxZoom() = layerJson.get("maxzoom", 24).asUInt();

            // Parse paint
            if (layerJson.isMember("paint"))
            {
                const Json::Value& paint = layerJson["paint"];
                getIfSet(paint, "background-color", layer.paint().backgroundColor());
                getIfSet(paint, "background-opacity", layer.paint().backgroundOpacity());
                getIfSet(paint, "fill-color", layer.paint().fillColor());

                getIfSet(paint, "line-color", layer.paint().lineColor());
                getIfSet(paint, "line-width", layer.paint().lineWidth());
            }

            if (layerJson.isMember("filter"))
            {
               layer.filter()._filter = layerJson["filter"];
            }

            //std::cout << "Got layer " << layer.id() << " type=" << layer.type() << " source=" << layer.source() << " source-layer=" << layer.sourceLayer() << " zoom=" << layer.minZoom() << "-" << layer.maxZoom() << std::endl;
            styleSheet._layers.emplace_back(std::move(layer));
        }
    }
    std::cout << "Found " << styleSheet._layers.size() << " layers" << std::endl;

    return styleSheet;
}


Config
MapBoxGLImageLayer::Options::getConfig() const
{
    Config conf = ImageLayer::Options::getConfig();
    conf.set("url", _url);
    return conf;
}

void
MapBoxGLImageLayer::Options::fromConfig(const Config& conf)
{
    conf.get("url", url());
}

void
MapBoxGLImageLayer::init()
{
    ImageLayer::init();

    // Default profile (WGS84) if not set
    if (!getProfile())
    {
        setProfile(Profile::create("global-geodetic"));
    }
}

Status
MapBoxGLImageLayer::openImplementation()
{
    Status parent = ImageLayer::openImplementation();
    if (parent.isError())
        return parent;

#if 0
    auto rr = getURL().readString(this->getReadOptions());
    if (rr.failed())
    {
        return Status::Error(Stringify() << "Failed to read style from " << getURL().full());
    }
    _styleSheet = MapBoxStyleSheet::load(rr.getString());
#endif
    _styleSheet = MapBoxStyleSheet::load(getURL(), getReadOptions());

    return Status::NoError;
}


void
MapBoxGLImageLayer::addedToMap(const Map* map)
{
    ImageLayer::addedToMap(map);

    _map = map;
}

void
MapBoxGLImageLayer::removedFromMap(const Map* map)
{
    ImageLayer::removedFromMap(map);
}

struct LayeredFeatures
{
    std::map< std::string, FeatureList > features;
};


bool evalFilter(const Json::Value& filter, osgEarth::Feature* feature)
{
    if (!filter.isArray())
    {
        return false;
    }

    //std::cout << Json::StyledWriter().write(filter) << std::endl;

    auto op = osgEarth::trim(filter[0u].asString());

    // https://docs.mapbox.com/mapbox-gl-js/style-spec/other/#other-filter

    // Comparison filters
    if (op == "all")
    {
        bool finalResult = true;
        for (unsigned int i = 1; i < filter.size(); ++i)
        {
            finalResult &= evalFilter(filter[i], feature);
            // Early out.
            if (!finalResult)
            {
                return false;
            }
        }
        return finalResult;
    }
    else if (op == "any")
    {
        bool finalResult = false;
        for (unsigned int i = 1; i < filter.size(); ++i)
        {
            finalResult |= evalFilter(filter[i], feature);
            // Early out.
            if (finalResult)
            {
                return true;
            }
        }
        return finalResult;
    }
    else if (op == "none")
    {
        bool finalResult = false;
        for (unsigned int i = 1; i < filter.size(); ++i)
        {
            finalResult &= !evalFilter(filter[i], feature);
            // Early out.
            if (finalResult)
            {
                return false;
            }
        }
        return finalResult;
    }
    // Existential filters
    else if (op == "has")
    {
        std::string key = filter[1u].asString();
        return feature->hasAttr(key);
    }
    else if (op == "!has")
    {
        std::string key = filter[1u].asString();
        return !feature->hasAttr(key);
    }

    // Comparison filters
    else if (op == "==")
    {
        std::string key = filter[1u].asString();
        const Json::Value& value = filter[2u];

        if (!feature->hasAttr(key)) return false;

        if (key == "$type")
        {
            std::string typeString;
            if (feature->getGeometry()->getType() == Geometry::Type::TYPE_LINESTRING)
            {
                typeString = "LineString";
            }
            else if (feature->getGeometry()->getType() == Geometry::Type::TYPE_POLYGON)
            {
                typeString = "Polygon";
            }
            else if (feature->getGeometry()->getType() == Geometry::Type::TYPE_POINT || feature->getGeometry()->getType() == Geometry::Type::TYPE_POINTSET)
            {
                typeString = "Polygon";
            }
            return typeString == value.asString();
        }
        else if (value.isString())
        {
            return feature->getString(key) == value.asString();
        }
        else if (value.isBool())
        {
            return feature->getBool(key) == value.asBool();
        }
        else if (value.isDouble())
        {
            return feature->getDouble(key) == value.asDouble();
        }
        else if (value.isIntegral())
        {
            return feature->getInt(key) == value.asInt();
        }
        return false;
    }
    else if (op == "!=")
    {
        std::string key = filter[1u].asString();
        const Json::Value& value = filter[2u];

        if (!feature->hasAttr(key)) return false;

        if (value.isString())
        {
            return feature->getString(key) != value.asString();
        }
        else if (value.isBool())
        {
            return feature->getBool(key) != value.asBool();
        }
        else if (value.isDouble())
        {
            return feature->getDouble(key) != value.asDouble();
        }
        else if (value.isIntegral())
        {
            return feature->getInt(key) != value.asInt();
        }
        return false;
    }
    else if (op == ">")
    {
        std::string key = filter[1u].asString();
        const Json::Value& value = filter[2u];

        if (!feature->hasAttr(key)) return false;

        if (value.isString())
        {
            return feature->getString(key) > value.asString();
        }
        else if (value.isBool())
        {
            return feature->getBool(key) > value.asBool();
        }
        else if (value.isDouble())
        {
            return feature->getDouble(key) > value.asDouble();
        }
        else if (value.isIntegral())
        {
            return feature->getInt(key) > value.asInt();
        }
        return false;
    }
    else if (op == ">=")
    {
        std::string key = filter[1u].asString();
        const Json::Value& value = filter[2u];

        if (!feature->hasAttr(key)) return false;

        if (value.isString())
        {
            return feature->getString(key) >= value.asString();
        }
        else if (value.isBool())
        {
            return feature->getBool(key) >= value.asBool();
        }
        else if (value.isDouble())
        {
            return feature->getDouble(key) >= value.asDouble();
        }
        else if (value.isIntegral())
        {
            return feature->getInt(key) >= value.asInt();
        }
        return false;
    }
    else if (op == "<")
    {
        std::string key = filter[1u].asString();
        const Json::Value& value = filter[2u];

        if (!feature->hasAttr(key)) return false;

        if (value.isString())
        {
            return feature->getString(key) < value.asString();
        }
        else if (value.isBool())
        {
            return feature->getBool(key) < value.asBool();
        }
        else if (value.isDouble())
        {
            return feature->getDouble(key) < value.asDouble();
        }
        else if (value.isIntegral())
        {
            return feature->getInt(key) < value.asInt();
        }
        return false;
    }
    else if (op == "<=")
    {
        std::string key = filter[1u].asString();
        const Json::Value& value = filter[2u];

        if (!feature->hasAttr(key)) return false;

        if (value.isString())
        {
            return feature->getString(key) <= value.asString();
        }
        else if (value.isBool())
        {
            return feature->getBool(key) <= value.asBool();
        }
        else if (value.isDouble())
        {
            return feature->getDouble(key) <= value.asDouble();
        }
        else if (value.isIntegral())
        {
            return feature->getInt(key) <= value.asInt();
        }
        return false;
    }
    // Set membership filters
    else if (op == "in")
    {
        std::string key = filter[1u].asString();

        if (!feature->hasAttr(key)) return false;

        for (unsigned int i = 2; i < filter.size(); ++i)
        {
            const Json::Value& value = filter[i];

            if (value.isString())
            {
                if (feature->getString(key) == value.asString())
                {
                    return true;
                }
            }
            else if (value.isBool())
            {
                if (feature->getBool(key) == value.asBool())
                {
                    return true;
                }
            }
            else if (value.isDouble())
            {
                if (feature->getDouble(key) == value.asDouble())
                {
                    return true;
                }
            }
            else if (value.isIntegral())
            {
                if (feature->getInt(key) == value.asInt())
                {
                    return true;
                }
            }
        }
        return false;
    }
    else if (op == "!in")
    {
        std::string key = filter[1u].asString();

        std::string featureValue = feature->getString(key);

        if (!feature->hasAttr(key))
        {
            return true;
        }

        for (unsigned int i = 2; i < filter.size(); ++i)
        {
            const Json::Value& value = filter[i];

            if (value.isString())
            {
                if (feature->getString(key) == value.asString())
                {
                    return false;
                }
            }
            else if (value.isBool())
            {
                if (feature->getBool(key) == value.asBool())
                {
                    return false;
                }
            }
            else if (value.isDouble())
            {
                if (feature->getDouble(key) == value.asDouble())
                {
                    return false;
                }
            }
            else if (value.isIntegral())
            {
                if (feature->getInt(key) == value.asInt())
                {
                    return false;
                }
            }
        }
        return true;
    }


    return true;
}

GeoImage
MapBoxGLImageLayer::createImageImplementation(const TileKey& key, ProgressCallback* progress) const
{
    if (getStatus().isError())
    {
        return GeoImage::INVALID;
    }

    // Allocate the intial image
    osg::ref_ptr<osg::Image> image = new osg::Image;
    image->allocateImage(getTileSize(), getTileSize(), 1, GL_RGBA, GL_UNSIGNED_BYTE);
    ::memset(image->data(), 0x00, image->getTotalSizeInBytes());

    std::map< std::string, LayeredFeatures > sourceToFeatures;

    for (auto& layer : _styleSheet.layers())
    {
        if (key.getLevelOfDetail() >= layer.minZoom() && key.getLevelOfDetail() <= layer.maxZoom())
        {
            if (layer.type() == "background")
            {
                if (layer.paint().backgroundColor().isSet())
                {
                    Color backgroundColor = Color(layer.paint().backgroundColor().get());
                    ImageUtils::PixelWriter write(image.get());
                    write.assign(backgroundColor);
                }
            }
            else
            {
                osg::ref_ptr< FeatureImageLayer > featureImage;

                osg::ref_ptr< Session > session;

                // Find the Source
                for (auto& s : _styleSheet.sources())
                {
                    if (s.name() == layer.source())
                    {
                        FeatureSource* featureSource = const_cast<FeatureSource*>(s.featureSource());
                        if (!featureSource)
                        {
                            break;
                        }

                        osg::ref_ptr< StyleSheet > styleSheet = new StyleSheet;
                        session = new Session(_map.get(), styleSheet.get(), nullptr, nullptr);
                        featureImage = new FeatureImageLayer();
                        featureImage->setFeatureSource(featureSource);
                        featureImage->open();

                        break;
                    }
                }

                if (!featureImage.valid())
                {
                    continue;
                }


                LayeredFeatures layeredFeatures;
                // See if we already got the features for this tile for this source
                auto featuresItr = sourceToFeatures.find(layer.source());
                if (featuresItr == sourceToFeatures.end())
                {
                    FeatureList allFeatures;
                    TileKey queryKey = key;
                    while (allFeatures.empty() && queryKey.valid())
                    {
                        // Get all the features from the feature source.
                        osg::ref_ptr< FeatureCursor > cursor = featureImage->getFeatureSource()->createFeatureCursor(queryKey, nullptr);
                        if (cursor.valid())
                        {
                            cursor->fill(allFeatures);
                        }
                        if (allFeatures.empty())
                        {
                            queryKey = queryKey.createParentKey();
                        }
                    }

                    for (auto& f : allFeatures)
                    {
                        layeredFeatures.features[f->getString("mvt_layer")].push_back(f.get());
                    }

                    sourceToFeatures[layer.source()] = layeredFeatures;
                }
                else
                {
                    layeredFeatures = featuresItr->second;
                }


                if (layeredFeatures.features.find(layer.sourceLayer()) != layeredFeatures.features.end())
                {
                    // Run any filters on the layer.
                    FeatureList features;
                    if (!layer.filter()._filter.empty())
                    {
                        for (auto& f : layeredFeatures.features[layer.sourceLayer()])
                        {
                            if (evalFilter(layer.filter()._filter, f.get()))
                            {
                                features.push_back(f.get());
                            }
                        }
                    }
                    else
                    {
                        features = layeredFeatures.features[layer.sourceLayer()];
                    }

                    if (layer.type() == "fill")
                    {
                        Style style;
                        style.getOrCreateSymbol<PolygonSymbol>()->fill() = Color(layer.paint().fillColor().get());
                        featureImage->renderFeaturesForStyle(session.get(), style, features, key.getExtent(), image.get(), nullptr);
                    }
                    else if (layer.type() == "line")
                    {
                        Style style;
                        style.getOrCreateSymbol<LineSymbol>()->stroke()->color() = Color(layer.paint().lineColor().get());
                        style.getOrCreateSymbol<LineSymbol>()->stroke()->width() = layer.paint().lineWidth().get();
                        style.getOrCreateSymbol<LineSymbol>()->stroke()->widthUnits() = Units::PIXELS;
                        featureImage->renderFeaturesForStyle(session.get(), style, features, key.getExtent(), image.get(), nullptr);
                    }
                }
            }
        }
    }



    return GeoImage(image.get(), key.getExtent());
}