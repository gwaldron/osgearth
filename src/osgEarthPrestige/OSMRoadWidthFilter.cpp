/* osgEarth
 * Copyright 2026 Pelican Mapping
 * MIT License
 */
#include <osgEarthPrestige/OSMRoadWidthFilter>
#include <osgEarth/Units>
#include <algorithm>
#include <cmath>
#include <locale>
#include <sstream>

using namespace osgEarth;
using namespace osgEarthPrestige;

OSGEARTH_REGISTER_SIMPLE_FEATUREFILTER(prestige_osm_road_width, OSMRoadWidthFilter);

namespace
{
    //! Parses one finite nonnegative decimal, consuming the whole value independently of the process locale.
    bool number(const std::string& text, double& value)
    {
        std::istringstream input(text);
        input.imbue(std::locale::classic());
        if (!(input >> value) || !std::isfinite(value) || value < 0.0) return false;
        input >> std::ws;
        return input.eof();
    }

    //! Reads OSM metric/unit-qualified widths or feet/inches; rejects ambiguous ranges and unknown units.
    double widthMeters(const std::string& text)
    {
        double value = 0.0;
        const auto feetMark = text.find('\'');
        if (feetMark != std::string::npos)
        {
            double feet = 0.0, inches = 0.0;
            if (!number(text.substr(0, feetMark), feet)) return 0.0;
            auto tail = trim(text.substr(feetMark+1));
            if (!tail.empty())
            {
                if (tail.back() != '"' || !number(tail.substr(0, tail.size()-1), inches) || inches >= 12.0)
                    return 0.0;
            }
            value = feet*0.3048 + inches*0.0254;
        }
        else
        {
            UnitsType units;
            if (!Units::parse(text, value, units, Units::METERS) || !units.canConvert(Units::METERS)) return 0.0;
            value = Distance(value, units).as(Units::METERS);
        }
        return std::isfinite(value) && value > 0.0 ? value : 0.0;
    }

    //! Sums a complete pipe-separated list of measured lane widths; any missing/invalid lane rejects the list.
    double laneWidths(const std::string& text)
    {
        double sum = 0.0;
        std::size_t first = 0;
        while (first < text.size())
        {
            const auto end = text.find('|', first);
            const double width = widthMeters(text.substr(first, end == std::string::npos ? end : end-first));
            if (width <= 0.0) return 0.0;
            sum += width;
            if (end == std::string::npos) return sum;
            first = end+1;
        }
        return 0.0;
    }

    //! Accepts integral lane counts only; zero is useful for explicitly absent directional lanes.
    bool laneCount(const std::string& text, double& value)
    {
        return number(text, value) && std::floor(value) == value && value <= 20.0;
    }

    //! Uses a total count first; directional counts require both directions unless the road is one-way.
    double lanes(const Feature& feature)
    {
        double total = 0.0, forward = 0.0, backward = 0.0, both = 0.0;
        if (laneCount(feature.getString("lanes"), total) && total > 0.0) return total;
        const bool hasForward = laneCount(feature.getString("lanes:forward"), forward);
        const bool hasBackward = laneCount(feature.getString("lanes:backward"), backward);
        if (!hasForward) forward = 0.0;
        if (!hasBackward) backward = 0.0;
        if (feature.hasAttr("lanes:both_ways") && !laneCount(feature.getString("lanes:both_ways"), both)) return 0.0;
        const auto oneway = feature.getString("oneway");
        if (hasForward && (hasBackward || oneway == "yes" || oneway == "1" || oneway == "true"))
            return forward + backward + both;
        if (hasBackward && oneway == "-1") return backward + both;
        return 0.0;
    }

    //! Supplies demo estimates when OSM carries no usable measurements or counts; these are not engineering standards.
    double classWidth(const std::string& highway, double laneWidth)
    {
        if (endsWith(highway, "_link")) return laneWidth;
        if (highway == "service") return 5.0;
        if (highway == "living_street") return 4.5;
        if (highway == "residential" || highway == "unclassified") return 6.0;
        return 2.0*laneWidth;
    }
}

OSMRoadWidthFilter::OSMRoadWidthFilter(const Config& conf)
{
    conf.get("attribute", _attribute);
    conf.get("lane_width", _laneWidth);
    conf.get("max_width", _maxWidth);
}

Status OSMRoadWidthFilter::initialize(const osgDB::Options*)
{
    if (_attribute.empty() || !std::isfinite(_laneWidth) || _laneWidth <= 0.0 ||
        !std::isfinite(_maxWidth) || _maxWidth <= 0.0 || _maxWidth > 10000.0 || _laneWidth > _maxWidth)
        return Status(Status::ConfigurationError, "Invalid OSM road width settings");
    return Status::NoError;
}

Config OSMRoadWidthFilter::getConfig() const
{
    return Config("prestige_osm_road_width").set("attribute", _attribute)
        .set("lane_width", _laneWidth).set("max_width", _maxWidth);
}

FilterContext OSMRoadWidthFilter::push(FeatureList& features, FilterContext& context)
{
    for (auto& feature : features)
    {
        if (!feature || !feature->getGeometry() ||
            feature->getGeometry()->getComponentType() != Geometry::TYPE_LINESTRING || !feature->hasAttr("highway"))
            continue;
        double width = 0.0;
        std::string source;
        for (const auto* tag : {"width", "width:carriageway", "est_width", "width:lanes"})
        {
            const auto text = feature->getString(tag);
            width = std::string(tag) == "width:lanes" ? laneWidths(text) : widthMeters(text);
            if (width > 0.0 && width <= _maxWidth) { source = tag; break; }
            width = 0.0;
        }
        if (width <= 0.0)
        {
            width = lanes(*feature)*_laneWidth;
            source = "lanes";
            if (width <= 0.0 || width > _maxWidth)
            {
                width = classWidth(feature->getString("highway"), _laneWidth);
                source = "road_class";
            }
        }
        feature->set(_attribute, std::min(width, _maxWidth));
        feature->set(_attribute + "_source", source);
    }
    return context;
}
