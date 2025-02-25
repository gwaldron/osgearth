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
#include "BridgeLayer"
#include <osgEarth/GeometryCompiler>
#include <osgEarth/AltitudeFilter>
#include <osgEarth/TessellateOperator>

using namespace osgEarth;
using namespace osgEarth::Procedural;


#define LC "[BridgeLayer] "

#define OE_TEST OE_NULL

REGISTER_OSGEARTH_LAYER(bridges, BridgeLayer);


//....................................................................

void
BridgeLayer::Options::fromConfig(const Config& conf)
{
}

Config
BridgeLayer::Options::getConfig() const
{
    auto conf = super::getConfig();
    return conf;
}

//....................................................................

#define DECK_STYLE_NAME "deck"
#define GIRDER_STYLE_NAME "girder"
#define RAILING_STYLE_NAME "railing"

void
BridgeLayer::init()
{
    super::init();

    // some reasonable defaults
    options().minLevel().setDefault(14u);
    options().maxLevel().setDefault(14u);
    options().additive().setDefault(false);
}

Config
BridgeLayer::getConfig() const
{
    Config conf = super::getConfig();
    return conf;
}


Status
BridgeLayer::openImplementation()
{
    Status parent = super::openImplementation();
    if (parent.isError())
        return parent;

    return Status::NoError;
}

Status
BridgeLayer::closeImplementation()
{
    super::closeImplementation();    
    return getStatus();
}

void
BridgeLayer::addedToMap(const Map* map)
{
    super::addedToMap(map);    
}

void
BridgeLayer::removedFromMap(const Map* map)
{
    super::removedFromMap(map);    
}

namespace
{
    Geometry* line_to_polygon(const Geometry* line, double width)
    {
        OE_SOFT_ASSERT_AND_RETURN(line && line->size() >= 2 && width > 0.0, nullptr);

        double halfWidth = 0.5 * width;

        Polygon* poly = new Polygon();
        poly->resize(line->size() * 2);

        for(int i = 0; i < line->size(); ++i)
        {
            osg::Vec3d dir;
            if (i == 0)
                dir = (*line)[i + 1] - (*line)[i];
            else if (i == line->size()-1)
                dir = (*line)[i] - (*line)[i - 1];
            else
                dir = (*line)[i + 1] - (*line)[i - 1];
            dir.normalize();

            osg::Vec3d right = dir ^ osg::Vec3d(0, 0, 1);

            (*poly)[i] = (*line)[i] + right * halfWidth;
            (*poly)[line->size() * 2 - i - 1] = (*line)[i] - right * halfWidth;
        }

        return poly;
    }

    osg::ref_ptr<osg::Node> createDeck(const FeatureList& c_features, const Style& deckStyle, float roadWidth, FilterContext& context)
    {
        Style style;

        auto* line = style.getOrCreate<LineSymbol>();
        line->stroke()->color() = Color::White;
        line->stroke()->width() = roadWidth;
        line->stroke()->widthUnits() = Units::METERS;
        line->tessellationSize() = Distance("10m");
        line->imageURI()->setLiteral("../data/road.png");

        auto* render = style.getOrCreate<RenderSymbol>();
        render->backfaceCulling() = false;

        // clone the features
        FeatureList features;
        features.reserve(c_features.size());
        std::transform(c_features.begin(), c_features.end(), std::back_inserter(features), [](Feature* f) {
            return new Feature(*f); });
        
        auto node = GeometryCompiler().compile(features, style, context);
        return node;
    }

    osg::ref_ptr<osg::Node> createGirders(const FeatureList& c_features, const Style& girderStyle, float girderHeight, FilterContext& context)
    {
        const float road_width = 10.0f;

        // convert our lines to polygons.
        // TODO: handle multis
        FeatureList features;
        for (auto& feature : c_features)
        {
            auto* poly = line_to_polygon(feature->getGeometry(), road_width);
            if (poly)
            {
                auto* f = new Feature(*feature);
                f->setGeometry(poly);
                features.emplace_back(f);
            }
        }

        // style for girders:
        Style style;

        auto* poly = style.getOrCreate<PolygonSymbol>();
        poly->fill()->color() = Color::Gray;
        
        auto* extrude = style.getOrCreate<ExtrusionSymbol>();
        extrude->height() = -girderHeight;
        extrude->flatten() = false;

        auto* render = style.getOrCreate<RenderSymbol>();
        render->backfaceCulling() = false;

        auto node = GeometryCompiler().compile(features, style, context);
        return node;
    }

    osg::ref_ptr<osg::Node> createRailings(const FeatureList& c_features, const Style& girderStyle, FilterContext& context)
    {
        return {};
    }
}

osg::ref_ptr<osg::Node>
BridgeLayer::createTileImplementation(const TileKey& key, ProgressCallback* progress) const
{
    if (progress && progress->isCanceled())
        return nullptr;

    if (!getStatus().isOK() || !getFeatureSource())
        return nullptr;

    // fetch the feature data for this tile.
    auto cursor = getFeatureSource()->createFeatureCursor(Query(key), _filters, nullptr, progress);
    if (cursor.valid() && cursor->hasMore())
    {
        FeatureList features;
        cursor->fill(features);

        FilterContext context(_session.get(), getFeatureSource()->getFeatureProfile(), key.getExtent(), nullptr); // index

        Style s;
        auto* alt = s.getOrCreate<AltitudeSymbol>();
        alt->clamping() = alt->CLAMP_TO_TERRAIN;
        alt->binding() = alt->BINDING_ENDPOINT;

        auto* line = s.getOrCreate<LineSymbol>();
        line->tessellationSize() = Distance(10.0, Units::METERS);

        // clamp the lines:
        AltitudeFilter clamper;
        clamper.setPropertiesFromStyle(s);
        context = clamper.push(features, context);

        // tessellate the lines:
        TessellateOperator filter;
        filter.setMaxPartitionSize(Distance("10m"));
        context = filter.push(features, context);

        auto deck = createDeck(features, _deckStyle, _roadWidth, context);
        auto girders = createGirders(features, _girderStyle, _girderHeight, context);
        auto railings = createRailings(features, _railingStyle, context);

        osg::Group* group = nullptr;
        if (deck.valid() || girders.valid() || railings.valid())
            group = new osg::Group();

        if (deck.valid())
            group->addChild(deck.get());

        if (girders.valid())
            group->addChild(girders.get());

        if (railings.valid())
            group->addChild(railings.get());

        return group;
    }

    return {};
}

