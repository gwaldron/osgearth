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
#include <osgEarth/FeatureStyleSorter>

using namespace osgEarth;
using namespace osgEarth::Procedural;


#define LC "[BridgeLayer] "

#define OE_TEST OE_NULL

REGISTER_OSGEARTH_LAYER(bridges, BridgeLayer);

OSGEARTH_REGISTER_SIMPLE_SYMBOL_LAMBDA(
    bridge,
    [](const Config& c) { return new osgEarth::Procedural::BridgeSymbol(c); },
    [](const Config& c, class Style& s) { osgEarth::Procedural::BridgeSymbol::parseSLD(c, s); }
);

//....................................................................

BridgeSymbol::BridgeSymbol(const Config& conf)
{
}

BridgeSymbol::BridgeSymbol(const BridgeSymbol& rhs, const osg::CopyOp& copyop) :
    Symbol(rhs, copyop),
    _deckSkin(rhs._deckSkin),
    _girderSkin(rhs._girderSkin),
    _railingSkin(rhs._railingSkin),
    _width(rhs._width),
    _girderHeight(rhs._girderHeight),
    _railingHeight(rhs._railingHeight)
{
    //nop
}

Config
BridgeSymbol::getConfig() const
{
    Config conf = Symbol::getConfig();
    conf.set("deck_skin", deckSkin());
    conf.set("girder_skin", girderSkin());
    conf.set("railing_skin", railingSkin());
    conf.set("width", width());
    conf.set("girder_height", girderHeight());
    conf.set("railing_height", railingHeight());
    return conf;
}

void
BridgeSymbol::mergeConfig(const Config& conf)
{
    conf.get("deck_skin", deckSkin());
    conf.get("girder_skin", girderSkin());
    conf.get("railing_skin", railingSkin());
    conf.get("width", width());
    conf.get("girder_height", girderHeight());
    conf.get("railing_height", railingHeight());
}

void
BridgeSymbol::parseSLD(const Config& c, Style& style)
{
    if (match(c.key(), "library") && !c.value().empty())
        style.getOrCreate<BridgeSymbol>()->library() = Strings::unquote(c.value());
    else if (match(c.key(), "bridge-deck-skin"))
        style.getOrCreate<BridgeSymbol>()->deckSkin() = URI(Strings::unquote(c.value()), c.referrer());
    else if (match(c.key(), "bridge-girder-skin"))
        style.getOrCreate<BridgeSymbol>()->girderSkin() = URI(Strings::unquote(c.value()), c.referrer());
    else if (match(c.key(), "bridge-railing-skin"))
        style.getOrCreate<BridgeSymbol>()->railingSkin() = URI(Strings::unquote(c.value()), c.referrer());
    else if (match(c.key(), "bridge-width"))
        style.getOrCreate<BridgeSymbol>()->width() = NumericExpression(c.value());
    else if(match(c.key(), "bridge-girder-height"))
        style.getOrCreate<BridgeSymbol>()->girderHeight() = NumericExpression(c.value());
    else if(match(c.key(), "bridge-railing-height"))
        style.getOrCreate<BridgeSymbol>()->railingHeight() = NumericExpression(c.value());
}

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

        //OE_INFO << "width = " << width << "; poly dist = " << (poly->front() - poly->back()).length() << std::endl;

        return poly;
    }

    Geometry* line_to_offset_curves(const Geometry* line, double width)
    {
        OE_SOFT_ASSERT_AND_RETURN(line && line->size() >= 2 && width > 0.0, nullptr);

        double halfWidth = 0.5 * width;

        LineString* left = new LineString();
        left->resize(line->size());

        LineString* right = new LineString();
        right->resize(line->size());

        for (int i = 0; i < line->size(); ++i)
        {
            osg::Vec3d dir;
            if (i == 0)
                dir = (*line)[i + 1] - (*line)[i];
            else if (i == line->size() - 1)
                dir = (*line)[i] - (*line)[i - 1];
            else
                dir = (*line)[i + 1] - (*line)[i - 1];

            osg::Vec3d right_vec = dir ^ osg::Vec3d(0, 0, 1);

            right_vec.normalize();

            (*right)[i] = (*line)[i] + right_vec * halfWidth;
            (*left)[i] = (*line)[i] - right_vec * halfWidth;

             //OE_INFO << "width = " << width << "; dist = " << ((*right)[i] - (*left)[i]).length() << std::endl;
        }

        MultiGeometry* mg = new MultiGeometry();
        mg->add(left);
        mg->add(right);
        return mg;
    }


    osg::ref_ptr<osg::Node> createDeck(const FeatureList& c_features, const Style& in_style, FilterContext& context)
    {
        Style style(in_style);

        auto* bridgeSymbol = style.get<BridgeSymbol>();
        OE_SOFT_ASSERT_AND_RETURN(bridgeSymbol, {});

        auto* line = style.getOrCreate<LineSymbol>();
        line->library() = bridgeSymbol->library();
        line->uriContext() = bridgeSymbol->uriContext();
        line->stroke()->color() = Color::White;
        line->stroke()->width() = bridgeSymbol->width()->eval();
        line->stroke()->widthUnits() = Units::METERS;
        line->imageURI() = bridgeSymbol->deckSkin();

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

    osg::ref_ptr<osg::Node> createGirders(const FeatureList& c_features, const Style& in_style, FilterContext& context)
    {
        Style style(in_style);

        auto* bridgeSymbol = style.get<BridgeSymbol>();
        OE_SOFT_ASSERT_AND_RETURN(bridgeSymbol, {});

        // convert our lines to polygons.
        // TODO: handle multis
        FeatureList features;
        for (auto& feature : c_features)
        {
            auto expr = bridgeSymbol->width().value();
            auto* poly = line_to_polygon(feature->getGeometry(), feature->eval(expr, &context));
            if (poly)
            {
                auto* f = new Feature(*feature);
                f->setGeometry(poly);
                features.emplace_back(f);
            }
        }

        auto* extrude = style.getOrCreate<ExtrusionSymbol>();
        extrude->uriContext() = bridgeSymbol->uriContext();
        extrude->library() = bridgeSymbol->library();
        extrude->height() = -bridgeSymbol->girderHeight()->eval();
        extrude->flatten() = false;
        extrude->wallSkinName() = bridgeSymbol->girderSkin()->base();

        auto* render = style.getOrCreate<RenderSymbol>();
        render->backfaceCulling() = false;

        auto node = GeometryCompiler().compile(features, style, context);
        return node;
    }

    osg::ref_ptr<osg::Node> createRailings(const FeatureList& c_features, const Style& in_style, FilterContext& context)
    {
        Style style(in_style);

        auto* bridgeSymbol = style.get<BridgeSymbol>();
        OE_SOFT_ASSERT_AND_RETURN(bridgeSymbol, {});

        // convert our lines to offset lines.
        // TODO: handle multis
        FeatureList features;
        for (auto& feature : c_features)
        {
            auto expr = bridgeSymbol->width().value();
            auto* geom = line_to_offset_curves(feature->getGeometry(), feature->eval(expr, &context));
            if (geom)
            {
                auto* f = new Feature(*feature);
                f->setGeometry(geom);
                features.emplace_back(f);
            }
        }

        auto* extrude = style.getOrCreate<ExtrusionSymbol>();
        extrude->uriContext() = bridgeSymbol->uriContext();
        extrude->library() = bridgeSymbol->library();
        extrude->height() = bridgeSymbol->railingHeight()->eval();
        extrude->flatten() = false;
        extrude->wallSkinName() = bridgeSymbol->railingSkin()->base();

        auto* render = style.getOrCreate<RenderSymbol>();
        render->backfaceCulling() = false;

        auto* line = style.getOrCreate<LineSymbol>();
        line->stroke()->color() = Color::Yellow;
        line->stroke()->width() = 6;
        line->stroke()->smooth() = true;

        return GeometryCompiler().compile(features, style, context);
    }
}

osg::ref_ptr<osg::Node>
BridgeLayer::createTileImplementation(const TileKey& key, ProgressCallback* progress) const
{
    if (progress && progress->isCanceled())
        return nullptr;

    if (!getStatus().isOK() || !getFeatureSource())
        return nullptr;

    osg::ref_ptr<osg::Group> tileGroup;

    auto run = [&](const Style& in_style, FeatureList& features, ProgressCallback* progress)
        {
            FilterContext context(_session.get(), getFeatureSource()->getFeatureProfile(), key.getExtent(), nullptr); // index

            // clamp the lines:
            AltitudeFilter clamper;
            auto* alt = clamper.getOrCreateSymbol();
            alt->clamping() = alt->CLAMP_TO_TERRAIN;
            alt->binding() = alt->BINDING_ENDPOINT;
            context = clamper.push(features, context);

            // tessellate the lines:
            auto deck_width = std::max(4.0, in_style.get<BridgeSymbol>()->width()->eval());
            TessellateOperator filter;
            filter.setMaxPartitionSize(Distance(deck_width, Units::METERS));
            context = filter.push(features, context);

            Style style(in_style);

            auto deck = createDeck(features, style, context);
            auto girders = createGirders(features, style, context);
            auto railings = createRailings(features, style, context);

            osg::Group* styleGroup = nullptr;

            if (deck.valid() || girders.valid() || railings.valid())
                styleGroup = new osg::Group();

            if (deck.valid())
                styleGroup->addChild(deck.get());

            if (girders.valid())
                styleGroup->addChild(girders.get());

            if (railings.valid())
                styleGroup->addChild(railings.get());

            if (styleGroup)
            {
                if (!tileGroup.valid())
                {
                    tileGroup = new osg::Group();
                }

                tileGroup->addChild(styleGroup);
            }
        };

    FeatureStyleSorter().sort(key, Distance{}, _session.get(), _filters, run, progress);

    return tileGroup;
}

