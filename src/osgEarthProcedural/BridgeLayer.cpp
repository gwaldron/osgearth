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
#include <osgEarth/JoinLines>
#include <osgEarth/TessellateOperator>
#include <osgEarth/FeatureStyleSorter>
#include <osgEarth/TerrainConstraintLayer>

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
    mergeConfig(conf);
}

BridgeSymbol::BridgeSymbol(const BridgeSymbol& rhs, const osg::CopyOp& copyop) :
    Symbol(rhs, copyop),
    _deckSkin(rhs._deckSkin),
    _girderSkin(rhs._girderSkin),
    _railingSkin(rhs._railingSkin),
    _deckWidth(rhs._deckWidth),
    _girderHeight(rhs._girderHeight),
    _railingHeight(rhs._railingHeight),
    _spanLift(rhs._spanLift)
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
    conf.set("deck_width", deckWidth());
    conf.set("girder_height", girderHeight());
    conf.set("railing_height", railingHeight());
    conf.set("span_lift", spanLift());
    return conf;
}

void
BridgeSymbol::mergeConfig(const Config& conf)
{
    conf.get("deck_skin", deckSkin());
    conf.get("girder_skin", girderSkin());
    conf.get("railing_skin", railingSkin());
    conf.get("deck_width", deckWidth());
    conf.get("girder_height", girderHeight());
    conf.get("railing_height", railingHeight());
    conf.get("span_lift", spanLift());
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
    else if (match(c.key(), "bridge-deck-width") || match(c.key(), "bridge-width")) {
        style.getOrCreate<BridgeSymbol>()->deckWidth() = c.value();
        style.getOrCreate<BridgeSymbol>()->deckWidth()->setDefaultUnits(Units::METERS);
    }
    else if (match(c.key(), "bridge-girder-height")) {
        style.getOrCreate<BridgeSymbol>()->girderHeight() = Distance(c.value(), Units::METERS);
    }
    else if (match(c.key(), "bridge-railing-height")) {
        style.getOrCreate<BridgeSymbol>()->railingHeight() = Distance(c.value(), Units::METERS);
    }
    else if (match(c.key(), "bridge-span-lift")) {
        style.getOrCreate<BridgeSymbol>()->spanLift() = Distance(c.value(), Units::METERS);
    }
}

//....................................................................

void
BridgeLayer::Options::fromConfig(const Config& conf)
{
    conf.get("constraint_min_level", constraintMinLevel());
}

Config
BridgeLayer::Options::getConfig() const
{
    auto conf = super::getConfig();
    conf.set("constraint_min_level", constraintMinLevel());
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

void
BridgeLayer::addedToMap(const Map* map)
{
    super::addedToMap(map);

    // create a terrain constraint layer that will 'clamp' the terrain to the end caps of our bridges.
    auto* c = new TerrainConstraintLayer();
    c->setName(getName() + "_constraints");
    c->setRemoveInterior(false);
    c->setRemoveExterior(false);
    c->setHasElevation(true);
    c->setMinLevel(std::max(options().constraintMinLevel().value(), getMinLevel()));

    c->constraintCallback([this](const TileKey& key, MeshConstraint& result, FilterContext*, ProgressCallback* progress)
        {
            this->addConstraint(key, result, progress);
        });

    auto status = c->open(getReadOptions());
    if (!status.isOK())
    {
        OE_WARN << LC << "Failed to open constraint layer: " << status.message() << std::endl;
        return;
    }
    
    Map* mutableMap = const_cast<Map*>(map);
    mutableMap->addLayer(c);

    _constraintLayer = c;
}

void
BridgeLayer::removedFromMap(const Map* map)
{
    if (_constraintLayer.valid())
    {
        Map* mutableMap = const_cast<Map*>(map);
        mutableMap->removeLayer(_constraintLayer.get());
        _constraintLayer = nullptr;
    }

    super::removedFromMap(map);    
}

namespace
{
    Geometry* line_to_polygon(const Geometry* geom, double width)
    {
        MultiGeometry* mg = new MultiGeometry();
        double halfWidth = 0.5 * width;

        ConstGeometryIterator i(geom, false);
        while (i.hasMore())
        {
            auto* line = i.next();
            if (line->size() < 2)
                continue;

            Polygon* poly = new Polygon();
            poly->resize(line->size() * 2);

            for (int i = 0; i < line->size(); ++i)
            {
                osg::Vec3d dir;
                if (i == 0)
                    dir = (*line)[i + 1] - (*line)[i];
                else if (i == line->size() - 1)
                    dir = (*line)[i] - (*line)[i - 1];
                else
                    dir = (*line)[i + 1] - (*line)[i - 1];
                dir.normalize();

                osg::Vec3d right = dir ^ osg::Vec3d(0, 0, 1);

                (*poly)[i] = (*line)[i] + right * halfWidth;
                (*poly)[line->size() * 2 - i - 1] = (*line)[i] - right * halfWidth;
            }

            mg->add(poly);
        }

        return mg;
    }

    Geometry* line_to_offset_curves(const Geometry* geom, double width)
    {
        MultiGeometry* mg = new MultiGeometry();
        double halfWidth = 0.5 * width;

        ConstGeometryIterator i(geom, false);
        while (i.hasMore())
        {
            auto* line = i.next();
            if (line->size() < 2)
                continue;

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
            }

            mg->add(left);
            mg->add(right);
        }
        return mg;
    }

    Geometry* line_to_end_caps(const Geometry* geom, double width)
    {
        auto mg = new MultiGeometry();
        double halfWidth = 0.5 * width;

        ConstGeometryIterator i(geom, false);
        while (i.hasMore())
        {
            auto* line = i.next();
            if (line->size() < 2)
                continue;

            LineString* start = new LineString();
            start->resize(2);

            LineString* end = new LineString();
            end->resize(2);

            auto dir0 = (*line)[1] - (*line)[0];
            auto right0 = dir0 ^ osg::Vec3d(0, 0, 1);
            right0.normalize();
            (*start)[0] = (*line)[0] + right0 * halfWidth;
            (*start)[1] = (*line)[0] - right0 * halfWidth;

            auto dir1 = (*line)[line->size() - 1] - (*line)[line->size() - 2];
            auto right1 = dir1 ^ osg::Vec3d(0, 0, 1);
            right1.normalize();
            (*end)[0] = (*line)[line->size() - 1] + right1 * halfWidth;
            (*end)[1] = (*line)[line->size() - 1] - right1 * halfWidth;

            mg->add(start);
            mg->add(end);
        }
        return mg;
    }

    void addConstraints(const FeatureList& c_features, const Style in_style, FilterContext& context, MeshConstraint& constriant)
    {
        Style style(in_style);

        auto* bridge = style.get<BridgeSymbol>();
        OE_SOFT_ASSERT_AND_RETURN(bridge, void());

        osg::ref_ptr<const SpatialReference> localSRS = context.extent()->getSRS()->createTangentPlaneSRS(
            context.extent()->getCentroid().vec3d());

        // convert our lines to polygons.
        // TODO: handle multis
        for (auto& feature : c_features)
        {
            osg::ref_ptr<Feature> f = new Feature(*feature);
            f->transform(localSRS);

            auto deckWidth = bridge->deckWidth()->eval(f, context);

            // just a reminder to implement multi-geometries if necessary
            auto* geom = line_to_end_caps(f->getGeometry(), deckWidth.as(Units::METERS));
            if (geom)
            {
                f->setGeometry(geom);
                f->transform(feature->getSRS());
                constriant.features.emplace_back(f);
            }
        }
    }

    osg::ref_ptr<osg::Node> createDeck(const FeatureList& c_features, const Style& in_style, FilterContext& context)
    {
        Style style(in_style);

        auto* bridge = style.get<BridgeSymbol>();
        OE_SOFT_ASSERT_AND_RETURN(bridge, {});

        auto* line = style.getOrCreate<LineSymbol>();
        line->library() = bridge->library();
        line->uriContext() = bridge->uriContext();
        line->stroke()->color() = Color::White;
        line->stroke()->width() = bridge->deckWidth();
        line->stroke()->width()->setDefaultUnits(Units::METERS);
        line->imageURI() = bridge->deckSkin();

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

        auto* bridge = style.get<BridgeSymbol>();
        OE_SOFT_ASSERT_AND_RETURN(bridge, {});

        Distance girderHeight = bridge->girderHeight().value();
        if (girderHeight.getValue() <= 0.0)
            return {};

        osg::ref_ptr<const SpatialReference> localSRS = context.extent()->getSRS()->createTangentPlaneSRS(
            context.extent()->getCentroid().vec3d());

        // convert our lines to polygons.
        // TODO: handle multis
        FeatureList features;
        for (auto& feature : c_features)
        {
            osg::ref_ptr<Feature> f = new Feature(*feature);
            f->transform(localSRS);

            auto deckWidth = bridge->deckWidth()->eval(feature, context);

            auto* geom = line_to_polygon(f->getGeometry(), deckWidth.as(Units::METERS) * 0.9);
            if (geom)
            {
                f->setGeometry(geom);
                f->transform(feature->getSRS());
                features.emplace_back(f);
            }
        }

        auto* extrude = style.getOrCreate<ExtrusionSymbol>();
        extrude->uriContext() = bridge->uriContext();
        extrude->library() = bridge->library();
        extrude->height() = -girderHeight.getValue(); // bridge->girderHeight().value();
        extrude->flatten() = false;
        extrude->wallSkinName() = bridge->girderSkin()->base();
        extrude->roofSkinName() = bridge->girderSkin()->base();

        auto* render = style.getOrCreate<RenderSymbol>();
        render->backfaceCulling() = false;

        auto node = GeometryCompiler().compile(features, style, context);
        return node;
    }

    osg::ref_ptr<osg::Node> createRailings(const FeatureList& c_features, const Style& in_style, FilterContext& context)
    {
        OE_SOFT_ASSERT_AND_RETURN(context.extent()->isValid(), {});

        Style style(in_style);

        auto* bridge = style.get<BridgeSymbol>();
        if (!bridge)
            return {};

        Distance railingHeight = bridge->railingHeight().value();
        if (railingHeight.getValue() <= 0.0)
            return {};

        osg::ref_ptr<const SpatialReference> localSRS = context.extent()->getSRS()->createTangentPlaneSRS(
            context.extent()->getCentroid().vec3d());

        // convert our lines to offset lines.
        // TODO: handle multis
        FeatureList features;
        for (auto& feature : c_features)
        {
            osg::ref_ptr<Feature> f = new Feature(*feature);
            f->transform(localSRS);

            auto deckWidth = bridge->deckWidth()->eval(feature, context);

            auto* geom = line_to_offset_curves(f->getGeometry(), deckWidth.as(Units::METERS));
            if (geom)
            {
                f->setGeometry(geom);
                f->transform(feature->getSRS());
                features.emplace_back(f);
            }
        }

        auto* extrude = style.getOrCreate<ExtrusionSymbol>();
        extrude->uriContext() = bridge->uriContext();
        extrude->library() = bridge->library();
        extrude->height() = railingHeight.as(Units::METERS);
        extrude->flatten() = false;
        extrude->wallSkinName() = bridge->railingSkin()->base();

        auto* render = style.getOrCreate<RenderSymbol>();
        render->backfaceCulling() = false;

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
            FilterContext context(_session.get(), key.getExtent());

            // combine the lines:
            JoinLinesFilter joiner;
            context = joiner.push(features, context);

            // clamp the lines:
            AltitudeFilter clamper;
            auto* alt = clamper.getOrCreateSymbol();
            alt->clamping() = alt->CLAMP_TO_TERRAIN;
            alt->binding() = alt->BINDING_ENDPOINT;
            context = clamper.push(features, context);

            // tessellate the lines:
            TessellateOperator filter;
            filter.setMaxPartitionSize(Distance(10, Units::METERS));
            context = filter.push(features, context);

            // wee hack to elevate the ends of the bridge just a bit.
            auto* bridge = in_style.get<BridgeSymbol>();
            Distance lift = bridge ? bridge->spanLift().value() : Distance(0.5, Units::METERS);
            double lift_m = lift.as(Units::METERS);
            for (auto& f : features)
            {
                GeometryIterator iter(f->getGeometry(), true);
                iter.forEach([&](Geometry* geom)
                    {
                        for (int i = 1; i < geom->size() - 1; ++i) {
                            (*geom)[i].z() += lift_m;
                        }
                    });
            }

            Style style(in_style);

            auto deck = createDeck(features, style, context);
            auto girders = createGirders(features, style, context);
            auto railings = createRailings(features, style, context);

            osg::Group* styleGroup = nullptr;

            if (deck.valid() || girders.valid() || railings.valid())
                styleGroup = new StyleGroup(style);

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

void
BridgeLayer::addConstraint(const TileKey& key, MeshConstraint& result, ProgressCallback* progress) const
{
    auto run = [&](const Style& in_style, FeatureList& features, ProgressCallback* progress)
        {
            FilterContext context(_session.get(), key.getExtent());

            // clamp the lines:
            AltitudeFilter clamper;
            auto* alt = clamper.getOrCreateSymbol();
            alt->clamping() = alt->CLAMP_TO_TERRAIN;
            alt->binding() = alt->BINDING_ENDPOINT;
            context = clamper.push(features, context);

            addConstraints(features, in_style, context, result);
        };

    FeatureStyleSorter().sort(key, Distance{}, _session.get(), _filters, run, progress);
}
