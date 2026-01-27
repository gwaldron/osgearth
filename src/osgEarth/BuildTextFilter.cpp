/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include <osgEarth/BuildTextFilter>
#include <osgEarth/FilterContext>
#include <osgEarth/TextSymbol>
#include <osgEarth/PlaceNode>
#include <osgEarth/FeatureIndex>
#include <osgEarth/Notify>
#include <osgEarth/TextSymbolizer>
#include <osgEarth/Text>
#include <osgEarth/AnnotationUtils>
#include <osg/ShapeDrawable>

#define LC "[BuildTextFilter] "

using namespace osgEarth;
using namespace osgEarth::Util;

namespace
{
    PlaceNode* makePlaceNode(
        FilterContext& context,
        Feature* feature,
        const Style& style)
    {
        osg::Vec3d center = feature->getGeometry()->getBounds().center();

        AltitudeMode mode = ALTMODE_ABSOLUTE;
        osg::Vec3d localOffset;

        GeoPoint point;

        const AltitudeSymbol* alt = style.get<AltitudeSymbol>();

        // If the symbol asks for map-clamping, disable any auto-scene-clamping on the annotation:
        if (alt != NULL &&
            alt->clamping() != alt->CLAMP_NONE &&
            alt->technique().isSetTo(alt->TECHNIQUE_MAP))
        {
            point.set(feature->getSRS(), center.x(), center.y(), center.z(), ALTMODE_ABSOLUTE);
        }

        // If the symbol says clamp to terrain (but not using the map), zero out the 
        // Z value and dynamically clamp to the surface:
        else if (
            alt != NULL &&
            alt->clamping() == alt->CLAMP_TO_TERRAIN &&
            !alt->technique().isSetTo(alt->TECHNIQUE_MAP))
        {
            point.set(feature->getSRS(), center.x(), center.y(), 0.0, ALTMODE_RELATIVE);
        }

        // By default, use terrain-relative scene clamping Zm above the terrain.
        else
        {
            point.set(feature->getSRS(), center.x(), center.y(), center.z(), ALTMODE_RELATIVE);
        }

        PlaceNode* node = new PlaceNode();
        //LabelNode* node = new LabelNode();
        node->setStyle(style, context.getDBOptions());
        node->setPosition(point);

        return node;
    }

    /**
    * Creates a complete set of positioned label nodes from a feature list.
    */
    osg::Node* createNode(FeatureList& input, const Style& style, FilterContext& context)
    {
        if (style.get<TextSymbol>() == 0L && style.get<IconSymbol>() == 0L)
            return 0L;

        // copy the style so we can (potentially) modify the text symbol.
        Style styleCopy = style;
        auto* text = styleCopy.get<TextSymbol>();
        auto* icon = styleCopy.get<IconSymbol>();
        auto* alt = styleCopy.get<AltitudeSymbol>();

        osg::Group* group = new osg::Group();

        for(auto& feature : input)
        {
            if (!feature)
                continue;

            // run a symbol script if present.
            if (text && text->script().isSet())
            {
                text->script()->eval(feature, context);
            }

            // run a symbol script if present.
            if (icon && icon->script().isSet())
            {
                icon->script()->eval(feature, context);
            }

            const Geometry* geom = feature->getGeometry();
            if (!geom)
                continue;

            Style tempStyle = styleCopy;

            // evaluate expressions into literals.
            // TODO: Later we could replace this with a generate "expression evaluator" type
            // that we could pass to PlaceNode in the DB options. -gw

            bool simple = false;

            if (text)
            {
                if (text->content().isSet())
                    tempStyle.get<TextSymbol>()->content() = text->content()->eval(feature, context);

                if (text->size().isSet())
                    tempStyle.get<TextSymbol>()->size() = text->size()->eval(feature, context);

                if (text->onScreenRotation().isSet())
                    tempStyle.get<TextSymbol>()->onScreenRotation() = text->onScreenRotation()->eval(feature, context);

                if (text->geographicCourse().isSet())
                    tempStyle.get<TextSymbol>()->geographicCourse() = text->geographicCourse()->eval(feature, context);

                if (text->simple() == true)
                    simple = true;
            }

            if (icon)
            {
                if (icon->url().isSet())
                    tempStyle.get<IconSymbol>()->url() = icon->url()->eval(feature, context);

                if (icon->scale().isSet())
                    tempStyle.get<IconSymbol>()->scale() = icon->scale()->eval(feature, context);

                if (icon->heading().isSet())
                    tempStyle.get<IconSymbol>()->heading() = icon->heading()->eval(feature, context);
            }

            if (simple)
            {
                GeoPoint centroid(feature->getSRS(), feature->getGeometry()->getBounds().center(), ALTMODE_RELATIVE);

                auto* mt = new GeoTransform();
                mt->setPosition(centroid);

                auto* node = new osgEarth::Text();
                TextSymbolizer ts(tempStyle.getOrCreate<TextSymbol>());
                ts.apply(node, feature, &context, nullptr);

                node->setCharacterSizeMode(osgEarth::Text::SCREEN_COORDS);
                node->setAutoRotateToScreen(true);
                node->setAlignment(osgText::Text::CENTER_CENTER);
                node->setFont(osgEarth::Registry::instance()->getDefaultFont());

                mt->addChild(node);

                group->addChild(mt);

                if (context.featureIndex())
                {
                    context.featureIndex()->tagNode(node, feature);
                }
            }

            else
            {

                PlaceNode* node = makePlaceNode(
                    context,
                    feature,
                    tempStyle);

                if (node)
                {
                    if (text->priority().isSet())
                    {
                        float val = text->priority()->eval(feature, context);
                        node->setPriority(val >= 0.0f ? val : FLT_MAX);
                    }

                    if (alt && alt->technique() == alt->TECHNIQUE_SCENE && alt->verticalOffset().isSet())
                    {
                        float val = alt->verticalOffset()->eval(feature, context).as(Units::METERS);
                        const osg::Vec3d& off = node->getLocalOffset();
                        node->setLocalOffset(osg::Vec3d(off.x(), off.y(), val));
                    }

                    if (context.featureIndex())
                    {
                        context.featureIndex()->tagNode(node, feature);
                    }

                    group->addChild(node);
                }
            }
        }

        return group;
    }
}

BuildTextFilter::BuildTextFilter(const Style& style) :
    _style(style)
{
    //nop
}

osg::Node*
BuildTextFilter::push( FeatureList& input, FilterContext& context )
{
    const TextSymbol* text = _style.get<TextSymbol>();
    const IconSymbol* icon = _style.get<IconSymbol>();

    if ( !text && !icon )
    {
        OE_WARN << LC << "Insufficient symbology (no TextSymbol/IconSymbol)" << std::endl;
        return 0L;
    }

    return createNode( input, _style, context );
}
