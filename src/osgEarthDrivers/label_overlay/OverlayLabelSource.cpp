/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2014 Pelican Mapping
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
#include <osgEarthFeatures/LabelSource>
#include <osgEarthSymbology/Expression>
#include <osgEarthUtil/Controls>
#include <osgEarth/CullingUtils>
#include <osgEarth/ECEF>
#include <osg/ClusterCullingCallback>
#include <osg/MatrixTransform>
#include <osgDB/FileNameUtils>
#include <set>

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;
using namespace osgEarth::Util;


/******* Deprecated ---- please use AnnotationLabelSource instead *************/


class OverlayLabelSource : public LabelSource
{
public:
    OverlayLabelSource( const LabelSourceOptions& options )
        : LabelSource( options )
    {
        //nop
    }

    /**
     * Creates a simple label. The caller is responsible for placing it in the scene.
     */
    osg::Node* createNode(
        const std::string& text,
        const Style&       style )
    {
        const TextSymbol* symbol = style.get<TextSymbol>();

        Controls::LabelControl* label = new Controls::LabelControl( text );
        if ( symbol )
        {
            if ( symbol->fill().isSet() )
                label->setForeColor( symbol->fill()->color() );
            if ( symbol->halo().isSet() )
                label->setHaloColor( symbol->halo()->color() );
            //if ( symbol->size().isSet() )
            //    label->setFontSize( *symbol->size() );
            if ( symbol->font().isSet() )
            {
                osgText::Font* font = osgText::readFontFile(*symbol->font() );
                // mitigates mipmapping issues that cause rendering artifacts for some fonts/placement
                if ( font )
                    font->setGlyphImageMargin( 2 );
                label->setFont( font );
            }
            if ( symbol->encoding().isSet() )
            {
                osgText::String::Encoding enc;
                switch(symbol->encoding().value())
                {
                case TextSymbol::ENCODING_ASCII: enc = osgText::String::ENCODING_ASCII; break;
                case TextSymbol::ENCODING_UTF8: enc = osgText::String::ENCODING_UTF8; break;
                case TextSymbol::ENCODING_UTF16: enc = osgText::String::ENCODING_UTF16; break;
                case TextSymbol::ENCODING_UTF32: enc = osgText::String::ENCODING_UTF32; break;
                default: enc = osgText::String::ENCODING_UNDEFINED; break;
                }
                label->setEncoding( enc );
            }
        }
        Controls::ControlNode* node = new Controls::ControlNode( label );
        return node;
    }

    /**
     * Creates a complete set of positioned label nodes from a feature list.
     */
    osg::Node* createNode(
        const FeatureList&   input,
        const Style&         style,
        const FilterContext& context )
    {
        const TextSymbol* text = style.get<TextSymbol>();

        osg::Group* group = 0L;
        std::set<std::string> used; // to prevent dupes
        bool skipDupes = (text->removeDuplicateLabels() == true);

        StringExpression  contentExpr ( *text->content() );
        NumericExpression priorityExpr( *text->priority() );

        //bool makeECEF = false;
        const SpatialReference* ecef = 0L;
        if ( context.isGeoreferenced() )
        {
            //makeECEF = context.getSession()->getMapInfo().isGeocentric();
            ecef = context.getSession()->getMapSRS()->getECEF();
        }

        for( FeatureList::const_iterator i = input.begin(); i != input.end(); ++i )
        {
            const Feature* feature = i->get();
            if ( !feature )
                continue;

            const Geometry* geom = feature->getGeometry();
            if ( !geom )
                continue;

            osg::Vec3d centroid  = geom->getBounds().center();

            if ( ecef )
            {
                context.profile()->getSRS()->transform( centroid, ecef, centroid );
                //context.profile()->getSRS()->transformToECEF( centroid, centroid );
            }

            const std::string& value = feature->eval( contentExpr, &context );

            if ( !value.empty() && (!skipDupes || used.find(value) == used.end()) )
            {
                if ( !group )
                {
                    group = new osg::Group();
                }

                double priority = feature->eval( priorityExpr, &context );

                Controls::LabelControl* label = new Controls::LabelControl( value );
                if ( text->fill().isSet() )
                    label->setForeColor( text->fill()->color() );
                if ( text->halo().isSet() )
                    label->setHaloColor( text->halo()->color() );
                //if ( text->size().isSet() )
                //    label->setFontSize( *text->size() );
                if ( text->font().isSet() )
                {
                    // mitigates mipmapping issues that cause rendering artifacts for some fonts/placement
                    osgText::Font* font = osgText::readFontFile(*text->font() );
                    // mitigates mipmapping issues that cause rendering artifacts for some fonts/placement
                    if ( font )
                        font->setGlyphImageMargin( 2 );
                    label->setFont( font );
                }

                Controls::ControlNode* node = new Controls::ControlNode( label, priority );

                osg::MatrixTransform* xform = new osg::MatrixTransform( osg::Matrixd::translate(centroid) );
                xform->addChild( node );

                // for a geocentric map, do a simple dot product cull.
                if ( ecef )
                {
                    xform->setCullCallback( new CullNodeByHorizon(
                        centroid, 
                        ecef->getEllipsoid() ) );
                        //context.getSession()->getMapSRS()->getEllipsoid() ) ); //getMapInfo().getProfile()->getSRS()->getEllipsoid()) );
                    group->addChild( xform );
                }
                else
                {
                    group->addChild( xform );
                }

                if ( skipDupes )
                {
                    used.insert( value );
                }
            }
        }

        return group;
    }
};

//------------------------------------------------------------------------

class OverlayLabelSourceDriver : public LabelSourceDriver
{
public:
    OverlayLabelSourceDriver()
    {
        supportsExtension( "osgearth_label_overlay", "osgEarth overlay label plugin" );
    }

    virtual const char* className()
    {
        return "osgEarth Overlay Label Plugin";
    }

    virtual ReadResult readObject(const std::string& file_name, const Options* options) const
    {
        if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
            return ReadResult::FILE_NOT_HANDLED;

        return new OverlayLabelSource( getLabelSourceOptions(options) );
    }
};

REGISTER_OSGPLUGIN(osgearth_label_overlay, OverlayLabelSourceDriver)
