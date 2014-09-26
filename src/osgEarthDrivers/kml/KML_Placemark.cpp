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
#include "KML_Placemark"
#include "KML_Geometry"
#include "KML_Style"

#include <osgEarthAnnotation/FeatureNode>
#include <osgEarthAnnotation/PlaceNode>
#include <osgEarthAnnotation/LabelNode>
#include <osgEarthAnnotation/ModelNode>
#include <osgEarthAnnotation/LocalGeometryNode>
#include <osgEarth/Decluttering>

#include <osg/Depth>
#include <osgDB/WriteFile>

using namespace osgEarth_kml;
using namespace osgEarth::Features;
using namespace osgEarth::Annotation;

void 
KML_Placemark::build( const Config& conf, KMLContext& cx )
{
	Style masterStyle;

	if (conf.hasValue("styleurl"))
	{	// process a "stylesheet" style
		const Style* ref_style = cx._sheet->getStyle( conf.value("styleurl"), false );
		if (ref_style)
		{
			masterStyle = masterStyle.combineWith(*ref_style);
		}
	}
	if ( conf.hasChild("style") )
	{	// process an "inline" style
		KML_Style kmlStyle;
		kmlStyle.scan(conf.child("style"), cx);
		masterStyle = masterStyle.combineWith(cx._activeStyle);
	}

    // parse the geometry. the placemark must have geometry to be valid. The 
    // geometry parse may optionally specify an altitude mode as well.
    KML_Geometry geometry;
    geometry.build(conf, cx, masterStyle);

    Geometry* allGeom = geometry._geom.get();
    if ( allGeom )
    {
        GeometryIterator giter( allGeom, false );
        while( giter.hasMore() )
        {
            Geometry* geom = giter.next();
            Style style = masterStyle;

            AltitudeSymbol* alt = style.get<AltitudeSymbol>();
            
            if ( geom && geom->getTotalPointCount() > 0 )
            {
                // resolve the proper altitude mode for the anchor point
                AltitudeMode altMode = ALTMODE_RELATIVE;
                if (alt && 
                    !alt->clamping().isSetTo( alt->CLAMP_TO_TERRAIN ) &&
                    !alt->clamping().isSetTo( alt->CLAMP_RELATIVE_TO_TERRAIN ) )
                {
                    altMode = ALTMODE_ABSOLUTE;
                }

                GeoPoint position(cx._srs.get(), geom->getBounds().center(), altMode);

                bool isPoly = geom->getComponentType() == Geometry::TYPE_POLYGON;
                bool isPoint = geom->getComponentType() == Geometry::TYPE_POINTSET;

                // check for symbols.
                ModelSymbol*    model = style.get<ModelSymbol>();
                IconSymbol*     icon  = style.get<IconSymbol>();
                TextSymbol*     text  = style.get<TextSymbol>();

                if ( !text && cx._options->defaultTextSymbol().valid() )
                    text = cx._options->defaultTextSymbol().get();

                // the annotation name:
                std::string name = conf.hasValue("name") ? conf.value("name") : "";
                if ( text && !name.empty() )
                {
                    text->content()->setLiteral( name );
                }

                AnnotationNode* featureNode = 0L;
                AnnotationNode* iconNode    = 0L;
                AnnotationNode* modelNode   = 0L;

                // one coordinate? It's a place marker or a label.
                if ( model || icon || text || geom->getTotalPointCount() == 1 )
                {
                    // load up the default icon if there we don't have one.
                    if ( !model && !icon )
                    {
                        icon = cx._options->defaultIconSymbol().get();
                        if ( icon )
                            style.add( icon );
                    }

                    // if there's a model, render that - models do NOT get labels.
                    if ( model )
                    {
                        ModelNode* node = new ModelNode( cx._mapNode, style, cx._dbOptions );
                        node->setPosition( position );

                        // model scale:
                        if ( cx._options->modelScale() != 1.0f )
                        {
                            float s = *cx._options->modelScale();
                            node->setScale( osg::Vec3f(s,s,s) );
                        }

                        // model local tangent plane rotation:
                        if ( !cx._options->modelRotation()->zeroRotation() )
                        {
                            node->setLocalRotation( *cx._options->modelRotation() );
                        }

                        modelNode = node;
                    }

                    // is there a label?
                    else if ( !text && !name.empty() )
                    {
                        text = style.getOrCreate<TextSymbol>();
                        text->content()->setLiteral( name );
                    }

                    // is there an icon?
                    if ( icon )
                    {
                        iconNode = new PlaceNode( cx._mapNode, position, style, cx._dbOptions );
                    }

                    else if ( !model && text && !name.empty() )
                    {
                        // note: models do not get labels.
                        iconNode = new LabelNode( cx._mapNode, position, style );
                    }
                }

                // multiple coords? feature:
                if ( geom->getTotalPointCount() > 1 )
                {
                    ExtrusionSymbol* extruded = style.get<ExtrusionSymbol>();

                    // Remove symbols that we have already processed so the geometry
                    // compiler doesn't get confused.
                    if ( model )
                        style.removeSymbol( model );
                    if ( icon )
                        style.removeSymbol( icon );
                    if ( text )
                        style.removeSymbol( text );

                    Feature* feature = new Feature(geom, cx._srs.get(), style);
                    featureNode = new FeatureNode( cx._mapNode, feature );
                }


                // assemble the results:
                if ( (iconNode || modelNode) && featureNode )
                {
                    osg::Group* group = new osg::Group();
                    group->addChild( featureNode );
                    if ( iconNode )
                        group->addChild( iconNode );
                    if ( modelNode )
                        group->addChild( modelNode );

                    cx._groupStack.top()->addChild( group );

                    if ( iconNode && cx._options->declutter() == true )
                    {
                        Decluttering::setEnabled( iconNode->getOrCreateStateSet(), true );
                    }

                    if ( iconNode )
                        KML_Feature::build( conf, cx, iconNode );
                    if ( modelNode )
                        KML_Feature::build( conf, cx, modelNode );
                    if ( featureNode )
                        KML_Feature::build( conf, cx, featureNode );
                }

                else
                {
                    if ( iconNode )
                    {
                        if ( cx._options->iconAndLabelGroup().valid() )
                        {
                            cx._options->iconAndLabelGroup()->addChild( iconNode );
                        }
                        else
                        {
                            cx._groupStack.top()->addChild( iconNode );
                            if ( cx._options->declutter() == true )
                            {
                                Decluttering::setEnabled( iconNode->getOrCreateStateSet(), true );
                            }
                        }
                        KML_Feature::build( conf, cx, iconNode );
                    }
                    if ( modelNode )
                    {
                        cx._groupStack.top()->addChild( modelNode );
                        KML_Feature::build( conf, cx, modelNode );
                    }
                    if ( featureNode )
                    {
                        cx._groupStack.top()->addChild( featureNode );
                        KML_Feature::build( conf, cx, featureNode );
                    }
                }
            }
        }
    }
}
