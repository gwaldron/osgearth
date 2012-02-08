/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2010 Pelican Mapping
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
#include <osgEarthAnnotation/Decluttering>

#include <osg/Depth>

using namespace osgEarth::Features;
using namespace osgEarth::Annotation;

void 
KML_Placemark::build( const Config& conf, KMLContext& cx )
{
    Style style;
    if ( conf.hasValue("styleurl") )
    {
        // process a "stylesheet" style
        const Style* ref_style = cx._sheet->getStyle( conf.value("styleurl"), false );
        if ( ref_style )
            style = *ref_style;
    }
    else if ( conf.hasChild("style") )
    {
        // process an "inline" style
        KML_Style kmlStyle;
        kmlStyle.scan( conf.child("style"), cx );
        style = cx._activeStyle;
    }

    // KML's default altitude mode is clampToGround.
    if ( style.get<AltitudeSymbol>() == 0L )
        style.getOrCreate<AltitudeSymbol>()->clamping() = AltitudeSymbol::CLAMP_TO_TERRAIN;

    // parse the geometry. the placemark must have geometry to be valid.
    KML_Geometry geometry;
    geometry.build(conf, cx, style);
    
    if ( geometry._geom.valid() && geometry._geom->getTotalPointCount() > 0 )
    {
        Geometry* geom = geometry._geom.get();

        GeoPoint position(cx._srs.get(), geom->getBounds().center());
        //osg::Vec3d position = geom->getBounds().center();
        bool isPoly = geom->getComponentType() == Geometry::TYPE_POLYGON;
        bool isPoint = geom->getComponentType() == Geometry::TYPE_POINTSET;

        // read in the Marker if there is one.
        URI                      markerURI;
        osg::ref_ptr<osg::Image> markerImage;
        osg::ref_ptr<osg::Node>  markerModel;

        MarkerSymbol* marker = style.get<MarkerSymbol>();    
        if ( marker && marker->url().isSet() )
        {
            markerURI = URI( marker->url()->expr(), marker->url()->uriContext() );
            ReadResult result = marker->isModel() == true? markerURI.readNode() : markerURI.readImage();
            markerImage = result.getImage();
            markerModel = result.getNode();

            // We can't leave the marker symbol in the style, or the GeometryCompiler will
            // think we want to do Point-model substitution. So remove it. A bit of a hack
            if ( marker )
                style.removeSymbol(marker);
        }

        std::string text = 
            conf.hasValue("name") ? conf.value("name") :
            conf.hasValue("description") ? conf.value("description") :
            "Unnamed";  

        FeatureNode*    fNode = 0L;
        AnnotationNode* pNode = 0L;

        // place a 3D model:
        if ( markerModel.valid() )
        {
            Feature* feature = new Feature(geometry._geom.get(), cx._srs.get(), style);
            fNode = new FeatureNode( cx._mapNode, feature, false );
        }

        // Place node (icon + text) or Label node (text only)
        else if ( marker || geometry._geom->getTotalPointCount() == 1 )
        {
            if ( !markerImage.valid() )
            {
                markerImage = cx._options->defaultIconImage().get();
                if ( !markerImage.valid() )
                {
                    markerImage = cx._options->defaultIconURI()->getImage();
                }
            }
            
            if ( !style.get<TextSymbol>() && cx._options->defaultTextSymbol().valid() )
            {
                style.addSymbol( cx._options->defaultTextSymbol().get() );
            }

            if ( markerImage.valid() )
                pNode = new PlaceNode( cx._mapNode, position, markerImage.get(), text, style );
            else
                pNode = new LabelNode( cx._mapNode, position, text, style );
        }

        if ( geometry._geom->getTotalPointCount() > 1 )
        {
            const ExtrusionSymbol* ex = style.get<ExtrusionSymbol>();
            const AltitudeSymbol* alt = style.get<AltitudeSymbol>();        

            bool draped =
                isPoly   && 
                ex == 0L && 
                (alt == 0L || alt->clamping() == AltitudeSymbol::CLAMP_TO_TERRAIN);

            // Make a feature node; drape if we're not extruding.
            GeometryCompilerOptions options;
            options.clustering() = false;            
            Feature* feature = new Feature(geometry._geom.get(), cx._srs.get(), style);
            fNode = new FeatureNode( cx._mapNode, feature, draped, options );

            if ( !ex )
            {
                fNode->getOrCreateStateSet()->setMode(GL_LIGHTING, 0);
            }
        }
        
        if ( pNode && fNode )
        {
            osg::Group* group = new osg::Group();
            group->addChild( fNode );
            group->addChild( pNode );
            cx._groupStack.top()->addChild( group );
            if ( cx._options->declutter() == true )
                Decluttering::setEnabled( pNode->getOrCreateStateSet(), true );
            KML_Feature::build( conf, cx, pNode );
            KML_Feature::build( conf, cx, fNode );
        }

        else if ( pNode )
        {
            if ( cx._options->iconAndLabelGroup().valid() )
            {
                cx._options->iconAndLabelGroup()->addChild( pNode );
            }
            else
            {
                cx._groupStack.top()->addChild( pNode );
                if ( cx._options->declutter() == true )
                    Decluttering::setEnabled( pNode->getOrCreateStateSet(), true );
            }
            KML_Feature::build( conf, cx, pNode );
        }

        else if ( fNode )
        {
            cx._groupStack.top()->addChild( fNode );
            KML_Feature::build( conf, cx, fNode );
        }
    }
}
