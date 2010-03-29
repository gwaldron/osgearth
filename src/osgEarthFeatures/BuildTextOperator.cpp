/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2009 Pelican Ventures, Inc.
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
#include <osgEarthSymbology/GeometryInput>
#include <osgEarthFeatures/BuildTextOperator>
#include <osgEarthFeatures/Annotation>
#include <osgEarthSymbology/GeometrySymbol>
#include <osgDB/ReadFile>
#include <osgDB/ReaderWriter>
#include <osg/Geometry>
#include <osg/MatrixTransform>
#include <osg/Material>
#include <osg/Geode>
#include <osg/Version>
#include <osg/Geometry>
#include <osg/LineWidth>
#include <osg/Point>
#include <osg/Depth>
#include <osg/PolygonOffset>
#include <osg/ClusterCullingCallback>
#include <osgText/Text>



using namespace osgEarth::Symbology;
using namespace osgEarth::Features;

osg::Node* BuildTextOperator::operator()(const FeatureList& features, 
                                         const TextSymbol *symbol,
                                         const FilterContext& context)
{
    if (!symbol) return 0;

    osg::Geode* result = new osg::Geode;
    for (FeatureList::const_iterator itr = features.begin(); itr != features.end(); ++itr)
    {
        Feature* feature = itr->get();
        if (!feature->getGeometry()) continue;

        std::string text;
        //If the feature is a TextAnnotation, just get the value from it
        TextAnnotation* annotation = dynamic_cast<TextAnnotation*>(feature);
        if (annotation)
        {
            text = annotation->text();
        }
        else if (symbol->attribute().isSet())
        {
            //Get the text from the specified attribute
            std::string attr = symbol->attribute().value();
            text = feature->getAttr(attr);
        }

        if (text.empty()) continue;

        // find the centroid
        osg::Vec3d centroid = feature->getGeometry()->getBounds().center();
        
        osgText::Text* t = new osgText::Text();
        t->setText( text );

        std::string font = "fonts/arial.ttf";
        if (symbol->font().isSet() && !symbol->font().get().empty())
        {
            font = symbol->font().value();
        }

        t->setFont( font );
        t->setAutoRotateToScreen( true );
        t->setCharacterSizeMode( osgText::TextBase::SCREEN_COORDS );
        float size = symbol->size().isSet() ? symbol->size().get() : 32.0f;
        t->setCharacterSize( size );
        //t->setCharacterSizeMode( osgText::TextBase::OBJECT_COORDS_WITH_MAXIMUM_SCREEN_SIZE_CAPPED_BY_FONT_HEIGHT );
        //t->setCharacterSize( 300000.0f );
        t->setPosition( centroid );
        t->setAlignment( osgText::TextBase::CENTER_CENTER );
        t->getOrCreateStateSet()->setAttributeAndModes( new osg::Depth(osg::Depth::ALWAYS), osg::StateAttribute::ON );
        t->getOrCreateStateSet()->setRenderBinDetails( 99999, "RenderBin" );

        // apply styling as appropriate:
        osg::Vec4f textColor = symbol->fill()->color();
        osg::Vec4f haloColor = symbol->halo()->color();

        t->setColor( textColor );
        t->setBackdropColor( haloColor );
        t->setBackdropType( osgText::Text::OUTLINE );

        //TODO:  Make cluster culling work correctly
        /*if ( context.isGeocentric() )
        {
            // install a cluster culler: note that the CCC control point and normal must be
            // in world coordinates
            const osg::EllipsoidModel* ellip = context.profile()->getSRS()->getEllipsoid();
            osg::Vec3d cp = centroid * context.inverseReferenceFrame();
            //double lat, lon, height;
            //ellip->convertXYZToLatLongHeight(cp.x(), cp.y(), cp.z(), lat, lon, height);
            //OE_NOTICE << text << " inverse " << cp << ": " << osg::RadiansToDegrees(lat) << "," << osg::RadiansToDegrees(lon) << "," << height << std::endl;
            osg::Vec3d normal = ellip->computeLocalUpVector( cp.x(), cp.y(), cp.z() );
            osg::ClusterCullingCallback* ccc = new osg::ClusterCullingCallback( cp, normal, 0.0f );
            t->setCullCallback( ccc );
        }*/

        result->addDrawable( t );
    }
    return result;
}
