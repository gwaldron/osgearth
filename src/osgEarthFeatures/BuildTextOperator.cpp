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
#if 0
#include <osgEarthFeatures/BuildTextOperator>
#include <osgEarth/Utils>
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
#include <osgUtil/IntersectionVisitor>
#include <osgUtil/PolytopeIntersector>

#define LC "[BuildTextOperator] "

using namespace osgEarth::Symbology;
using namespace osgEarth::Features;

// Parse a content string and replace attributes (marked with delim) with their value
// eg. "City [name], [state]" -> "City San Fransisco, California" if delim is "[]"
static std::string parseAttributes(Feature*           feature, 
                                   const std::string& content,
                                   const std::string& delim)
{
    if (delim.size()<2)
        return content;

    std::string out;
    size_t pos = 0;
    while(true) {
        // Search for an attribute delimiter
        size_t d = content.find_first_of(delim[0], pos);
        if (d==std::string::npos) {
            // No more delimiters? Add remaining text and we're done
            out += content.substr(pos);
            break;
        }
        // An attribute was found!
        // Add non-attribute text
        out += content.substr(pos, d-pos);
        pos = d+1;
        // Extract attribute name
        d = content.find_first_of(delim[1], pos);
        if (d==std::string::npos) {
            // No more delimiters? Add remaining text and we're done
            out += content.substr(pos);
            break;
        }
        // Add attribute and continue
        out += feature->getString( content.substr(pos, d-pos) );
        pos = d+1;
    }

    return out;
}

osg::Node* BuildTextOperator::operator()(const FeatureList&   features, 
                                         const TextSymbol*    symbol,
                                         const FilterContext& context)
{
    if (!symbol) return 0;

    std::set< std::string > labelNames;

    bool removeDuplicateLabels = symbol->removeDuplicateLabels().isSet() ? symbol->removeDuplicateLabels().get() : false;

    StringExpression contentExpr = *symbol->content();

    osg::Geode* result = new osg::Geode;
    for (FeatureList::const_iterator itr = features.begin(); itr != features.end(); ++itr)
    {
        Feature* feature = itr->get();
        if (!feature->getGeometry()) continue;

        std::string text;
        if (symbol->content().isSet())
        {
            //Get the text from the specified content and referenced attributes
            text = feature->eval( contentExpr, &context );
        }

        if (text.empty()) continue;

        //See if there is a duplicate name
        if (removeDuplicateLabels && labelNames.find(text) != labelNames.end()) continue;

        bool rotateToScreen = symbol->rotateToScreen().isSet() ? symbol->rotateToScreen().value() : false;

        // find the centroid
        osg::Vec3d position;
        osg::Quat orientation;

        GeometryIterator gi( feature->getGeometry() );
        while( gi.hasMore() )
        {
            Geometry* geom = gi.next();

            TextSymbol::LinePlacement linePlacement = symbol->linePlacement().isSet() ? symbol->linePlacement().get() : TextSymbol::LINEPLACEMENT_ALONG_LINE;
            if (geom->getType() == Symbology::Geometry::TYPE_LINESTRING && linePlacement == TextSymbol::LINEPLACEMENT_ALONG_LINE)
            {
                //Compute the "middle" of the line string
                LineString* lineString = static_cast<LineString*>(geom);
                double length = lineString->getLength();
                double center = length / 2.0;
                osg::Vec3d start, end;
                if (lineString->getSegment(center, start, end))
                {
                    TextSymbol::LineOrientation lineOrientation = symbol->lineOrientation().isSet() ? symbol->lineOrientation().get() : TextSymbol::LINEORIENTATION_HORIZONTAL;

                    position = (end + start) / 2.0;
                    //We don't want to orient the text at all if we are rotating to the screen
                    if (!rotateToScreen && lineOrientation != TextSymbol::LINEORIENTATION_HORIZONTAL)
                    {
                        osg::Vec3d dir = (end-start);
                        dir.normalize();

                        if (lineOrientation == TextSymbol::LINEORIENTATION_PERPENDICULAR)
                        {
                            osg::Vec3d up(0,0,1);
                            const SpatialReference* srs = context.profile()->getSRS();
                            if (srs && context.isGeocentric() && srs->getEllipsoid())
                            {
                                osg::Vec3d w = context.toWorld( position );
                                up = srs->getEllipsoid()->computeLocalUpVector(w.x(), w.y(), w.z());
                            }
                            dir = up ^ dir;
                        }
                        orientation.makeRotate(osg::Vec3d(1,0,0), dir);
                    }                
                }
                else
                {
                    //Fall back on using the center
                    position = lineString->getBounds().center();
                }
            }
            else
            {
                position = geom->getBounds().center();
            }
        }

        osgText::Text* t = new osgText::Text();
        t->setText( text );

        std::string font = "fonts/arial.ttf";
        if (symbol->font().isSet() && !symbol->font().get().empty())
        {
            font = symbol->font().value();
        }

        t->setFont( font );
        t->setAutoRotateToScreen( rotateToScreen );

        TextSymbol::SizeMode sizeMode = symbol->sizeMode().isSet() ? symbol->sizeMode().get() : TextSymbol::SIZEMODE_SCREEN;
        if (sizeMode == TextSymbol::SIZEMODE_SCREEN) {
            t->setCharacterSizeMode( osgText::TextBase::SCREEN_COORDS );
        }
        else if (sizeMode == TextSymbol::SIZEMODE_OBJECT) {
            t->setCharacterSizeMode( osgText::TextBase::OBJECT_COORDS );
        }
        float size = symbol->size().isSet() ? symbol->size().get() : 32.0f;
        t->setCharacterSize( size );
        /*
        //TODO:  We need to do something to account for autotransformed text that is under a LOD.  Setting the initial bound works sometimes but not all the time.
        if (rotateToScreen)
        {
        //Set the initial bound so that OSG will traverse the text even if it's under an LOD.
        osg::BoundingBox bb;
        bb.expandBy( osg::BoundingSphere(position, size));
        t->setInitialBound( bb);
        }*/
        //t->setCharacterSizeMode( osgText::TextBase::OBJECT_COORDS_WITH_MAXIMUM_SCREEN_SIZE_CAPPED_BY_FONT_HEIGHT );
        //t->setCharacterSize( 300000.0f );
        t->setPosition( position );
        t->setRotation( orientation);
        t->setAlignment( osgText::TextBase::CENTER_CENTER );
        t->getOrCreateStateSet()->setAttributeAndModes( new osg::Depth(osg::Depth::ALWAYS), osg::StateAttribute::ON );
        t->getOrCreateStateSet()->setRenderBinDetails( 99999, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS );

        // apply styling as appropriate:
        osg::Vec4f textColor = symbol->fill()->color();
        osg::Vec4f haloColor = symbol->halo()->color();

        t->setColor( textColor );
        t->setBackdropColor( haloColor );
        t->setBackdropType( osgText::Text::OUTLINE );

        if ( context.isGeocentric() )
        {
            // install a cluster culler
            t->setCullCallback( new CullDrawableByNormal(position * context.inverseReferenceFrame()) );
        }

        if (_hideClutter)
        {
            osg::BoundingBox tBound = t->getBound();
            osg::ref_ptr<osgUtil::PolytopeIntersector> intersector = new osgUtil::PolytopeIntersector(osgUtil::Intersector::MODEL, tBound.xMin(), tBound.yMin(), tBound.xMax(), tBound.yMax());
            osgUtil::IntersectionVisitor intersectVisitor(intersector.get());
            result->accept(intersectVisitor);

            if (!intersector->containsIntersections())
            {
                result->addDrawable( t );
                if (removeDuplicateLabels) labelNames.insert(text);
            }
        }
        else
        {
            result->addDrawable( t );
            if (removeDuplicateLabels) labelNames.insert(text);
        }
    }
    return result;
}
#endif
