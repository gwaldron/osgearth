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
#include <osgEarthFeatures/LabelSource>
#include <osgEarthSymbology/Expression>
#include <osgEarthUtil/Controls>
#include <osgEarth/Utils>
#include <osg/ClusterCullingCallback>
#include <osg/MatrixTransform>
#include <osgDB/FileNameUtils>
#include <set>

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;
using namespace osgEarth::Util;

class OverlayLabelSource : public LabelSource
{
public:
    OverlayLabelSource( const LabelSourceOptions& options ) :
      LabelSource( options )
    {
        //nop
    }

    osg::Node* createNode(
        const FeatureList&   input,
        const TextSymbol*    text,
        const FilterContext& context )
    {
        osg::Group* group = 0L;
        std::set<std::string> used; // to prevent dupes
        bool skipDupes = (text->removeDuplicateLabels() == true);

        StringExpression  contentExpr ( *text->content() );
        NumericExpression priorityExpr( *text->priority() );

        const MapInfo& mi = context.getSession()->getMapInfo();

        for( FeatureList::const_iterator i = input.begin(); i != input.end(); ++i )
        {
            const Feature* feature = i->get();
            if ( !feature )
                continue;

            const Geometry* geom = feature->getGeometry();
            if ( !geom )
                continue;

            osg::Vec3d centroid      = geom->getBounds().center();
            osg::Vec3d centroidWorld = context.toWorld(centroid);

            if ( context.isGeocentric() && geom->getComponentType() != Geometry::TYPE_POINTSET )
            {
                // "clamp" the centroid to the ellipsoid
                osg::Vec3d centroidMap;
                mi.worldPointToMapPoint(centroidWorld, centroidMap);
                centroidMap.z() = 0.0;
                mi.mapPointToWorldPoint(centroidMap, centroidWorld);
                centroid = context.toLocal(centroidWorld);
            }

            const std::string& value = feature->eval( contentExpr );

            if ( !value.empty() && (!skipDupes || used.find(value) == used.end()) )
            {
                if ( !group )
                {
                    group = new osg::Group();
                }

                double priority = feature->eval( priorityExpr );

                Controls::LabelControl* label = new Controls::LabelControl( value );
                if ( text->fill().isSet() )
                    label->setForeColor( text->fill()->color() );
                if ( text->halo().isSet() )
                    label->setBackColor( text->halo()->color() );
                if ( text->size().isSet() )
                    label->setFontSize( *text->size() );
                if ( text->font().isSet() )
                    label->setFont( osgText::readFontFile(*text->font()) );

                Controls::ControlNode* node = new Controls::ControlNode( label, priority );

                osg::MatrixTransform* xform = new osg::MatrixTransform( osg::Matrixd::translate(centroid) );
                xform->addChild( node );

                // for a geocentric map, do a simple dot product cull.
                if ( context.isGeocentric() )
                {
                    osg::Vec3d labelNormal = context.toWorld(centroid);
                    xform->setCullCallback( new CullNodeByHorizon(centroidWorld, mi.getProfile()->getSRS()->getEllipsoid()) );
                    group->addChild( xform );
                }
                else
                {
                    group->addChild( xform );
                }

                if ( skipDupes )
                    used.insert( value );
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
