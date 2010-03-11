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
#include <osgEarthSymbology/MarkerSymbolizer>
#include <osgEarthSymbology/MarkerSymbol>
#include <osgEarthFeatures/Feature>
#include <osgDB/ReadFile>
#include <osg/Geometry>
#include <osg/MatrixTransform>
#include <osg/Material>
#include <osg/Geode>

using namespace osgEarth::Symbology;
using namespace osgEarth::Features;

MarkerSymbolizer::MarkerSymbolizer()
{
}


bool 
MarkerSymbolizer::update(FeatureDataSet* dataSet,
                         const osgEarth::Symbology::Style* style,
                         osg::Group* attachPoint,
                         SymbolizerContext* context )
{
    if (!dataSet || !attachPoint || !style)
        return false;

    osg::ref_ptr<osgEarth::Features::FeatureCursor> cursor = dataSet->createCursor();
    if (!cursor)
        return false;

    osg::ref_ptr<osg::Group> newSymbolized = new osg::Group;

    osgEarth::Features::Feature* feature = 0;
    while( cursor->hasMore() )
    {
        feature = cursor->nextFeature();
        if (!feature || !feature->getGeometry())
            continue;

        Geometry* geometry = feature->getGeometry();

        GeometryIterator geomIterator( geometry );
        geomIterator.traverseMultiGeometry() = true;
        geomIterator.traversePolygonHoles() = true;

        while( geomIterator.hasMore() )
        {
            Geometry* part = geomIterator.next();
            if (!part)
                continue;

            switch( part->getType())
            {
            case Geometry::TYPE_POINTSET:
                if (style->getPoint())
                {
                    const MarkerSymbol* point = dynamic_cast<const MarkerSymbol*>(style->getPoint());
                    if (point && part->size() && !point->marker().value().empty())
                    {
                        osg::ref_ptr<osgDB::Options> options = new osgDB::Options;
                        options->setObjectCacheHint(osgDB::Options::CACHE_ALL);
                        osg::Node* node = osgDB::readNodeFile(point->marker().value(), options.get());
                        if (!node) {
                            osg::notify(osg::WARN) << "can't load Marker Node " << point->marker().value() << std::endl;
                            continue;
                        }
                        osg::Group* group = new osg::Group;
                        for ( osg::Vec3dArray::iterator it = part->begin(); it != part->end(); ++it)
                        {
                            osg::Vec3d pos = *it;
                            osg::MatrixTransform* tr = new osg::MatrixTransform;
                            tr->setMatrix(osg::Matrix::translate(pos));
                            tr->addChild(node);
                            group->addChild(tr);
                        }
                        newSymbolized->addChild(group);
                    }
                }
                break;

            case Geometry::TYPE_LINESTRING:
                break;

            case Geometry::TYPE_RING:
                break;

            case Geometry::TYPE_POLYGON:
                break;
            }
        }
    }

    if (newSymbolized->getNumChildren())
    {
        attachPoint->removeChildren(0, attachPoint->getNumChildren());
        attachPoint->addChild(newSymbolized.get());
        return true;
    }

    return false;
}
