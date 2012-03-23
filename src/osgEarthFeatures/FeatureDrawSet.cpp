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
#include <osgEarthFeatures/FeatureDrawSet>
#include <osg/Geode>
#include <osg/MatrixTransform>

using namespace osgEarth;
using namespace osgEarth::Features;

#define LC "[FeatureDrawSet] "

//-----------------------------------------------------------------------------

osg::Node*
FeatureDrawSet::createCopy()
{
    osg::Group* group = new osg::Group();

    for( NodeVector::iterator n = _nodes.begin(); n != _nodes.end(); ++n )
    {
        osg::Node* node = *n;
        osg::Node* nodeCopy = osg::clone(node, osg::CopyOp::SHALLOW_COPY);
        osg::Matrix local2world = osg::computeLocalToWorld( node->getParentalNodePaths()[0] );
        if ( !local2world.isIdentity() )
        {
            osg::MatrixTransform* xform = new osg::MatrixTransform(local2world);
            xform->addChild( nodeCopy );
            group->addChild( xform );
        }
        else
        {
            group->addChild( nodeCopy );
        }
    }

    osg::Geode* geode = 0L;
    for( PrimitiveSetGroups::iterator p = _primSetGroups.begin(); p != _primSetGroups.end(); ++p )
    {
        osg::Drawable* d = p->first;
        const PrimitiveSetList& psets = p->second;
        if ( psets.size() > 0 )
        {        
            osg::Geometry* featureGeom = d->asGeometry();

            if ( !geode )
            {
                geode = new osg::Geode();
            }

            osg::Geometry* copiedGeom = new osg::Geometry( *featureGeom, osg::CopyOp::SHALLOW_COPY );
            copiedGeom->setPrimitiveSetList( psets );

            geode->addDrawable( copiedGeom );

            // include a matrix transform if necessary:
            osg::Matrix local2world = osg::computeLocalToWorld( featureGeom->getParent(0)->getParentalNodePaths()[0] );
            if ( !local2world.isIdentity() )
            {
                osg::MatrixTransform* xform = new osg::MatrixTransform(local2world);
                xform->addChild( geode );
                group->addChild( xform );
            }
        }
    }

    return group;
}