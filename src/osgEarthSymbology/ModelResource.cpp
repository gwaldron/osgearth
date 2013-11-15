/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2013 Pelican Mapping
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
#include <osgEarthSymbology/ModelResource>
#include <osgEarth/StringUtils>
#include <osgUtil/Optimizer>
#include <osgEarth/Utils>

#define LC "[ModelResource] "

using namespace osgEarth;
using namespace osgEarth::Symbology;

//---------------------------------------------------------------------------

ModelResource::ModelResource( const Config& conf ) :
InstanceResource( conf )
{
    mergeConfig( conf );
}

void
ModelResource::mergeConfig( const Config& conf )
{
    //nop
}

Config
ModelResource::getConfig() const
{
    Config conf = InstanceResource::getConfig();
    conf.key() = "model";
    //nop
    return conf;
}

osg::Node*
ModelResource::createNodeFromURI( const URI& uri, const osgDB::Options* dbOptions ) const
{
    osg::Node* node = 0L;

    ReadResult r = uri.getNode( dbOptions );
    if ( r.succeeded() )
    {
        node = r.releaseNode();

        OE_DEBUG << LC << "Loading " << uri.full() << std::endl;

#if 1
        osgUtil::Optimizer o;
        o.optimize( node,
            o.DEFAULT_OPTIMIZATIONS |
            o.INDEX_MESH |
            o.VERTEX_PRETRANSFORM |
            o.VERTEX_POSTTRANSFORM );
            
#else
        osgUtil::Optimizer o;
        o.optimize( node, osgUtil::Optimizer::INDEX_MESH );

        // GPU performance optimization:
        VertexCacheOptimizer vco;
        node->accept( vco );
#endif
    }
    else // failing that, fall back on the old encoding format..
    {
        StringVector tok;
        StringTokenizer( *uri, tok, "()" );
        if (tok.size() >= 2)
            return createNodeFromURI( URI(tok[1]), dbOptions );
    }

    return node;
}
