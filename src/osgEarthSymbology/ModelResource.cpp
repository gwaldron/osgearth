/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2015 Pelican Mapping
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
#include <osgEarth/Utils>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgUtil/Optimizer>
#include <osg/ComputeBoundsVisitor>

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

const osg::BoundingBox&
ModelResource::getBoundingBox(const osgDB::Options* dbo)
{
    if ( !_bbox.valid() )
    {
        Threading::ScopedMutexLock lock(_mutex);
        if ( !_bbox.valid() )
        {
            osg::ref_ptr<osg::Node> node = createNodeFromURI( uri().get(), dbo );
            if ( node.valid() )
            {
                osg::ComputeBoundsVisitor cbv;
                node->accept(cbv);
                _bbox = cbv.getBoundingBox();
            }
        }
    }
    return _bbox;
}

osg::Node*
ModelResource::createNodeFromURI( const URI& uri, const osgDB::Options* dbOptions ) const
{
    osg::ref_ptr< osgDB::Options > options = dbOptions ? new osgDB::Options( *dbOptions ) : 0L;

    // Explicitly cache images so that models that share images will only load one copy.
    if ( options.valid() )
    {
        options->setObjectCacheHint( osgDB::Options::CACHE_IMAGES );
    }
    osg::Node* node = 0L;

    ReadResult r = uri.readNode( options.get() );
    if ( r.succeeded() )
    {
        node = r.releaseNode();
        
        OE_INFO << LC << "Loaded " << uri.base() << "(from " << (r.isFromCache()? "cache" : "source") << ")"
            << std::endl;

        osgUtil::Optimizer o;
        o.optimize( node,
            o.DEFAULT_OPTIMIZATIONS |
            o.INDEX_MESH |
            o.VERTEX_PRETRANSFORM |
            o.VERTEX_POSTTRANSFORM );
    }
    else // failing that, fall back on the old encoding format..
    {
        StringVector tok;
        StringTokenizer( *uri, tok, "()" );
        if (tok.size() >= 2)
        {
            node = createNodeFromURI( URI(tok[1]), options.get() );
        }
    }

    return node;
}
