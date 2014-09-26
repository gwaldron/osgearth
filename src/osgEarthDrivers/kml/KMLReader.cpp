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
#include "KMLReader"
#include "KML_Root"
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/XmlUtils>
#include <osgEarth/VirtualProgram>
#include <osgEarth/Decluttering>
#include <stack>
#include <iterator>

using namespace osgEarth_kml;
using namespace osgEarth;

KMLReader::KMLReader( MapNode* mapNode, const KMLOptions* options ) :
_mapNode( mapNode ),
_options( options )
{
    //nop
}

osg::Node*
KMLReader::read( std::istream& in, const osgDB::Options* dbOptions )
{
    // pull the URI context out of the DB options:
    URIContext context(dbOptions);

    // read the KML from an XML stream:
    osg::ref_ptr<XmlDocument> xml = XmlDocument::load( in, context );
    if ( !xml.valid() )
        return 0L;

    // convert to a config:
    Config config = xml->getConfig();

    osg::Node* node = read( config, dbOptions );
    node->setName( context.referrer() );

    return node;
}

osg::Node*
KMLReader::read( const Config& conf, const osgDB::Options* dbOptions )
{
    osg::Group* root = new osg::Group();
    root->ref();

    root->setName( conf.referrer() );

    KMLContext cx;
    cx._mapNode   = _mapNode;
    cx._sheet     = new StyleSheet();
    cx._options   = _options;
    cx._srs       = SpatialReference::create( "wgs84", "egm96" );
    cx._groupStack.push( root );

    // clone the dbOptions, and install a resource cache if there isn't one already:
    URIResultCache defaultUriCache;
    if ( !URIResultCache::from(dbOptions) )
    {
        osgDB::Options* newOptions = Registry::instance()->cloneOrCreateOptions();
        defaultUriCache.apply( newOptions );
        cx._dbOptions = newOptions;
    }
    else
    {
        cx._dbOptions = dbOptions;
    }

    // intialize the KML options with the defaults if necessary:
    KMLOptions blankOptions;
    if ( cx._options == 0L )
        cx._options = &blankOptions;

    if ( cx._options->iconAndLabelGroup().valid() && cx._options->declutter() == true )
    {
        Decluttering::setEnabled( cx._options->iconAndLabelGroup()->getOrCreateStateSet(), true );
    }

    const Config* top = conf.hasChild("kml" ) ? conf.child_ptr("kml") : &conf;

    if ( top && !top->empty() )
    {
        KML_Root kmlRoot;
        kmlRoot.scan ( *top, cx );    // first pass
        kmlRoot.scan2( *top, cx );   // second pass
        kmlRoot.build( *top, cx );   // third pass.
    }

    URIResultCache* cacheUsed = URIResultCache::from(cx._dbOptions.get());
    CacheStats stats = cacheUsed->getStats();
    OE_INFO << LC << "URI Cache: " << stats._queries << " reads, " << (stats._hitRatio*100.0) << "% hits" << std::endl;

    return root;
}
