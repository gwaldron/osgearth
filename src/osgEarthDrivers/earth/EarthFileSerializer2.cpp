/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2016 Pelican Mapping
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
#include "EarthFileSerializer"
#include <osgEarth/FileUtils>
#include <osgEarth/MapFrame>
#include <osgEarth/Extension>
#include <osgEarth/StringUtils>
#include <osgEarth/FileUtils>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>
#include <stdio.h>
#include <ctype.h>

using namespace osgEarth_osgearth;
using namespace osgEarth;

#undef  LC
#define LC "[EarthSerializer2] "

static const char * const PATH_SEPARATORS = "/\\";
static unsigned int PATH_SEPARATORS_LEN = 2;

namespace
{

	class PathIterator
    {
		public:
			PathIterator(const std::string & v);
			bool valid() const { return start!=end; }
			PathIterator & operator++();
			std::string operator*();

		protected:
			std::string::const_iterator end;     ///< End of path string
			std::string::const_iterator start;   ///< Points to the first char of an element, or ==end() if no more
			std::string::const_iterator stop;    ///< Points to the separator after 'start', or ==end()

			/// Iterate until 'it' points to something different from a separator
			std::string::const_iterator skipSeparators(std::string::const_iterator it);
			std::string::const_iterator next(std::string::const_iterator it);
    };

	PathIterator::PathIterator(const std::string & v) : end(v.end()), start(v.begin()), stop(v.begin()) { operator++(); }

	PathIterator & PathIterator::operator++()
	{
		if (!valid()) return *this;
			start = skipSeparators(stop);
		if (start != end) stop = next(start);
		return *this;
	}
	std::string PathIterator::operator*()
	{
		if (!valid()) return std::string();
		return std::string(start, stop);
	}

	std::string::const_iterator PathIterator::skipSeparators(std::string::const_iterator it)
	{
		for (; it!=end && std::find_first_of(it, it+1, PATH_SEPARATORS, PATH_SEPARATORS+PATH_SEPARATORS_LEN) != it+1; ++it) {}
		return it;
	}

	std::string::const_iterator PathIterator::next(std::string::const_iterator it)
	{
		return std::find_first_of(it, end, PATH_SEPARATORS, PATH_SEPARATORS+PATH_SEPARATORS_LEN);
	}

    bool isReservedWord(const std::string& k)
    {
        return
            k == "options" ||
            k == "image" ||
            k == "elevation" ||
            k == "heightfield" ||
            k == "model" ||
            k == "mask" ||
            k == "external" ||
            k == "extensions";
    }

    /**
     * Looks at each key in a Config and tries to match that key to a shared library name;
     * loads the shared library associated with the name. This will "pre-load" all the DLLs
     * associated with extensions in the earth file even if they weren't linked.
     *
     * Will also pre-load any expressly indicated shared libraries in the "libraries" element.
     */
    void preloadExtensionLibs(const Config& conf)
    {
        for(ConfigSet::const_iterator i = conf.children().begin(); i != conf.children().end(); ++i)
        {
            const std::string& name = i->key();

            if ( isReservedWord(name) )
                continue;

            if ( !name.empty() )
            {
                // Load the extension library if necessary.
                std::string libName = osgDB::Registry::instance()->createLibraryNameForExtension("osgearth_" + name);
                osgDB::Registry::LoadStatus status = osgDB::Registry::instance()->loadLibrary(libName);
                if ( status == osgDB::Registry::LOADED )
                {
                    OE_INFO << LC << "Loaded extension lib \"" << libName << "\"\n";
                }
                else
                {
                    // If it failed to load, try loading an extension from an osgEarth library with the same name.
                    // Capitalize the name of the extension,.
                    std::string capName = name;
                    capName[0] = ::toupper(capName[0]);
                    std::stringstream buf;
                    buf << "osgEarth" << capName;
                    libName = osgDB::Registry::instance()->createLibraryNameForNodeKit(buf.str());
                    status = osgDB::Registry::instance()->loadLibrary(libName);
                    if (status == osgDB::Registry::LOADED)
                    {
                        OE_INFO << LC << "Loaded extension lib \"" << libName << "\"\n";
                    }
                }
            }
        }

        // Preload any libraries
        Config libraries = conf.child("libraries");
        if (!libraries.value().empty())
        {
            StringTokenizer izer( ";" );
            StringVector libs;
            izer.tokenize( libraries.value(), libs );
            for (StringVector::iterator itr = libs.begin(); itr != libs.end(); ++itr)
            {
                std::string lib = *itr;
                trim2(lib);
                std::string libName = osgDB::Registry::instance()->createLibraryNameForNodeKit(lib);
                osgDB::Registry::LoadStatus status = osgDB::Registry::instance()->loadLibrary(libName);
                if (status == osgDB::Registry::LOADED)
                {
                    OE_INFO << LC << "Loaded library \"" << libName << "\"\n";
                }
                else
                {
                    OE_WARN << LC << "Failed to load library \"" << libName << "\"\n";
                }
            }
        }        
    }

    /**
     * Visits a Config hierarchy and rewrites relative pathnames to be relative to a new referrer.
     */
    struct RewritePaths
    {
        bool        _rewriteAbsolutePaths;
        std::string _newReferrerAbsPath;
        std::string _newReferrerFolder;

        RewritePaths(const std::string& referrer)
        {
            _rewriteAbsolutePaths = false;
            _newReferrerAbsPath = osgDB::convertFileNameToUnixStyle( osgDB::getRealPath(referrer) );
            _newReferrerFolder  = osgDB::getFilePath( osgDB::findDataFile(_newReferrerAbsPath) );
        }

        /** Whether to make absolute paths into relative paths if possible */
        void setRewriteAbsolutePaths(bool value)
        {
            _rewriteAbsolutePaths = value;
        }

        bool isLocation(const Config& input) const
        {
            if ( input.value().empty() )
                return false;

            if ( input.referrer().empty() )
                return false;

            return 
                input.key() == "url"      ||
                input.key() == "uri"      ||
                input.key() == "href"     ||
                input.key() == "filename" ||
                input.key() == "file"     ||
                input.key() == "pathname" ||
                input.key() == "path";
        }

        void apply(Config& input)
        {
            if ( isLocation(input) )
            {
                // resolve the absolute path of the input:
                URI inputURI( input.value(), URIContext(input.referrer()) );

                std::string newValue = resolve(inputURI);
                if ( newValue != input.value() )
                {
                    input.value() = newValue;
                    input.setReferrer( _newReferrerAbsPath );
                }

                if ( !input.externalRef().empty() )
                {
                    URI xrefURI( input.externalRef(), URIContext(input.referrer()) );
                    std::string newXRef = resolve(xrefURI);
                    if ( newXRef != input.externalRef() )
                    {
                        input.setExternalRef( newXRef );
                        input.setReferrer( _newReferrerAbsPath );
                    }
                }
            }

            for(ConfigSet::iterator i = input.children().begin(); i != input.children().end(); ++i)
            {
                apply( *i );
            }
        }

        std::string resolve(const URI& inputURI )
        {
            std::string inputAbsPath = osgDB::convertFileNameToUnixStyle( inputURI.full() );

            // if the abs paths have different roots, no resolution; use the input.
            if (osgDB::getPathRoot(inputAbsPath) != osgDB::getPathRoot(_newReferrerFolder))
            {
                return inputURI.full();
            }

            // see whether the file exists (this is how we verify that it's actually a path)
            //if ( osgDB::fileExists(inputAbsPath) )
            {
                if ( !osgDB::isAbsolutePath(inputURI.base()) || _rewriteAbsolutePaths )
                {
                    std::string inputNewRelPath = getPathRelative( _newReferrerFolder, inputAbsPath );
                    
                    //OE_DEBUG << LC << "\n"
                    //    "   Rewriting \"" << input.value() << "\" as \"" << inputNewRelPath << "\"\n"
                    //    "   Absolute = " << inputAbsPath << "\n"
                    //    "   ReferrerFolder = " << _newReferrerFolder << "\n";

                    return inputNewRelPath;
                }
            }

            return inputURI.base();
        }

		std::string getPathRelative(const std::string& from, const std::string& to)
		{
			// This implementation is not 100% robust, and should be replaced with C++0x "std::path" as soon as possible.

			// Definition: an "element" is a part between slashes. Ex: "/a/b" has two elements ("a" and "b").
			// Algorithm:
			// 1. If paths are neither both absolute nor both relative, then we cannot do anything (we need to make them absolute, but need additionnal info on how to make it). Return.
			// 2. If both paths are absolute and root isn't the same (for Windows only, as roots are of the type "C:", "D:"), then the operation is impossible. Return.
			// 3. Iterate over two paths elements until elements are equal
			// 4. For each remaining element in "from", add ".." to result
			// 5. For each remaining element in "to", add this element to result

			// 1 & 2
			const std::string root = osgDB::getPathRoot(from);
			if (root != osgDB::getPathRoot(to)) {
				OSG_INFO << "Cannot relativise paths. From=" << from << ", To=" << to << ". Returning 'to' unchanged." << std::endl;
				//return to;
				return osgDB::getSimpleFileName(to);
			}

			// 3
			PathIterator itFrom(from), itTo(to);
			// Iterators may point to Windows roots. As we tested they are equal, there is no need to ++itFrom and ++itTo.
			// However, if we got an Unix root, we must add it to the result.
			// std::string res(root == "/" ? "/" : "");
			// Since result is a relative path, even in unix, no need to add / to the result first.
			std::string res = "";
			for(; itFrom.valid() && itTo.valid() && *itFrom==*itTo; ++itFrom, ++itTo) {}

			// 4
			for(; itFrom.valid(); ++itFrom) res += "../";

			// 5
			for(; itTo.valid(); ++itTo) res += *itTo + "/";

			// Remove trailing slash before returning
			if (!res.empty() && std::find_first_of(res.rbegin(), res.rbegin()+1, PATH_SEPARATORS, PATH_SEPARATORS+PATH_SEPARATORS_LEN) != res.rbegin()+1)
			{
				return res.substr(0, res.length()-1);
			}
			return res;
			}
    };
}

//............................................................................

namespace
{
    void addImageLayer(const Config& conf, Map* map)
    {
        ImageLayerOptions options( conf );
        options.name() = conf.value("name");
        ImageLayer* layer = new ImageLayer(options);
        map->addImageLayer(layer);
        if (layer->getStatus().isError())
            OE_WARN << LC << "Layer \"" << layer->getName() << "\" : " << layer->getStatus().toString() << std::endl;
    }

    void addElevationLayer(const Config& conf, Map* map)
    {
        ElevationLayerOptions options( conf );
        options.name() = conf.value( "name" );
        ElevationLayer* layer = new ElevationLayer(options);
        map->addElevationLayer(layer);
        if (layer->getStatus().isError())
            OE_WARN << LC << "Layer \"" << layer->getName() << "\" : " << layer->getStatus().toString() << std::endl;
    }

    void addModelLayer(const Config& conf, Map* map)
    {
        ModelLayerOptions options( conf );
        options.name() = conf.value( "name" );
        options.driver() = ModelSourceOptions( conf );
        ModelLayer* layer = new ModelLayer(options);
        map->addModelLayer(layer);
        if (layer->getStatus().isError())
            OE_WARN << LC << "Layer \"" << layer->getName() << "\" : " << layer->getStatus().toString() << std::endl;
    }

    void addMaskLayer(const Config& conf, Map* map)
    {
        MaskLayerOptions options(conf);
        options.name() = conf.value( "name" );
        options.driver() = MaskSourceOptions(options);
        MaskLayer* layer = new MaskLayer(options);
        map->addTerrainMaskLayer(layer);
        if (layer->getStatus().isError())
            OE_WARN << LC << "Layer \"" << layer->getName() << "\" : " << layer->getStatus().toString() << std::endl;
    }

    // support for "special" extension names (convenience and backwards compat)
    Extension* createSpecialExtension(const Config& conf, MapNode* mapNode)
    {
        // special support for the default sky extension:
        if (conf.key() == "sky" && !conf.hasValue("driver"))
            return Extension::create("sky_simple", conf);

        if (conf.key() == "ocean" && !conf.hasValue("driver"))
            return Extension::create("ocean_simple", conf);

        return 0L;
    }

    void addExtension(const Config& conf, MapNode* mapNode)
    {
        std::string name = conf.key();
        Extension* extension = Extension::create( conf.key(), conf );
        if ( !extension )
        {
            name = conf.key() + "_" + conf.value("driver");
            extension = Extension::create(name, conf);

            if (!extension)
                extension = createSpecialExtension(conf, mapNode);
        }

        if (extension)
        {
            mapNode->addExtension(extension);
        }
        else
        {            
            OE_INFO << LC << "Failed to find an extension for \"" << name << "\"\n";
        }
    }
}

EarthFileSerializer2::EarthFileSerializer2() :
_rewritePaths        ( true ),
_rewriteAbsolutePaths( false )
{
    // nop
}


osg::Node*
EarthFileSerializer2::deserialize( const Config& conf, const std::string& referrer ) const
{
    // First, pre-load any extension DLLs.
    preloadExtensionLibs(conf);
    preloadExtensionLibs(conf.child("extensions"));
    preloadExtensionLibs(conf.child("external"));

    MapOptions mapOptions( conf.child( "options" ) );

    // legacy: check for name/type in top-level attrs:
    if ( conf.hasValue( "name" ) || conf.hasValue( "type" ) )
    {
        Config legacy;
        if ( conf.hasValue("name") ) legacy.add( "name", conf.value("name") );
        if ( conf.hasValue("type") ) legacy.add( "type", conf.value("type") );
        mapOptions.mergeConfig( legacy );
    }

    Map* map = new Map( mapOptions );

    // Yes, MapOptions and MapNodeOptions share the same Config node. Weird but true.
    MapNodeOptions mapNodeOptions( conf.child( "options" ) );

    // Create a map node.
    osg::ref_ptr<MapNode> mapNode = new MapNode( map, mapNodeOptions );

    // Read all the elevation layers in FIRST so other layers can access them for things like clamping.
    for(ConfigSet::const_iterator i = conf.children().begin(); i != conf.children().end(); ++i)
    {
        if ( i->key() == "elevation" || i->key() == "heightfield" )
        {
            addElevationLayer( *i, map );
        }
    }


    // Read the layers in LAST (otherwise they will not benefit from the cache/profile configuration)
    for(ConfigSet::const_iterator i = conf.children().begin(); i != conf.children().end(); ++i)
    {
        if (i->key() == "options" || i->key() == "name" || i->key() == "type" || i->key() == "version")
        {
            // nop - handled earlier
        }

        else if ( i->key() == "image" )
        {
            addImageLayer( *i, map );
        }

        else if ( i->key() == "model" )
        {
            addModelLayer( *i, map );
        }

        else if ( i->key() == "mask" )
        {
            addMaskLayer( *i, map );
        }

        else if ( i->key() == "external" || i->key() == "extensions" )
        {
            mapNode->externalConfig() = *i;
            for(ConfigSet::const_iterator e = i->children().begin(); e != i->children().end(); ++e)
            {
                addExtension( *e, mapNode.get() );
            }
        }

        else if ( !isReservedWord(i->key()) ) // plugins/extensions.
        {
            addExtension( *i, mapNode.get() );
        }
    }

    // return the topmost parent of the mapnode. It's possible that
    // an extension added parents!
    osg::Node* top = mapNode.release();

    while( top->getNumParents() > 0 )
        top = top->getParent(0);

    return top;
}


Config
EarthFileSerializer2::serialize(const MapNode* input, const std::string& referrer) const
{
    Config mapConf("map");
    mapConf.set("version", "2");

    if ( !input || !input->getMap() )
        return mapConf; 

    const Map* map = input->getMap();
    MapFrame mapf( map, Map::ENTIRE_MODEL );

    // the map and node options:
    Config optionsConf = map->getInitialMapOptions().getConfig();
    optionsConf.merge( input->getMapNodeOptions().getConfig() );
    mapConf.add( "options", optionsConf );

    // the layers
    for( ImageLayerVector::const_iterator i = mapf.imageLayers().begin(); i != mapf.imageLayers().end(); ++i )
    {
        ImageLayer* layer = i->get();
        //Config layerConf = layer->getInitialOptions().getConfig();
        Config layerConf = layer->getImageLayerOptions().getConfig();
        layerConf.set("name", layer->getName());
        layerConf.set("driver", layer->getInitialOptions().driver()->getDriver());        
        mapConf.add( "image", layerConf );
    }

    for( ElevationLayerVector::const_iterator i = mapf.elevationLayers().begin(); i != mapf.elevationLayers().end(); ++i )
    {
        ElevationLayer* layer = i->get();
        //Config layerConf = layer->getInitialOptions().getConfig();
        Config layerConf = layer->getElevationLayerOptions().getConfig();
        layerConf.set("name", layer->getName());
        layerConf.set("driver", layer->getInitialOptions().driver()->getDriver());        
        mapConf.add( "elevation", layerConf );
    }

    for( ModelLayerVector::const_iterator i = mapf.modelLayers().begin(); i != mapf.modelLayers().end(); ++i )
    {
        ModelLayer* layer = i->get();
        Config layerConf = layer->getModelLayerOptions().getConfig();
        layerConf.set("name", layer->getName());
        layerConf.set("driver", layer->getModelLayerOptions().driver()->getDriver());
        mapConf.add( "model", layerConf );
    }

    typedef std::vector< osg::ref_ptr<Extension> > Extensions;
    for(Extensions::const_iterator i = input->getExtensions().begin(); i != input->getExtensions().end(); ++i)
    {
        Extension* e = i->get();
        Config conf = e->getConfigOptions().getConfig();
        if ( !conf.key().empty() )
        {
            mapConf.add( conf );
        }
    }

    Config ext = input->externalConfig();
    if ( !ext.empty() )
    {
        ext.key() = "external";
        mapConf.add( ext );
    }

    // Re-write pathnames in the Config so they are relative to the new referrer.
    if ( _rewritePaths && !referrer.empty() )
    {
        RewritePaths rewritePaths( referrer );
        rewritePaths.setRewriteAbsolutePaths( _rewriteAbsolutePaths );
        rewritePaths.apply( mapConf );
    }

    return mapConf;
}
