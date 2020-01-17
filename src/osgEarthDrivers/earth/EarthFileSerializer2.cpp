/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2019 Pelican Mapping
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
#include <osgEarth/Extension>
#include <osgEarth/StringUtils>
#include <osgEarth/FileUtils>
#include <osgEarth/URI>
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

    // Config tags that we handle specially (versus just letting the plugin mechanism
    // take take of them)
    bool isReservedWord(const std::string& k)
    {
        return
            k == "options" ||
            //k == "image" ||
            k == "elevation" ||
            k == "heightfield" ||
            //k == "model" ||
            //k == "mask" ||
            k == "external" ||
            k == "extensions" ||
            k == "libraries";
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
            _newReferrerFolder  = osgDB::getFilePath( _newReferrerAbsPath );
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
                    input.setValue(newValue);
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
			// 1. If paths are neither both absolute nor both relative, then we cannot do anything (we need to make them absolute, but need additional info on how to make it). Return.
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
    // support for "special" extension names (convenience and backwards compat)
    Extension* createSpecialExtension(const Config& conf)
    {
        // special support for the default sky extension:
        if (conf.key() == "sky" && !conf.hasValue("driver"))
            return Extension::create("sky_simple", conf);

        if (conf.key() == "ocean" && !conf.hasValue("driver"))
            return Extension::create("ocean_simple", conf);

        return 0L;
    }

    bool addLayer(const Config& conf, Map* map)
    {
        Layer* layer = Layer::create(conf);
        if (layer)
        {
            map->addLayer(layer);
        }
        return layer != 0L;
    }

    Extension* loadExtension(const Config& conf)
    {
        std::string name = conf.key();
        Extension* extension = Extension::create( conf.key(), conf );
        if ( !extension )
        {
            name = conf.key() + "_" + conf.value("driver");
            extension = Extension::create(name, conf);

            if (!extension)
                extension = createSpecialExtension(conf);
        }

        if (!extension)
        {
            OE_INFO << LC << "Failed to find an extension for \"" << name << "\"\n";
        }

        return extension;
    }

    void reportErrors(const Map* map)
    {
        for (unsigned i = 0; i < map->getNumLayers(); ++i)
        {
            const Layer* layer = map->getLayerAt(i);
            if (layer->getStatus().isError())
            {
                OE_WARN << LC << layer->getTypeName() << " \"" << layer->getName() << "\" : " << layer->getStatus().toString() << std::endl;
            }
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

    osg::ref_ptr< Map > map = new Map( mapOptions );

    // Start a batch update of the map:
    map->beginUpdate();

    // Read all the elevation layers in FIRST so other layers can access them for things like clamping.
    // TODO: revisit this since we should really be listening for elevation data changes and
    // re-clamping based on that..
    for(ConfigSet::const_iterator i = conf.children().begin(); i != conf.children().end(); ++i)
    {
        // for backwards compatibility:
        if (i->key() == "heightfield")
        {
            Config temp = *i;
            temp.key() = "elevation";
            addLayer(temp, map.get());
        }

        else if ( i->key() == "elevation" ) // || i->key() == "heightfield" )
        {
            addLayer(*i, map.get());
        }
    }

    Config externalConfig;
    std::vector<osg::ref_ptr<Extension> > extensions;

    // Read the layers in LAST (otherwise they will not benefit from the cache/profile configuration)
    for(ConfigSet::const_iterator i = conf.children().begin(); i != conf.children().end(); ++i)
    {
        if (i->key() == "options" || i->key() == "name" || i->key() == "type" || i->key() == "version")
        {
            // nop - handled earlier
        }

        else if ( i->key() == "external" || i->key() == "extensions" )
        {
            externalConfig = *i;
            
            for(ConfigSet::const_iterator e = i->children().begin(); e != i->children().end(); ++e)
            {
                Extension* extension = loadExtension(*e);
                if (extension)
                    extensions.push_back(extension);
            }
        }

        else if ( !isReservedWord(i->key()) ) // plugins/extensions.
        {
            // try to add as a plugin Layer first:
            bool addedLayer = addLayer(*i, map.get()); 

            // failing that, try to load as an extension:
            if ( !addedLayer )
            {
                Extension* extension = loadExtension(*i);
                if (extension)
                    extensions.push_back(extension);
            }
        }
    }

    // Complete the batch update of the map
    map->endUpdate();

    // If any errors occurred, report them now.
    reportErrors(map.get());

    // Yes, MapOptions and MapNodeOptions share the same Config node. Weird but true.
    MapNodeOptions mapNodeOptions( conf.child("options") );

    // Create a map node.
    osg::ref_ptr<MapNode> mapNode = new MapNode( map.get(), mapNodeOptions );

    // Apply the external conf if there is one.
    if (!externalConfig.empty())
    {
        mapNode->externalConfig() = externalConfig;
    }

    // Install the extensions
    for (unsigned i = 0; i < extensions.size(); ++i)
    {
        mapNode->addExtension(extensions[i].get());
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
    
    if (input && input->getMap())
    {
        mapConf = input->getConfig();

        // Re-write pathnames in the Config so they are relative to the new referrer.
        if ( _rewritePaths && !referrer.empty() )
        {
            RewritePaths rewritePaths( referrer );
            rewritePaths.setRewriteAbsolutePaths( _rewriteAbsolutePaths );
            rewritePaths.apply( mapConf );
        }
    }

    return mapConf;
}
