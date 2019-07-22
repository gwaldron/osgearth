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
#ifndef OSGEARTHSYMBOLOGY_STYLE_SHEET_H
#define OSGEARTHSYMBOLOGY_STYLE_SHEET_H 1

#include <osgEarthSymbology/Common>
#include <osgEarthSymbology/Style>
#include <osgEarthSymbology/StyleSelector>
#include <osgEarthSymbology/Skins>
#include <osgEarthSymbology/ResourceLibrary>

namespace osgEarth { namespace Symbology 
{
    /**
     * A complete definition of style information.
     */
    class OSGEARTHSYMBOLOGY_EXPORT StyleSheet : public osg::Referenced
    {
    public:
      /**
       * A simple representation of a script used locally
       */
      struct ScriptDef : osg::Referenced
      {
          ScriptDef() { }
          ScriptDef(const std::string& code, const std::string& language="javascript", const std::string& name="") :
            code(code), language(language), name(name) { }

        std::string code;
        std::string language;
        std::string name;
        std::string profile;
        optional<URI> uri;
      };

    public:
        /** Constructs an empty style sheet. */
        StyleSheet();

        /** Constructs a new style sheet */
        StyleSheet( const Config& conf );

        /** Optional name of the style sheet */
        optional<std::string>& name() { return _name; }
        const optional<std::string>& name() const { return _name; }

        /** Gets the context for relative path resolution */
        const URIContext& uriContext() const { return _uriContext; }

        /** Adds a style to this sheet. */
        void addStyle( const Style& style );

        /** Removes a style from this sheet. */
        void removeStyle( const std::string& name );
        
        /** Gets a named style from this sheet. If the name isn't found, optionally falls back on the
            "default" style. Note: if the name has a hashtag prefix (e.g., "#name") it will search for
            the name with and without the hash. (they are considered equivalent) */
        Style* getStyle( const std::string& name, bool fallBackOnDefault =true );
        const Style* getStyle( const std::string& name, bool fallBackOnDefault =true ) const;

        /** Gets the default style in this sheet. */
        Style* getDefaultStyle();
        const Style* getDefaultStyle() const;

        /** Get access to styles for manual configuration */
        StyleMap& styles() { return _styles; }
        const StyleMap& styles() const { return _styles; }

        /** Selectors pick a style from the sheet based on some criteria. */
        StyleSelectorList& selectors() { return _selectors; }
        const StyleSelectorList& selectors() const { return _selectors; }
        const StyleSelector* getSelector( const std::string& name ) const;

        /** Adds a resource library. */
        void addResourceLibrary( ResourceLibrary* library );

        /** Gets a resource library by name. */
        ResourceLibrary* getResourceLibrary( const std::string& name ) const;

        /** Gets the first library. */
        ResourceLibrary* getDefaultResourceLibrary() const;

        /** Script accessors */
        void setScript( ScriptDef* script );
        ScriptDef* script() const { return _script.get(); }

    public: // serialization functions

        virtual Config getConfig() const;
        virtual void mergeConfig( const Config& conf );

    protected:
        optional<std::string>        _name;
        URIContext                   _uriContext;
        osg::ref_ptr<ScriptDef>      _script;
        StyleSelectorList            _selectors;
        StyleMap                     _styles;
        Style                        _emptyStyle;
        
        typedef std::map<std::string, osg::ref_ptr<ResourceLibrary> > ResourceLibraries;
        ResourceLibraries          _resLibs;
        Threading::ReadWriteMutex  _resLibsMutex;

    };

} } // namespace osgEarth::Symbology

//OSGEARTH_SPECIALIZE_REF_CONFIG(osgEarth::Symbology::StyleSheet);

#endif // OSGEARTHSYMBOLOGY_STYLE_SHEET_H
