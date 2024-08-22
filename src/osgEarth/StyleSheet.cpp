/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2020 Pelican Mapping
* http://osgearth.org
*
* osgEarth is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/
#include <osgEarth/StyleSheet>
#include <osgEarth/CssUtils>
#include <algorithm>

#define LC "[StyleSheet] "

using namespace osgEarth;

//...................................................................

Config
StyleSheet::Options::getConfig() const
{
    Config conf = Layer::Options::getConfig();

    conf.remove("selector");
    for (StyleSelectors::const_iterator i = selectors().begin();
        i != selectors().end();
        ++i)
    {
        conf.add("selector", i->second.getConfig());
    }

    conf.remove("style");
    for (StyleMap::const_iterator i = styles().begin();
        i != styles().end();
        ++i)
    {
        conf.add("style", i->second.getConfig());
    }

    conf.remove("library");
    for (ResourceLibraries::const_iterator i = libraries().begin();
        i != libraries().end(); ++i)
    {
        if (i->second.valid())
        {
            Config libConf = i->second->getConfig();
            conf.add("library", libConf);
        }
    }

    conf.remove("script");
    if (_script.valid())
    {
        Config scriptConf("script");

        if (!_script->name.empty())
            scriptConf.set("name", _script->name);
        if (!_script->language.empty())
            scriptConf.set("language", _script->language);
        if (_script->uri.isSet())
            scriptConf.set("url", _script->uri->base());
        //if (!_script->profile.empty())
        //    scriptConf.set("profile", _script->profile);
        else if (!_script->code.empty())
            scriptConf.setValue(_script->code);

        conf.add(scriptConf);
    }

    return conf;
}

void
StyleSheet::Options::fromConfig(const Config& conf)
{
    //_uriContext = URIContext(conf.referrer());

    // read in any resource library references
    _libraries.clear();
    ConfigSet librariesConf = conf.children("library");
    for (ConfigSet::iterator i = librariesConf.begin(); i != librariesConf.end(); ++i)
    {
        const Config& libConf = *i;
        ResourceLibrary* resLib = new ResourceLibrary(libConf);
        if (resLib && libConf.value("name").empty() == false)
            resLib->setName(libConf.value("name"));

        _libraries[resLib->getName()] = resLib;
    }

    // read in any scripts
    _script = NULL;
    const Config& scriptConf = conf.child("script");
    if (!scriptConf.empty())
    {
        _script = new ScriptDef();

        // load the code from a URI if there is one:
        if (scriptConf.hasValue("url"))
        {
            _script->uri = URI(scriptConf.value("url"), conf.referrer());
            OE_DEBUG << LC << "Loading script from \"" << _script->uri->full() << "\"" << std::endl;
            _script->code = _script->uri->getString();
        }
        else
        {
            _script->code = scriptConf.value();
        }

        // name is optional and unused at the moment
        _script->name = scriptConf.value("name");

        std::string lang = scriptConf.value("language");
        _script->language = lang.empty() ? "javascript" : lang;
    }

    // read any style class definitions. either "class" or "selector" is allowed
    _selectors.clear();
    ConfigSet selectors = conf.children("selector");
    for (ConfigSet::iterator i = selectors.begin(); i != selectors.end(); ++i)
    {
        StyleSelector s(*i);
        std::string unique = Stringify() << s.name().get() << ":" << s.styleName().get();
        _selectors[unique] = s;
    }

    // read in the actual styles
    _styles.clear();
    ConfigSet stylesConf = conf.children("style");
    for (ConfigSet::iterator i = stylesConf.begin();
        i != stylesConf.end();
        ++i)
    {
        const Config& styleConf = *i;

        // if there is a non-empty "text" value, assume it is CSS.
        bool has_css = 
            (styleConf.value("type") == "text/css") ||
            (trim(styleConf.value()).empty() == false);

        if (has_css)
        {
            // for CSS data, there may be multiple styles in one CSS block. So
            // parse them all out and add them to the stylesheet.

            // read the inline data:
            std::string cssString = styleConf.value();

            // if there's a URL, read the CSS from the URL:
            if (styleConf.hasValue("url"))
            {
                URI uri(styleConf.value("url"), styleConf.referrer());
                cssString = uri.readString().getString();
            }

            // break up the CSS into multiple CSS blocks and parse each one individually.
            std::vector<std::string> blocks;
            CssUtils::split(cssString, blocks);

            for (std::vector<std::string>::iterator i = blocks.begin(); i != blocks.end(); ++i)
            {
                Config blockConf(styleConf);
                blockConf.setValue(*i);
                Style style(blockConf, &styles());
                _styles[style.getName()] = style;
            }
        }
        else
        {
            Style style(styleConf);
            _styles[style.getName()] = style;
        }
    }
}

//...................................................................

REGISTER_OSGEARTH_LAYER(styles, StyleSheet);
REGISTER_OSGEARTH_LAYER(stylesheet, StyleSheet);

void
StyleSheet::init()
{
    Layer::init();
}

void
StyleSheet::addStyle( const Style& style )
{
    options().styles()[ style.getName() ] = style;
}

void
StyleSheet::removeStyle( const std::string& name )
{
    options().styles().erase( name );
}

void
StyleSheet::addStylesFromCSS(const std::string& css)
{
    ConfigSet blocks;
    CssUtils::readConfig(css, "", blocks);

    for(auto& block : blocks)
    {
        Style style;
        style.fromSLD(block, &_options->styles());
        if (!style.empty())
            addStyle(style);
    }
}

Style*
StyleSheet::getStyle( const std::string& name, bool fallBackOnDefault )
{
    StyleMap::iterator i = options().styles().find( name );
    if ( i != options().styles().end() ) {
        return &i->second;
    }
    else if ( name.length() > 1 && name[0] == '#' ) {
        std::string nameWithoutHash = name.substr( 1 );
        return getStyle( nameWithoutHash, fallBackOnDefault );
    }
    else if ( fallBackOnDefault ) {
        return getDefaultStyle();
    }
    else {
        return 0L;
    }
}

const Style*
StyleSheet::getStyle( const std::string& name, bool fallBackOnDefault ) const
{
    StyleMap::const_iterator i = options().styles().find( name );
    if ( i != options().styles().end() ) {
        return &i->second;
    }
    else if ( name.length() > 1 && name[0] == '#' ) {
        std::string nameWithoutHash = name.substr( 1 );
        return getStyle( nameWithoutHash, fallBackOnDefault );
    }
    else if ( fallBackOnDefault ) {
        return getDefaultStyle();
    }
    else {
        return 0L;
    }
}

std::pair<const Style*, int>
StyleSheet::getStyleAndIndex(const std::string& name) const
{
    StyleMap::const_iterator i = options().styles().find(name);
    if (i != options().styles().end())
    {
        int index = std::distance(options().styles().begin(), i);
        return { &i->second, index };
    }
    else
    {
        return { nullptr, -1 };
    }
}

StyleMap&
StyleSheet::getStyles()
{
    return options().styles();
}

const StyleMap&
StyleSheet::getStyles() const
{
    return options().styles();
}

StyleSelectors&
StyleSheet::getSelectors()
{
    return options().selectors();
}

const StyleSelectors&
StyleSheet::getSelectors() const
{
    return options().selectors();
}

const StyleSelector*
StyleSheet::getSelector(const std::string& name) const
{
    for(StyleSelectors::const_iterator i = options().selectors().begin();
        i != options().selectors().end();
        ++i)
    {
        if (i->second.name().isSetTo(name))
            return &i->second;
    }
    return NULL;
}

void
StyleSheet::addSelector(const StyleSelector& value)
{
    getSelectors()[value.name().get()] = value;
}

Style*
StyleSheet::getDefaultStyle()
{
    StyleMap& styles = options().styles();

    if (styles.find( "default" ) != styles.end() ) {
        return &styles.find( "default" )->second;
    }
    else if (styles.find( "" ) != styles.end() ) {
        return &styles.find( "" )->second;
    }
    if (styles.size() > 0 ) {
        return &styles.begin()->second;
    }
    else {
        // insert the empty style and return it.
        styles["default"] = _emptyStyle;
        return &styles.begin()->second;
    }
}

const Style*
StyleSheet::getDefaultStyle() const
{
    const StyleMap& styles = options().styles();

    if (styles.size() == 1 ) {
        return &styles.begin()->second;
    }
    else if (styles.find( "default" ) != styles.end() ) {
        return &styles.find( "default" )->second;
    }
    else if (styles.find( "" ) != styles.end() ) {
        return &styles.find( "" )->second;
    }
    else {
        return &_emptyStyle;
    }
}

void
StyleSheet::addResourceLibrary( ResourceLibrary* lib )
{
    Threading::ScopedWriteLock exclusive( _resLibsMutex );
    options().libraries()[ lib->getName() ] = lib;
}

ResourceLibrary*
StyleSheet::getResourceLibrary( const std::string& name ) const
{
    Threading::ScopedReadLock shared( const_cast<StyleSheet*>(this)->_resLibsMutex );
    ResourceLibraries::const_iterator i = options().libraries().find( name );
    if ( i != options().libraries().end() )
        return i->second.get();
    else
        return 0L;
}

ResourceLibrary*
StyleSheet::getDefaultResourceLibrary() const
{
    Threading::ScopedReadLock shared( const_cast<StyleSheet*>(this)->_resLibsMutex );
    if (options().libraries().size() > 0 )
        return options().libraries().begin()->second.get();
    else
        return 0L;
}

void
StyleSheet::setScript( ScriptDef* script )
{
    options().script() = script;
}

StyleSheet::ScriptDef*
StyleSheet::getScript() const
{
    return options().script().get();
}
