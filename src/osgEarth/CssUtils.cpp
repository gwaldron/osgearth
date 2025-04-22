/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include <osgEarth/CssUtils>
#include <osgEarth/StringUtils>

using namespace osgEarth;

void
CssUtils::split( const std::string& input, std::vector<std::string>& output )
{
    auto blocks = StringTokenizer()
        .delim("{")
        .delim("}")
        .standardQuotes()
        .tokenize(input);

    for( unsigned i=0; i<blocks.size(); ++i )
    {
        if ( startsWith( blocks[i], "{" ) )
        {
            output.push_back( "default { " + blocks[i] + " }" );
        }
        else if ( i+1 < blocks.size() )
        {
            output.push_back( blocks[i] + "{ " + blocks[i+1] + " }" );
            ++i;
        }
    }
}

void
CssUtils::readConfig( const std::string& css, const std::string& referrer, ConfigSet& output )
{
    ConfigSet result;

    // if there's no brackets, assume this is a single default block.
    std::string temp = css;
    if ( css.find_first_of("{") == std::string::npos )
    {
        temp = "default { " + css + " }";
    }
    else if ( css.size() > 0 && css[0] == '{' )
    {
        temp = "default " + css;
    }

    // tokenize the CSS into a config object..
    Config conf( "css" );

    StringTokenizer tokenizePropertySet;
    tokenizePropertySet
        .delim(";")
        .standardQuotes();

    StringTokenizer tokenizeProperty;
    tokenizeProperty
        .delim(":")
        .standardQuotes()
        .quotePair('(', ')');

    StringVector blocks = StringTokenizer()
        .delim("{").delim("}")
        .standardQuotes()
        .tokenize(temp);

    for( unsigned i=0; i<blocks.size(); )
    {
        const std::string& name = blocks[i++];
        if ( i < blocks.size() )
        {
            Config elementConf( name );
            elementConf.setReferrer( referrer );

            auto propSet = tokenizePropertySet(blocks[i++]);
            
            for( unsigned j=0; j<propSet.size(); ++j )
            {
                auto prop = tokenizeProperty(propSet[j]);

                if ( prop.size() == 2 )
                {
                    elementConf.set( prop[0], prop[1] );
                }
            }

            output.push_back( elementConf );
        }
    }
}
