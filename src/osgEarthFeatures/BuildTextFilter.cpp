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
#include <osgEarthFeatures/BuildTextFilter>
//#include <osgEarthFeatures/BuildTextOperator> // this should be in symbology -gw
#include <osgEarthFeatures/LabelSource>
#include <osgEarthSymbology/TextSymbol>
#include <osgText/Text>

#define LC "[BuildTextFilter] "

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;

BuildTextFilter::BuildTextFilter( const Style& style ) :
_style( style )
{
    //nop
}

osg::Node*
BuildTextFilter::push( FeatureList& input, FilterContext& context )
{
    osg::Node* result = 0L;

    const TextSymbol* text = _style.get<TextSymbol>();
    if ( !text )
    {
        OE_WARN << LC << "Insufficient symbology (no TextSymbol)" << std::endl;
        return 0L;
    }

    // if a provider is set, load the plugin and create the node.
    if ( true ) //!text->provider()->empty() && !text->provider().isSetTo("legacy") )
    {
        LabelSourceOptions options;
        options.setDriver( *text->provider() );
        osg::ref_ptr<LabelSource> source = LabelSourceFactory::create( options );
        if ( source.valid() )
        {
            result = source->createNode( input, text, context );
        }
        else
        {
            OE_WARN << LC << "FAIL, unable to load label provider \"" << (*text->provider()) << "\"" << std::endl;
            return 0L;
        }
    }

#if 0
    else // legacy behavior... will be deprecated.
    {
        BuildTextOperator op;
        result = op( input, text, context );
    }
#endif

    return result;
}
