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
#include <osgEarthSymbology/ModelSymbolizer>
#include <osgEarthSymbology/MarkerSymbol>
#include <osgEarth/HTTPClient>
#include <osg/Geode>
#include <osg/Version>
#include <osgDB/ReadFile>
#include <osgDB/ReaderWriter>

using namespace osgEarth::Symbology;

ModelSymbolizer::ModelSymbolizer()
{
}


static osg::Node* getNode(const std::string& str)
{
    osg::ref_ptr<osg::Node> output;
#if OSG_VERSION_LESS_THAN(2,9,8)
    osg::ref_ptr<osgDB::ReaderWriter::Options> options = new osgDB::ReaderWriter::Options;
    options->setObjectCacheHint(osgDB::ReaderWriter::Options::CACHE_ALL);
#else
    osg::ref_ptr<osgDB::Options> options = new osgDB::Options;
    options->setObjectCacheHint(osgDB::Options::CACHE_ALL);
#endif
    if ( HTTPClient::readNodeFile( str, output, options.get(), 0 ) == HTTPClient::RESULT_OK )
    {
        osg::notify(osg::NOTICE) << "loaded " << str << std::endl;
    }
    else
    {
        osg::notify(osg::NOTICE) << "can't load " << str << std::endl;
    }
    return output.release();
}


bool 
ModelSymbolizer::compile(State<GeometryContent>* state,
                         osg::Group* attachPoint)
{
    if ( !state || !attachPoint || !state->getContent() || !state->getStyle() )
        return false;

    const MarkerSymbol* symbol = state->getStyle()->getSymbol<MarkerSymbol>();
    if (!symbol)
        return false;

    std::string model = symbol->marker().value();
    if (model.empty())
        return false;

    osg::ref_ptr<osg::Group> newSymbolized = new osg::Group;
    osg::Node* node = getNode(model);
    if (!node)
        return false;

    newSymbolized->addChild(node);
    attachPoint->removeChildren(0, attachPoint->getNumChildren());
    attachPoint->addChild(newSymbolized.get());
    return true;
}
