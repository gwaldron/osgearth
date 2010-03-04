/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2009 Pelican Ventures, Inc.
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
#include <osgEarthSymbology/BasicSymbolizer>

using namespace osgEarth::Symbology;

BasicSymbolizer::BasicSymbolizer()
{
}


bool 
BasicSymbolizer::update(FeatureDataSet* dataSet,
                        const osgEarth::Symbology::Style& style,
                        osg::Group* attachPoint,
                        SymbolizerContext* context )
{

#if 0
    osg::ref_ptr<osg::Group> newSymbolized = new osg::Group;
    for (FeatureDataSet::iterator it = dataSet->begin(); it != dataSet->end(); ++it)
    {
        Feature* feature = *it;

        Symbol* symbol = style.getSymbol(feature);
        SymbolDisplayFunctor* displayer = Registry::instance()->getOrCreateDisplay((*it)->symbol->getName());
        osg::Node* node = displayer->apply(*symbol);
        if (node)
            newSymbolized->addChild(node);
    }

    if (newSymbolized->getNumChildren())
        attachPoint->addChild(newSymbolized);
#endif
    return false;
}
