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
#include <osgEarthFeatures/SubstituteModelFilter>
#include <osgEarth/HTTPClient>
#include <osg/MatrixTransform>
#include <list>
#include <deque>

#define LC "[SubstituteModelFilter] "

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;

//------------------------------------------------------------------------

namespace
{
    static osg::Node* s_defaultModel =0L;
}

//------------------------------------------------------------------------

SubstituteModelFilter::SubstituteModelFilter( const Style* style ) :
_style( style )
{
    //NOP
}

bool
SubstituteModelFilter::push(Feature*                     input,
                            SubstituteModelFilter::Data& data,
                            osg::Group*                  attachPoint,
                            FilterContext&               context )
{
    Geometry* geom = input->getGeometry();
    if ( geom )
    {
        //TODO: this is the simple, inefficient way of doing it.
        //TODO: add vertex shifting and clustering support.
        osg::Vec2d c = geom->getBounds().center2d();
        osg::MatrixTransform* xform = new osg::MatrixTransform();
        xform->setMatrix( osg::Matrixd::translate( c.x(), c.y(), 0.0 ) );
        xform->addChild( data._model.get() );
        attachPoint->addChild( xform );
    }

    return true;
}

FilterContext
SubstituteModelFilter::push(FeatureList&             input, 
                            const FilterContext&     context )
{
    if ( !isSupported() ) {
        OE_WARN << "SubstituteModelFilter support not enabled" << std::endl;
        return context;
    }

    if ( !_style.valid() ) {
        OE_WARN << LC << "No style supplied; cannot process features" << std::endl;
        return context;
    }

    const ModelSymbol* symbol = _style->getSymbol<ModelSymbol>();
    if ( !symbol ) {
        OE_WARN << LC << "No ModelSymbol found in style; cannot process feautres" << std::endl;
        return context;
    }

    FilterContext newContext( context );

    // assemble the data for this pass
    Data data;
    data._model = newContext.getSession()->getModel( *symbol->url() );
    if ( !data._model.valid() )
    {
        OE_WARN << LC << "Unable to load model from \"" << *symbol->url() << "\"" << std::endl;
        return newContext;
    }

    osg::Group* group = new osg::Group();

    bool ok = true;
    for( FeatureList::iterator i = input.begin(); i != input.end(); ++i )
        if ( !push( i->get(), data, group, newContext ) )
            ok = false;

    _result = group;

    return newContext;
}
