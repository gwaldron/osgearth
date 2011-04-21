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
#include <osg/Drawable>
#include <osg/Geode>
#include <osg/MatrixTransform>
#include <osg/NodeVisitor>
#include <osgUtil/SmoothingVisitor>
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

SubstituteModelFilter::SubstituteModelFilter( const Style& style ) :
_style( style ),
_cluster( false )
{
    //NOP
}

bool
SubstituteModelFilter::pushFeature(Feature*                     input,
                                   SubstituteModelFilter::Data& data,
                                   osg::Group*                  attachPoint,
                                   FilterContext&               context )
{
    GeometryIterator gi( input->getGeometry(), false );
    while( gi.hasMore() )
    {
        Geometry* geom = gi.next();

        //TODO: this is the simple, inefficient way of doing it.
        //TODO: add vertex shifting and clustering support.
        for( unsigned i=0; i<geom->size(); ++i )
        {
            const osg::Vec3d& point = (*geom)[i];
            osg::MatrixTransform* xform = new osg::MatrixTransform();
            xform->setMatrix( _modelMatrix * osg::Matrixd::translate( point ) );
            xform->setDataVariance( osg::Object::STATIC );
            xform->addChild( data._model.get() );
            attachPoint->addChild( xform );
        }
    }

    return true;
}


//clustering:
//  troll the external model for geodes. for each geode, create a geode in the target
//  model. then, for each geometry in that geode, replicate it once for each instance of
//  the model in the feature batch and transform the actual verts to a local offset
//  relative to the tile centroid. Finally, reassemble all the geodes and optimize. 
//  hopefully stateset sharing etc will work out. we may need to strip out LODs too.
bool
SubstituteModelFilter::cluster(const FeatureList&           features,
                               SubstituteModelFilter::Data& data,
                               osg::Group*                  attachPoint,
                               FilterContext&               cx )
{
    struct ClusterVisitor : public osg::NodeVisitor
    {
        ClusterVisitor( const FeatureList& features, const osg::Matrixd& modelMatrix, FilterContext& cx )
            : _features( features ),
              _modelMatrix( modelMatrix ),
              _cx( cx ),
              osg::NodeVisitor( osg::NodeVisitor::TRAVERSE_ALL_CHILDREN )
        {
            //nop
        }

        void apply( osg::Geode& geode )
        {
            // save the geode's drawables..
            osg::Geode::DrawableList old_drawables = geode.getDrawableList();

            // ..and clear out the drawables list.
            geode.removeDrawables( 0, geode.getNumDrawables() );

            // foreach each drawable that was originally in the geode...
            for( osg::Geode::DrawableList::iterator i = old_drawables.begin(); i != old_drawables.end(); i++ )
            {
                osg::Geometry* originalDrawable = dynamic_cast<osg::Geometry*>( i->get() );
                if ( !originalDrawable )
                    continue;

                // go through the list of input features...
                for( FeatureList::const_iterator j = _features.begin(); j != _features.end(); j++ )
                {
                    const Feature* feature = j->get();

                    ConstGeometryIterator gi( feature->getGeometry(), false );
                    while( gi.hasMore() )
                    {
                        const Geometry* geom = gi.next();

                        for( Geometry::const_iterator k = geom->begin(); k != geom->end(); ++k )
                        {
                            osg::Matrixd mx = _modelMatrix * osg::Matrixd::translate( *k );

                            // clone the source drawable once for each input feature.
                            osg::ref_ptr<osg::Geometry> newDrawable = osg::clone( 
                                originalDrawable, 
                                osg::CopyOp::DEEP_COPY_ARRAYS | osg::CopyOp::DEEP_COPY_PRIMITIVES );

                            osg::Vec3Array* verts = dynamic_cast<osg::Vec3Array*>( newDrawable->getVertexArray() );
                            if ( verts )
                            {
                                for( osg::Vec3Array::iterator v = verts->begin(); v != verts->end(); ++v )
                                {
                                    (*v).set( *v * mx );
                                }
                                
                                // add the new cloned, translated drawable back to the geode.
                                geode.addDrawable( newDrawable.get() );
                            }
                        }

                    }
                }
            }

            geode.dirtyBound();

            // merge the geometry...
            osgUtil::Optimizer opt;
            opt.optimize( &geode, osgUtil::Optimizer::MERGE_GEOMETRY );

#if 0

            // automatically generate normals.
            // TODO: maybe this should be an option.
            osgUtil::SmoothingVisitor smoother;
            geode.accept( smoother );
#endif
            
            osg::NodeVisitor::apply( geode );
        }

    private:
        const FeatureList&   _features;
        FilterContext        _cx;
        osg::Matrixd         _modelMatrix;
    };


    // make a copy of the model:
	osg::Node* clone = dynamic_cast<osg::Node*>( data._model->clone( osg::CopyOp::DEEP_COPY_ALL ) );

    // ..and apply the clustering to the copy.
	ClusterVisitor cv( features, _modelMatrix, cx );
	clone->accept( cv );

    attachPoint->addChild( clone );
    return true;
}

FilterContext
SubstituteModelFilter::push(FeatureList&         features, 
                            const FilterContext& context )
{
    if ( !isSupported() ) {
        OE_WARN << "SubstituteModelFilter support not enabled" << std::endl;
        return context;
    }

    if ( _style.empty() ) { //!_style.valid() ) {
        OE_WARN << LC << "Empty style; cannot process features" << std::endl;
        return context;
    }

    const MarkerSymbol* symbol = _style.getSymbol<MarkerSymbol>();
    if ( !symbol ) {
        OE_WARN << LC << "No MarkerSymbol found in style; cannot process feautres" << std::endl;
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

    if ( _cluster )
    {
        ok = cluster( features, data, group, newContext );
    }

    else
    {
        for( FeatureList::iterator i = features.begin(); i != features.end(); ++i )
            if ( !pushFeature( i->get(), data, group, newContext ) )
                ok = false;

#if 0
        // speeds things up a bit, at the expense of creating tons of geometry..

        // this optimizer pass will remove all the MatrixTransform nodes that we
        // used to offset each instance
        osgUtil::Optimizer optimizer;
        optimizer.optimize( group, osgUtil::Optimizer::FLATTEN_STATIC_TRANSFORMS_DUPLICATING_SHARED_SUBGRAPHS );
#endif

    }

    _result = group;

    return newContext;
}
