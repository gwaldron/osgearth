/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2013 Pelican Mapping
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

#include <osgEarthAnnotation/HighlightDecoration>
#include <osgEarthAnnotation/AnnotationUtils>
#include <osgEarthAnnotation/OrthoNode>
#include <osgEarth/NodeUtils>
#include <osg/Stencil>

#undef  LC
#define LC "[HighlightDecoration] "

using namespace osgEarth::Annotation;

namespace
{
    struct HighlightGroup : public osg::Group
    {
        osg::ref_ptr<osg::StateSet> _pass1, _pass2;
        osg::ref_ptr<osg::Node>     _fillNode;

        void traverse(osg::NodeVisitor& nv)
        {
            osgUtil::CullVisitor* cv = Culling::asCullVisitor(nv);
            if ( cv && _fillNode.valid() && _pass1.valid() )
            {
                const osg::GraphicsContext* gc = cv->getCurrentCamera()->getGraphicsContext();
                if ( gc && gc->getTraits() && gc->getTraits()->stencil < 1 )
                {
                    OE_WARN << LC << "Insufficient stencil buffer bits available; disabling highlighting." << std::endl;
                    OE_WARN << LC << "Please call osg::DisplaySettings::instance()->setMinimumNumStencilBits()" << std::endl;
                    _pass1 = 0L;
                }
                else
                {
                    // first render the geometry to the stencil buffer:
                    cv->pushStateSet(_pass1);
                    osg::Group::traverse( nv );
                    cv->popStateSet();

                    // the render the coverage quad
                    cv->pushStateSet(_pass2);
                    _fillNode->accept( nv );
                    cv->popStateSet();
                }
            }
            else
            {
                osg::Group::traverse(nv);
            }
        }
    };
}

HighlightDecoration::HighlightDecoration(const osg::Vec4f& color) :
InjectionDecoration( new HighlightGroup() ),
_color( color )
{
    HighlightGroup* hg = dynamic_cast<HighlightGroup*>( _injectionGroup.get() );

    hg->_pass1 = new osg::StateSet();
    {
        osg::Stencil* stencil  = new osg::Stencil();
        stencil->setFunction(osg::Stencil::ALWAYS, 1, ~0u);
        stencil->setOperation(osg::Stencil::KEEP, osg::Stencil::KEEP, osg::Stencil::REPLACE);
        hg->_pass1->setAttributeAndModes(stencil, 1);
        hg->_pass1->setBinNumber(0);
    }

    hg->_pass2 = new osg::StateSet();
    {
        osg::Stencil* stencil  = new osg::Stencil();
        stencil->setFunction(osg::Stencil::NOTEQUAL, 0, ~0u);
        stencil->setOperation(osg::Stencil::REPLACE, osg::Stencil::REPLACE, osg::Stencil::REPLACE);
        hg->_pass2->setAttributeAndModes(stencil, 1);
        hg->_pass2->setBinNumber(1);
    }
}

bool
HighlightDecoration::apply(OrthoNode& node, bool enable)
{
    if ( node.getAttachPoint() )
    {
        node.setDynamic( true );
        FindNodesVisitor<osg::Geode> fnv;
        node.getAttachPoint()->accept( fnv );
        if ( enable )
        {
            osg::BoundingBox box;
            for( std::vector<osg::Geode*>::iterator i = fnv._results.begin(); i != fnv._results.end(); ++i )
            {
                osg::Geode* geode = *i;
                box.expandBy( geode->getBoundingBox() );
            }
            
            if ( fnv._results.size() > 0 )
            {
                osg::Drawable* geom = AnnotationUtils::create2DOutline( box, 3.0f, _color );  
                geom->setUserData( node.getAnnotationData() );
                for( std::vector<osg::Geode*>::iterator i = fnv._results.begin(); i != fnv._results.end(); ++i )
                {
                    (*i)->addDrawable(geom);
                }
            }
        }
        else
        {
            for( std::vector<osg::Geode*>::iterator i = fnv._results.begin(); i != fnv._results.end(); ++i )
            {
                osg::Geode* geode = *i;
                geode->removeDrawable( geode->getDrawable(geode->getNumDrawables()-1) );
            }
        }

        return true;
    }
    return false;
}

bool
HighlightDecoration::apply(osg::Group* ap, bool enable)
{
    HighlightGroup* hg = dynamic_cast<HighlightGroup*>( _injectionGroup.get() );
    if ( !hg->_fillNode.valid() && ap != 0L )
    {
        const osg::BoundingSphere& bs = ap->getBound();

        osg::Node* quad = AnnotationUtils::createFullScreenQuad( _color );
        quad->setCullingActive( false );
        hg->_fillNode = quad;
    }

    return InjectionDecoration::apply(ap, enable);
}
