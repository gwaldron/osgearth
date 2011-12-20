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

#include <osgEarthAnnotation/DrawState>

#include <osgEarthAnnotation/AnnotationNode>
#include <osgEarthAnnotation/LocalizedNode>
#include <osgEarthAnnotation/OrthoNode>
#include <osgEarthAnnotation/LabelNode>
#include <osgEarthAnnotation/PlaceNode>
#include <osgEarthAnnotation/TrackNode>

#include <osgEarthSymbology/Color>
#include <osgEarthSymbology/MeshSubdivider>

#include <osg/MatrixTransform>
#include <osgFX/Outline>
#include <osg/ShapeDrawable>
#include <osg/CullFace>
#include <osg/Depth>
#include <osg/Stencil>
#include <osg/Geometry>


using namespace osgEarth::Annotation;

//---------------------------------------------------------------------------

void
DrawStateInstaller::apply(osg::Node& node)
{
    if ( dynamic_cast<AnnotationNode*>(&node) )
    {
        if ( _tech.valid() )
            static_cast<AnnotationNode*>(&node)->installAltDrawState( _name, _tech );
        else if ( _callback.valid() )
            _callback->operator()( static_cast<AnnotationNode*>(&node) );
    }
    traverse(node);
}

//---------------------------------------------------------------------------

bool
DrawStateTechnique::apply(class AnnotationNode& node, bool enable)
{
    return false;
}

bool
DrawStateTechnique::apply(class LocalizedNode& node, bool enable)
{ 
    return apply(static_cast<AnnotationNode&>(node), enable);
}

bool
DrawStateTechnique::apply(class OrthoNode& node, bool enable)
{
    return apply(static_cast<AnnotationNode&>(node), enable);
}

//---------------------------------------------------------------------------

InjectionDrawStateTechnique::InjectionDrawStateTechnique( osg::Group* group ) :
_injectionGroup( group )
{
    if ( !_injectionGroup.valid() )
        _injectionGroup = new osg::Group();
}

bool
InjectionDrawStateTechnique::apply(AnnotationNode& node, bool enable)
{
    bool success = apply( node.getAttachPoint(), enable );
    return success ? true : DrawStateTechnique::apply(node, enable);
}

bool
InjectionDrawStateTechnique::apply(osg::Group* ap, bool enable)
{
    if ( _injectionGroup.valid() && ap )
    {
        if ( enable )
        {
            for( unsigned i=0; i<ap->getNumChildren(); ++i )
            {
                _injectionGroup->addChild( ap->getChild(i) );
            }
            ap->removeChildren(0, ap->getNumChildren() );
            ap->addChild( _injectionGroup.get() );
        }
        else // if ( !enable)
        {
            for( unsigned i=0; i<_injectionGroup->getNumChildren(); ++i )
            {
                ap->addChild( _injectionGroup->getChild(i) );
            }
            ap->removeChild(0, 1);
            _injectionGroup->removeChildren(0, _injectionGroup->getNumChildren());
        }
        return true;
    }
    return false;
}

//---------------------------------------------------------------------------

ScaleDrawStateTechnique::ScaleDrawStateTechnique( float factor ) :
InjectionDrawStateTechnique( new osg::MatrixTransform( osg::Matrix::scale(factor,factor,factor) ) ),
_factor( factor )
{
    //nop
}

//--------------------------------------------------------------------------

namespace
{
    struct TraverseNodeCallback : public osg::NodeCallback
    {
        osg::ref_ptr<osg::Node> _node;
        TraverseNodeCallback(osg::Node* node) : _node(node) { }
        void operator()(osg::Node* node, osg::NodeVisitor* nv)
        {
            _node->accept(*nv);
            traverse(node, nv);
        }
    };
};

namespace
{
    osg::Node* createGeodesicSphere( float r, const osg::Vec4& color )
    {
        osg::Geometry* geom = new osg::Geometry();

        osg::Vec3Array* v = new osg::Vec3Array();
        v->reserve(6);
        v->push_back( osg::Vec3(0,0,r) ); // top
        v->push_back( osg::Vec3(0,0,-r) ); // bottom
        v->push_back( osg::Vec3(-r,0,0) ); // left
        v->push_back( osg::Vec3(r,0,0) ); // right
        v->push_back( osg::Vec3(0,r,0) ); // back
        v->push_back( osg::Vec3(0,-r,0) ); // front
        geom->setVertexArray(v);

        osg::DrawElementsUByte* b = new osg::DrawElementsUByte(GL_TRIANGLES);
        b->reserve(24);
        b->push_back(0); b->push_back(3); b->push_back(4);
        b->push_back(0); b->push_back(4); b->push_back(2);
        b->push_back(0); b->push_back(2); b->push_back(5);
        b->push_back(0); b->push_back(5); b->push_back(3);
        b->push_back(1); b->push_back(3); b->push_back(5);
        b->push_back(1); b->push_back(4); b->push_back(3);
        b->push_back(1); b->push_back(2); b->push_back(4);
        b->push_back(1); b->push_back(5); b->push_back(2);
        geom->addPrimitiveSet( b );

        MeshSubdivider ms;
        ms.run( *geom, osg::DegreesToRadians(15.0f), GEOINTERP_GREAT_CIRCLE );

        osg::Vec4Array* c = new osg::Vec4Array(1);
        (*c)[0] = color;
        geom->setColorArray( c );
        geom->setColorBinding( osg::Geometry::BIND_OVERALL );

        osg::Geode* geode = new osg::Geode();
        geode->addDrawable( geom );

        return geode;
    }

    osg::Node* createQuad( const osg::Vec4& color )
    {
        osg::Geometry* geom = new osg::Geometry();

        osg::Vec3Array* v = new osg::Vec3Array();
        v->reserve(4);
        v->push_back( osg::Vec3(0,0,0) );
        v->push_back( osg::Vec3(1,0,0) );
        v->push_back( osg::Vec3(1,1,0) );
        v->push_back( osg::Vec3(0,1,0) );
        geom->setVertexArray(v);

        osg::DrawElementsUByte* b = new osg::DrawElementsUByte(GL_TRIANGLES);
        b->reserve(6);
        b->push_back(0); b->push_back(1); b->push_back(2);
        b->push_back(2); b->push_back(3); b->push_back(0);
        geom->addPrimitiveSet( b );

        osg::Vec4Array* c = new osg::Vec4Array(1);
        (*c)[0] = color;
        geom->setColorArray( c );
        geom->setColorBinding( osg::Geometry::BIND_OVERALL );

        osg::Geode* geode = new osg::Geode();
        geode->addDrawable( geom );
        
        osg::StateSet* s = geom->getOrCreateStateSet();
        s->setMode(GL_LIGHTING,0);
        s->setMode(GL_BLEND,1);
        s->setMode(GL_DEPTH_TEST,0);
        s->setMode(GL_CULL_FACE,0);
        s->setAttributeAndModes( new osg::Depth(osg::Depth::ALWAYS, 0, 1, false), 1 );

        osg::MatrixTransform* xform = new osg::MatrixTransform( osg::Matrix::identity() );
        xform->setReferenceFrame( osg::Transform::ABSOLUTE_RF );
        xform->addChild( geode );

        osg::Projection* proj = new osg::Projection( osg::Matrix::ortho(0,1,0,1,0,-1) );
        proj->addChild( xform );

        return proj;
    }
}

//--------------------------------------------------------------------------

bool
EncircleDrawStateTechnique::apply(osg::Group* ap, bool enable)
{
    if ( _injectionGroup->getCullCallback() == 0L && ap != 0L )
    {
        const osg::BoundingSphere& bs = ap->getBound();

        osg::Node* geode = createGeodesicSphere(bs.radius(), osg::Vec4(1,0,0,0.4));

        osg::StateSet* s = geode->getOrCreateStateSet();
        s->setMode(GL_LIGHTING,0);
        s->setMode(GL_BLEND,1);
        s->setAttributeAndModes( new osg::Depth(osg::Depth::LESS, 0, 1, false), 1 );
        s->setAttributeAndModes( new osg::CullFace(osg::CullFace::BACK), 1 );

        _injectionGroup->addCullCallback( new TraverseNodeCallback(geode) );
    }

    return InjectionDrawStateTechnique::apply(ap, enable);
}

//--------------------------------------------------------------------------

#undef  LC
#define LC "[HighlightDrawState] "
namespace
{
    struct HighlightGroup : public osg::Group
    {
        osg::ref_ptr<osg::StateSet> _pass1, _pass2;
        osg::ref_ptr<osg::Node>     _fillNode;

        void traverse(osg::NodeVisitor& nv)
        {
            osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(&nv);
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

HighlightDrawStateTechnique::HighlightDrawStateTechnique(const osg::Vec4f& color) :
InjectionDrawStateTechnique( new HighlightGroup() ),
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
HighlightDrawStateTechnique::apply(osg::Group* ap, bool enable)
{
    HighlightGroup* hg = dynamic_cast<HighlightGroup*>( _injectionGroup.get() );
    if ( !hg->_fillNode.valid() && ap != 0L )
    {
        const osg::BoundingSphere& bs = ap->getBound();

        osg::Node* quad = createQuad( _color );
        quad->setCullingActive( false );
        hg->_fillNode = quad;
    }

    return InjectionDrawStateTechnique::apply(ap, enable);
}
