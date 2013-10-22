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
#include <osgEarthSymbology/StencilVolumeNode>
#include <osgEarth/Registry>
#include <osgEarth/NodeUtils>
#include <osgEarth/Capabilities>
#include <osg/Stencil>
#include <osg/StencilTwoSided>
#include <osg/Depth>
#include <osg/Drawable>
#include <osg/CopyOp>
#include <osg/CullFace>
#include <osg/MatrixTransform>
#include <osg/Projection>
#include <osg/GLExtensions>
#include <osg/Geode>
#include <osg/Notify>
#include <osgUtil/Tessellator>
#include <algorithm>

#define ON_AND_PROTECTED  osg::StateAttribute::ON | osg::StateAttribute::PROTECTED
#define OFF_AND_PROTECTED osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED

using namespace osgEarth;
using namespace osgEarth::Symbology;

/***************************************************************************/

#define OFF_PROTECTED osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED

osg::Node*
StencilVolumeNode::createFullScreenQuad( const osg::Vec4f& color )
{
    // make a full screen quad:
    osg::Geometry* quad = new osg::Geometry();
    quad->setUseVertexBufferObjects(true);

    osg::Vec3Array* verts = new osg::Vec3Array(4);
    (*verts)[0].set( 0, 1, 0 );
    (*verts)[1].set( 0, 0, 0 );
    (*verts)[2].set( 1, 0, 0 );
    (*verts)[3].set( 1, 1, 0 );
    quad->setVertexArray( verts );
    quad->addPrimitiveSet( new osg::DrawArrays( osg::PrimitiveSet::QUADS, 0, 4 ) );
    osg::Vec4Array* colors = new osg::Vec4Array(1);
    (*colors)[0] = color;
    quad->setColorArray( colors );
    quad->setColorBinding( osg::Geometry::BIND_OVERALL );
    osg::Geode* quad_geode = new osg::Geode();
    quad_geode->addDrawable( quad );

    osg::StateSet* quad_ss = quad->getOrCreateStateSet();
    quad_ss->setMode( GL_CULL_FACE, OFF_PROTECTED );
    quad_ss->setMode( GL_DEPTH_TEST, OFF_PROTECTED );
    quad_ss->setMode( GL_LIGHTING, OFF_PROTECTED );
    osg::MatrixTransform* abs = new osg::MatrixTransform();
    abs->setReferenceFrame( osg::Transform::ABSOLUTE_RF );
    abs->setMatrix( osg::Matrix::identity() );
    abs->addChild( quad_geode );

    osg::Projection* proj = new osg::Projection();
    proj->setMatrix( osg::Matrix::ortho(0, 1, 0, 1, 0, -1) );
    proj->addChild( abs );

    proj->getOrCreateStateSet()->setMode( GL_BLEND, 1 );    

    return proj;
}


/***************************************************************************/

StencilVolumeNode::StencilVolumeNode( bool preRenderChildrenToDepthBuffer, bool inverted ) :
osgEarth::MaskNode(),
_stencilGroup1( 0L ),
_stencilGroup2( 0L ),
_depthPass( 0L ),
_renderPass( 0L ),
_inverted( inverted ),
_preRenderChildrenToDepthBuffer( preRenderChildrenToDepthBuffer )
{
    init();
}

StencilVolumeNode::StencilVolumeNode(const StencilVolumeNode& rhs,
                                     const osg::CopyOp& op) :
osgEarth::MaskNode( rhs, op ),
_root( rhs._root.get() ),
_stencilGroup1( rhs._stencilGroup1 ),
_stencilGroup2( rhs._stencilGroup2 ),
_depthPass( rhs._depthPass ),
_renderPass( rhs._renderPass ),
_inverted( rhs._inverted ),
_preRenderChildrenToDepthBuffer( rhs._preRenderChildrenToDepthBuffer )
{
    //nop
}

int
StencilVolumeNode::setBaseRenderBin( int bin )
{
    if ( _depthPass )
    {
        _depthPass->getOrCreateStateSet()->setRenderBinDetails( bin++, "RenderBin" );
    }
    if ( _stencilGroup1 )
    {
        _stencilGroup1->getOrCreateStateSet()->setRenderBinDetails( bin++, "RenderBin" );
    }
    if ( _stencilGroup2 )
    {
        _stencilGroup2->getOrCreateStateSet()->setRenderBinDetails( bin++, "RenderBin" );
    }
    if ( _renderPass )
    {
        _renderPass->getOrCreateStateSet()->setRenderBinDetails( bin++, "RenderBin" );
    }
    return bin;
}

void
StencilVolumeNode::addVolumes( osg::Node* node, bool onNextUpdate )
{
    if ( onNextUpdate )
    {
        Threading::ScopedMutexLock lock( _addVolumesMutex );
        _volumesToAdd.push( node );
        ADJUST_UPDATE_TRAV_COUNT( this, 1 );
    }
    else
    {
        if ( _stencilGroup1 )
            _stencilGroup1->addChild( node );
        if ( _stencilGroup2 )
            _stencilGroup2->addChild( node );
    }
}

void
StencilVolumeNode::addQueuedVolumes()
{
    Threading::ScopedMutexLock lock( _addVolumesMutex );
    while( !_volumesToAdd.empty() )
    {
        osg::Node* node = _volumesToAdd.front().get();

        if ( _stencilGroup1 )
            _stencilGroup1->addChild( node );
        if ( _stencilGroup2 )
            _stencilGroup2->addChild( node );        

        _volumesToAdd.pop();
    }
    
    ADJUST_UPDATE_TRAV_COUNT( this, -1 );
}

/** Override all the osg::Group methods: */

bool 
StencilVolumeNode::addChild( Node *child ) {
    if ( !child ) return false;
    dirtyBound();
    if ( _depthPass )  _depthPass->addChild( child );
    return _renderPass->addChild( child );
}
bool 
StencilVolumeNode::insertChild( unsigned int index, Node *child ) {
    if ( !child ) return false;
    dirtyBound();
    if ( _depthPass ) _depthPass->insertChild( index, child );
    return _renderPass->insertChild( index, child );
}
bool 
StencilVolumeNode::removeChildren(unsigned int pos,unsigned int numChildrenToRemove) {
    dirtyBound();
    if ( _depthPass ) _depthPass->removeChildren( pos, numChildrenToRemove );
    return _renderPass->removeChildren( pos, numChildrenToRemove );
}
bool 
StencilVolumeNode::replaceChild( Node *origChild, Node* newChild ) {
    dirtyBound();
    if ( _depthPass ) _depthPass->replaceChild( origChild, newChild );
    return _renderPass->replaceChild( origChild, newChild );
}
bool 
StencilVolumeNode::setChild( unsigned  int i, Node* node ) {
    dirtyBound();
    if ( _depthPass ) _depthPass->setChild( i, node );
    return _renderPass->setChild( i, node );
}

osg::BoundingSphere
StencilVolumeNode::computeBound() const {
    if ( _depthPass ) return _depthPass->computeBound();
    else return _renderPass->computeBound();
}


void
StencilVolumeNode::traverse( osg::NodeVisitor& nv )
{
    if ( nv.getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR && _volumesToAdd.size() > 0 )
    {
        addQueuedVolumes();
    }

    _root->accept( nv );
}

void
StencilVolumeNode::init()
{
    this->getOrCreateStateSet()->setMode( GL_CULL_FACE, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED );

    _root = new osg::Group();
    int baseBin = 100;

    // First, if we need to render the children to the depth buffer, create a depth
    // buffer only pass. This is necessary if we are masking out terrain, for example. We have
    // to populate the depth buffer before this algoritm can work.
    if ( _preRenderChildrenToDepthBuffer )
    {
        _depthPass = new osg::Group();
        _depthPass->setName( "StencilVolumeNode::depth_pass" );
        osg::StateSet* ss = _depthPass->getOrCreateStateSet();
        ss->setAttributeAndModes(new osg::ColorMask(false,false,false,false), ON_AND_PROTECTED);
        //ss->setAttributeAndModes( new osg::CullFace(osg::CullFace::BACK), ON_AND_PROTECTED);
        ss->setAttributeAndModes( new osg::Depth(osg::Depth::LESS), ON_AND_PROTECTED);
        ss->setRenderBinDetails( baseBin++, "RenderBin" );

        _root->addChild( _depthPass );
    }

    // Create the stenciled geometry pass:

    // For now we are hard-coding these to what are hopefully "safe" values.
    // In the future we hope to rewrite this as a custom StencilVolumeNode that 
    // will automatically detect extension availability at draw time and choose
    // the optimal GL rendering path.
    const Capabilities& caps = osgEarth::Registry::instance()->getCapabilities();
    bool s_EXT_stencil_wrap     = caps.supportsStencilWrap(); //true;
    bool s_EXT_stencil_two_side = caps.supportsTwoSidedStencil(); //false;


    // zFail=true if more compute intensive, but lets you get inside the volume.
    // Again, a custom node will give us a better opportunity to choose between zFail and zPass based on
    // the eye location (you only need zFail if you camera is inside the volume).
    bool zFail = true;

    OE_DEBUG << "Stencil buffer wrap = " << s_EXT_stencil_wrap << std::endl;

    if ( s_EXT_stencil_two_side )
    {
        OE_DEBUG << "Two-sided stenciling" << std::endl;

        osg::StencilTwoSided::Operation incrOp = s_EXT_stencil_wrap ? osg::StencilTwoSided::INCR_WRAP : osg::StencilTwoSided::INCR;
        osg::StencilTwoSided::Operation decrOp = s_EXT_stencil_wrap ? osg::StencilTwoSided::DECR_WRAP : osg::StencilTwoSided::DECR;
        _stencilGroup1 = new osg::Group();
        //osg::Group* stencil_group = new osg::Group();
        osg::StateSet* ss = _stencilGroup1->getOrCreateStateSet();
        ss->setRenderBinDetails( baseBin++, "RenderBin" );

        if ( zFail )
        {
            osg::StencilTwoSided* stencil = new osg::StencilTwoSided();
            stencil->setFunction(osg::StencilTwoSided::BACK, osg::StencilTwoSided::ALWAYS, 1, ~0u); 
            stencil->setOperation(osg::StencilTwoSided::BACK, osg::StencilTwoSided::KEEP, incrOp, osg::StencilTwoSided::KEEP);

            stencil->setFunction(osg::StencilTwoSided::FRONT, osg::StencilTwoSided::ALWAYS, 1, ~0u); 
            stencil->setOperation(osg::StencilTwoSided::FRONT, osg::StencilTwoSided::KEEP, decrOp, osg::StencilTwoSided::KEEP);
            ss->setAttributeAndModes( stencil, ON_AND_PROTECTED );
        }
        else
        {
            osg::StencilTwoSided* stencil = new osg::StencilTwoSided();
            stencil->setFunction(osg::StencilTwoSided::FRONT, osg::StencilTwoSided::ALWAYS, 1, ~0u); 
            stencil->setOperation(osg::StencilTwoSided::FRONT, osg::StencilTwoSided::KEEP, osg::StencilTwoSided::KEEP, incrOp);

            stencil->setFunction(osg::StencilTwoSided::BACK, osg::StencilTwoSided::ALWAYS, 1, ~0u); 
            stencil->setOperation(osg::StencilTwoSided::BACK, osg::StencilTwoSided::KEEP, osg::StencilTwoSided::KEEP, decrOp);
            ss->setAttributeAndModes( stencil, ON_AND_PROTECTED );
        }

        ss->setAttributeAndModes(new osg::ColorMask(false,false,false,false), ON_AND_PROTECTED);
        ss->setAttributeAndModes(new osg::Depth(osg::Depth::LESS,0,1,false), ON_AND_PROTECTED);

        _root->addChild( _stencilGroup1 );
    }
    else
    {
        OE_INFO << "One-sided stenciling" << std::endl;

        if ( !zFail )  // Z-Pass
        {
            _stencilGroup1 = new osg::Group();
            osg::StateSet* front_ss = _stencilGroup1->getOrCreateStateSet();
            front_ss->setRenderBinDetails( baseBin++, "RenderBin" );

            // incrementing stencil op for front faces:
            osg::Stencil* front_stencil = new osg::Stencil();
            front_stencil->setFunction( osg::Stencil::ALWAYS, 1, ~0u );
            osg::Stencil::Operation incrOp = s_EXT_stencil_wrap ? osg::Stencil::INCR_WRAP : osg::Stencil::INCR;
            front_stencil->setOperation( osg::Stencil::KEEP, osg::Stencil::KEEP, incrOp );
            front_ss->setAttributeAndModes( front_stencil, ON_AND_PROTECTED );

            front_ss->setAttributeAndModes( new osg::ColorMask(false,false,false,false), ON_AND_PROTECTED);
            front_ss->setAttributeAndModes( new osg::CullFace(osg::CullFace::BACK), ON_AND_PROTECTED);
            front_ss->setAttributeAndModes( new osg::Depth(osg::Depth::LESS,0,1,false), ON_AND_PROTECTED);

            _root->addChild( _stencilGroup1 );

            // decrementing stencil op for back faces:
            _stencilGroup2 = new osg::Group();
            osg::StateSet* back_ss = _stencilGroup2->getOrCreateStateSet();
            back_ss->setRenderBinDetails( baseBin++, "RenderBin" );

            osg::Stencil* back_stencil = new osg::Stencil();
            back_stencil->setFunction( osg::Stencil::ALWAYS, 1, ~0u );
            osg::Stencil::Operation decrOp = s_EXT_stencil_wrap ? osg::Stencil::DECR_WRAP : osg::Stencil::DECR;
            back_stencil->setOperation( osg::Stencil::KEEP, osg::Stencil::KEEP, decrOp );
            back_ss->setAttributeAndModes( back_stencil, ON_AND_PROTECTED );

            back_ss->setAttributeAndModes( new osg::ColorMask(false,false,false,false), ON_AND_PROTECTED);
            back_ss->setAttributeAndModes( new osg::CullFace(osg::CullFace::FRONT), ON_AND_PROTECTED);
            back_ss->setAttributeAndModes( new osg::Depth(osg::Depth::LESS,0,1,false), ON_AND_PROTECTED);

            _root->addChild( _stencilGroup2 );    
        }
        else
        {
            // incrementing stencil op for back faces:
            {
                _stencilGroup1 = new osg::Group();
                osg::StateSet* front_ss = _stencilGroup1->getOrCreateStateSet();
                front_ss->setRenderBinDetails( baseBin++, "RenderBin" );

                osg::Stencil* front_stencil = new osg::Stencil();
                front_stencil->setFunction( osg::Stencil::ALWAYS ); //, 1, ~0u );
                osg::Stencil::Operation incrOp = s_EXT_stencil_wrap ? osg::Stencil::INCR_WRAP : osg::Stencil::INCR;
                front_stencil->setOperation( osg::Stencil::KEEP, incrOp, osg::Stencil::KEEP );
                front_ss->setAttributeAndModes( front_stencil, ON_AND_PROTECTED );

                front_ss->setAttributeAndModes( new osg::ColorMask(false,false,false,false), ON_AND_PROTECTED);
                front_ss->setAttributeAndModes( new osg::CullFace(osg::CullFace::FRONT), ON_AND_PROTECTED);
                front_ss->setAttributeAndModes( new osg::Depth(osg::Depth::LESS,0,1,false), ON_AND_PROTECTED);

                _root->addChild( _stencilGroup1 );
            }

            // decrementing stencil buf for front faces
            {
                _stencilGroup2 = new osg::Group();
                osg::StateSet* back_ss = _stencilGroup2->getOrCreateStateSet();
                back_ss->setRenderBinDetails( baseBin++, "RenderBin" );

                osg::Stencil* back_stencil = new osg::Stencil();
                back_stencil->setFunction( osg::Stencil::ALWAYS ); //, 1, ~0u );
                osg::Stencil::Operation decrOp = s_EXT_stencil_wrap ? osg::Stencil::DECR_WRAP : osg::Stencil::DECR;
                back_stencil->setOperation( osg::Stencil::KEEP, decrOp, osg::Stencil::KEEP );
                back_ss->setAttributeAndModes( back_stencil, ON_AND_PROTECTED );

                back_ss->setAttributeAndModes( new osg::ColorMask(false,false,false,false), ON_AND_PROTECTED);
                back_ss->setAttributeAndModes( new osg::CullFace(osg::CullFace::BACK), ON_AND_PROTECTED);
                back_ss->setAttributeAndModes( new osg::Depth(osg::Depth::LESS,0,1,false), ON_AND_PROTECTED);

                _root->addChild( _stencilGroup2 );
            }
        }
    }
    

    // NOW build the full-screen quad mask that will paint the volume:

    _renderPass = new osg::Group();
    _renderPass->setName( "StencilVolumeNode::render_pass" );

    osg::Stencil* quad_stencil = new osg::Stencil();
    osg::Stencil::Function func = _inverted ? osg::Stencil::EQUAL : osg::Stencil::NOTEQUAL;
    quad_stencil->setFunction( func, 128, (unsigned int)~0 );
    // this will conveniently re-clear the stencil buffer in prep for the next pass:
    quad_stencil->setOperation( osg::Stencil::REPLACE, osg::Stencil::REPLACE, osg::Stencil::REPLACE );

    osg::StateSet* renderSS = _renderPass->getOrCreateStateSet();
    renderSS->setAttributeAndModes( quad_stencil, ON_AND_PROTECTED );
    renderSS->setRenderBinDetails( baseBin++, "RenderBin" );

    // if we did a pre-render depth pass, adjust the depth buffer function to account for that
    if ( _preRenderChildrenToDepthBuffer )
    {
        renderSS->setAttributeAndModes( new osg::Depth( osg::Depth::LEQUAL ), osg::StateAttribute::ON );
    }

    // testing
    //renderSS->setMode( GL_DEPTH_TEST, OFF_AND_PROTECTED );
    //renderSS->setAttributeAndModes( new osg::Depth( osg::Depth::ALWAYS ), ON_AND_PROTECTED );

    _root->addChild( _renderPass );
}
