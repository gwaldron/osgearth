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
#include <osgEarth/AutoScale>
#include <osgEarth/ThreadingUtils>
#include <osgEarth/Utils>
#include <osgEarth/VirtualProgram>
#include <osgUtil/RenderBin>

#define LC "[AutoScale] "

using namespace osgEarth;

#define AUTO_SCALE_BIN_NAME "osgEarth::AutoScale"
const std::string osgEarth::AUTO_SCALE_BIN = AUTO_SCALE_BIN_NAME;

namespace
{
    const char* vs =
        "#version " GLSL_VERSION_STR "\n"
        GLSL_DEFAULT_PRECISION_FLOAT "\n"
        "uniform float oe_autoscale_v; \n"      // viewport width / 2.
        "uniform float oe_autoscale_f; \n"      // horizontal FOV / 2.
        "uniform vec3  oe_autoscale_c; \n"

        "void oe_autoscale_vertex( inout vec4 VertexVIEW ) \n"
        "{ \n"
        "    VertexVIEW.xyz /= VertexVIEW.w; \n"
        "    float tanf    = tan(oe_autoscale_f); \n"
        "    float z       = -VertexVIEW.z; \n"
        "    float zp      = oe_autoscale_v / tanf; \n"
        "    VertexVIEW.z *= (zp/z); \n"
        "    vec3 push     = normalize(VertexVIEW.xyz); \n"
        "    float newf    = atan(oe_autoscale_v/zp); \n"
        "    float a       = z/cos(newf); \n"
        "    VertexVIEW.xyz = push * a * VertexVIEW.w; \n"
        "} \n";


    class AutoScaleRenderBin : public osgUtil::RenderBin
    {
    public:
        struct PerCameraData
        {
        };

        // shared across ALL render bin instances.
        typedef Threading::PerObjectMap<osg::Camera*, PerCameraData> PerCameraDataMap;
        PerCameraDataMap* _pvd;

        osg::ref_ptr<osg::Uniform>  _viewportWidth;
        osg::ref_ptr<osg::Uniform>  _hfovDiv2;

        // support cloning (from RenderBin):
        virtual osg::Object* cloneType() const { return new AutoScaleRenderBin(); }
        virtual osg::Object* clone(const osg::CopyOp& copyop) const { return new AutoScaleRenderBin(*this,copyop); } // note only implements a clone of type.
        virtual bool isSameKindAs(const osg::Object* obj) const { return dynamic_cast<const AutoScaleRenderBin*>(obj)!=0L; }
        virtual const char* libraryName() const { return "osgEarth"; }
        virtual const char* className() const { return "AutoScaleRenderBin"; }


        // constructs the prototype for this render bin.
        AutoScaleRenderBin() : osgUtil::RenderBin()
        {
            this->setName( osgEarth::AUTO_SCALE_BIN );
            _pvd = new PerCameraDataMap();

            _stateset = new osg::StateSet();

            VirtualProgram* vp = new VirtualProgram();
            vp->setFunction( "oe_autoscale_vertex", vs, ShaderComp::LOCATION_VERTEX_VIEW );
            _stateset->setAttributeAndModes( vp, 1 );

            _viewportWidth = _stateset->getOrCreateUniform("oe_autoscale_v", osg::Uniform::FLOAT);
            _hfovDiv2      = _stateset->getOrCreateUniform("oe_autoscale_f", osg::Uniform::FLOAT);
        }

        AutoScaleRenderBin( const AutoScaleRenderBin& rhs, const osg::CopyOp& op )
            : osgUtil::RenderBin( rhs, op ),
              _pvd          ( rhs._pvd ),
              _viewportWidth( rhs._viewportWidth.get() ),
              _hfovDiv2     ( rhs._hfovDiv2.get() )
        {
            //nop
        }

        // override.
        void sortImplementation()
        {
            copyLeavesFromStateGraphListToRenderLeafList();
        }

        /**
         * Draws a bin. Most of this code is copied from osgUtil::RenderBin::drawImplementation.
         * The modifications are (a) skipping code to render child bins, (b) setting a bin-global
         * projection matrix in orthographic space, and (c) calling our custom "renderLeaf()" method 
         * instead of RenderLeaf::render()
         * (override)
         */
        void drawImplementation( osg::RenderInfo& renderInfo, osgUtil::RenderLeaf*& previous)
        {
            osg::State& state = *renderInfo.getState();

            unsigned int numToPop = (previous ? osgUtil::StateGraph::numToPop(previous->_parent) : 0);
            if (numToPop>1) --numToPop;
            unsigned int insertStateSetPosition = state.getStateSetStackSize() - numToPop;

            //if (_stateset.valid() )
            {
                state.insertStateSet(insertStateSetPosition, _stateset.get());
            }

            // apply a window-space projection matrix.
            const osg::Viewport* vp = renderInfo.getCurrentCamera()->getViewport();
            if ( vp )
            {
                _viewportWidth->set( 0.5f*(float)vp->width() );
                _hfovDiv2->set( osg::DegreesToRadians(45.0f) );
            }

            // render the list
            osgUtil::RenderBin::RenderLeafList& leaves = this->getRenderLeafList();
            for(osgUtil::RenderBin::RenderLeafList::reverse_iterator rlitr = leaves.rbegin();
                rlitr!= leaves.rend();
                ++rlitr)
            {
                osgUtil::RenderLeaf* leaf = *rlitr;
                leaf->render( renderInfo, previous );
                previous = leaf;
            }

            //if ( this->getStateSet() ) // always true
            {
                state.removeStateSet(insertStateSetPosition);
            }
        }
    };
}

/** static registration of the bin */
extern "C" void osgEarth_AutoScaleBin_registration(void) {}
static osgEarthRegisterRenderBinProxy<AutoScaleRenderBin> s_regbin(AUTO_SCALE_BIN_NAME);
