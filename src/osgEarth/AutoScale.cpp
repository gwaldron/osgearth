/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2016 Pelican Mapping
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
#include <osgEarth/Registry>
#include <osgEarth/ShaderFactory>
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

        "uniform float oe_autoscale_zp; \n"

        "void oe_autoscale_vertex( inout vec4 VertexVIEW ) \n"
        "{ \n"
        "    float z       = -VertexVIEW.z; \n"

        "    vec4  cp       = gl_ModelViewMatrix * vec4(0.0,0.0,0.0,1.0); \n" // control point into view space
        "    float d        = length(cp.xyz); \n"
        "    vec3  cpn      = cp.xyz/d; \n"
        "    vec3  off      = VertexVIEW.xyz - cp.xyz; \n"

        "    float dp = (d * oe_autoscale_zp) / z; \n"
        "    cp.xyz   = cpn * dp; \n"

        "    VertexVIEW.z *= (oe_autoscale_zp/z); \n"
        "    VertexVIEW.xy = cp.xy + off.xy; \n"

        "    vec3 push      = normalize(VertexVIEW.xyz); \n"
        "    VertexVIEW.xyz = push * z; \n"
        "} \n";


    class AutoScaleRenderBin : public osgUtil::RenderBin
    {
    public:
        osg::ref_ptr<osg::Uniform>  _zp;

        // support cloning (from RenderBin):
        virtual osg::Object* cloneType() const { return new AutoScaleRenderBin(); }
        virtual osg::Object* clone(const osg::CopyOp& copyop) const { return new AutoScaleRenderBin(*this,copyop); } // note only implements a clone of type.
        virtual bool isSameKindAs(const osg::Object* obj) const { return dynamic_cast<const AutoScaleRenderBin*>(obj)!=0L; }
        virtual const char* libraryName() const { return "osgEarth"; }
        virtual const char* className() const { return "AutoScaleRenderBin"; }


        // constructs the prototype for this render bin.
        AutoScaleRenderBin() : osgUtil::RenderBin()
        {
            //OE_NOTICE << LC << "AUTOSCALE: created bin." << std::endl;

            this->setName( osgEarth::AUTO_SCALE_BIN );

            _stateset = new osg::StateSet();

            VirtualProgram* vp = VirtualProgram::getOrCreate(_stateset.get());
            vp->setFunction( "oe_autoscale_vertex", vs, ShaderComp::LOCATION_VERTEX_VIEW, 0L, 0.5f );

            _zp = _stateset->getOrCreateUniform("oe_autoscale_zp", osg::Uniform::FLOAT);
        }

        AutoScaleRenderBin( const AutoScaleRenderBin& rhs, const osg::CopyOp& op )
            : osgUtil::RenderBin( rhs, op ),
              _zp      ( rhs._zp.get() )
        {
            //nop
            //OE_NOTICE << LC << "AUTOSCALE: cloned bin." << std::endl;
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
            //OE_NOTICE << LC << "AUTOSCALE: leaves = " << getNumLeaves(this) << std::endl;

            // apply a window-space projection matrix.
            const osg::Viewport* vp = renderInfo.getCurrentCamera()->getViewport();
            if ( vp )
            {
                float vpw = 0.5f*(float)vp->width();

                double fovy, aspectRatio, n, f;
                renderInfo.getCurrentCamera()->getProjectionMatrixAsPerspective(fovy, aspectRatio, n, f);
                float hfovd2 = (float)(0.5*fovy*aspectRatio);

                _zp->set( vpw / tanf(osg::DegreesToRadians(hfovd2)) );
            }
    
            osgUtil::RenderBin::drawImplementation(renderInfo, previous);
        }

        // debugging - counts the total # of leaves in this bin and its children
        static unsigned getNumLeaves(osgUtil::RenderBin* bin)
        {
            unsigned here = bin->getRenderLeafList().size();
            for( RenderBinList::iterator i = bin->getRenderBinList().begin(); i != bin->getRenderBinList().end(); ++i )
                here += getNumLeaves( i->second.get() );
            return here;
        }
    };
}

/** static registration of the bin */
//extern "C" void osgEarth_AutoScaleBin_registration(void) {}
//static osgEarthRegisterRenderBinProxy<AutoScaleRenderBin> s_regbin(AUTO_SCALE_BIN_NAME);
