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
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/
#include <osgEarthUtil/Shadowing>
#include <osgEarthUtil/Shaders>
#include <osgEarth/CullingUtils>
#include <osgEarth/VirtualProgram>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/Shadowing>
#include <osg/Texture2D>
#include <osg/CullFace>
#include <osg/ValueObject>
#include <osgShadow/ConvexPolyhedron>

#define LC "[ShadowCaster] "

using namespace osgEarth::Util;



ShadowCaster::ShadowCaster() :
_size         ( 2048 ),
_texImageUnit ( 7 ),
_blurFactor   ( 0.001f ),
_color        ( 0.4f ),
_traversalMask( ~0 )
{
    _castingGroup = new osg::Group();

    _supported = Registry::capabilities().supportsGLSL();
    if ( _supported )
    {
        // default slices:
        _ranges.push_back(0.0f);
        _ranges.push_back(1750.0f);
        _ranges.push_back(5000.0f);

        reinitialize();
    }
    else
    {
        OE_WARN << LC << "ShadowCaster not supported (no GLSL); disabled." << std::endl;
    }
}

void
ShadowCaster::setRanges(const std::vector<float>& ranges)
{
    _ranges = ranges;
    reinitialize();
}

void
ShadowCaster::setTextureImageUnit(int unit)
{
    _texImageUnit = unit;
    reinitialize();
}

void
ShadowCaster::setTextureSize(unsigned size)
{
    _size = size;
    reinitialize();
}

void
ShadowCaster::setBlurFactor(float value)
{
    _blurFactor = value;
    if ( _shadowBlurUniform.valid() )
        _shadowBlurUniform->set(value);
}

void
ShadowCaster::setShadowColor(float value)
{
    _color = value;
    if ( _shadowColorUniform.valid() )
        _shadowColorUniform->set(value);
}

void
ShadowCaster::reinitialize()
{
    if ( !_supported )
        return;

    _shadowmap = 0L;
    _rttCameras.clear();

    int numSlices = (int)_ranges.size() - 1;
    if ( numSlices < 1 )
    {
        OE_WARN << LC << "Illegal. Must have at least one range slice." << std::endl;
        return ;
    }

    // create the projected texture:
    _shadowmap = new osg::Texture2DArray();
    _shadowmap->setTextureSize( _size, _size, numSlices );
    _shadowmap->setInternalFormat( GL_DEPTH_COMPONENT );
    _shadowmap->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR );
    _shadowmap->setFilter( osg::Texture::MAG_FILTER, osg::Texture::LINEAR );
    _shadowmap->setWrap( osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_BORDER );
    _shadowmap->setWrap( osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_BORDER );
    _shadowmap->setBorderColor(osg::Vec4(1,1,1,1));

    // set up the RTT camera:
    for(int i=0; i<numSlices; ++i)
    {
        osg::Camera* rtt = new osg::Camera();
        Shadowing::setIsShadowCamera(rtt);
        rtt->setReferenceFrame( osg::Camera::ABSOLUTE_RF_INHERIT_VIEWPOINT );
        rtt->setClearDepth( 1.0 );
        rtt->setClearMask( GL_DEPTH_BUFFER_BIT );
        rtt->setComputeNearFarMode( osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR );
        rtt->setViewport( 0, 0, _size, _size );
        rtt->setRenderOrder( osg::Camera::PRE_RENDER );
        rtt->setRenderTargetImplementation( osg::Camera::FRAME_BUFFER_OBJECT );
        rtt->setImplicitBufferAttachmentMask(0, 0);
        rtt->attach( osg::Camera::DEPTH_BUFFER, _shadowmap.get(), 0, i );
        rtt->addChild( _castingGroup.get() );
        _rttCameras.push_back(rtt);
    }

    _rttStateSet = new osg::StateSet();

    // only draw back faces to the shadow depth map
    _rttStateSet->setAttributeAndModes( 
        new osg::CullFace(osg::CullFace::FRONT),
        osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);

    _rttStateSet->addUniform(new osg::Uniform("oe_isShadowCamera", true), osg::StateAttribute::OVERRIDE);


    _renderStateSet = new osg::StateSet();
    

    // Establish a Virtual Program on the stateset.
    VirtualProgram* vp = VirtualProgram::getOrCreate(_renderStateSet.get());

    // Load the shadowing shaders.
    Shaders package;
    package.replace("$OE_SHADOW_NUM_SLICES", Stringify()<<numSlices);
    package.load(vp, package.Shadowing_Vertex);
    package.load(vp, package.Shadowing_Fragment);

    // the texture coord generator matrix array (from the caster):
    _shadowMapTexGenUniform = _renderStateSet->getOrCreateUniform(
        "oe_shadow_matrix",
        osg::Uniform::FLOAT_MAT4,
        numSlices );

    // bind the shadow map texture itself:
    _renderStateSet->setTextureAttribute(_texImageUnit, _shadowmap.get(), osg::StateAttribute::ON );
    _renderStateSet->addUniform( new osg::Uniform("oe_shadow_map", _texImageUnit) );

    // blur factor:
    _shadowBlurUniform = _renderStateSet->getOrCreateUniform("oe_shadow_blur", osg::Uniform::FLOAT);
    _shadowBlurUniform->set(_blurFactor);

    // shadow color:
    _shadowColorUniform = _renderStateSet->getOrCreateUniform("oe_shadow_color", osg::Uniform::FLOAT);

    _shadowColorUniform->set(_color);
}

void
ShadowCaster::traverse(osg::NodeVisitor& nv)
{
    if (_supported                             && 
        _light.valid()                         &&
        nv.getVisitorType() == nv.CULL_VISITOR && 
        _castingGroup->getNumChildren() > 0    && 
        _shadowmap.valid() )
    {
        osgUtil::CullVisitor* cv = Culling::asCullVisitor(nv);
        osg::Camera* camera = cv->getCurrentCamera();
        if ( camera )
        {
            osg::Matrix MV = *cv->getModelViewMatrix();
            osg::Matrix inverseMV;
            inverseMV.invert(MV);

            osg::Vec3d camEye, camTo, camUp;
            MV.getLookAt( camEye, camTo, camUp, 1.0 );

            // position the light. We only really care about the directional vector.
            osg::Vec4d lp4 = _light->getPosition();
            osg::Vec3d lightVectorWorld( -lp4.x(), -lp4.y(), -lp4.z() );
            lightVectorWorld.normalize();
            osg::Vec3d lightPosWorld = osg::Vec3d(0,0,0) * inverseMV;
            //osg::Vec3d lightPosWorld( lp4.x(), lp4.y(), lp4.z() ); // jitter..

            // construct the view matrix for the light. The up vector doesn't really
            // matter so we'll just use the camera's.
            osg::Matrix lightViewMat;
            osg::Vec3d lightUp(0,0,1);
            osg::Vec3d side = lightVectorWorld ^ lightUp;
            lightUp = side ^ lightVectorWorld;
            lightUp.normalize();
            lightViewMat.makeLookAt(lightPosWorld, lightPosWorld+lightVectorWorld, lightUp);
            
            //int i = nv.getFrameStamp()->getFrameNumber() % (_ranges.size()-1);
            int i;
            for(i=0; i < (int) _ranges.size()-1; ++i)
            {
                double n = _ranges[i];
                double f = _ranges[i+1];

                // take the camera's projection matrix and clamp it's near and far planes
                // to our shadow map slice range.
                osg::Matrix proj = _prevProjMatrix;
                double fovy,ar,zn,zf;
                proj.getPerspective(fovy,ar,zn,zf);
                proj.makePerspective(fovy,ar,std::max(n,zn),std::min(f,zf));
                
                // extract the corner points of the camera frustum in world space.
                osg::Matrix MVP = MV * proj;
                osg::Matrix inverseMVP;
                inverseMVP.invert(MVP);
                osgShadow::ConvexPolyhedron frustumPH;
                frustumPH.setToUnitFrustum(true, true);
                frustumPH.transform( inverseMVP, MVP );
                std::vector<osg::Vec3d> verts;
                frustumPH.getPoints( verts );

                // project those on to the plane of the light camera and fit them
                // to a bounding box. That box will form the extent of our orthographic camera.
                osg::BoundingBoxd bbox;
                for( std::vector<osg::Vec3d>::iterator v = verts.begin(); v != verts.end(); ++v )
                    bbox.expandBy( (*v) * lightViewMat );

                osg::Matrix lightProjMat;
                n = -std::max(bbox.zMin(), bbox.zMax());
                f = -std::min(bbox.zMin(), bbox.zMax());
                // TODO: consider extending "n" so that objects outside the main view can still cast shadows
                lightProjMat.makeOrtho(bbox.xMin(), bbox.xMax(), bbox.yMin(), bbox.yMax(), n, f);

                // configure the RTT camera for this slice:
                _rttCameras[i]->setViewMatrix( lightViewMat );
                _rttCameras[i]->setProjectionMatrix( lightProjMat );

                // this xforms from clip [-1..1] to texture [0..1] space
                static osg::Matrix s_scaleBiasMat = 
                    osg::Matrix::translate(1.0,1.0,1.0) * 
                    osg::Matrix::scale(0.5,0.5,0.5);
                
                // set the texture coordinate generation matrix that the shadow
                // receiver will use to sample the shadow map. Doing this on the CPU
                // prevents nasty precision issues!
                osg::Matrix VPS = lightViewMat * lightProjMat * s_scaleBiasMat;
                _shadowMapTexGenUniform->setElement(i, inverseMV * VPS);
            }

            // install the shadow-casting traversal mask:
            unsigned saveMask = cv->getTraversalMask();
            cv->setTraversalMask( _traversalMask & saveMask );

            // render the shadow maps.
            cv->pushStateSet( _rttStateSet.get() );
            for(i=0; i < (int) _rttCameras.size(); ++i)
            {
                _rttCameras[i]->accept( nv );
            }
            cv->popStateSet();

            // restore the previous mask
            cv->setTraversalMask( saveMask );
            
            // render the shadowed subgraph.
            cv->pushStateSet( _renderStateSet.get() );
            osg::Group::traverse( nv );
            cv->popStateSet();

            // save the projection matrix for the next frame.
            _prevProjMatrix = *cv->getProjectionMatrix();

            return;
        }
    }

    osg::Group::traverse(nv);
}
