/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2014 Pelican Mapping
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
#include <osgEarth/Utils>
#include <osgEarth/ECEF>
#include <osgEarth/CullingUtils>
#include <osg/Version>
#include <osg/CoordinateSystemNode>
#include <osg/MatrixTransform>
#include <osgUtil/MeshOptimizers>

using namespace osgEarth;

//------------------------------------------------------------------------

void osgEarth::removeEventHandler(osgViewer::View* view, osgGA::GUIEventHandler* handler)
{
    osgViewer::View::EventHandlers::iterator itr = std::find(view->getEventHandlers().begin(), view->getEventHandlers().end(), handler);
    if (itr != view->getEventHandlers().end())
    {
        view->getEventHandlers().erase(itr);
    }
}

//------------------------------------------------------------------------

#undef LC
#define LC "[PixelAutoTransform] "

PixelAutoTransform::PixelAutoTransform() :
osg::AutoTransform         (),
_rotateInScreenSpace       ( false ),
_screenSpaceRotationRadians( 0.0 )
{
    // deactivate culling for the first traversal. We will reactivate it later.
    setCullingActive( false );
    setMinimumScale ( 1.0 );
    setMinPixelWidthAtScaleOne( 10 );
}

void
PixelAutoTransform::accept( osg::NodeVisitor& nv )
{
    if ( nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR )
    {
        // re-activate culling now that the first cull traversal has taken place.
        this->setCullingActive( true );
        osgUtil::CullVisitor* cv = Culling::asCullVisitor(nv);
        if ( cv )
        {
            osg::Viewport::value_type width  = _previousWidth;
            osg::Viewport::value_type height = _previousHeight;

            osg::Viewport* viewport = cv->getViewport();
            if (viewport)
            {
                width = viewport->width();
                height = viewport->height();
            }

            osg::Vec3d eyePoint = cv->getEyeLocal(); 
            osg::Vec3d localUp = cv->getUpLocal(); 
            osg::Vec3d position = getPosition();

            const osg::Matrix& projection = *(cv->getProjectionMatrix());

            bool doUpdate = _firstTimeToInitEyePoint || _dirty;
            if ( !_firstTimeToInitEyePoint )
            {
                osg::Vec3d dv = _previousEyePoint - eyePoint;

                if (dv.length2() > getAutoUpdateEyeMovementTolerance() * (eyePoint-getPosition()).length2())
                {
                    doUpdate = true;
                }
                else
                {
                    osg::Vec3d dupv = _previousLocalUp - localUp;

                    // rotating the camera only affects ROTATE_TO_*
                    if ((_autoRotateMode && dupv.length2() > getAutoUpdateEyeMovementTolerance()) ||
                        (width != _previousWidth || height != _previousHeight) ||
                        (projection != _previousProjection) ||
                        (position != _previousPosition) )
                    {
                        doUpdate = true;
                    }
                }
            }
            _firstTimeToInitEyePoint = false;

            if ( doUpdate )
            {            
                if ( getAutoScaleToScreen() )
                {
                    double radius =
                        _sizingNode.valid() ? _sizingNode->getBound().radius() :
                        getNumChildren() > 0 ? getChild(0)->getBound().radius() : 
                        0.48;

                    double pixels = cv->pixelSize( getPosition(), radius );

                    double scaledMinPixels = _minPixels * _minimumScale;
                    double scale = pixels < scaledMinPixels ? scaledMinPixels / pixels : 1.0;

                    //OE_DEBUG << LC << "Pixels = " << pixels << ", minPix = " << _minPixels << ", scale = " << scale << std::endl;

                    setScale( scale );
                }

                _previousEyePoint = eyePoint;
                _previousLocalUp = localUp;
                _previousWidth = width;
                _previousHeight = height;
                _previousProjection = projection;
                _previousPosition = position;

                _matrixDirty = true;
            }

            if (_rotateInScreenSpace==true)
            {
                osg::Vec3d translation, scale;
                osg::Quat  rotation, so;
                osg::RefMatrix& mvm = *(cv->getModelViewMatrix());

                mvm.decompose( translation, rotation, scale, so );

                // this will rotate the object into screen space.
                osg::Quat toScreen( rotation.inverse() );

                // we need to compensate for the "heading" of the camera, so compute that.
                // From (http://goo.gl/9bjM4t).
                // GEOCENTRIC ONLY!

                const osg::Matrixd& view = cv->getCurrentCamera()->getViewMatrix();
                osg::Matrixd viewInverse;
                viewInverse.invert(view);

                osg::Vec3d N(0, 0, 6356752); // north pole, more or less
                osg::Vec3d b( -view(0,2), -view(1,2), -view(2,2) ); // look vector
                osg::Vec3d E = osg::Vec3d(0,0,0)*viewInverse;
                osg::Vec3d u = E; u.normalize();

                // account for looking straight downish
                if ( osg::equivalent(b*u, -1.0, 1e-4) )
                {
                    // up vec becomes the look vec.
                    b = osg::Matrixd::transform3x3(view, osg::Vec3f(0.0,1.0,0.0));
                    b.normalize();
                }

                osg::Vec3d proj_d = b - u*(b*u);
                osg::Vec3d n = N - E;
                osg::Vec3d proj_n = n - u*(n*u);
                osg::Vec3d proj_e = proj_n^u;

                double cameraHeading = atan2(proj_e*proj_d, proj_n*proj_d);

                //OE_NOTICE << "h=" << osg::RadiansToDegrees(cameraHeading) << std::endl;

                while (cameraHeading < 0.0)
                    cameraHeading += osg::PI*2.0;
                double objHeading = _screenSpaceRotationRadians;
                while ( objHeading < 0.0 )
                    objHeading += osg::PI*2.0;
                double finalRot = cameraHeading - objHeading;
                while( finalRot > osg::PI )
                    finalRot -= osg::PI*2.0;

                osg::Quat toRotation( finalRot, osg::Vec3(0,0,1) );

                setRotation( toRotation * toScreen );
            }

            else if (_autoRotateMode==ROTATE_TO_SCREEN)
            {
                osg::Vec3d translation;
                osg::Quat rotation;
                osg::Vec3d scale;
                osg::Quat so;

                cv->getModelViewMatrix()->decompose( translation, rotation, scale, so );

                setRotation(rotation.inverse());
            }
            else if (_autoRotateMode==ROTATE_TO_CAMERA)
            {
                osg::Vec3d PosToEye = _position - eyePoint;
                osg::Matrix lookto = osg::Matrix::lookAt(
                    osg::Vec3d(0,0,0), PosToEye, localUp);
                osg::Quat q;
                q.set(osg::Matrix::inverse(lookto));
                setRotation(q);
            }

            else if (_autoRotateMode==ROTATE_TO_AXIS)
            {
                osg::Matrix matrix;
                osg::Vec3 ev(eyePoint - _position);

                switch(_cachedMode)
                {
                case(AXIAL_ROT_Z_AXIS):
                    {
                        ev.z() = 0.0f;
                        float ev_length = ev.length();
                        if (ev_length>0.0f)
                        {
                            //float rotation_zrotation_z = atan2f(ev.x(),ev.y());
                            //mat.makeRotate(inRadians(rotation_z),0.0f,0.0f,1.0f);
                            float inv = 1.0f/ev_length;
                            float s = ev.x()*inv;
                            float c = -ev.y()*inv;
                            matrix(0,0) = c;
                            matrix(1,0) = -s;
                            matrix(0,1) = s;
                            matrix(1,1) = c;
                        }
                        break;
                    }
                case(AXIAL_ROT_Y_AXIS):
                    {
                        ev.y() = 0.0f;
                        float ev_length = ev.length();
                        if (ev_length>0.0f)
                        {
                            //float rotation_zrotation_z = atan2f(ev.x(),ev.y());
                            //mat.makeRotate(inRadians(rotation_z),0.0f,0.0f,1.0f);
                            float inv = 1.0f/ev_length;
                            float s = -ev.z()*inv;
                            float c = ev.x()*inv;
                            matrix(0,0) = c;
                            matrix(2,0) = s;
                            matrix(0,2) = -s;
                            matrix(2,2) = c;
                        }
                        break;
                    }
                case(AXIAL_ROT_X_AXIS):
                    {
                        ev.x() = 0.0f;
                        float ev_length = ev.length();
                        if (ev_length>0.0f)
                        {
                            //float rotation_zrotation_z = atan2f(ev.x(),ev.y());
                            //mat.makeRotate(inRadians(rotation_z),0.0f,0.0f,1.0f);
                            float inv = 1.0f/ev_length;
                            float s = -ev.z()*inv;
                            float c = -ev.y()*inv;
                            matrix(1,1) = c;
                            matrix(2,1) = -s;
                            matrix(1,2) = s;
                            matrix(2,2) = c;
                        }
                        break;
                    }
                case(ROTATE_TO_AXIS): // need to implement 
                    {
                        float ev_side = ev*_side;
                        float ev_normal = ev*_normal;
                        float rotation = atan2f(ev_side,ev_normal);
                        matrix.makeRotate(rotation,_axis);
                        break;
                    }
                }
                osg::Quat q;
                q.set(matrix);
                setRotation(q);
            }

            _dirty = false;

            // update the LOD Scale based on the auto-scale.
            cv->setLODScale( 1.0/getScale().x() );

        } // if (cv)
    } // if is cull visitor

    // finally, skip AT's accept and do Transform.
    Transform::accept(nv);
}

void
PixelAutoTransform::dirty()
{
    _dirty = true;
    setCullingActive( false );
}

//-----------------------------------------------------------------------------

#undef  LC
#define LC "[VertexCacheOptimizer] "

VertexCacheOptimizer::VertexCacheOptimizer() :
osg::NodeVisitor( TRAVERSE_ALL_CHILDREN ) 
{
    //nop
}

void
VertexCacheOptimizer::apply(osg::Geode& geode)
{
    if (geode.getDataVariance() == osg::Object::DYNAMIC)
        return;

    for(unsigned i=0; i<geode.getNumDrawables(); ++i )
    {
        osg::Geometry* geom = geode.getDrawable(i)->asGeometry();

        if ( geom )
        {
            if ( geom->getDataVariance() == osg::Object::DYNAMIC )
                return;

            // vertex cache optimizations currently only support surface geometries.
            // all or nothing in the geode.
            osg::Geometry::PrimitiveSetList& psets = geom->getPrimitiveSetList();
            for( osg::Geometry::PrimitiveSetList::iterator i = psets.begin(); i != psets.end(); ++i )
            {
                switch( (*i)->getMode() )
                {
                case GL_TRIANGLES:
                case GL_TRIANGLE_FAN:
                case GL_TRIANGLE_STRIP:
                case GL_QUADS:
                case GL_QUAD_STRIP:
                case GL_POLYGON:
                    break;

                default:
                    return;
                }
            }
        }
    }

    //OE_NOTICE << LC << "VC optimizing..." << std::endl;

    // passed the test; run the optimizer.
    osgUtil::VertexCacheVisitor vcv;
    geode.accept( vcv );
    vcv.optimizeVertices();

    osgUtil::VertexAccessOrderVisitor vaov;
    geode.accept( vaov );
    vaov.optimizeOrder();

    traverse( geode );
}

//-----------------------------------------------------------------------------

#undef  LC
#define LC "[SetDataVarianceVisitor] "

SetDataVarianceVisitor::SetDataVarianceVisitor(osg::Object::DataVariance value) :
osg::NodeVisitor( TRAVERSE_ALL_CHILDREN ),
_value( value )
{
    //nop
}

void
SetDataVarianceVisitor::apply(osg::Geode& geode)
{
    for(unsigned i=0; i<geode.getNumDrawables(); ++i)
    {
        osg::Drawable* d = geode.getDrawable(i);
        if ( d )
            d->setDataVariance( _value );
    }

    traverse(geode);
}

//-----------------------------------------------------------------------------

namespace
{
    template<typename DE>
    void validateDE( DE* de, unsigned maxIndex, unsigned numVerts )
    {
        for( unsigned i=0; i<de->getNumIndices(); ++i )
        {
            typename DE::value_type index = de->getElement(i);
            if ( index > maxIndex )
            {
                OE_WARN << "MAXIMUM Index exceeded in DrawElements" << std::endl;
                break;
            }
            else if ( index > numVerts-1 )
            {
                OE_WARN << "INDEX OUT OF Range in DrawElements" << std::endl;
            }
        }
    }
}


GeometryValidator::GeometryValidator()
{
    setVisitorType(this->NODE_VISITOR);
    setTraversalMode(this->TRAVERSE_ALL_CHILDREN);
    setNodeMaskOverride(~0);
}

void
GeometryValidator::apply(osg::Geometry& geom)
{
    unsigned numVerts = geom.getVertexArray()->getNumElements();

    if ( geom.getColorArray() )
    {
        if ( geom.getColorBinding() == osg::Geometry::BIND_OVERALL && geom.getColorArray()->getNumElements() != 1 )
        {
            OE_WARN << "Color: BIND_OVERALL with wrong number of elements" << std::endl;
        }
        else if ( geom.getColorBinding() == osg::Geometry::BIND_PER_VERTEX && geom.getColorArray()->getNumElements() != numVerts )
        {
            OE_WARN << "Color: BIND_PER_VERTEX with color.size != verts.size" << std::endl;
        }
    }

    if ( geom.getNormalArray() )
    {
        if ( geom.getNormalBinding() == osg::Geometry::BIND_OVERALL && geom.getNormalArray()->getNumElements() != 1 )
        {
            OE_WARN << "Normal: BIND_OVERALL with wrong number of elements" << std::endl;
        }
        else if ( geom.getNormalBinding() == osg::Geometry::BIND_PER_VERTEX && geom.getNormalArray()->getNumElements() != numVerts )
        {
            OE_WARN << "Normal: BIND_PER_VERTEX with color.size != verts.size" << std::endl;
        }
    }

    const osg::Geometry::PrimitiveSetList& plist = geom.getPrimitiveSetList();

    for( osg::Geometry::PrimitiveSetList::const_iterator p = plist.begin(); p != plist.end(); ++p )
    {
        osg::PrimitiveSet* pset = p->get();

        osg::DrawElementsUByte* de_byte = dynamic_cast<osg::DrawElementsUByte*>(pset);
        if ( de_byte )
        {
            if ( numVerts > 0xFF )
            {
                OE_WARN << "DrawElementsUByte used when numVerts > 255 (" << numVerts << ")" << std::endl;
            }
            validateDE(de_byte, 0xFF, numVerts );
        }

        osg::DrawElementsUShort* de_short = dynamic_cast<osg::DrawElementsUShort*>(pset);
        if ( de_short )
        {
            if ( numVerts > 0xFFFF )
            {
                OE_WARN << "DrawElementsUShort used when numVerts > 65535 (" << numVerts << ")" << std::endl;
            }
            validateDE(de_short, 0xFFFF, numVerts );
        }

        osg::DrawElementsUInt* de_int = dynamic_cast<osg::DrawElementsUInt*>(pset);
        if ( de_int )
        {
            validateDE(de_int, 0xFFFFFFFF, numVerts );
        }
    }
}

void
GeometryValidator::apply(osg::Geode& geode)
{
    for(unsigned i=0; i<geode.getNumDrawables(); ++i)
    {
        osg::Geometry* geom = geode.getDrawable(i)->asGeometry();
        if ( geom )
            apply( *geom );
    }
}