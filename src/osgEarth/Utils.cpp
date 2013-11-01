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
#include <osgEarth/Utils>
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

void
DoNotComputeNearFarCullCallback::operator()(osg::Node* node, osg::NodeVisitor* nv)
{
    osgUtil::CullVisitor* cv = static_cast< osgUtil::CullVisitor*>( nv );
    osg::CullSettings::ComputeNearFarMode oldMode;
    if( cv )
    {
        oldMode = cv->getComputeNearFarMode();
        cv->setComputeNearFarMode( osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR );
    }
    traverse(node, nv);
    if( cv )
    {
        cv->setComputeNearFarMode(oldMode);
    }
}

//------------------------------------------------------------------------

#undef LC
#define LC "[PixelAutoTransform] "

PixelAutoTransform::PixelAutoTransform() :
osg::AutoTransform()
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
        osg::CullStack* cs = dynamic_cast<osg::CullStack*>(&nv);
        if ( cs )
        {
            osg::Viewport::value_type width  = _previousWidth;
            osg::Viewport::value_type height = _previousHeight;

            osg::Viewport* viewport = cs->getViewport();
            if (viewport)
            {
                width = viewport->width();
                height = viewport->height();
            }

            osg::Vec3d eyePoint = cs->getEyeLocal(); 
            osg::Vec3d localUp = cs->getUpLocal(); 
            osg::Vec3d position = getPosition();

            const osg::Matrix& projection = *(cs->getProjectionMatrix());

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

                    double pixels = cs->pixelSize( getPosition(), radius );

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

            if (_autoRotateMode==ROTATE_TO_SCREEN)
            {
                osg::Vec3d translation;
                osg::Quat rotation;
                osg::Vec3d scale;
                osg::Quat so;

                cs->getModelViewMatrix()->decompose( translation, rotation, scale, so );

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

#if OSG_MIN_VERSION_REQUIRED(3,0,0)
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
#endif

            _dirty = false;
        }
    }

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
