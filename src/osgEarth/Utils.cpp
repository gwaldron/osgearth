/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2019 Pelican Mapping
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
#include <osgUtil/MeshOptimizers>

using namespace osgEarth;

//------------------------------------------------------------------------

#ifdef OE_HAVE_PIXEL_AUTO_TRANSFORM

#undef LC
#define LC "[PixelAutoTransform] "

PixelAutoTransform::PixelAutoTransform() :
osg::AutoTransform         (),
_rotateInScreenSpace       ( false ),
_screenSpaceRotationRadians( 0.0 ),
_dirty( true )
{
    // deactivate culling for the first traversal. We will reactivate it later.
    setCullingActive( false );
    setMinimumScale ( 1.0 );
    setMinPixelWidthAtScaleOne( 10 );
}

void
PixelAutoTransform::accept( osg::NodeVisitor& nv )
{
    // optimization - don't bother with mathing if the node is hidden.
    // (this occurs in Node::accept, which we override here)
    if ( !nv.validNodeMask(*this) )
        return;

    bool resetLodScale = false;
    double oldLodScale = 1.0;
    if ( nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR )
    {
        // re-activate culling now that the first cull traversal has taken place.
        this->setCullingActive( true );
        osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(&nv);
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
            const double xScale = getScale().x();
            if (xScale != 1.0 && xScale != 0.0)
            {
                oldLodScale = cv->getLODScale();
                resetLodScale = true;
                cv->setLODScale( 1.0/xScale );
            }

        } // if (cv)
    } // if is cull visitor

    // finally, skip AT's accept and do Transform.
    Transform::accept(nv);

    // Reset the LOD scale if we changed it
    if (resetLodScale)
    {
        osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(&nv);
        if ( cv )
            cv->setLODScale( oldLodScale );
    }
}

void
PixelAutoTransform::dirty()
{
    _dirty = true;
    setCullingActive( false );
}

#endif // OE_HAVE_PIXEL_AUTO_TRANSFORM

//-----------------------------------------------------------------------------

#undef  LC
#define LC "[VertexCacheOptimizer] "

VertexCacheOptimizer::VertexCacheOptimizer() :
osg::NodeVisitor( TRAVERSE_ALL_CHILDREN ) 
{
    //nop
}

void
VertexCacheOptimizer::apply(osg::Drawable& drawable)
{
    if (drawable.getDataVariance() == osg::Object::DYNAMIC)
        return;

    osg::Geometry* geom = drawable.asGeometry();

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

    //OE_NOTICE << LC << "VC optimizing..." << std::endl;

    // passed the test; run the optimizer.
    osgUtil::VertexCacheVisitor vcv;
    drawable.accept( vcv );
    vcv.optimizeVertices();

    osgUtil::VertexAccessOrderVisitor vaov;
    drawable.accept( vaov );
    vaov.optimizeOrder();

    traverse( drawable );
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
SetDataVarianceVisitor::apply(osg::Drawable& drawable)
{
    drawable.setDataVariance(_value);
    traverse(drawable);
}

//-----------------------------------------------------------------------------

#undef  LC
#define LC "[GeometryValidator] "

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
    if ( geom.getVertexArray() == 0L )
    {
        OE_NOTICE << LC << "NULL vertex array!!\n";
        return;
    }

    unsigned numVerts = geom.getVertexArray()->getNumElements();
    if ( numVerts == 0 )
    {
        OE_NOTICE << LC << "No verts!! name=" << geom.getName() << "\n";
        return;
    }

    std::set<osg::BufferObject*> _vbos;

    osg::Geometry::ArrayList arrays;
    geom.getArrayList(arrays);

    for(unsigned i=0; i<arrays.size(); ++i)
    {
        osg::Array* a = arrays[i].get();
        if ( a )
        {
            if ( a->getBinding() == a->BIND_OVERALL && a->getNumElements() != 1 )
            {
                OE_NOTICE << LC << "Found an array with BIND_OVERALL and size <> 1\n";
            }
            else if ( a->getBinding() == a->BIND_PER_VERTEX && a->getNumElements() != numVerts )
            {
                OE_NOTICE << LC << "Found BIND_PER_VERTEX with wrong number of elements (expecting " << numVerts << "; found " << a->getNumElements() << ")\n";
            }

            _vbos.insert( a->getVertexBufferObject() );
        }
        else
        {
            OE_NOTICE << LC << "Found a NULL array\n";
        }

    }

    if ( _vbos.size() != 1 )
    {
        OE_NOTICE << LC << "Found a Geometry that uses more than one VBO (non-optimal sharing)\n";
    }

    const osg::Geometry::PrimitiveSetList& plist = geom.getPrimitiveSetList();
    
    std::set<osg::BufferObject*> _ebos;

    for( osg::Geometry::PrimitiveSetList::const_iterator p = plist.begin(); p != plist.end(); ++p )
    {
        osg::PrimitiveSet* pset = p->get();

        osg::DrawArrays* da = dynamic_cast<osg::DrawArrays*>(pset);
        if ( da )
        {
            if ( da->getFirst() >= numVerts )
            {
                OE_NOTICE << LC << "DrawArrays: first > numVerts\n";
            }
            if ( da->getFirst()+da->getCount() > numVerts )
            {
                OE_NOTICE << LC << "DrawArrays: first/count out of bounds\n";
            }
            if ( da->getCount() < 1 )
            {
                OE_NOTICE << LC << "DrawArrays: count is zero\n";
            }
        }

        bool isDe = pset->getDrawElements() != 0L;

        osg::DrawElementsUByte* de_byte = dynamic_cast<osg::DrawElementsUByte*>(pset);
        if ( de_byte )
        {
            validateDE(de_byte, 0xFF, numVerts );
            _ebos.insert( de_byte->getElementBufferObject() );
        }

        osg::DrawElementsUShort* de_short = dynamic_cast<osg::DrawElementsUShort*>(pset);
        if ( de_short )
        {
            validateDE(de_short, 0xFFFF, numVerts );
            _ebos.insert( de_short->getElementBufferObject() );
        }

        osg::DrawElementsUInt* de_int = dynamic_cast<osg::DrawElementsUInt*>(pset);
        if ( de_int )
        {
            validateDE(de_int, 0xFFFFFFFF, numVerts );
            _ebos.insert( de_int->getElementBufferObject() );
        }

        if ( pset->getNumIndices() == 0 )
        {
            OE_NOTICE << LC << "Primset: num elements = 0; class=" << pset->className() << ", name=" << pset->getName() << "\n";
        }
        else if ( pset->getType() >= GL_TRIANGLES && pset->getNumIndices() < 3 )
        {
            OE_NOTICE << LC << "Primset: not enough indicies for surface prim type\n";
        }
        else if ( pset->getType() >= GL_LINE_STRIP && pset->getNumIndices() < 2 )
        {
            OE_NOTICE << LC << "Primset: not enough indicies for linear prim type\n";
        }
        else if ( isDe && pset->getType() == GL_LINES && pset->getNumIndices() % 2 != 0 )
        {
            OE_NOTICE << LC << "Primset: non-even index count for GL_LINES\n";
        }
    }

    if ( _ebos.size() != 1 )
    {
        OE_NOTICE << LC << "Found a Geometry that uses more than one EBO (non-optimal sharing)\n";
    }
}

void
GeometryValidator::apply(osg::Group& group)
{
    for(unsigned i=0; i<group.getNumChildren(); ++i)
    {
        osg::Geometry* geom = group.getChild(i)->asGeometry();
        if ( geom )
        {
            apply( *geom );
            if ( geom->getVertexArray() == 0L )
            {
                OE_NOTICE << "removing " << geom->getName() << " b/c of null vertex array\n";
                group.removeChild(geom);
                --i;
            }
        }
    }
}
//------------------------------------------------------------------------

AllocateAndMergeBufferObjectsVisitor::AllocateAndMergeBufferObjectsVisitor()
{
    setVisitorType(NODE_VISITOR);
    setTraversalMode(TRAVERSE_ALL_CHILDREN);
    setNodeMaskOverride(~0);
}

void
AllocateAndMergeBufferObjectsVisitor::apply(osg::Drawable& drawable)
{
    osg::Geometry* geom = drawable.asGeometry();
    if ( geom )
    {
        // We disable vbo's and then re-enable them to enable sharing of all the arrays.
        geom->setUseDisplayList( false );
        geom->setUseVertexBufferObjects( false );
        geom->setUseVertexBufferObjects( true );
    }
    traverse(drawable);
}


//------------------------------------------------------------------------

namespace
{
    unsigned getTotalNumRenderLeavesInStateGraph(const osgUtil::StateGraph* sg)
    {
        unsigned count = sg->_leaves.size();
        for(osgUtil::StateGraph::ChildList::const_iterator i = sg->_children.begin(); i != sg->_children.end(); ++i)
            count += getTotalNumRenderLeavesInStateGraph( i->second.get() );
        return count;
    }
}

unsigned
RenderBinUtils::getTotalNumRenderLeaves(osgUtil::RenderBin* bin)
{
    if ( !bin ) return 0u;
    unsigned count = bin->getRenderLeafList().size();

    for(osgUtil::RenderBin::StateGraphList::const_iterator i = bin->getStateGraphList().begin(); i != bin->getStateGraphList().end(); ++i)
    {
        count += getTotalNumRenderLeavesInStateGraph( *i );
    }

    for(osgUtil::RenderBin::RenderBinList::const_iterator i = bin->getRenderBinList().begin(); i != bin->getRenderBinList().end(); ++i)
    {
        count += getTotalNumRenderLeaves( i->second.get() );
    }

    return count;
}
