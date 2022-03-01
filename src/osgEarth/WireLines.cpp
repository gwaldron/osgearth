#include <osgEarth/WireLines>
#include <osgEarth/Array>
#include <osgEarth/Math>
#include <osgEarth/Shaders>
#include <osgEarth/VirtualProgram>

#include <osg/Multisample>
#include <osg/Math>
#include <osg/Quat>

using namespace osgEarth;

WireLinesOperator::WireLinesOperator(const Stroke& stroke)
    : _stroke(stroke)
{
}

// Find the bisecting plane between two "wires". The plane is defined
// by its normal and assumed to contain the center point.

namespace
{
    typedef Array::View<osg::Vec3> GeomView;


    osg::Vec3d origin(const Ray& plane)
    {
        return plane._a;
    }

    osg::Vec3d normal(const Ray& plane)
    {
        return plane._dv;
    }
    
    // Plane with normalized normal
    Ray makeUnitPlane(const osg::Vec3f& norm, const osg::Vec3f& point)
    {
        osg::Vec3d planeNorm(norm);
        planeNorm.normalize();
        return Ray(point, planeNorm);
    }
    
    Ray bisectingPlane(osg::Vec3f& prev, osg::Vec3f& center, osg::Vec3f& next)
    {
        return makeUnitPlane(next - prev, center);
    }

    // Fill in vertices and normals for one circle of the wire. The
    // vertices are constant (origin of the plane); the normals are
    // used in the shader to create the wire geometry.
    void makeCircleGeometry(const Ray& plane, const osg::Vec3& up, GeomView& verts,
                            GeomView& norms, int geomRow)
    {
        // To get a starting point, project up vector into the
        // circle's plane.
        osg::Vec3 planeNormal = normal(plane);
        osg::Vec3 planeOrigin = origin(plane);
        osg::Vec3 start = vecRejection(up, planeNormal);
        if (osg::equivalent(start * start, 0.0f))
        {
            // The up vector is parallel to the plane normal! uh oh.
            // This is poor, but gotta try something
            start = osg::Vec3(1.0f, 0.0f ,0.0f);
        }
        start.normalize();
        for (unsigned i = 0; i < norms.colDim; ++i)
        {
            float angle = (2.0f * (float)osg::PI / WireLinesOperator::numWireVerts) * i;
            osg::Quat qrot(angle, planeNormal);
            norms(geomRow, i) = qrot * start;
            verts(geomRow, i) = planeOrigin;
        }
    }

    // Quad verts are counter-clockwise
    void addQuad(std::vector<unsigned>& ebo, unsigned i0, unsigned i1, unsigned i2, unsigned i3)
    {
        ebo.push_back(i0);
        ebo.push_back(i1);
        ebo.push_back(i2);
        ebo.push_back(i0);
        ebo.push_back(i2);
        ebo.push_back(i3);
    }
}

// XXX Turns out that normals are not up vectors; they're pretty much
// the half-normal that we calculate in this code. Should we just use
// them?

osg::Geometry* WireLinesOperator::operator()(osg::Vec3Array* verts, osg::Vec3Array* normals, Callback* callback, bool twosided) const
{
    const size_t numVerts = verts->size();
    osg::Geometry* geom = new osg::Geometry();
    geom->setName(typeid(*this).name());
    geom->setUseVertexBufferObjects(true);
    osg::ref_ptr<osg::Vec3Array> wireVerts = new osg::Vec3Array(osg::Array::BIND_PER_VERTEX,
                                                                numWireVerts * numVerts);
    osg::ref_ptr<osg::Vec3Array> wireNorms = new osg::Vec3Array(osg::Array::BIND_PER_VERTEX,
                                                                numWireVerts * numVerts);
    GeomView matWireVerts(&(*wireVerts)[0], verts->size(), numWireVerts);
    GeomView matWireNorms(&(*wireNorms)[0], verts->size(), numWireVerts);

    osg::Vec3 up(0.0f, 0.0f, 1.0f);
    // end caps are vertical planes
    Ray startPlane = makeUnitPlane(vecRejection((*verts)[1] - (*verts)[0], up), (*verts)[0]);
    Ray endPlane  = makeUnitPlane(vecRejection((*verts)[numVerts - 1] - (*verts)[numVerts - 2],
                                               up), (*verts)[numVerts - 1]);
        
    makeCircleGeometry(startPlane, up, matWireVerts, matWireNorms, 0);
    for (int v0 = 0, v1 = 1, v2 = 2; v2 < numVerts; ++v0, ++v1, ++v2)
    {
        // Get the normal plane between each segment.
        Ray vertPlane = bisectingPlane((*verts)[v0], (*verts)[v1], (*verts)[v2]);
        // Add new vertices and (*normals) for the cylinder.
        makeCircleGeometry(vertPlane, up, matWireVerts, matWireNorms, v1);
    }
    makeCircleGeometry(endPlane, up, matWireVerts, matWireNorms, numVerts - 1);
    // Add indices for the quads that make up the wire
    geom->setVertexArray(wireVerts.get());
    geom->setNormalArray(wireNorms.get());
    std::vector<unsigned> ebo;
    for (unsigned lineVert0 = 0, lineVert1 = 1; lineVert1 < numVerts; ++lineVert0, ++lineVert1)
    {
        for (unsigned circleVert0 = 0, circleVert1 = 1; circleVert0 < numWireVerts;
             ++circleVert0, ++circleVert1)
        {
            // Could be cute with mod, but no.
            if (circleVert1 == numWireVerts)
            {
                circleVert1 = 0;
            }
            unsigned v0 = lineVert0 * numWireVerts + circleVert0;
            unsigned v1 = lineVert0 * numWireVerts + circleVert1;
            unsigned v2 = lineVert1 * numWireVerts + circleVert1;
            unsigned v3 = lineVert1 * numWireVerts + circleVert0;
            addQuad(ebo, v0, v1, v2, v3);
        }
    }
    // XXX triangle fans for end caps
    // copy the ebo into a primitive set of appropriate size:
    int totalVerts = numWireVerts * numVerts + 2;
    osg::DrawElements* primset =
        totalVerts > 0xFFFF ? (osg::DrawElements*)new osg::DrawElementsUInt  (GL_TRIANGLES) :
        totalVerts > 0xFF   ? (osg::DrawElements*)new osg::DrawElementsUShort(GL_TRIANGLES) :
        (osg::DrawElements*)new osg::DrawElementsUByte (GL_TRIANGLES);
    primset->reserveElements(ebo.size());
    for(int i=0; i < ebo.size(); ++i)
    {
        primset->addElement(ebo[i]);
    }
    geom->addPrimitiveSet(primset);

    // generate colors. XXX Why not BIND_OVERALL?
    {
        osg::Vec4Array* colors = new osg::Vec4Array(osg::Array::BIND_PER_VERTEX, totalVerts);
        colors->assign(colors->size(), _stroke.color());
        geom->setColorArray(colors);
    }

    return geom;
}

#define SHADER_NAME "osgEarth::WireLines"

void WireLinesOperator::installShaders(osg::Node* node) const
{
    if (!node)
        return;
    float width = Distance(*_stroke.width(), *_stroke.widthUnits()).as(Units::METERS);
    float radius = width / 2.0f;
    osg::StateSet* stateset = node->getOrCreateStateSet();
    VirtualProgram* vp = VirtualProgram::getOrCreate(stateset);
    if ( vp->getName().compare( SHADER_NAME ) == 0 )
        return;
    vp->setName(SHADER_NAME);
    // bail if already installed.
    Shaders shaders;
    shaders.load(vp, shaders.WireLines);
    stateset->getOrCreateUniform("oe_WireDrawable_radius", osg::Uniform::FLOAT)->set(radius);
    stateset->setMode(GL_BLEND, 0);
    stateset->setMode(GL_SAMPLE_ALPHA_TO_COVERAGE_ARB, 1);
    // XXX shouldn't be needed
    stateset->setMode(GL_CULL_FACE, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE | osg::StateAttribute::PROTECTED);
}
