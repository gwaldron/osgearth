#include <osgEarth/LODGenerator>

#ifdef OSGEARTH_HAVE_MESH_OPTIMIZER

#include <osgEarth/NodeUtils>
#include <osgUtil/Optimizer>

#include <meshoptimizer.h>

using namespace osgEarth;

osg::LOD* createLODFromGeometry(osg::Geometry* originalGeometry, const std::vector<LODGenerator::LODOptions>& options)
{
    double radius = originalGeometry->getBound().radius();

    // Turn the geometry into an indexed mesh.
    osgUtil::Optimizer o;
    o.optimize(originalGeometry,
        o.INDEX_MESH |
        o.VERTEX_PRETRANSFORM |
        o.VERTEX_POSTTRANSFORM);

    osg::ref_ptr<osg::LOD> lodNode = new osg::LOD();

    osg::Vec3Array* vertices = dynamic_cast<osg::Vec3Array*>(originalGeometry->getVertexArray());
    osg::DrawElements* drawElements = dynamic_cast<osg::DrawElements*>(originalGeometry->getPrimitiveSet(0));

    if (!vertices || !drawElements) {
        OE_WARN << "Invalid geometry data." << std::endl;
        return nullptr;
    }

    size_t vertexCount = vertices->size();
    size_t indexCount = drawElements->getNumIndices();
    std::vector<unsigned int> indices(indexCount);
    std::vector<float> vertexData(vertexCount * 3);

    for (size_t i = 0; i < indexCount; ++i) {
        indices[i] = drawElements->getElement(i);
    }
    for (size_t i = 0; i < vertexCount; ++i) {
        vertexData[i * 3 + 0] = (*vertices)[i].x();
        vertexData[i * 3 + 1] = (*vertices)[i].y();
        vertexData[i * 3 + 2] = (*vertices)[i].z();
    }


    // Add the highest level of detail first
    float minDistance = 0.0f;
    float maxDistance = options[0].rangeFactor * radius;
    lodNode->addChild(originalGeometry, minDistance, maxDistance);

    float prevMaxDistance = maxDistance;

    for (unsigned int level = 0; level < options.size(); ++level)
    {
        const LODGenerator::LODOptions& opt = options[level];

        float target_error = 1e-2f;
        float target_error_aggressive = 1e-1f;

        size_t target_index_count = size_t(double(indexCount / 3) * opt.threshold) * 3;

        unsigned int mesh_opt_flags = 0;

        std::vector< unsigned int > newIndices(indices.size());
        unsigned int newIndicesCount = meshopt_simplify(&newIndices[0], &indices[0], indexCount, &vertexData[0], vertexCount, sizeof(float) * 3, target_index_count, target_error, mesh_opt_flags);

        if (opt.aggressive && newIndicesCount > target_index_count) {
            newIndicesCount = meshopt_simplifySloppy(&newIndices[0], &indices[0], indexCount, &vertexData[0], vertexCount, sizeof(float) * 3, target_index_count, target_error_aggressive);
        }

        OE_INFO << "Simplified from " << indexCount / 3 << " triangles to target=" << target_index_count / 3 << " actual=" << newIndicesCount / 3 << " triangles" << std::endl;

        newIndices.resize(newIndicesCount);

        osg::ref_ptr<osg::Geometry> simplifiedGeometry = new osg::Geometry(*originalGeometry, osg::CopyOp::SHALLOW_COPY);
        osg::ref_ptr<osg::DrawElementsUInt> newDrawElements = new osg::DrawElementsUInt(GL_TRIANGLES);


        for (size_t i = 0; i < newIndices.size(); ++i) {
            newDrawElements->push_back(newIndices[i]);
        }

        simplifiedGeometry->setVertexArray(vertices);
        simplifiedGeometry->setPrimitiveSet(0, newDrawElements);

        float minDistance = prevMaxDistance;
        float maxDistance = level == options.size() - 1 ? FLT_MAX : options[level + 1].rangeFactor * radius;

        lodNode->addChild(simplifiedGeometry, minDistance, maxDistance);
        prevMaxDistance = maxDistance;
    }

    return lodNode.release();
}


osg::Node* LODGenerator::generateLODs(osg::Node* node, const std::vector<LODOptions>& options)
{
    FindNodesVisitor<osg::Geometry> nv;
    node->accept(nv);

    for (osg::Geometry* geom : nv._results)
    {
        osg::LOD* lod = createLODFromGeometry(geom, options);
        if (lod)
        {
            osg::Group* parent = geom->getParent(0);
            if (parent)
            {
                osg::Geode* geode = parent->asGeode();
                if (geode)
                {
                    lod->setStateSet(geode->getStateSet());
                    if (geode->getNumParents() == 0)
                    {
                        return lod;
                    }
                    else
                    {
                        geode->getParent(0)->replaceChild(geode, lod);
                    }
                }
                else
                {
                    parent->replaceChild(geom, lod);
                }
            }
            else
            {             
                return lod;
            }
        }
    }

    return node;
}

#endif // OSGEARTH_HAVE_MESH_OPTIMIZER