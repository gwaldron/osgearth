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

#include "SimpleModelOptions"
#include <osgEarth/ModelSource>
#include <osgEarth/Registry>
#include <osgEarth/Map>
#include <osgEarth/ShaderGenerator>
#include <osgEarth/FileUtils>
#include <osgEarth/StateSetCache>
#include <osg/CullStack>
#include <osg/LOD>
#include <osg/ProxyNode>
#include <osg/Notify>
#include <osg/MatrixTransform>
#include <osg/io_utils>
#include <osgDB/FileNameUtils>

using namespace osgEarth;
using namespace osgEarth::Drivers;

//--------------------------------------------------------------------------

namespace
{
    class LODScaleOverrideNode : public osg::Group
    {
    public:
        LODScaleOverrideNode() : m_lodScale(1.0f) {}
        virtual ~LODScaleOverrideNode() {}
    public:
        void setLODScale(float scale) { m_lodScale = scale; }
        float getLODScale() const { return m_lodScale; }

        virtual void traverse(osg::NodeVisitor& nv)
        {
            if(nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR)
            {
                osg::CullStack* cullStack = dynamic_cast<osg::CullStack*>(&nv);
                if(cullStack)
                {
                    float oldLODScale = cullStack->getLODScale();
                    cullStack->setLODScale(oldLODScale * m_lodScale);
                    osg::Group::traverse(nv);
                    cullStack->setLODScale(oldLODScale);
                }
                else
                    osg::Group::traverse(nv);
            }
            else
                osg::Group::traverse(nv);
        }

    private:
        float m_lodScale;
    };

    class SetLoadPriorityVisitor : public osg::NodeVisitor
    {
    public:
        SetLoadPriorityVisitor(float scale=1.0f, float offset=0.0f)
            : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
            , m_scale(scale)
            , m_offset(offset)
        {}

        virtual void apply(osg::PagedLOD& node)
        {
            for(unsigned n = 0; n < node.getNumFileNames(); n++)
            {
                float old;
                old = node.getPriorityScale(n);
                node.setPriorityScale(n, old * m_scale);
                old = node.getPriorityOffset(n);
                node.setPriorityOffset(n, old + m_offset);
            }
            traverse(node);
        }

    private:
        float m_scale;
        float m_offset;
    };
}

//--------------------------------------------------------------------------

class SimpleModelSource : public ModelSource
{
public:
    SimpleModelSource( const ModelSourceOptions& options )
        : ModelSource( options ), _options(options) { }

    //override
    void initialize( const osgDB::Options* dbOptions )
    {
        ModelSource::initialize( dbOptions );
    }

    // override
    osg::Node* createNodeImplementation(const Map* map, const osgDB::Options* dbOptions, ProgressCallback* progress )
    {
        osg::ref_ptr<osg::Node> result;
        
        // Only support paging if they've enabled it and provided a min/max range
        bool usePagedLOD = *_options.paged() &&
                          (_options.minRange().isSet() || _options.maxRange().isSet());
        
        if (_options.node() != NULL)
        {
            result = _options.node();
        }
        else
        {
            // Only load the model if it's not paged or we don't have a location set.
            if (!usePagedLOD || !_options.location().isSet())
            {
                // required if the model includes local refs, like PagedLOD or ProxyNode:
                osg::ref_ptr<osgDB::Options> localOptions = 
                    Registry::instance()->cloneOrCreateOptions( dbOptions );

                localOptions->getDatabasePathList().push_back( osgDB::getFilePath(_options.url()->full()) );

                result = _options.url()->getNode( localOptions.get(), progress );                
            }
        }

        // Always create a matrix transform
        osg::MatrixTransform* mt = new osg::MatrixTransform;        

        if (_options.location().isSet() && map != 0L)
        {
            GeoPoint geoPoint(
                map->getProfile()->getSRS(), 
                (*_options.location()).x(), 
                (*_options.location()).y(), 
                (*_options.location()).z(),
                ALTMODE_ABSOLUTE );
            
            osg::Matrixd matrix;
            geoPoint.createLocalToWorld( matrix );

            if (_options.orientation().isSet())
            {
                //Apply the rotation
                osg::Matrix rot_mat;
                rot_mat.makeRotate( 
                    osg::DegreesToRadians((*_options.orientation()).y()), osg::Vec3(1,0,0),
                    osg::DegreesToRadians((*_options.orientation()).x()), osg::Vec3(0,0,1),
                    osg::DegreesToRadians((*_options.orientation()).z()), osg::Vec3(0,1,0) );
                matrix.preMult(rot_mat);
            }            
            mt->setMatrix( matrix );            
        }

        if ( _options.minRange().isSet() || _options.maxRange().isSet() )
        {                
            float minRange = _options.minRange().isSet() ? (*_options.minRange()) : 0.0f;
            float maxRange = _options.maxRange().isSet() ? (*_options.maxRange()) : FLT_MAX;

            osg::LOD* lod = 0;

            if (!usePagedLOD)
            {
                // Just use a regular LOD
                lod = new osg::LOD();                
                lod->addChild(result.release(), minRange, maxRange);                                           
            }
            else
            {
                // Use a PagedLOD
                osg::PagedLOD* plod =new osg::PagedLOD();                
                plod->setFileName(0, _options.url()->full());     

                // If they want the model to be paged but haven't given us a location we have to load
                // up the node up front and figure out what it's center and radius are or it won't page in.
                if (!_options.location().isSet() && result.valid())
                {                    
                    osg::Vec3d center = result->getBound().center();
                    OE_DEBUG << "Radius=" << result->getBound().radius() << " center=" << center.x() << "," << center.y() << "," << center.z() << std::endl;                    
                    plod->setCenter(result->getBound().center());
                    plod->setRadius(result->getBound().radius());                    
                }
                lod = plod;
            }   
            lod->setRange(0, minRange, maxRange);                
            mt->addChild(lod);                
        }
        else
        {
            // Simply add the node to the matrix transform
            if (result.valid())
            {            
                mt->addChild( result.get() );
            }            
        }

        result = mt;

        // generate a shader program to render the model.
        if ( result.valid() )
        {
            if(_options.loadingPriorityScale().isSet() || _options.loadingPriorityOffset().isSet())
            {
                SetLoadPriorityVisitor slpv(_options.loadingPriorityScale().value(), _options.loadingPriorityOffset().value());
                result->accept(slpv);
            }
    
            if(_options.lodScale().isSet())
            {
                LODScaleOverrideNode * node = new LODScaleOverrideNode;
                node->setLODScale(_options.lodScale().value());
                node->addChild(result.release());
                result = node;
            }

            if ( _options.shaderPolicy() == SHADERPOLICY_GENERATE )
            {
                osg::ref_ptr<StateSetCache> cache = new StateSetCache();

                Registry::shaderGenerator().run(
                    result,
                    _options.url()->base(),
                    cache.get() );
            }
            else if ( _options.shaderPolicy() == SHADERPOLICY_DISABLE )
            {
                result->getOrCreateStateSet()->setAttributeAndModes(
                    new osg::Program(),
                    osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE );
            }
        }


        return result.release();
    }

protected:

    const SimpleModelOptions _options;
};


class SimpleModelSourceFactory : public ModelSourceDriver
{
public:
    SimpleModelSourceFactory()
    {
        supportsExtension( "osgearth_model_simple", "osgEarth simple model plugin" );
    }

    virtual const char* className()
    {
        return "osgEarth Simple Model Plugin";
    }

    virtual ReadResult readObject(const std::string& file_name, const Options* options) const
    {
        if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
            return ReadResult::FILE_NOT_HANDLED;

        return ReadResult( new SimpleModelSource( getModelSourceOptions(options) ) );
    }
};

REGISTER_OSGPLUGIN(osgearth_model_simple, SimpleModelSourceFactory) 
