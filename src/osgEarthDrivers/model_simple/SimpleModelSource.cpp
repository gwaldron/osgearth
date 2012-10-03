/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2012 Pelican Mapping
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
#include <osg/LOD>
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
    osg::Node* createNode(const Map* map, const osgDB::Options* dbOptions, ProgressCallback* progress )
    {
        osg::ref_ptr<osg::Node> result;

        if (_options.node() != NULL)
        {
            result = _options.node();
        }
        else
        {
            // required if the model includes local refs, like PagedLOD or ProxyNode:
            osg::ref_ptr<osgDB::Options> localOptions = 
                Registry::instance()->cloneOrCreateOptions( dbOptions );

            localOptions->getDatabasePathList().push_back( osgDB::getFilePath(_options.url()->full()) );

            result = _options.url()->getNode( localOptions.get(), progress );
        }

        if (_options.location().isSet() && map != 0L)
        {
            GeoPoint geoPoint(
                map->getProfile()->getSRS(), 
                (*_options.location()).x(), 
                (*_options.location()).y(), 
                (*_options.location()).z(),
                ALTMODE_ABSOLUTE );

            OE_NOTICE << "Read location " << geoPoint.vec3d() << std::endl;
            
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

            osg::MatrixTransform* mt = new osg::MatrixTransform;
            mt->setMatrix( matrix );
            mt->addChild( result.get() );
            result = mt;

            if ( _options.minRange().isSet() || _options.maxRange().isSet() )
            {
                osg::LOD* lod = new osg::LOD();
                lod->addChild(
                    result.release(),
                    _options.minRange().isSet() ? (*_options.minRange()) : 0.0f,
                    _options.maxRange().isSet() ? (*_options.maxRange()) : FLT_MAX );
                result = lod;
            }
        }

        if(_options.lodScale().isSet())
        {
            LODScaleOverrideNode * node = new LODScaleOverrideNode;
            node->setLODScale(_options.lodScale().value());
            node->addChild(result.release());
            result = node;
        }

        // generate a shader program to render the model.
        if ( result.valid() )
        {
            if ( _options.shaderPolicy() == SHADERPOLICY_GENERATE )
            {
                ShaderGenerator gen;
                result->accept( gen );
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
