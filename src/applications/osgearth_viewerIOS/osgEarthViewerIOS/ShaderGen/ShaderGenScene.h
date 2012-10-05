#pragma once

#include <osg/ShapeDrawable>
#include <osg/Texture2D>
#include <osg/MatrixTransform>
#include <osgDB/ReadFile>
#include <osgDB/FileUtils>
#include <osgUtil/TangentSpaceGenerator>

#include "GLES2ShaderGenVisitor.h"

class GenerateTangentsVisitor : public osg::NodeVisitor
{
public:
    std::vector<osg::Geode*> _geodesList;
    GenerateTangentsVisitor()
    : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
    {
    }
    virtual void apply(osg::Geode& geode)
    {
        _geodesList.push_back(&geode);
        //loop the geoms
        for(unsigned int i=0; i<geode.getNumDrawables(); i++)
        {
            //cast drawable to geometry
            osg::Geometry* geom = geode.getDrawable(i)->asGeometry();
            
            if(geom)
            {
                //check if the geom already has the vectors
                if( (geom->getVertexAttribArray(6) == 0) && (geom->getVertexAttribArray(7) == 0) )
                {
                    
                    osgUtil::TangentSpaceGenerator* tangGen = new osgUtil::TangentSpaceGenerator();
                    tangGen->generate(geom, 0);
                    
                    if(tangGen)
                    {
                        osg::Vec4Array* tangentArray = tangGen->getTangentArray(); 
                        osg::Vec4Array* biNormalArray = tangGen->getBinormalArray();
                        
                        int size = tangentArray->size();
                        int sizeb = biNormalArray->size();
                        
                        if( (size>0) && (sizeb>0))
                        {
                            geom->setVertexAttribArray(6, tangentArray);
                            geom->setVertexAttribBinding(6, osg::Geometry::BIND_PER_VERTEX);  
                            
                            //geom->setVertexAttribArray(7, biNormalArray);
                            //geom->setVertexAttribBinding(7, osg::Geometry::BIND_PER_VERTEX);  
                        }
                    }
                }
            }
        }
        traverse(geode);
    }
protected:
    
};

class ShaderGenScene
{
public:
    ShaderGenScene(){
        
    }
    virtual ~ShaderGenScene(){
    }
    
    static osg::MatrixTransform* CreateScene(){
        osg::MatrixTransform* root = new osg::MatrixTransform();
        //move whole scene in front of camera
        root->setMatrix(osg::Matrix::translate(osg::Vec3(0.0f,0.0f,-100.0f)));

        //create each type with an offset
        float size = 20.0f;
        float offset = size+1.0f;
        
        //top row
        root->addChild(ShaderGenScene::CreateColoredShape(osg::Vec3(-offset,offset,0.0f), size));
        
        root->addChild(ShaderGenScene::CreateColoredLitShape(osg::Vec3(0.0f,offset,0.0f), size));
        
        root->addChild(ShaderGenScene::CreateTexturedShape(osg::Vec3(offset,offset,0.0f), size));
        
        //bottom row
        root->addChild(ShaderGenScene::CreateTexturedLitShape(osg::Vec3(-offset,-offset,0.0f), size));
        
        root->addChild(ShaderGenScene::CreateColoredNormalMappedShape(osg::Vec3(0.0f,-offset,0.0f), size));
        
        root->addChild(ShaderGenScene::CreateTexturedNormalMappedShape(osg::Vec3(offset,-offset,0.0f), size));
        
        //apply shader gen to entire root
        //osgUtil::GLES2ShaderGenVisitor shaderGen;
        //root->accept(shaderGen);
        
        return root;
    }
    
    static osg::MatrixTransform* CreateColoredShape(osg::Vec3 offset, float size){
        osg::Sphere* sphere = new osg::Sphere();
        sphere->setRadius(size*0.5f);
        osg::ShapeDrawable* shape = new osg::ShapeDrawable(sphere);
        osg::Geode* geode = new osg::Geode();
        geode->addDrawable(shape);
        
        osg::StateSet* state = geode->getOrCreateStateSet();
        state->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
        
        osg::MatrixTransform* transform = new osg::MatrixTransform();
        transform->setMatrix(osg::Matrix::translate(offset));
        transform->addChild(geode);
        return transform;
    }
    
    static osg::MatrixTransform* CreateColoredLitShape(osg::Vec3 offset, float size){
        osg::Sphere* sphere = new osg::Sphere();
        sphere->setRadius(size*0.5f);
        osg::ShapeDrawable* shape = new osg::ShapeDrawable(sphere);
        osg::Geode* geode = new osg::Geode();
        geode->addDrawable(shape);
        
        osg::StateSet* state = geode->getOrCreateStateSet();
        state->setMode(GL_LIGHTING, osg::StateAttribute::ON);
        
        osg::MatrixTransform* transform = new osg::MatrixTransform();
        transform->setMatrix(osg::Matrix::translate(offset));
        transform->addChild(geode);
        return transform;
    }
    
    static osg::MatrixTransform* CreateTexturedShape(osg::Vec3 offset, float size){
        osg::Sphere* sphere = new osg::Sphere();
        sphere->setRadius(size*0.5f);
        osg::ShapeDrawable* shape = new osg::ShapeDrawable(sphere);
        osg::Geode* geode = new osg::Geode();
        geode->addDrawable(shape);
        
        osg::StateSet* state = geode->getOrCreateStateSet();
        state->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
        state->setTextureAttributeAndModes(0, new osg::Texture2D(osgDB::readImageFile(osgDB::findDataFile("Images/diffuse.png"))), osg::StateAttribute::ON);
        
        osg::MatrixTransform* transform = new osg::MatrixTransform();
        transform->setMatrix(osg::Matrix::translate(offset));
        transform->addChild(geode);
        return transform;
    } 
    
    static osg::MatrixTransform* CreateTexturedLitShape(osg::Vec3 offset, float size){
        osg::Sphere* sphere = new osg::Sphere();
        sphere->setRadius(size*0.5f);
        osg::ShapeDrawable* shape = new osg::ShapeDrawable(sphere);
        osg::Geode* geode = new osg::Geode();
        geode->addDrawable(shape);
        
        osg::StateSet* state = geode->getOrCreateStateSet();
        state->setMode(GL_LIGHTING, osg::StateAttribute::ON);
        state->setTextureAttributeAndModes(0, new osg::Texture2D(osgDB::readImageFile(osgDB::findDataFile("Images/diffuse.png"))), osg::StateAttribute::ON);
        
        osg::MatrixTransform* transform = new osg::MatrixTransform();
        transform->setMatrix(osg::Matrix::translate(offset));
        transform->addChild(geode);
        return transform;
    } 

    static osg::MatrixTransform* CreateColoredNormalMappedShape(osg::Vec3 offset, float size){
        osg::Node* model = osgDB::readNodeFile(osgDB::findDataFile("Models/sphere.osg"));
        if(!model){
            OSG_FATAL << "ERROR: Failed to load model 'Models/sphere.osg' no normal mapping example avaliable." << std::endl;
            return false;
        }
        
        //generate tangent vectors
        GenerateTangentsVisitor genTangents;
        model->accept(genTangents);
        
        osg::StateSet* state = model->getOrCreateStateSet();
        state->setMode(GL_LIGHTING, osg::StateAttribute::ON);
        //state->setTextureAttributeAndModes(0, new osg::Texture2D(osgDB::readImageFile(osgDB::findDataFile("Images/diffuse.png"))), osg::StateAttribute::ON);
        state->setTextureAttributeAndModes(1, new osg::Texture2D(osgDB::readImageFile(osgDB::findDataFile("Images/normal.png"))), osg::StateAttribute::ON);
        
        //make sure the model fits the scene scale
        float radius = model->computeBound().radius();
        OSG_FATAL << "RADIUS: " << radius << std::endl;
        float scale = (size)/radius;
        
        
        osg::MatrixTransform* transform = new osg::MatrixTransform();
        transform->setMatrix(osg::Matrix::rotate(osg::DegreesToRadians(-90.0f), osg::Vec3(1,0,0)) * osg::Matrix::scale(osg::Vec3(scale,scale,scale)) * osg::Matrix::translate(offset));
        transform->addChild(model);
        return transform;
    } 
    
    static osg::MatrixTransform* CreateTexturedNormalMappedShape(osg::Vec3 offset, float size){
        osg::Node* model = osgDB::readNodeFile(osgDB::findDataFile("Models/sphere.osg"));
        if(!model){
            OSG_FATAL << "ERROR: Failed to load model 'Models/sphere.osg' no normal mapping example avaliable." << std::endl;
            return false;
        }
        
        //generate tangent vectors
        GenerateTangentsVisitor genTangents;
        model->accept(genTangents);

        osg::StateSet* state = model->getOrCreateStateSet();
        state->setMode(GL_LIGHTING, osg::StateAttribute::ON);
        state->setTextureAttributeAndModes(0, new osg::Texture2D(osgDB::readImageFile(osgDB::findDataFile("Images/diffuse.png"))), osg::StateAttribute::ON);
        state->setTextureAttributeAndModes(1, new osg::Texture2D(osgDB::readImageFile(osgDB::findDataFile("Images/normal.png"))), osg::StateAttribute::ON);

        //make sure the model fits the scene scale
        float radius = model->computeBound().radius();
        OSG_FATAL << "RADIUS: " << radius << std::endl;
        float scale = (size)/radius;

        
        osg::MatrixTransform* transform = new osg::MatrixTransform();
        transform->setMatrix(osg::Matrix::rotate(osg::DegreesToRadians(-90.0f), osg::Vec3(1,0,0)) * osg::Matrix::scale(osg::Vec3(scale,scale,scale)) * osg::Matrix::translate(offset));
        transform->addChild(model);
        return transform;
    } 
    
protected:

};