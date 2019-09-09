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
#include <osgEarth/Text>
#include <osgEarth/VirtualProgram>
#include <osgEarth/Lighting>
#include <osgEarth/Shaders>
#include <osgEarth/Registry>
#include <osg/Version>
#include <osgText/Font>
#include <sstream>
#include <iomanip>

#include <osgDB/ObjectWrapper>
#include <osgDB/InputStream>
#include <osgDB/OutputStream>

using namespace osgEarth;

#define LC "[Text] "

//....................................................................

REGISTER_OBJECT_WRAPPER( osgEarth_Text,
                         new osgEarth::Text,
                         osgEarth::Text,
                         "osg::Object osg::Node osg::Drawable osgText::TextBase osgText::Text osgEarth::Text" )
{
    //nop
}

//....................................................................

Text::Text() : 
osgText::Text()
{
#if OSG_VERSION_GREATER_OR_EQUAL(3,5,8)
    if (osg::DisplaySettings::instance()->getTextShaderTechnique().empty())
    {
        setShaderTechnique(osgText::ALL_FEATURES);
    }
#endif
}

Text::Text(const std::string& str) :
osgText::Text()
{
#if OSG_VERSION_GREATER_OR_EQUAL(3,5,8)
    if (osg::DisplaySettings::instance()->getTextShaderTechnique().empty())
    {
        setShaderTechnique(osgText::ALL_FEATURES);
    }
#endif

    setText(str);
}

Text::Text(const Text& rhs, const osg::CopyOp& copy) :
osgText::Text(rhs, copy)
{
    //nop
}

Text::~Text()
{
    //nop
}

osg::StateSet*
Text::createStateSet()
{
#if OSG_VERSION_GREATER_OR_EQUAL(3,5,8)
    // NOTE: Most of this is copied from the parent class, except for the 
    // added osgEarth defines and the VirtualProgram in place of the Program.
    osgText::Font* activeFont = getActiveFont();
    if (!activeFont) return 0;

    osgText::Font::StateSets& statesets = activeFont->getCachedStateSets();

    std::stringstream ss;
    ss<<std::fixed<<std::setprecision(3);

    osg::StateSet::DefineList defineList;
    if (_backdropType!=NONE)
    {
        ss.str("");
        ss << "vec4("<<_backdropColor.r()<<", "<<_backdropColor.g()<<", "<<_backdropColor.b()<<", "<<_backdropColor.a()<<")";

        defineList["BACKDROP_COLOR"] = osg::StateSet::DefinePair(ss.str(), osg::StateAttribute::ON);


        if (_backdropType==OUTLINE)
        {
            ss.str("");
            ss <<_backdropHorizontalOffset;
            defineList["OUTLINE"] = osg::StateSet::DefinePair(ss.str(), osg::StateAttribute::ON);
        }
        else
        {
            osg::Vec2 offset(_backdropHorizontalOffset, _backdropVerticalOffset);
            switch(_backdropType)
            {
                case(DROP_SHADOW_BOTTOM_RIGHT) :    offset.set(_backdropHorizontalOffset, -_backdropVerticalOffset); break;
                case(DROP_SHADOW_CENTER_RIGHT) :    offset.set(_backdropHorizontalOffset, 0.0f); break;
                case(DROP_SHADOW_TOP_RIGHT) :       offset.set(_backdropHorizontalOffset, _backdropVerticalOffset); break;
                case(DROP_SHADOW_BOTTOM_CENTER) :   offset.set(0.0f, -_backdropVerticalOffset); break;
                case(DROP_SHADOW_TOP_CENTER) :      offset.set(0.0f, _backdropVerticalOffset); break;
                case(DROP_SHADOW_BOTTOM_LEFT) :     offset.set(-_backdropHorizontalOffset, -_backdropVerticalOffset); break;
                case(DROP_SHADOW_CENTER_LEFT) :     offset.set(-_backdropHorizontalOffset, 0.0f); break;
                case(DROP_SHADOW_TOP_LEFT) :        offset.set(-_backdropHorizontalOffset, _backdropVerticalOffset); break;
                default : break;
            }

            ss.str("");
            ss << "vec2("<<offset.x()<<", "<<offset.y()<<")";

            defineList["SHADOW"] = osg::StateSet::DefinePair(ss.str(), osg::StateAttribute::ON);
        }
    }

    {
        ss<<std::fixed<<std::setprecision(1);

        ss.str("");
        ss << float(_fontSize.second);

        defineList["GLYPH_DIMENSION"] = osg::StateSet::DefinePair(ss.str(), osg::StateAttribute::ON);

        ss.str("");
        ss << float(activeFont->getTextureWidthHint());
        defineList["TEXTURE_DIMENSION"] = osg::StateSet::DefinePair(ss.str(), osg::StateAttribute::ON);
    }

    if (_shaderTechnique>osgText::GREYSCALE)
    {
        defineList["SIGNED_DISTANCE_FIELD"] = osg::StateSet::DefinePair("1", osg::StateAttribute::ON);
    }

#if 0
    OSG_NOTICE<<"Text::createStateSet() defines:"<<defineList.size()<<std::endl;
    for(osg::StateSet::DefineList::iterator itr = defineList.begin();
        itr != defineList.end();
        ++itr)
    {
        OSG_NOTICE<<"   define["<<itr->first<<"] = "<<itr->second.first<<std::endl;
    }
#endif

    // osgEarth:: add defines
#if defined(OSG_GL3_AVAILABLE) && !defined(OSG_GL2_AVAILABLE) && !defined(OSG_GL1_AVAILABLE)
    defineList["OSGTEXT_GLYPH_ALPHA_FORMAT_IS_RED"] = osg::StateSet::DefinePair("1", osg::StateAttribute::ON);
#endif
    defineList[OE_LIGHTING_DEFINE] = osg::StateSet::DefinePair("", osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED);

    // The remaining of this method is exclusive so we don't corrupt the
    // stateset cache when creating text objects from multiple threads. -gw
    static Threading::Mutex mutex;
    Threading::ScopedMutexLock lock(mutex);

    if (!statesets.empty())
    {
        for(osgText::Font::StateSets::iterator itr = statesets.begin();
            itr != statesets.end();
            ++itr)
        {
            if ((*itr)->getDefineList()==defineList)
            {
                // OSG_NOTICE<<"Text::createStateSet() : Matched DefineList, return StateSet "<<itr->get()<<std::endl;
                return itr->get();
            }
            else
            {
            }
        }
    }

    osg::ref_ptr<osg::StateSet> stateset = new osg::StateSet;

    stateset->setDefineList(defineList);

    statesets.push_back(stateset.get());

    stateset->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    stateset->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    stateset->setMode(GL_BLEND, osg::StateAttribute::ON);


#if defined(OSG_GL_FIXED_FUNCTION_AVAILABLE)
    osg::DisplaySettings::ShaderHint shaderHint = osg::DisplaySettings::instance()->getShaderHint();
    if (_shaderTechnique==osgText::NO_TEXT_SHADER && shaderHint==osg::DisplaySettings::SHADER_NONE)
    {
        stateset->setTextureMode(0, GL_TEXTURE_2D, osg::StateAttribute::ON);
        return stateset.release();
    }
#endif

    // set up the StateSet to use shaders
    stateset->addUniform(new osg::Uniform("glyphTexture", 0));
    
    // osgEarth: add shaders
    VirtualProgram* vp = VirtualProgram::getOrCreate(stateset.get());
    vp->setName("osgEarth::Text");
    osgEarth::Shaders coreShaders;
    coreShaders.load(vp, coreShaders.TextVertex);
    coreShaders.load(vp, coreShaders.TextFragment);

    return stateset.release();
#else
    return 0L;
#endif
}

void
Text::setFont(osg::ref_ptr<osgText::Font> font)
{
    if (_font.get() == font.get())
        return;

#if OSG_VERSION_GREATER_OR_EQUAL(3,5,8)
    osgText::Text::setFont(font);
#else
    static Threading::Mutex mutex;
    Threading::ScopedMutexLock lock(mutex);

    osg::StateSet* previousFontStateSet = _font.valid() ? _font->getStateSet() : osgText::Font::getDefaultFont()->getStateSet();
    osg::StateSet* newFontStateSet = font.valid() ? font->getStateSet() : osgText::Font::getDefaultFont()->getStateSet();
    //if (getStateSet() == previousFontStateSet)
    {
        if (newFontStateSet && VirtualProgram::get(newFontStateSet) == 0L)
        {
            VirtualProgram* vp = VirtualProgram::getOrCreate(newFontStateSet);
            vp->setName("osgEarth::Text");
            osgEarth::Shaders coreShaders;
            coreShaders.load(vp, coreShaders.TextLegacy);
#if defined(OSG_GL3_AVAILABLE) && !defined(OSG_GL2_AVAILABLE) && !defined(OSG_GL1_AVAILABLE)
            newFontStateSet->setDefine("OSGTEXT_GLYPH_ALPHA_FORMAT_IS_RED");
#endif
            Lighting::set(newFontStateSet, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED);

            OE_INFO << LC << "Installed VPs on a font" << std::endl;
        }

        setStateSet( newFontStateSet );
    }

    osgText::TextBase::setFont(font);
#endif
}
