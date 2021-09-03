/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2020 Pelican Mapping
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
#include "MaterialLoader"
#include <osg/Texture2D>

using namespace osgEarth;
using namespace osgEarth::Util;

#undef LC
#define LC "[MaterialLoader] "

MaterialLoader::MaterialLoader()
{
    setTraversalMode(TRAVERSE_ALL_CHILDREN);
    setNodeMaskOverride(~0);
}

void
MaterialLoader::setOptions(const osgDB::Options* options)
{
    _options = options;
}

void
MaterialLoader::setMangler(
    int unit,
    MaterialLoader::Mangler mangler)
{
    _manglers[unit] = mangler;
}

void
MaterialLoader::setTextureFactory(
    int unit,
    MaterialLoader::TextureFactory factory)
{
    _factories[unit] = factory;
}

void
MaterialLoader::apply(osg::Node& node)
{
    if (node.getStateSet())
        apply(node.getStateSet());
    traverse(node);
}

void
MaterialLoader::apply(osg::StateSet* ss)
{
    OE_HARD_ASSERT(ss != nullptr);

    if (ss->getTextureAttributeList().empty())
        return;

    osg::Texture* t = dynamic_cast<osg::Texture*>(
        ss->getTextureAttribute(0, osg::StateAttribute::TEXTURE));
    if (t == nullptr || t->getImage(0) == nullptr)
        return;

    std::string filename = t->getImage(0)->getFileName();

    for (auto& m : _manglers)
    {
        int unit = m.first;

        // if the unit is already occupies, ignore it
        if (ss->getTextureAttribute(unit, osg::StateAttribute::TEXTURE) != nullptr)
            continue;

        // if it's already loaded re-use it
        osg::ref_ptr<osg::Texture> mat_tex;
        auto cache_iter = _cache.find(filename);
        if (cache_iter != _cache.end())
        {
            mat_tex = cache_iter->second;
        }
        else
        {
            MaterialLoader::Mangler& mangler = m.second;
            URI materialURI(mangler(filename));
            osg::ref_ptr<osg::Image> image = materialURI.getImage(_options);
            if (image.valid())
            {
                auto iter = _factories.find(unit);
                if (iter != _factories.end())
                    mat_tex = iter->second(image.get());
                else
                    mat_tex = new osg::Texture2D(image);

                _cache[filename] = mat_tex;
                OE_INFO << LC << "..loaded material tex '" << materialURI.base() << "' to unit " << unit << std::endl;
            }
        }   

        if (mat_tex.valid())
        {
            ss->setTextureAttribute(unit, mat_tex, 1);
        }
    }
}
