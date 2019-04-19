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

#ifndef OSGEARTH_PLUGIN_LOADER_H
#define OSGEARTH_PLUGIN_LOADER_H 1

#include <osgEarth/Common>
#include <osgDB/Registry>
#include <osgDB/FileNameUtils>

namespace osgEarth
{
    /**
     * Template that helps with the registration of plugins.
     */
    template<class T>
    class RegisterPluginLoader
    {
        public:
            RegisterPluginLoader(const std::string& name)
            {
                if (osgDB::Registry::instance())
                {
                    _rw = new T(name);
                    osgDB::Registry::instance()->addReaderWriter(_rw.get());
                }
            }

            ~RegisterPluginLoader()
            {
                if (osgDB::Registry::instance())
                {
                    osgDB::Registry::instance()->removeReaderWriter(_rw.get());
                }
            }

            T* get() { return _rw.get(); }

        protected:
            osg::ref_ptr<T> _rw;
    };

    /**
     * Template that create a plugin loader.
     */
    template<typename T, typename U>
    class PluginLoader : public osgDB::ReaderWriter
    {
    public: // Plugin stuff
        PluginLoader(const std::string& name) {
            supportsExtension( name, name );
        }

        virtual ~PluginLoader() { }

        ReadResult readObject(const std::string& filename, const osgDB::Options* dbOptions) const
        {
          if ( !acceptsExtension(osgDB::getLowerCaseFileExtension(filename)) )
                return ReadResult::FILE_NOT_HANDLED;

          return ReadResult( new T(U::getConfigOptions(dbOptions)) );
        }
    };

} // namespace osgEarth

#endif // OSGEARTH_PLUGIN_LOADER_H
