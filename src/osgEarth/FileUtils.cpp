/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2009 Pelican Ventures, Inc.
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

#include <osgEarth/FileUtils>
#include <osgDB/FileNameUtils>
#include <osg/Notify>

using namespace osgEarth;

bool osgEarth::isRelativePath(const std::string& fileName)
{
    std::string real_path = osgDB::convertToLowerCase( osgDB::convertFileNameToNativeStyle( osgDB::getRealPath( fileName ) ) );
    std::string tmp_path = osgDB::convertToLowerCase( osgDB::convertFileNameToNativeStyle ( fileName ) );

    return (real_path != tmp_path);
}

std::string osgEarth::getFullPath(const std::string& relativeTo, const std::string &relativePath)
{
    if (!isRelativePath(relativePath))
    {
        return relativePath;
    }

    std::string fullPath = osgDB::convertFileNameToUnixStyle(osgDB::concatPaths( osgDB::getFilePath( osgDB::getRealPath( relativeTo )), relativePath));
    fullPath = osgDB::getRealPath(fullPath);    
    return fullPath;
}

bool osgEarth::isZipPath(const std::string &path)
{
    return (path.find(".zip") != std::string::npos);
}