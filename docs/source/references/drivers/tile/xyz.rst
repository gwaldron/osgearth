XYZ
===
The XYZ plugin is useful for reading web map tile repositories with a
standard X/Y/LOD setup but that don't explicitly report any metadata.
Many of the popular web mapping services (like MapQuest_) fall into 
this category. You need to provide osgEarth with a ``profile`` when
using this driver.

Example usage::

    <image name="mapquest_open_aerial" driver="xyz">
        <url>http://oatile[1234].mqcdn.com/tiles/1.0.0/sat/{z}/{x}/{y}.jpg</url>
        <profile>spherical-mercator</profile>
    </image>
    
Properties:

    :url:            Location of the tile repository
    :profile:        Spatial profile of the repository
    :invert_y:       Set to true to invert the Y axis for tile indexing
    :format:         If the format is not part of the URL itself, you can specify it here.
    
Also see:

    ``mapquest_open_aerial.earth`` and ``openstreetmap.earth`` samples
    in the repo ``tests`` folder.
