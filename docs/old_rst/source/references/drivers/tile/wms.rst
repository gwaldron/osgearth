WMS (OGC Web Map Service)
=========================
This plugin reads image data from an OGC `Web Map Service`_ resource.

Example usage::

    <image name="Landsat" driver="wms">
        <url>http://onearth.jpl.nasa.gov/wms.cgi</url>
        <srs>EPSG:4326</srs>
        <tile_size>512</tile_size>
        <layers>global_mosaic</layers>
        <styles>visual</styles>
        <format>jpeg</format>
    </image>  
    
Properties:

    :url:            Location of the WMS resource
    :srs:            Spatial reference in which to return tiles
    :tile_size:      Override the default tile size (default = 256)
    :layers:         WMS layer list to composite and return
    :styles:         WMS styles to render
    :format:         Image format to return

Notes:

    * This plugin will recognize the JPL WMS-C implementation and use it if detected.
    
Also see:

    ``wms_jpl_landsat.earth`` sample in the repo ``tests`` folder
    

.. _Web Map Service:  http://en.wikipedia.org/wiki/Web_Map_Service
