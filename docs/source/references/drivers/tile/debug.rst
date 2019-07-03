Debug Display
=============
This plugin renders an overlay that shows the outline of each tile
along with its tile key (x, y, and LOD).

Example usage::

    <image driver="debug">
    </image>
    
Properties:

    None.
    
Notes:

    Data from this driver is not cacheable.
    
    On Unix like operating systems this driver requires the environment variable OSGEARTH_DEFAULT_FONT to be set to a true type font file name (if the font lives in /usr/share/fonts/ttf) or a full path otherwise.

