# WMS (Web Map Service)

WMS ([Web Map Service](https://en.wikipedia.org/wiki/Web_Map_Service)) is an Open Geospatial Consortium (OGC) standard protocol for serving raster images over the web. WMS is the most popular format you will encounter when accessing governmental or other public web data repositories. The images are usually backed by a GIS database and generated on demand, though the server may also be configured to serve pre-tiled data.

There are many options for WMS access, and many versions of WMS, and osgEarth only supports a small subset as described below.

## WMSImage

CLASS: WMSImageLayer (inherits from [ImageLayer](image.md))

| Property       | Description                                                  | Type    | Default |
| ---------------- | ------------------------------------------------------------ | ------- | ------- |
| url              | Location of the WMS service                                  | URI     |         |
| format           | Image format to generate, usually at a mime-type. Commonly supported formats include `jpg` and `png` (some WMS servers expect the `mime-type` format like `image/jpeg` or `image/png` instead) | string  |         |
| layers           | WMS `LAYERS` option (see specification) and requests the data be generated from a given layer or comma-separated collection of layers | string  |         |
| srs              | WMS `SRS` option (see specification) as exposed in the capabilities response, and refers to the spatial reference system of the image to generate | string  |         |
| transparent      | Hint as to whether the server should create imagery with a transparency channel (alpha). This requires the use of a `format` that supports alpha (like `png`) | boolean | false   |
| crs              | WMS `CRS` option (see specification) as used by later WMS versions as a refinement of the old `SRS` option. | string  |         |
| style            | WMS `STYLES` option (see specification) to request from the server | string  |         |
| capabilities_url | Optionally specificy the URL osgEarth should use to request server capabilities. Usually this is devined automatically and you do not need to set it. | URI     |         |
| wms_version      | Expressly request a WMS version string from the server; usually you do not need this, but sometimes a WMS server requires it. | string  | 1.0.0   |

### Example

```xml
<WMSImage name="Weather RADAR">
    <url>http://mesonet.agron.iastate.edu/cgi-bin/wms/nexrad/n0r.cgi</url>
    <format>png</format>
    <layers>nexrad-n0r</layers>
    <srs>EPSG:4326</srs>
    <transparent>true</transparent>
</WMSImage>
```
