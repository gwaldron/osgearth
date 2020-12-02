Cesium Ion
==========
The Cesium Ion plugin reads imagery tiles from the `Cesium Ion <https://cesium.com>`_ service.
By providing your own access_token you'll gain access to your layers.

Cesium Ion requires your CURL library to be compiled with SSL support to support https links.

Example usage::

    <image name="cesiumion bluemarble" driver="cesiumion">
        <asset_id>3845</asset_id>    
        <token>eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJqdGkiOiI0NDViM2NkNi0xYTE2LTRlZTUtODBlNy05M2Q4ODg4M2NmMTQiLCJpZCI6MjU5LCJpYXQiOjE1MTgxOTc4MDh9.sld5jPORDf_lWavMEsugh6vHPnjR6j3qd1aBkQTswNM</token>
    </image>

Properties:

    :server:         The Cesium Ion server to access.  Default is https://api.cesium.com/
    :asset_id:       The id of the asset to access.  Only imagery layers are currently supported.
    :token:          Your access token to the Cesium Ion service
    :format:         The format of the layer.  Default is png
    
Also see:

    ``cesium_ion.earth`` in the repo ``tests`` folder.

