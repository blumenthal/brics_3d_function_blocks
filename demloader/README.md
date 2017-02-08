Demloader function block
========================

The ``demloader`` function block loads a *Digital Elevation Map (DEM)* map and provides capabilities to query elevation data.
The provided data should be in the GeoTIFF file format. Others might work as well, but this is not tested. 

The block allows to load a single DEM file. In order to do so first load the block then send a ``LOAD_MAP`` command as explained below.
After successful loading of a map the following queries can be performed:

* Get the **elevation data** for at a particular position defined by **latitude** and **longitude** (WGS84). 
  A query beyond the raster or in case the raster contains invalid values the result  will return a ``false`` in the field ``operationSucess``.
* Get the **minimum** and **maximum** elevation value for the complete the map.



Usage
=====

1. [Load the ``demloader`` function block](#load-the-demloader-function-block)
2. [Execute commands for ``demloader`` function block](#execute-commands-for-the-demloader-function-block)

### Load the ``demloader`` function block

As for every function block the ``demloader`` has to be loaded before. It can also be attempted to load it multiple times, since it internally checks if it exists.
Thus, it is safe (and recommended) to to call it every time before performing a query. Please note the ``path`` has to be adopted. As best practice obtain this value
from the ``FBX_MODULES`` environment variable.
 
The below message **loads** a ``demloader`` is a function block:

```javascript
{
  "@worldmodeltype": "RSGFunctionBlock",
  "metamodel":       "rsg-functionBlock-schema.json",
  "name":            "demloader",
  "operation":       "LOAD",
  "input": {
    "metamodel": "rsg-functionBlock-path-schema.json",
    "path":      "/workspace/brics_3d_function_blocks/lib/",
    "comment":   "path is the same as the FBX_MODULES environment variable appended with a lib/ folder"
  }
}
```

It can be also unloaded and will close the DEM file.

```javascript
{
  "@worldmodeltype": "RSGFunctionBlock",
  "metamodel":       "rsg-functionBlock-schema.json",
  "name":            "demloader",
  "operation":       "UNLOAD",
  "input": {
    "metamodel": "rsg-functionBlock-path-schema.json",
    "path":      "/workspace/brics_3d_function_blocks/lib/",
    "comment":   "path is the same as the FBX_MODULES environment variable appended with a lib/ folder"
  }
}
```


### Execute commands for the ``demloader`` function block

The ``demloader`` supports 3 commands:

* ``LOAD_MAP``: to load a map
* ``GET_ELEVATION``: to get elevation data for a geodetic point
* ``GET_MIN_MAX_ELEVATION``: to get the min and max elevation for the loaded map or within a sub-area.

Every command has the the same basic structure:

```
{
  "@worldmodeltype": "RSGFunctionBlock",
  "metamodel":       "rsg-functionBlock-schema.json",
  "name":            "demloader",
  "operation":       "EXECUTE",
  "input": {
    "metamodel": "fbx-demloader-input-schema.json",
    "command":  ...
    ...
  }
}
```

``@worldmodeltype, metamodel, name`` and ``operation`` in section are always the same for a command. 
``command`` can be set to ``LOAD_MAP``, ``GET_ELEVATION`` or ``GET_MIN_MAX_ELEVATION``. 

The meta model for the ``input`` is defined in the file [fbx-demloader-input-schema.json](../models/fbx-demloader-input-schema.json) and can be used as a reference.

#### Load a map file

| Field          | Possible value   |      Description   |
|----------------|------------------|--------------------|
| ``"command"``  | ``"LOAD_MAP"``   | Load a DEM file    |
| ``"file"``     | String e.g. ``"examples/maps/dem/davos.tif"`` | File to GeoTiff map. A relative path has to be resolved to where the SWM has been launched. |


```javascript
{
  "@worldmodeltype": "RSGFunctionBlock",
  "metamodel":       "rsg-functionBlock-schema.json",
  "name":            "demloader",
  "operation":       "EXECUTE",
  "input": {
    "metamodel": "fbx-demloader-input-schema.json",
    "command":  "LOAD_MAP",
    "file":     "examples/maps/dem/davos.tif"
  }
}
```

``"operationSuccess":`` will indicate if a map was successfully loaded. Multiple invocations will discard older maps.

#### Get elevation 



A get elevation query has the below format: 

```javascript
{
  "@worldmodeltype": "RSGFunctionBlock",
  "metamodel":       "rsg-functionBlock-schema.json",
  "name":            "demloader",
  "operation":       "EXECUTE",
  "input": {
    "metamodel": "fbx-demloader-input-schema.json",
    "command":   "GET_ELEVATION", 
    "latitude":  9.849468, 
    "longitude": 46.812785
  }
}
```

The ``input`` section allows to define the following fields:

| Field          | Possible value   |      Description   |
|----------------|------------------|--------------------|
| ``"command"``  | ``"GET_ELEVATION"``   | Command to get an elevation value |
| ``"latitude"`` | Number e.g.  ``9.849468`` | Latitude (WGS84)  |
| ``"longitude"``| Number e.g.  ``46.812785``| Longitude (WGS84) |

A typical reply message looks like:

```javascript
{

  "@worldmodeltype": "RSGFunctionBlockResult",
  "metamodel": "rsg-functionBlock-schema.json",
  "operation": "EXECUTE",
  "operationSuccess": true,
  "queryId": "af8447ca-3a3e-e4fc-95f0-5dbdf4eed3a7",
  "output": {
    "elevation": 1563.8599853515625, 
    "metamodel": "fbx-demloader-output-schema.json", 
    "result": "ELEVATION_VALUE_EXISTS"
  }
}
```

The header fields ``@worldmodeltype``, ``metamodel``, ``operation`` and ``queryId``  are generic and identical for every reply.
The ``"operationSuccess":`` will indicate if a valid values exists for the map. It will be false in case of a general error (block not loaded),
in case of an invalid pixel in the map or if a query asks for a value beyond the boundaries of the map.
The ``output`` always carries ``"metamodel": "fbx-demloader-output-schema.json",`` to indicate how its data is modeled.
``elevation`` is the actual elevation value/altitude in meters. The ``result`` gives further feedback for debugging purposes.


#### Get minimum and maximum elevation

The ``input`` section allows to define the following fields:

| Field          | Possible value   |      Description   |
|----------------|------------------|--------------------|
| ``"command"``  | ``"GET_MIN_MAX_ELEVATION"``| Command to get the min/max elevation values |
| ``"areaId"``  | ``8ce59f8e-6072-49c0-a0fc-481ee288e24b``| Id that points to area Connection to describe a closed polygon. An example to create an area in the SWM can be found [here](https://github.com/blumenthal/ubx_robotscenegraph/blob/master/examples/json_api/add_area.py). |


```javascript
{
  "@worldmodeltype": "RSGFunctionBlock",
  "metamodel":       "rsg-functionBlock-schema.json",
  "name":            "demloader",
  "operation":       "EXECUTE",
  "input": {
    "metamodel": "fbx-demloader-input-schema.json",
    "command":   "GET_MIN_MAX_ELEVATION"
  }
}
```

A typical result message looks like:

```javascript
{
  "@worldmodeltype": "RSGFunctionBlockResult",
  "metamodel": "rsg-functionBlock-schema.json",
  "operation": "EXECUTE",
  "queryId": "10b20f0f-db25-0c68-9cc8-bc4a8370aab9",
  "operationSuccess": true, 
  "output": {
    "metamodel": "fbx-demloader-output-schema.json"
    "minElevation": 1558.0703125,
    "maxElevation": 1715.953857421875, 
    "result": "ELEVATION_VALUE_EXISTS", 
  }
} 
```

The output section is the same as for a get [``GET_ELEVATION``](#get-elevation) query, 
except that the ``elevation`` field is replaced by ``minElevation`` and ``maxElevation``.


