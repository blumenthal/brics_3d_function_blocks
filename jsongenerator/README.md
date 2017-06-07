jsongenerator function block
=======================

The ``jsongenerator`` is a query function block to generate a RSG-JSON model file that comlies to the rsg-core schema.

Usage
=====

1. [Load the ``jsongenerator`` function block](#load-the-jsongenerator-function-block)
2. [Execute the ``jsongenerator`` function block](#execute-the-jsongenerator-function-block)

Note, the usage is similar to the dotgenerator. It only differs in the optional ``fileName`` argument that is called ``dotFileName`` in case for the ``dotgenerator``. 

### Load the ``jsongenerator`` function block

As for every function block the ``jsongenerator`` has to be loaded before. It can also be attempted to load it multiple times, since it internally checks if it exists.
Thus, it is safe (and recommended) to call it every time before performing a request. Please note the ``path`` has to be adopted. As best practice obtain this value
from the ``FBX_MODULES`` environment variable.
 
The below message **loads** a ``jsongenerator`` as a function block:

```javascript
{
  "@worldmodeltype": "RSGFunctionBlock",
  "metamodel":       "rsg-functionBlock-schema.json",
  "name":            "jsongenerator",
  "operation":       "LOAD",
  "input": {
    "metamodel": "rsg-functionBlock-path-schema.json",
    "path":      "/workspace/brics_3d_function_blocks/lib/",
    "comment":   "path is the same as the FBX_MODULES environment variable appended with a lib/ folder"
  }
}
```

### Execute the ``jsongenerator`` function block

The ``jsongenerator`` essentially needs to specify as input:

* ``subgraphId``: *Optional* subgraph ID. The dot file will generate the graph by usign this as root node to further traverse the graph. Use an appropriate query to getreive such an it e.g. look for a node with an attribute ``( name = swm)``
* ``path``: *Optional* path where RGS-JSON file will be stored. If not specified, it will store it relative to the file location, the SWM has been started.
* ``fileName``: *Optional* file name for the RGS-JSON files to be stored. If not specified, it will generated based on a time stamp. <path>/<fileName>.json will be the result.

The meta model for the ``input`` is defined in the file [fbx-subgraph-and-file-schema.json](../models/fbx-subgraph-and-file-schema.json) and can be used as a reference.

The other fields like ``@worldmodeltype`` or ``metamodel`` comply the general function block model and are always the same for a ``jsongenerator`` query.

```javascript
{
  "@worldmodeltype": "RSGFunctionBlock",
  "metamodel":       "rsg-functionBlock-schema.json",
  "name":            "jsongenerator",
  "operation":       "EXECUTE",
  "input": {
    "metamodel": "fbx-subgraph-and-file-schema.json",
    "subgraphId": "e379121f-06c6-4e21-ae9d-ae78ec1986a1",
    "path": "/tmp/",
    "fileName": "rsg_snapshot_1"
  }
} 
``` 

The ``output`` contains a field ``file`` to  report back to which file the got grapgh has been written to.
The file [fbx-file-schema.json](../models/fbx-file-schema.json) and can be used as a reference to retrieve the output.

Please note ``"operationSuccess":`` will indicate if a block was successfully executed. If yes it will be set to ``true``. If not, there is either a syntax error or some if the ids
do not exist.

```javascript
{
  "@worldmodeltype": "RSGFunctionBlockResult", 
  "operationSuccess": true, 
  "operation": "EXECUTE", 
  "metamodel": "rsg-functionBlock-schema.json", 
  "output": {
    "file": "/tmp/rsg_snapshot_1.json", 
    "metamodel": "fbx-file-schema.json"
  }
} 

```
