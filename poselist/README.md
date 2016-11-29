Poselist function block
=======================

The ``poselist`` is a query function block to aggregate multiple pose queries into a single one, w.r.t to a **single** common reference and time stamp.

Usage
=====

1. [Load the ``poselist`` function block](#load-the-poselist-function-block)
2. [Execute the ``poselist`` function block](#execute-the-poselist-function-block)

### Load the ``poselist`` function block

As for every function block the ``poselist`` has to be loaded before. It can also be attempted to load it multiple times, since it internally checks if it exists.
Thus, it is safe (and recommended) to to call it every time before performing a query. Please note the ``path`` has to be adopted. As best practice obtain this value
from the ``FBX_MODULES`` environment variable.
 
The below message **loads** a ``poselist`` is a function block:

```javascript
{
  "@worldmodeltype": "RSGFunctionBlock",
  "metamodel":       "rsg-functionBlock-schema.json",
  "name":            "poselist",
  "operation":       "LOAD",
  "input": {
    "metamodel": "rsg-functionBlock-path-schema.json",
    "path":      "/workspace/brics_3d_function_blocks/lib/",
    "comment":   "path is the same as the FBX_MODULES environment variable appended with a lib/ folder"
  }
}
```

### Execute the ``poselist`` function block

The ``poselist`` essentially needs to specify as input:

* ``ids``: A list of IDs of nodes to be queried.
* ``idReferenceNode``: Id of the origin. The resulting data is expressed in the origin of this node. For most queries this will a node with a tag: gis:origin. Same for all poses.
* ``timeStamp``: The given point in time for the poses. Same for all poses.

The meta model for the ``input`` is defined in the file [fbx-poselist-input-schema.json](../models/fbx-poselist-input-schema.json) and can be used as a reference.

The other fields like ``@worldmodeltype`` or ``metamodel`` comply the general function block model and are always the same for a ``poselist`` query.

```javascript
{
  "@worldmodeltype": "RSGFunctionBlock",
  "metamodel":       "rsg-functionBlock-schema.json",
  "name":            "poselist",
  "operation":       "EXECUTE",
  "input": {
    "metamodel": "fbx-poselist-input-schema.json",
	"ids": ["10d49594-3f90-48e0-89ba-95c8c329480e", "6b63a87d-9dc5-4c58-8b71-87b3f675e76a", "767a13be-55b9-4b35-a7d5-8b00bfd70075"]
	"idReferenceNode": "84e7bcd9-a797-464c-a916-61bc2f9279ef",
    "timeStamp": {
      "@stamptype": "TimeStampDate",
      "stamp": "2016-11-11T11:11:11Z",
    } 
  }
}
``` 

The ``output`` contains as list labeled as ``poses`` represented as 4 times 4 homogeneous matrix. Its meta model is defined in the 
file [fbx-poselist-output-schema.json](../models/fbx-poselist-output-schema.json) and can be used as a reference to retrieve the output.

Please note ``"operationSuccess":`` will indicate if a block was successfully executed. If yes it will be set to ``true``. If not, there is either a syntax error or some if the ids
do not exist.

```javascript
{
  "@worldmodeltype": "RSGFunctionBlockResult",
  "metamodel": "rsg-functionBlock-schema.json",
  "operation": "EXECUTE",
  "operationSuccess": true,
  "output": {
    "metamodel": "fbx-poselist-output-schema.json",
    "poses": [
      {
        "matrix": [
	        [1.0000000000000000,0.0000000000000000,0.0000000000000000,1.0000000000000000],
	        [0.0000000000000000,1.0000000000000000,0.0000000000000000,2.0000000000000000],
	        [0.0000000000000000,0.0000000000000000,1.0000000000000000,3.0000000000000000],
	        [0.0000000000000000,0.0000000000000000,0.0000000000000000,1.0000000000000000]
	     ],
        "type": "HomogeneousMatrix44",
        "unit": "m"
      },
      {
        "matrix": [
        [1.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000],
        [0.0000000000000000,1.0000000000000000,0.0000000000000000,0.0000000000000000],
        [0.0000000000000000,0.0000000000000000,1.0000000000000000,0.0000000000000000],
        [0.0000000000000000,0.0000000000000000,0.0000000000000000,1.0000000000000000]],
        "type": "HomogeneousMatrix44",
        "unit": "m"
      },
      {
        "matrix": [
        [1.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000],
        [0.0000000000000000,1.0000000000000000,0.0000000000000000,0.0000000000000000],
        [0.0000000000000000,0.0000000000000000,1.0000000000000000,0.0000000000000000],
        [0.0000000000000000,0.0000000000000000,0.0000000000000000,1.0000000000000000]],
        "type": "HomogeneousMatrix44",
        "unit": "m"
      }
    ]
  }
} 
```
