> Project: "Automatic Row Crop Generator"

> Owner: "Marco Ambrosio, Brenno Tuberga" 

> Date: "2022:12" 

---

# Automatic Row Crop Generator User Guide

## Licence 

All the code and models inside this repository are released under the MIT Licence.

```
MIT License

Copyright (c) 2022 Marco Ambrosio, Brenno Tuberga

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following  conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
```

## Description of the project

The code in this repository provides useful tools to automatically generate models of row-based crops for Blender and Gazebo.

## Installation procedure

### Setup Blender Python environment

Open a terminal inside this folder and run the following command:

``` 
    <PATH-TO-BLENDER-EXECUTABLE> --python install_requirements.py 
```

### Setup Gazebo environment

To setup the Gazebo environment, run the following command or pasting it in your `.bashrc` file:

```
    source /usr/share/gazebo/setup.sh 

    export GAZEBO_MODEL_PATH=<path-to-this-repo>/gazebo/models:$GAZEBO_MODEL_PATH

    export GAZEBO_RESOURCE_PATH=<path-to-this-repo>/gazebo:$GAZEBO_RESOURCE_PATH 
```
## User Guide

### List of parameters

The following parameters can be set in the `configuration.yaml` file.

__File names:__

- `plant_name`: name of the plant
- `terrain_name`: name of the terrain

__Field dimensions:__

- `magic_number`: a number, in meters, used to space plants and rows. Typically, it is the space between two rows.
- `row_length`: length of the row in meters
- `number_of_rows`: number of rows
- `rows_in_group`: the rows may be grouped. Between two groups of rows the distance is greater than the distance between two rows in the same group.
- `avg_plant_footprint`: average plant footprint in meters, used for spacing.


__Function parameters:__

- `generate_blender_model`: if True, Blender model is generated.
- `generate_gazebo_models`: if True, Gazebo models are generated inside _`gazebo/models`_ folder. One folder for each plant model is created.
- `generate_gazebo_world`: if True, Gazebo world is generated inside _`gazebo/worlds`_ folder.

### Models requirements
- Row lenght is disposed along Y axis, while field width is disposed along X axis.
- The origin of the object must be at the bottom of the plant.
- Plant models must be made of a single object, named _plant_. The _plant_ object may have children objects.
- Suggested options for OBJ mesh generation:
    - Forward: +Y
    - Up: +Z
    - Scale: 1
    - Path Mode: Strip
    

### Folder structure for plants and terrains:

```
plants
├── artichoke
│   ├── artichoke1
│   │   ├── artichoke1.blend
│   │   ├── artichoke1.mtl
│   │   ├── artichoke1.obj
│   │   └── textures
│   │       ├── petalo1.jpg
│   │       └── texture foglia.jpg
│   └── artichokeN
│       ├── artichokeN.blend
│       ├── artichokeN.mtl
│       ├── artichokeN.obj
│       └── textures
│           └── texture.jpg
│
├── another_plant
│   ├── another_plant1
|
└── terrain
    ├── terrain1
    │   ├── terrain1.blend
    │   ├── terrain1.mtl
    │   ├── terrain1.obj
    │   └── textures
    │       └── texture.jpg
    └── terrainN
        ├── terrainN.blend
        ├── terrainN.mtl
        ├── terrainN.obj
        └── textures
            └── texture.jpg
```


### Run the script

After setting the parameters in the `configuration.yaml` file, run the following command:

```
    <PATH-TO-BLENDER-EXECUTABLE> --python generate_models.py 
```

### Run Gazebo simulation

```
    gazebo worlds/<plant-name>_<terrain_name>.world 
```


## Authors

Marco Ambrosio, Brenno Tuberga

## Acknowledgments

This project has been developed thank to PIC4SeR Center.
