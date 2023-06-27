# Object Modelling and Placement Tool

This tool is a companion tool for the FloorPlan DSL, which enables the modelling of objects and their placement in the indoor environments from the FloorPlan DSL. 

![](images/gazebo-screenshot.png)

## Features:
* **Model objects with movement constraits**: the tool enables the specification of objects with revolute, prismatic, or fixed joints. 
* **Place objects in indoor environments**: by using the composable modelling approach the FloorPlan models can become scenes filled with objects with ease.
* **Model object states**: the tool allows for finate state machines for objects with motion constraints to be modelled, as well as selecting the intial state of each object in the scene.
* **Gazebo world generation and plugin**: the tool generates SDF format world files for gazebo, while a companion plugin sets up the scene as determined by the initial state. The plugin is available [here](https://github.com/hbrs-sesame/floorplan-gazebo-initial-state-plugin)

## Getting Startedith Python 3.8.10,


Install the [requirements](requirements.txt), then you may run the example:

```sh
python3 main.py <input folder> 
```
Tested on Python 3.8.10

## Tutorials

Tutorials on how to model objects with movement constraints, and how to place them in floor plan models is available [here](docs/tutorial.md).

# Acknowledgement

This work is part of a project that has received funding from the European Union's Horizon 2020 research and innovation programme SESAME under grant agreement No 101017258.

<p align="center">
    <img src="images/EU.jpg" alt="drawing" height="100"/>
    <img src="images/SESAME.jpg" alt="drawing" height="100"/>
</p>