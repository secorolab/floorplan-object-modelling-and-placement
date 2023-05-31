#!/usr/bin/env python

import sys, os, configparser
import rdflib
from rdflib import RDF
import json
from pyld import jsonld
from pprint import pprint

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as Pol

from helpers.helpers import loader, prefixed, get_transformation_matrix_wrt_frame
from helpers.sdf import (
    get_sdf_geometry, 
    get_sdf_intertia, 
    get_sdf_pose_from_transformation_matrix,
    get_sdf_joint_type,
    get_sdf_axis_of_rotation,
    write_object_model_sdf
)
from helpers.constants import *

DEBUG = True

if __name__ == "__main__":

    # Read folder where composable models are located and other configs
    argv = sys.argv
    input_folder = argv[1]

    config = configparser.ConfigParser()
    config.read('setup.cfg')

    model_config = config["models"]
    DEBUG = config["dev"].getboolean('debug')

    output_folder = model_config.get("gazebo_models_location") if model_config.getboolean('save_to_gazebo') else model_config.get("output_folder")

    worlds_output_path = config["worlds"]["output"]
    
    # Build the graph
    load = loader("")
    g = rdflib.Graph()
    for (dirpath, dirnames, filenames) in os.walk(input_folder):
        for filename in filenames:
            #filenames_array.append(os.path.join(dirpath, filename))
            g.parse(os.path.join(dirpath, filename), format="json-ld")
    
    fp_model_name = ''
    for floorplan in g.subjects(RDF.type, FP.FloorPlan):
        fp_model_name = prefixed(g, floorplan).split('fp:')[1]

    for my_object, _, _ in g.triples((None, RDF.type, FP["Object"])):
        
        joint_list = []
        link_list = []

        object_frame = g.value(my_object, FP["object-frame"])

        # Collect the link information
        for _, _, link in g.triples((my_object, FP["object-links"], None)):

            # Get the objects for the link geometry
            visual_link = g.value(link, FP["visual-link"])
            physics_link = g.value(link, FP["physics-link"])
            simplices_link = g.value(link, FP["link"])

            # Get the sdf geometry description
            sdf_visual_geometry = get_sdf_geometry(g, visual_link)
            sdf_physics_geometry = get_sdf_geometry(g, physics_link)
            
            # Get the sdf inertia description
            sdf_inertia = {}
            for inertia in g.subjects(RBD["of-body"], simplices_link):
                sdf_inertia = get_sdf_intertia(g, inertia)

            # Get the frame for the link
            link_frame = g.value(link, FP["link-frame"])
            
            T = get_transformation_matrix_wrt_frame(g, link_frame, object_frame)
            pose_coordinates = get_sdf_pose_from_transformation_matrix(T)

            if (DEBUG):
                print(sdf_visual_geometry, sdf_physics_geometry, sdf_inertia, pose_coordinates)

            link_list.append({
                "pose": pose_coordinates,
                "inertial": sdf_inertia,
                "collision": sdf_physics_geometry,
                "visual": sdf_visual_geometry,
                "name" : prefixed(g, simplices_link)
            })
                
        # If the object is a kinematic chain, collect the joint information
        if (my_object, RDF.type, FP["ObjectWithKinematicChain"]):
            kin_chain = g.value(my_object, FP["kinematic-chain"])
            for _, _, joint in g.triples((kin_chain, KIN["joints"], None)):
                
                # determine axis of joint
                joint_axis = get_sdf_axis_of_rotation(g, joint)
                
                # determine type of joint
                joint_type = get_sdf_joint_type(g, joint)

                # Get parent and children bodies
                joint_with_tree = g.value(predicate=FP["joint-without-tree"], object=joint)
                parent = g.value(joint_with_tree, FP["parent"])
                children = [prefixed(g, c) for _, _, c in g.triples((joint_with_tree , FP["children"], None))]

                common_axis = g.value(joint, KIN["common-axis"])
                joint_frame = None

                for _, _, vector in g.triples((common_axis, GEOM["lines"], None)):
                    for p in g.predicates(None, vector):

                        # If the vector is not related to the parent then ignore this vector
                        if len([p for p in g.predicates(parent, vector)]) == 0:
                            continue

                        subject = g.value(predicate=p, object=vector)
                        for _, _, _ in g.triples((subject, RDF.type, GEO["Frame"])):
                            joint_frame = subject

                # determine frame of reference and pose wrt object frame
                T = get_transformation_matrix_wrt_frame(g, joint_frame, object_frame)
                pose_coordinates = get_sdf_pose_from_transformation_matrix(T)

                limits = {"upper": None, "lower": None}
                for position, pre, _ in g.triples((None, KSTATE["of-joint"], joint)):
                    for p in g.objects(position, RDF.type):
                        if p == FP["JointLowerLimit"]:
                            limits["lower"] = g.value(position, QUDT["value"]).toPython()
                        elif p == FP["JointUpperLimit"]:
                            limits["upper"] = g.value(position, QUDT["value"]).toPython()

                joint_list.append({
                    "name": prefixed(g, joint),
                    "type": joint_type,
                    "axis": joint_axis,
                    "pose": pose_coordinates,
                    "parent": prefixed(g, parent),
                    "children": children,
                    "limits": limits
                })

        my_object_tree = {
            "name": prefixed(g, my_object),
            "static": "false",
            "links": link_list,
            "joints": joint_list
        }
        
        if DEBUG:
            pprint(my_object_tree)
        write_object_model_sdf(my_object_tree, output_folder)

    # Querie for the pose path from the object instance to the world frame

    # Build the sdf world
    
    # Write the sdf world