#!/usr/bin/env python

import sys
import json
from utils import *
from plan_visualizer import Visualizer
from plan import *

if __name__ == '__main__':

    
    CIFP_parsed_filename = 'CIFP_parsed/CIFP_parsed'
    

    visualizer = Visualizer(CIFP_parsed_filename)

    # plan = {"origin":"KBOS",
    #         "destination":"KJFK",
    #         "SID":None,
    #         "route":["SSOXS", "BUZRD", "SEY", "PARCH", "CCC", "ROBER"],
    #         "STAR":None}

    # plan = {"origin":"KBOS",
    #         "destination":"KJFK",
    #         "SID":{"SID_id":"SSOXS5",
    #                "transitions":[{"transition_id":"RW04R"}]},
    #         "route":["SSOXS", "BUZRD", "SEY"],
    #         "STAR":{"STAR_id":"PARCH3",
    #                 "transitions":[{"transition_id":"SEY"}, {"transition_id":""}, {"transition_id":"RW04B"}]}
    #         }


    # plan = {"origin":"KBOS",
    #         "destination":"KLAX",
    #         "SID":{"SID_id":"HYLND5",
    #                "transitions":[{"transition_id":"RW04R"}]},
    #         "routes":#[["HYLND", "CAM", "GOATR", "WOZEE", "HOCKE", "GRB", "RST", "ONL", "CYS", "EKR", "HVE", "PROMT",  "HAKMN"]],
    # #        ["HYLND", "CAM", "GOATR", "WOZEE", "HOCKE", "GRB", "RST", "ONL", "CYS", "EKR", "HVE", "PROMT",  "HAKMN"],
    #        [["HYLND", "CAM", "GOATR", "WOZEE", "Q935", "HOCKE", "GRB", "RST", "ONL", "CYS", "EKR", "HVE", "PROMT", "Q88", "HAKMN"]],
    #         "STAR":{"STAR_id":"ANJLL4",
    #                 "transitions":[{"transition_id":"HAKMN"}, {"transition_id":"ALL"}]}
    #         }



    plan = {"origin":"KBOS",
            "destination":"KLAX",
            "SID":{"SID_id":"HYLND5",
                   "transitions":[{"transition_id":"RW04R"}, {"transition_id":"RW09"}, {"transition_id":"RW15R"}, {"transition_id":"RW22L"}, {"transition_id":"RW33L"}]},
            "routes": [
                ["HYLND", "CAM", "GOATR", "WOZEE", "Q935", "HOCKE", "GRB", "RST", "ONL", "CYS", "EKR", "HVE", "PROMT", "Q88", "HAKMN"],
                       ["HYLND", "HANAA", "Q816", "HOCKE", "Q935", "MONEE", "DABJU", "MCW", "SNY", "KD57S", "HVE", "GARDD", "Q88", "ZZYZX", "Q73", "HAKMN"],
                       ["HYLND", "HANAA", "Q816", "HOCKE", "Q935", "MONEE", "DABJU", "MCW", "J148", "CYS", "HVE", "GARDD", "Q88", "ZZYZX", "Q73", "HAKMN"]
                       ],
            "STAR":[{"STAR_id":"ANJLL4",
                     "transitions":[{"transition_id":"HAKMN"}, {"transition_id":"ALL"}]},
                     {"STAR_id":"BIGBR3",
                      "transitions":[{"transition_id":"HAKMN"}, {"transition_id":""}, {"transition_id":"RW06B"}]}],

            "planned-route":[["HYLND", "CAM", "GOATR", "WOZEE", "Q935", "MONEE", "DABJU", "MCW", "SNY", "KD57S", "HVE", "GARDD", "Q88", "ZZYZX", "Q73", "HAKMN"]],

            "alternative":"KSAN",
            # "alt-STAR":[{"STAR_id":"BAUBB2",
            #              "transitions":[{"transition_id":""}, {"transition_id":"TILLT"},{"transition_id":"RW12"},{"transition_id":"RW26R"},{"transition_id":"RW30"}]}],

            # "alt-routes":[["PROMT", "TILLT"]]
            "alt-routes":[["SASSI","SLI", "DANAH", "KELPS", "OCN", "HURSI", "CARIF", "MZB", "KLOMN", "AJADE", "CATDG", "CRSNR", "SAYAE", "CIJHI", "REEBO"]],

            # "alt-routes-2":[["GARDD","PKE","LNDON","WBERG","SHADI","MOMAR","DSURT","TRIXI","HSKER","BARET","HNAHH","LUCKI","LYNDI","VYDDA"]]

            }

    route = ["SNY","KD57S", "HVE", "GARDD", "NOOTN", "LAKRR"]

    plan = Plan(plan,CIFP_parsed_filename)

    weather_cell = [[42.933791666666664, -78.73878888888889],[42.933791666666664+0.01, -78.73878888888889],[42.933791666666664+0.01, -78.73878888888889+0.01],[42.933791666666664, -78.73878888888889+0.01]]


    cell_ref = ((1.903225806451613, 4.182900432900434),
            (1.1169354838709675, 3.425324675324676),
            (1.758064516129032, 2.83008658008658),
            (2.060483870967742, 2.2619047619047623),
            (2.07258064516129, 1.8425324675324677),
            (2.375, 1.8154761904761907),
            (2.616935483870968, 2.234848484848485),
            (2.725806451612903, 2.816558441558442),
            (2.435483870967742, 3.195346320346321),
            (2.411290322580645, 3.6553030303030303),
            (2.798387096774193, 4.088203463203464),
            (2.774193548387097, 4.6022727272727275),
            (2.169354838709677, 4.710497835497836))

    cell_center = [-111.60135592592592+0.8, 37.780637962962956+0.7]

    weather_cell = []
    factor = 0.3
    for ref in cell_ref:
        weather_cell.append([cell_center[0]+ref[0]*factor, cell_center[1]+ref[1]*factor])

    temp_route = [[-108.34710964285715,39.3607325],
                  [-109+0.5, 39-0.15],
                  [-110.69974166666667, 38.41680833333333]]

    route = ["SNY","KD57S", [-108.34710964285715,39.3607325], [-109+0.5, 39-0.15], [-110.69974166666667, 38.41680833333333]]
    
    # weather_cell = [[-78.73878888888889,42.933791666666664],
    #                 [-78.73878888888889+0.1,42.933791666666664],
    #                 [-78.73878888888889+0.1,42.933791666666664+0.1],
    #                 [-78.73878888888889,42.933791666666664+0.1]]
    visualizer.simulate(plan, route, weather_cell=weather_cell,zoom=False,zoom_size=[-500000,100000,-150000,150000],save_img=False)

    # visualizer.visualize_plan(plan, lat_0=42.214660, lon_0=-95.002300, \
    #                           width=5E6, height=2E6,show=True)
