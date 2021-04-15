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


            "alternative":"KLGB",
            "alt-STAR":[{"STAR_id":"BAUBB2",
                         "transitions":[{"transition_id":""}, {"transition_id":"TILLT"},{"transition_id":"RW12"},{"transition_id":"RW26R"},{"transition_id":"RW30"}]}],

            "alt-routes":[["PROMT", "TILLT"]]

            }

    route = ["WOZEE", "HOCKE", "GRB", "RST"]

    plan = Plan(plan,CIFP_parsed_filename)

    weather_cell = [[42.933791666666664, -78.73878888888889],[42.933791666666664+0.01, -78.73878888888889],[42.933791666666664+0.01, -78.73878888888889+0.01],[42.933791666666664, -78.73878888888889+0.01]]

    weather_cell = [[-78.73878888888889,42.933791666666664],
                    [-78.73878888888889+0.1,42.933791666666664],
                    [-78.73878888888889+0.1,42.933791666666664+0.1],
                    [-78.73878888888889,42.933791666666664+0.1]]
    visualizer.simulate(plan, route, weather_cell=weather_cell,zoom=True)
