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
                      "transitions":[{"transition_id":"HAKMN"}, {"transition_id":""}, {"transition_id":"RW06B"}]}]



            }

    print(json.dumps(plan, sort_keys=True, indent=4))

    raise ValueError(123)

    plan = Plan(plan,CIFP_parsed_filename)
    print(json.dumps(plan.augmented_plan, sort_keys=True, indent=4))

    # raise ValueError(123)

    
    # visualizer.visualize_plan(plan, lat_0=42.214660, lon_0=-71.002300, \
    #                           width=1.4E6, height=0.4E6)

    visualizer.visualize_plan(plan, lat_0=42.214660, lon_0=-95.002300, \
                              width=7E6, height=3E6)
