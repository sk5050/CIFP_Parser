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



    # plan = {"origin":"KBOS",
    #         "destination":"KLAX",
    #         "SID":{"SID_id":"HYLND5",
    #                "transitions":[{"transition_id":"RW04R"}, {"transition_id":"RW09"}, {"transition_id":"RW15R"}, {"transition_id":"RW22L"}, {"transition_id":"RW33L"}]},
    #         "routes": [
    #             ["HYLND", "CAM", "GOATR", "WOZEE", "Q935", "HOCKE", "GRB", "RST", "ONL", "CYS", "EKR", "HVE", "PROMT", "Q88", "HAKMN"],
    #                    ["HYLND", "HANAA", "Q816", "HOCKE", "Q935", "MONEE", "DABJU", "MCW", "SNY", "KD57S", "HVE", "GARDD", "Q88", "ZZYZX", "Q73", "HAKMN"],
    #                    ["HYLND", "HANAA", "Q816", "HOCKE", "Q935", "MONEE", "DABJU", "MCW", "J148", "CYS", "HVE", "GARDD", "Q88", "ZZYZX", "Q73", "HAKMN"]
    #                    ],
    #         "STAR":[{"STAR_id":"ANJLL4",
    #                  "transitions":[{"transition_id":"HAKMN"}, {"transition_id":"ALL"}]},
    #                  {"STAR_id":"BIGBR3",
    #                   "transitions":[{"transition_id":"HAKMN"}, {"transition_id":""}, {"transition_id":"RW06B"}]}],


    #         "alternative":"KLGB",
    #         "alt-STAR":[{"STAR_id":"BAUBB2",
    #                      "transitions":[{"transition_id":""}, {"transition_id":"TILLT"},{"transition_id":"RW12"},{"transition_id":"RW26R"},{"transition_id":"RW30"}]}],

    #         "alt-routes":[["PROMT", "TILLT"]]
            

    #         }

  


    plan = {"origin":"KBOS",
            "destination":"KMCO",
            "SID":[{"SID_id":"PATSS6",
                    "transitions":[{"transition_id":"RW04R"}]},
                   {"SID_id":"SSOXS6",
                    "transitions":[{"transition_id":"RW04R"}]}],

            
            "routes": [
                ["PATSS", "NELIE", "BIZEX", "Q75", "SLOJO", "Q83", "ROYCO", "Q85", "LPERD"],
                       ["BUZRD", "SEY", "HTO", "J174", "ORF", "J121", "CHS", "IGARY", "Q85", "LPERD"],
                       ["BUZRD", "SEY", "HTO", "RIFLE", "J174", "SWL", "CEBEE", "WETRO", "ILM", "AR15", "HIBAC"]
                       ],
            
            "STAR":[{"STAR_id":"SNFLD1",
                     "transitions":[{"transition_id":"LPERD"}]},
                     {"STAR_id":"ALYNA1",
                      "transitions":[{"transition_id":"HIBAC"}]}],


            "alternatives": [{"alt-airport" : "KJFK",
                              "alt-routes" : [["SLOJO", "CAMRN"]],
                              "alt-STAR" : [{"STAR_id":"CAMRN4",
                                             "transitions":[{"transition_id":"ALL"}]}]},
                             {"alt-airport" : "KCLT",
                              "alt-routes" : [["CEBEE", "MAJIC"]],
                              "alt-STAR" : [{"STAR_id":"MAJIC4",
                                             "transitions":[{"transition_id":"LIB"}]}]},
                             {"alt-airport" : "KPHL",
                              "alt-routes" : [["CEBEE", "JST"]],
                              "alt-STAR" : [{"STAR_id":"BOJID2",
                                             "transitions":[{"transition_id":"JST"}]}]},
                             {"alt-airport" : "KIAD",
                              "alt-routes" : [["CEBEE", "DORRN"]],
                              "alt-STAR" : [{"STAR_id":"CAVLR4",
                                             "transitions":[{"transition_id":"DORRN"}]}]}
                             # {"alt-airport" : "KALT",
                             #  "alt-routes" : [["CEBEE", "MAJIC"]],
                             #  "alt-STAR" : [{"STAR_id":"MAJIC4",
                             #                 "transitions":[{"transition_id":"LIB"}]}]}

                             ]

            }




    

    print(json.dumps(plan, sort_keys=True, indent=4))

    # raise ValueError(123)

    plan = Plan(plan,CIFP_parsed_filename)
    print(json.dumps(plan.augmented_plan, sort_keys=True, indent=4))

    # raise ValueError(123)

    
    # visualizer.visualize_plan(plan, lat_0=42.214660, lon_0=-71.002300, \
    #                           width=1.4E6, height=0.4E6)

    visualizer.visualize_plan(plan, lat_0=42.214660, lon_0=-95.002300, \
                              width=7E6, height=3E6, show=True)
