#!/usr/bin/env python

import sys
import json
from utils import *
from plan_visualizer import Visualizer
from plan import *

import matplotlib.pyplot as plt

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
                # ["SSOXS", "BUZRD", "SEY", "HTO", "CEBEE", "IGARY", "LPERD"],
                # ["SSOXS", "BUZRD", "SEY", "HTO", "CEBEE", "ILM", "HIBAC"]
                ["SSOXS", "BUZRD", "SEY", "HTO", "wp1", "wp2", "wp3"],
                ["SSOXS", "BUZRD", "SEY", "HTO", "wp1", "wp4", "HIBAC"]
                       ],
            
            "STAR":[{"STAR_id":"SNFLD1",
                     "transitions":[{"transition_id":"LPERD"}]},
                     {"STAR_id":"ALYNA1",
                      "transitions":[{"transition_id":"HIBAC"}]}],


            "alternatives": [
                # {"alt-airport" : "KJFK",
                #               "alt-routes" : [["BIZEX", "CAMRN"]],
                #               "alt-STAR" : [{"STAR_id":"CAMRN4",
                #                              "transitions":[{"transition_id":"ALL"}]}]},
                #              {"alt-airport" : "KPHL",
                #               "alt-routes" : [["BIZEX", "JST"]],
                #               "alt-STAR" : [{"STAR_id":"BOJID2",
                #                              "transitions":[{"transition_id":"JST"}]}]},

                             {"alt-airport" : "KJFK",
                              "alt-routes" : [["HTO", "CAMRN"]],
                              "alt-STAR" : [{"STAR_id":"CAMRN4",
                                             "transitions":[{"transition_id":"ALL"}]}]},
                             {"alt-airport" : "KPHL",
                              "alt-routes" : [["HTO", "JST"]],
                              "alt-STAR" : [{"STAR_id":"BOJID2",
                                             "transitions":[{"transition_id":"JST"}]}]},
                             
                             
                             {"alt-airport" : "KRSW",
                              "alt-routes" : [["LPERD", "JOSFF"]],
                              "alt-STAR" : [{"STAR_id":"JOSFF5",
                                             "transitions":[{"transition_id":"ALL"}]}]},
                             {"alt-airport" : "KATL",
                              "alt-routes" : [["LPERD", "CHPPR"]],
                              "alt-STAR" : [{"STAR_id":"CHPPR1",
                                             "transitions":[{"transition_id":""}]}]},

                             
                             {"alt-airport" : "KRSW",
                              "alt-routes" : [["HIBAC", "JOSFF"]],
                              "alt-STAR" : [{"STAR_id":"JOSFF5",
                                             "transitions":[{"transition_id":"ALL"}]}]},
                             {"alt-airport" : "KMIA",
                              "alt-routes" : [["HIBAC", "ANNEY"]],
                              "alt-STAR" : [{"STAR_id":"ANNEY4",
                                             "transitions":[{"transition_id":"ALL"}]}]},

                             
                             ]

            }




    

    # print(json.dumps(plan, sort_keys=True, indent=4))

    plan = Plan(plan,CIFP_parsed_filename)
    # print(json.dumps(plan.augmented_plan, sort_keys=True, indent=4))

    # visualizer.visualize_plan(plan, lat_0=42.214660, lon_0=-71.002300, \
    #                           width=1.4E6, height=0.4E6)

    custom_wps = {'wp1':(940092.1658986174, 1203734.1153470196),
                  'wp2':(812859.5547806831, 903440.4533884865),
                  'wp3':(685626.9436627489, 603146.7914299534),
                  'wp4':(964516.1290322581, 825806.4516129044)
                  }

    # 'wp4':(1013419.3548387098, 824516.1290322589)

    visualizer.custom_wps = custom_wps

    # orientation = [[(x_1_prev, y_1_prev), (x_2_prev, y_2_prev)],
    #                [(x_1_new, y_1_new), (x_2_new, y_2_new)]]

    
    

    # point = plt.ginput(2)
    # print(point)
    # plt.pause(10000)


    # single segment

    # fig, axs = plt.subplots(ncols=1, nrows=1,figsize=(8,10))

    # visualizer.visualize_plan(plan, lat_0=34.214660, lon_0=-78.002300, \
    #                           width=2E6, height=2E6, show=True, ax=axs)
    
    # axs.set_xlim(754580.6451612904, 987096.7741935486)
    # axs.set_ylim(896421, 1211110)

    # static
    # visualizer.draw_single_segment_policy_static(fig,axs, \
    #                                              orientation=[[(0.5,15.5),(29.5,15.5)], \
    #                                                           [custom_wps['wp1'], \
    #                                                            custom_wps['wp2']]])

    # dynamic
    # visualizer.draw_single_segment_policy_dynamic(fig,axs, show=False,save_img=True,\
    #                                               orientation=[[(0.5,15.5),(29.5,15.5)], \
    #                                                            [custom_wps['wp1'], \
    #                                                             custom_wps['wp2']]])





    # two segment static
    
    # fig, axs = plt.subplots(ncols=1, nrows=2,figsize=(10,15))

    # visualizer.visualize_plan(plan, lat_0=34.214660, lon_0=-78.002300, \
    #                           width=2E6, height=2E6, show=True, ax=axs[0])

    # visualizer.visualize_plan(plan, lat_0=34.214660, lon_0=-78.002300, \
    #                           width=2E6, height=2E6, show=True, ax=axs[1])
    
    # axs[0].set_xlim(606405, 1011240)
    # axs[0].set_ylim(896421, 1211110)
    # axs[1].set_xlim(606405, 1011240)
    # axs[1].set_ylim(590661, 916805)

    # orientation = [[[(0.5,15.5),(29.5,15.5)], \
    #                 [custom_wps['wp1'], \
    #                  custom_wps['wp2']]],
    #                [[(0.5,15.5),(29.5,15.5)], \
    #                 [custom_wps['wp2'], \
    #                  custom_wps['wp3']]]]

    # visualizer.draw_two_segments_policies_static(fig,axs,orientation=orientation)

    


    # two segment w/o estimation
    
    # fig, axs = plt.subplots(ncols=1, nrows=2,figsize=(10,15))

    # visualizer.visualize_plan(plan, lat_0=34.214660, lon_0=-78.002300, \
    #                           width=2E6, height=2E6, show=True, ax=axs[0])

    # visualizer.visualize_plan(plan, lat_0=34.214660, lon_0=-78.002300, \
    #                           width=2E6, height=2E6, show=True, ax=axs[1])
    
    # axs[0].set_xlim(606405, 1011240)
    # axs[0].set_ylim(896421, 1211110)
    # axs[1].set_xlim(606405, 1111240)
    # axs[1].set_ylim(520661, 916805)

    # orientation = [[[(0.5,15.5),(29.5,15.5)], \
    #                 [custom_wps['wp1'], \
    #                  custom_wps['wp2']]],
    #                [[(0.5,15.5),(29.5,15.5)], \
    #                 [custom_wps['wp2'], \
    #                  custom_wps['wp3']]]]

    # visualizer.draw_two_segments_policies_dynamic_wo_estimation(fig,axs, show=False,save_img=True,\
    #                                                             orientation=orientation)





    # two segment w/ estimation
    
    # fig, axs = plt.subplots(ncols=1, nrows=2,figsize=(10,15))

    # visualizer.visualize_plan(plan, lat_0=34.214660, lon_0=-78.002300, \
    #                           width=2E6, height=2E6, show=True, ax=axs[0])

    # visualizer.visualize_plan(plan, lat_0=34.214660, lon_0=-78.002300, \
    #                           width=2E6, height=2E6, show=True, ax=axs[1])
    
    # axs[0].set_xlim(606405, 1011240)
    # axs[0].set_ylim(896421, 1211110)
    # axs[1].set_xlim(606405, 1111240)
    # axs[1].set_ylim(520661, 916805)

    # orientation = [[[(0.5,15.5),(29.5,15.5)], \
    #                 [custom_wps['wp1'], \
    #                  custom_wps['wp2']]],
    #                [[(0.5,15.5),(29.5,15.5)], \
    #                 [custom_wps['wp2'], \
    #                  custom_wps['wp3']]]]

    # visualizer.draw_two_segments_policies_dynamic_w_estimation(fig,axs, show=False,save_img=True,\
    #                                                             orientation=orientation)

    





    # hierarchical

    fig, axs = plt.subplots(ncols=1, nrows=1,figsize=(12,15))

    visualizer.visualize_plan(plan, lat_0=34.214660, lon_0=-78.002300, \
                              width=2E6, height=2E6, show=True, ax=axs)
    
    visualizer.draw_hierarchical_dynamic(fig,axs, show=True,save_img=False,\
                                         orientation=[[[(0.5,15.5),(29.5,15.5)], \
                                                       [custom_wps['wp1'], \
                                                        custom_wps['wp2']]],

                                                       [[(0.5,15.5),(29.5,15.5)], \
                                                        [custom_wps['wp2'], \
                                                         custom_wps['wp3']]],

                                                       [[(0.5,15.5),(29.5,15.5)], \
                                                        [custom_wps['wp1'], \
                                                         custom_wps['wp4']]]])
