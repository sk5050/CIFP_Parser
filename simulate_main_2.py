#!/usr/bin/env python

import sys
import json
from utils import *
from plan_visualizer import Visualizer
from plan import *

if __name__ == '__main__':

    
    CIFP_parsed_filename = 'CIFP_parsed/CIFP_parsed'
    

    visualizer = Visualizer(CIFP_parsed_filename)


    plan = {"origin":"KBOS",
            "destination":"KMCO",
            "SID":[{"SID_id":"PATSS6",
                    "transitions":[{"transition_id":"RW04R"}]},
                   {"SID_id":"SSOXS6",
                    "transitions":[{"transition_id":"RW04R"}]}],

            
            "routes": [
                # ["PATSS", "NELIE", "BIZEX", "Q75", "SLOJO", "Q83", "ROYCO", "Q85", "LPERD"],
                # ["BUZRD", "SEY", "HTO", "J174", "ORF", "J121", "CHS", "IGARY", "Q85", "LPERD"],
                # ["BUZRD", "SEY", "HTO", "RIFLE", "J174", "SWL", "CEBEE", "WETRO", "ILM", "AR15", "HIBAC"]
                # ["PATSS", "NELIE", "BIZEX", "SLOJO", "ROYCO", "LPERD"],
                ["SSOXS", "BUZRD", "SEY", "HTO", "CEBEE", "IGARY", "LPERD"],
                ["SSOXS", "BUZRD", "SEY", "HTO", "CEBEE", "ILM", "HIBAC"]
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

    # route = ["SSOXS", "BUZRD", "SEY"]

    # route = [[1166371.3069008384, 1288068.3471314851], [1243076.6789218928, 1132012.5902610642], [1068505.8322532861, 1110852.4876345664]]
    route = [[1257423.448701942, 1428725.8826406517], [1167325.2416562622, 1289717.2203416026], [1020593.8758961551, 1003977.1922824464], [848120.1652658533, 422200.19821605604],
             [703963.0339927657, 396457.85334586183]]
    plan = Plan(plan,CIFP_parsed_filename)

    # weather_cell = [[42.933791666666664, -78.73878888888889],[42.933791666666664+0.01, -78.73878888888889],[42.933791666666664+0.01, -78.73878888888889+0.01],[42.933791666666664, -78.73878888888889+0.01]]


    # cell_ref = ((1.903225806451613, 4.182900432900434),
    #         (1.1169354838709675, 3.425324675324676),
    #         (1.758064516129032, 2.83008658008658),
    #         (2.060483870967742, 2.2619047619047623),
    #         (2.07258064516129, 1.8425324675324677),
    #         (2.375, 1.8154761904761907),
    #         (2.616935483870968, 2.234848484848485),
    #         (2.725806451612903, 2.816558441558442),
    #         (2.435483870967742, 3.195346320346321),
    #         (2.411290322580645, 3.6553030303030303),
    #         (2.798387096774193, 4.088203463203464),
    #         (2.774193548387097, 4.6022727272727275),
    #         (2.169354838709677, 4.710497835497836))

    # cell_center = [-111.60135592592592+0.8, 37.780637962962956+0.7]

    # weather_cell = []
    # factor = 0.3
    # for ref in cell_ref:
    #     weather_cell.append([cell_center[0]+ref[0]*factor, cell_center[1]+ref[1]*factor])

    # weather_cell_ref = [[-50000,50000],[50000,50000],[50000,-50000],[-50000,-50000]]
    # weather_route = [[1599800, 1158430],[801784, 1194470]]

    weather_cell_ref = [[0.0, 0.0],
 [-51484.68974038819, -33465.04833125253],
 [-51484.68974038819, -72078.56563654379],
 [36039.282818272244, -64355.862175485585],
 [59207.39320144709, 0.0]]

    weather_route = [[1372868.8556240585, 1086352.695867068],[619013.2959211248, 1251103.703036311]]

    r1 = [[1267720.38665002, 1433874.3516146904], [1167325.2416562622, 1297439.9238026608], [1015445.4069221162, 998828.7233084075], [848120.1652658533, 424774.4327030755],
          [698814.5650187267, 391309.384371823]]

    
    
    r2 = [[850694.3997528728, 427348.6671900949], [801783.9444995036, 131311.701182861]]

    r3 = [[1229106.8693447283, 1377241.192900263], [1177622.17960434, 1297439.9238026608], [1190493.352039437, 1140411.6200944758], [1048910.4552533685, 1076055.7579189902],
          [1012871.1724350967, 1001402.957795427]]

    x1=[]
    y1=[]
    x2=[]
    y2=[]
    x3=[]
    y3=[]

    for xx in r1:
        x1.append(xx[0])
        y1.append(xx[1])

    for xx in r2:
        x2.append(xx[0])
        y2.append(xx[1])

    for xx in r3:
        x3.append(xx[0])
        y3.append(xx[1])

    add_routes_set = [{'time':5,
                       'routes':[[x1,y1],
                                 [x2,y2]]},
                      {'time':30,
                       'routes':[[x3,y3]]}]


    # temp_route = [[-108.34710964285715,39.3607325],
    #               [-109+0.5, 39-0.15],
    #               [-110.69974166666667, 38.41680833333333]]

    # route = ["SNY","KD57S", [-108.34710964285715,39.3607325], [-109+0.5, 39-0.15], [-110.69974166666667, 38.41680833333333]]

    visualizer.simulate(plan, route, weather_cell_ref=weather_cell_ref,weather_route=weather_route,zoom=False,zoom_size=[-500000,100000,-150000,150000],save_img=False, \
                        add_routes_set=add_routes_set)

