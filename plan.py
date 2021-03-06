#!/usr/bin/env python

import sys
import json
import copy



class Plan(object):

    def __init__(self, plan,parsed_cifp_filename=None):

        with open(parsed_cifp_filename) as parsed_cifp:
            self.cifp = json.load(parsed_cifp)

        self.plan = plan
        self.augment_plan()
        

    def augment_plan(self):


        routes = self.plan["routes"]
        route_graph = dict()

        for route in routes:

            expanded_route = []
            for i in range(len(route)):
                if route[i] in self.cifp["Airways"]:
                    route_segment = self.get_route_segment(route[i], route[i-1], route[i+1])
                    expanded_route.extend(route_segment)
                else:
                    expanded_route.append(route[i])

            for i in range(len(expanded_route)-1):
                if expanded_route[i] in route_graph:
                    route_graph[expanded_route[i]].append(expanded_route[i+1])
                else:
                    route_graph[expanded_route[i]] = [expanded_route[i+1]]


        new_route_graph = dict()
        for key,value in route_graph.items():
            new_route_graph[key] = list(set(value))

        route_graph = new_route_graph

        self.augmented_plan = copy.deepcopy(self.plan)

        self.augmented_plan["route graph"] = route_graph

        self.augmented_plan["route graph"]["root"] = routes[0][0]
        self.augmented_plan["route graph"]["terminal"] = routes[0][-1]

        print(route_graph)



    def get_route_segment(self,route_id,start_waypoint,end_waypoint):

        route = self.cifp["Airways"][route_id]

        start_pos = -1
        end_pos = -1

        for i in range(len(route)):

            if route[i]["FIX"] == start_waypoint:
                start_pos = i
            elif route[i]["FIX"] == end_waypoint:
                end_pos = i

        if start_pos < 0 or end_pos < 0:
            raise ValueError("fix not found in the airway")

        if start_pos < end_pos:
            route_segment = route[start_pos+1:end_pos]
        else:
            route_segment = route[end_pos+1:start_pos]
            route_segment.reverse()

        print("-------------")
        print(route_id)
        print(start_waypoint)
        print(end_waypoint)
        print("..................")
        print(route_segment)

        route_segment = [fix['FIX'] for fix in route_segment]
        print(route_segment)
        return route_segment
    
