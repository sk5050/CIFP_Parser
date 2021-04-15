#!/usr/bin/env python

import sys
import json
import math
import statistics
import matplotlib.pyplot as plt
import matplotlib as mlt
from utils import *



class Event(object):

    def __init__(self, name, is_root=False, is_terminal=False, \
                 is_choice=False, is_uncontrollable=False):

        self.name = name
        self.parents = []
        self.children = []
        self.is_root = is_root
        self.is_terminal = is_terminal
        self.is_choice = is_choice
        self.is_uncontrollable = is_uncontrollable
        
        self.depth = 0


class Episode(object):

    def __init__(self, name, start_e_name, end_e_name):

        self.name = name
        self.start_e_name = start_e_name
        self.end_e_name = end_e_name



class Graph(object):

    def __init__(self, start_event, end_event, event_set, episode_set):

        self.start_event = start_event
        self.end_event = end_event
        self.event_set = event_set
        self.episode_set = episode_set

        self.max_depth = self.compute_max_depth()
        self.max_width = self.compute_max_width()


    def compute_max_depth(self):

        return 4


    def compute_max_width(self):

        return 4
        


class Visualizer(object):

    def __init__(self, graph):

        self.graph = graph


    def draw_tpn(self):

        fig, axs = plt.subplots(1,1, figsize=(8, 4.8))



    def draw_events(self):

        # events_with_depth = sort_events_with_depth()


        queue = [self.graph.start_event]

        drawn_list = []

        while queue:
            
            node = queue.pop(0)

            width = len(node.parent.children)
            order = node.parent.children.index(node)

            draw_event(node,width,order)

            children = node.children

            for child in children:
                if child not in drawn_list:
                    queue.append(child)


    def draw_event(self,event,width,order):

        max_width = self.graph.max_width

        if event.is_root == True:
            pos = (0,0)
        else:
            parents = event.parents
            if len(parents)>1:
                x_list = [parent.pos[0] for parent in parents]
                y_list = [parent.pos[1] for parent in parents]
                pos = statistics.mean(x_list)
                pos = statistics.mean(y_list)
            else:
                if width==max_width:
                    interval = max_width / width
                

        circle = plt.Circle((x1,y1), 0.05, color='b',alpha=0.3)

        ax.add_patch(circle1)


    # def sort_events_with_depth(self):
        
    #     for event in graph.event_set:
    #         depth = event.depth

    #         if depth in events_with_depth:
    #             events_with_depth = events_with_depth[depth].append(event)
    #         else:
    #             events_with_depth[depth] = [event]



    def draw_connector(self,ax, x1,y1,x2,y2, connectionstyle):

        circle1 = plt.Circle((x1,y1), 0.05, color='b',alpha=0.3)
        circle2 = plt.Circle((x2,y2), 0.05, color='r',alpha=0.3)

        ax.add_patch(circle1)
        ax.add_patch(circle2)

        ax.annotate("",
                    xy=(x1, y1), xycoords='data',
                    xytext=(x2, y2), textcoords='data',
                    arrowprops=dict(arrowstyle="->", color="0.5",
                                    shrinkA=5, shrinkB=5,
                                    patchA=None, patchB=None,
                                    connectionstyle=connectionstyle,
                                    ),
                    )

        ax.text(.05, .95, connectionstyle.replace(",", ",\n"),
                transform=ax.transAxes, ha="left", va="top")
