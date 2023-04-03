#!/usr/bin/env python

import sys
import json
import matplotlib.pyplot as plt
import matplotlib as mlt
from mpl_toolkits.basemap import Basemap
from PIL import Image
from pylab import rcParams
from utils import *
import geopy.distance
import numpy as np
from plan import *
import math
import time
import random
import copy
import shapely
from shapely.geometry import Point, Polygon, LineString
from descartes import PolygonPatch
from matplotlib.collections import PatchCollection




from grid import Grid
# # import functools

from matplotlib.collections import LineCollection, PolyCollection
from matplotlib.patches import Ellipse

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import ticker
import numpy as np
from PIL import Image
import cv2

import math
import time
import random
import cProfile

import json



class Visualizer(object):

    def __init__(self, parsed_cifp_filename,lon_lb=0,lat_lb=0,lon_ub=1,lat_ub=1):

        with open(parsed_cifp_filename) as parsed_cifp:
            self.cifp = json.load(parsed_cifp)

        self.plan = None
        self.basemap = None

        self.custom_wps = dict()
        


    def visualize_plan(self,plan,lat_0,lon_0,width,height,resolution='h',show=False, ax=None):

        self.plan = plan.augmented_plan


        if ax==None:
            self.draw_map(lat_0,lon_0,width,height,resolution=resolution)
        else:
            self.draw_map(lat_0,lon_0,width,height,resolution=resolution, ax=ax)
            
        # self.draw_airport()

        if self.plan["SID"]==None:
            available_SIDs = self.find_available_SIDs(self.plan)
        else:
            for SID in self.plan["SID"]:
                self.draw_SID(airport=self.plan["origin"], SID_id=SID["SID_id"],\
                              transitions=SID["transitions"],ax=ax)
            
            # self.draw_SID(airport=self.plan["origin"], SID_id=self.plan["SID"]["SID_id"],\
            #               transitions=self.plan["SID"]["transitions"])

        self.draw_route(self.plan["route graph"])


        if "alternatives" in self.plan:
            for alternative in self.plan["alternatives"]:
                self.draw_alternatives(alternative)

        # if "alt route graph" in self.plan:
        #     self.draw_alt_route(self.plan["alt route graph"],style='kD-')

        # if "alt route 2 graph" in self.plan:
        #     self.draw_alt_route(self.plan["alt route 2 graph"])
            
        if "planned route graph" in self.plan:
            self.draw_route(self.plan["planned route graph"],style='bD-',alpha_val=0.8)

        if self.plan["STAR"]==None:
            available_STARs = self.find_available_STARs(self.plan)
        else:
            for STAR in self.plan["STAR"]:
                print(STAR)
                self.draw_STAR(airport=self.plan["destination"], STAR_id=STAR["STAR_id"],\
                              transitions=STAR["transitions"])


        

        # xx = plt.ginput(5)
        # print(xx)
        # xx = [(1166371.3069008384, 1288068.3471314851), (1243076.6789218928, 1132012.5902610642), (1068505.8322532861, 1110852.4876345664)]
        # x = []
        # y = []
        # for z in xx:
        #     x.append(z[0])
        #     y.append(z[1])
        # self.basemap.plot(x,y, linewidth=5,MarkerSize=10)
                

        if show==True:
            plt.show(block=False)

        # ax = plt.axes()

        

        return ax


    def draw_airport(self, ax=None):

        origin = self.plan["origin"]
        dest = self.plan["destination"]
           

        origin_coord = self.get_wp_coord(origin, airport=origin)
        dest_coord = self.get_wp_coord(dest, airport=dest)

        if ax==None:
            self.basemap.plot(origin_coord[1], origin_coord[0], 'y*',latlon=True,linewidth=5,MarkerSize=10,alpha=0.8)
            self.basemap.plot(dest_coord[1], dest_coord[0], 'y*',latlon=True,linewidth=5,MarkerSize=10,alpha=0.8)
        else:
            self.basemap.plot(origin_coord[1], origin_coord[0], 'y*',latlon=True,linewidth=5,MarkerSize=10,alpha=0.8, ax=ax)
            self.basemap.plot(dest_coord[1], dest_coord[0], 'y*',latlon=True,linewidth=5,MarkerSize=10,alpha=0.8,ax=ax)


        if "alternative" in self.plan:
            alt = self.plan["alternative"]
            alt_coord = self.get_wp_coord(alt, airport=alt)
            if ax==None:
                self.basemap.plot(origin_coord[1], origin_coord[0], 'y*',latlon=True,linewidth=5,MarkerSize=10,alpha=0.5)
            else:
                self.basemap.plot(origin_coord[1], origin_coord[0], 'y*',latlon=True,linewidth=5,MarkerSize=10,alpha=0.5,ax=ax)



    def draw_route(self,route_graph,style='ko-',alpha_val=0.8, ax=None):

        lat_set = []
        lon_set = []
        


        for head_waypoint, tail_waypoints in route_graph.items():

            if head_waypoint == "root" or head_waypoint=="terminal":
                continue

            for tail_waypoint in tail_waypoints:

                lat = []
                lon = []

                if head_waypoint==route_graph["root"]:
                    wp_coord = self.get_wp_coord(head_waypoint, airport=self.plan["origin"])
                elif head_waypoint==route_graph["terminal"]:
                    wp_coord = self.get_wp_coord(head_waypoint, airport=self.plan["destination"])
                else:
                    wp_coord = self.get_wp_coord(head_waypoint)

                if wp_coord==None:
                    wp_coord = self.get_wp_coord(head_waypoint, airport=self.plan["origin"])
                if wp_coord==None:
                    wp_coord = self.get_wp_coord(head_waypoint, airport=self.plan["destination"])
                
                    

                print(head_waypoint)
                print(tail_waypoints)
                lat.append(wp_coord[0])
                lon.append(wp_coord[1])

                if tail_waypoint==route_graph["root"]:
                    wp_coord = self.get_wp_coord(tail_waypoint, airport=self.plan["origin"])
                elif tail_waypoint==route_graph["terminal"]:
                    wp_coord = self.get_wp_coord(tail_waypoint, airport=self.plan["destination"])
                else:
                    wp_coord = self.get_wp_coord(tail_waypoint)

                if wp_coord==None:
                    wp_coord = self.get_wp_coord(head_waypoint, airport=self.plan["origin"])
                if wp_coord==None:
                    wp_coord = self.get_wp_coord(head_waypoint, airport=self.plan["destination"])

                lat.append(wp_coord[0])
                lon.append(wp_coord[1])

                lat_set.append(lat)
                lon_set.append(lon)

        for lat, lon in zip(lat_set, lon_set):
            if ax==None:
                self.basemap.plot(lon,lat, style,latlon=True,linewidth=2,MarkerSize=5,alpha=alpha_val)
            else:
                self.basemap.plot(lon,lat, style,latlon=True,linewidth=2,MarkerSize=5,alpha=alpha_val, ax=ax)




    def draw_alternatives(self, alternative, ax=None):

        alt_airport = alternative["airport"]

        if ax==None:
            self.draw_alt_route(alt_airport, alternative["route graph"])
        else:
            self.draw_alt_route(alt_airport, alternative["route graph"], ax=ax)

        
        
        # if "STAR" in alternative:
        #     for STAR in alternative["STAR"]:
        #         self.draw_STAR(airport=alternative["airport"], STAR_id=STAR["STAR_id"],\
        #                        transitions=STAR["transitions"])


                

    def draw_alt_route(self,alt_airport, route_graph,style='k--', ax=None):

        lat_set = []
        lon_set = []


        for head_waypoint, tail_waypoints in route_graph.items():

            if head_waypoint == "root" or head_waypoint=="terminal":
                continue

            for tail_waypoint in tail_waypoints:
                print(head_waypoint)

                lat = []
                lon = []

                # if head_waypoint==route_graph["root"]:
                #     wp_coord = self.get_wp_coord(head_waypoint, airport=self.plan["destination"])
                # elif head_waypoint==route_graph["terminal"]:
                #     wp_coord = self.get_wp_coord(head_waypoint, airport=self.plan["alternative"])
                # else:
                #     wp_coord = self.get_wp_coord(head_waypoint)

                wp_coord = self.get_wp_coord(head_waypoint, airport=self.plan["destination"])
                if wp_coord ==None:
                    wp_coord = self.get_wp_coord(head_waypoint, airport=alt_airport)
                if wp_coord ==None:
                    wp_coord = self.get_wp_coord(head_waypoint, airport=self.plan["origin"])
                if wp_coord ==None:
                    wp_coord = self.get_wp_coord(head_waypoint)

                
                lat.append(wp_coord[0])
                lon.append(wp_coord[1])

                # if tail_waypoint==route_graph["root"]:
                #     wp_coord = self.get_wp_coord(tail_waypoint, airport=self.plan["destination"])
                # elif tail_waypoint==route_graph["terminal"]:
                #     wp_coord = self.get_wp_coord(tail_waypoint, airport=self.plan["alternative"])
                # else:
                #     wp_coord = self.get_wp_coord(tail_waypoint)


                wp_coord = self.get_wp_coord(tail_waypoint, airport=self.plan["destination"])
                if wp_coord ==None:
                    wp_coord = self.get_wp_coord(tail_waypoint, airport=alt_airport)
                if wp_coord ==None:
                    wp_coord = self.get_wp_coord(tail_waypoint, airport=self.plan["origin"])
                if wp_coord ==None:
                    wp_coord = self.get_wp_coord(tail_waypoint)                

                lat.append(wp_coord[0])
                lon.append(wp_coord[1])

                lat_set.append(lat)
                lon_set.append(lon)

        for lat, lon in zip(lat_set, lon_set):
            # if alt_airport=="KATL":
            if ax==None:
                self.basemap.plot(lon,lat, style,latlon=True,linewidth=2,MarkerSize=5,alpha=0.8)
            else:
                self.basemap.plot(lon,lat, style,latlon=True,linewidth=2,MarkerSize=5,alpha=0.8, ax=ax)
                
            # else:
            #     self.basemap.plot(lon,lat, 'r-',latlon=True, linewidth=6, MarkerSize=3, alpha=0.8)
            
            

        if ax==None:
            self.basemap.plot(lon[-1], lat[-1], 'g*',latlon=True,linewidth=5,MarkerSize=20,alpha=1.0)
        else:
            self.basemap.plot(lon[-1], lat[-1], 'g*',latlon=True,linewidth=5,MarkerSize=20,alpha=1.0, ax=ax)



    def draw_planned_route(self,route_graph,ax=None):

        lat_set = []
        lon_set = []


        for head_waypoint, tail_waypoints in route_graph.items():

            if head_waypoint == "root" or head_waypoint=="terminal":
                continue

            for tail_waypoint in tail_waypoints:
                print(head_waypoint)

                lat = []
                lon = []

                if head_waypoint==route_graph["root"]:
                    wp_coord = self.get_wp_coord(head_waypoint, airport=self.plan["origin"])
                elif head_waypoint==route_graph["terminal"]:
                    wp_coord = self.get_wp_coord(head_waypoint, airport=self.plan["destination"])
                else:
                    wp_coord = self.get_wp_coord(head_waypoint)

                lat.append(wp_coord[0])
                lon.append(wp_coord[1])

                if tail_waypoint==route_graph["root"]:
                    wp_coord = self.get_wp_coord(tail_waypoint, airport=self.plan["origin"])
                elif tail_waypoint==route_graph["terminal"]:
                    wp_coord = self.get_wp_coord(tail_waypoint, airport=self.plan["destination"])
                else:
                    wp_coord = self.get_wp_coord(tail_waypoint)

                lat.append(wp_coord[0])
                lon.append(wp_coord[1])

                lat_set.append(lat)
                lon_set.append(lon)

        for lat, lon in zip(lat_set, lon_set):
            if ax==None:
                self.basemap.plot(lon,lat, 'rD-',latlon=True,linewidth=5,MarkerSize=10,alpha=0.2)
            else:
                self.basemap.plot(lon,lat, 'rD-',latlon=True,linewidth=5,MarkerSize=10,alpha=0.2,ax=ax)


    def draw_SID(self,airport, SID_id, transitions, ax=None):


        for transition in transitions:

            lat = []
            lon = []


            transition_id = transition["transition_id"]

            procedure = self.cifp['Airport'][airport]['SID'][SID_id][transition_id]['Procedure']

            for step in procedure:
                wp_name = step['FIX']
                mag_crs = step['MAG CRS']
                path_term = step['PATH TERM']

                wp_coord = self.get_wp_coord(wp_name,airport)
                lat.append(wp_coord[0])
                lon.append(wp_coord[1])

            lon_simplified = [lon[0], lon[-1]]
            lat_simplified = [lat[0], lat[-1]]

            if ax==None:
                self.basemap.plot(lon_simplified,lat_simplified, 'ko-',latlon=True,linewidth=2,MarkerSize=5,alpha=0.8)
            else:
                self.basemap.plot(lon_simplified,lat_simplified, 'ko-',latlon=True,linewidth=2,MarkerSize=5,alpha=0.8, ax=ax)




    def draw_STAR(self,airport, STAR_id, transitions, ax=None):


        for transition in transitions:

            lat = []
            lon = []


            transition_id = transition["transition_id"]

            procedure = self.cifp['Airport'][airport]['STAR'][STAR_id][transition_id]['Procedure']

            for step in procedure:
                wp_name = step['FIX']
                mag_crs = step['MAG CRS']
                path_term = step['PATH TERM']

                wp_coord = self.get_wp_coord(wp_name,airport)
                lat.append(wp_coord[0])
                lon.append(wp_coord[1])

            lon_simplified = [lon[0], lon[-1]]
            lat_simplified = [lat[0], lat[-1]]

            if ax==None:
                self.basemap.plot(lon_simplified,lat_simplified, 'ko-',latlon=True,linewidth=2,MarkerSize=5,alpha=0.8)
            else:
                self.basemap.plot(lon_simplified,lat_simplified, 'ko-',latlon=True,linewidth=2,MarkerSize=5,alpha=0.8, ax=ax)

        if ax==None:
            self.basemap.plot(lon_simplified[-1], lat_simplified[-1], 'y*',latlon=True,linewidth=5,MarkerSize=20,alpha=1.0)
        else:
            self.basemap.plot(lon_simplified[-1], lat_simplified[-1], 'y*',latlon=True,linewidth=5,MarkerSize=20,alpha=1.0,ax=ax)

    def draw_map(self,lat_0,lon_0,width,height,resolution='h', ax=None):

        if ax==None:
            plt.figure(figsize=(20, 15))

            self.basemap = Basemap(projection='lcc', resolution=resolution, 
                                   lat_0=lat_0, lon_0=lon_0,
                                   width=width, height=height)
        else:
            self.basemap = Basemap(projection='lcc', resolution=resolution, 
                                   lat_0=lat_0, lon_0=lon_0,
                                   width=width, height=height, ax=ax)

        self.basemap.shadedrelief()
        self.basemap.drawcoastlines(color='gray')
        self.basemap.drawcountries(color='gray')
        self.basemap.drawstates(color='gray')



    # def get_route_segment(self,route_id,start_waypoint,end_waypoint):

    #     route = self.cifp["Airways"][route_id]

    #     start_pos = -1
    #     end_pos = -1

    #     for i in range(len(route)):

    #         if route[i]["FIX"] == start_waypoint:
    #             start_pos = i
    #         elif route[i]["FIX"] == end_waypoint:
    #             end_pos = i

    #     if start_pos < 0 or end_pos < 0:
    #         raise ValueError("fix not found in the airway")

    #     if start_pos < end_pos:
    #         route_segment = route[start_pos:end_pos-1]
    #     else:
    #         route_segment = route[end_pos:start_pos-1].reverse()

    #     return route_segment
            

        
    def get_wp_coord(self,wp_name,airport=None):

        if wp_name in self.custom_wps:
            point = self.custom_wps[wp_name]
            lon,lat = self.basemap(point[0],point[1],inverse=True)

        else:

            if airport==None:
                if wp_name in self.cifp['Enroute Waypoints']:
                    lat_str = self.cifp['Enroute Waypoints'][wp_name]['LATITUDE']
                    lon_str = self.cifp['Enroute Waypoints'][wp_name]['LONGITUDE']

                elif wp_name in self.cifp['NDB']:
                    lat_str = self.cifp['NDB'][wp_name]['LATITUDE']
                    lon_str = self.cifp['NDB'][wp_name]['LONGITUDE']

                else:
                    return None
                    # print(wp_name)
                    # raise ValueError("Waypoint not found.")


            else:

                if wp_name in self.cifp['NDB']:
                    lat_str = self.cifp['NDB'][wp_name]['LATITUDE']
                    lon_str = self.cifp['NDB'][wp_name]['LONGITUDE']

                elif wp_name in self.cifp['Enroute Waypoints']:
                    lat_str = self.cifp['Enroute Waypoints'][wp_name]['LATITUDE']
                    lon_str = self.cifp['Enroute Waypoints'][wp_name]['LONGITUDE']

                else:

                    if wp_name==airport:
                        lat_str = self.cifp['Airport'][airport]['LATITUDE']
                        lon_str = self.cifp['Airport'][airport]['LONGITUDE']

                    elif wp_name in self.cifp['Airport'][airport]['Terminal Waypoints']:
                        lat_str = self.cifp['Airport'][airport]['Terminal Waypoints'][wp_name]['LATITUDE']
                        lon_str = self.cifp['Airport'][airport]['Terminal Waypoints'][wp_name]['LONGITUDE']

                    else:
                        return None
                        # print(wp_name)
                        # raise ValueError("Waypoint not found.")


            lat = float(lat_str[1:3]) + float(lat_str[3:5])/60 + float(lat_str[5:])/360000

            lat_dir = lat_str[0]

            if lat_dir=="S":
                lat = -lat


            lon = float(lon_str[1:4]) + float(lon_str[4:6])/60 + float(lon_str[6:])/360000
            lon_dir = lon_str[0]

            if lon_dir=="W":
                lon = -lon


        # xpt,ypt = self.basemap(lon,lat)
        # print("------------")
        # print(wp_name)
        # print(str(xpt)+', '+str(ypt))

        return [lat,lon]


    # def sort_route_graph(self,route_graph):

    #     route_segments = []

    #     queue = [(None,route_graph["root"])]

    #     visited_list = set()

    #     route_segment = []
    #     while queue:

    #         expanding_node = queue.pop()

    #         if expanding_node[1] in visited_list or expanding_node[1]==route_graph["terminal"]:
    #             route_segment.append(expanding_node[1])
    #             route_segments.append(route_segment)
    #             route_segment = []

    #         else:
    #             children = route_graph[expanding_node[1]]

    #             for child in children:
    #                 queue.append((expanding_node[1],child))

    #             if route_segment==[]:
    #                 if expanding_node[0]!=None:
    #                     route_segment.append(expanding_node[0])
    #                 route_segment.append(expanding_node[1])
    #             else:
    #                 route_segment.append(expanding_node[1])

    #             visited_list.add(expanding_node[1])

    #     for r in route_segments:
    #         print(r)

    #     raise ValueError(123)

    #     return route_segments





    def compute_traj(self, route, step_size=10000):

        wp_lat = []
        wp_lon = []
        for waypoint in route:

            if type(waypoint)==str:
                wp_coord = self.get_wp_coord(waypoint)
                wp_lat.append(wp_coord[0])
                wp_lon.append(wp_coord[1])

            else:
                wp_lat.append(waypoint[1])
                wp_lon.append(waypoint[0])


        traj_lat = []
        traj_lon = []
        for i in range(len(wp_lat)-1):

            coords_1 = (wp_lat[i], wp_lon[i])
            coords_2 = (wp_lat[i+1], wp_lon[i+1])
            # dist = geopy.distance.distance(coords_1, coords_2).nm
            dist = math.dist(coords_1, coords_2)

            num_steps = dist / step_size

            traj_lat_temp = list(np.linspace(coords_1[0], coords_2[0], num_steps))
            traj_lon_temp = list(np.linspace(coords_1[1], coords_2[1], num_steps))

            traj_lat.extend(traj_lat_temp)
            traj_lon.extend(traj_lon_temp)

        return traj_lat, traj_lon


    def compute_weather_traj(self, weather_cell, num_steps=1500):

        weather_traj = [weather_cell]

        for i in range(num_steps):

            new_weather_cell = copy.deepcopy(weather_traj[-1])

            for coords in new_weather_cell:

                coords[0] += 0.01*0.15
                coords[0] += 0.05*0.15

            weather_traj.append(new_weather_cell)

        return weather_traj


    def simulate(self, plan, route, weather_cell_ref_1=None, weather_route_1=None,weather_cell_ref_2=None, weather_route_2=None, \
                 zoom=False,zoom_size=[-1500000,500000,-1000000,500000],save_img=False,temp_route=None,add_routes_set=None):

        # ax = self.visualize_plan(plan, lat_0=42.214660, lon_0=-95.002300,\
        #                       width=7E6, height=3E6)

        ax = self.visualize_plan(plan, lat_0=34.214660, lon_0=-78.002300, \
                                 width=2E6, height=2E6)


        if temp_route!=None:
            lons = [x[0] for x in temp_route]
            lats = [x[1] for x in temp_route]
            mlons,mlats = self.basemap(lons,lats)

            for i in range(len(lons)-1):
                self.basemap.plot([lons[i],lons[i+1]],[lats[i],lats[i+1]], 'gD-',latlon=True,linewidth=5,MarkerSize=10,alpha=0.2)            
                


        traj_lat, traj_lon = self.compute_traj(route, 3000)


        # if weather_cell_ref!=None:
        weather_traj_lat_1, weather_traj_lon_1 = self.compute_traj(weather_route_1,2350)
        weather_traj_lat_2, weather_traj_lon_2 = self.compute_traj(weather_route_2,1450)

        if len(weather_traj_lat_1) < len(traj_lat):
            diff = len(traj_lat) - len(weather_traj_lat_1) + 100
            weather_traj_lat_1.extend([0]*diff)
            weather_traj_lon_1.extend([0]*diff)

        if len(weather_traj_lat_2) < len(traj_lat):
            diff = len(traj_lat) - len(weather_traj_lat_2) + 100
            weather_traj_lat_2.extend([0]*diff)
            weather_traj_lon_2.extend([0]*diff)
            


        k=0

        for lat,lon,weather_lat_1,weather_lon_1,weather_lat_2, weather_lon_2 in \
            zip(traj_lat,traj_lon,weather_traj_lat_1,weather_traj_lon_1,weather_traj_lat_2,weather_traj_lon_2):
            
            for add_routes in add_routes_set:
                if k==add_routes['time']:
                    for additional_route in add_routes['routes']:
                        route_object = self.basemap.plot(additional_route[0],additional_route[1], 'r-', linewidth=6, MarkerSize=3, alpha=0.8)
                        add_routes['object'].append(route_object)

                        # if k==50:
                        #     xx = plt.ginput(2)
                        #     print(xx)

                if k>=add_routes['end_time']:
                    for obj in add_routes['object']:
                        obj[0].remove()
                    add_routes['object'] = []
                    for additional_route in add_routes['routes']:
                        alpha_val = 0.5*(100 - (k*2-add_routes['end_time']))/100
                        if alpha_val<0:
                            alpha_val = 0.0
                        route_object = self.basemap.plot(additional_route[0],additional_route[1], 'k-', linewidth=5, MarkerSize=3, alpha=alpha_val)
                        add_routes['object'].append(route_object)
                
                

            # print(lon, lat)

            # x,y = self.basemap(lon,lat)

            traj_history = self.basemap.plot(lon,lat, 'o', markerfacecolor="None", markeredgecolor='blue', markeredgewidth=3, markersize=12)

            dx = traj_lon[k+6] - lon
            dy = traj_lat[k+6] - lat

            traj_arrow_history = self.basemap.quiver(x=lon, y=lat, u=dx, v=dy, linewidth=1.2, color='blue', headwidth=2, zorder=1000)

            # if weather_cell_ref!=None:

            if weather_traj_lon_1[k+50]!=0:

                lons_1 = [weather_lon_1+x[0] for x in weather_cell_ref_1]
                lats_1 = [weather_lat_1+x[1] for x in weather_cell_ref_1]
                cell_1 = [[x,y] for x,y in zip(lons_1,lats_1)]
                p_1 = Polygon(cell_1)
                pp_1 = PolygonPatch(p_1, fc='red', ec='black', alpha=0.4)
                weather_traj_history_1 = ax.add_patch(pp_1)

            
                lons_1_current_center = sum(lons_1) / len(weather_cell_ref_1)
                lats_1_current_center = sum(lats_1) / len(weather_cell_ref_1)

                lons_1_future_center = sum([weather_traj_lon_1[k+50]+x[0] for x in weather_cell_ref_1])/len(weather_cell_ref_1)
                lats_1_future_center = sum([weather_traj_lat_1[k+50]+x[1] for x in weather_cell_ref_1])/len(weather_cell_ref_1)

                dx = lons_1_future_center - lons_1_current_center
                dy = lats_1_future_center - lats_1_current_center

                weather_arrow_history_1 = self.basemap.quiver(x=lons_1_current_center, y=lats_1_current_center, u=dx, v=dy,linewidth=1.2, color='black', headwidth=2)


            lons_2 = [weather_lon_2+x[0] for x in weather_cell_ref_2]
            lats_2 = [weather_lat_2+x[1] for x in weather_cell_ref_2]
            cell_2 = [[x,y] for x,y in zip(lons_2,lats_2)]
            p_2 = Polygon(cell_2)
            pp_2 = PolygonPatch(p_2, fc='red', ec='black', alpha=0.4)
            weather_traj_history_2 = ax.add_patch(pp_2)

            if weather_traj_lon_2[k+50]!=0:
                lons_2_current_center = sum(lons_2) / len(weather_cell_ref_2)
                lats_2_current_center = sum(lats_2) / len(weather_cell_ref_2)

                lons_2_future_center = sum([weather_traj_lon_2[k+50]+x[0] for x in weather_cell_ref_2])/len(weather_cell_ref_2)
                lats_2_future_center = sum([weather_traj_lat_2[k+50]+x[1] for x in weather_cell_ref_2])/len(weather_cell_ref_2)

                dx = lons_2_future_center - lons_2_current_center
                dy = lats_2_future_center - lats_2_current_center

                weather_arrow_history_2 = self.basemap.quiver(x=lons_2_current_center, y=lats_2_current_center, u=dx, v=dy,linewidth=1.2, color='black', headwidth=2)

            # ref_x,ref_y = self.basemap(lon,lat)
            # xbound = [ref_x+zoom_size[0], ref_x+zoom_size[1]]
            # ybound = [ref_y+zoom_size[2], ref_y+zoom_size[3]]
            # if zoom==True:
            #     ax.set_xlim(xbound)
            #     ax.set_ylim(ybound)

            plt.show(block=False)

            if save_img==True:
                # k+=1
                filename = str(k)
                filename = "0"*(5 - len(filename)) + filename
                plt.savefig('sim_5/img'+filename+'.png')
            

            if k<51:
                plt.pause(0.001)
            else:
                plt.pause(0.001)
                

            traj_history[0].remove()

            traj_arrow_history.remove()

            if weather_traj_lon_1[k+50]!=0:
                weather_arrow_history_1.remove()
                weather_traj_history_1.remove()
            
            if weather_traj_lon_2[k+50]!=0:
                weather_arrow_history_2.remove()

            # if weather_cell_ref!=None:
            
            weather_traj_history_2.remove()

            k+=1






                





    def draw_routing_trajectory(self,fig,axs):


        test_grid_left = Grid(30, 50)
        axes_left = test_grid_left.draw(fig,axs[1])

        test_grid_right = Grid(30, 50)
        axes_right = test_grid_right.draw(fig,axs[2])

        failed_grid = dict()
        trajectory_dict = dict()
        cell_dict = dict()
        max_prob = 0

        c_3 = []

        k = 0
        for i in range(1,30):

            trajectory_file = '/home/shong/mtk/workspaces/primary/acdc/source/models/aircraft_routing/two_segments_3/' + 'seg_1_policy_sim_' + str(i)  + '.json'
            trajectory_txt = open(trajectory_file)
            trajectory_data = trajectory_txt.read()
            trajectory = json.loads(trajectory_data)


            cell_trajectory_file = '/home/shong/mtk/workspaces/primary/acdc/source/models/aircraft_routing/two_segments_3/' + 'cell_sim_' + str(i)  + '.json'
            cell_trajectory_txt = open(cell_trajectory_file)
            cell_trajectory_data = cell_trajectory_txt.read()
            cell_trajectory = json.loads(cell_trajectory_data)


            estimated_cell_file = '/home/shong/mtk/workspaces/primary/acdc/source/models/aircraft_routing/two_segments_3/' + 'estimated_cell_' + str(i)  + '.json'
            estimated_cell_txt = open(estimated_cell_file)
            estimated_cell_data = estimated_cell_txt.read()
            estimated_cell = json.loads(estimated_cell_data)

            current_path = []
            current_path_weight = []


            axes_left.plot(.5,15.5, marker='^', markersize=10, color='black')
            axes_left.plot(29.5,15.5, marker='^', markersize=10, color='black')
            axes_left.plot([.5,29.5],[15.5,15.5], color='black', linewidth=0.5)

            axes_left.text(-3.5,16.5, "Waypoint 1", fontsize=8)
            axes_left.text(26.5,16.5, "Waypoint 2", fontsize=8)


            axes_right.plot(.5,15.5, marker='^', markersize=10, color='black')
            axes_right.plot(29.5,15.5, marker='^', markersize=10, color='black')
            axes_right.plot([.5,29.5],[15.5,15.5], color='black', linewidth=0.5)

            axes_right.text(-3.5,16.5, "Waypoint 2", fontsize=8)
            axes_right.text(26.5,16.5, "Waypoint 3", fontsize=8)

            t = 0
            for transition in trajectory:



                if not transition[0]:
                    continue
                else:
                    if transition[1]=="failed":
                        if tuple(transition[0][1]) in failed_grid:
                            failed_grid[tuple(transition[0][1])] += 1
                        else:
                            failed_grid[tuple(transition[0][1])] = 1

                        test_grid_left.draw_cell_circle(axes_left, transition[0][1], failed_grid[tuple(transition[0][1])]/2, color="red")

                        c_1 = []
                        c_2 = []

                    else:
                        x = transition[0][1]
                        y = transition[1][1]

                        if (tuple(x),tuple(y)) in trajectory_dict:
                            trajectory_dict[(tuple(x),tuple(y))] += 1
                        else:
                            trajectory_dict[(tuple(x),tuple(y))] = 1

                        test_grid_left.draw_path_2(axes_left,[x,y],lw=min(trajectory_dict[(tuple(x),tuple(y))]/2,5) , color='red')
                        current_path.append([x,y])
                        current_path_weight.append(trajectory_dict[(tuple(x),tuple(y))]/2)


                        if transition[1][3]==None:
                            c_1 = []
                        else:
                            c_1 = test_grid_left.draw_cell(axes_left,transition[1][3])



                        ## draw cell for next segment
                        if len(cell_trajectory)>t:
                            if cell_trajectory[t]!=None:
                                c_2 = test_grid_right.draw_cell(axes_right,cell_trajectory[t])
                            else:
                                c_2 = []
                        else:
                            c_2 = []

                t += 1




                plt.show(block=False)

                if transition[1]=="failed":
                    for i in range(5):
                        filename = str(k)
                        filename = "0"*(5 - len(filename)) + filename
                        plt.savefig('weather_sim/img'+filename+'.png')
                        k+=1

                else:

                    filename = str(k)
                    filename = "0"*(5 - len(filename)) + filename
                    plt.savefig('weather_sim/img'+filename+'.png')
                    k+=1


                # if transition[1]=="failed":
                #     plt.pause(.5)
                # else:
                #     plt.pause(.1)

                for c_ in c_1:
                    c_.remove()

                for c_ in c_2:
                    c_.remove()



            if len(c_3)>0:
                for c_ in c_3:
                    c_.remove()

            c_3 = test_grid_right.draw_cell(axes_right,estimated_cell)



            for path, weight in zip(current_path,current_path_weight):
                test_grid_left.draw_path_2(axes_left,path,lw=min(weight,5), color='b') 



        if len(c_3)>0:
                for c_ in c_3:
                    c_.remove()

        for i in range(30):

            trajectory_file = '/home/shong/mtk/workspaces/primary/acdc/source/models/aircraft_routing/two_segments_3/' + 'seg_2_policy_sim_' + str(i)  + '.json'
            trajectory_txt = open(trajectory_file)
            trajectory_data = trajectory_txt.read()
            trajectory = json.loads(trajectory_data)


            current_path = []
            current_path_weight = []


            axes_right.plot(.5,15.5, marker='^', markersize=10, color='black')
            axes_right.plot(29.5,15.5, marker='^', markersize=10, color='black')
            axes_right.plot(59.5,15.5, marker='^', markersize=10, color='black')
            axes_right.plot([.5,59.5],[15.5,15.5], color='black', linewidth=0.5)

            axes_right.text(-3.5,16.5, "Waypoint 1", fontsize=12)
            axes_right.text(26.5,16.5, "Waypoint 2", fontsize=12)
            axes_right.text(56.5,16.5, "Waypoint 3", fontsize=12)

            t = 0
            for transition in trajectory:
                if not transition[0]:
                    continue
                else:
                    if transition[1]=="failed":
                        if (transition[0][1][0]+30, transition[0][1][1]) in failed_grid:
                            failed_grid[(transition[0][1][0]+30, transition[0][1][1])] += 1
                        else:
                            failed_grid[(transition[0][1][0]+30, transition[0][1][1])] = 1

                        test_grid_right.draw_cell_circle(axes_right, [transition[0][1][0]+30,transition[0][1][1]], failed_grid[(transition[0][1][0]+30, transition[0][1][1])]/2, color="red")

                        c_1 = []

                    else:
                        x = [transition[0][1][0]+30,transition[0][1][1]]
                        y = [transition[1][1][0]+30,transition[1][1][1]]

                        if (tuple(x),tuple(y)) in trajectory_dict:
                            trajectory_dict[(tuple(x),tuple(y))] += 1
                        else:
                            trajectory_dict[(tuple(x),tuple(y))] = 1

                        test_grid_right.draw_path_2(axes_right,[x,y],lw=min(trajectory_dict[(tuple(x),tuple(y))]/2,5) , color='red')
                        current_path.append([x,y])
                        current_path_weight.append(trajectory_dict[(tuple(x),tuple(y))]/2)


                        if transition[1][3]==None:
                            c_1 = []
                        else:
                            c_1 = test_grid_right.draw_cell(axes_right,transition[1][3], offset=[30,0])

                t += 1


                plt.show(block=False)

                if transition[1]=="failed":
                    for i in range(5):
                        filename = str(k)
                        filename = "0"*(5 - len(filename)) + filename
                        plt.savefig('weather_sim/img'+filename+'.png')
                        k+=1

                else:

                    filename = str(k)
                    filename = "0"*(5 - len(filename)) + filename
                    plt.savefig('weather_sim/img'+filename+'.png')
                    k+=1

                for c_ in c_1:
                    c_.remove()

            for path, weight in zip(current_path,current_path_weight):
                test_grid_right.draw_path_2(axes_right,path,lw=min(weight,5), color='b') 




    def draw_single_segment_policy_static(self,fig,ax,orientation=None):

        test_grid = Grid(30, 50)
        axes = test_grid.draw(fig,ax)

        failed_grid = dict()
        trajectory_dict = dict()
        cell_dict = dict()
        max_prob = 0

        # plt.text(-2.5,16.5, "Waypoint 1", fontsize=12)
        # plt.text(27.5,16.5, "Waypoint 2", fontsize=12)

        for i in range(50):

            trajectory_file = '/home/shong/mtk/workspaces/primary/acdc/source/models/aircraft_routing/single_segment_high_risk/' + 'policy_test_' + str(i)  + '.json'

            trajectory_txt = open(trajectory_file)
            trajectory_data = trajectory_txt.read()
            trajectory = json.loads(trajectory_data)

            for transition in trajectory:
                if transition[0]:
                    if transition[1]=="failed":
                        if tuple(transition[0][1]) in failed_grid:
                            failed_grid[tuple(transition[0][1])] += 1
                        else:
                            failed_grid[tuple(transition[0][1])] = 1
                    else:
                        x = transition[0][1]
                        y = transition[1][1]

                        if (tuple(x),tuple(y)) in trajectory_dict:
                            trajectory_dict[(tuple(x),tuple(y))] += 1
                        else:
                            trajectory_dict[(tuple(x),tuple(y))] = 1


                        cell = transition[1][3]
                        if cell:
                            for cell_pos,prob in cell:
                                if tuple(cell_pos) in cell_dict:
                                    cell_dict[tuple(cell_pos)] += prob
                                else:
                                    cell_dict[tuple(cell_pos)] = prob

                                if cell_dict[tuple(cell_pos)] > max_prob:
                                    max_prob = cell_dict[tuple(cell_pos)]

        for grid, freq in cell_dict.items():
            vert = [test_grid.cell_verts(grid[0], grid[1], orientation)]
            collection_rec = PolyCollection(vert, facecolors='r', alpha=freq/max_prob)
            axes.add_collection(collection_rec)


        max_lw = 0
        for traj, freq in trajectory_dict.items():
            if freq/10 > max_lw:
                max_lw = freq/10

        for traj, freq in trajectory_dict.items():
            test_grid.draw_path_2(axes,[traj[0], traj[1]],lw=min(max_lw, freq/3), color='b',orientation=orientation)

        for grid, freq in failed_grid.items():
            test_grid.draw_cell_circle(axes, grid, freq*5000, color="red", orientation=orientation)

        plt.show()
                



    def draw_single_segment_policy_dynamic(self,fig,ax,show=True,save_img=False,orientation=None):


        test_grid = Grid(30, 50)
        axes = test_grid.draw(fig,ax)

        failed_grid = dict()
        trajectory_dict = dict()
        cell_dict = dict()
        max_prob = 0

        k = 0
        for i in range(1,30):

            trajectory_file = '/home/shong/mtk/workspaces/primary/acdc/source/models/aircraft_routing/single_segment_medium_risk/' + 'policy_test_' + str(i)  + '.json'
            trajectory_txt = open(trajectory_file)
            trajectory_data = trajectory_txt.read()
            trajectory = json.loads(trajectory_data)

            current_path = []
            current_path_weight = []

            current_path_plot = []
            text = None

            t = 0
            for transition in trajectory:
                if not transition[0]:
                    continue
                else:
                    if transition[1]=="failed":
                        if tuple(transition[0][1]) in failed_grid:
                            failed_grid[tuple(transition[0][1])] += 1
                        else:
                            failed_grid[tuple(transition[0][1])] = 1

                        test_grid.draw_cell_circle(axes, transition[0][1], failed_grid[tuple(transition[0][1])]*5000, color="red",orientation=orientation)

                        if transition[0][3]==None:
                            cell_plot = []
                        else:
                            cell_plot = test_grid.draw_cell(axes,transition[0][3])
                            
                    else:
                        x = transition[0][1]
                        y = transition[1][1]

                        if (tuple(x),tuple(y)) in trajectory_dict:
                            trajectory_dict[(tuple(x),tuple(y))] += 1
                        else:
                            trajectory_dict[(tuple(x),tuple(y))] = 1

                        current_path_plot.append(test_grid.draw_path_2(axes,[x,y],lw=3, color='red',orientation=orientation))
                        current_path.append([x,y])
                        current_path_weight.append(trajectory_dict[(tuple(x),tuple(y))])

                        if transition[1][3]==None:
                            cell_plot = []
                        else:
                            cell_plot = test_grid.draw_cell(axes,transition[1][3], orientation=orientation)


                t += 1


                plt.show(block=False)

                if transition[1]=="failed":
                    new_x, new_y = test_grid.translate(transition[0][1][0]-3,transition[0][1][1]+2,\
                                                       orientation=orientation,point=True)
                    text = plt.text(new_x,new_y, "Encountered Cell!", fontsize=15, color='red')

                if show==True:
                    if transition[1]=="failed":
                        plt.pause(0.5)
                    else:
                        plt.pause(0.01)

                if save_img==True:
                    if transition[1]=="failed":
                        for i in range(100):
                            filename = str(k)
                            filename = "0"*(5 - len(filename)) + filename
                            plt.savefig('figures/single_segment_medium_risk/img'+filename+'.png')
                            k+=1

                    else:

                        filename = str(k)
                        filename = "0"*(5 - len(filename)) + filename
                        plt.savefig('figures/single_segment_medium_risk/img'+filename+'.png')
                        k+=1

                for c in cell_plot:
                    c.remove()

            if text:
                text.remove()

            for path, weight in zip(current_path,current_path_weight):
                test_grid.draw_path_2(axes,path,lw=min(weight,5), color='b',orientation=orientation) 

            for p in current_path_plot:
                p.remove()






    def draw_two_segments_policies_static(self,fig,axs,orientation=None):

        test_grid_left = Grid(30, 50)
        axes_left = test_grid_left.draw(fig,axs[0])

        test_grid_right = Grid(30, 50)
        axes_right = test_grid_right.draw(fig,axs[1])

        failed_grid = dict()
        trajectory_dict = dict()
        cell_dict = dict()
        max_prob = 0

        # plt.text(-2.5,16.5, "Waypoint 1", fontsize=12)
        # plt.text(27.5,16.5, "Waypoint 2", fontsize=12)

        for i in range(30):

            trajectory_file = '/home/shong/mtk/workspaces/primary/acdc/source/models/aircraft_routing/two_segments_3/' + 'seg_1_policy_sim_' + str(i)  + '.json'

            trajectory_txt = open(trajectory_file)
            trajectory_data = trajectory_txt.read()
            trajectory = json.loads(trajectory_data)

            for transition in trajectory:
                if transition[0]:
                    if transition[1]=="failed":
                        if tuple(transition[0][1]) in failed_grid:
                            failed_grid[tuple(transition[0][1])] += 1
                        else:
                            failed_grid[tuple(transition[0][1])] = 1
                    else:
                        x = transition[0][1]
                        y = transition[1][1]

                        if (tuple(x),tuple(y)) in trajectory_dict:
                            trajectory_dict[(tuple(x),tuple(y))] += 1
                        else:
                            trajectory_dict[(tuple(x),tuple(y))] = 1


                        cell = transition[1][3]
                        if cell:
                            for cell_pos,prob in cell:
                                if tuple(cell_pos) in cell_dict:
                                    cell_dict[tuple(cell_pos)] += prob
                                else:
                                    cell_dict[tuple(cell_pos)] = prob

                                if cell_dict[tuple(cell_pos)] > max_prob:
                                    max_prob = cell_dict[tuple(cell_pos)]

        # for grid, freq in cell_dict.items():
        #     vert = [test_grid.cell_verts(grid[0], grid[1], orientation)]
        #     collection_rec = PolyCollection(vert, facecolors='r', alpha=freq/max_prob)
        #     axes_left.add_collection(collection_rec)

        max_lw = 0
        for traj, freq in trajectory_dict.items():
            if freq/10 > max_lw:
                max_lw = freq/10

        for traj, freq in trajectory_dict.items():
            test_grid_left.draw_path_2(axes_left,[traj[0], traj[1]],lw=min(max_lw, freq/3), color='b',orientation=orientation[0])

        for grid, freq in failed_grid.items():
            test_grid_left.draw_cell_circle(axes_left, grid, freq*5000, color="red", orientation=orientation[0])



        # second segment

        failed_grid = dict()
        trajectory_dict = dict()
        cell_dict = dict()
        max_prob = 0

        # plt.text(-2.5,16.5, "Waypoint 1", fontsize=12)
        # plt.text(27.5,16.5, "Waypoint 2", fontsize=12)
        

        for i in range(30):

            trajectory_file = '/home/shong/mtk/workspaces/primary/acdc/source/models/aircraft_routing/two_segments_3/' + 'seg_2_policy_sim_' + str(i)  + '.json'

            trajectory_txt = open(trajectory_file)
            trajectory_data = trajectory_txt.read()
            trajectory = json.loads(trajectory_data)

            for transition in trajectory:
                if transition[0]:
                    if transition[1]=="failed":
                        if tuple(transition[0][1]) in failed_grid:
                            failed_grid[tuple(transition[0][1])] += 1
                        else:
                            failed_grid[tuple(transition[0][1])] = 1
                    else:
                        x = transition[0][1]
                        y = transition[1][1]

                        if (tuple(x),tuple(y)) in trajectory_dict:
                            trajectory_dict[(tuple(x),tuple(y))] += 1
                        else:
                            trajectory_dict[(tuple(x),tuple(y))] = 1


                        cell = transition[1][3]
                        if cell:
                            for cell_pos,prob in cell:
                                if tuple(cell_pos) in cell_dict:
                                    cell_dict[tuple(cell_pos)] += prob
                                else:
                                    cell_dict[tuple(cell_pos)] = prob

                                if cell_dict[tuple(cell_pos)] > max_prob:
                                    max_prob = cell_dict[tuple(cell_pos)]

        # for grid, freq in cell_dict.items():
        #     vert = [test_grid.cell_verts(grid[0], grid[1], orientation)]
        #     collection_rec = PolyCollection(vert, facecolors='r', alpha=freq/max_prob)
        #     axes_right.add_collection(collection_rec)

        max_lw = 0
        for traj, freq in trajectory_dict.items():
            if freq/10 > max_lw:
                max_lw = freq/10

        for traj, freq in trajectory_dict.items():
            test_grid_right.draw_path_2(axes_right,[traj[0], traj[1]],lw=min(max_lw, freq/3), color='b',orientation=orientation[1])

        for grid, freq in failed_grid.items():
            test_grid_right.draw_cell_circle(axes_right, grid, freq*5000, color="red", orientation=orientation[1])

        plt.show()



    # def draw_two_segments_policies_static(self,fig,axs,show=True,save_img=False,orientation=None):

    #     test_grid_left = Grid(30, 50)
    #     axes_left = test_grid_left.draw(fig,axs[0])

    #     test_grid_right = Grid(30, 50)
    #     axes_right = test_grid_right.draw(fig,axs[1])

    #     failed_grid = dict()
    #     trajectory_dict = dict()
    #     cell_dict = dict()
    #     max_prob = 0

    #     c_3 = []

    #     k = 0
    #     for i in range(1,30):

    #         trajectory_file = '/home/shong/mtk/workspaces/primary/acdc/source/models/aircraft_routing/two_segments_3/' + 'seg_1_policy_sim_' + str(i)  + '.json'
    #         trajectory_txt = open(trajectory_file)
    #         trajectory_data = trajectory_txt.read()
    #         trajectory = json.loads(trajectory_data)

    #         current_path = []
    #         current_path_weight = []
    #         current_path_plot = []

    #         t = 0
    #         for transition in trajectory:

    #             if not transition[0]:
    #                 continue
    #             else:
    #                 if transition[1]=="failed":
    #                     if tuple(transition[0][1]) in failed_grid:
    #                         failed_grid[tuple(transition[0][1])] += 1
    #                     else:
    #                         failed_grid[tuple(transition[0][1])] = 1

    #                     test_grid_left.draw_cell_circle(axes_left, transition[0][1], failed_grid[tuple(transition[0][1])]*5000, color="red",orientation=orientation[0])

    #                 else:
    #                     x = transition[0][1]
    #                     y = transition[1][1]

    #                     if (tuple(x),tuple(y)) in trajectory_dict:
    #                         trajectory_dict[(tuple(x),tuple(y))] += 1
    #                     else:
    #                         trajectory_dict[(tuple(x),tuple(y))] = 1

    #                     # test_grid_left.draw_path_2(axes_left,[x,y],lw=min(trajectory_dict[(tuple(x),tuple(y))]/2,5) , color='red')
    #                     current_path_plot.append(test_grid_left.draw_path_2(axes_left,[x,y],lw=2 , color='red',orientation=orientation[0]))
    #                     current_path.append([x,y])
    #                     current_path_weight.append(trajectory_dict[(tuple(x),tuple(y))])

    #             t += 1

    #             plt.show(block=False)

    #             if save_img==True:
    #                 if transition[1]=="failed":
    #                     for i in range(20):
    #                         filename = str(k)
    #                         filename = "0"*(5 - len(filename)) + filename
    #                         plt.savefig('figures/two_segments_static_alloc_1/img'+filename+'.png')
    #                         k+=1

    #                 else:

    #                     filename = str(k)
    #                     filename = "0"*(5 - len(filename)) + filename
    #                     plt.savefig('figures/two_segments_static_alloc_1/img'+filename+'.png')
    #                     k+=1


    #             if show==True:
    #                 if transition[1]=="failed":
    #                     plt.pause(0.5)
    #                 else:
    #                     plt.pause(0.01)



    #             for c_ in c_1:
    #                 c_.remove()

    #             for c_ in c_2:
    #                 c_.remove()



    #         if len(c_3)>0:
    #             for c_ in c_3:
    #                 c_.remove()

    #         c_3 = test_grid_right.draw_cell(axes_right,estimated_cell,orientation=orientation[1])


    #         for p in current_path_plot:
    #             p.remove()

    #         for path, weight in zip(current_path,current_path_weight):
    #             test_grid_left.draw_path_2(axes_left,path,lw=min(weight,5), color='b',orientation=orientation[0]) 



    #     if len(c_3)>0:
    #             for c_ in c_3:
    #                 c_.remove()

    #     for i in range(20):

    #         trajectory_file = '/home/shong/mtk/workspaces/primary/acdc/source/models/aircraft_routing/two_segments_3/' + 'seg_2_policy_sim_' + str(i)  + '.json'
    #         trajectory_txt = open(trajectory_file)
    #         trajectory_data = trajectory_txt.read()
    #         trajectory = json.loads(trajectory_data)


    #         current_path = []
    #         current_path_weight = []

    #         current_path_plot = []

    #         t = 0
    #         for transition in trajectory:
    #             if not transition[0]:
    #                 continue
    #             else:
    #                 if transition[1]=="failed":
    #                     if (transition[0][1][0]+30, transition[0][1][1]) in failed_grid:
    #                         failed_grid[(transition[0][1][0], transition[0][1][1])] += 1
    #                     else:
    #                         failed_grid[(transition[0][1][0], transition[0][1][1])] = 1

    #                     test_grid_right.draw_cell_circle(axes_right, [transition[0][1][0],transition[0][1][1]], failed_grid[(transition[0][1][0], transition[0][1][1])]*5000, color="red",orientation=orientation[1])

    #                     c_1 = []

    #                 else:
    #                     x = [transition[0][1][0],transition[0][1][1]]
    #                     y = [transition[1][1][0],transition[1][1][1]]

    #                     if (tuple(x),tuple(y)) in trajectory_dict:
    #                         trajectory_dict[(tuple(x),tuple(y))] += 1
    #                     else:
    #                         trajectory_dict[(tuple(x),tuple(y))] = 1

    #                     # test_grid_right.draw_path_2(axes_right,[x,y],lw=min(trajectory_dict[(tuple(x),tuple(y))]/2,5) , color='red')
    #                     current_path_plot.append(test_grid_right.draw_path_2(axes_right,[x,y],lw=2 , color='red',orientation=orientation[1]))
    #                     current_path.append([x,y])
    #                     current_path_weight.append(trajectory_dict[(tuple(x),tuple(y))])


    #                     if transition[1][3]==None:
    #                         c_1 = []
    #                     else:
    #                         c_1 = test_grid_right.draw_cell(axes_right,transition[1][3],orientation=orientation[1])

    #             t += 1


    #             plt.show(block=False)


    #             if save_img==True:
    #                 if transition[1]=="failed":
    #                     for i in range(5):
    #                         filename = str(k)
    #                         filename = "0"*(5 - len(filename)) + filename
    #                         plt.savefig('figures/two_segments_w_estimation/img'+filename+'.png')
    #                         k+=1

    #                 else:

    #                     filename = str(k)
    #                     filename = "0"*(5 - len(filename)) + filename
    #                     plt.savefig('figures/two_segments_w_estimation/img'+filename+'.png')
    #                     k+=1

    #             if show==True:
    #                 if transition[1]=="failed":
    #                     plt.pause(0.5)
    #                 else:
    #                     plt.pause(0.01)

    #             for c_ in c_1:
    #                 c_.remove()

    #         for p in current_path_plot:
    #             p.remove()

    #         for path, weight in zip(current_path,current_path_weight):
    #             test_grid_right.draw_path_2(axes_right,path,lw=min(weight,5), color='b') 






                
    def draw_two_segments_policies_dynamic_w_estimation(self,fig,axs,show=True,save_img=False,orientation=None):

        test_grid_left = Grid(30, 50)
        axes_left = test_grid_left.draw(fig,axs[0])

        test_grid_right = Grid(30, 50)
        axes_right = test_grid_right.draw(fig,axs[1])

        failed_grid = dict()
        trajectory_dict = dict()
        cell_dict = dict()
        max_prob = 0

        c_3 = []

        k = 0
        for i in range(1,20):

            trajectory_file = '/home/shong/mtk/workspaces/primary/acdc/source/models/aircraft_routing/two_segments_3/' + 'seg_1_policy_sim_' + str(i)  + '.json'
            trajectory_txt = open(trajectory_file)
            trajectory_data = trajectory_txt.read()
            trajectory = json.loads(trajectory_data)


            cell_trajectory_file = '/home/shong/mtk/workspaces/primary/acdc/source/models/aircraft_routing/two_segments_3/' + 'cell_sim_' + str(i)  + '.json'
            cell_trajectory_txt = open(cell_trajectory_file)
            cell_trajectory_data = cell_trajectory_txt.read()
            cell_trajectory = json.loads(cell_trajectory_data)


            estimated_cell_file = '/home/shong/mtk/workspaces/primary/acdc/source/models/aircraft_routing/two_segments_3/' + 'estimated_cell_' + str(i)  + '.json'
            estimated_cell_txt = open(estimated_cell_file)
            estimated_cell_data = estimated_cell_txt.read()
            estimated_cell = json.loads(estimated_cell_data)

            current_path = []
            current_path_weight = []

            current_path_plot = []

            t = 0
            for transition in trajectory:

                if not transition[0]:
                    continue
                else:
                    if transition[1]=="failed":
                        if tuple(transition[0][1]) in failed_grid:
                            failed_grid[tuple(transition[0][1])] += 1
                        else:
                            failed_grid[tuple(transition[0][1])] = 1

                        test_grid_left.draw_cell_circle(axes_left, transition[0][1], failed_grid[tuple(transition[0][1])]*5000, color="red",orientation=orientation[0])

                        c_1 = []
                        c_2 = []

                    else:
                        x = transition[0][1]
                        y = transition[1][1]

                        if (tuple(x),tuple(y)) in trajectory_dict:
                            trajectory_dict[(tuple(x),tuple(y))] += 1
                        else:
                            trajectory_dict[(tuple(x),tuple(y))] = 1

                        # test_grid_left.draw_path_2(axes_left,[x,y],lw=min(trajectory_dict[(tuple(x),tuple(y))]/2,5) , color='red')
                        current_path_plot.append(test_grid_left.draw_path_2(axes_left,[x,y],lw=2 , color='red',orientation=orientation[0]))
                        current_path.append([x,y])
                        current_path_weight.append(trajectory_dict[(tuple(x),tuple(y))])


                        if transition[1][3]==None:
                            c_1 = []
                        else:
                            c_1 = test_grid_left.draw_cell(axes_left,transition[1][3],orientation=orientation[0])



                        ## draw cell for next segment
                        if len(cell_trajectory)>t:
                            if cell_trajectory[t]!=None:
                                c_2 = test_grid_right.draw_cell(axes_right,cell_trajectory[t],orientation=orientation[1])
                            else:
                                c_2 = []
                        else:
                            c_2 = []

                t += 1




                plt.show(block=False)

                if save_img==True:
                    if transition[1]=="failed":
                        for i in range(20):
                            filename = str(k)
                            filename = "0"*(5 - len(filename)) + filename
                            plt.savefig('figures/two_segments_w_estimation/img'+filename+'.png')
                            k+=1

                    else:

                        filename = str(k)
                        filename = "0"*(5 - len(filename)) + filename
                        plt.savefig('figures/two_segments_w_estimation/img'+filename+'.png')
                        k+=1


                if show==True:
                    if transition[1]=="failed":
                        plt.pause(0.5)
                    else:
                        plt.pause(0.01)



                for c_ in c_1:
                    c_.remove()

                for c_ in c_2:
                    c_.remove()



            if len(c_3)>0:
                for c_ in c_3:
                    c_.remove()

            c_3 = test_grid_right.draw_cell(axes_right,estimated_cell,orientation=orientation[1])


            for p in current_path_plot:
                p.remove()

            for path, weight in zip(current_path,current_path_weight):
                test_grid_left.draw_path_2(axes_left,path,lw=min(weight,5), color='b',orientation=orientation[0]) 



        if len(c_3)>0:
                for c_ in c_3:
                    c_.remove()

        for i in range(20):

            trajectory_file = '/home/shong/mtk/workspaces/primary/acdc/source/models/aircraft_routing/two_segments_3/' + 'seg_2_policy_sim_' + str(i)  + '.json'
            trajectory_txt = open(trajectory_file)
            trajectory_data = trajectory_txt.read()
            trajectory = json.loads(trajectory_data)


            current_path = []
            current_path_weight = []

            current_path_plot = []

            t = 0
            for transition in trajectory:
                if not transition[0]:
                    continue
                else:
                    if transition[1]=="failed":
                        if (transition[0][1][0]+30, transition[0][1][1]) in failed_grid:
                            failed_grid[(transition[0][1][0], transition[0][1][1])] += 1
                        else:
                            failed_grid[(transition[0][1][0], transition[0][1][1])] = 1

                        test_grid_right.draw_cell_circle(axes_right, [transition[0][1][0],transition[0][1][1]], failed_grid[(transition[0][1][0], transition[0][1][1])]*5000, color="red",orientation=orientation[1])

                        c_1 = []

                    else:
                        x = [transition[0][1][0],transition[0][1][1]]
                        y = [transition[1][1][0],transition[1][1][1]]

                        if (tuple(x),tuple(y)) in trajectory_dict:
                            trajectory_dict[(tuple(x),tuple(y))] += 1
                        else:
                            trajectory_dict[(tuple(x),tuple(y))] = 1

                        # test_grid_right.draw_path_2(axes_right,[x,y],lw=min(trajectory_dict[(tuple(x),tuple(y))]/2,5) , color='red')
                        current_path_plot.append(test_grid_right.draw_path_2(axes_right,[x,y],lw=2 , color='red',orientation=orientation[1]))
                        current_path.append([x,y])
                        current_path_weight.append(trajectory_dict[(tuple(x),tuple(y))])


                        if transition[1][3]==None:
                            c_1 = []
                        else:
                            c_1 = test_grid_right.draw_cell(axes_right,transition[1][3],orientation=orientation[1])

                t += 1


                plt.show(block=False)


                if save_img==True:
                    if transition[1]=="failed":
                        for i in range(5):
                            filename = str(k)
                            filename = "0"*(5 - len(filename)) + filename
                            plt.savefig('figures/two_segments_w_estimation/img'+filename+'.png')
                            k+=1

                    else:

                        filename = str(k)
                        filename = "0"*(5 - len(filename)) + filename
                        plt.savefig('figures/two_segments_w_estimation/img'+filename+'.png')
                        k+=1

                if show==True:
                    if transition[1]=="failed":
                        plt.pause(0.5)
                    else:
                        plt.pause(0.01)

                for c_ in c_1:
                    c_.remove()

            for p in current_path_plot:
                p.remove()

            for path, weight in zip(current_path,current_path_weight):
                test_grid_right.draw_path_2(axes_right,path,lw=min(weight,5), color='b', orientation=orientation[1]) 






    def draw_two_segments_policies_dynamic_wo_estimation(self,fig,axs,show=True,save_img=False,orientation=None):

        test_grid_left = Grid(30, 50)
        axes_left = test_grid_left.draw(fig,axs[0])

        test_grid_right = Grid(30, 50)
        axes_right = test_grid_right.draw(fig,axs[1])

        failed_grid = dict()
        trajectory_dict = dict()
        cell_dict = dict()
        max_prob = 0

        k = 0
        for i in range(1,20):

            trajectory_file = '/home/shong/mtk/workspaces/primary/acdc/source/models/aircraft_routing/two_segments_3/' + 'seg_1_policy_sim_' + str(i)  + '.json'
            trajectory_txt = open(trajectory_file)
            trajectory_data = trajectory_txt.read()
            trajectory = json.loads(trajectory_data)


            cell_trajectory_file = '/home/shong/mtk/workspaces/primary/acdc/source/models/aircraft_routing/two_segments_3/' + 'cell_sim_' + str(i)  + '.json'
            cell_trajectory_txt = open(cell_trajectory_file)
            cell_trajectory_data = cell_trajectory_txt.read()
            cell_trajectory = json.loads(cell_trajectory_data)

            current_path = []
            current_path_weight = []
            current_path_plot = []

            t = 0
            for transition in trajectory:

                if not transition[0]:
                    continue
                else:
                    if transition[1]=="failed":
                        if tuple(transition[0][1]) in failed_grid:
                            failed_grid[tuple(transition[0][1])] += 1
                        else:
                            failed_grid[tuple(transition[0][1])] = 1

                        test_grid_left.draw_cell_circle(axes_left, transition[0][1], failed_grid[tuple(transition[0][1])]*5000, color="red",orientation=orientation[0])

                        c_1 = []
                        c_2 = []

                    else:
                        x = transition[0][1]
                        y = transition[1][1]

                        if (tuple(x),tuple(y)) in trajectory_dict:
                            trajectory_dict[(tuple(x),tuple(y))] += 1
                        else:
                            trajectory_dict[(tuple(x),tuple(y))] = 1

                        # test_grid_left.draw_path_2(axes_left,[x,y],lw=min(trajectory_dict[(tuple(x),tuple(y))]/2,5) , color='red')
                        current_path_plot.append(test_grid_left.draw_path_2(axes_left,[x,y],lw=2 , color='red',orientation=orientation[0]))
                        current_path.append([x,y])
                        current_path_weight.append(trajectory_dict[(tuple(x),tuple(y))])


                        if transition[1][3]==None:
                            c_1 = []
                        else:
                            c_1 = test_grid_left.draw_cell(axes_left,transition[1][3],orientation=orientation[0])



                        ## draw cell for next segment
                        if len(cell_trajectory)>t:
                            if cell_trajectory[t]!=None:
                                c_2 = test_grid_right.draw_cell(axes_right,cell_trajectory[t],orientation=orientation[1])
                            else:
                                c_2 = []
                        else:
                            c_2 = []

                t += 1


                plt.show(block=False)

                if save_img==True:
                    if transition[1]=="failed":
                        for i in range(20):
                            filename = str(k)
                            filename = "0"*(5 - len(filename)) + filename
                            plt.savefig('figures/two_segments_wo_estimation/img'+filename+'.png')
                            k+=1

                    else:

                        filename = str(k)
                        filename = "0"*(5 - len(filename)) + filename
                        plt.savefig('figures/two_segments_wo_estimation/img'+filename+'.png')
                        k+=1


                if show==True:
                    if transition[1]=="failed":
                        plt.pause(0.5)
                    else:
                        plt.pause(0.01)



                for c_ in c_1:
                    c_.remove()

                for c_ in c_2:
                    c_.remove()



            for p in current_path_plot:
                p.remove()

            for path, weight in zip(current_path,current_path_weight):
                test_grid_left.draw_path_2(axes_left,path,lw=min(weight,5), color='b',orientation=orientation[0]) 









    def draw_hierarchical_dynamic(self,fig,ax,show=True,save_img=False,orientation=None):


        global global_k
        global_k = 0

        global global_save_img
        global_save_img = True


        trajectory_file = '/home/shong/mtk/workspaces/primary/acdc/source/models/aircraft_routing/hierarchical/sol2/wp1-wp2_' + 'policy_sim_1.json'
        cell_trajectory_file = '/home/shong/mtk/workspaces/primary/acdc/source/models/aircraft_routing/hierarchical/sol2/wp1-wp2_' + 'cell_sim_1.json'
        plots = self.simulate_initial_part(fig,ax,orientation,trajectory_file,cell_trajectory_file)


        

        # ## sol 1
        # plots = []
        # trajectory_file_head = '/home/shong/mtk/workspaces/primary/acdc/source/models/aircraft_routing/hierarchical/sol1/wp1-wp4_' + 'policy_sim_'
        # current_orientation = orientation[0]
        # plots.extend(self.draw_single_segment_policy_for_hierarchical(fig,ax,current_orientation,trajectory_file_head))

        # # point = plt.ginput(3)
        # # print(point)
        # # plt.pause(20000)

        # plt.pause(2)

        # for p in plots:
        #     if p!= None:
        #         p.remove()

            
        
        # ## sol 2
        # plots = []
        # trajectory_file_head = '/home/shong/mtk/workspaces/primary/acdc/source/models/aircraft_routing/hierarchical/sol2/wp1-wp2_' + 'policy_sim_'
        # current_orientation = orientation[0]
        # plots.extend(self.draw_single_segment_policy_for_hierarchical(fig,ax,current_orientation,trajectory_file_head))

        # trajectory_file_head = '/home/shong/mtk/workspaces/primary/acdc/source/models/aircraft_routing/hierarchical/sol2/wp2-wp3_' + 'policy_sim_'
        # current_orientation = orientation[1]
        # plots.extend(self.draw_single_segment_policy_for_hierarchical(fig,ax,current_orientation,trajectory_file_head))

        # plt.pause(2)

        # for p in plots:
        #     if p!=None:
        #         p.remove()

        # ## sol 3
        # trajectory_file_head = '/home/shong/mtk/workspaces/primary/acdc/source/models/aircraft_routing/hierarchical/sol3/wp1-wp2_' + 'policy_sim_'
        # current_orientation = orientation[0]
        # self.draw_single_segment_policy_for_hierarchical(fig,ax,current_orientation,trajectory_file_head)


        # trajectory_file_head = '/home/shong/mtk/workspaces/primary/acdc/source/models/aircraft_routing/hierarchical/sol3/wp2-wp3_' + 'policy_sim_'
        # current_orientation = orientation[1]
        # self.draw_single_segment_policy_for_hierarchical(fig,ax,current_orientation,trajectory_file_head)


        ## simulate policy for first segment
        trajectory_file = '/home/shong/mtk/workspaces/primary/acdc/source/models/aircraft_routing/hierarchical/sol2/wp1-wp2_' + 'policy_sim_1.json'
        cell_trajectory_file = '/home/shong/mtk/workspaces/primary/acdc/source/models/aircraft_routing/hierarchical/sol2/wp1-wp2_' + 'cell_sim_1.json'
        self.simulate_policy(fig,ax,orientation,trajectory_file,cell_trajectory_file)



        for p in plots:
            if p!=None:
                p.remove()




        ## new sol for second segments
        plots = []
        trajectory_file_head = '/home/shong/mtk/workspaces/primary/acdc/source/models/aircraft_routing/hierarchical/second_segment/policy_sim_'
        current_orientation = orientation[1]
        plots.extend(self.draw_single_segment_policy_for_hierarchical(fig,ax,current_orientation,trajectory_file_head))

        text_loc = (1106451.6129032257, 816129.0322580657)
        text = plt.text(text_loc[0], text_loc[1], "Policy Updated For Next Segment.", fontsize=16)

        if global_save_img==True:
            for i in range(6):
                    filename = str(global_k)
                    filename = "0"*(5 - len(filename)) + filename
                    plt.savefig('figures/hierarchical/img'+filename+'.png')
                    global_k+=1
        
        text.remove()

        ## simulate policy for second segment
        trajectory_file = '/home/shong/mtk/workspaces/primary/acdc/source/models/aircraft_routing/hierarchical/second_segment/policy_sim_0.json'
        self.simulate_second_policy(fig,ax,orientation,trajectory_file)


        

        plt.pause(10000)



    def draw_single_segment_policy_for_hierarchical(self,fig,ax,orientation=None,trajectory_file_head=None):

        test_grid = Grid(30, 50)
        axes = test_grid.draw(fig,ax)

        failed_grid = dict()
        trajectory_dict = dict()
        cell_dict = dict()
        max_prob = 0

        for i in range(30):

            trajectory_file = trajectory_file_head + str(i)  + '.json'

            trajectory_txt = open(trajectory_file)
            trajectory_data = trajectory_txt.read()
            trajectory = json.loads(trajectory_data)

            for transition in trajectory:
                if transition[0]:
                    if transition[1]=="failed":
                        if tuple(transition[0][1]) in failed_grid:
                            failed_grid[tuple(transition[0][1])] += 1
                        else:
                            failed_grid[tuple(transition[0][1])] = 1
                    else:
                        x = transition[0][1]
                        y = transition[1][1]

                        if (tuple(x),tuple(y)) in trajectory_dict:
                            trajectory_dict[(tuple(x),tuple(y))] += 1
                        else:
                            trajectory_dict[(tuple(x),tuple(y))] = 1


                        cell = transition[1][3]
                        if cell:
                            for cell_pos,prob in cell:
                                if tuple(cell_pos) in cell_dict:
                                    cell_dict[tuple(cell_pos)] += prob
                                else:
                                    cell_dict[tuple(cell_pos)] = prob

                                if cell_dict[tuple(cell_pos)] > max_prob:
                                    max_prob = cell_dict[tuple(cell_pos)]

        # for grid, freq in cell_dict.items():
        #     vert = [test_grid.cell_verts(grid[0], grid[1], orientation)]
        #     collection_rec = PolyCollection(vert, facecolors='r', alpha=freq/max_prob)
        #     axes.add_collection(collection_rec)


        max_lw = 0
        for traj, freq in trajectory_dict.items():
            if freq/10 > max_lw:
                max_lw = freq/10


        plots=[]
        
        for traj, freq in trajectory_dict.items():
            plots.append(test_grid.draw_path_2(axes,[traj[0], traj[1]],lw=min(max_lw, freq/3), color='b',orientation=orientation))

        # for grid, freq in failed_grid.items():
        #     plots.append(test_grid.draw_cell_circle(axes, grid, freq*5000, color="red", orientation=orientation))

        plt.show(block=False)

        return plots






    def simulate_policy(self,fig,ax,orientation=None,trajectory_file=None, cell_trajectory_file=None,draw_arrow=False):

        global global_k
        global global_save_img

        test_grid = Grid(30, 50)
        axes = test_grid.draw(fig,ax)

        trajectory_txt = open(trajectory_file)
        trajectory_data = trajectory_txt.read()
        trajectory = json.loads(trajectory_data)
        
        cell_trajectory_txt = open(cell_trajectory_file)
        cell_trajectory_data = cell_trajectory_txt.read()
        cell_trajectory = json.loads(cell_trajectory_data)

        current_path = []
        current_path_weight = []

        current_path_plot = []
        text = None

        t = 0
        for transition in trajectory:
            if not transition[0]:
                continue
            else:
                x = transition[0][1]
                y = transition[1][1]

                current_path_plot.append(test_grid.draw_path_2(axes,[x,y],lw=3, color='red',\
                                                               orientation=orientation[0]))
                current_path.append([x,y])

                xx,yy = test_grid.translate([x[0],y[0]],[x[1],y[1]],orientation[0])

                traj_history = self.basemap.plot(xx[0],yy[0], 'o', markerfacecolor="None", markeredgecolor='red',\
                                                 markeredgewidth=3, markersize=10)
                dx = xx[1] - xx[0]
                dy = yy[1] - yy[0]
                traj_arrow_history = self.basemap.quiver(xx[0], yy[0], u=dx, v=dy, linewidth=1.2, \
                                                         color='red', headwidth=2, zorder=1000)
                

                if transition[1][3]==None:
                    cell_plot = []
                else:
                    cell_plot,x_center, y_center = test_grid.draw_cell(axes,transition[1][3], \
                                                                       orientation=orientation[0],return_center=True)
                    if draw_arrow==True:
                        weather_arrow_history_1 = self.basemap.quiver(x=x_center,\
                                                                      y=y_center, \
                                                                      u=50, v=50,linewidth=1.2, \
                                                                      color='black', headwidth=2)


                ## draw cell for next segment
                if len(cell_trajectory)>t:
                    if cell_trajectory[t]!=None:
                        c_2,x_center,y_center = test_grid.draw_cell(axes,cell_trajectory[t],\
                                                                    orientation=orientation[1],return_center=True)
                        if draw_arrow==True:
                            weather_arrow_history_2 = self.basemap.quiver(x=x_center,\
                                                                          y=y_center, \
                                                                          u=50, v=50,linewidth=1.2, \
                                                                          color='black', headwidth=2)
                    else:
                        c_2 = []
                else:
                    c_2 = []

            t += 1


            plt.show(block=False)

            plt.pause(0.2)


            if global_save_img==True:
                # if i in [2,6,10]:
                #     for i in range(20):
                #         filename = str(global_k)
                #         filename = "0"*(5 - len(filename)) + filename
                #         plt.savefig('figures/hierarchical/img'+filename+'.png')
                #         global_k+=1

                # else:

                filename = str(global_k)
                filename = "0"*(5 - len(filename)) + filename
                plt.savefig('figures/hierarchical/img'+filename+'.png')
                global_k+=1


            for c in cell_plot:
                c.remove()

            for c_ in c_2:
                c_.remove()

            if draw_arrow==True:
                weather_arrow_history_1.remove()
                weather_arrow_history_2.remove()


            traj_history[0].remove()
            traj_arrow_history.remove()



    def simulate_second_policy(self,fig,ax,orientation=None,trajectory_file=None):

        global global_k
        global global_save_img

        test_grid = Grid(30, 50)
        axes = test_grid.draw(fig,ax)

        trajectory_txt = open(trajectory_file)
        trajectory_data = trajectory_txt.read()
        trajectory = json.loads(trajectory_data)
        
        current_path = []
        current_path_weight = []

        current_path_plot = []
        text = None

        t = 0
        for transition in trajectory:
            if not transition[0]:
                continue
            else:
                x = transition[0][1]
                y = transition[1][1]

                current_path_plot.append(test_grid.draw_path_2(axes,[x,y],lw=3, color='red',\
                                                               orientation=orientation[1]))
                current_path.append([x,y])


                xx,yy = test_grid.translate([x[0],y[0]],[x[1],y[1]],orientation[1])

                traj_history = self.basemap.plot(xx[0],yy[0], 'o', markerfacecolor="None", markeredgecolor='red',\
                                                 markeredgewidth=3, markersize=10)
                dx = xx[1] - xx[0]
                dy = yy[1] - yy[0]
                traj_arrow_history = self.basemap.quiver(xx[0], yy[0], u=dx, v=dy, linewidth=1.2, \
                                                         color='red', headwidth=2, zorder=1000)
            

                if transition[1][3]==None:
                    cell_plot = []
                else:
                    cell_plot,x_center, y_center = test_grid.draw_cell(axes,transition[1][3], \
                                                                       orientation=orientation[1],return_center=True)
            t += 1


            plt.show(block=False)

            plt.pause(0.2)



            if global_save_img==True:
                # if i in [2,6,10]:
                #     for i in range(20):
                #         filename = str(global_k)
                #         filename = "0"*(5 - len(filename)) + filename
                #         plt.savefig('figures/hierarchical/img'+filename+'.png')
                #         global_k+=1

                # else:

                filename = str(global_k)
                filename = "0"*(5 - len(filename)) + filename
                plt.savefig('figures/hierarchical/img'+filename+'.png')
                global_k+=1


                    

            for c in cell_plot:
                c.remove()

            traj_history[0].remove()
            traj_arrow_history.remove()




    def simulate_initial_part(self,fig,ax,orientation=None,trajectory_file=None, cell_trajectory_file=None,draw_arrow=True):

        global global_k
        global global_save_img

        trajectory_txt = open(trajectory_file)
        trajectory_data = trajectory_txt.read()
        trajectory = json.loads(trajectory_data)
        
        cell_trajectory_txt = open(cell_trajectory_file)
        cell_trajectory_data = cell_trajectory_txt.read()
        cell_trajectory = json.loads(cell_trajectory_data)

        ac_x_off,ac_y_off,first_cell_x_off,first_cell_y_off,second_cell_x_off,second_cell_y_off = \
            self.generate_head_traj(orientation,trajectory_file,cell_trajectory_file)



        #################################################
        #################################################


        test_grid = Grid(30, 50)
        axes = test_grid.draw(fig,ax)

        trajectory_txt = open(trajectory_file)
        trajectory_data = trajectory_txt.read()
        trajectory = json.loads(trajectory_data)
        
        cell_trajectory_txt = open(cell_trajectory_file)
        cell_trajectory_data = cell_trajectory_txt.read()
        cell_trajectory = json.loads(cell_trajectory_data)

        initial_ac_traj = trajectory[0][1][1]
        initial_first_cell = trajectory[0][1][3]
        initial_second_cell = cell_trajectory[0]

        current_path = []
        current_path_weight = []

        current_path_plot = []
        text = None


        route_1 = [(1485708.593816702, 1757676.946866138), (943081.0241621046, 1199870.9836547687), (688842.9320861746, 598427.1389676803), (707815.9240321394, 399210.7235350484)]
        route_2 = [(688842.9320861746, 594632.5405784872), (556031.9884644197, 207583.50488080247)]

        xx_1 = []
        yy_1 = []
        for point in route_1:
            xx_1.append(point[0])
            yy_1.append(point[1])

        xx_2 = []
        yy_2 = []
        for point in route_2:
            xx_2.append(point[0])
            yy_2.append(point[1])

        route_1 = ax.plot(xx_1,yy_1,'b-',linewidth=2)
        route_2 = ax.plot(xx_2,yy_2,'b-',linewidth=2)
        

        for i in range(14):

            ac_x = ac_x_off[i]
            ac_y = ac_y_off[i]
            first_cell_x = first_cell_x_off[i]
            first_cell_y = first_cell_y_off[i]
            second_cell_x = second_cell_x_off[i]
            second_cell_y = second_cell_y_off[i]


            current_path_plot.append(test_grid.draw_path_3(axes,[0.5,15.5], \
                                                               [ac_x_off[i],ac_y_off[i]], \
                                                               [ac_x_off[i+1],ac_y_off[i+1]],\
                                                               lw=3, color='red',\
                                                               orientation=orientation[0]))

            xx,yy = test_grid.get_path(axes,[0.5,15.5], \
                                       [ac_x_off[i],ac_y_off[i]], \
                                       [ac_x_off[i+1],ac_y_off[i+1]],\
                                       lw=3, color='red',\
                                       orientation=orientation[0])
            
            traj_history = self.basemap.plot(xx[0],yy[0], 'o', markerfacecolor="None", markeredgecolor='red',\
                                             markeredgewidth=3, markersize=10)
            dx = xx[1] - xx[0]
            dy = yy[1] - yy[0]
            traj_arrow_history = self.basemap.quiver(xx[0], yy[0], u=dx, v=dy, linewidth=1.2, \
                                                     color='red', headwidth=2, zorder=1000)



            cell_plot,x_center, y_center = test_grid.draw_cell_2(axes,initial_first_cell, \
                                                                 offset_2=[first_cell_x,first_cell_y],\
                                                                 orientation=orientation[0],return_center=True)

            if draw_arrow==True:
                weather_arrow_history_1 = self.basemap.quiver(x=x_center,\
                                                              y=y_center, \
                                                              u=-17, v=87,linewidth=1.2, \
                                                              color='black', headwidth=2)



            ## draw cell for next segment
            c_2,x_center,y_center = test_grid.draw_cell_2(axes,initial_second_cell,\
                                                          offset_2=[second_cell_x,second_cell_y],\
                                                          orientation=orientation[1],return_center=True)
            
            if draw_arrow==True:
                weather_arrow_history_2 = self.basemap.quiver(x=x_center,\
                                                              y=y_center, \
                                                              u=-43, v=43,linewidth=1.2, \
                                                              color='black', headwidth=2)


            # path1 = plt.ginput(5)
            # print(path1)
            # path2 = plt.ginput(3)
            # print(path2)
            # path3 = plt.ginput(5)
            # print(path3)
            # path4 = plt.ginput(3)
            # print(path4)
            # plt.pause(10000)

            if i==2:
                text_loc = (1106451.6129032257, 816129.0322580657)
                text = plt.text(text_loc[0], text_loc[1], "Convective Weather Cells Detected.", fontsize=16)

                plt.pause(6)



            if i==6:
                ## sol 1
                plots = []
                trajectory_file_head = '/home/shong/mtk/workspaces/primary/acdc/source/models/aircraft_routing/hierarchical/sol1/wp1-wp4_' + 'policy_sim_'
                current_orientation = orientation[0]
                plots.extend(self.draw_single_segment_policy_for_hierarchical(fig,ax,current_orientation,trajectory_file_head))
                text = plt.text(text_loc[0], text_loc[1], "Hierarchical Policy Found.", fontsize=16)

                route_1[0].remove()
                route_2[0].remove()


                route_1 = [(1483811.2946221058, 1753882.348476945), (943081.0241621046, 1197973.6844601722), (969643.2128864557, 822308.4439300664), (850113.3636268766, 427670.2114539958), (705918.6248375431, 401108.0227296449)]
                route_2 = [(848216.0644322799, 427670.2114539958), (557929.2876590164, 205686.205686206)]
                

                xx_1 = []
                yy_1 = []
                for point in route_1:
                    xx_1.append(point[0])
                    yy_1.append(point[1])

                xx_2 = []
                yy_2 = []
                for point in route_2:
                    xx_2.append(point[0])
                    yy_2.append(point[1])

                route_1 = ax.plot(xx_1,yy_1,'b-',linewidth=2)
                route_2 = ax.plot(xx_2,yy_2,'b-',linewidth=2)


                plt.pause(5)

                

                


            if i==10:

                for p in plots:
                    if p!= None:
                        p.remove()
                
                route_1[0].remove()
                route_2[0].remove()
                
                ## sol 2
                plots = []
                trajectory_file_head = '/home/shong/mtk/workspaces/primary/acdc/source/models/aircraft_routing/hierarchical/sol2/wp1-wp2_' + 'policy_sim_'
                current_orientation = orientation[0]
                plots.extend(self.draw_single_segment_policy_for_hierarchical(fig,ax,current_orientation,trajectory_file_head))

                trajectory_file_head = '/home/shong/mtk/workspaces/primary/acdc/source/models/aircraft_routing/hierarchical/sol2/wp2-wp3_' + 'policy_sim_'
                current_orientation = orientation[1]
                plots.extend(self.draw_single_segment_policy_for_hierarchical(fig,ax,current_orientation,trajectory_file_head))
                text = plt.text(text_loc[0], text_loc[1], "Policy Updated.", fontsize=16)

                route_1 = [(1485708.593816702, 1757676.946866138), (943081.0241621046, 1199870.9836547687), (688842.9320861746, 598427.1389676803), (707815.9240321394, 399210.7235350484)]
                route_2 = [(688842.9320861746, 594632.5405784872), (556031.9884644197, 207583.50488080247)]

                xx_1 = []
                yy_1 = []
                for point in route_1:
                    xx_1.append(point[0])
                    yy_1.append(point[1])

                xx_2 = []
                yy_2 = []
                for point in route_2:
                    xx_2.append(point[0])
                    yy_2.append(point[1])

                route_1 = ax.plot(xx_1,yy_1,'b-',linewidth=2)
                route_2 = ax.plot(xx_2,yy_2,'b-',linewidth=2)


                plt.pause(5)

                
                


            plt.show(block=False)
            plt.pause(0.2)

            if global_save_img==True:
                if i in [2,6,10]:
                    for i in range(6):
                        filename = str(global_k)
                        filename = "0"*(5 - len(filename)) + filename
                        plt.savefig('figures/hierarchical/img'+filename+'.png')
                        global_k+=1

                else:

                    filename = str(global_k)
                    filename = "0"*(5 - len(filename)) + filename
                    plt.savefig('figures/hierarchical/img'+filename+'.png')
                    global_k+=1



            

            for c in cell_plot:
                c.remove()

            for c_ in c_2:
                c_.remove()

            if draw_arrow==True:
                weather_arrow_history_1.remove()
                weather_arrow_history_2.remove()

            if text!=None:
                text.remove()
                text = None

            traj_history[0].remove()
            traj_arrow_history.remove()


        return plots


    def generate_head_traj(self,orientation=None,trajectory_file=None, cell_trajectory_file=None,timestep=15):

        test_grid = Grid(30, 50)

        trajectory_txt = open(trajectory_file)
        trajectory_data = trajectory_txt.read()
        trajectory = json.loads(trajectory_data)
        
        cell_trajectory_txt = open(cell_trajectory_file)
        cell_trajectory_data = cell_trajectory_txt.read()
        cell_trajectory = json.loads(cell_trajectory_data)

        initial_ac_traj = trajectory[0][1][1]
        initial_first_cell = trajectory[0][1][3]
        initial_second_cell = cell_trajectory[0]

        new_ac_traj = (1058064.5161290322, 1316129.0322580657)
        new_first_cell_pos = (970967.7419354838, 754838.7096774206)
        new_second_cell_pos = (1067741.935483871, 412612.9032258076)

        ## compute ac traj
        
        x_center,y_center = test_grid.translate(0.5,15.5,orientation[0],point=True)

        ac_x_off = []
        ac_y_off = []

        for i in range(timestep):
            x_temp = (new_ac_traj[0]-x_center)/timestep*i
            y_temp = (new_ac_traj[1]-y_center)/timestep*i
            ac_x_off.append(x_temp)
            ac_y_off.append(y_temp)

        ac_x_off.reverse()
        ac_y_off.reverse()


        ## compute initial cell
        x_center = 0
        y_center = 0
        for cell_point in initial_first_cell:
            x_center += cell_point[0][0]
            y_center += cell_point[0][1]
        x_center /= len(initial_first_cell)
        y_center /= len(initial_first_cell)
        x_center,y_center = test_grid.translate(x_center,y_center,orientation[0],point=True)

        first_cell_x_off = []
        first_cell_y_off = []

        for i in range(timestep):
            x_temp = (new_first_cell_pos[0]-x_center)/timestep*i
            y_temp = (new_first_cell_pos[1]-y_center)/timestep*i
            first_cell_x_off.append(x_temp)
            first_cell_y_off.append(y_temp)

        first_cell_x_off.reverse()
        first_cell_y_off.reverse()



        ## compute initial cell
        x_center = 0
        y_center = 0
        for cell_point in initial_second_cell:
            x_center += cell_point[0][0]
            y_center += cell_point[0][1]
        x_center /= len(initial_second_cell)
        y_center /= len(initial_second_cell)
        x_center,y_center = test_grid.translate(x_center,y_center,orientation[1],point=True)

        second_cell_x_off = []
        second_cell_y_off = []

        for i in range(timestep):
            x_temp = (new_second_cell_pos[0]-x_center)/timestep*i
            y_temp = (new_second_cell_pos[1]-y_center)/timestep*i
            second_cell_x_off.append(x_temp)
            second_cell_y_off.append(y_temp)

        second_cell_x_off.reverse()
        second_cell_y_off.reverse()
        
        
        return ac_x_off,ac_y_off,first_cell_x_off,first_cell_y_off,second_cell_x_off,second_cell_y_off

        
