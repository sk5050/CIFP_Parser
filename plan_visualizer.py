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


class Visualizer(object):

    def __init__(self, parsed_cifp_filename,lon_lb=0,lat_lb=0,lon_ub=1,lat_ub=1):

        with open(parsed_cifp_filename) as parsed_cifp:
            self.cifp = json.load(parsed_cifp)

        self.plan = None
        self.basemap = None
        


    def visualize_plan(self,plan,lat_0,lon_0,width,height,resolution='h',show=False):

        self.plan = plan.augmented_plan


        self.draw_map(lat_0,lon_0,width,height,resolution=resolution)

        # self.draw_airport()

        if self.plan["SID"]==None:
            available_SIDs = self.find_available_SIDs(self.plan)
        else:
            for SID in self.plan["SID"]:
                self.draw_SID(airport=self.plan["origin"], SID_id=SID["SID_id"],\
                              transitions=SID["transitions"])
            
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
            plt.show()

        ax = plt.axes()

        

        return ax


    def draw_airport(self):

        origin = self.plan["origin"]
        dest = self.plan["destination"]
           

        origin_coord = self.get_wp_coord(origin, airport=origin)
        dest_coord = self.get_wp_coord(dest, airport=dest)
        
        self.basemap.plot(origin_coord[1], origin_coord[0], 'y*',latlon=True,linewidth=5,MarkerSize=10,alpha=0.8)
        self.basemap.plot(dest_coord[1], dest_coord[0], 'y*',latlon=True,linewidth=5,MarkerSize=10,alpha=0.8)


        if "alternative" in self.plan:
            alt = self.plan["alternative"]
            alt_coord = self.get_wp_coord(alt, airport=alt)
            self.basemap.plot(origin_coord[1], origin_coord[0], 'y*',latlon=True,linewidth=5,MarkerSize=10,alpha=0.5)



    def draw_route(self,route_graph,style='ko-',alpha_val=0.8):

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
            self.basemap.plot(lon,lat, style,latlon=True,linewidth=2,MarkerSize=5,alpha=alpha_val)




    def draw_alternatives(self, alternative):

        alt_airport = alternative["airport"]

        self.draw_alt_route(alt_airport, alternative["route graph"])

        
        
        # if "STAR" in alternative:
        #     for STAR in alternative["STAR"]:
        #         self.draw_STAR(airport=alternative["airport"], STAR_id=STAR["STAR_id"],\
        #                        transitions=STAR["transitions"])


                

    def draw_alt_route(self,alt_airport, route_graph,style='k--'):

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
            self.basemap.plot(lon,lat, style,latlon=True,linewidth=2,MarkerSize=5,alpha=0.8)
            # else:
            #     self.basemap.plot(lon,lat, 'r-',latlon=True, linewidth=6, MarkerSize=3, alpha=0.8)
            
            

        self.basemap.plot(lon[-1], lat[-1], 'g*',latlon=True,linewidth=5,MarkerSize=20,alpha=1.0)



    def draw_planned_route(self,route_graph):

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
            self.basemap.plot(lon,lat, 'rD-',latlon=True,linewidth=5,MarkerSize=10,alpha=0.2)            


    def draw_SID(self,airport, SID_id, transitions):


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

            self.basemap.plot(lon_simplified,lat_simplified, 'ko-',latlon=True,linewidth=2,MarkerSize=5,alpha=0.8)
            




    def draw_STAR(self,airport, STAR_id, transitions):


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
 
            self.basemap.plot(lon_simplified,lat_simplified, 'ko-',latlon=True,linewidth=2,MarkerSize=5,alpha=0.8)

        self.basemap.plot(lon_simplified[-1], lat_simplified[-1], 'y*',latlon=True,linewidth=5,MarkerSize=20,alpha=1.0)

    def draw_map(self,lat_0,lon_0,width,height,resolution='h'):

        plt.figure(figsize=(20, 15))

        self.basemap = Basemap(projection='lcc', resolution=resolution, 
                               lat_0=lat_0, lon_0=lon_0,
                               width=width, height=height)

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






                
