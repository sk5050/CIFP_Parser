#!/usr/bin/env python

import sys
import json
import matplotlib.pyplot as plt
import matplotlib as mlt
from mpl_toolkits.basemap import Basemap
from PIL import Image
from pylab import rcParams
from utils import *



class Visualizer(object):

    def __init__(self, parsed_cifp_filename,lon_lb=0,lat_lb=0,lon_ub=1,lat_ub=1):

        with open(parsed_cifp_filename) as parsed_cifp:
            self.cifp = json.load(parsed_cifp)

        self.plan = None
        self.basemap = None
        


    def visualize_plan(self,plan,lat_0,lon_0,width,height,resolution='h'):

        self.plan = plan


        self.draw_map(lat_0,lon_0,width,height,resolution=resolution)

        if plan["SID"]==None:
            available_SIDs = self.find_available_SIDs(self.plan)
        else:
            # print(self.plan)
            # print(self.plan["SID"]["SID_id"])
            # print(self.plan["SID"]["transition_id"])
            self.draw_SID(airport=self.plan["origin"], SID_id=self.plan["SID"]["SID_id"],\
                          transitions=self.plan["SID"]["transitions"])

        self.draw_route(self.plan["route"])

        if plan["STAR"]==None:
            available_STARs = self.find_available_STARs(self.plan)
        else:
            self.draw_STAR(airport=self.plan["destination"], STAR_id=self.plan["STAR"]["STAR_id"],\
                          transitions=self.plan["STAR"]["transitions"])

        
        # self.draw_SID(airport='KBOS',SID_id='BLZZR4', transition_id='RW04R')
        # self.draw_SID(airport='KBOS',SID_id='BLZZR4', transition_id='RW09')
        # self.draw_SID(airport='KBOS',SID_id='BLZZR4', transition_id='RW15R')


        
        plt.show()



    def draw_route(self,route):

        lat = []
        lon = []
        
        for element in route:
            if element in self.cifp["Airways"]:
                route_id = element
                
                route_segment = self.get_route_segment(route_id,start_waypoint,end_waypoint)

                for waypoint in route_segment:
                    wp_coord = self.get_wp_coord(waypoint["FIX"])

                    lat.append(wp_coord[0])
                    lon.append(wp_coord[1])

            else:
                waypoint = element
                
                if waypoint==route[0]:
                    wp_coord = self.get_wp_coord(waypoint, airport=self.plan["origin"])
                elif waypoint==route[-1]:
                    wp_coord = self.get_wp_coord(waypoint, airport=self.plan["destination"])
                else:
                    wp_coord = self.get_wp_coord(waypoint)

                lat.append(wp_coord[0])
                lon.append(wp_coord[1])

        self.basemap.plot(lon,lat, 'kD-',latlon=True,linewidth=5,MarkerSize=10,alpha=0.4)


    def draw_SID(self,airport, SID_id, transitions):

        lat = []
        lon = []

        for transition in transitions:

            transition_id = transition["transition_id"]

            procedure = self.cifp['Airport'][airport]['SID'][SID_id][transition_id]['Procedure']

            for step in procedure:
                wp_name = step['FIX']
                mag_crs = step['MAG CRS']
                path_term = step['PATH TERM']

                wp_coord = self.get_wp_coord(wp_name,airport)
                lat.append(wp_coord[0])
                lon.append(wp_coord[1])


        self.basemap.plot(lon,lat, 'kD-',latlon=True,linewidth=5,MarkerSize=10,alpha=0.4)




    def draw_STAR(self,airport, STAR_id, transitions):

        lat = []
        lon = []

        for transition in transitions:

            transition_id = transition["transition_id"]

            procedure = self.cifp['Airport'][airport]['STAR'][STAR_id][transition_id]['Procedure']

            for step in procedure:
                wp_name = step['FIX']
                mag_crs = step['MAG CRS']
                path_term = step['PATH TERM']

                wp_coord = self.get_wp_coord(wp_name,airport)
                lat.append(wp_coord[0])
                lon.append(wp_coord[1])        
 
        self.basemap.plot(lon,lat, 'kD-',latlon=True,linewidth=5,MarkerSize=10,alpha=0.4)        


    def draw_map(self,lat_0,lon_0,width,height,resolution='h'):

        self.basemap = Basemap(projection='lcc', resolution=resolution, 
                               lat_0=lat_0, lon_0=lon_0,
                               width=width, height=height)

        self.basemap.shadedrelief()
        self.basemap.drawcoastlines(color='gray')
        self.basemap.drawcountries(color='gray')
        self.basemap.drawstates(color='gray')



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
            route_segment = route[start_pos:end_pos-1]
        else:
            route_segment = route[end_pos:start_pos-1].reverse()

        return route_segment
            

        
    def get_wp_coord(self,wp_name,airport=None):

        if airport==None:
            if wp_name in self.cifp['Enroute Waypoints']:
                lat_str = self.cifp['Enroute Waypoints'][wp_name]['LATITUDE']
                lon_str = self.cifp['Enroute Waypoints'][wp_name]['LONGITUDE']

            elif wp_name in self.cifp['NDB']:
                lat_str = self.cifp['NDB'][wp_name]['LATITUDE']
                lon_str = self.cifp['NDB'][wp_name]['LONGITUDE']

            else:
                print(wp_name)
                raise ValueError("Waypoint not found.")
                    

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
                    print(wp_name)
                    raise ValueError("Waypoint not found.")


        lat = float(lat_str[1:3]) + float(lat_str[3:5])/60 + float(lat_str[5:])/360000
            
        lat_dir = lat_str[0]

        if lat_dir=="S":
            lat = -lat
        

        lon = float(lon_str[1:4]) + float(lon_str[4:6])/60 + float(lon_str[6:])/360000
        lon_dir = lon_str[0]
        
        if lon_dir=="W":
            lon = -lon

        
        return [lat,lon]
