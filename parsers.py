#!/usr/bin/env python

import sys
import json
from utils import *

class Parser(object):

    def __init__(self):

        self.CIFP_parsed = dict()



    def parse(self,Lines):

        while Lines:

            sec_code = Lines[0][4]

            if sec_code == "P":    # airport
                sec_end_linum = find_end_linum(Lines,'section',sec_code)
                self.parse_airports_sec(Lines[0:sec_end_linum])
                Lines = Lines[sec_end_linum:]

            elif sec_code == "E":  # En-route waypoints
                sec_end_linum = find_end_linum(Lines,'section',sec_code)
                self.parse_enroute_sec(Lines[0:sec_end_linum])
                Lines = Lines[sec_end_linum:]

            elif sec_code == "D":  # NDB waypoints
                sec_end_linum = find_end_linum(Lines,'section',sec_code)
                self.parse_NDB_sec(Lines[0:sec_end_linum])
                Lines = Lines[sec_end_linum:]


            else:
                sec_end_linum = find_end_linum(Lines,'section',sec_code)
                Lines = Lines[sec_end_linum:]



    def parse_NDB_sec(self,Lines):

        if "NDB" not in self.CIFP_parsed:
            self.CIFP_parsed["NDB"] = dict()

        for line in Lines:
            NDB_id = line[13:17].rstrip()
            lat = line[32:41]
            lon = line[41:51]

            self.CIFP_parsed["NDB"][NDB_id] \
                = {"LATITUDE": lat,
                   "LONGITUDE": lon}

                

    def parse_enroute_sec(self,Lines):

        while Lines:

            subsec_code = Lines[0][5]

            if subsec_code == "A":
                subsec_end_linum = find_end_linum(Lines,'enroute_subsec',subsec_code)
                self.parse_enroute_waypoints(Lines[0:subsec_end_linum])
                Lines = Lines[subsec_end_linum:]

            elif subsec_code == "R":
                subsec_end_linum = find_end_linum(Lines,'enroute_subsec',subsec_code)
                self.parse_airways(Lines[0:subsec_end_linum])
                Lines = Lines[subsec_end_linum:]



    def parse_enroute_waypoints(self,Lines):

        if "Enroute Waypoints" not in self.CIFP_parsed:
            self.CIFP_parsed["Enroute Waypoints"] = dict()

        for line in Lines:
            waypoint_id = line[13:18].rstrip()
            lat = line[32:41]
            lon = line[41:51]

            self.CIFP_parsed["Enroute Waypoints"][waypoint_id] \
                = {"LATITUDE": lat,
                   "LONGITUDE": lon}


    def parse_airways(self,Lines):

        if "Airways" not in self.CIFP_parsed:
            self.CIFP_parsed["Airways"] = dict()


        while Lines:

            route_id = Lines[0][13:18].rstrip()

            route_end_linum = find_end_linum(Lines, 'route', route_id)
            self.parse_airway(Lines[0:route_end_linum])

            Lines = Lines[route_end_linum:]



    def parse_airway(self,Lines):

        route_id = Lines[0][13:18].rstrip()

        route = []

        for line in Lines:

            fix = line[29:34].rstrip()

            route.append({"FIX": fix})

        self.CIFP_parsed["Airways"][route_id] = route

        


        

        
    def parse_airports_sec(self,Lines):

        while Lines:

            airport_code = Lines[0][6:10].rstrip()
            print(airport_code)

            airport_end_linum = find_end_linum(Lines,'airport',airport_code)

            airports_scope = ["KBOS", "KJFK", "KPHL", "KIAD", "KCLT", "KATL", "KMCO"]

            # if airport_code == "KBOS" or airport_code=="KJFK" or airport_code=="KLAX" or airport_code=="KLGB" or airport_code=="KSAN":
            #     self.parse_airport(Lines[0:airport_end_linum])

            if airport_code in airports_scope:
                self.parse_airport(Lines[0:airport_end_linum])

            Lines = Lines[airport_end_linum:]



    def parse_airport(self,Lines):

        while Lines:

            subsec_code = Lines[0][12]

            subsec_end_linum = find_end_linum(Lines, 'airport_subsec', subsec_code)
            self.parse_airport_subsec(Lines[0:subsec_end_linum])

            Lines = Lines[subsec_end_linum:]



    def parse_airport_subsec(self,Lines):

        subsec_code = Lines[0][12]

        if subsec_code == "A":
            self.parse_airport_header(Lines)

        elif subsec_code == "C":
            self.parse_airport_waypoints(Lines)

        elif subsec_code == "D":
            self.parse_airport_SIDs(Lines)

        elif subsec_code == "E":
            self.parse_airport_STARs(Lines)

        elif subsec_code == "F":
            self.parse_airport_IAPs(Lines)

            

        



    def parse_airport_header(self,Lines):
        airport_code = Lines[0][6:10].rstrip()
        name = Lines[0][93:123].rstrip()
        lat = Lines[0][32:41].rstrip()
        lon = Lines[0][41:51].rstrip()


        if "Airport" not in self.CIFP_parsed:
            self.CIFP_parsed["Airport"] = dict()
        
        self.CIFP_parsed["Airport"][airport_code] = {"NAME": name,
                                           "CODE": airport_code,
                                           "LATITUDE": lat,
                                           "LONGITUDE": lon}


    def parse_airport_waypoints(self,Lines):

        airport_code = Lines[0][6:10].rstrip()

        if "Terminal Waypoints" not in self.CIFP_parsed["Airport"][airport_code]:
            self.CIFP_parsed["Airport"][airport_code]["Terminal Waypoints"] = dict()

        for line in Lines:

            waypoint_id = line[13:18].rstrip()
            lat = line[32:41]
            lon = line[41:51]

            self.CIFP_parsed["Airport"][airport_code]["Terminal Waypoints"][waypoint_id] \
                = {"LATITUDE": lat,
                   "LONGITUDE": lon}
            

        

    def parse_airport_SIDs(self,Lines):

        airport_code = Lines[0][6:10].rstrip()


        while Lines:

            SID_id = Lines[0][13:19].rstrip()

            SID_end_linum = find_end_linum(Lines, 'SID', SID_id)
            self.parse_airport_SID(Lines[0:SID_end_linum])

            Lines = Lines[SID_end_linum:]


    def parse_airport_SID(self,Lines):

        SID_id = Lines[0][13:19].rstrip()

        while Lines:
            transition = Lines[0][20:25].rstrip()

            transition_end_linum = find_end_linum(Lines, 'transition', transition)
            self.parse_airport_SID_transition(Lines[0:transition_end_linum])

            Lines = Lines[transition_end_linum:]


    def parse_airport_SID_transition(self,Lines):

        airport_code = Lines[0][6:10].rstrip()
        SID_id = Lines[0][13:19].rstrip()
        RT_type = Lines[0][19]             
        transition = Lines[0][20:25].rstrip()

        procedure = []
        for line in Lines:

            fix = line[29:34].rstrip()
            if fix=='':
                fix = line[6:10]
            path_term = line[47:49]
            mag_crs = line[70:74]

            procedure.append({'FIX':fix,
                              'PATH TERM':path_term,
                              'MAG CRS':mag_crs})


        if "SID" not in self.CIFP_parsed["Airport"][airport_code]:
            self.CIFP_parsed["Airport"][airport_code]["SID"] = dict()

        if SID_id not in self.CIFP_parsed["Airport"][airport_code]["SID"]:
            self.CIFP_parsed["Airport"][airport_code]["SID"][SID_id] = dict()

        self.CIFP_parsed["Airport"][airport_code]["SID"][SID_id][transition] = {"RT Type":RT_type,
                                                                       "Procedure": procedure}


    def parse_airport_STARs(self,Lines):

        airport_code = Lines[0][6:10].rstrip()


        while Lines:

            STAR_id = Lines[0][13:19].rstrip()

            STAR_end_linum = find_end_linum(Lines, 'STAR', STAR_id)
            self.parse_airport_STAR(Lines[0:STAR_end_linum])

            Lines = Lines[STAR_end_linum:]


    def parse_airport_STAR(self,Lines):

        STAR_id = Lines[0][13:19].rstrip()

        while Lines:
            transition = Lines[0][20:25].rstrip()

            transition_end_linum = find_end_linum(Lines, 'transition', transition)
            self.parse_airport_STAR_transition(Lines[0:transition_end_linum])

            Lines = Lines[transition_end_linum:]


    def parse_airport_STAR_transition(self,Lines):

        airport_code = Lines[0][6:10].rstrip()
        STAR_id = Lines[0][13:19].rstrip()
        RT_type = Lines[0][19]             
        transition = Lines[0][20:25].rstrip()

        procedure = []
        for line in Lines:

            fix = line[29:34].rstrip()
            if fix=='':
                fix = line[6:10]
            path_term = line[47:49]
            mag_crs = line[70:74]

            procedure.append({'FIX':fix,
                              'PATH TERM':path_term,
                              'MAG CRS':mag_crs})


        if "STAR" not in self.CIFP_parsed["Airport"][airport_code]:
            self.CIFP_parsed["Airport"][airport_code]["STAR"] = dict()

        if STAR_id not in self.CIFP_parsed["Airport"][airport_code]["STAR"]:
            self.CIFP_parsed["Airport"][airport_code]["STAR"][STAR_id] = dict()

        self.CIFP_parsed["Airport"][airport_code]["STAR"][STAR_id][transition] = {"RT Type":RT_type,
                                                                       "Procedure": procedure}


    def parse_airport_IAPs(self,Lines):

        airport_code = Lines[0][6:10].rstrip()


        while Lines:

            IAP_id = Lines[0][13:19].rstrip()

            IAP_end_linum = find_end_linum(Lines, 'IAP', IAP_id)
            self.parse_airport_IAP(Lines[0:IAP_end_linum])

            Lines = Lines[IAP_end_linum:]


    def parse_airport_IAP(self,Lines):

        IAP_id = Lines[0][13:19].rstrip()

        while Lines:
            transition = Lines[0][20:25].rstrip()

            transition_end_linum = find_end_linum(Lines, 'transition', transition)
            self.parse_airport_IAP_transition(Lines[0:transition_end_linum])

            Lines = Lines[transition_end_linum:]


    def parse_airport_IAP_transition(self,Lines):

        airport_code = Lines[0][6:10].rstrip()
        IAP_id = Lines[0][13:19].rstrip()
        RT_type = Lines[0][19]             
        transition = Lines[0][20:25].rstrip()

        procedure = []
        for line in Lines:

            fix = line[29:34].rstrip()
            if fix=='':
                fix = line[6:10]
            path_term = line[47:49]
            mag_crs = line[70:74]

            procedure.append({'FIX':fix,
                              'PATH TERM':path_term,
                              'MAG CRS':mag_crs})


        if "IAP" not in self.CIFP_parsed["Airport"][airport_code]:
            self.CIFP_parsed["Airport"][airport_code]["IAP"] = dict()

        if IAP_id not in self.CIFP_parsed["Airport"][airport_code]["IAP"]:
            self.CIFP_parsed["Airport"][airport_code]["IAP"][IAP_id] = dict()

        self.CIFP_parsed["Airport"][airport_code]["IAP"][IAP_id][transition] = {"RT Type":RT_type,
                                                                       "Procedure": procedure}
        

        
