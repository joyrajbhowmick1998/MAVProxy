#!/usr/bin/env python
'''
pd Module
Joyraj Bhowmick, Milind Sharma, June 2018
Team UAS-DTU
'''

import os
import os.path
import sys
from pymavlink import mavutil
import errno
import time

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_settings

sys.path.append('/home/jbhowmick/interop/client')
import interop

import random
from math import *
from geopy.distance import *
import geopy

#payload drop programme starts here
#this will give distanec between two points based on ellipsoidal form of earth
def distance(cent_lat,cent_lon,target_lat,target_lon):
    obj1=(cent_lat,cent_lon)
    obj2=(target_lat,target_lon)
    return (vincenty(obj1, obj2))

#to calculate the distance payload covers
def distance_payload(alti,vx,vy):
    delta_t=0.05
    x=0.0
    y=0.0
    t=0.0
    Range=0.0
    k=.5*.82*1.225*0.0088
    m=.35
    g=9.81

    for i in range (10000) :
        ax=(-1.0)*(k/m)*vx*vx
        ay=9.81-(k/m)*vy*vy
        vx=vx+ax*delta_t
        vy=vy+ay*delta_t
        x=x+vx*delta_t+((ax*delta_t*delta_t)/2)
        y=y+vy*delta_t+((ay*delta_t*delta_t)/2)
        t=t+delta_t

        if (y>alti-1.0) :
            if(y<=alti) :
             Range=x
             #print Range
             break
    return (Range/1000)

#calculates bearing taking care of negative angles
def bearing( lat1 , lon1, lat2 , lon2):
    theta =atan2(sin(lon2-lon1)*cos(lat2),(cos(lat1)*sin(lat2))-(sin(lat1)* cos(lat2)*cos(lon2-lon1)))
    if (theta < 0) :
       theta+= 360
    return theta

def lat_lon(cent_lat,cent_lon,dist,bear):
    start=geopy.Point(cent_lat,cent_lon) #bottle_drop point
    d=geopy.distance.VincentyDistance(kilometers=dist)
    dest=d.destination(point=start,bearing=bear)
    print (dest.latitude,dest.longitude)
    return dest
    #obj=(cent_lat,cent_lon)
    #dest = vincenty.distance(kilometers=dist).destination(obj, bear)


class pd(mp_module.MPModule):
    def __init__(self, mpstate):
        """Initialise module"""
        super(pd, self).__init__(mpstate, "pd", "")
        self.status_callcount = 0
        self.boredom_interval = 10 # seconds
        self.last_bored = time.time()
        self.packets_mytarget = 0
        self.packets_othertarget = 0
        self.verbose = False
        self.lat_c=0
        self.lon_c=0
        self.alt_c=0
        self.bear=0
        self.Gspeed=0
        self.verticalSpeed=0
        self.target_lat=0
        self.target_lon=0
        self.off_set=0
        self.d=0
        self.payload_drop=0
        self.dist=0
        self.a=0
        self.flag=0

        self.pd_settings = mp_settings.MPSettings(
            [ ('verbose', bool, False),
          ])
        self.add_command('pd', self.cmd_pd, "pd module", ['status','set (LOGSETTING)'])

        #ip_port=input('Specify ip and port of interop server: ')
        self.client = interop.Client(url='http://localhost:8000',
                            username='testuser',
                            password='testpass')

    def usage(self):
        '''show help on command line options'''
        return "Usage: pd <status|set>"

    def cmd_pd(self, args):
        '''control behaviour of the module'''
        if len(args) == 0:
            print(self.usage())
        elif args[0] == "status":
            print(self.status())
        elif args[0] == "set":
            self.pd_settings.command(args[1:])
        else:
            print(self.usage())

        mav = self.master
        mav.mav.command_long_send(0, 0,
                                  mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
                                  7, 1800, 0, 0, 0, 0, 0)

        missions = self.client.get_missions()
        for mission in missions:
            mission_data=mission.__dict__
            if (str(mission_data['active'])=='True'):
                print 'Active Mission'
                loc=mission_data['air_drop_pos'].__dict__
                print 'Payload drop co-ordinates: ', loc['latitude'], loc['longitude']
                self.target_lat=loc['latitude']
                self.target_lon=loc['longitude']
                self.off_set=input("enter offset: ")
                self.a=input('Enter 1 to start: ')

                if(self.a==1):

                    while (True) :
                        print self.lat_c
                        print self.lon_c

                        self.d = distance_payload(self.alt_c, self.Gspeed, self.verticalSpeed)
                        #as the func depends on many constants as per payload dimensions so for sample  i have taken 10
                        #d=0.015
                        print "payload distance ", self.d

                        #bearing between starting and end point as per the text data
                        #bear=bearing( lat_c , lon_c, final_lat , final_lon)
                        print "bearing", self.bear

                        #this will give the point where the paylod drops
                        self.payload_drop = lat_lon(self.lat_c, self.lon_c, self.d, self.bear)

                        #this will give the distance between the payload drop point and target
                        self.dist = distance(self.target_lat, self.target_lon, self.payload_drop.latitude, self.payload_drop.longitude)
                        print self.dist

                        if self.dist < self.off_set :
                            #this can be changed as per requirement
                            mav.mav.command_long_send(0, 0,
                                                      mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
                                                      7, 1100, 0, 0, 0, 0, 0)
                            flag=1
                            print "payload dropped"

            else:
                print 'Inactive Mission'


    def status(self):
        '''returns information about module'''
        self.status_callcount += 1
        self.last_bored = time.time() # status entertains us
        return("status called %(status_callcount)d times.  My target positions=%(packets_mytarget)u  Other target positions=%(packets_mytarget)u" %
               {"status_callcount": self.status_callcount,
                "packets_mytarget": self.packets_mytarget,
                "packets_othertarget": self.packets_othertarget,
               })

    '''def boredom_message(self):
        if self.pd_settings.verbose:
            return ("I'm very bored")
        return ("I'm bored")'''

    #def idle_task(self):
        #called rapidly by mavproxy
    #    now = time.time()
    #    if now-self.last_bored > self.boredom_interval:
    #        self.last_bored = now
    #        message = self.boredom_message()
    #        self.say("%s: %s" % (self.name,message))
            # Se if whatever we're connected to would like to play:
    #        self.master.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_NOTICE,
    #                                        message)

    def mavlink_packet(self, m):
        #handle mavlink packets
        '''if m.get_type() == 'GLOBAL_POSITION_INT':
            if self.settings.target_system == 0 or self.settings.target_system == m.get_srcSystem():
                self.packets_mytarget += 1
            else:
                self.packets_othertarget += 1'''
        type = m.get_type()
        if type == 'VFR_HUD':
            self.Gspeed = m.groundspeed
            self.bear = m.heading
        if type == 'GLOBAL_POSITION_INT':
            self.lat_c = m.lat/ 1.0e7
            self.lon_c = m.lon/ 1.0e7
        if type == 'GLOBAL_POSITION_INT':
            self.alt_c = m.relative_alt/ 1.0e6
        if type == 'POSITION_TARGET_GLOBAL_INT':
            self.verticalSpeed = m.vz



def init(mpstate):
    '''initialise module'''
    return pd(mpstate)
