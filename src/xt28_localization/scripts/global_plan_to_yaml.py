#!/usr/bin/env python

"""
Description: 
    loads and plots results from .bag files
Usage:
    tbd
"""

import rosbag
import matplotlib.pyplot as plt
import Tkinter, tkFileDialog
import yaml

from scipy import interpolate
from lxml import etree 
import re


plt.close('all')

# get filepath from gui
root = Tkinter.Tk()
root.withdraw()
file_path = tkFileDialog.askopenfilename()
root.destroy()

# load bag
bag = rosbag.Bag(file_path)

# unpack
lat = []
lon = []

for topic, msg, t in bag.read_messages(topics=['/fix']):
    if (topic == "/fix"):
        lat.append(msg.latitude)
        lon.append(msg.longitude)
bag.close()

path =	{
  "lat": lat,
  "lon": lon
}

#f = open("sample.yaml", "w")
with open("sample.yaml", 'w') as outfile:
    yaml.dump(path, outfile)



# TODO kml format

        # load global plan kml
#        filename = rospy.get_param('/global_plan_kml')
#        filepath = rospkg.RosPack().get_path('xt28_localization') + '/data/global_plans_kml/' + filename
#        X_utm_out, Y_utm_out, utm_nr, utm_letter = self.get_cl_from_kml(filepath)
#        print(utm_nr)
#        print(utm_letter)
        
        
        
        
    def get_cl_from_kml(self,filepath):    
        tree = etree.parse(filepath)    
        lineStrings = tree.findall('.//{http://www.opengis.net/kml/2.2}LineString')    
        for attributes in lineStrings:
            for subAttribute in attributes:
                if subAttribute.tag == '{http://www.opengis.net/kml/2.2}coordinates':
                    coords = subAttribute.text
        
        # clean and cast
        coords = re.split(',| ',coords)
        coords[0] = coords[0].translate(None, "\n\t\t\t\t")
        coords = coords[:-1]
        
        # split in lon lat and alt
        if(len(coords) % 3 != 0):
            print "Error: len(coords) not divisible by three"
        lat = []
        lon = []
        alt = []
        for i in range(len(coords)/3):
            lon.append(float(coords[3*i]))
            lat.append(float(coords[3*i+1]))
            alt.append(float(coords[3*i+2]))
        lat = np.array(lat)
        lon = np.array(lon)
        alt = np.array(alt)
        
        # convert to utm
        X_utm,Y_utm,utm_nr,utm_letter = utm.from_latlon(lat, lon)
        
        # interpolate 
        ds = 1.0 # want approx 1 m between pts
        # approximate stot
        stot = 0
        for i in range(X_utm.size-1):
            stot += np.sqrt((X_utm[i+1]-X_utm[i])**2 + (Y_utm[i+1]-Y_utm[i])**2)
        N = int(stot/ds)
        unew = np.arange(0, 1.0, 1.0/N) # N equidistant pts
        tck, u = interpolate.splprep([X_utm, Y_utm], s=0)
        out = interpolate.splev(unew, tck)
        X_utm_out = out[0]
        Y_utm_out = out[1]    
    
        return X_utm_out, Y_utm_out, utm_nr, utm_letter 
        
        