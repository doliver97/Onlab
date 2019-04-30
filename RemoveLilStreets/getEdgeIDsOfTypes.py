from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import optparse
import random

if 'SUMO_HOME' in os.environ:
	tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
	sys.path.append(tools)
else:
	sys.exit("please declare environment variable 'SUMO_HOME'")

import sumolib

######################################################

net = sumolib.net.readNet('osm.net.xml')

minPriority = 8 # primary_link

patchFile = open('patch.edg.xml', "w")

patchFile.write("<edges>\n")

edges = net.getEdges()
for e in edges:
	if e.getPriority()<minPriority:
		patchFile.write('<delete id="'+ e.getID() +'"/>\n')

patchFile.write("</edges>")

patchFile.close()

