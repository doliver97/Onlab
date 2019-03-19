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

def isCarAllowed(net,edge):
	lanes = edge.getLanes()

	isCarEdge = True

	for l in lanes:
		if not l.allows("passenger"):
			isCarEdge = False
	
	return isCarEdge


def BFS(net, start, end):
	queue = [] # temporary queue for the algo
	vertices = [] # visited edges
	vpre = [] # predecessors of visited edges
	queue.append(start)
	vertices.append(start)
	vpre.append(None)

	reachable = True

	while end not in queue:
		children = queue[0].getOutgoing()
		for c in children:
			if isCarAllowed(net,c): 
				if c not in vertices:
					queue.append(c)
					vertices.append(c)
					vpre.append(queue[0])
		del queue[0]
		if len(queue)==0:
			reachable = False
			break
	
	if reachable:
		#reconstruct path
		path = [] # id of edges in the shortest path
		actualVertexPos = vertices.index(end)
		path.append(end.getID())

		while vpre[actualVertexPos]!=None:
			actualVertexPos = vertices.index(vpre[actualVertexPos])
			path.append(vertices[actualVertexPos].getID())
		
		path.reverse()

		return path
	else:
		return None

def vTypeXMLline(id, length, speedfactor, speeddev):
	return """<vType id="%s" length="%s" speedFactor="%s" speedDev="%s"/>\n""" % (id,length,speedfactor,speeddev)

def vehicleXMLline(net,id,ctype,fromedge,toedge,depart):
	start = net.getEdge(fromedge)
	end = net.getEdge(toedge)
	route = BFS(net,start,end)
	if route is not None:
		s = """<vehicle id="%s" type="%s" depart="%s">\n""" % (id,ctype,depart)
		s += '<route edges="'
		for edge in route:
			s+= str(edge) + " "
		s+= '"/>\n'
		s+= """</vehicle>\n"""
		return s
	else:
		 return None

def getCarEdges(net):
	carEdges = []
	allEdges = net.getEdges(False)

	for e in allEdges:
		if isCarAllowed(net,e):
			carEdges.append(e)
	
	return carEdges

net = sumolib.net.readNet('osm.net.xml')

caredges = getCarEdges(net)

#CONSTANTS
Nfiles = 1
NcarTypes = 3
carLengthMin = 3
carLengthMax = 6
speedFactorMin = 0.8
speedFactorMax = 1.2
speedDev = 0.1
Ncars = 1000
time = 360

for f in range(Nfiles):

	fileName = "gen" + str(f) + ".rou.xml"
	routeFile = open(fileName, "w")

	routeFile.write("<routes>\n")

	#GENERATE TYPES
	for x in range(NcarTypes):
		id = "carType" + str(x)
		length = random.randint(carLengthMin,carLengthMax)
		speedFactor = round(random.uniform(speedFactorMin,speedFactorMax),2)

		routeFile.write(vTypeXMLline(id,length,speedFactor,speedDev))
	
	#DEPARTURE TIMES
	departures = []
	for x in range(Ncars):
		departures.append(random.randint(0,time))
	departures.sort()

	#GENERATE VEHICLES AND ROUTES
	for x in range(Ncars):

		print(str(x) + "/" + str(Ncars))

		id = "car" + str(x)
		ctype = "carType" + str(random.randint(0,NcarTypes-1))
		fromedge = random.choice(caredges).getID()
		toedge = random.choice(caredges).getID()
		while fromedge==toedge:
			toedge = random.choice(caredges).getID()
		depart = departures[x]

		s = vehicleXMLline(net,id,ctype,fromedge,toedge,depart)
		while s == None:
			fromedge = random.choice(caredges).getID()
			toedge = random.choice(caredges).getID()
			s = vehicleXMLline(net,id,ctype,fromedge,toedge,depart)
		
		routeFile.write(s)
	
	routeFile.write("</routes>\n")

	routeFile.close()


#input("Press enter to exit ;)")