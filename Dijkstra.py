#!/usr/bin/env python
# Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
# Copyright (C) 2009-2018 German Aerospace Center (DLR) and others.
# This program and the accompanying materials
# are made available under the terms of the Eclipse Public License v2.0
# which accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v20.html
# SPDX-License-Identifier: EPL-2.0

# @file	runner.py

from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import optparse
import random
import sumolib
import json
import datetime

#HIPERPARAMETERS
dataCache = 5 # data will consist averaging the last "dataCache" measurements
timeStep = 10 # traffic is measured in every "timeStep" seconds

# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
	tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
	sys.path.append(tools)
else:
	sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary  # noqa
import traci  # noqa

############################################### MY FUNCTIONS

def getNotInternalEdges(edgeIDs): #internal edges start with ':'
	notInternalEdgeIDs = ()
	for edgeID in edgeIDs:
		if(edgeID[0]!=':'):
			notInternalEdgeIDs = notInternalEdgeIDs + (edgeID,)
	return notInternalEdgeIDs

def getMeanSpeeds(edgeIDs):
	meanSpeeds = []
	for edgeID in edgeIDs:
		meanSpeeds.append((edgeID,traci.edge.getLastStepMeanSpeed(edgeID))) # when empty, returns with maximum allowed speed
	return meanSpeeds

# ID-s of loaded vehicles in this step
def vehiclesIn():
	return traci.simulation.getDepartedIDList()

#Assuming that every lane is approximately the same length
def edgeLength(net,edge):
	laneID = net.getEdge(edge).getLanes()[0].getID()
	return traci.lane.getLength(laneID)

#Assuming every lane has the same speed limit
def edgeEmptySpeed(net,edge):
	laneID = net.getEdge(edge).getLanes()[0].getID()
	return traci.lane.getMaxSpeed(laneID)

def edgeEmptyTripTime(net,edge):
	return edgeLength(net,edge)/edgeEmptySpeed(net,edge)

def edgeTripTime(net,edge,eWA):
	return edgeLength(net,edge)/eWA[edge]


def edgeWeightAverages(edgeWeights):
	eWA = {}
	for eW in edgeWeights:
		eWA[eW] = sum(edgeWeights[eW])/len(edgeWeights[eW])
	return eWA

def updateEdgeWeights(net, edgeWeights):
	for eW in edgeWeights:
		del edgeWeights[eW][0]

		#do not divide with zero
		if traci.edge.getLastStepMeanSpeed(eW) == 0:
			lastMeanSpeed = 0.1
		else:
			lastMeanSpeed = traci.edge.getLastStepMeanSpeed(eW)
			
		edgeWeights[eW].append(edgeLength(net,eW)/lastMeanSpeed)
   
#use startEdge, endEdge = getEndpoints(vehicleID)	
def getEndpoints(vehicleID):
	edgeList = traci.vehicle.getRoute(vehicleID)
	return edgeList[0], edgeList[-1]

def isCarAllowed(net,edge):
	lanes = edge.getLanes()

	isCarEdge = True

	for l in lanes:
		if not l.allows("passenger"):
			isCarEdge = False
	
	return isCarEdge

def Dijkstra(net, edges, startID, endID,eWA):
	queue = [] # queue for the algo
	vpre = {} # predecessor
	distance = {} #distance from start

	for e in edges:
		queue.append(net.getEdge(e))
		distance[net.getEdge(e)] = 1000000000
		vpre[net.getEdge(e)] = None

	start = net.getEdge(startID)
	end = net.getEdge(endID)

	vpre[start] = None
	distance[start] = 0

	reachable = True

	while len(queue)>0 and distance[end]==1000000000:

		# Get lowest value in queue
		lowestVal = distance[queue[0]]
		lowest = queue[0]
		for x in queue:
			if distance[x] < lowestVal:
				lowestVal = distance[x]
				lowest = x


		children = lowest.getOutgoing()
		for e in children:
			if isCarAllowed(net,e):
				distThisWay = distance[lowest] + edgeTripTime(net,e.getID(),eWA)
				if e in distance:
					if distThisWay < distance[e]:
						distance[e] = distThisWay
						vpre[e] = lowest
				else:
					vpre[e] = queue
					distance[e] = distThisWay
		queue.remove(lowest)
		if len(queue)==0:
			reachable = False
			break
	
	if reachable:
		#reconstruct path
		path = [] # id of edges in the shortest path
		actualVertex = end
		path.append(end.getID())

		while vpre[actualVertex]!=None:
			actualVertex = vpre[actualVertex]
			path.append(actualVertex.getID())
		
		path.reverse()

		return path
	else:
		return None

def writeJSON(step,outfile,edgeWeightAverages):
	record = {step : edgeWeightAverages}
	json.dump(record,outfile,indent=4,separators=(',', ': '))
	outfile.write(",")

############################################### END OF MY FUNCTIONS

def run():
	"""execute the TraCI control loop"""
	step = 0
	################################### INIT
	print(str(datetime.datetime.now().time()) + " INIT started")
	idlist = traci.edge.getIDList()
	edges = getNotInternalEdges(idlist)
	net = sumolib.net.readNet('patched.net.xml')

	# fill weights with travel times of empty map
	edgeWeights = {} # a dictionary, containing weight value (time) for each edge key 
	for e in edges:
		weights = []
		emptyTripTime = edgeEmptyTripTime(net,e)
		for x in range(dataCache):
			weights.append(emptyTripTime)
		edgeWeights[e] = weights
	outfile = open("outputData.json","w")
	outfile.write('{\n"root":[')
	################################### INIT END
	while traci.simulation.getMinExpectedNumber() > 0:
		traci.simulationStep()
		######################################################### CODE START

		# measuring traffic on edges 		
		if step%timeStep == 0:
			print(str(datetime.datetime.now().time()) + " Edge measurement started")
			updateEdgeWeights(net, edgeWeights)
			eWA = edgeWeightAverages(edgeWeights)
			writeJSON(step,outfile,eWA)

		# set route for loaded vehicles
		departedVehicles = vehiclesIn()
		for lv in departedVehicles:
			print(str(datetime.datetime.now().time()) + " Setting route for " + lv)
			edgeList = None
			while edgeList == None:
				startEdge, endEdge = getEndpoints(lv)
				edgeList = Dijkstra(net,edges,startEdge,endEdge,eWA)
			traci.vehicle.setRoute(lv,edgeList)
			
		######################################################### CODE END
		step += 1
	traci.close()
	outfile.write("]}")
	outfile.close()
	sys.stdout.flush()


def get_options():
	optParser = optparse.OptionParser()
	optParser.add_option("--nogui", action="store_true",
						 default=False, help="run the commandline version of sumo")
	options, args = optParser.parse_args()
	return options


# this is the main entry point of this script
if __name__ == "__main__":
	options = get_options()

	# this script has been called from the command line. It will start sumo as a
	# server, then connect and run
	if options.nogui:
		sumoBinary = checkBinary('sumo')
	else:
		sumoBinary = checkBinary('sumo-gui')

	# this is the normal way of using traci. sumo is started as a
	# subprocess and then the python script connects and runs
	traci.start([sumoBinary, "-c", "osm.sumocfg"]) #################### SET THE RIGHT FILE
	run()
