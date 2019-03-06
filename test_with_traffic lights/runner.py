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
	meanSpeeds = ()
	for edgeID in edgeIDs:
		meanSpeeds = meanSpeeds + (traci.edge.getLastStepMeanSpeed(edgeID),) # when empty, returns with maximum allowed speed
	return meanSpeeds

############################################### END OF MY FUNCTIONS

def run():
	"""execute the TraCI control loop"""
	step = 0
	################################### INIT
	list = traci.edge.getIDList()
	notInternals = getNotInternalEdges(list)
	print(len(notInternals))
	################################### INIT END
	while traci.simulation.getMinExpectedNumber() > 0:
		traci.simulationStep()
		######################################################### CODE START
		meanSpeeds = getMeanSpeeds(notInternals)
		print("Step: ", step)
		for speed in meanSpeeds:
			print(speed)
		######################################################### CODE END
		step += 1
	traci.close()
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
	traci.start([sumoBinary, "-c", "testTL.sumocfg"]) #################### SET THE RIGHT FILE
	run()
