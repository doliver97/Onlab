import matplotlib.pyplot as plt
import json

x = [] # x axis values
y = [] # y axis values

#Receiving this edge's data
#edgeName = "654591108#2" 
#edgeName = "-435059227"
#edgeName = "-169375096#1"

edgeName = "-72675518#0" #This looks good
#edgeName = "-366892712"

#load and deserialize json
with open('outputData.json') as json_file:  
    data = json.load(json_file)
    for root in data['root']:
        for step in root:
            x.append(step)
            for edge in root[step]:
                if(edge==edgeName):
                    y.append(root[step][edge])

  
# plotting the points  
plt.plot(x, y) 
  
# naming the x axis 
plt.xlabel('Timestep') 
# naming the y axis 
plt.ylabel('Travelling time along the edge') 
  
# giving a title to my graph 
plt.title('SUMO simulation results') 
  
# function to show the plot 
plt.show() 