import copy
import math
fr= open("listofwaypoints","r")
f = open("listofcoords","w")
f.write("")
f = open("listofcoords","a")

min_step = 0.0001

lines = fr.readlines()

start = [float(i) for i in lines[0].split(" ")]

for line in lines[1:]:
   print(line)
   linef = [float(i) for i in line.split(" ")]
   direction = [i - j for i, j in zip(linef,start)]
   print(direction)
   mindir = min([math.fabs(i) for i in direction if i!=0])
   step = [i*min_step/mindir for i in direction]
   i = 0

   last = start
   while math.floor(mindir/min_step) > i:
       i = i+1
       last=[i+j for i,j in zip(last,step)]
       f.write(" ".join([str(i) for i in last])+'\n')
   start = copy.deepcopy(linef)

f.close()
