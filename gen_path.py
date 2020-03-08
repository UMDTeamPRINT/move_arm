f = open("listofcoords","w")
f.write("")
f = open("listofcoords","a")
c=.3
while c < .6:
    f.write(".2 .2 "+str(c)+" 0 0 0\n")
    c = .001 + c
f.close()
