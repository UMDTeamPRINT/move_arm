f = open("listofcoords","w")
f.write("")
f = open("listofcoords","a")
c=0
while c < 3.14:
    f.write(".2 .2 .3 0 0 "+str(c)+"\n")
    c = .001 + c
f.close()
