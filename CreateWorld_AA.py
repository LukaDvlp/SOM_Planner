import pickle,random,sys
args=sys.argv


def NodeGen(width,height,interval):
    #Nodes = []
    Nodes = {}
    nodes = []
    #i = -0.5*width
    #j = -0.5*height
    i = 0
    j = 0
    n = 1
    #while i <= 0.5*height:
    while i <= height:
        #j = -0.5*height
        j = 0
        #while j <= 0.5*width:
        while j <= width:
            #Nodes.append((i,j))
            Nodes[n]=((i,j))
            nodes.append((i,j))
            n += 1
            j += interval
        i += interval

    return Nodes,nodes

#def NodeGen_Rand1(width,height,interval):
#    Nodes = []
#    i = -0.5*width
#    j = -0.5*height
#    while i <= 0.5*height:
#        j = -0.5*height
#        while j <= 0.5*width:
#            if random.randint(1,2)==1:
#                Nodes.append((i,j))
#            j += interval
#        i += interval
#
#    return Nodes

def NodeGen_Rand2(Max_width,Max_height,Num):
     Nodes = {}
     nodes = []
     n=1
     for i in range(Num):
         #P=(random.uniform(-Max_width/2,Max_width/2),random.uniform(-Max_height/2,Max_height/2))
         P=(random.uniform(0,Max_width),random.uniform(0,Max_height))
         Nodes[n]=(P)
         nodes.append(P)
         n+=1
     #Goal = (float(args[2]),float(args[3]))
     #Nodes[n]=(Goal)
     #nodes.append(Goal)
 
     return Nodes,nodes

GNum = int(args[1])
Width = 40
SR = 4 #Sensing Range

#Nodes,nodes=NodeGen_Rand2(Width,Width,GNum)
Nodes,nodes=NodeGen(Width,Width,SR)


with open("Init_Goals_Dic.pickle",mode="wb") as fN:
    pickle.dump(Nodes,fN)
with open("Init_Goals_Tup.pickle",mode="wb") as fn:
    pickle.dump(nodes,fn)
