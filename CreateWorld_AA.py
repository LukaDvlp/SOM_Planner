import pickle,random,sys
args=sys.argv

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
Width = 10

Nodes,nodes=NodeGen_Rand2(Width,Width,GNum)

with open("Init_Goals_Dic.pickle",mode="wb") as fN:
    pickle.dump(Nodes,fN)
with open("Init_Goals_Tup.pickle",mode="wb") as fn:
    pickle.dump(nodes,fn)
