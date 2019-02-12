class Robot():
#parent = []   Finally I understood how class member/ Instance member works!
    def __init__(self,parent,pos,ID):
        self.parent = []
        self.child = []
        if parent is not None: # Parent Robot of the robot
            self.parent.append(parent)
        self.pos = pos        # Position of the robot
        self.ID = ID          # ID of the robot (robot No. WayPoint No)
        self.Flag = False   # Once moved, Flag is True



# MultiRobots-Path Plan Koushi-Exploration    # Nigouki, Const, Reward, Budged OK 2018/11/13
import numpy as np
from statistics import mean,variance
from matplotlib import pyplot as plt
import matplotlib.patches as patches
import matplotlib.ticker as tick
import matplotlib.animation as animation
import random,copy,csv,sys,pickle,time
from operator import itemgetter
args = sys.argv # mu, alpha, total neuron, G0, Robot Number, Node Number, badged, ComRange
#from Init_Solution_SOM import WyPts

class PathAdapt():
    
    def __init__(self,pos,Nodes,WyPts,mu,alpha,Robots,HopNum,MaxRange,r,G,Minimum,travel_budged,BS,Freq,Max_Ite): #mu:Learning Ratio, alpha: Decreasing Ratio, Minimum: Allowable distance Error
        #self.alpha = alpha
        #self.beta = beta
        self.mu = mu
        self.alpha = alpha
        #self.delta = delta
        #self.sigma = 1
        self.goal = [(float(args[8]),float(args[9]))] 
        self.Nodes = Nodes
        self.WyPts = WyPts
        self.MaxRange = MaxRange   #MaximumCommunication Range
        self.Range = MaxRange
        self.BS = Robot(None,BS,None) 
        self.r =0.5 
        self.th = 0
        self.HopNum = HopNum
        self.Robots = Robots 
        self.G = G            # Gain for Neighbour Food Function
        self.G0 = G
        self.Minimum = Minimum    # Sensing Observation Distance
        self.travel_budged = travel_budged  # budged of each robot = [b1,b2,.....,bn]
        #self.NodeStatus = self.set_NodeStatus(self.Nodes)
        self.Freq = Freq # Constraint Frequency
        self.Max = [0 for cc in range(self.Robots)]   # Maximum Index that communication was established
        self.Reward = [] # Reward History
        self.B_Reward = [] # Best_Reward History
        self.PTime1= []
        self.PTime2= []
        self.Max_Ite= Max_Ite
        
        self.W=self.AddWp_h(WyPts) #Initial WayPoints
        self.Process()   #Do Planning
    
    
    def WinnerSelect(self,WyPts,node):
        l=[]
        for w in WyPts:
            l.append(self.TwoD_Dis(w,node))
        i = l.index(min(l))
        return WyPts[i]
    
    def ThreeD_Dis(self,wypt, node):
        dis = np.sqrt((wypt[0]-node[0])**2 + (wypt[1]-node[1])**2 + (wypt[2]-node[2])**2)
        return dis
    
    def TwoD_Dis(self,wypt, node):
        dis = np.sqrt((wypt[0]-node[0])**2 + (wypt[1]-node[1])**2)
        return dis
    
    def NHF(self,d,G):
        #d = float(args[13])*d 
        A = np.e**(-(d/G)**2)
        #A = np.e**(-(d/G*10)**2)
        #print("Neighbourhood value desu {0}".format(A))
        if A < 1.0*10**-20:
            A = 0
        return A
    
    def Win_Adapt(self,X, Node, G):
        X = np.array(X)
        Node = np.array(Node)
        d = 1.0 * 10**-10
    
        #X = X + self.mu*(Node - X)
        X = X + self.mu*self.NHF(d,G)*(Node - X)
        return X.tolist()   
        
    def Nghbr_Adapt(self,X, Node, win_id, nghbr_id, G):
        X = np.array(X)
        Node = np.array(Node)
        d = abs(win_id-nghbr_id)                     # Difinition of neightbour: SOM space? Euculid Space? Still Fumei
        X = X + self.mu*self.NHF(d,G)*(Node - X) 
        return X.tolist()      
        
    #def AddWp_h(self,Wpts):
    def AddWp_h(self,WyPts):
        Array = np.array([])
        for b in WyPts:
            for i in b:
                Array = np.hstack((Array,i))
        return (Array)
    
    def WinRobot_Old(self,WyPts,node):
        Dis = [0 for k in range(self.Robots)]    # Sum of distances of waypoints from the node
        for r in range(self.Robots):
            if WyPts[r]==[]:
                Dis[r] = float("inf")
            else:
                for w in WyPts[r]:
                    #for n in self.Nodes:
                    #    Dis[r] += self.TwoD_Dis(w, n)
                    Dis[r] += self.TwoD_Dis(w, node)
        #print("\n\n Dis desu {0}".format(Dis))            
        return Dis.index(min(Dis))
    
    def WinRobot(self,WyPts,WyPts_Avairable,node): #Consider Dis, TotalDis from Node
        Dis = [0 for k in range(self.Robots)]        # Sum of distances of waypoints from the node
        TD = [0 for j in range(self.Robots)]         # Total traverse Distance of the Path of the robot
        Value = [0 for p in range(self.Robots)]      # Value of the path of the robot
        #Travel L
        for r in range(self.Robots):
            if WyPts_Avairable[r] == []:
                Value[r] = float("inf")
            else:
                for num in range(len(WyPts[r])):
                    if num == 0:
                        pass
                    else:
                        TD[r] += self.TwoD_Dis(WyPts[r][num-1],WyPts[r][num])
                    Dis[r] += self.TwoD_Dis(WyPts[r][num], node)
                if TD[r] < self.travel_budged[r]:
                    Value[r] = Dis[r]*TD[r]
                else:
                    Value[r] = float("inf")
        #print("\n\n Value desu {0}".format(Value))  
        WinRobo =  Value.index(min(Value))
        #if WinRobo == float("inf"):
        if Value[WinRobo] == float("inf"):
            #print("None kaeshimasita!")
            return None
        else:
            return WinRobo

    def WinRobot_New(self,WyPts,WyPts_Avairable,node): # The newest, consider TD/Budged
        Dis = [0 for k in range(self.Robots)]        # Sum of distances of waypoints from the node
        TD = [0 for j in range(self.Robots)]         # Total traverse Distance of the Path of the robot
        Value = [0 for p in range(self.Robots)]      # Value of the path of the robot
        #Travel L
        for r in range(self.Robots):
            if WyPts_Avairable[r] == []:
                Value[r] = float("inf")
            else:
                for num in range(len(WyPts[r])):
                    if num == 0:
                        pass
                    else:
                        TD[r] += self.TwoD_Dis(WyPts[r][num-1],WyPts[r][num])
                    Dis[r] += self.TwoD_Dis(WyPts[r][num], node)
                if TD[r] < self.travel_budged[r]:
                    #Value[r] = Dis[r]*TD[r]
                    Value[r] = TD[r]/travel_budged[r]
                else:
                    Value[r] = float("inf")
        #print("\n\n Value desu {0}".format(Value))  
        WinRobo =  Value.index(min(Value))
        #print("WinRobo desu! {0}".format(WinRobo))
        if Value[WinRobo] == float("inf"):
            #print("None kaeshimasita!")
            return None
        else:
            return WinRobo

    
    def Extrude(self,WyPts,Flags): # Extrude Avairable Waypoints
        W = copy.deepcopy(WyPts)
        r = self.Robots-1
        n = self.HopNum
        while r>=0:
            n = self.HopNum
            while n>=0:
                if Flags[r][n] == 1:
                    del W[r][n]
                else:
                    pass
                n-=1
            r-=1
        return W
    
    def TotalTravel(self, WyPts):
        Dis=[0 for i in range(self.Robots)]
        for r in range(self.Robots):
            w0 = WyPts[r][0]
            for num in range(len(WyPts[r])):
                Dis[r] += self.TwoD_Dis(WyPts[r][num],w0)
                w0 = WyPts[r][num]
        return Dis

####### Network Preservation  #######  
    
    def BreakCheck(self,R,X):
        PFlag = False
        CFlag = True
        for p in R.parent:
            if self.TwoD_Dis(p.pos,X)<self.MaxRange:
                PFlag=True
        for c in R.child:
            if self.TwoD_Dis(c.pos,X)>self.MaxRange:
                CFlag=False
    
        return PFlag&CFlag
    
    def getClosestRobot(self,rn,R_v):
        CRDic = {} # Close Robots in ascending order
        for rv in R_v:
            CRDic[self.TwoD_Dis(rn.pos,rv.pos)] = rv
        CRList = sorted(CRDic.items())
        i = 0
        while i<len(R_v): 
            R = CRList[i][1] # Closest Robot
            if self.BreakCheck(R,self.CheckPoint(R.pos,rn.pos)):
                break
            elif i==len(R_v)-1:
                R = None
                break
            else:
                i+=1
        return R

    def Check(self,A,B,a): # A:Robot Location, B.CheckPoint, a:ComRange
        if self.TwoD_Dis(A,B)<=a+0.01:
            return True
        else:
            return False
    
    def CheckPoint(self,A,B): # A will be adapted, a:ComRange
        a = np.array(A)
        b = np.array(B)
        p = b+self.Range*(a-b)/np.linalg.norm(a-b)
        return p.tolist()
    
    def getClosesetParent(self,A): # A: Robot
        dis = []
        for p in A.parent:
            dis.append(self.TwoD_Dis(A.pos,p.pos))
        if len(dis)>0:
            return dis.index(min(dis))
        else:
            return 0

    def ConnectionCheck(self,A,X,index,Adaptlist): # A:Robot, X:Checkpoint. Check whether it is ok or not if A moves to X
        if A is self.BS:
            return False
        elif self.TwoD_Dis(A.parent[self.getClosesetParent(A)].pos,X[index]) <= self.Range:
            Adaptlist.append(A)
            return True
        else:
            X.append(self.CheckPoint(A.parent[self.getClosesetParent(A)].pos,X[-1]))
            #Adaptlist.append(A.parent[0])
            Adaptlist.append(A)
            if self.ConnectionCheck(A.parent[self.getClosesetParent(A)],X,index+1,Adaptlist):
                return True
            else:
                return False
            #return True
            
    def Refine(self,R_v,R_nv,Wp,WpNumber):
        MoveRobots = []
        for rn in R_nv:
            Adaptlist = []
            AdaptlistID = []
            X = []
            #print("getClosestRobot Hajime")
            r=self.getClosestRobot(rn,R_v)
            #print("\n\n closestRobot.pos desu {0}".format(r.pos))
            if r is not None:
                X.append(self.CheckPoint(r.pos,rn.pos)) # the first argument is the robot which adapts
                #print("\n\n X at the first desu {0}".format(X))
                #print("ConnectionCheck Hajime")
                if self.ConnectionCheck(r,X,0,Adaptlist):
                    #print("X desu {0}".format(X))
                    #print("Adaptlist desu {0}".format(Adaptlist,key=attrgetter("pos")))
                    AdaptlistPos = [i.pos for i in Adaptlist]
                    #print("AdaptlistPos desu {0}".format(AdaptlistPos))
                    #for i in range(len(Adaptlist)):
                    #    print("{0}th Adaptlist.pos desu {1}".format(i, Adaptlist[i].pos))
                    #    print("{0}th X desu {1}".format(i, X[i]))
                    #print("\n\n Refine Mae WpNumber is {0} ha {1}".format(WpNumber,Wp))
                    for i in range(len(Adaptlist)):
                        #print("{0}th Adaptlist.pos desu {1}".format(i, Adaptlist[i].pos))
                        #print("{0}th X desu {1}".format(i, X[i]))
                        Adaptlist[i].pos = X[i]
                        Wp[Adaptlist[i].ID][WpNumber] = X[i]
                    #print("\n\n Refine Ato WpNumber is {0} ha {1}".format(WpNumber,Wp))
                    rn.parent.append(r)
                    r.child.append(rn)
                    #print("True!")
                    
                else:
                    #print("False!")
                    X[0] = self.CheckPoint(rn.pos,r.pos)
                    rn.pos=X[0]
                    #Adaptlist.append(rn)
                    rn.parent.append(BS)
                    Wp[rn.ID][WpNumber] = X[0]
            
                #MoveRobots.append(Adaptlist)
               # print("Adaptilist ID tsukuri at r is not None")
                #print("Adaptlist desu {0} Nagasaha {1}".format(Adaptlist, len(Adaptlist)))
                for a in Adaptlist:
                    AdaptlistID.append([a.ID,WpNumber])
                MoveRobots.append(AdaptlistID)
        else:
            XP = self.CheckPoint(rn.pos,self.BS.pos)
            #print("\n\n XP desu {0}".format(XP))
            Wp[rn.ID][WpNumber] = XP
            #print("Addaptilist ID tsukuri at r is None")
            AdaptlistID.append([rn.ID,WpNumber])
            MoveRobots.append(AdaptlistID)
            
        return MoveRobots
        #return MoveRobots
        #print("Refine shita ato no Wp desu {0}".format(Wp))
        #return(Wp)
        
    def set_Connection(self,R_v,R_nv):
    #def set_Connection(self,R_v,R_nv,MaxRange):
        X = []  # List robots that will be removed from R_nv
        Index = len(R_nv)
        for w in R_nv:
            Flag = False
            for v in R_v:
                if self.Check(w.pos,v.pos,self.Range):
                    w.parent.append(v)
                    v.child.append(w)
                    Flag = True
                else:
                    pass
                
            if Flag==True:
                X.append(w)
                R_v.append(w)
        
        if len(X)>0:
            for x in X:
                R_nv.remove(x)
        
        if len(R_nv)==Index: # Network Topology Found!
            return True
        else:                # Still seek the netrork topology
            return False
        
    def set_Network(self,Robots_path):
        R_v = []   # List of visible robots
        R_nv = []  # List of non-visible robots
        
        # Create R_v, R_nv from BS
        for r in range(self.Robots):
            if self.Check(Robots_path[r],self.BS.pos,self.Range):  # True: Visible from BS, a:Communication Range
                R_v.append(Robot(self.BS,Robots_path[r],r)) #
            else:
                R_nv.append(Robot(None,Robots_path[r],r))    # Parent, Position, ID
                
        # Connect R_nv to R_v
        while True:
            if self.set_Connection(R_v,R_nv):
            #if self.set_Connection(R_v,R_nv,MaxRange):
                break
        
        return R_v, R_nv
    
    def set_Network2(self,Robots_path): # For No Cooperation
        R_v = []   # List of visible robots
        R_nv = []  # List of non-visible robots
        
        # Create R_v, R_nv from BS
        for r in range(self.Robots):
            if self.Check(Robots_path[r],self.BS.pos,self.Range):  # True: Visible from BS, a:Communication Range
                R_v.append(Robot(self.BS,Robots_path[r],r)) #
            else:
                R_nv.append(Robot(None,Robots_path[r],r))    # Parent, Position, ID
                        
        return R_v, R_nv
    
    def NetworkPreservation(self,Wp,i): #Boss
        #Robots_path = [[] for k in range(Step)]
        Robots_path = []
        MoveRobots = None
        # Robots Waypoints for consideration
        for r in range(self.Robots):
            #print("r desu {0}".format(r))
            #for j in range(Step):    
            Robots_path.append(Wp[r][i])
            #j+=1
        
        # Get lists of visible/isolated robots
        R_v,R_nv = self.set_Network(Robots_path)   # R_v, R_nv: list of visible(connected)/invisible robots
        
        R_vPos=[i.pos for i in R_v]
        R_nvPos=[j.pos for j in R_nv]
        #print("\n\n R_vPos in Preservation1 desu {0}".format(R_vPos))
        #print("\n\n R_nvPos in Preservation1 desu {0}".format(R_nvPos))
        random.shuffle(R_v)
        random.shuffle(R_nv)
        #print("\n\n R_vPos desu {0}".format(R_vPos))
        #print("^n^n R_nvPos desu {0}".format(R_nvPos))
        if len(R_nv)>0 and len(R_v)>0:
        # Maintain network topology
            #print("Refine Hajime")
            MoveRobots = self.Refine(R_v,R_nv,Wp,i)
            #print("\n\n Refine Dekitayo")
            R_vPos=[i.pos for i in R_v]
            R_nvPos=[j.pos for j in R_nv]
            #print("\n\n R_vPos After Refine in Preservation1 desu {0}".format(R_vPos))
            #print("\n\n R_nvPos After Refine in Preservation1 desu {0}".format(R_nvPos))
            #self.Refine(R_v,R_nv,Wp,i)
        elif len(R_v)==0:
            #print("Backhome Hajime")
            self.backhome(R_nv,Wp,i)
            #print("\n\n Backhome Dekitayo")
            R_vPos=[i.pos for i in R_v]
            R_nvPos=[j.pos for j in R_nv]
            #print("\n\n R_vPos After backhome in Preservation1 desu {0}".format(R_vPos))
            #print("\n\n R_nvPos After backhome in Preservation1 desu {0}".format(R_nvPos))
            MoveRobots = [[[rnv.ID,i]] for rnv in R_nv]
            #MoveRobots = [[rnv] for rnv in R_nv]
        else:
            #print("No R_nv")
            pass
        
        return MoveRobots
        #MoveRobotsID = []
        #for rnv in range(len(R_nv)):
        #    for mv in MoveRobots[rnv]
        #       MoveRobotsID.append([mv.ID,i])
        #
        #return MoveRobtosID
        
    def NetworkPreservation2(self,Wp,i): # No cooperation
        #Robots_path = [[] for k in range(Step)]
        Robots_path = []
        MoveRobots = None
        #j = i
        # Robots Waypoints for consideration
        #print("Robots_path desu {0}".format(Robots_path))
        for r in range(self.Robots):
            #print("r desu {0}".format(r))
            #for j in range(Step):    
            Robots_path.append(Wp[r][i])
            #j+=1
        
        # Get lists of visible/isolated robots
        R_v,R_nv = self.set_Network2(Robots_path)   # R_v, R_nv: list of visible(connected)/invisible robots
        
        random.shuffle(R_v)
        random.shuffle(R_nv)
        R_vPos=[i.pos for i in R_v]
        R_nvPos=[j.pos for j in R_nv]
        #print("\n\n R_vPos in Preservation2 desu {0}".format(R_vPos))
        #print("\n\n R_nvPos in Preservation2 desu {0}".format(R_nvPos))
        if len(R_nv)>0 and len(R_v)>=0:
        # Maintain network topology
            self.backhome(R_nv,Wp,i)
            MoveRobots = [[[rnv.ID,i]] for rnv in R_nv]
            #self.Refine(R_v,R_nv,Wp,i)
        else:
            #print("No R_nv")
            pass
        
        return MoveRobots
        #MoveRobotsID = []
        #for rnv in range(len(R_nv)):
        #    for mv in MoveRobots[rnv]
        #       MoveRobotsID.append([mv.ID,i])
        #
        #return MoveRobtosID
        
    def backhome(self,R_nv,Wp,i):
        dis=[]
        for rnv in R_nv:
            #dis.append(self.TwoD_Dis(rnv.pos,self.BS.pos))
            #Wp[rnv.ID][i] = self.CheckPoint(rnv.pos,self.BS.pos)
            dis.append(self.TwoD_Dis(rnv.pos,self.goal[0]))
            Wp[rnv.ID][i] = self.CheckPoint(rnv.pos,self.goal[0])
        #print("No R_v, We all backed home")

    def RewardPropagation(self,Wp,r,i,Max): # WayPts, robot ID, Waypoint ID
        maxx = 0
        for w in range(len(Wp[r])):
            A=self.PropagationCheck(Wp,r,w)
            if A:
                maxx = w
                #print(A)
            else:
                #print(A)
                pass
            #print("Dis between Wp[r][w] and BS ha {0}".format(self.TwoD_Dis(Wp[r][w],self.BS.pos)))
        #Max[r]=maxx      #When no constraint working
        if Max[r]<maxx:     # When Constraint working
            Max[r]=maxx
        #print("r desu {0} i desu {1}".format(r,i))
        #print("Max desu {0}".format(Max))
        #print("maxx desu {0}".format(maxx))
        
        if i<=Max[r]:
            return True
        else:
            return False
                
        
    def PropagationCheck(self,Wp,r,i): # WayPts, robot ID, Waypoint ID
        #Robots_path = [[] for k in range(Step)]
        Robots_path = []
        #j = i
        # Robots Waypoints for consideration
        #print("Robots_path desu {0}".format(Robots_path))
        for rn in range(self.Robots):
            #print("r desu {0}".format(r))
            #for j in range(Step):    
            Robots_path.append(Wp[rn][i])
            #j+=1
        
        # Get lists of visible/isolated robots
        R_v,R_nv = self.set_Network(Robots_path)   # R_v, R_nv: list of visible(connected)/invisible robots
       
        List = [rnv.ID for rnv in R_nv]
        #print("List of R_nv desu {0}".format(List))
        #print("r desu {0}".format(r))
        if r in List:
            return False
        else:
            return True
        
    def Find_Closest(self,n,Wp):
        dis = [[] for i in range(self.Robots)]
        Mins = []
        for w1 in range(len(Wp)): # loop for Robots
            for w in Wp[w1]: # loop for waypoints for each robots
                dis[w1].append(self.TwoD_Dis(n,w))
        for i in range(len(dis)):
            Mins.append(min(dis[i]))
        Min = min(Mins)
        #return (Wp[Mins.index(Min)][dis[Mins.index(Min)].index(Min)]), Min
        return Mins.index(Min), Min     # Robot ID and Distance
        
    def Evaluate(self,Wp,TD):
        rewards = [0 for i in range(self.Robots)]
        Rewards = [0 for j in range(self.Robots)]
        for n in self.Nodes:
            ir,d = self.Find_Closest(n,Wp)
            #print("d ha {0} desu at node {1}".format(d,n))
            if d < self.Minimum:
                rewards[ir] += 1  # Rewards 
        
        #print("rewards desu {0}".format(rewards))
        #print("TD desu {0}".format(TD))
        for r in range(self.Robots):
            Rewards[r] = float(rewards[r]/TD[r])
            
        TR = 100*sum(Rewards)
        return TR, rewards

    def CheckReach(self,node,w):   # Checkif the node is observed(reached)
        if self.TwoD_Dis(node,w) <= self.Minimum:
            return 1
        else:
            return 0

    def NeighbourAdapt2(self,WyPts,MoveRobot):
        IDlist = []
        for a in range(len(MoveRobot)):
            for b in MoveRobot[a]:
                IDlist.append(b)
        for r1 in range(len(MoveRobot)):
            #print("\n\n r1 desu {0}".format(r1))
            for r2 in MoveRobot[r1]:
                #print("\n\n r2 desu {0}".format(r2))
                for w in WyPts[r2[0]]: #WyPts[RobotID]
                    nghbr_id = WyPts[r2[0]].index(w)
                    if nghbr_id!=0 and nghbr_id!=r2[1] and [r2[0],nghbr_id] not in IDlist and nghbr_id is not None:
                        w_=self.Nghbr_Adapt(w,WyPts[r2[0]][r2[1]],r2[1],nghbr_id,self.G)
                        WyPts[r2[0]][nghbr_id] = w
                    else:
                        pass
                    
    def Process(self):
        i=0
        P_R=0
        Converge = False
        CnvCnt = 0
        #while Converge==False:  #Convergence Condition
        while i<self.Max_Ite:
            i+=1
            start_time=time.time()
            #print("\n\n Learning Epoch{0}".format(i))
            #print("\n\n G is now {0}. G0 was {1}".format(self.G, self.G0))
            Error = 0
            random.shuffle(self.Nodes)
            Flags = [[0 for j in range(self.HopNum+1)] for i in range(self.Robots)]
            NodeNum = 0
            #self.Max = [0 for cc in range(self.Robots)]
            ErrorCount = 0
            #CnvCnt = 0
            for node in self.Nodes:
                WyPts_Tmp = [[] for gg in range(self.Robots)]
                WyPts_Avairable = self.Extrude(self.WyPts,Flags)
                #print("WyPts_Avairable at Epoch {0} NodeNum {1} is {2}".format(i,NodeNum,WyPts_Avairable))
                #print("\n\n Epoch {0}, nodeNum {1}, self.WyPts desu {2}".format(i, NodeNum, self.WyPts))
                #print("\n\n Epoch {0}, nodeNum {1}, WyPts_Avairable desu {2}".format(i, NodeNum, WyPts_Avairable))
                #Error_store = []  # Errors for each winner waypoints of the robots
                win_id_store = []
                for robo in range(self.Robots):
                    New_WyPts = copy.deepcopy(self.WyPts[robo])
                    #x = self.WinnerSelect(New_WyPts,node)
                    if WyPts_Avairable[robo] == []:
                        win_id = None
                    else:
                        x = self.WinnerSelect(WyPts_Avairable[robo],node)
                        #x = self.WinnerSelect(New_WyPts,node)
                        win_id = New_WyPts.index(x)
                        if win_id==0:
                            win_id+=1
                        elif win_id==self.HopNum:
                            win_id-=1
                    
                    if (win_id !=0) and win_id is not None:
                        #x_ = self.Win_Adapt(x,node)
                        x_ = self.Win_Adapt(x,node,self.G)
                        #x_ = self.Refine(x_,New_WyPts[win_id-1])
                        New_WyPts[win_id] = x_
                    else:
                        x_ = x
                        #pass  #Starting Position can not Adapt
                    #Error_store.append(self.TwoD_Dis(x_,node))
                    win_id_store.append(win_id)
                    for w in New_WyPts:
                        if w is not x_:
                            nghbr_id = New_WyPts.index(w) 
                            if (nghbr_id != 0) and win_id is not None:
                                w_ = self.Nghbr_Adapt(w,x,win_id,nghbr_id,self.G)
                                #w_ = self.Refine(w_,New_WyPts[nghbr_id-1])
                                New_WyPts[nghbr_id] = w_
                            else:
                                pass 
                            
                    #self.WyPts[robo] = New_WyPts #WyPts should be Tmp
                    WyPts_Tmp[robo] = New_WyPts #WyPts should be Tmp
                RIndex = self.WinRobot(WyPts_Tmp,WyPts_Avairable,node)  #Best robot selection. For other robots path, ignore
                #print("\n\n RIndex at Epoch {0}, NodeNum {1} is {2}".format(i,NodeNum,RIndex))
                #Error=max(Error,self.TwoD_Dis(self.WyPts[RIndex][win_id_store[RIndex]],node))
                if RIndex is not None:
                    if win_id_store[RIndex] is not None:
                        Flags[RIndex][win_id_store[RIndex]] = 1
                        self.WyPts[RIndex] = WyPts_Tmp[RIndex]
                        ## NEW  preservation at every Iteration ##
                        #self.WyPts = self.ComRefine(self.WyPts, self.MaxRange, [0,0])
                        #self.ComRefine(self.WyPts, self.MaxRange, [0,0])
                        #if i % 10 == 0:     # one in ten epoch, one waypoint in Waypoints, add constraint
                        #    print("Constraint Hatudou!")
                        #    kk=1
                        #    while kk < self.HopNum:
                        #        print("kk desu {0}".format(kk))
                                #MoveRobotID = self.NetworkPreservation(self.WyPts,kk)
                                #print("\n\n Rifine Mae {0}".format(self.WyPts))
                        #        self.NetworkPreservation(self.WyPts,kk)
                                #print("\n\n Rifine Ato {0}".format(self.WyPts))
                        #        kk += int(self.HopNum/10)
                        #        kk+=5
                        ErrorCount += 1
                        Error=max(Error,self.TwoD_Dis(self.WyPts[RIndex][win_id_store[RIndex]],node))
                        #print("\n\n Error at iteration {0} desu {1} Node ha {2} WyPt ha {3}".format(i,Error,node,self.WyPts[RIndex][win_id_store[RIndex]]))
                        #self.NodeStatus[node]=[1,self.CheckReach(node,self.WyPts[RIndex][win_id_store[RIndex]]),RIndex,win_id_store[RIndex],self.RewardPropagation(self.WyPts,RIndex,win_id_store[RIndex],self.Max)]
                        #print("\n\n RIndex desu {0}, win_id desu {1}".format(RIndex, win_id_store[RIndex]))
                    else:
                        pass
                else:
                    #print("\n\n Budged Koemashita~~~~ at Epoch{0}, self.WyPts is {1}".format(i,self.WyPts))
                    pass
                #print("\n\n RIndex desu {0}, win_id desu {1}".format(RIndex, win_id_store[RIndex]))
                #print("Node ha {0}".format(node))
                #print("\n\n WyPts (Adapt Mae) desu {0}".format(self.WyPts))
                #print("Flag at Epoch{0}, Node.No{1}, ha Konnakannji {2}".format(i,NodeNum,Flags))
                #print("\n\n WyPts Avairable desu {0}".format(WyPts_Avairable))
                #print("RIndex desu {0}".format(RIndex))
                #print("Node ha {0}".format(node))
                #print("Iteration ha {0}".format(i))
                #print("Mae no WyPts {0}".format(self.WyPts))
                #Error=max(Error,Error_store[RIndex])
                #Error_store.append(self.TwoD_Dis(x_,node))
                NodeNum += 1
                #print("Ato no WyPts {0}".format(self.WyPts))
                #Stack = AddWp_h(WyPts) 
                #W = np.vstack((W,Stack))
            #print("\n\n self.WyPts desu {0}".format(self.WyPts))
            #print("NodeStatus at iteration {0} desu {1}".format(i,self.NodeStatus))
            if i % 1 == 0:     # one in ten epoch, one waypoint in Waypoints, add constraint
                #print("Constraint Hatudou!")
                kk=1
                while kk <= self.HopNum:
                    if kk==1:
                        pass
                    else:
                        #print("kk desu {0}".format(kk))
                        #MoveRobotID = self.NetworkPreservation(self.WyPts,kk)
                        #self.NetworkPreservation(self.WyPts,kk)
                        #MoveRobot = self.NetworkPreservation(self.WyPts,kk)    # Cooperation
                        MoveRobot = self.NetworkPreservation2(self.WyPts,kk)   # No cooperation
                        if MoveRobot is not None:
                            self.NeighbourAdapt2(self.WyPts,MoveRobot)
                            for ab in range(len(MoveRobot)):
                                for bc in MoveRobot[ab]:
                                    self.Max[bc[0]]=max(self.Max[bc[0]],bc[1])
                        #kk += int(self.HopNum/5)
                    kk+=self.Freq
            
            process_time1=time.time()-start_time
            Stack = self.AddWp_h(self.WyPts) 
            #print("\n\n Stack desu {0}".format(Stack))
            self.W = np.vstack((self.W,Stack))
            #i+=1
            
            # Evaluation in the loop
            TD=self.TotalTravel(self.WyPts)
            #T_reward = self.Evaluate(self.WyPts,self.NodeStatus,TD)
            T_reward,rewards = self.Evaluate(self.WyPts,TD)
            #print("Error desu {0}".format(Error))
            #print("ErrorCount desu {0}. Node size ha {1}".format(ErrorCount, np.size(self.Nodes)))
            #print("T_reward desu {0}, type ha {1}".format(T_reward,type(T_reward)))
            if T_reward>=P_R:
                self.B_Reward.append(T_reward)
                P_R=T_reward
            else:
                self.B_Reward.append(P_R)
            self.Reward.append(T_reward)
            
            if self.G < 1.0*10**-20:
                self.G = 1.0*10**-20
            else:
                self.G = (1-self.alpha)*self.G
                
            if Error <= self.Minimum:
                #print("Converge Shita!")
                CnvCnt +=1
                #Conv = i
                if CnvCnt > 10:
                    Converge = True
                #pass
            
            elif i >= self.Max_Ite: 
                Converge = True
            process_time2=time.time()-start_time
            if int(1%10)==1:
                self.PTime1.append(process_time1)
                self.PTime2.append(process_time2)
            #print("\n\n Total Traverse Dis Robot{0} is {1}".format(robo,TD[robo]))
        with open("Solution_SOM.pickle",mode="wb") as f:
            pickle.dump(self.WyPts,f)
        #print("\n\n Total Rewards ha {0}".format(T_reward))
        #listData.append(T_reward) 
        #listData.append(TD) 
        #listData.append(rewards) 
    
