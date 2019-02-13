from Plots import Plots

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

from SOM_Planner import PathAdapt

##Simumation Module    
    
def InitWayPoints(WyPts,r,pos,HopNum,Robots): #r:HopInterval, pos:Robot Positions, HopNum:Number or hops, Robots:Number of Robots(Not necessary)
    th = 0
    T = 2*np.pi/Robots
    
    for rob in range(Robots):
        WyPts.append([])
    for w in WyPts:
        for k in range(HopNum):
            w.append([0,0])
    rn = 0
    for wp in WyPts:
        i = 0
        while i < len(wp):
            wp[i] = [pos[rn][0]+(i+1)*r*np.cos(th),pos[rn][1]+(i+1)*r*np.sin(th)]
            i+=1
        th += T
        rn += 1
        
    return WyPts

def AddPos(pos,WyPts,Robots,n):
    r=0
    for r in range(Robots):
        #WyPts[r].insert(0,pos[r])
        WyPts[r].insert(n,pos[r])
    return WyPts

def NodeGen(width,height,interval):
    Nodes = []
    i = -0.5*width
    j = -0.5*height
    while i <= 0.5*height:
        j = -0.5*height
        while j <= 0.5*width:
            Nodes.append((i,j))
            j += interval
        i += interval    
    
    return Nodes

def NodeGen_Rand1(width,height,interval):
    Nodes = []
    i = -0.5*width
    j = -0.5*height
    while i <= 0.5*height:
        j = -0.5*height
        while j <= 0.5*width:
            if random.randint(1,2)==1:
            	Nodes.append((i,j))
            j += interval
        i += interval    
    
    return Nodes

def NodeGen_Rand2(Max_width,Max_height,Num):
    Nodes = []
    for i in range(Num):
    	Nodes.append((random.uniform(-Max_width/2,Max_width/2),random.uniform(-Max_height/2,Max_height/2)))
    
    return Nodes
          
if __name__=="__main__":
    #print("args desu {0}".format(args))
    #Parameters
    G0s = [10.0,0.1,0.01]
    Mus = [0.1,0.5,0.9]
    Alphas = [0.01,0.08,0.5]
    NIs = [1,2,3]
    NNum = int(args[1])
    Max_Ite=int(args[2])
    G = G0s[int(args[3])]*NNum
    #G = 0.06 + 12.4*NNum # G = 0.06 +12.4 n
    mu = Mus[int(args[4])] 
    alpha = Alphas[int(args[5])]
    NIndex = NIs[int(args[6])]
    SRange = 4
    Width = 40
    with open("Init_Goals_Tup.pickle",mode="rb") as fn:
        Nodes = pickle.load(fn)
    
    MaxRange = 0.1
    th = 0
    Robots = 2
    GPos = [[[0,10],[0,15]],[[10,0],[15,0]]]
    HopNum = int(NNum*NIndex/Robots) 
    #Freq = HopNum-1   #Constraint Frequency
    Freq = int(HopNum/2)   #Constraint Frequency
    #if float(args[4])==-1:
    #    G = 0.06 + 12.4*len(Nodes) # G = 0.06 +12.4 n
    #else:
    #    G = float(args[4])
    Minimum = 0.1  # Allowable Maximum Distance from Nodes
    pos = [[0,0+0.001*o] for o in range(Robots)]
    BS = [0,0]
    r=0.01
    Conv=0
    travel_budged = []
    for b in range(Robots):
        travel_budged.append(10000)
    WyPts = InitWayPoints([],r,pos,HopNum,Robots)
    WyPts = AddPos(pos,WyPts,Robots,0)
    Result = PathAdapt(pos,Nodes,WyPts,mu,alpha,Robots,HopNum,MaxRange,r,G,Minimum,travel_budged,BS,Freq,Max_Ite,GPos)
    W = Result.W
    Rwd = Result.Reward
    B_Rwd = Result.B_Reward
    PT1_His = Result.PTime1
    PT2_His = Result.PTime2
    PT1 = mean(PT1_His)
    PT1_v = variance(PT1_His)
    PT2 = mean(PT2_His)
    PT2_v = variance(PT2_His)
    with open(args[7]+"/Init_W_SOM_G0="+str(G0s[int(args[4])])+".pickle",mode="wb") as f:
        pickle.dump(W,f)
    with open(args[7]+"/Init_R_SOM_G0="+str(G0s[int(args[4])])+".pickle",mode="wb") as f:
        pickle.dump(Rwd,f)
    with open(args[7]+"/Init_BR_SOM_G0="+str(G0s[int(args[4])])+".pickle",mode="wb") as f:
        pickle.dump(B_Rwd,f)
    with open(args[7]+"/Init_PT1_His_SOM_G0="+str(G0s[int(args[4])])+".pickle",mode="wb") as f:
        pickle.dump(PT1_His,f)
    with open(args[7]+"/Init_PT2_His_SOM_G0="+str(G0s[int(args[4])])+".pickle",mode="wb") as f:
        pickle.dump(PT2_His,f)
    
    #W=AddWp_h(WyPts)
    last = len(W)-1
    rr=last-1
    #print("last_SOM {0}".format(last))
    #print("Rwd_SOM {0}, type {1}, size {2}".format(Rwd,type(Rwd),np.shape(Rwd)))
    while rr >=0:
        if Rwd[last-1]>0:
            if (Rwd[last-1]-Rwd[rr])/Rwd[last-1]>0.01:
                Conv = rr
                break
            elif r==0:
                Conv = last-1
                break
        else:
            Conv=1
        rr-=1

    #f = open(args[3],"a")
    #csvWriter = csv.writer(f)
    ##Data[G0,mu,alpha,NI,EnvNum,TryNum,Reward,Conv,Caltime1_m,PT1_v,Caltime2_m,PT2_v]
    #Data = [int(args[4]),int(args[5]),int(args[6]),int(args[7]),int(args[8]),int(args[9]),Rwd[Conv],Conv,Conv*PT1,PT1_v,Conv*PT2,PT2_v]
    #csvWriter.writerow(Data)
    #f.close()
            
        
        
        
    ## For Plotting    
    P=Plots(W,Nodes,Robots)
    # Animation
    fig = plt.figure()
    ax = plt.axes()
    #P.ShowAnime(fig,ax)
    lines = [[] for q in range(Robots)]
    #colors = ["red", "blue", "yellow", "green", "purple"]
    colors = ["red","blue","yellow","green","purple","coral","darkgoldenrod","greenyellow", \
                       "aqua","indigo","lightpink","grey"]
    for tt in range(Robots):
        lines[tt], = ax.plot([],[], 'X-', lw=1, markersize=10, color = colors[tt])
    #    #line2, = ax.plot([],[], 'X-', lw=1, markersize=10, color = "blue")
    time_template = 'time = %.1fs'
    time_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)
    
    def init():
        for ttt in range(Robots):
            lines[ttt].set_data([], [])
        #line2.set_data([], [])
        time_text.set_text('')
        #return line1, line2, time_text
        return lines, time_text
    
    
    TM_thin = 1
    Index = int(len(W[0])/Robots)-1
    #print(Index)
    def animate(i):
        thisx=[[] for x_num in range(Robots)]
        thisy=[[] for y_num in range(Robots)]
        c = 0
        while c < Index:
            for r_num in range(Robots):
                thisx[r_num].append(W[i*TM_thin][r_num*(Index+1)+c])
                thisy[r_num].append(W[i*TM_thin][r_num*(Index+1)+c+1])
                #thisx2.append(W[i*TM_thin][Index+c])
                #thisy2.append(W[i*TM_thin][Index+c+1])
            c += 2
            
        #line1.set_data(thisx1, thisy1)
        #line2.set_data(thisx2, thisy2)
        for tttt in range(Robots):
            lines[tttt].set_data(thisx[tttt], thisy[tttt])
        time_text.set_text("Epoch: {0}".format(int(i)))
        #return line1, line2, time_text
        return lines, time_text
    
    for n in Nodes:
        c = patches.Circle(xy=n,radius=0.2,color="blue")
        ax.add_patch(c)
        #ax.plot(w[0],w[1],"X", color="red", markersize=10)
       
    #plt.xlim(-22,22)
    #plt.ylim(-22,27)
    plt.xlim(-2,Width+2)
    plt.ylim(-2,Width+7)
    plt.grid()
    #print("W shape {0}".format(np.shape(W)))
    #print("W ha {0}".format(W))
    ani = animation.FuncAnimation(fig, animate, np.arange(0, len(W)),interval=25, blit=False, init_func=init)
    ani.save(args[7]+"/Anime_SOM_Init.mp4", fps=15)
#    
    # Initial Condition Plotting
    fig2 = plt.figure()
    ax2 = plt.axes()
    #ax2.set_xlim(-5, 5)
    #ax2.set_ylim(-5, 5)
    P.StaticPlot2(fig2,ax2,0,[-Width/2.0-2,Width/2.0+2],[-Width/2.0-2,Width/2.0+7],[[0,0],MaxRange],Freq,HopNum,True,args[7]+"/InitPath_SOM.png")
#    
    # Final Condition Plogging
    fig3 = plt.figure()
    ax3 = plt.axes()
    P.StaticPlot2(fig3,ax3,last,[-2,Width+2],[-2,Width+3],[[0,0],MaxRange],Freq,HopNum,True,args[7]+"/Final_Path_SOM_G0="+str(G0s[int(args[3])])+".png")
#    
#    if last>200:
#        fig4 = plt.figure()
#        ax4 = plt.axes()
#        #P.StaticPlot2(fig4,ax4,200,[-Width/2.0-2,Width/2.0+2],[-Width/2.0-2,Width/2.0+7],[[0,0],MaxRange],Freq,HopNum,True,args[11]+"/Path200"+args[12]+".png")
#    
#    if last>600:
#        fig5 = plt.figure()
#        ax5 = plt.axes()
#        #P.StaticPlot2(fig5,ax5,600,[-Width/2.0-2,Width/2.0+2],[-Width/2.0-2,Width/2.0+7],[[0,0],MaxRange],Freq,HopNum,True,args[11]+"/Path600"+args[12]+".png")
#    
#    if last>800:
#        fig6 = plt.figure()
#        ax6 = plt.axes()
#        #P.StaticPlot2(fig6,ax6,800,[-Width/2.0-2,Width/2.0+2],[-Width/2.0-2,Width/2.0+7],[[0,0],MaxRange],Freq,HopNum,True,args[11]+"/Path800"+args[12]+".png")
#    
#    if last>1000:
#        fig7 = plt.figure()
#        ax7 = plt.axes()
#        #P.StaticPlot2(fig7,ax7,1000,[-Width/2.0-2,Width/2.0+2],[-Width/2.0-2,Width/2.0+7],[[0,0],MaxRange],Freq,HopNum,True,args[11]+"/Path1000"+args[12]+".png")
#    
#    if last>1200:
#        fig8 = plt.figure()
#        ax8 = plt.axes()
#        #P.StaticPlot2(fig8,ax8,1200,[-Width/2.0-2,Width/2.0+2],[-Width/2.0-2,Width/2.0+7],[[0,0],MaxRange],Freq,HopNum,True,args[11]+"/Path1200"+args[12]+".png")
#    
#    if last>1400:
#        fig9 = plt.figure()
#        ax9 = plt.axes()
#        #P.StaticPlot2(fig9,ax9,1400,[-Width/2.0-2,Width/2.0+2],[-Width/2.0-2,Width/2.0+7],[[0,0],MaxRange],Freq,HopNum,True,args[11]+"/Path1400"+args[12]+".png")
#    
#    if last>1600:
#        fig10 = plt.figure()
#        ax10 = plt.axes()
#        #P.StaticPlot2(fig10,ax10,1600,[-Width/2.0-2,Width/2.0+2],[-Width/2.0-2,Width/2.0+7],[[0,0],MaxRange],Freq,HopNum,True,args[11]+"/Path1600"+args[12]+".png")
#    
#    if last>1800:
#        fig11 = plt.figure()
#        ax11 = plt.axes()
#        #P.StaticPlot2(fig11,ax11,1800,[-Width/2.0-2,Width/2.0+2],[-Width/2.0-2,Width/2.0+7],[[0,0],MaxRange],Freq,HopNum,True,args[11]+"/Path1800"+args[12]+".png")
#    
#    if last>2000:
#        fig12 = plt.figure()
#        ax12 = plt.axes()
#        #P.StaticPlot2(fig12,ax12,2000,[-Width/2.0-2,Width/2.0+2],[-Width/2.0-2,Width/2.0+7],[[0,0],MaxRange],Freq,HopNum,True,args[11]+"/Path2000"+args[12]+".png")
#
#    if last>2200:
#        fig13 = plt.figure()
#        ax13 = plt.axes()
#        #P.StaticPlot2(fig13,ax13,2200,[-Width/2.0-2,Width/2.0+2],[-Width/2.0-2,Width/2.0+7],[[0,0],MaxRange],Freq,HopNum,True,args[11]+"/Path2200"+args[12]+".png")
#
#    if last>2400:
#        fig14 = plt.figure()
#        ax14 = plt.axes()
#        #P.StaticPlot2(fig14,ax14,2400,[-Width/2.0-2,Width/2.0+2],[-Width/2.0-2,Width/2.0+7],[[0,0],MaxRange],Freq,HopNum,True,args[11]+"/Path2400"+args[12]+".png")
#
#    if last>2600:
#        fig15 = plt.figure()
#        ax15 = plt.axes()
#        #P.StaticPlot2(fig15,ax15,2600,[-Width/2.0-2,Width/2.0+2],[-Width/2.0-2,Width/2.0+7],[[0,0],MaxRange],Freq,HopNum,True,args[11]+"/Path2600"+args[12]+".png")
#
#    if last>2800:
#        fig16 = plt.figure()
#        ax16 = plt.axes()
#        #P.StaticPlot2(fig16,ax16,2800,[-Width/2.0-2,Width/2.0+2],[-Width/2.0-2,Width/2.0+7],[[0,0],MaxRange],Freq,HopNum,True,args[11]+"/Path2800"+args[12]+".png")
#
#    # Final Condition Plogging
#    fig3 = plt.figure()
#    ax3 = plt.axes()
#    #ax3.set_xlim(-5, 5)
#    #ax3.set_ylim(-5, 5)
#    #last = len(W)-1
#    P.StaticPlot2(fig3,ax3,last,[-Width/2.0-2,Width/2.0+2],[-Width/2.0-2,Width/2.0+7],[[0,0],MaxRange],Freq,HopNum,True,args[11]+"/Path"+args[12]+".png")
#    
    Ep = np.arange(0,last)
    fig12 = plt.figure()
    ax12 = plt.axes()
    ax12.plot(Ep,Rwd,label="Reward")
    ax12.set_xlabel("Epoch",fontsize=18)
    ax12.set_ylabel("Reward",fontsize=18)
    ax12.legend(fontsize=18)
    ax12.tick_params(labelsize=18)
    ax12.set_ylim([0,80])
    plt.gca().xaxis.set_minor_locator(tick.MultipleLocator(100))
    plt.gca().yaxis.set_minor_locator(tick.MultipleLocator(2))
    plt.grid(which="minor")
    fig12.savefig(args[7]+"/Init_Reward_SOM_G0="+str(G0s[int(args[4])])+".png",bbox_inches="tight")

    fig13 = plt.figure()
    ax13 = plt.axes()
    ax13.plot(Ep,B_Rwd,label="Reward")
    ax13.set_xlabel("Epoch",fontsize=18)
    ax13.set_ylabel("Reward",fontsize=18)
    ax13.legend(fontsize=18)
    ax13.tick_params(labelsize=18)
    ax13.set_ylim([0,80])
    plt.gca().xaxis.set_minor_locator(tick.MultipleLocator(100))
    plt.gca().yaxis.set_minor_locator(tick.MultipleLocator(2))
    plt.grid(which="minor")
    fig13.savefig(args[7]+"/Init_B_Reward_SOM_G0="+str(G0s[int(args[4])])+".png",bbox_inches="tight")
#
#    while len(Rwd)<Max_Ite:
#        Rwd.append(Rwd[last])
#        B_Rwd.append(B_Rwd[last])
#    fig14 = plt.figure()
#    ax14 = plt.axes()
#    ax14.plot(Ep,Rwd,label="Reward")
#    ax14.set_xlabel("Epoch",fontsize=18)
#    ax14.set_ylabel("Reward",fontsize=18)
#    ax14.legend(fontsize=18)
#    ax14.tick_params(labelsize=18)
#    ax14.set_ylim([0,30])
#    ax14.set_xlim([0,Max_Ite])
#    plt.gca().xaxis.set_minor_locator(tick.MultipleLocator(100))
#    plt.gca().yaxis.set_minor_locator(tick.MultipleLocator(2))
#    plt.grid(which="minor")
#    fig14.savefig(args[11]+"/Reward_long"+args[12]+".png",bbox_inches="tight")
#
#    fig15 = plt.figure()
#    ax15 = plt.axes()
#    ax15.plot(Ep,B_Rwd,label="Reward")
#    ax15.set_xlabel("Epoch",fontsize=18)
#    ax15.set_ylabel("Reward",fontsize=18)
#    ax15.legend(fontsize=18)
#    ax15.tick_params(labelsize=18)
#    ax15.set_ylim([0,30])
#    ax15.set_xlim([0,Max_Ite])
#    plt.gca().xaxis.set_minor_locator(tick.MultipleLocator(100))
#    plt.gca().yaxis.set_minor_locator(tick.MultipleLocator(2))
#    plt.grid(which="minor")
#    fig15.savefig(args[11]+"/B_Reward_long"+args[12]+".png",bbox_inches="tight")
#  
#    #plt.show()
