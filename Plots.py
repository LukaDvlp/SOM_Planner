#Class: Plot Functions
from matplotlib import pyplot as plt
import matplotlib.patches as patches

class Plots():
    
    def __init__(self,W,Nodes,Robots):
        self.W = W
        self.Nodes = Nodes 
        self.Robots = Robots
        self.colors = ["red","blue","orange","green","purple","coral","darkgoldenrod","greenyellow", \
                       "aqua","indigo","lightpink","grey","pink","magenta","darkkhaki","lime","lightskyblue","black","saddlebrown", "olive"]
        self.TM_thin = 1
        self.Index = int(len(W[0])/Robots)-1
        self.lines = [[] for q in range(Robots)]
        
    def StaticPlot(self,fig,ax,ite,xlim,ylim,Range,save=False, filename=""):
        e=0
        ee=0
        x = [[] for e in range(self.Robots)]
        y = [[] for ee in range(self.Robots)]
        
        #p=0
        rr=0
        jj=0
        for rr in range(self.Robots):
            p=0
            for jj in range(int((self.Index+1)/2)):
                x[rr].append(self.W[ite][rr*(self.Index+1)+p])
                y[rr].append(self.W[ite][rr*(self.Index+1)+p+1])
                p+=2
            
        #L = len(x[0])-1
        rrr=0
        n=0
        for rrr in range(self.Robots):
            ax.plot(x[rrr],y[rrr],"X-",color=self.colors[rrr])    
        ax.text(0.05,0.9,"Epoch:{0}".format(ite), transform=ax.transAxes)
        for n in self.Nodes:
            c = patches.Circle(xy=n,radius=0.2,color="blue")
            ax.add_patch(c)
        RC = patches.Circle(xy=Range[0], radius=Range[1], ec="red", fill=False)
        ax.add_patch(RC)
            
        #ax2.plot(3,3,"X")
        #print("W is {0}".format(np.array(self.W)))
        #print("X is {0}, Y is {1}".format(x,y))
        ax.set_xlim(xlim)
        ax.set_ylim(ylim)
        plt.grid() 
        if save:
            fig.savefig(filename)
        else:
            pass
        
    def StaticPlot2(self,fig,ax,ite,xlim,ylim,Range,Step,HopNum,save=False, filename=""):
        e=0
        ee=0
        x = [[] for e in range(self.Robots)]
        y = [[] for ee in range(self.Robots)]
        X_ = [[] for e2 in range(self.Robots)]     # For Exclamation
        Y_ = [[] for ee2 in range(self.Robots)]    # For Exclamation
        
        #p=0
        rr=0
        jj=0
        for rr in range(self.Robots):
            p=0
            for jj in range(int((self.Index+1)/2)):
                x[rr].append(self.W[ite][rr*(self.Index+1)+p])
                y[rr].append(self.W[ite][rr*(self.Index+1)+p+1])
                if (jj-1)%Step==0 and jj-1!=0:
                    X_[rr].append(self.W[ite][rr*(self.Index+1)+p])
                    Y_[rr].append(self.W[ite][rr*(self.Index+1)+p+1])
                    #X_[rr].append(self.W[ite][rr*(self.Index+1)+p+2])
                    #Y_[rr].append(self.W[ite][rr*(self.Index+1)+p+1+2])
                p+=2
        #print("\n X_ desu {0}".format(X_))
        #print("\n Y_ desu {0}".format(Y_))
        #L = len(x[0])-1
        rrr=0
        n=0
       
        for rrr in range(self.Robots):
            ax.plot(x[rrr],y[rrr],"X-",color=self.colors[rrr])
            #ax.plot(X_[rrr],Y_[rrr],"X",color=self.colors[rrr])
            Num=0
            #for Num in range(int(HopNum/Step)):
            #ax.plot(0,0,"*",color=self.colors[Num],markersize=15)
            ax.plot(0,0,"*",color=self.colors[rrr],markersize=15)
            for Num in range(len(X_[rrr])):
                #ax.plot(X_[rrr][Num],Y_[rrr][Num],"*",color=self.colors[Num],markersize=15)
                ax.plot(X_[rrr][Num],Y_[rrr][Num],"*",color=self.colors[rrr],markersize=15)
                #print("Num desu {0}".format(Num))#ax.plot(X_[rrr][Num],Y_[rrr][Num],"*",color=self.colors[Num],markersize=15)
                #print("X_ desu {0}".format(X_))
                Num+=1
        ax.text(0.05,0.9,"Epoch:{0}".format(ite), transform=ax.transAxes,fontsize=15)
        for n in self.Nodes:
            c = patches.Circle(xy=n,radius=0.2,color="blue")
            ax.add_patch(c)
        #RC = patches.Circle(xy=Range[0], radius=Range[1], ec="red", fill=False)
        #ax.add_patch(RC)   
        #ax2.plot(3,3,"X")
        #print("W is {0}".format(np.array(self.W)))
        #print("X is {0}, Y is {1}".format(x,y))
        ax.set_xlim(xlim)
        ax.set_ylim(ylim)
        ax.tick_params(labelsize=18)
        plt.grid() 
        if save:
            fig.savefig(filename)
        else:
            pass
    
    def StaticPlotNode(self,fig,ax,ite,xlim,ylim,Range,Step,HopNum,save=False, filename=""):
        e=0
        ee=0
        x = [[] for e in range(self.Robots)]
        y = [[] for ee in range(self.Robots)]
        X_ = [[] for e2 in range(self.Robots)]     # For Exclamation
        Y_ = [[] for ee2 in range(self.Robots)]    # For Exclamation
        
        #p=0
        rr=0
        jj=0
        for rr in range(self.Robots):
            p=0
            for jj in range(int((self.Index+1)/2)):
                x[rr].append(self.W[ite][rr*(self.Index+1)+p])
                y[rr].append(self.W[ite][rr*(self.Index+1)+p+1])
                if (jj-1)%Step==0 and jj-1!=0:
                    X_[rr].append(self.W[ite][rr*(self.Index+1)+p])
                    Y_[rr].append(self.W[ite][rr*(self.Index+1)+p+1])
                    #X_[rr].append(self.W[ite][rr*(self.Index+1)+p+2])
                    #Y_[rr].append(self.W[ite][rr*(self.Index+1)+p+1+2])
                p+=2
        #print("\n X_ desu {0}".format(X_))
        #print("\n Y_ desu {0}".format(Y_))
        #L = len(x[0])-1
        rrr=0
        n=0
       
        #for rrr in range(self.Robots):
        #    ax.plot(x[rrr],y[rrr],"X-",color=self.colors[rrr])
        #    #ax.plot(X_[rrr],Y_[rrr],"X",color=self.colors[rrr])
        #    Num=0
        #    #for Num in range(int(HopNum/Step)):
        #    for Num in range(len(X_[rrr])):
        #        ax.plot(X_[rrr][Num],Y_[rrr][Num],"*",color=self.colors[Num],markersize=15)
        #        Num+=1
        #ax.text(0.05,0.9,"Iteration={0}".format(ite), transform=ax.transAxes)
        for n in self.Nodes:
            c = patches.Circle(xy=n,radius=0.2,color="blue")
            ax.add_patch(c)
        RC = patches.Circle(xy=Range[0], radius=Range[1], ec="red", fill=False)
        #ax.add_patch(RC)   
        #ax2.plot(3,3,"X")
        #print("W is {0}".format(np.array(self.W)))
        #print("X is {0}, Y is {1}".format(x,y))
        ax.set_xlim(xlim)
        ax.set_ylim(ylim)
        plt.grid() 
        if save:
            fig.savefig(filename)
        else:
            pass
        
    def init_ani(self):
        ttt=0
        for ttt in range(self.Robots):
            self.lines[ttt].set_data([], [])
        self.time_text.set_text('')
        return self.lines, self.time_text
    
    def animate(self,i):
        thisx=[[] for x_num in range(self.Robots)]
        thisy=[[] for y_num in range(self.Robots)]
        c = 0
        r_num=0
        ttt=0
        while c < self.Index:
            for r_num in range(self.Robots):
                thisx[r_num].append(self.W[i*self.TM_thin][r_num*(self.Index+1)+c])
                thisy[r_num].append(self.W[i*self.TM_thin][r_num*(self.Index+1)+c+1])
                #thisx2.append(W[i*TM_thin][Index+c])
                #thisy2.append(W[i*TM_thin][Index+c+1])
            c += 2
            
        #line1.set_data(thisx1, thisy1)
        #line2.set_data(thisx2, thisy2)
        for tttt in range(self.Robots):
            self.lines[tttt].set_data(thisx[tttt], thisy[tttt])
        self.time_text.set_text("Iteration: {0}".format(int(i)))
        #return line1, line2, time_text
        return self.lines, self.time_text
    
    def ShowAnime(self,fig,ax):
        #lines = [[] for q in range(Robots)]
        #colors = ["red", "blue", "yellow", "green", "purple"]
        #print("self.W shape desu {0}".format(np.shape(self.W)))
        #print("self.Index desu {0}".format(self.Index))
        #print("self.W desu {0}".format(self.W))
        #print("self.lines[5] desu {0}".format(self.lines[5]))
        tt=0
        n=0
        for tt in range(self.Robots):
            #print("tt desu {0}".format(tt))
            self.lines[tt], = ax.plot([],[], 'X-', lw=1, markersize=10, color = self.colors[tt])
            #line2, = ax.plot([],[], 'X-', lw=1, markersize=10, color = "blue")
        time_template = 'time = %.1fs'
        #time_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)
        
        for n in self.Nodes:
            c = patches.Circle(xy=n,radius=0.2,color="blue")
            ax.add_patch(c)
            
        plt.xlim(-20,20)
        plt.ylim(-20,20)
        plt.grid()
        ani = animation.FuncAnimation(fig, self.animate, np.arange(0, len(self.W)),
                                      interval=25, blit=False, init_func=self.init_ani)

