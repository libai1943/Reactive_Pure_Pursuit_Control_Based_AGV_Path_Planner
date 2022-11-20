import math
import numpy as np
import matplotlib.colors 
import matplotlib.pyplot as plt
def draw_obs (real_obs,ax):
    for i in range(len(real_obs)):
        p1 = plt.Polygon(xy=real_obs[i], color='k',linewidth=0.2,alpha=0.5)
        ax.add_patch(p1)
def draw_car(x, y, yaw, RB, RF, W, color='green'):
    for i in range(len(x)):
        car = np.array([[-RB, -RB, RF, RF, -RB],
                        [W / 2, -W / 2, -W / 2, W / 2, W / 2]])
        Rot1 = np.array([[math.cos(yaw[i]), -math.sin(yaw[i])],
                         [math.sin(yaw[i]), math.cos(yaw[i])]])

        car = np.dot(Rot1, car)

        car += np.array([[x[i]], [y[i]]])

        plt.plot(car[0, :], car[1, :], color)
def plot_graph(ox,oy,sx,sy,syaw,gx,gy,pres,real_obs,counter):
    a=0
    fig = plt.figure(figsize=(6, 6))
    ax = fig.add_subplot(111)
    ax.set_xbound(-2,102)
    ax.set_ybound(-2,102)
    xmajor = plt.MultipleLocator(20)  
    xminor = plt.MultipleLocator(5)  
    ymajor_1 = plt.MultipleLocator(20)
    yminor_1 = plt.MultipleLocator(5)
    ax.xaxis.set_minor_locator(xminor)
    ax.xaxis.set_major_locator(xmajor)
    ax.yaxis.set_minor_locator(yminor_1)
    ax.yaxis.set_major_locator(ymajor_1)    
    ax.grid(axis='x', which='both',linewidth = 0.2)
    ax.grid(axis='y', which='both',linewidth = 0.2)   
    draw_obs(real_obs,ax)
    plt.plot(gx, gy, marker='o', markeredgecolor='red', markersize=8, markerfacecolor='none')
    plt.arrow(sx, sy, np.cos(syaw), np.sin(syaw), width=0.2, color = "red")
    plt.title('Case %d' % counter,x=0.1,y=0.9,fontdict={'weight':'normal','size': 20})
    plt.xlabel('X / m', fontdict={'weight':'normal','size': 10})
    plt.ylabel('Y / m', fontdict={'weight':'normal','size': 10})
    if type(pres)==type([1]):
        if len(pres)==1:
            a=len(pres[0])
            v = [i for i in range(0,a)]
            a_color = [plt.get_cmap("rainbow", a)(int(i)) for i in v]
            for j in range(len(pres[0])):
                plt.plot(pres[0][j].x,pres[0][j].y,color=a_color[-j-1],linewidth=0.3)
        else:    
            for j in pres:
                a=len(j)
                v = [i for i in range(0,a)]
                a_color = [plt.get_cmap("rainbow", a)(int(i)) for i in v]
                if a==1:
                    plt.plot(j[0].x,j[0].y,color='#8B00FF',linewidth=0.3)
                else:    
                    for i in range(a):
                        plt.plot(j[i].x,j[i].y,color=a_color[-i-1],linewidth=0.3)
    elif type(pres)==type('a'):
        pass
    else:
        plt.plot(pres.x,pres.y, color='#8B00FF',linewidth=0.3)
    ax.axis("equal")
    ax.set_xlim(0,100)
    ax.set_ylim(0,100)
#     plt.savefig('Case %d.svg' % counter,dpi=600)
#     plt.savefig('Case %d.png' % counter,dpi=600)
    plt.show()
def plot_final_graph(ox,oy,sx,sy,syaw,gx,gy,pres,real_obs,counter,RB, RF, W):
    fig = plt.figure(figsize=(6, 6))
    ax = fig.add_subplot(111)
    ax.set_xbound(-2,102)
    ax.set_ybound(-2,102)
    xmajor = plt.MultipleLocator(20)  
    xminor = plt.MultipleLocator(5)  
    ymajor_1 = plt.MultipleLocator(20)
    yminor_1 = plt.MultipleLocator(5)
    ax.xaxis.set_minor_locator(xminor)
    ax.xaxis.set_major_locator(xmajor)
    ax.yaxis.set_minor_locator(yminor_1)
    ax.yaxis.set_major_locator(ymajor_1)    
    ax.grid(axis='x', which='both',linewidth = 0.2)
    ax.grid(axis='y', which='both',linewidth = 0.2)   
    draw_obs(real_obs,ax)
    plt.title('Case %d' % counter,x=0.1,y=0.9,fontdict={'weight':'normal','size': 20})
    plt.xlabel('X / m', fontdict={'weight':'normal','size': 10})
    plt.ylabel('Y / m', fontdict={'weight':'normal','size': 10})
    draw_car(pres.x, pres.y, pres.yaw, RB, RF, W)
    plt.plot(pres.x,pres.y, color='#8B00FF',linewidth=1)
    ax.axis("equal")
    ax.set_xlim(0,100)
    ax.set_ylim(0,100)
    plt.show()