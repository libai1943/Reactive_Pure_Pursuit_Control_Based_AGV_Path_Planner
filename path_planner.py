import time
import math
import heapq
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors 
import scipy.spatial.kdtree as kd
import draw_graph
from random import randint
LB = 2.8  # vehicle wheelbase
RF = 3.76  # [m] distance from rear to vehicle front end of vehicle
RB = 0.929  # [m] distance from rear to vehicle back end of vehicle
W = 1.942  # [m] width of vehicle
XW=YW=101 # map witdth and length
SX = 10.0  # [m]
SY = 10.0  # [m]
GX = 90.0  # [m]
GY = 90.0  # [m]
SYAW= np.deg2rad(45.0)
GYAW= np.deg2rad(45.0)
ROBOT_RADIUS = 1.5 #[m]
GRID_RESOLUTION = 1.0 #[m]
KF= 0.1  # look-ahead distance factor
LFC = 2.0  # [m] look-ahead distance 
KSF = 1.0  # speed controler factor
DT = 0.1  # [s] time interval
TARGET_SPEED = 10.0 / 3.6  #[m/s] target speed
LD = KF* TARGET_SPEED + LFC
# Astar Part
class Node:
    def __init__(self, x, y, cost, pind):
        self.x = x  # x position of node
        self.y = y  # y position of node
        self.cost = cost  # g cost of node
        self.pind = pind  # parent index of node


class Para:
    def __init__(self, minx, miny, maxx, maxy, xw, yw, reso, motion):
        
        self.minx = minx
        self.miny = miny
        self.maxx = maxx
        self.maxy = maxy
        self.xw = xw
        self.yw = yw
        self.reso = reso  # resolution of grid world
        self.motion = motion  # motion set


def astar_planning(sx, sy, gx, gy, ox, oy, reso, rr,obs):
    """
    return path of A*.
    :param sx: starting node x [m]
    :param sy: starting node y [m]
    :param gx: goal node x [m]
    :param gy: goal node y [m]
    :param ox: obstacles x positions [m]
    :param oy: obstacles y positions [m]
    :param reso: xy grid resolution
    :param rr: robot radius
    :return: path
    """

    n_start = Node(round(sx / reso), round(sy / reso), 0.0, -1)
    n_goal = Node(round(gx / reso), round(gy / reso), 0.0, -1)

    ox = [x / reso for x in ox]
    oy = [y / reso for y in oy]
    obsmap = obs
    P = calc_parameters(ox, oy, rr, reso,XW,YW)

    open_set, closed_set = dict(), dict()
    open_set[calc_index(n_start, P)] = n_start

    q_priority = []
    heapq.heappush(q_priority,
                   (fvalue(n_start, n_goal), calc_index(n_start, P)))

    while True:
        if not open_set:
            break

        _, ind = heapq.heappop(q_priority)
        n_curr = open_set[ind]
        closed_set[ind] = n_curr
        open_set.pop(ind)

        for i in range(len(P.motion)):
            node = Node(n_curr.x + P.motion[i][0],
                        n_curr.y + P.motion[i][1],
                        n_curr.cost + u_cost(P.motion[i]), ind)

            if not check_node(node, P, obsmap):
                continue

            n_ind = calc_index(node, P)
            if n_ind not in closed_set:
                if n_ind in open_set:
                    if open_set[n_ind].cost > node.cost:
                        open_set[n_ind].cost = node.cost
                        open_set[n_ind].pind = ind
                else:
                    open_set[n_ind] = node
                    heapq.heappush(q_priority,
                                   (fvalue(node, n_goal), calc_index(node, P)))

    pathx, pathy = extract_path(closed_set, n_start, n_goal, P)

    return pathx, pathy



def check_node(node, P, obsmap):
    if node.x <= P.minx or node.x >= P.maxx or             node.y <= P.miny or node.y >= P.maxy:
        return False

    if obsmap[node.x - P.minx][node.y - P.miny]:
        return False

    return True


def u_cost(u):
    return math.hypot(u[0], u[1])


def fvalue(node, n_goal):
    return node.cost + h(node, n_goal)


def h(node, n_goal):
    return math.hypot(node.x - n_goal.x, node.y - n_goal.y)


def calc_index(node, P):
    return (node.y - P.miny) * P.xw + (node.x - P.minx)


def calc_parameters(ox, oy, rr, reso,xw,yw):
    motion = get_motion()
    P = Para(0, 0, xw, yw, xw, yw, reso, motion)
    return P

def extract_path(closed_set, n_start, n_goal, P):
    pathx, pathy = [n_goal.x], [n_goal.y]
    n_ind = calc_index(n_goal, P)

    while True:
        node = closed_set[n_ind]
        pathx.append(node.x)
        pathy.append(node.y)
        n_ind = node.pind

        if node == n_start:
            break

    pathx = [x * P.reso for x in reversed(pathx)]
    pathy = [y * P.reso for y in reversed(pathy)]

    return pathx, pathy


def get_motion():
    motion = [[-1, 0], [-1, 1], [0, 1], [1, 1],
              [1, 0], [1, -1], [0, -1], [-1, -1]]

    return motion

def acalc_obsmap(ox, oy, rr, xw,yw,reso):
    obsmap = [[False for _ in range(yw)] for _ in range(xw)]
    for x in range(xw):
        xx = x
        for y in range(yw):
            yy = y 
            if obsmap[x][y] == True:
                break
            else:
                for oxx, oyy in zip(ox, oy):
                    if math.hypot(oxx - xx, oyy - yy) <= rr / reso:
                        obsmap[x][y] = True
                        break
    return obsmap
def extract_lines(astar):
    res=[]
    bit=[astar[0]]
    a_res=[]
    for i in range(len(astar)-1):
        if astar[i+1][0]==astar[i][0]:
            k='n'
            res.append(k)
        else:
            k=(astar[i+1][1]-astar[i][1])/(astar[i+1][0]-astar[i][0])
            res.append(k)
    for i in range(1,len(res)-1):
        if res[i+1]!=res[i]:
            bit.append(astar[i+1])
    bit.append(astar[-1])
    return bit
def check_col(px, py, obsmap):
    for i in range(len(px)):
        if obsmap[round(px[i])][round(py[i])]:
            return True
    return False    
def get_line_segment_points(curr_point,target_point, interval = 1):
    point1 = curr_point
    point2 = target_point
    a = point2[1] - point1[1]
    b = point2[0] - point1[0]
    c=math.sqrt(math.pow(a,2)+math.pow(b,2))
    n=math.floor(c/interval)
    R_X=[]
    R_Y=[]
    if n==0:
        R_X=[round(point1[0],2),round(point2[0],2)]
        R_Y=[round(point1[1],2),round(point2[1],2)]
    else:
        for i in range(n):
            x = round((b / c) * (interval * i) + point1[0], 2)
            y = round((a / c) * (interval * i) + point1[1], 2)
            R_X.append(x)
            R_Y.append(y)      
        if R_X[-1] != point2[0]:
            R_X.append(round(point2[0],2))
            R_Y.append(round(point2[1],2))
    return R_X,R_Y
def short_cutting_path(astar,obs):
    real_path_x=[]
    real_path_y=[]
    i = 0
    while i < len(astar)-1:
        i = i+1
        if i == len(astar)-1:
            bx,by=get_line_segment_points(astar[i-1],astar[i])
            real_path_x+=bx
            real_path_y+=(by)
            break
        pathx,pathy=get_line_segment_points(astar[i-1],astar[i+1])
        if not check_col(pathx,pathy,obs):
            real_path_x+=(pathx)
            real_path_y+=(pathy)
            i = i+1
        else:
            ax,ay=get_line_segment_points(astar[i-1],astar[i])
            real_path_x+=(ax)
            real_path_y+=(ay)
    return real_path_x,real_path_y
#deformation part

def normal(ref,kdtree,ox,oy):
    res=[]
    a_r=[ref[1][0],ref[1][1]]
    nor=get_deformation_nor(a_r,ref[3],kdtree,ox,oy)  
    res.append(nor)            
    return res

def get_deformation_nor(ref,yaw,kdtree,ox,oy):
    nor=[0,0]
    d = 0.2 
    dl = (RF - RB) / 2.0
    dh = (RF + RB) / 2.0 
    r = dh + d 
    u_counter=0
    d_counter=0
    ix=ref[0]
    iy=ref[1]
    iyaw=yaw
    cos=math.cos(iyaw)
    sin=math.sin(iyaw)
    u_cx = ix + dl * cos-sin*W/4
    u_cy = iy + dl * sin+cos*W/4
    d_cx = ix + dl * cos+sin*W/4
    d_cy = iy + dl * sin-cos*W/4
    u_ids = kdtree.query_ball_point([u_cx, u_cy], r)
    if u_ids:
        for j in u_ids:
            xo = ox[j] - u_cx
            yo = oy[j] - u_cy
            dx = xo * cos + yo * sin
            dy = -xo * sin + yo * cos
            if abs(dx) < r and abs(dy) < W / 4 +d:
                u_counter+=1 
    d_ids = kdtree.query_ball_point([d_cx, d_cy], r)
    if d_ids:
        for j in d_ids:
            xo = ox[j] - d_cx
            yo = oy[j] - d_cy
            dx = xo * cos + yo * sin
            dy = -xo * sin + yo * cos
            if abs(dx) < r and abs(dy) < W / 4+d:
                d_counter+=1    
    if u_counter>d_counter:
        if u_counter<=2:
            nor=[sin,-cos]
        else:
            nor=[1.5*sin,-1.5*cos]
    elif u_counter==d_counter:
        nor=[sin,cos]
    else:
        if d_counter<=2:
            nor=[-sin,cos]
        else:
            nor=[-1.5*sin,1.5*cos]
    return nor

def collision_check(ref,yaw,kdtree,ox,oy):
    d = 0.2
    dl = (RF - RB) / 2.0
    r = (RF + RB) / 2.0 + d 
    iter_c=[]
    col=[]
    for i in range(1,len(ref)-1):
        ix=ref[i][0]
        iy=ref[i][1]
        iyaw=yaw[i]
        cx = ix + dl * math.cos(iyaw)
        cy = iy + dl * math.sin(iyaw)
        ids = kdtree.query_ball_point([cx, cy], r)
        if not ids:
            continue

        for j in ids:
            xo = ox[j] - cx
            yo = oy[j] - cy
            dx = xo * math.cos(iyaw) + yo * math.sin(iyaw)
            dy = -xo * math.sin(iyaw) + yo * math.cos(iyaw)
            if abs(dx) < r and abs(dy) < W / 2 + d:
                col.append([ref[i-1],ref[i],ref[i+1],iyaw])
                iter_c.append(i)
                break
    if len(iter_c)>=len(ref):
        print("error")
    return col,iter_c
class VehicleState:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v


def update(state, a, delta):

    state.x = state.x + state.v * math.cos(state.yaw) * DT
    state.y = state.y + state.v * math.sin(state.yaw) * DT
    state.yaw = state.yaw + state.v / LB * math.tan(delta) * DT
    state.v = state.v + a * DT

    return state
def speed_control(target, current):
    a = KSF * (target - current)

    return a


def pure_pursuit_control(state, cx, cy, pind):
    ind = calc_target_index(state, cx, cy)

    if pind >= ind:
        ind = pind

    if ind < len(cx):
        tx = cx[ind]
        ty = cy[ind]
    else:
        tx = cx[-1]
        ty = cy[-1]
        ind = len(cx) - 1

    alpha = math.atan2(ty - state.y, tx - state.x) - state.yaw

    if state.v < 0:  # back
        alpha = math.pi - alpha

    Lf = KF* state.v + LFC

    delta = math.atan2(2.0 * LB * math.sin(alpha) / Lf, 1.0)

    return delta, ind
def calc_target_index(state, cx, cy):
    dx = [state.x - icx for icx in cx]
    dy = [state.y - icy for icy in cy]
    ind = np.argmin(np.hypot(dx, dy))
    Ls = 0.0
    Lf = KF* state.v + LFC

    while Lf > Ls and (ind + 1) < len(cx):
        dx = cx[ind + 1] - cx[ind]
        dy = cx[ind + 1] - cx[ind]
        Ls += math.hypot(dx, dy)
        ind += 1

    return ind


def add_line(gx,gy,yaw):
    px=[gx+math.cos(yaw)*0.1*(LD+5)*i for i in range(1,21)]
    py=[gy+math.sin(yaw)*0.1*(LD+5)*i for i in range(1,21)]
    return px,py
def calc_astar(state, cx, cy):
    dx = [state[0] - icx for icx in cx]
    dy = [state[1] - icy for icy in cy]
    ind = np.argmin(np.hypot(dx, dy))
    return ind

def cal_guide(cx,cy,state):
    T = 100.0 
    lastIndex = len(cx) - 1
    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]

    t = [0.0]
    target_ind = calc_target_index(state, cx, cy)

    while T >= time and lastIndex > target_ind:
        dis = np.hypot([GX - state.x], [GY - state.y])
        if dis<= LFC:
            ai = speed_control(0, state.v)
        else:
            ai = speed_control(TARGET_SPEED, state.v)
        di, target_ind = pure_pursuit_control(state, cx, cy, target_ind)
        state = update(state, ai, di)

        time = time + DT

        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)
    return x,y,yaw,v,t 


def get_end_index(x_line,y_line,gx,gy):
    dx = [gx - icx for icx in x_line]
    dy = [gy - icy for icy in y_line]
    ind = np.argmin(np.hypot(dx, dy))
    return ind
def first_guide(cx,cy,state):
    T = 100.0  
    lastIndex = len(cx) - 1
    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    target_ind = calc_target_index(state, cx, cy)
    while T >= time and lastIndex > target_ind:
        dis = np.hypot([GX - state.x], [GY - state.y])
        if dis<= LFC:
            ai = speed_control(0, state.v)
        else:
            ai = speed_control(TARGET_SPEED, state.v)
        di, target_ind = pure_pursuit_control(state, cx, cy, target_ind)
        state = update(state, ai, di)

        time = time + DT

        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)
    final_end=get_end_index(x,y,GX,GY)
    a_hx=x[:final_end]
    a_hy=y[:final_end]
    a_yaw=yaw[:final_end]
    a_v=v[:final_end]
    a_t=t[:final_end]
    return a_hx,a_hy,a_yaw,a_v,a_t
class REF:
    def __init__(self,x,y,yaw,v,t ):
        self.x = x  # x position of node
        self.y = y  # y position of node
        self.yaw = yaw  # y position of node
        self.v = v # y position of node
        self.t = t  # y position of node

def cut_line(r,iter_c,R,pathx,pathy):
    a_ind=[]
    for i in range(len(r)):
        state=[R.x[iter_c[i]],R.y[iter_c[i]]]
        a=calc_astar(state, pathx, pathy)
        a_ind.append(a)
    tind=list(sorted(set(a_ind)))
    r_ind=[]
    slow=0
    fast=0
    while fast<len(tind):
        fast+=1
        if fast==len(tind):
            r_ind+=[tind[slow:fast]]
        elif tind[fast]!=tind[fast-1]+1:
            r_ind+=[tind[slow:fast]]
            slow=fast    
    return r_ind


def combine_lines(r_ind,pathx,pathy):
    new_ind=[]
    t_ind=[]

    for i in range(len(r_ind)):
        if r_ind[i][0]<=2:
            if r_ind[i][-1]+10>=len(pathx):
                print('all collision')
                pass
            else:
                r_ind[i]=[j for j in range(r_ind[i][-1]+10)]   
        elif r_ind[i][-1]+10>=len(pathx): 
            r_ind[i]=[j for j in range(r_ind[i][0]-3,len(pathx))]
        else:
            r_ind[i]=[j for j in range(r_ind[i][0]-3,r_ind[i][-1]+10)]
    for i in r_ind:
        new_ind+=i
    new_ind=list(sorted(set(new_ind)))
    slow=0
    fast=0
    while fast<len(new_ind):
        fast+=1
        if fast==len(new_ind):
            t_ind+=[new_ind[slow:fast]]
        elif new_ind[fast]!=new_ind[fast-1]+1:
            t_ind+=[new_ind[slow:fast]]
            slow=fast    

    return t_ind


def first_pur(ox,oy,obs):
    kdtree = kd.KDTree([[x, y] for x, y in zip(ox, oy)])
    apathx, apathy = astar_planning(SX, SY, GX, GY, ox, oy, GRID_RESOLUTION, ROBOT_RADIUS,obs)
    astar=extract_lines(list(zip(apathx, apathy)))
    pathx, pathy=short_cutting_path(astar,obs)
    px,py=add_line(GX,GY,GYAW)
    pathx+=px
    pathy+=py
    state = VehicleState(x=SX, y=SY, yaw=SYAW, v=0.0)
    hx,hy,yaw,v,t =first_guide(pathx,pathy,state)
    R=REF(hx,hy,yaw,v,t)
    ref=list(zip(R.x,R.y))
    r,iter_c=collision_check(ref,R.yaw,kdtree,ox,oy)
    if not r:
        return 1,0,0,0,R,0
    r_ind=cut_line(r,iter_c,R,pathx,pathy)
    t_ind=combine_lines(r_ind,pathx,pathy)
    status=2
    return status,t_ind,pathx,pathy,R,kdtree

def test(ox,oy,a_state,R,kdtree,pathx,pathy,sx,sy,syaw,real_obs):
    hx,hy,yaw,v,t =first_guide(pathx,pathy,a_state)
    pres=[]
    all_res=0
    ref=list(zip(R.x,R.y))
    r,iter_c=collision_check(ref,R.yaw,kdtree,ox,oy)
    if not r:
        all_res=R
    aiter=0
    while r:
        aiter+=1
        if aiter<=500:
            res=[]
            n_res=[]
            ind=[]
            col_x=[]
            col_y=[]
            col_yaw=[]  
            for i in r:
                b=normal(i,kdtree,ox,oy)
                n_res+=b
            for i in range(len(r)):
                state=[R.x[iter_c[i]],R.y[iter_c[i]]]
                a=calc_astar(state, pathx, pathy)
                ind.append(a)
            cost=0.1
            for i in range(len(ind)):
                pathx[ind[i]]=pathx[ind[i]]+n_res[i][0]*cost
                pathy[ind[i]]=pathy[ind[i]]+n_res[i][1]*cost
            b_state = VehicleState(x=sx, y=sy, yaw=syaw, v=0.0)
            b_x,b_y,b_yaw,b_v,b_t =first_guide(pathx,pathy,b_state)
            R=REF(b_x,b_y,b_yaw,b_v,b_t)
            pres.append(R)
            ref=list(zip(R.x,R.y))
            r,iter_c=collision_check(ref,R.yaw,kdtree,ox,oy) 
            if not r:
                all_res=R
            for i in range(len(iter_c)):
                col_x.append(R.x[iter_c[i]])
                col_y.append(R.y[iter_c[i]])
                col_yaw.append(R.yaw[iter_c[i]])     
        else:
            print('path deformatin has reached maxiter')
            r=0    
    return all_res,pres
def final_plan(t_ind,pathx,pathy,R,kdtree,ox,oy):
    a_resx=[]
    a_resy=[]
    res_pres=[]
    for item in t_ind:
        pres=[]
        star=[pathx[item[0]],pathy[item[0]]]
        a=calc_astar(star, R.x, R.y)
        new_state=VehicleState(R.x[a], R.y[a], R.yaw[a], R.v[a])
        time_1 = time.time()
        new_pathx=pathx[item[0]:item[-1]+1]
        new_pathy=pathy[item[0]:item[-1]+1]
        hx,hy,yaw,v,t =cal_guide(new_pathx,new_pathy,new_state)
        a_R=REF(hx,hy,yaw,v,t)
        a_ref=list(zip(a_R.x,a_R.y))
        r,iter_c=collision_check(a_ref,a_R.yaw,kdtree,ox,oy)
        aiter=0
        while r:
            aiter+=1
            if aiter<=500:
                res=[]
                n_res=[]
                ind=[]
                for j in r:
                    b=normal(j,kdtree,ox,oy)
                    n_res+=b     
                for g in range(len(r)):
                    state=[a_R.x[iter_c[g]],a_R.y[iter_c[g]]]
                    a=calc_astar(state, new_pathx, new_pathy)
                    ind.append(a)
                cost=0.1
                for _a in range(len(ind)):
                    new_pathx[ind[_a]]=new_pathx[ind[_a]]+n_res[_a][0]*cost
                    new_pathy[ind[_a]]=new_pathy[ind[_a]]+n_res[_a][1]*cost

                anew_state=VehicleState(a_R.x[0], a_R.y[0], a_R.yaw[0], a_R.v[0])
                ax,ay,ayaw,av,at =cal_guide(new_pathx,new_pathy,anew_state)
                a_R=REF(ax,ay,ayaw,av,at)
                pres.append(a_R)
                a_ref=list(zip(a_R.x,a_R.y))
                r,iter_c=collision_check(a_ref,a_R.yaw,kdtree,ox,oy)
                if not r:
                    res_pres.append(pres)
            else:  
                print('path deformatin has reached maxiter')
                r=0
                res_pres.append(pres)
                return 0,0,0,res_pres
        a_resx.append(new_pathx)
        a_resy.append(new_pathy)
    
    return 1,a_resx,a_resy,res_pres

def pco_planning(ox,oy,real_obs,counter):
    obs=acalc_obsmap(ox, oy, ROBOT_RADIUS, XW,YW,GRID_RESOLUTION)
    time_1=time.time()
    status,t_ind,pathx,pathy,R,kdtree=first_pur(ox,oy,obs)
    if status==0:
        print("Astar failed")
        all_time="astar failed"
        all_res="astar failed"
        draw_graph.plot_graph(ox,oy,SX,SY,SYAW,GX,GY,all_res,real_obs,counter)
        return all_time,all_res
    elif status==1:
        print("first pure pursuit track is good")
        time_2=time.time()
        all_time=time_2-time_1
        all_res=R
        draw_graph.plot_graph(ox,oy,SX,SY,SYAW,GX,GY,R,real_obs,counter)
        draw_graph.plot_final_graph(ox,oy,SX,SY,SYAW,GX,GY,all_res,real_obs,counter,RB,RF,W)
        return all_time,all_res
    else:
        b_status,a_resx,a_resy,res_pres=final_plan(t_ind,pathx,pathy,R,kdtree,ox,oy)
        if b_status==0:
            print('path deformation failed')
            all_time="plan failed"
            all_res="plan failed"
            draw_graph.plot_graph(ox,oy,SX,SY,SYAW,GX,GY,res_pres,real_obs,counter)
            return all_time,all_res 
        else:
            for i in range(len(t_ind)):
                for j in range(len(t_ind[i])):
                    pathx[t_ind[i][j]]=a_resx[i][j]
                    pathy[t_ind[i][j]]=a_resy[i][j]
            state = VehicleState(x=SX, y=SY, yaw=SYAW, v=0.0)
            hx,hy,yaw,v,t =first_guide(pathx,pathy,state)
            b_R=REF(hx,hy,yaw,v,t)
            final_path=[b_R]
            res_pres.append(final_path)
            ref=list(zip(b_R.x,b_R.y))
            r,iter_c=collision_check(ref,b_R.yaw,kdtree,ox,oy)
            col_x=[]
            col_y=[]
            col_yaw=[]
            try:
                for i in range(len(iter_c)):
                    col_x.append(R.x[iter_c[i]])
                    col_y.append(R.y[iter_c[i]])
                    col_yaw.append(R.yaw[iter_c[i]]) 
            except:
                pass
            if len(r)!=0:
                print("first combine collision check failed")
                A,a_pres=test(ox,oy,state,b_R,kdtree,pathx,pathy,SX,SY,SYAW,real_obs)
                res_pres.append(a_pres)
                if A==0:
                    print("final combine collision check failed")
                    draw_graph.plot_graph(ox,oy,SX,SY,SYAW,GX,GY,res_pres,real_obs,counter)
                    all_time="failed"
                    all_res="failed"
                    return all_time,all_res
                else:
                    print("final combine collision check succeed")
                    time_3=time.time()
                    all_time=time_3-time_1
                    all_res=A
                    draw_graph.plot_graph(ox,oy,SX,SY,SYAW,GX,GY,res_pres,real_obs,counter)
                    draw_graph.plot_final_graph(ox,oy,SX,SY,SYAW,GX,GY,all_res,real_obs,counter,RB,RF,W)
                    return all_time,all_res

            else:
                print("first combine collision check succeed")
                time_4=time.time()
                all_time=time_4-time_1
                all_res=b_R
                draw_graph.plot_graph(ox,oy,SX,SY,SYAW,GX,GY,res_pres,real_obs,counter)
                draw_graph.plot_final_graph(ox,oy,SX,SY,SYAW,GX,GY,all_res,real_obs,counter,RB,RF,W)
                return all_time,all_res
  

