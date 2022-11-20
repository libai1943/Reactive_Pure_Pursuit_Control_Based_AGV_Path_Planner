import math
def add_path(x,y,s=0):
    for i in range(len(x)-1):
        ds=math.sqrt((x[i+1]-x[i])**2+(y[i+1]-y[i])**2)
        s+=ds
    return s    