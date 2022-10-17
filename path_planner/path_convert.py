import pickle
import matplotlib.pyplot as plt
import numpy as np
from CAN_BUS_listenner import CAN_listener
from algo import direction
COEFF=0.5 #50 cm par case
RAD_TO_DEG=(180.0/np.pi)


def detect_direction(prev_elt,curr_elt,cluster_option=False):
    p_row,p_col=prev_elt
    cur_row,cur_col=curr_elt
    d_row=cur_row-p_row
    d_col=cur_col-p_col
    if(d_col==0 and d_row!=0):
        if(d_row<0):
            dir="HAUT"
        elif(d_row>0):
            dir="BAS"
    elif(d_col!=0 and d_row==0):
        if(d_col<0):
            dir="GAUCHE"
        elif(d_col>0):
            dir="DROITE"
    if(d_row not in [-1,1,0] or d_col not in [-1,1,0] and cluster_option==True):
        dir=None
    return(dir)

def get_referential_vector(pt,start):
    a=start
    b=pt
    return((b[1]-a[1],b[0]-a[0]))

def find_cluster(point,all=True):
    pt_good=list()
    pt_good.append(point[0])
    if(all):
        #On ne prends pas le premier et le dernier point car i-1 et i+1
        for i in range(1,len(point)-1):
            pt_prev=point[i-1]
            pt_curr=point[i]
            pt_next=point[i+1]
            dir_next=detect_direction(pt_next,pt_curr,cluster_option=True)
            dir_prev=detect_direction(pt_curr,pt_prev,cluster_option=True)
            if(dir_next==None or dir_prev==None):
                pt_good.append(pt_curr)
        pt_good.append(point[len(point)-1])
    else:
        return(point)

    return(pt_good)

def norm(vec,coeff):
    x,y=vec
    return (np.sqrt(x*x+y*y)*coeff)

def plot_scatter(list_pt,pt_start=None):
    for elt in list_pt:
        plt.scatter(elt[1],-elt[0],s=100,color='black')
    if(pt_start!=None):
        y,x=pt_start
        plt.scatter(x,-y,s=100,color='red')
        y2,x2=list_pt[0]
        plt.plot([x,x2],[-y,-y2],'b')
    
    plt.plot([y for x,y in list_pt],[-x for x,y in list_pt],'b') 
    #plt.legend()
    plt.show()
        

def get_angle_distance_from_pt(start_pt,end_pt,vec2):
    a=start_pt
    b=end_pt
    vec1=(b[1]-a[1],b[0]-a[0])
    distance=norm(vec1,COEFF)
    #angle = arccos[(xa * xb + ya * yb) / (√(xa2 + ya2) * √(xb2 + yb2))]
    ya,xa=vec2
    yb,xb=vec1
    angle=np.arccos((xa * xb + ya * yb) / (np.sqrt(xa*xa + ya*ya) * np.sqrt(xb*xb + yb*yb)))
    if(xa*yb - ya*xb < 0):
        angle=-angle
    return(distance,angle,vec1)

def draw_vec(vec1,vec2,origin):
    origin=(origin[1],origin[0])
    point_ori = (origin[0],origin[0])
    point2 = (origin[0]+vec1[0],origin[1]+vec1[1])
    point3 = (origin[0]+vec2[0],origin[1]+vec2[1])
    x_values = [origin[0], point2[0]]
    y_values = [origin[1], point2[1]]
    plt.plot(x_values, y_values,label="Vec1")
    x_values = [origin[0], point3[0]]
    y_values = [origin[1], point3[1]]
    plt.plot(x_values, y_values,label="Referential Vector")


#Entrée : path renvoyer par l'algo A-Star (donc des cases)
#Sortie renvoie les extrémités du path et les distances entre chaque point du chemin
#ainsi que l'angle par rapport a un vecteur de reference
def main(path_n):
    #row,col format des elts dans path
    path=path_n.copy()
    path.reverse()
    prev_dir=None
    prev_elt=None
    dir_to_go=list()
    changement_point=list()
    for i,elt in enumerate(path) : #Ca commence par la derniere case pour arriver jusqu'a la premiere
        if(prev_elt!=None):
            cur_dir=detect_direction(prev_elt,elt)
            dir_to_go.append(cur_dir)
            if(prev_dir!=None):
                if(cur_dir!=prev_dir):
                    changement_point.append(prev_elt)
            prev_dir=cur_dir
            
        if(i==0):
            pass
        prev_elt=elt
    if(not changement_point):
        #Si liste vide donc que un point dans une seule direction
        distance=(len(path))*COEFF
        if(cur_dir=="DROITE"):
            angle=0.0
        elif(cur_dir=="GAUCHE"):
            angle=180.0
        elif(cur_dir=="HAUT"):
            angle=90.0
        elif(cur_dir=="BAS"):
            angle=-90.0
        return([[distance,angle]])
            
    #Print tous les points ou seulement les extrémintés du clusters
    #FAIRE ATTENTION LE PRINT EST AVEC X, -Y car le réferentiel du gui est comme cela
    point_GPS=find_cluster(changement_point,all=True)
    if(point_GPS==None):#CA SERAIT A FAIRE MAIS FLEMME
        return(0)
    car_start=path[0]
    vec_avant=(car_start[0],car_start[1]+2)
    c=get_referential_vector(vec_avant,car_start)
    st=car_start
    dist,angle,vec1=get_angle_distance_from_pt(st,point_GPS[0],c)
    #draw_vec(vec1,c,st)
    #plot_scatter(point_GPS,car_start)
    distance_each_pt=list()
    distance_each_pt.append((dist,angle*RAD_TO_DEG))
    for i in range(0,len(point_GPS)-1):
        a=point_GPS[i]
        b=point_GPS[i+1]
        dist,angle,_=get_angle_distance_from_pt(a,b,c)
        distance_each_pt.append((dist,angle*RAD_TO_DEG))
    #Renvoie une liste avec dist_angle
    return(distance_each_pt,point_GPS,car_start)



            



if __name__=="__main__":
    with open('path_test.pickle', 'rb') as handle:
        path = pickle.load(handle)
    point_trajectorie=main(path)
    print([(1.5, 0.0), (1.0, -90.0), (4.5, 0.0), (1.1180, -63.43494), (5.5, 0.0), (2.5, -90.0), (1.414213562, -135.0)])