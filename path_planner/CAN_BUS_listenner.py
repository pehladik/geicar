import roslibpy
import time
import numpy as np
import matplotlib.pyplot as plt
MAX_VALUE_CPT=3
DEG_TO_RAD = np.pi/180
TIME_SLEEP=0.2
def find_offset_quaternion(quaternion_depart):
    q1_c,q2_c=np.abs(quaternion_depart)
    alpha_t=[(np.pi/1000)*i for i in range(0,1000)]
    for alpha in alpha_t:
        q2=np.sin(alpha/2-np.pi)*1
        q1=np.cos(alpha/2-np.pi)*1
        q1=np.abs(q1)
        q2=np.abs(q2)
        if (np.abs(q1_c-q1)<0.001 and np.abs(q2_c-q2)<0.001):
                d=np.around(alpha*(1/DEG_TO_RAD),3)
                break
    return d
def calcul_distance (latitude_1, longitude_1, latitude_2, longitude_2):
    # Conversion latitude et longitude en radian
    latRad_1 = latitude_1*DEG_TO_RAD
    lonRad_1 = longitude_1*DEG_TO_RAD
    latRad_2 = latitude_2*DEG_TO_RAD
    lonRad_2 = longitude_2*DEG_TO_RAD
    distance = round(6371000*np.arccos(np.sin(latRad_1)*np.sin(latRad_2) + np.cos(latRad_1)*np.cos(latRad_2)*np.cos(lonRad_2-lonRad_1)), 3)
    #print("Distance en mètres entre les deux points : "+str(distance)+" mètres")
    return distance
class CAN_listener:
    def __init__(self,client):
        self.gps_listener = roslibpy.Topic(client, '/canbus', 'lgsvl_msgs/CanBusData')
        self.seq = 0
        self.can = {}
        self.speed=0
        self.orientation=0
    
    def receive_message(self,msg):
        self.can = msg
        self.speed=msg["speed_mps"]
        self.orientation=msg["orientation"]
        self.latitude=msg["gps_latitude"]
        self.longitude=msg["gps_longitude"]

    def listener_sub(self):
        self.gps_listener.subscribe(self.receive_message)

class Talker:
    def __init__(self,client):
        self.talker = roslibpy.Topic(client, '/lgsvl_cmd', 'lgsvl_msgs/VehicleControlData')

    def talker_pub(self,acceleration_pct,braking_pct,target_wheel_angle,target_wheel_angular_rate,target_gear):
        self.talker.publish(
            roslibpy.Message(
            {"header": roslibpy.Header(stamp=roslibpy.Time.now(), frame_id='',seq=0),
            'acceleration_pct': acceleration_pct,
            'braking_pct': braking_pct,
            'target_wheel_angle': target_wheel_angle,
            'target_wheel_angular_rate': target_wheel_angular_rate,
            'target_gear': target_gear
            }))


#orientation x,y,z,w : 
# TOUT DROIT = 0,0,1.0,0
# ARRIERE = 0,0,0,-1.0

def turn_left(talk,sub,dir=None,inv=False):
    if(dir==None):
        dir=np.array([0.7,0.7],dtype=float)
    #dir=np.array([0.2,1.0],dtype=float)
    if(inv==False):
        change=-1
    else:
        change=1
    cpt=0
    one_time=True
    curr_ori=np.array([0,0],dtype=float)
    while(1):
        sub.listener_sub()
        if sub.orientation!=0:
            z=sub.orientation["z"]
            w=sub.orientation["w"]
            curr_ori[0]=np.abs(z)
            curr_ori[1]=np.abs(w)
            err=dir-curr_ori
            err_abs=np.sum(np.abs(err))
            signe=np.sign(err[0])
            #Invariant de la gauche : tous les chiffres positive ou negative
            if((np.sign(z)==-1 and np.sign(w)==-1) or (np.sign(z)==1 and np.sign(w)==1)):
                left=True
            else:
                left=False
            #On va se fixer un erreur <0.02
            if(err_abs>0.05 or left==False):
                if(signe==change and left):
                    talk.talker_pub(0.2,0,err_abs*np.pi,0.01,1)
                elif(signe==-change and left):
                    talk.talker_pub(0.2,0,err_abs*-np.pi,0.01,1)
                elif(left==False):
                    if one_time==True:
                        if(curr_ori[0]>curr_ori[1]):
                            angle_sign=1
                        else:
                            angle_sign=-1
                        one_time=False
                    talk.talker_pub(0.2,0,angle_sign*np.pi,0.01,1)
                cpt=0
            elif err_abs<=0.05 and left:
                cpt=cpt+1
                if(cpt>=MAX_VALUE_CPT):
                    break
            time.sleep(TIME_SLEEP)
    return("Left")

def turn_right(talk,sub,dir=None,inv=False):
    if(dir==None):
        dir=np.array([0.7,0.7],dtype=float)
    if(inv==False):
        change=1
    else:
        change=-1
    cpt=0
    one_time=True
    curr_ori=np.array([0,0],dtype=float)
    while(1):
        sub.listener_sub()
        if sub.orientation!=0:
            z=sub.orientation["z"]
            w=sub.orientation["w"]
            curr_ori[0]=np.abs(z)
            curr_ori[1]=np.abs(w)
            err=dir-curr_ori
            err_abs=np.sum(np.abs(err))
            signe=np.sign(err[0])
            #Invariant de la gauche : tous les chiffres positive ou negative
            if((np.sign(z)==1 and np.sign(w)==-1) or (np.sign(z)==-1 and np.sign(w)==1)):
                right=True
            else:
                right=False
            #On va se fixer un erreur <0.02
            if(err_abs>0.05 or right==False):
                if(signe==change and right):
                    talk.talker_pub(0.2,0,err_abs*np.pi,0.01,1)
                elif(signe==-change and right):
                    talk.talker_pub(0.2,0,err_abs*-np.pi,0.01,1)
                elif(right==False):
                    if one_time==True:
                        if(curr_ori[0]>curr_ori[1]):
                            angle_sign=-1
                        else:
                            angle_sign=1
                        one_time=False
                    talk.talker_pub(0.2,0,angle_sign*np.pi,0.01,1)
                cpt=0
            elif err_abs<=0.05 and right:
                cpt=cpt+1
                if(cpt>=MAX_VALUE_CPT):
                    break
            time.sleep(TIME_SLEEP)
    return("Right")

def turn_avant(talk,sub,dir=None,inv=False):
    if(dir==None):
        dir=np.array([1,0],dtype=float)
    if(inv==False):
        change=-1
    else:
        change=1
    cpt=0
    one_time=True
    curr_ori=np.array([0,0],dtype=float)
    while(1):
        sub.listener_sub()
        if sub.orientation!=0:
            z=sub.orientation["z"]
            w=sub.orientation["w"]
            curr_ori[0]=np.abs(z)
            curr_ori[1]=np.abs(w)
            err=dir-curr_ori
            err_abs=np.sum(np.abs(err))
            signe=np.sign(z*w)
           
            #Invariant de l'avant
            if(curr_ori[0]>curr_ori[1]):
                right=True
            else:
                right=False
            #On va se fixer un erreur <0.02
            if(err_abs>0.05 or right==False):
                if(signe==change and right):
                    talk.talker_pub(0.2,0,err_abs*np.pi,0.01,1)
                elif(signe==-change and right):
                    talk.talker_pub(0.2,0,err_abs*-np.pi,0.01,1)
                elif(right==False):
                    if one_time==True:
                        if(np.sign(z*w)==-1):
                            angle_sign=1
                        else:
                            angle_sign=-1
                        one_time=False
                    talk.talker_pub(0.2,0,angle_sign*np.pi,0.01,1)
                cpt=0

            elif err_abs<=0.05 and right:
                cpt=cpt+1
                if(cpt>=MAX_VALUE_CPT):
                    break
            time.sleep(TIME_SLEEP)
    return("Avant")

def turn_arriere(talk,sub,dir=None,inv=False):
    if(dir==None):
        dir=np.array([0,1],dtype=float)
    if(inv==False):
        change=1
    else:
        change=-1
    cpt=0
    one_time=True
    curr_ori=np.array([0,0],dtype=float)
    while(1):
        sub.listener_sub()
        if sub.orientation!=0:
            z=sub.orientation["z"]
            w=sub.orientation["w"]
            curr_ori[0]=np.abs(z)
            curr_ori[1]=np.abs(w)
            err=dir-curr_ori
            err_abs=np.sum(np.abs(err))
            signe=np.sign(z*w)
            #Invariant de l'avant
            if(curr_ori[0]<curr_ori[1]):
                right=True
            else:
                right=False
            #On va se fixer un erreur <0.02
            if(err_abs>0.05 or right==False):
                if(signe==change and right):
                    talk.talker_pub(0.2,0,err_abs*np.pi,0.01,1)
                elif(signe==-change and right):
                    talk.talker_pub(0.2,0,err_abs*-np.pi,0.01,1)
                elif(right==False):
                    if one_time==True:
                        if(np.sign(z*w)==-1):
                            angle_sign=-1
                        else:
                            angle_sign=1
                        one_time=False
                    talk.talker_pub(0.2,0,angle_sign*np.pi,0.01,1)
                cpt=0
            elif err_abs<=0.05 and right:
                cpt=cpt+1
                if(cpt>=MAX_VALUE_CPT):
                    break
            time.sleep(TIME_SLEEP)
    return("Arriere")

def transform_angle_to_quaternion(alpha):
    alpha=DEG_TO_RAD*alpha
    q2=np.sin(alpha/2-np.pi)*1
    q1=np.cos(alpha/2-np.pi)*1
    return((np.abs(q1),np.abs(q2)))

def forward_m(talk,sub,dist,speed):
    consigne_speed=speed
    start=time.time()
    #dist en metre
    #speed en m/s
    #t=m/(m/3,6s)
    t=dist/speed
    print(t)
    while(1):
        sub.listener_sub()
        curr_speed=sub.speed
        err=consigne_speed-curr_speed
        if(err>0.5):
            talk.talker_pub(0.3,0,0,0,1)
        elif(err<-0.5):
            talk.talker_pub(0,0.3,0,0,1)
        time.sleep(0.5)
        end=time.time()
        deltat=end-start
        if(t<deltat):
            #On FREINE
            talk.talker_pub(0,1,0,0,1)
            break

        

def forward_gps(talk,sub,dist,speed,lat_start,long_start,coord_gps):
    consigne_speed=speed
    while(1):
        sub.listener_sub()
        curr_speed=sub.speed
        curr_lat=sub.latitude
        curr_long=sub.longitude
        coord_gps.append((curr_lat,curr_long))
        curr_dist=calcul_distance(lat_start,long_start,curr_lat,curr_long)
        err=consigne_speed-curr_speed
        if(err>0.5):
            talk.talker_pub(0.3,0,0,0,1)
        elif(err<-0.5):
            talk.talker_pub(0,0.3,0,0,1)
        time.sleep(TIME_SLEEP)
        if(curr_dist>dist):
            #On FREINE
            talk.talker_pub(0,1,0,0,1)
            print("Erreur distance (%) :",((curr_dist-dist)/(curr_dist))*100)
            break
def get_lat_long(sub):
    sub.listener_sub()
    time.sleep(0.5)
    lat_s=sub.latitude
    long_s=sub.longitude
    return(lat_s,long_s)

def plot_gps_coord(list_lat_long):
    list_lat=list()
    list_long=list()
    for elt in list_lat_long:
        lat,long=elt
        list_lat.append(-lat)
        list_long.append(-long)
    plt.plot(list_long,list_lat,'-or')
    plt.xlabel("Longitude")
    plt.title("Trajectoire parcourue d'aprés le GPS")
    plt.ylabel("Latitude")
    plt.show()


def main(point_traj):
    client = roslibpy.Ros(host='localhost', port=9090)
    client.run()
    sub=CAN_listener(client)
    talk=Talker(client)
    #On sait qu'on commence en avant sur la map ori=(1,0)
    ori_prec="Avant"
    choix=0
    coord_gps=list()
    while client.is_connected:
        #choix=int(input("Choix 1:Left 2:Right 3:Avant 4:Arriere 5:Gauche Relatif 6:Droite Relatif 7:GPS :"))
        if(choix==1):
            sub.listener_sub()
            time.sleep(0.5)
            z=sub.orientation["z"]
            w=sub.orientation["w"]
            ori=[z,w]
            off=find_offset_quaternion(ori)
            new_dir=transform_angle_to_quaternion(off)
            ori_prec=turn_left(talk,sub,new_dir)
        elif(choix==2):
            ori_prec=turn_right(talk,sub)
        elif(choix==3):
            ori_prec=turn_avant(talk,sub)
        elif(choix==4):
            ori_prec=turn_arriere(talk,sub)
        elif(choix==5 or choix==6):
            if(ori_prec=="Avant"):
                if(choix==5):
                    ori_prec=turn_right(talk,sub)
                elif(choix==6):
                    ori_prec=turn_left(talk,sub)
            elif(ori_prec=="Left"):
                if(choix==5):
                    ori_prec=turn_avant(talk,sub)
                elif(choix==6):
                    ori_prec=turn_arriere(talk,sub)
            elif(ori_prec=="Right"):
                if(choix==5):
                    ori_prec=turn_arriere(talk,sub)
                elif(choix==6):
                    ori_prec=turn_avant(talk,sub)
            elif(ori_prec=="Arriere"):
                if(choix==5):
                    ori_prec=turn_left(talk,sub)
                elif(choix==6):
                    ori_prec=turn_arriere(talk,sub)
        elif(choix==7):
            sub.listener_sub()
            time.sleep(0.5)
            lat_s=sub.latitude
            long_s=sub.longitude
            
            forward_gps(talk,sub,75,3,lat_s,long_s)
        if(True):
            if(False):
                sub.listener_sub()
                time.sleep(0.5)
                z=sub.orientation["z"]
                w=sub.orientation["w"]
                ori=[z,w]
                off=find_offset_quaternion(ori)
            else:
                off=0
            for dist,angle in point_traj:
                dist=dist*5
                if(angle==0):#POS DE BASE (TT DROIT)
                    lat_s,long_s=get_lat_long(sub)
                    ori_prec=turn_avant(talk,sub)
                    coord_gps.append((lat_s,long_s))
                    forward_gps(talk,sub,dist,3,lat_s,long_s,coord_gps)
                elif(angle==-90.0 or angle<0):#LEFT
                    if(angle!=-90.0):
                        dir_new=transform_angle_to_quaternion(angle)
                    else:
                        dir_new=[0.7,0.7]
                    lat_s,long_s=get_lat_long(sub)
                    coord_gps.append((lat_s,long_s))
                    ori_prec=turn_left(talk,sub,dir_new)
                    forward_gps(talk,sub,dist,3,lat_s,long_s,coord_gps)
                elif(angle==90.0 or 180.0>angle>0):#RIGHT
                    if(angle!=90.0):
                        dir_new=transform_angle_to_quaternion(angle)
                    else:
                        dir_new=[0.7,0.7]
                    lat_s,long_s=get_lat_long(sub)
                    coord_gps.append((lat_s,long_s))
                    ori_prec=turn_right(talk,sub,dir_new)
                    forward_gps(talk,sub,dist,3,lat_s,long_s,coord_gps)
                elif(angle==180):
                    lat_s,long_s=get_lat_long(sub)
                    coord_gps.append((lat_s,long_s))
                    ori_prec=turn_arriere(talk,sub)
                    forward_gps(talk,sub,dist,3,lat_s,long_s,coord_gps)
                
            #plot_gps_coord(coord_gps)
            break
    return(coord_gps)

if __name__=="__main__":
    #point_traj=[(1.5, 0.0), (1.0, -90.0), (5.0, 0.0), (1.0, -90.0), (5.5, 0.0), (3.5, -90.0), (1.0, 180.0)]
    point_traj=[(1.5, 0.0), (1.0, -90.0), (4.5, 0.0), (1.1180, -63.43494), (5.5, 0.0), (2.5, -90.0), (1.414213562, -135.0)]
    main(point_traj)
