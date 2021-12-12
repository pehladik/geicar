#%%
import numpy as np
from algo import Node
import platform
import time
plateforme=0
if(platform.system()=="Windows"):
    import roslibpy
    plateforme=1
if(platform.system()=="Linux"):
    import rospy
    from radar_ros_msgs.msg import frame
    plateforme=2
 



#Permet de capée la valeur au supérieur et de retourner un int et non un float
#cap_val(-1.5)=-2
#cap_val(1.5)=2
def cap_val(x):
            if(x<=0):
                return(int(np.floor(x)))
            else:
                return(int(np.ceil(x)))

#Marche pour une implémentation en ROS mais a adapter
#y peut etre négatif (axe sur les cotés)
#x toujours positif (axe vers l'avant)
class Radar:
    def __init__(self,data,max_row,max_col,coeff,option):
        #Position de chaque objet (x,y) detecté par le radar
        self.pos_obs=list()
        #Taille de chaque objet (radar_cross_section)
        self.taille_obs=list()
        #Vitesse de chaque objet (vx,vy)
        self.vec_vit_obs=list()
        self.max_row=max_row
        self.max_col=max_col
        self.positions = [(i, j) for i in range(self.max_row) for j in range(self.max_col)]
        self.arr2d=np.ones((self.max_row,self.max_col),dtype=object)
        self.op=option
        self.coeff=coeff
        if(self.op=="TXT"):
            ######A CHANGER SI ON VEUT ADAPTER A ROS##########################
            data=data.split(" ")
            for obj in data:
                try:
                    elts=[float(x) for x in obj.split(",")]
                    taille,x,y,vx,vy=elts
                    self.pos_obs.append((x*self.coeff,y*self.coeff))
                    self.taille_obs.append(taille*self.coeff)
                    self.vec_vit_obs.append((vx*self.coeff,vy*self.coeff))
                except ValueError as e:
                    if(len(obj)==1):
                        continue
                    else:
                        raise Exception("Prob Radar ligne 32")
            ###################################################################
        else:
            if(plateforme==1):
                client = roslibpy.Ros(host='localhost', port=9090)
                client.run()
                self.listen=Listener(client,self.coeff,plateforme)
            elif(plateforme==2):
                self.listen=Listener(0,self.coeff,plateforme)
            


    def update(self):
        if(self.op=="ROS"):
            self.listen.update()
            time.sleep(0.2)
            self.pos_obs=self.listen.pos_obs
            self.taille_obs=self.listen.taille_obs
            self.vec_vit_obs=self.listen.vec_vit_obs
        elif(self.op=="TXT"):
            #Actuellement on ne fait rien
            pass
        #Si taille différente ca veut dire qu'un obstacle lui manque sa taille ou sa position donc erreur
        if(len(self.pos_obs)!=len(self.taille_obs)):
            raise Exception("Liste de taille différente")
        
        
        #Dans chaque case on met une node avec Node((row,col),0,0,True) et True veut dire obstacle ou non
        #Donc la il faut réussir a convertir une range de chiffre entre 0 et self.max_col pour 
        #On va déja faire un tri des obstacles detecter trop loin
        i=0
        #On va chercher en meme temps le y minimun
        min_y=np.inf
        for pos in self.pos_obs.copy():
            x=pos[0]
            y=pos[1]
            #On élimine si au dessus de 50 metre en x et si au dessus de -25 et 25 en y
            if(not((-(self.max_col-1)/2)<y<((self.max_col-1)/2)) or not(0<x<self.max_row-1)):
                self.pos_obs.pop(i)
                self.taille_obs.pop(i)
                self.vec_vit_obs.pop(i)
                i=i-1
            else:
                #Minimun y des cas valide
                if(y<min_y):
                    min_y=y
            i=i+1
        self.min=min_y
        #On ajoute et on converti en int pour avoir des pos avec des int pour le tableau
        self.pos_obs=[(int((np.round(x,0))),int(np.round((y-min_y),0))) for x,y in self.pos_obs]
        self.taille_obs=[int(taille) for taille in self.taille_obs]
        #On va floor la vitesse pour etre sur
        self.vec_vit_obs=[(cap_val(vx),cap_val(vy)) for vx,vy in self.vec_vit_obs]

        #Maintenant il faut gerer la taille de l'obstacle et la vitesse
        #init du tableau avec que des nodes falses
        for position in self.positions:
            row=position[0]
            col=position[1]
            self.arr2d[row][col]=Node((row,col),0,0,False)

        #ON NE GERE PAS LA TAILLE ENCORE CAR IL JE N'AI PAS COMPRIS L'UNITE DB/m²
        for position in self.positions:
            row=position[0]
            col=position[1]
            if((row,col) in self.pos_obs):
                ind=self.pos_obs.index((row,col))
                taille_obj=self.taille_obs[ind]
                vitesse_obj=self.vec_vit_obs[ind]
                #COMMENT OUT SI ON VEUT PAS VITESSE
                """
                voisins=self.ind_vit(row,col,*vitesse_obj)
                for v in voisins:
                    self.arr2d[v[0]][v[1]]=Node((v[0],v[1]),0,0,True)
                """
                self.arr2d[row][col]=Node((row,col),0,0,True)
                
    #Renvoie les indices des cases du tableau a mettre en obstacles car c'est la future vitesse
    def ind_vit(self,row_start,col_start,vx,vy):
        new_row=row_start+vx
        new_col=col_start+vy
        if(col_start<=new_col):
            range_vy=range(col_start,new_col+1)
        else:
            range_vy=range(new_col,col_start+1)

        if(row_start<=new_row):
            range_vx=range(row_start,new_row+1)
        else:
            range_vx=range(new_row,row_start+1)
        
        pos_voisins_obs=list()
        for row in range_vx:
            for col in range_vy:
                if(row>=self.max_row or row<0 or col>=self.max_col or col<0 or (row,col)==(row_start,col_start) ):
                    continue
                else:
                    pos_voisins_obs.append((row,col))
        return(pos_voisins_obs)

    def print_arr_2d(self):
        for row in range(self.max_row):
            for col in range(self.max_col):
                obs=self.arr2d[row][col].obstacle
                if(obs==True):
                    print("X",end='')
                else:
                    print("0",end='')
            print("")
    def get_arr2d(self):
        return(self.arr2d)


class Listener:
    def __init__(self,client,coeff,plateforme):
        self.pos_obs=list()
        self.taille_obs=list()
        self.vec_vit_obs=list()
        self.coeff=coeff
        self.plateforme=plateforme
        if(plateforme==1):
            self.listener = roslibpy.Topic(client, '/radar_frames', 'radar_ros_msgs/frame')
        elif(plateforme==2):
            rospy.init_node('listener', anonymous=True)

    def receive_message(self,msg):
        self.pos_obs=list()
        self.taille_obs=list()
        self.vec_vit_obs=list()
        if(self.plateforme==1):
            for obj in msg['objects']:
                x=obj['distance_long']
                y=obj['distance_lat']
                vx=obj['velocity_long']
                vy=obj['velocity_lat']
                taille=obj['radar_cross_section']
                self.pos_obs.append((x*self.coeff,y*self.coeff))
                self.taille_obs.append(taille*self.coeff)
                self.vec_vit_obs.append((vx*self.coeff,vy*self.coeff))
        elif(self.plateforme==2):
            for obj in msg.objects:
                x=obj.distance_long
                y=obj.distance_lat
                vx=obj.velocity_long
                vy=obj.velocity_lat
                taille=obj.radar_cross_section
                self.pos_obs.append((x*self.coeff,y*self.coeff))
                self.taille_obs.append(taille*self.coeff)
                self.vec_vit_obs.append((vx*self.coeff,vy*self.coeff))

    def update(self):
        if(plateforme==1):
            self.listener.subscribe(self.receive_message)
        elif(plateforme==2):
            msg=rospy.wait_for_message('radar_frames',frame)
            self.receive_message(msg)

if __name__=="__main__":
    print('Start')
    client = roslibpy.Ros(host='localhost', port=9090)
    client.run()
    #talk=Talker(client)
    print("PASS")
    #listen=Listener(client,1)
    #while(1):
        #listen.listener_sub()
        #time.sleep(5)
        


# %%
