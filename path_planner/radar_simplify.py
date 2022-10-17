import numpy as np
import platform
import time
if(platform.system()=="Windows"):
    pass
    #import roslibpy
elif(platform.system()=="Linux"):
    import rospy
    from radar_ros_msgs.msg import frame

#Classe RADAR
class Radar:
    #Init de la classe, soit on prends les données d'un TXT, soit directement de ROS
    def __init__(self,txt_filename,option="TXT"):
        self.option=option
        if(self.option=="TXT"):
            with open(txt_filename, "r") as f:
                self.data=f.read()
        if(self.option=="ROS"):
            self.listen=Listener(0)
            
    #Update, les données du radar sont position des obstacles en m, tailles en db/m et vecteur vitesse en m/s
    def update(self,cote_max,longueur_max):
        self.pos_obs=list()
        self.taille_obs=list()
        self.vec_vit_obs=list()
        if(self.option=="TXT"):
            data=self.data.split(" ")
            for obj in data:
                try:
                    elts=[float(x) for x in obj.split(",")]
                    taille,x,y,vx,vy=elts
                    #On garde que les obstacles dans une certaines zone
                    if(0<=x<longueur_max and -cote_max<y<cote_max):
                        self.pos_obs.append((x,cote_max+y))
                        self.taille_obs.append(taille)
                        self.vec_vit_obs.append((vx,vy))
                except ValueError as e:
                    if(len(obj)==1):
                        continue
                    else:
                        raise Exception("Prob Radar ligne 32")
                        
        if(self.option=="ROS"):
            self.listen.update(cote_max,longueur_max)
            time.sleep(0.2)
            self.pos_obs=self.listen.pos_obs
            self.taille_obs=self.listen.taille_obs
            self.vec_vit_obs=self.listen.vec_vit_obs

    def create_grid(self,cote_max,longueur_max,coeff):
        #255 = BLANC = PAS d'OBSTACLE
        self.grid=np.ones(((2*cote_max*coeff)+1,(longueur_max*coeff)+1))*255
        for obs in self.pos_obs:
            x=int(np.around((obs[0]*coeff)))
            y=int(np.around(obs[1]*coeff))
            self.grid[y][x]=0
        








class Listener:
    def __init__(self,client):
        self.pos_obs=list()
        self.taille_obs=list()
        self.vec_vit_obs=list()
        
        if(platform.system()=="Linux"):
            rospy.init_node('listener', anonymous=True)

    def receive_message(self,msg):
        self.pos_obs=list()
        self.taille_obs=list()
        self.vec_vit_obs=list()
        if(platform.system()=="Linux"):
            for obj in msg.objects:
                x=obj.distance_long
                y=obj.distance_lat
                vx=obj.velocity_long
                vy=obj.velocity_lat
                taille=obj.radar_cross_section
                if(0<=x<self.longueur_max and -self.cote_max<y<self.cote_max):
                    self.pos_obs.append((x,self.cote_max+y))
                    self.taille_obs.append(taille)
                    self.vec_vit_obs.append((vx,vy))

    def update(self,cote_max,longueur_max):
        self.cote_max=cote_max
        self.longueur_max=longueur_max
        if(platform.system()=="Linux"):
            msg=rospy.wait_for_message('radar_frames',frame)
            self.receive_message(msg)




if __name__=="__main__":
    cote_max=5
    longueur_max=10
    file="obstacles_real.txt"
    rad=Radar(file,"TXT")
    rad.update(cote_max,longueur_max)
    rad.create_grid(cote_max,longueur_max,1)
    print(rad.grid)


        
