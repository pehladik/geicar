#%%
import numpy as np
from algo import Node
import roslibpy
#Data de la forme pos=((50,400),(300,400),(200,300),(5,10),(9,20)) taille=10,50,68,61,20,40 

def cap_val(x):
            if(x<=0):
                return(int(np.floor(x)))
            else:
                return(int(np.ceil(x)))

class Radar:
    def __init__(self,data,max_row,max_col,coeff):
        self.pos_obs=list()
        self.taille_obs=list()
        self.vec_vit_obs=list()
        data=data.split(" ")
        self.coeff=coeff
        #y=distance latérale 
        #x=axe du radar donc jamais négatif
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

        #Si taille différente ca veut dire qu'un obstacle lui manque sa taille ou sa position
        if(len(self.pos_obs)!=len(self.taille_obs)):
            raise Exception("probléme taille")
        
        self.max_row=max_row
        self.max_col=max_col
        self.positions = [(i, j) for i in range(self.max_row) for j in range(self.max_col)]
        self.arr2d=np.ones((self.max_row,self.max_col),dtype=object)
        #dans chaque case on met une node avec Node((row,col),0,0,True) et True veut dire obstacle ou non
        #Donc la il faut réussir a convertir une range de chiffre entre 0 et self.max_col pour 
        #On va déja faire un tri des obstacles detecter trop loin
        #e=list(filter(lambda x: (-100<x[0][0]<100),zip(self.pos_obs,self.taille_obs)))
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

                
       

if __name__=="__main__":
    print('Start')


# %%
