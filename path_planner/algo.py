import numpy as np

#This class is configured to work for 2D grid representation
class Node:
    def __init__(self,pos,cost,heuristique,obstacle:bool):
        #Each node will correspond to one case in the grid
        self.row,self.col=pos
        self.cost=cost
        self.heuristique=heuristique
        #Define if a node is walkable or not
        self.obstacle=obstacle
        #Will be usefull for the path
        self.parent=None

    def set_parent(self,n):
        #Position of the parent node
        self.parent=(n.row,n.col)


#Graph will RECEIVE a 2D tab composed of Node walkable or not and the position of each node is the index of the grid (row ↓,col →)
#The 2D TAB is created in the pyqt5
class Graph:
    def __init__(self,max_row,max_col,arr2d,coeff):
        #Dim of the 2D tab which is given by the graphical interface (PYQT5)
        self.max_row=max_row
        self.max_col=max_col
        self.arr2d=arr2d
        #Dimension du tricycle 3m->longueur/1m->largeur environ
        long_voit=1 #metre
        larg_voit=3 #metre
        #coeff correspond a la précision qu'on souhaite avoir pour chaque case avec : 
        #coeff=1 on a une case = 1m 
        #coeff=2 on a une case = 50cm
        #nbr_case_voit permet de donner la zone autour de laquelle la voiture ne doit pas avoir d'obstacles pour que le chemin
        #soit valide. Ici c'est le nombre de case totale (on ne part pas du millieu de la voiture donc si coeff=1 la voiture fait 1 case donc 1m)
        self.nbr_case_voit=int(coeff*long_voit) 

    #Permet de recontruire le chemin en partant de la node de fin et en remontant les parents de chaque node
    #Sauvegarde dans self.path le shortest path trouvé
    def reconstruire_path(self,u:Node):
        parent=u.parent
        self.path=list()
        self.path.append((u.row,u.col))
        while parent!=None:
            row_par=parent[0]
            col_par=parent[1]
            self.path.append(parent)
            parent=self.arr2d[row_par][col_par].parent

    #Renvoie un booléen qui donne l'information si le voisin trouvé est valide ou non 
    #en fonction de si la zone autour de la voiture rentre en collision avec les obstacles autour du voisins
    #pour l'instant ce n'est qu'un carré donc on ne prends pas en compte le fait que la voiture est plus longue que large
    #Possiblement ajouter la direction de la voiture pour savoir son orientation et donc largeur/longueur car ici on fait un carré
    def voiture_collide(self,row,col,direction):
        #On divise le nombre total de case par 2 comme ca on part du millieu de la voiture et
        #On check un nombre de case égaux sur les 2 cotés
        cote_case=self.nbr_case_voit/2
        cote_case=int(np.ceil(cote_case))
        try:
            #x,y avec row↓ et col→ 
            for i in range(1,cote_case+1):

                obs_d=self.arr2d[row][col+i].obstacle#Droite
                obs_g=self.arr2d[row][col-i].obstacle#Gauche
                obs_b=self.arr2d[row+i][col].obstacle#Bas
                obs_h=self.arr2d[row-i][col].obstacle#Haut

                obs_hg=self.arr2d[row-i][col-i].obstacle#Haut Gauche
                obs_hd=self.arr2d[row-i][col+i].obstacle#Haut Droite
                obs_bg=self.arr2d[row+i][col-i].obstacle#Bas Gauche
                obs_bd=self.arr2d[row+i][col+i].obstacle#Bas Droite
                #Si il y a un seul obstacle autour on dit que le voisin n'est pas valide
                if(obs_g==True or obs_d==True or obs_b==True or obs_h==True or obs_hd or obs_bd or obs_hg or obs_bg):
                    return(True)#Voisin not valide
        #Si on sort de notre zone c'est compté comme un voisin non valide
        #Les bords de la grille compté comme des "murs", comportement a changer peut etre
        except IndexError:
            return(True)


#C'est une liste avec des fonctions en plus car elle permet de renvoyer les heuritiques les plus faibles
#ainsi que de savoir si dans la liste il y a déja le noeud avec un heurtique plus faibles
class Openlist():
    def __init__(self):
        self.l=list()

    def len(self):
        return(len(self.l))

    def add(self,elt):
        self.l.append(elt)

    #Key function to sort by heuristique value
    def sorted_func(self,elt:Node):
        return elt.heuristique

    #Range la liste avec les heuritiques les plus faibles en premiers (croissant)
    def sort_l(self):
        self.l=sorted(self.l,key=self.sorted_func)

    #Renvoie et supprime le premier élement de la liste qui est la node avec l'heuriqtique la plus basse
    def next(self):
        if(not self.vide()):
            #Meilleur noeud de la liste car on a sorted
            self.sort_l()
            return self.l.pop(0)
    #Si le noeud existe déja avec un heuritiques plus faible on renvoie vrai
    def true_if_v_exist_with_inferior_cost(self,node:Node):
        for n in self.l:
            if(node.row==n.row and node.col==n.col and node.heuristique<=n.heuristique):
                return(True)
        return(False)

    def vide(self):
        if(len(self.l)==0):
            return(True)
        else:
            return(False)

#retourne les indices des voisins autour de la node (gestion des index error dans l'algo)
def neighbor(n:Node): 
    dirs = [[1, 0], [0, 1], [-1, 0], [0, -1]]
    result = []
    for dir in dirs:
        result.append([n.row + dir[0], n.col + dir[1]])
    return(result)

#Return distance euclidienne qui est notre heuristique pour A*
def distance(row1,col1,row2,col2):
    return(np.sqrt((row1-row2)*(row1-row2) + (col1-col2)*(col1-col2)))

#Fonction qui permet de savoir si le noeud est déja dans une liste (ici la closed list)
def true_if_node_in_list(node:Node,closed_l):
    for n in closed_l:
        if(node.row==n.row and node.col==n.col):
            return(True)
    return(False)


#Pas utilisé mais peut servir car techniquement elle est censer nous donner le déplacement
#entre le noeud d'avant et le voisin
def direction(u_row,u_col,v_row,v_col):
            diff_col=v_col-u_col
            diff_row=v_row-u_row
            if(diff_row==0 and diff_col==0):
                raise Exception("BUG ?")
            if(diff_col==0):
                dep="DEPLACEMENT_VERTI"
            elif(diff_row==0):
                dep="DEPLACEMENT_HORI"
            return(dep)

#Algo A*
def shortestpath(Graph,End_node:Node,Start_node:Node):
    close_l=list()
    open_l=Openlist()
    open_l.add(Start_node)
    i=0
    while not open_l.vide():
        i=i+1
        #Print pour savoir le nombre d'itération
        if(i%100==0):
            print("Itération :"+str(i))
            print("Closed List len :"+str(len(close_l)))
            print("Open List len :"+str(open_l.len()))

        #Node sur laquelle on est
        u:Node=open_l.next()
        if (u.row==End_node.row and u.col==End_node.col):
            Graph.reconstruire_path(u)
            return(Graph.path)
            #endProg

        #Return tous les voisins possibles (H,B,G,D) autour de la node actuelle
        voisins=neighbor(u)

        #Tri des voisins
        valid_node_voisin=[]
        #Ici on peut rajouter les contraintes
        for row,col in voisins:
            #Si le voisin est hors du tableau/grid
            if((row<0 or col<0 or col>=Graph.max_col or row>=Graph.max_row)):
                continue
            #Si le voisin est un obstacle
            elif Graph.arr2d[row][col].obstacle==True:
                continue
            #Si le voisin est proche d'un obstacle (dimension voiture)
            #(direction non utilisé)
            elif Graph.voiture_collide(row,col,direction(u.row,u.col,row,col)) and True:
                continue
            #Sinon la node est valide
            else:
                valid_node_voisin.append(Graph.arr2d[row][col])

        for v in valid_node_voisin:
            #Si le voisin n'est pas déja la closed_list ou si le voisin n'est pas dans la openlist avec un cout égal ou inférieur on rentre
            #et on l'ajoute a l'open list
            if(not (true_if_node_in_list(v,close_l) or open_l.true_if_v_exist_with_inferior_cost(v))):
                v.set_parent(u)
                v.cost=u.cost+1
                v.heuristique=v.cost+distance(v.row,v.col,End_node.row,End_node.col)
                open_l.add(v)
        #On ajoute la node exploré a la liste fermée
        close_l.append(u)


if __name__=="__main__":
    #Ne marche plus car le programme fonctionne seulement avec l'interface maintenant, si on veut le faire fonctionner par soi-meme
    #Il faut donner au Graph un 2D tab array avec des nodes à l'intérieur qui sont init avec des couts,heuristique=0 et obstacle=TRUE ou FALSE
    #et la pos = (row,col) de la 2D tab array
    graph=Graph(10,10)
    end=Node((8,6),3,0,False)
    start=Node((0,0),0,0,False)
    shortestpath(graph,end,start)


                
                

            


        
        

