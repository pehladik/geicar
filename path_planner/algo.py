import numpy as np

class Node:
    def __init__(self,pos,cost,heuristique,obstacle:bool):
        self.row,self.col=pos
        self.cost=cost
        self.heuristique=heuristique
        self.obstacle=obstacle
        self.parent=None

    def set_parent(self,n):
        #Pos du noeud parent
        self.parent=(n.row,n.col)


class Graph:
    def __init__(self,max_row,max_col,arr2d,coeff):
        self.max_row=max_row
        self.max_col=max_col
        self.arr2d=arr2d
        coeff=coeff #Chaque case = 50cm la voiture fait environ 3m longeur/1m largeur (largeur compte surtout longeur nique un peu)
        long_voit=1 #metre
        self.nbr_case_voit=int(coeff*long_voit) #si impair ex=3 rajouter +1 comme ca on a forcément de la marge ?

    def ancien(self,max_row,max_col):
        self.max_row=max_row
        self.max_col=max_col
        self.positions = [(i, j) for i in range(self.max_row) for j in range(self.max_col)]
        self.arr2d=np.ones((self.max_row,self.max_col),dtype=object)
        for row,col in self.positions:
            self.arr2d[row][col]=Node((row,col),0,0,False)

    def reconstruire_path(self,u:Node):
        parent=u.parent
        self.path=list()
        self.path.append((u.row,u.col))
        while parent!=None:
            row_par=parent[0]
            col_par=parent[1]
            #print("Pos",str(row_par)+str(col_par))
            self.path.append(parent)
            parent=self.arr2d[row_par][col_par].parent

    def voiture_collide(self,row,col,direction):
        cote_case=self.nbr_case_voit/2
        cote_case=int(np.ceil(cote_case))
        try:
            #x,y avec col→ et row↓
            for i in range(1,cote_case+1):

                obs_d=self.arr2d[row][col+i].obstacle#Droite
                obs_g=self.arr2d[row][col-i].obstacle#Gauche
                obs_b=self.arr2d[row+i][col].obstacle#BAS
                obs_h=self.arr2d[row-i][col].obstacle#HAUT

                obs_hg=self.arr2d[row-i][col-i].obstacle
                obs_hd=self.arr2d[row-i][col+i].obstacle
                obs_bg=self.arr2d[row+i][col-i].obstacle
                obs_bd=self.arr2d[row+i][col+i].obstacle
                if(obs_g==True or obs_d==True or obs_b==True or obs_h==True or obs_hd or obs_bd or obs_hg or obs_bg):
                    return(True)#Voisin not valide
        except IndexError:
            return(True)



def neighbor(n:Node): #Retourne indice des neightbors
    dirs = [[1, 0], [0, 1], [-1, 0], [0, -1],
            [-1,-1],[-1,1], [1, -1], [1 ,1]]
    dirs = [[1, 0], [0, 1], [-1, 0], [0, -1]]
    result = []
    for dir in dirs:
        result.append([n.row + dir[0], n.col + dir[1]])
    return(result)

def distance(row1,col1,row2,col2):
    return(np.sqrt((row1-row2)*(row1-row2) + (col1-col2)*(col1-col2)))

class Openlist():
    def __init__(self):
        self.l=list()
        self.curr=None

    def len(self):
        return(len(self.l))

    def add(self,elt):
        self.l.append(elt)

    def sorted_func(self,elt:Node):
        return elt.heuristique

    def sort_l(self):
        self.l=sorted(self.l,key=self.sorted_func)

    def next(self):
        if(not self.vide()):
            #Meilleur noeud de la liste car on a sorted
            self.sort_l()
            return self.l.pop(0)
    def true_if_v_exist_with_inferior_cost(self,node:Node):
        for i,n in enumerate(self.l):
            if(node.row==n.row and node.col==n.col and node.heuristique<=n.heuristique):
                #print("Pass")
                #self.l.pop(i)
                return(True)
        return(False)

    def vide(self):
        if(len(self.l)==0):
            return(True)
        else:
            return(False)

def true_if_node_in_list(node:Node,closed_l):
    for n in closed_l:
        if(node.row==n.row and node.col==n.col):
            return(True)
    return(False)



def direction(u_row,u_col,v_row,v_col):
            diff_col=v_col-u_col
            diff_row=v_row-u_row
            dep=None
            if(diff_row==0 and diff_col==0):
                raise Exception("BUG ?")
            if(diff_col==0):
                dep="DEPLACEMENT_VERTI"
            elif(diff_row==0):
                dep="DEPLACEMENT_HORI"
            if(dep==None):
                raise Exception("Prob")
            return(dep)

def shortestpath(Graph,End_node:Node,Start_node:Node):
    close_l=list()
    open_l=Openlist()
    open_l.add(Start_node)
    i=0
    while not open_l.vide():
        i=i+1
        if(i%100==0):
            print("Itération :"+str(i))
            print("Closed List len :"+str(len(close_l)))
            print("Open List len :"+str(open_l.len()))
        u:Node=open_l.next()
        if (u.row==End_node.row and u.col==End_node.col):
            Graph.reconstruire_path(u)
            return(Graph.path)
            #reconstituerChemin(u)
            #endProg
        #Return tous les voisins possibles
        voisins=neighbor(u)

        #Tri des voisins
        valid_node_voisin=[]
        for row,col in voisins:
            
            if((row<0 or col<0 or col>=Graph.max_col or row>=Graph.max_row)):
                continue
            elif Graph.arr2d[row][col].obstacle==True:
                continue
            #elif Graph.arr2d[row][col+1].obstacle==True:
                #continue
            elif Graph.voiture_collide(row,col,direction(u.row,u.col,row,col)):
                continue
            else:
                valid_node_voisin.append(Graph.arr2d[row][col])

        for v in valid_node_voisin:
            if(not (true_if_node_in_list(v,close_l) or open_l.true_if_v_exist_with_inferior_cost(v))):
                v.set_parent(u)
                v.cost=u.cost+1
                v.heuristique=v.cost+distance(v.row,v.col,End_node.row,End_node.col)
                open_l.add(v)
        close_l.append(u)

        #On sort avec une liste de Node qui sont les voisins de notre node


if __name__=="__main__":
    graph=Graph(10,10)
    end=Node((8,6),3,0,False)
    start=Node((0,0),0,0,False)
    shortestpath(graph,end,start)


                
                

            


        
        

