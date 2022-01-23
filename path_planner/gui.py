#%%
import PySimpleGUI as sg
from radar_simplify import Radar
import numpy as np
from algo import Node
import algo
import pickle
import path_convert
import CAN_BUS_listenner
import matplotlib.pyplot as plt
#4 variable global 
x_expand=y_expand=1
canvas_size=(960,560)
saved_path_A_STAR=list() #Permet de sauvegarder la trajectoire trouvé par l'A_STAR

# Add a touch of color
sg.theme('DarkAmber')   
# All the stuff inside your window.
graph=sg.Graph(enable_events=True,canvas_size=canvas_size,graph_bottom_left=(0,canvas_size[0]),graph_top_right=(canvas_size[1],0),key="-GRAPH-",drag_submits=False)
layout = [  [graph],
            [sg.Text('Coeff'), sg.InputText(key="-COEFF-",default_text="2",size=(10)),
            sg.Text('Longueur Max (metre)'), sg.InputText(key="-LONG_MAX-",default_text="50",size=(10)),
            sg.Text('Cote Max (metre)'), sg.InputText(key="-COTE_MAX-",default_text="15",size=(10)),sg.Text('',key='-DIM-')],
            [sg.Button('Create_Grid_Radar'), sg.Button('Launch A_Star'),sg.Button('Next'),sg.Button('LGSVL Follow Trajectory')]
        ]



# Create the Window
window = sg.Window('Window Title', layout,resizable=True)
window.Finalize()

#Init du graph avec un rectangle blanc de la taille du canvas
graph.DrawRectangle((0,0),(canvas_size[1],canvas_size[0]),fill_color="white")
#On bind le click gauche pour le detecter (tkinter method)
graph.bind('<Button-1>', '_Left_Click')
graph.bind('<Button-3>', '_Right_Click')
#Dans une grille il y a 4 valeurs : 0 ->obstacle/ 255->rien /50 ->car_start /60->car_end
PT_START=50
PT_END=60

def get_path_to_distance(p):
    print(p)
    #with open('filename.pickle', 'wb') as handle:
        #pickle.dump(p, handle, protocol=pickle.HIGHEST_PROTOCOL)
    for elt in p:
        #Case y=row x=col
        y,x=elt



#Fonction qui permet de dessiner la grille de l'astar sur le graph
def draw_grid_on_img(draw,canvas_size,grid,draw_obs=False):
    #(x,y) pour les pixels du canvas
    #(y,x) pour l'array grid
    #Variable qu'on met en global pour pouvoir les changers hors du scope local
    global x_expand,y_expand
    size_y,size_x=grid.shape
    cv_y,cv_x=canvas_size
    #On calcul de combien doivent etre les step en x et y (en pixel du graph) pour pouvoir remplir entierement la taille du grpah
    x_expand=cv_x/size_x
    y_expand=cv_y/size_y
    #On dessine des lignes verticales
    for i in range(0,size_x):
        i=i*x_expand
        draw.DrawLine((i, 0), (i,cv_y), color="red")
    #On dessine des lignes horizontales
    for i in range(0,size_y):
        i=i*y_expand
        draw.DrawLine((0, i), (cv_x,i), color="red")

    #Si on veut dessiner les obstacles de la grille
    if draw_obs:
            for y in range(0,size_y):
                for x in range(0,size_x):
                    #si il y a un obstacles on dessine un rectangle dans une case de la grille dessiné precedemment
                    if grid[y][x]==0:
                        draw.DrawRectangle((x*x_expand, y*y_expand), (x*x_expand+x_expand,y*y_expand+y_expand),fill_color='black')

 

def plot_all(list_pt,pt_start,list_lat_long):
    list_lat=list()
    list_long=list()
    for elt in list_lat_long:
        lat,long=elt
        list_lat.append(-lat)
        list_long.append(-long)

    plt.subplot(1, 2, 1)
    plt.plot(list_long,list_lat,'-or')
    plt.xlabel("Longitude")
    plt.title("Trajectoire parcourue d'aprés le GPS")
    plt.ylabel("Latitude")

    plt.subplot(1, 2, 2)
    for elt in list_pt:
        plt.scatter(elt[1],-elt[0],s=100,color='black')
    if(pt_start!=None):
        y,x=pt_start
        plt.scatter(x,-y,s=100,color='red')
        y2,x2=list_pt[0]
        plt.plot([x,x2],[-y,-y2],'b')
    
    plt.plot([y for x,y in list_pt],[-x for x,y in list_pt],'b') 
    plt.xlabel("X")
    plt.title("Trajectoire à suivre")
    plt.ylabel("Y")
    plt.show()

while True:
    event, values = window.read()
    if event == sg.WIN_CLOSED:
        break  # exit
    #CLICK GAUCHE
    if event=="-GRAPH-_Left_Click": #Permet de définir le point de début et de fin à la main
        x,y=map(int,values["-GRAPH-"]) #Recuperation de la position du click
        #Notre y et x récupérer sont en coordonées du graph donc entre 0 et 640 sauf que nous on veut avoir la position de la case
        #de la grille cliqué donc il faut diviser par le _expand et convertir en int pour garder que l'intervalle souhaité
        y_grid=int(y/y_expand)
        x_grid=int(x/x_expand)
        #On affiche la case qu'on a cliqué sur la grille
        print("CASE :",y_grid,x_grid)
        res=rad.grid[y_grid][x_grid]# valeur sur cette case

        #En fonction du résultat on va dessiner le point de début ou de fin si il n'y a pas un obstacle dessus
        if(res==0):
            pass
        elif(res==255):
            rad.grid[y_grid][x_grid]=PT_START
            graph.DrawRectangle((x_grid*x_expand,y_grid*y_expand),(x_grid*x_expand+x_expand,y_grid*y_expand+y_expand),fill_color='green',line_color='red')
        elif(res==PT_START):
            rad.grid[y_grid][x_grid]=PT_END
            graph.DrawRectangle((x_grid*x_expand,y_grid*y_expand),(x_grid*x_expand+x_expand,y_grid*y_expand+y_expand),fill_color='red',line_color='red')
        elif(res==PT_END):
            rad.grid[y_grid][x_grid]=255
            graph.DrawRectangle((x_grid*x_expand,y_grid*y_expand),(x_grid*x_expand+x_expand,y_grid*y_expand+y_expand),fill_color='white',line_color='red')
    #WIP
    if event=='LGSVL Follow Trajectory':
        #Prochain groupe refaite le par vous meme d'une facon plus propre
        #Ca permettait de prendre le chemin A-Star et l'envoyer sur LGSVL pour deplacer la voiture
        #et a la fin on plot le vrai chemin effectué sous LGSVL avec leGPS de LGSVL par rapport a celui qu'on voulait
        t,point_GPS,car_start=path_convert.main(path)
        print(t)
        coord_gps=CAN_BUS_listenner.main(t)
        plot_all(point_GPS,car_start,coord_gps)
    #WIP
    if event=='Next': 
        #C'était censer etre une fonction qui permet de prendre en compte l'orientation de la voiture 
        #pour que le chemin donné par le GUI soit bon meme quand la voiture tourne ce qui n'est pas le cas actuellement
        #get_path_to_distance(path_follow)
        prev_pos=path_follow.pop()
        pos=path_follow[-1]
        #dir = (y,x) si y=1 -> on descend si y=-1 on monte si x=1 on va a gauche si x=-1 on va a droite
        dir=(pos[0]-prev_pos[0],pos[1]-prev_pos[1])
        str_dir=""
        if(dir[0]==1):
            str_dir="BAS"
        elif(dir[0]==-1):
            str_dir="HAUT"
        elif(dir[1]==1):
            str_dir="GAUCHE"
        elif(dir[1]==-1):
            str_dir="DROITE"
        if "id_list" in locals():
            for id in id_list:
                graph.delete_figure(id)
        id_list=list()
        for i in range(len(path_follow)):
            path_follow[i]=(path_follow[i][0]-dir[0],path_follow[i][1]-dir[1])
            y=path_follow[i][0]-start_pt.row
            x=path_follow[i][1]-start_pt.col
            
            y_grid=path_follow[i][0]
            x_grid=path_follow[i][1]
            if(str_dir=="BAS"):
                x_grid=y+start_pt.col
                y_grid=-x+start_pt.row
            if(str_dir=="HAUT"):
                x_grid=-y+start_pt.col
                y_grid=x+start_pt.row
            if(str_dir=="DROITE"):
                x_grid=-x+start_pt.col
                y_grid=-y+start_pt.row
            if(str_dir=="GAUCHE"):
                x_grid=x+start_pt.col
                y_grid=y+start_pt.row
            x_p=(x_grid*x_expand,y_grid*y_expand)
            y_p=(x_grid*x_expand+x_expand,y_grid*y_expand+y_expand)
            id=graph.DrawRectangle(x_p,y_p,fill_color='purple',line_color='red')
            id_list.append(id)
        print(str_dir)
        
    if event=="-GRAPH-_Right_Click": #Meme chose que Left_Click mais dessine des obstacles
        x,y=map(int,values["-GRAPH-"])
        y_grid=int(y/y_expand)
        x_grid=int(x/x_expand)
        print("CASE :",y_grid,x_grid)
        res=rad.grid[y_grid][x_grid]
        if(res==0):
            rad.grid[y_grid][x_grid]=255
            graph.DrawRectangle((x_grid*x_expand,y_grid*y_expand),(x_grid*x_expand+x_expand,y_grid*y_expand+y_expand),fill_color='white',line_color='red')
        else:
            rad.grid[y_grid][x_grid]=0
            graph.DrawRectangle((x_grid*x_expand,y_grid*y_expand),(x_grid*x_expand+x_expand,y_grid*y_expand+y_expand),fill_color='black',line_color='black')

    if event=='Launch A_Star': 
        #Lance l'algorithme A_Start à partir de la grille comprenant (0,255,60,50) pour créer une grille 
        #de meme dimension mais avec des Node Object pour faire le calcul A_star
        #Transform Grid in Node grid
        node_grid=np.ones((rad.grid.shape),dtype=object)
        for row,_ in enumerate(rad.grid):
            for col,elt in enumerate(_):
                #Normal
                if(elt==255):
                    node_grid[row,col]=Node((row,col),0,0,False)
                elif(elt==PT_START):
                    start_pt=Node((row,col),0,0,False)
                    node_grid[row,col]=Node((row,col),0,0,False)
                elif(elt==PT_END):
                    end_pt=Node((row,col),0,0,False)
                    node_grid[row,col]=Node((row,col),0,0,False)
                elif(elt==0):
                    node_grid[row,col]=Node((row,col),0,0,True)
        max_row,max_col=node_grid.shape
        graph_algo=algo.Graph(max_row,max_col,node_grid,coeff)
        path=algo.shortestpath(graph_algo,end_pt,start_pt)
        for id in saved_path_A_STAR:
            graph.delete_figure(id)
        saved_path_A_STAR=list()
        if(path!=None):
            path_follow=path.copy()
            for position in path:
                y_grid,x_grid=position
                if position==None:
                    break
                if(position==(end_pt.row,end_pt.col) or position==(start_pt.row,start_pt.col)):
                    continue
                id=graph.DrawRectangle((x_grid*x_expand,y_grid*y_expand),(x_grid*x_expand+x_expand,y_grid*y_expand+y_expand),fill_color='yellow',line_color='red')
                saved_path_A_STAR.append(id)
        else:
            print("Pas de chemin possible")

    if event=='Create_Grid_Radar':
        graph.DrawRectangle((0,0),(canvas_size[1],canvas_size[0]),fill_color="white")
        coeff=int(values['-COEFF-'])
        cote_max=int(values['-COTE_MAX-'])
        longueur_max=int(values['-LONG_MAX-'])
        txt="1 case = "+str(1/coeff)+"m"
        window['-DIM-'].update(txt)
        file="obstacles_real.txt"
        rad=Radar(file,"TXT")
        rad.update(cote_max,longueur_max)
        rad.create_grid(cote_max,longueur_max,coeff)
        #Draw_Start_Point
        draw_grid_on_img(graph,canvas_size,rad.grid,draw_obs=True)
        x_grid=0
        y_grid=int(cote_max*coeff)
        rad.grid[y_grid][0]=PT_START
        graph.DrawRectangle((x_grid*x_expand,y_grid*y_expand),(x_grid*x_expand+x_expand,y_grid*y_expand+y_expand),fill_color='green',line_color='red')

        





window.close()
# %%
