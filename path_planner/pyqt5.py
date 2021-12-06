import sys
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QApplication, QHBoxLayout, QWidget, QPushButton, QVBoxLayout,QGridLayout,QSpacerItem,QSizePolicy,QLabel,QInputDialog
import numpy as np
import algo
from algo import Node
import random
from get_radar import Radar
import pickle
from path import detect_patern

class NewBouton(QPushButton):
    def __init__(self):
        QPushButton.__init__(self,'')
        self.states=['normal','obstacles','start_pt','end_pt']
        self.curr_state='normal'

    def mousePressEvent(self, QMouseEvent):
        if QMouseEvent.button() == Qt.LeftButton:
            #Si case en obstacles, on doit la faire passer en normal, dans tous les autres cas il faut la faire passer en normal
            if(self.curr_state=='normal'):
                self.curr_state='obstacles'
                self.setStyleSheet("background-color : yellow;border : 2px solid black;")
            else:
                self.curr_state='normal'
                self.setStyleSheet("background-color : white;border : 2px solid black;")

        elif QMouseEvent.button() == Qt.RightButton:
            if(self.curr_state=='normal'):
                self.curr_state='start_pt'
                self.setStyleSheet("background-color : crimson;border : 2px solid black;")
            elif(self.curr_state=='start_pt'):
                self.curr_state='end_pt'
                self.setStyleSheet("background-color : DodgerBlue;border : 2px solid black;")
            else:
                self.curr_state='normal'
                self.setStyleSheet("background-color : white;border : 2px solid black;")


class Fenetre(QWidget):
    def __init__(self):
        QWidget.__init__(self)
        self.grid = QGridLayout()
        self.grid.setHorizontalSpacing(0)
        self.grid.setVerticalSpacing(0)
        #self.setLayout(grid)
        #self.resize(1080,860)
        self.coeff=1
        self.max_row=int(40)
        self.max_col=int(40)#En gros c'est le nombre de metre qu'on veut mais il faut le multiplier par le coeff du coups
        self.positions = [(i, j) for i in range(self.max_row) for j in range(self.max_col)]
        for position in self.positions:
            button = NewBouton()
            size=50
            button.setFixedSize(15,15)
            #button.clicked.connect(button.test)
            #button.setMinimumSize(size,size)
            #button.setMaximumSize(size,size)
            button.setStyleSheet("border : 2px solid black")
            self.grid.addWidget(button, *position)

        vbox=QVBoxLayout()
        hbox=QHBoxLayout()
        vbox.addLayout(self.grid)
        button=QPushButton("Validate")
        button.clicked.connect(self.save_tab)
        hbox.addWidget(button)
        button=QPushButton("BestPath")
        button.clicked.connect(self.short_path)
        hbox.addWidget(button)
        button=QPushButton("Clean")
        button.clicked.connect(self.clean_btn)
        hbox.addWidget(button)
        button=QPushButton("Randomn")
        button.clicked.connect(self.rand_obs)
        hbox.addWidget(button)
        button=QPushButton("Get Radar")
        button.clicked.connect(self.get_radar_obs)
        hbox.addWidget(button)
        button = QPushButton('Show Dialog')
        button.clicked.connect(self.showDialog)
        hbox.addWidget(button)
        vbox.addLayout(hbox)
        hbox=QHBoxLayout()

        states=['rien','obstacles','car_start_pt','car_end_pt']
        color_name=['white','yellow','crimson','DodgerBlue']
        for color,state in zip(color_name,states):
            label =QLabel()
            label.mousePressEvent=self.print_t
            label.setText(state)
            label.setStyleSheet('color:'+color+';background-color : black;border : 2px solid black;font-weight: bold;font: 14pt;')
            hbox.addWidget(label)
        vbox.addLayout(hbox)
        self.setLayout(vbox)
        self.move(300, 150)
        self.setWindowTitle('Map')
        self.show()
        #print(btn.curr_state)
        vspacer = QSpacerItem(QSizePolicy.Minimum,QSizePolicy.Expanding)
        #grid.addItem(vspacer, 10, 0, 1, -1)

        hspacer =QSpacerItem(QSizePolicy.Expanding, QSizePolicy.Minimum)
        #grid.addItem(hspacer, 0,10, -1, 1)
    def print_t(self,event):
        print("OK")

    def showDialog(self):
        coeff, ok = QInputDialog.getInt(self, 'Coeff', 'OK',value=self.coeff)
        if ok:
            self.coeff=coeff

    def save_tab(self):
        self.arr2d=np.ones((self.max_row,self.max_col),dtype=object)
        states=['normal','obstacles','start_pt','end_pt']
        for position in self.positions:
            row=position[0]
            col=position[1]
            btn=self.grid.itemAtPosition(*position).widget()
            if(btn.curr_state=='normal'):
                self.arr2d[row][col]=Node((row,col),0,0,False)
            elif(btn.curr_state=='obstacles'):
                self.arr2d[row][col]=Node((row,col),0,0,True)
            elif(btn.curr_state=='start_pt'):
                self.start=Node((row,col),0,0,False)
                self.arr2d[row][col]=Node((row,col),0,0,False)
            elif(btn.curr_state=='end_pt'):
                self.end=Node((row,col),0,0,False)
                self.arr2d[row][col]=Node((row,col),0,0,False)

    def clean_btn(self):
        #return list of tupple (0,1)
        for position in self.positions:
            row=position[0]
            col=position[1]
            btn=self.grid.itemAtPosition(*position).widget()
            btn.curr_state='normal'
            btn.setStyleSheet("background-color : white;border : 2px solid black;")

    def get_radar_obs(self):
        #data="12,47.4,18.4,0,0 16,41.4,-43.2,0,0 -1,40.4,4.8,1.5,0.25 6,52.4,-8.6,0,0 1,7.6,-5,0,0 -6.5,23.8,-6.6,0,0.25 -14,2.6,1.6,0,0 13.5,24.2,-13.4,0,0 0.5,69.8,21.8,0,0 5,39,-11.2,0,1 7,49.8,19.2,0,0 4,46.4,-9.6,0,0.25 0.5,117,-19,0,0.25 0.5,22.4,4.6,0,0 -1.5,13,11.6,2.25,0.25 9,61,21,0,0 -4.5,43.2,6.6,1.25,0.25 2.5,22.4,-9.4,0,0 2.5,37.4,-0.6,0,0 5.5,18.4,16,0,0 -8.5,6,-2.6,-0.5,-0.25 11,85.2,24.6,0,0 4,67.4,23.2,0,0 9.5,64,21.6,0,-0.25 25,79.8,6.8,0,0 10,71.4,22.4,0,0 10,55.6,-8.4,0,0 11,62.8,21,0,0 3,58.2,-7.4,0,-0.25 -3.5,9.8,-4.2,0,0 0.5,60.2,14.4,0,0.25 6.5,54.6,-29.2,0,0 -3,14.6,-2.8,0,-0.5 10,52.6,55.2,0,0 14.5,39,17.8,0,0.5 13,31.8,16.4,0,0 8,44.4,0.8,0,0 7.5,42.4,17.6,0,0 -0.5,61.8,12.4,0,0 5.5,36.2,17,0,0 1.5,24.2,21.4,0,0 9.5,25,6.2,0,0 3.5,86.4,16.6,0,0 10.5,48,-9.2,0,0 6,113.4,21.8,0,0 8.5,54.2,19.8,0,0 12,47.4,18.4,0,0 16,41.4,-43.2,0,0 -1.5,40.4,4.8,1.5,0.25 6,52.4,-8.6,0,0 1,7.6,-5,0,0 -5,23.8,-6.6,0,0.25 -14,2.6,1.6,0,0 13.5,24.2,-13.4,0,0 0.5,69.8,21.8,0,0 5,39,-11,0,1 7.5,50,19.2,0,0 4,46.4,-9.6,0,0 0.5,117,-19,0,0.25 0.5,22.4,4.6,0,0 -1.5,13.2,11.6,2.25,0.25 8.5,61,20.8,0,-0.25 -3,43.2,6.6,1.25,0.25 2.5,22.4,-9.4,0,0 2.5,37.4,-0.6,0,0 5.5,18.4,16,0,0 -7.5,6,-2.6,-0.5,-0.25 11,85.2,24.6,0,0 4.5,67.4,23.2,0,0 9.5,64,21.6,0,-0.25 25,79.8,6.8,0,0 10,71.4,22.6,0,0 10,55.6,-8.4,0,0 11,62.8,21,0,0 3,58.2,-7.4,0,0 -2,9.8,-4.2,0,0 0,60.2,14.4,0,0.25 6.5,54.6,-29.2,0,0 -2.5,14.6,-3,0,-0.5 10,52.6,55.2,0,0 14.5,39,18,0,0.5 13,31.8,16.4,0,0 8,44.4,0.8,0,0 7.5,42.4,17.6,0,0 -0.5,61.8,12.4,0,0 7,36.2,17,0,0 1.5,24.2,21.6,0,0 -2.5,8,-1,-0.75,0 10,25,6.2,0,0 4,86.4,16.4,0,0 10,48,-9.2,0,0 6,113.4,21.8,0,0 8.5,54.2,19.8,0,0 12,47.4,18.4,0,0 16,41.4,-43.2,0,0 -0.5,40.6,5,1.5,0.25 6,52.4,-8.6,0,0 1,7.6,-5,0,0 -4.5,23.8,-6.6,0,0.25 -14,2.6,1.6,0,0 11,24.2,-13.4,0,0 0.5,69.8,22,0,0 6.5,38.8,-11.2,0,0.25 7.5,50,19.4,0,0 4,46.4,-9.6,0,0 0.5,117,-19,0,0.25 0.5,22.2,4.6,0,0 -1.5,13.4,11.8,2.25,0.25 7.5,61,20.8,0,-0.25 -3,43.4,6.6,1.25,0.25 1.5,22.4,-9.6,0,-0.25 2.5,37.4,-0.6,0,0 5.5,18.2,16.2,0,0.25 -8,6,-2.6,-0.5,-0.5 11,85.2,24.6,0,0 4.5,67.4,23.2,0,0 9.5,64,21.6,0,-0.25 25.5,79.8,6.8,0,0 10,71.4,22.6,0,0 10.5,55.6,-8.4,0,0 11,62.8,21,0,0 3.5,58.2,-7.4,0,0 -0.5,9.8,-4,0,0 -0.5,60.2,14.4,0,0.25 6.5,54.6,-29.2,0,0 -2.5,14.6,-3.2,0,-0.75 10,52.6,55.2,0,0 14.5,39.2,18,0,0.5 13.5,31.8,16.4,0,0 8,44.4,0.8,0,0 7.5,42.4,17.6,0,0.25 -0.5,61.8,12.2,0,0 6.5,36.2,17,0,0 1.5,24.2,21.6,0,0 -10.5,7.8,-0.8,-0.5,0 10,25,6.2,0,0 4.5,86.4,16.4,0,0 10,48,-9.2,0,0 1,56,-22,0,0 11.5,40.2,-45.4,0,0 5,113.4,22,0,0 8.5,54.2,19.8,0,0 12,47.4,18.4,0,0 16,41.4,-43.2,0,0 -0.5,40.8,5,1.5,0.25 6,52.4,-8.6,0,0 1,7.6,-5,0,0 -3.5,23.6,-6.6,0,0.25 -14,2.6,1.6,0,0 10,24.2,-13.4,0,0 0.5,69.8,22,0,0 7,38.8,-11.2,0,0.25 7.5,50,19.4,0,0 4,46.4,-9.6,0,0 0.5,117,-19,0,0.25 0.5,22.2,4.8,0,0 -1.5,13.6,11.8,2.25,0.25 7,61,20.8,0,-0.25 -1.5,43.6,6.6,1.25,0.25 1.5,22.4,-9.6,0,-0.25 2.5,37.4,-0.6,0,0 5.5,18,16.2,0,0.25 -7,6,-2.6,-0.5,-0.25 11.5,85.2,24.6,0,0 4.5,67.4,23.2,0,0 10,64,21.6,0,-0.25 25.5,79.8,6.8,0,0 10,71.4,22.6,0,0 10.5,55.6,-8.4,0,0 11,62.8,21,0,0 3.5,58.2,-7.4,0,0 0,9.8,-4,0,0 -0.5,60.2,14.4,0,0.25 6.5,54.6,-29.2,0,0 -2,14.6,-3.4,0,-0.75 14.5,39.2,18,0,0.5 13.5,31.8,16.4,0,0 8,44.4,0.8,0,0 7.5,42.4,17.6,0,0.25 -0.5,61.8,12.2,0,0 6.5,36.2,17,0,0 1.5,24.2,21.6,0,0 10,25,6.2,0,0 5,86.4,16.4,0,0 10,48,-9.2,0,0 1,56,-22,0,0 11.5,40.2,-45.4,0,0 5,113.4,22,0,0 8.5,54.2,19.8,0,0"
        with open("obstacles_real.txt", "r") as f:
            data=f.read()
        #data=data.replace('\n','')
        #CHANGER FONCTIONNEMENT
        rad=Radar(data,self.max_row,self.max_col,self.coeff,option="ROS")
        rad.update()
        self.arr2d=rad.get_arr2d()
        #rad.print_arr_2d()

        for position in self.positions:
            row=position[0]
            col=position[1]
            btn=self.grid.itemAtPosition(*position).widget()
            if(self.arr2d[row][col].obstacle==True):
                btn.setStyleSheet("background-color : yellow;border : 2px solid black;")
                btn.curr_state='obstacles'
            elif(self.arr2d[row][col].obstacle==False):
                btn.setStyleSheet("background-color : white;border : 2px solid black;")
                btn.curr_state='normal'
        btn=self.grid.itemAtPosition(0,int(0-rad.min)).widget()
        btn.setStyleSheet("background-color : crimson;border : 2px solid black;")
        btn.curr_state='start_pt'


    def short_path(self):
        graph=algo.Graph(self.max_row,self.max_col,self.arr2d,self.coeff)
        path=algo.shortestpath(graph,self.end,self.start)
        #save_pickle(path,"path.pickle")
        #print(path)
        #return list of tupple (0,1)
        if(path!=None):
            #Si on veut pas le path smoothing qui est globalement pas fou je trouve
            #path=detect_patern(path)
            for position in path:
                if position==None:
                    break
                row=position[0]
                col=position[1]
                #ne pas colorier les point d'arrivé et de start
                if((row==self.end.row and col==self.end.col) or (row==self.start.row and col==self.start.col)):
                    continue
                btn=self.grid.itemAtPosition(*position).widget()
                btn.setStyleSheet("background-color : blue;border : 2px solid black;")
    
    def rand_obs(self):
        self.clean_btn()
        choices=['normal','obstacles']
        distribution = [0.85, 0.15]
        for position in self.positions:
            row=position[0]
            col=position[1]
            btn=self.grid.itemAtPosition(*position).widget()
            choix=np.random.choice(choices,p=distribution)
            btn.curr_state=choix
            if(choix=='normal'):
                btn.setStyleSheet("background-color : white;border : 2px solid black;")
            elif(choix=='obstacles'):
                btn.setStyleSheet("background-color : yellow;border : 2px solid black;")

def save_pickle(obj,filename):
    with open(filename, 'wb') as handle:
        pickle.dump(obj, handle, protocol=pickle.HIGHEST_PROTOCOL)

def load_pickle(filename):
    with open(filename, 'rb') as handle:
        b = pickle.load(handle)
    return(b)

class Vehicle():
    def __init__(self) -> None:
        self.path=None
    def set_path(self,path):
        self.path=path
    def follow_path(self):
        if(self.path!=None):
            for i in range(0,len(self.path)):
                print(self.path[0])


if __name__=='__main__':
    app = QApplication.instance() 
    if not app:
        app = QApplication(sys.argv)
        
    fen = Fenetre()
    fen.show()

    app.exec_()