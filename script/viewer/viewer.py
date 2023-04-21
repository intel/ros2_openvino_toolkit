import sys
from PyQt5.QtWidgets import QWidget, QDesktopWidget, QApplication,QPushButton,QLabel
from PyQt5.QtGui import QPainter,QPen,QBrush,QColor
from PyQt5.QtCore import QRect
from service import reqPipelineService,getTree
from openvino_msgs.srv import *
from pipeTree import TreeNode



    
class PipelineWidget(object):
    def __init__(self,convas,name,x,y):
        self.name = name
        self.convas = convas
        self.name_label = QLabel(self.name,convas)
        self.name_label.move(x,y)
        self.run_btn = QPushButton('run pipeline     ',convas)
        self.run_btn.move(x, y+20)
        self.run_btn.clicked.connect(lambda:self.onClick(self.run_btn))

    

        self.pause_btn = QPushButton('pause pipeline',convas)

        self.pause_btn.move(x, y+50)
        self.pause_btn.clicked.connect(lambda:self.onClick(self.pause_btn))


        self.stop_btn = QPushButton('stop pipeline   ',convas)
        self.stop_btn.move(x, y+80)
        self.stop_btn.clicked.connect(lambda:self.onClick(self.stop_btn))

        self.refresh()

    def onClick(self, whichbtn):
        if whichbtn == self.run_btn:
            reqPipelineService("RUN_PIPELINE",self.name)
        elif whichbtn == self.pause_btn:
            reqPipelineService("PAUSE_PIPELINE",self.name)
        elif whichbtn == self.stop_btn:
            reqPipelineService("STOP_PIPELINE",self.name)
        self.refresh()
        
    def refresh(self):
        response = reqPipelineService("GET_PIPELINE","")
        self.convas.response = response
        for id,pipeline in enumerate(response.pipelines):
            if self.name == pipeline.name:
                if pipeline.running_status == '1':
                    self.run_btn.setEnabled(False)
                    self.pause_btn.setEnabled(False)
                    self.stop_btn.setEnabled(False)
                elif pipeline.running_status == '2':
                    self.run_btn.setEnabled(False)
                    self.pause_btn.setEnabled(True)
                    self.stop_btn.setEnabled(True)
                elif pipeline.running_status == '3':
                    self.run_btn.setEnabled(True)
                    self.pause_btn.setEnabled(False)
                    self.stop_btn.setEnabled(True)
        self.convas.update()
        


class Window(QWidget):
    

    def __init__(self,window_width=1000,window_height=800):
        super(Window,self).__init__()
        self.window_width = window_width
        self.window_height = window_height

        self.initWindow()

        self.show()
        
    def initWindow(self):               
        #set window param
        print('Waiting for pipeling service...')
        self.response = reqPipelineService("GET_PIPELINE","")
        self.window_height = len(self.response.pipelines) * 200 
        self.resize(self.window_width,self.window_height)
        self.center()
        self.setWindowTitle('Pipeline Viewer')

        for id,pipeline in enumerate(self.response.pipelines):
            PipelineWidget(self,pipeline.name,10,id*self.window_height/(len(self.response.pipelines)+1)+30)


    def paintEvent(self,event):
        response = self.response
        #response = reqPipelineService("GET_PIPELINE","")
        for id,pipeline in enumerate(self.response.pipelines):
            pipe_root = getTree(TreeNode('root'),'',pipeline)
            pipe_root.dump_graphics(self,pipeline.name,pipeline.running_status,10,id*self.window_height/(len(response.pipelines)+1) + 30,self.window_width,self.window_height)
            
  
  
    def center(self):
        
       
        qr = self.frameGeometry()
    
        cp = QDesktopWidget().availableGeometry().center()
        
        qr.moveCenter(cp)
        self.move(qr.topLeft())
        
        
if __name__ == '__main__':
    
    app = QApplication(sys.argv)
    ex = Window()
    sys.exit(app.exec_())  