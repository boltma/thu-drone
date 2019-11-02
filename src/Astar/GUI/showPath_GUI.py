"""
运行此程序可交互式查看路径规划算法结果：

$ python showPath_GUI.py

你只需要在同一目录下findPath.py文件中实现以下函数：
--------------------------------------------------
def findPath(map, startPosition, goalPosition):
    ...
    return path

map: 一个矩阵，'1'元素表示有障碍物，'0'表示没有
startPosition: 一个tuple，存放起点(x,y)坐标
goalPosition: 一个tuple，存放终点(x,y)坐标
return: 一个list，每个元素表示路径上的一个点，例如
    [(1,2),(1,3),(2,4)]
--------------------------------------------------
findPath函数中可以采用任意路径规划算法，只需要按照
上述格式返回计算得到的路径，比如你可以对 Astar_demo 
中的代码稍加修改，将其封装为findPath()函数
"""

import numpy as np
from tkinter import Tk, Canvas, PhotoImage, Toplevel, Menu, IntVar, Frame, Button, Scale, messagebox, HORIZONTAL, RIGHT
from findPath import findPath

global imgs

class showPathGUI():
    def __init__(self):
        self.MAP_WIDTH = 15
        self.MAP_HEIGHT =14
        self.BLOCK_SIZE = 40
        self.X_OFFSET = 0
        self.Y_OFFSET = 0
        self.BOARD_WIDTH = self.BLOCK_SIZE*self.MAP_WIDTH + 2*self.X_OFFSET
        self.BOARD_HEIGHT = self.BLOCK_SIZE*self.MAP_HEIGHT + 2*self.Y_OFFSET

        self.new_window(destroy=False)
        self.plotImg()

    
    def new_window(self, destroy=True):
        if destroy:
            self.root.destroy()
        
        # initialize map
        self.mapmap = np.zeros((self.MAP_WIDTH,self.MAP_HEIGHT),dtype=np.int)
        self.startPosition = (2, 0)    #Initial point
        self.goalPosition = (self.MAP_WIDTH-2, self.MAP_HEIGHT-3)   #End point
        
        # create window
        self.root = Tk()
        self.root.resizable(width=False, height=False)
        self.root.title('路径规划')
        
        self.WIN_WIDTH = self.root.winfo_screenwidth()
        self.WIN_HEIGHT = self.root.winfo_screenheight()
        self.MAX_WIDTH = int((self.WIN_WIDTH)/self.BLOCK_SIZE)
        self.MAX_HEIGHT = int((self.WIN_HEIGHT-100)/self.BLOCK_SIZE)
        
        # create menubar
        self.menubar = Menu(self.root)
        self.menubar.add_command(label='Setting', command=self.setup_config)
        self.menubar.add_command(label='Help', command=self.help)
        self.root.config(menu=self.menubar)
        
        # create Canvas
        self.cv = Canvas(self.root, background='white', width=self.BOARD_WIDTH, height=self.BOARD_HEIGHT)
        self.cv.pack()
        self.cv.bind('<Button-1>', self.click_handler)
        self.cv.bind('<Double-Button-1>', self.dclick1_handler)
        self.cv.bind('<Button-3>', self.dclick3_handler)
        
        global imgs
        imgs = [PhotoImage(file="images/000.png"),
           PhotoImage(file="images/111.png"),
           PhotoImage(file="images/444.png"),
           PhotoImage(file="images/ppp.png"),
           PhotoImage(file="images/222.png")]
    
    def click_handler(self,event):
        # compute selected node
        userX = max(0, min(int((event.x - self.X_OFFSET) / self.BLOCK_SIZE), self.MAP_WIDTH-1))
        userY = max(0, min(int((event.y - self.Y_OFFSET) / self.BLOCK_SIZE), self.MAP_HEIGHT-1))
        if (userX,userY)==self.startPosition or (userX,userY)==self.goalPosition:
            return
        self.mapmap[userX][userY] = 1 - self.mapmap[userX][userY]
        self.plotImg()
        
    def dclick1_handler(self,event):
        userX = max(0, min(int((event.x - self.X_OFFSET) / self.BLOCK_SIZE), self.MAP_WIDTH-1))
        userY = max(0, min(int((event.y - self.Y_OFFSET) / self.BLOCK_SIZE), self.MAP_HEIGHT-1))
        self.mapmap[self.startPosition[0],self.startPosition[1]]=0
        self.startPosition = (userX, userY)
        self.plotImg()
    
    def dclick3_handler(self,event):
        userX = max(0, min(int((event.x - self.X_OFFSET) / self.BLOCK_SIZE), self.MAP_WIDTH-1))
        userY = max(0, min(int((event.y - self.Y_OFFSET) / self.BLOCK_SIZE), self.MAP_HEIGHT-1))
        self.goalPosition = (userX, userY)
        self.plotImg()

    def plotImg(self):
        ##################################################################
        # The function you need to implement in `findPath.py`.
        path = findPath(self.mapmap, self.startPosition, self.goalPosition)
        ##################################################################
        newMap = self.mapmap.copy()
        newMap[self.startPosition[0],self.startPosition[1]] = 2
        newMap[self.goalPosition[0],self.goalPosition[1]] = 4
        for node in path:
            newMap[node[0],node[1]] = 3
        for i in range(self.MAP_WIDTH):
            for j in range(self.MAP_HEIGHT):
                self.cv.create_image((i+1/2) * self.BLOCK_SIZE + self.X_OFFSET,
                                     (j+1/2) * self.BLOCK_SIZE + self.Y_OFFSET,
                                     image=imgs[newMap[i,j]])
    
    def setup_config(self):
        # setup configure of map
        res = self.ask_userinfo()
        if res is None:
            return
        self.MAP_WIDTH, self.MAP_HEIGHT = res
        self.BOARD_WIDTH = self.BLOCK_SIZE*self.MAP_WIDTH + 2*self.X_OFFSET
        self.BOARD_HEIGHT = self.BLOCK_SIZE*self.MAP_HEIGHT + 2*self.Y_OFFSET
        self.new_window()
        self.plotImg()
        
    def ask_userinfo(self):
        # receive configure data
        current_parameter = [self.MAP_WIDTH, self.MAP_HEIGHT]
        setting_info = Setting(self.MAX_WIDTH, self.MAX_HEIGHT, current_parameter)
        self.root.wait_window(setting_info)
        return setting_info.userinfo
    
    def help(self):
        messagebox.showinfo(title='Hi,这是你需要的帮助（^-^）',
                            message='''黄色：起点\n红色：终点\n蓝色：路径\n试着点击一下地图！\n然后双击地图！\n然后再右键单击！''')
        
        
class Setting(Toplevel):
    def __init__(self, maxCol, maxRow, data):
        super().__init__()
        self.title('设置地图尺寸')
        self.MAX_COL = maxCol
        self.MAX_ROW = maxRow
        self.col = IntVar()
        self.col.set(data[0])
        self.row = IntVar()
        self.row.set(data[1])
        self.userinfo = [self.col.get(), self.row.get()]
        self.setup_UI()
        
    def setup_UI(self):
        row1 = Frame(self)
        row1.pack(fill="x")
        scale_col = Scale(row1, label='列', from_=5, to=self.MAX_COL,
                                orient=HORIZONTAL, length=200, showvalue=1,
                                tickinterval=3, resolution=1, variable=self.col)
        scale_col.set(self.col.get())
        scale_col.pack(side='top',expand='YES',fill='both')
        scale_row = Scale(row1, label='行', from_=5, to=self.MAX_ROW,
                              orient=HORIZONTAL, length=200, showvalue=1,
                              tickinterval=3, resolution=1, variable=self.row)
        scale_row.set(self.row.get())
        scale_row.pack(side='top',expand='YES',fill='both')
        
        row3 = Frame(self)
        row3.pack(fill="x")
        Button(row3, text="取消", command=self.cancel).pack(side=RIGHT)
        Button(row3, text="确定", command=self.ok).pack(side=RIGHT)
    def ok(self):
        self.userinfo = [self.col.get(), self.row.get()]
        self.destroy()
    def cancel(self):
        self.destroy()


if __name__ == '__main__':
    show = showPathGUI()
    show.root.mainloop()

