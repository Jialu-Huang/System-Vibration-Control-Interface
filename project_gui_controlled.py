#!/usr/bin/env python3
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QSlider, QLabel, \
    QGroupBox
from PyQt5.QtCore import Qt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
import project_system as ps
import time
import math
import numpy as np
from matplotlib import animation
'''
#the system data or code import there 
#from msd import MassSpringDamper
'''
class Double_inverse_pendulum_system_MassDamperGUI(QMainWindow):
    def __init__(self):
        QMainWindow.__init__(self)
        self.setWindowTitle('Double_inverse_pendulum_system_MassDamper Playgound')
        # Control buttons for the interface, three buttons
        quit_button = QPushButton('Quit')
        quit_button.clicked.connect(app.exit)

        # The display for the left control panel
        # The parameters of the system we're simulating
        parameters = QGroupBox('System parameters')
        parameter_layout = QVBoxLayout()
        #the value of four mass
        self.masslabel = QLabel('Mass')
        self.M = SliderDisplay('Mass of car', 1, 100,990)
        self.mone = SliderDisplay('Mass of barone', 0.1, 20,1990)
        self.mtwo = SliderDisplay('Mass of bartwo', 0.01, 2,1990)
        self.mthree = SliderDisplay('Mass of joint', 0.1, 20,1990)
        parameter_layout.addWidget(self.masslabel)
        parameter_layout.addWidget(self.M)
        parameter_layout.addWidget(self.mone)
        parameter_layout.addWidget(self.mtwo)
        parameter_layout.addWidget(self.mthree)

        # the value of three dampers
        self.damperlabel = QLabel('Damper')
        self.bone = SliderDisplay('Damper of car', 0.1, 10,990)
        self.btwo = SliderDisplay('Damper bwtween car and bar', 0.01, 1,990)
        self.bthree = SliderDisplay('Damper bwtween bar and bar', 0.01, 1,990)
        parameter_layout.addWidget(self.damperlabel)
        parameter_layout.addWidget(self.bone)
        parameter_layout.addWidget(self.btwo)
        parameter_layout.addWidget(self.bthree)

        # the value of Moment_of_inertia
        self.lenlabel = QLabel('Length')
        self.l = SliderDisplay('Length', 0.01, 1,990)
        parameter_layout.addWidget(self.lenlabel)
        parameter_layout.addWidget(self.l)
        parameters.setLayout(parameter_layout)

        #controller part
        controller = QGroupBox('Controller part')
        controller_layout = QVBoxLayout()
        self.qlabel = QLabel('q')
        self.qone = SliderDisplay('q1', 1000, 10000,9)
        self.qtwo = SliderDisplay('q2', 1000, 10000,9)
        self.qthree = SliderDisplay('q3', 1000, 10000,9)
        self.qfour = SliderDisplay('q4', 0, 10000,10)
        self.qfive = SliderDisplay('q5', 0, 10000,10)
        self.qsix = SliderDisplay('q6', 0, 10000,10)
        controller_layout.addWidget(self.qlabel)
        controller_layout.addWidget(self.qone)
        controller_layout.addWidget(self.qtwo)
        controller_layout.addWidget(self.qthree)
        controller_layout.addWidget(self.qfour)
        controller_layout.addWidget(self.qfive)
        controller_layout.addWidget(self.qsix)
        #r value
        self.rlabel = QLabel('r')
        self.r = SliderDisplay('r', 0.01, 10,9990)
        controller_layout.addWidget(self.rlabel)
        controller_layout.addWidget(self.r)

        controller.setLayout(controller_layout)


        # The parameters of the simulation themselves
        systemtime = QGroupBox('Time parameters')
        systemtime_layout = QVBoxLayout()
        self.t = SliderDisplay('Time (s)', 0.1, 10,990)
        self.dt = SliderDisplay('Time step (s)', 0.001, 0.1,990)
        systemtime_layout.addWidget(self.t)
        systemtime_layout.addWidget(self.dt)
        systemtime.setLayout(systemtime_layout)

        # this is for plot
        Figureplot = QGroupBox('Figures and animation')
        Figure_layout = QHBoxLayout()
        self.figure = Figure()
        self.display = FigureCanvas(self.figure)
        # self.draw()
        Figure_layout.addWidget(self.display)
        Figureplot.setLayout(Figure_layout)

        # The time control bar
        timecontrol = QGroupBox('Time slider')
        timecontrol_layout = QHBoxLayout()
        self.timecontrol = SliderDisplay('Time (%)', 0, 100, 100)
        show_button = QPushButton('show the instance position')
        show_button.clicked.connect(self.stopshow)
        timecontrol_layout.addWidget(self.timecontrol)
        timecontrol_layout.addWidget(show_button)
        timecontrol.setLayout(timecontrol_layout)

        #The initial setting
        inisetting = QGroupBox('initial and reference setting')
        inisetting_layout = QVBoxLayout()
        self.xcar = SliderDisplay('car initial position (m)', 0, 50,100)
        self.angle1 = SliderDisplay('pendulum initial angle1 (degree)', -20, 20, 100)
        self.angle2 = SliderDisplay('pendulum initial angle2 (degree)', -20, 20, 100)
        self.rcar = SliderDisplay('car reference position (m)', 0, 50, 100)
        simulate_button = QPushButton('Simulation\nSystem')
        simulate_button.clicked.connect(self.simulate)
        stateplot_button = QPushButton('state\nplot')
        stateplot_button.clicked.connect(self.stateplot)
        inisetting_layout.addWidget(self.rcar)
        inisetting_layout.addWidget(self.xcar)
        inisetting_layout.addWidget(self.angle1)
        inisetting_layout.addWidget(self.angle2)
        inisetting_layout.addWidget(simulate_button)
        inisetting_layout.addWidget(stateplot_button)
        inisetting.setLayout(inisetting_layout)


        # The layout of the interface
        widget = QWidget()
        self.setCentralWidget(widget)

        top_level_layout = QHBoxLayout()
        widget.setLayout(top_level_layout)

        left_side_layout = QVBoxLayout()
        right_side_layout = QVBoxLayout()

        left_side_layout.addWidget(parameters)
        left_side_layout.addWidget(controller)
        left_side_layout.addWidget(systemtime)
        left_side_layout.addWidget(quit_button)

        right_side_layout.addWidget(Figureplot)
        right_side_layout.addWidget(timecontrol)
        right_side_layout.addWidget(inisetting)

        top_level_layout.addLayout(left_side_layout)
        top_level_layout.addLayout(right_side_layout)

    def call_system(self):
        m1 = self.M.value()
        m2 = self.mone.value()
        m3 = self.mtwo.value()
        m4 = self.mthree.value()
        m = [m1, m2, m3, m4]
        b1 = self.bone.value()
        b2 = self.btwo.value()
        b3 = self.bthree.value()
        b = [b1, b2, b3]
        l = self.l.value()
        A, B, C, D = ps.sys_state(m, b, l)
        x0 = [self.xcar.value(), self.angle1.value() * math.pi / 180, self.angle2.value() * math.pi / 180, 0, 0, 0]
        t_in = [0, self.t.value()]

        q1 = self.qone.value()
        q2 = self.qtwo.value()
        q3 = self.qthree.value()
        q4 = self.qfour.value()
        q5 = self.qfive.value()
        q6 = self.qsix.value()
        r1 = self.r.value()

        in_q = [q1, q2, q3, q4, q5, q6]
        CA, CB, CC, CD = ps.lqr_sys(A, B, C, D, in_q, r1)
        ref = self.rcar.value()
        e0 = [0.5, 0.1, 0.1, -0.5, -0.1, -0.1]
        cx0 = x0 + e0
        num_t = int(self.t.value() / self.dt.value())
        state_x, state_t = ps.sys_simulation(CA, CB, t_in, num_t, cx0, ref, control=True)

        return state_x, state_t


    def stopshow(self):

        state_x, state_t = self.call_system()
        pos_x1 = [a for a in state_x[:,0]]
        pos_x2 = np.zeros(len(state_t))
        pos_x3 = np.zeros(len(state_t))
        pos_y1 = np.zeros(len(state_t))
        pos_y2 = np.zeros(len(state_t))
        pos_y3 = np.zeros(len(state_t))

        for i in range(len(state_t)):
            pos_x2[i] = pos_x1[i]+math.sin(state_x[i,1])*self.l.value()
            pos_x3[i] = pos_x1[i]+math.sin(state_x[i,1])*self.l.value()+math.sin(state_x[i,2])*self.l.value()

        for i in range(len(state_t)):
            pos_y2[i] = pos_y1[i] + math.cos(state_x[i, 1])*self.l.value()
            pos_y3[i] = pos_y1[i] + math.cos(state_x[i, 1])*self.l.value() + math.cos(state_x[i, 2])*self.l.value()

        p1 = np.row_stack((pos_x1,pos_y1))
        p2 = np.row_stack((pos_x2,pos_y2))
        p3 = np.row_stack((pos_x3,pos_y3))
        ind_now = int(self.timecontrol.value()*len(state_t)/100)
        if ind_now !=0:
            ind_now = ind_now-1

        self.stopmovie(p1,p2,p3,ind_now)

    def stopmovie(self,p1,p2,p3,ind_t):
        self.figure.clear()
        print(ind_t)
        if len(p1) > 0 and len(p2) > 0 and len(p3) > 0:
            ax = self.figure.add_subplot(111)
            ax.set_xlim(-3, 3)
            ax.set_ylim(-3, 3)
            ax.set_aspect('equal', 'box')
            x = np.array([p1[0,ind_t],p2[0,ind_t],p3[0,ind_t]])
            y = np.array([p1[1,ind_t],p2[1,ind_t],p3[1,ind_t]])
            ax.plot(x,y,'-*r')
        self.display.draw()

    def simulate(self):

        state_x, state_t = self.call_system()
        pos_x1 = [a for a in state_x[:,0]]
        pos_x2 = np.zeros(len(state_t))
        pos_x3 = np.zeros(len(state_t))
        pos_y1 = np.zeros(len(state_t))
        pos_y2 = np.zeros(len(state_t))
        pos_y3 = np.zeros(len(state_t))

        for i in range(len(state_t)):
            pos_x2[i] = pos_x1[i]+math.sin(state_x[i,1])*self.l.value()*10
            pos_x3[i] = pos_x1[i]+math.sin(state_x[i,1])*self.l.value()*10+math.sin(state_x[i,2])*self.l.value()*10

        for i in range(len(state_t)):
            pos_y2[i] = pos_y1[i] + math.cos(state_x[i, 1])*self.l.value()*10
            pos_y3[i] = pos_y1[i] + math.cos(state_x[i, 1])*self.l.value()*10 + math.cos(state_x[i, 2])*self.l.value()*10

        p1 = np.row_stack((pos_x1,pos_y1))
        p2 = np.row_stack((pos_x2,pos_y2))
        p3 = np.row_stack((pos_x3,pos_y3))
        self.movie(p1,p2,p3,len(state_t))

    def movie(self,p1,p2,p3,len_t):
        self.figure.clear()
        if len(p1) > 0 and len(p2) > 0 and len(p3) > 0:
            ax = self.figure.add_subplot(111)
            line, = ax.plot([], [],'-*r', lw=2)

            def init():
                ax.set_xlim(-20, 20)
                ax.set_ylim(-20, 20)
                ax.set_aspect('equal', 'box')
                line.set_data([], [])
                return line,

            def animate(i):
                x = np.array([p1[0,i],p2[0,i],p3[0,i]])
                y = np.array([p1[1,i],p2[1,i],p3[1,i]])

                line.set_data(x, y)
                return line,

        anim = animation.FuncAnimation(self.figure, animate, init_func=init,
                                       frames=len_t, interval=3,repeat=False,  blit=True)

        self.display.draw()

    def stateplot(self):
        s_time = time.time()
        state_x, state_t = self.call_system()
        e_time = time.time()
        print(e_time-s_time)
        self.draw(state_t,state_x)

    def draw(self, t=[], state=[]):
        self.figure.clear()

        if len(t) > 0 and len(state) > 0:
            ax = self.figure.add_subplot(131)
            ax.plot(t, state[:, 0])
            ax.plot(t, state[:, 3])
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Amplitude')
            ax.grid(True)
            ax.legend(('car position', 'car velocity'),loc=0, shadow=True)
            ax = self.figure.add_subplot(132)
            ax.plot(t, state[:, 1])
            ax.plot(t, state[:, 4])
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Amplitude')
            ax.grid(True)
            ax.legend(('car position', 'car velocity'), loc=0, shadow=True)
            ax = self.figure.add_subplot(133)
            ax.plot(t, state[:, 2])
            ax.plot(t, state[:, 5])
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Amplitude')
            ax.grid(True)
            ax.legend(('car position', 'car velocity'), loc=0, shadow=True)
            self.figure.subplots_adjust(top=0.95, bottom=0.1, left=0.12, right=0.98, hspace=0.25,
                                wspace=0.8)
        self.display.draw()

class SliderDisplay(QWidget):
    def __init__(self, name, low, high, ticks=1000):
        QWidget.__init__(self)
        self.name = name
        self.low = low
        self.range = high - low
        self.ticks = ticks
        # Horizontal layout
        layout = QHBoxLayout()
        self.setLayout(layout)
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimum(0)
        self.slider.setMaximum(ticks)
        self.slider.valueChanged.connect(self.set_value)
        self.display = QLabel()
        self.set_value()
        layout.addWidget(self.display)
        layout.addWidget(self.slider)

    def value(self):
        return (self.slider.value() / self.ticks) * self.range + self.low
    def set_value(self):
        self.display.setText('{0}: {1:.3f}'.format(self.name, self.value()))

'''
class GraphingDisplay(FigureCanvas):
    def __init__(self):
        self.figure = Figure()
        FigureCanvas.__init__(self, self.figure)

    def clear(self):
        self.figure.clear()

    def figure(self):
        return self.add_subplot(111)
'''

if __name__ == '__main__':
    app = QApplication([])

    gui = Double_inverse_pendulum_system_MassDamperGUI()

    gui.show()

    app.exec_()