# This Python file uses the following encoding: utf-8
#from PySide2 import QtWidgets
import matplotlib.pyplot as plt
import matplotlib
import pandas as pd
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

class VisLoops:
    def __init__(self, df):
        self.df = df

        self.fig=plt.figure()
        # plt.axis('equal')
        print(len(self.df.index))
        self.df = self.df.reset_index()  # make sure indexes pair with number of rows

    def Visualize(self):
        for i in range(1, self.df.shape[0]-1):
            row = self.df.iloc[i]
            row_next = self.df.iloc[i+1]
            x = row["from.x"]
            y = row["from.y"]
            dx = row_next["from.x"] - x
            dy = row_next["from.y"] - y

            plt.arrow(x, y, dx, dy, width=0.1, alpha=0.5)

            if row["y_test_pred"]==1:
                if row["y_test"]:
                    plt.plot([row["from.x"], row["to.x"]], [row["from.y"], row["to.y"]], color='g', linestyle='-', linewidth=3, alpha=0.7)
                else:
                    plt.plot([row["from.x"], row["to.x"]], [row["from.y"], row["to.y"]], color='r', linestyle='-', linewidth=4)
                    #plt.text(row["from.x"], row["from.y"],"r={:.2f}".format(row["candidate rot_error"])+", "+"t= {:.2f}".format(row["candidate transl_error"]) ,fontsize=12)
            elif row["y_test_pred"]==0:
                #if row["y_test"]:
                if row["candidate close"]:
                    plt.plot([row["from.x"], row["close.x"]], [row["from.y"], row["close.y"]], linestyle='-', linewidth=4, color="blue")
                    #plt.text(row["from.x"], row["from.y"],"r={:.2f}".format(row["candidate rot_error"])+", "+"t= {:.2f}".format(row["candidate transl_error"]) ,fontsize=12)
                elif not row["candidate close"] and row["y_test"]:
                    plt.plot([row["from.x"], row["close.x"]], [row["from.y"], row["close.y"]], linestyle='-', linewidth=2, color="orange")


                 #   plt.plot([row["from.x"], row["to.x"]], [row["from.y"], row["to.y"]], linestyle='-', linewidth=4, color="orange")


    def Show(self):
        plt.show()

    def Save(self, dir, name="loops.pdf"):
        self.fig.savefig(dir + "/" + name, bbox_inches='tight', format='pdf')

class VisLoops3d(VisLoops):
    def Visualize(self):
        ax = self.fig.gca(projection='3d')
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        ax.set_zlabel('id_from')
        alpha_value = 1
        linewidth_value = 0.5
        
        for i in range(self.df.shape[0]-1):
            row = self.df.iloc[i]

            id_from = i
            id_to = row["id_to"]
            id_close = row["id_close"]

            z_order_value = row["from.x"] - row["from.y"]
            path_color = str(i / ((self.df.shape[0]-1)*2))

            if row["y_test_pred"]==1:
                if row["y_test"]:
                    ax.plot([row["from.x"], row["to.x"]], [row["from.y"], row["to.y"]], [id_from, id_to], color='green', linestyle='-', linewidth=linewidth_value, alpha=alpha_value, zorder=z_order_value)
                else:
                    ax.plot([row["from.x"], row["to.x"]], [row["from.y"], row["to.y"]], [id_from, id_to], color="red", linestyle='-', linewidth=linewidth_value, alpha=alpha_value, zorder=z_order_value)
            elif row["y_test_pred"]==0:
                if row["candidate close"]:
                    ax.plot([row["from.x"], row["to.x"]], [row["from.y"], row["to.y"]], [id_from, id_to], color="blue", linestyle='-', linewidth=linewidth_value, alpha=alpha_value, zorder=z_order_value)
                elif not row["candidate close"] and row["y_test"]:
                    # ax.plot([row["from.x"], row["to.x"]], [row["from.y"], row["to.y"]], [height_range[id_from], height_range[id_to]], color="orange", linestyle='-', linewidth=linewidth_value, alpha=alpha_value, zorder=z_order_value)
                    ax.plot([row["from.x"], row["close.x"]], [row["from.y"], row["close.y"]], [id_from, id_close], color="orange", linestyle='-', linewidth=linewidth_value, alpha=alpha_value, zorder=z_order_value)

            row_next = self.df.iloc[i+1]
            x = row["from.x"]
            y = row["from.y"]
            x2 = row_next["from.x"]
            y2 = row_next["from.y"]
            ax.plot([x, x2], [y, y2], [i, i+1], linewidth=2, alpha=1, color=path_color, zorder=z_order_value)
        # ax.plot(self.df["from.x"].values, self.df["from.y"].values, height_range, linewidth=5, alpha=1, color="black")