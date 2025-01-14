import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from Tensegrity_model_inputs import inPairs_3, number_of_rods, L

def plot_MPC_prediction(trajectory,COMs,PAs):
    unit_vector_length = 0.1
    plt.plot(trajectory[:,0],trajectory[:,1],'go')
    for COM,PA in zip(COMs,PAs):
        plt.plot(COM[0],COM[1],'ro')
        tip = COM + unit_vector_length * PA
        plt.plot([COM[0],tip[0]],[COM[1],tip[1]],'m-')
    plt.xlim([-0.1,2])
    plt.ylim([-0.1,2])
    plt.show()

class Visualiser:
    def __init__(self):
        # figure and axes
        self.fig = plt.figure(figsize=(4,4))
        self.ax = self.fig.add_subplot(111, projection='3d')
        # reconstruction and mocap scatters
        self.rscat = self.ax.scatter([],[],[],color='black',alpha=1)
        self.mscat = self.ax.scatter([],[],[],color='magenta',alpha=1)
        # data
        self.rdata = [[0,0,0]]*6

        # node labels
        colors = ['red','red','green','green','blue','blue']
        self.texts = [self.ax.text(0,0,0,str(i),color=colors[i]) for i in range(2*number_of_rods)]

        # bars and sensors
        self.lines = []
        for i,pair in enumerate(inPairs_3):
        	if i < number_of_rods:
        		color = 'red' # rod
        	elif i < 9:
        		color = 'green' # short sensor
        	else:
        		color = 'blue' # long sensor

        	# plot
        	self.lines.append(self.ax.plot([],[],[],color=color)[0])

        # raw imu direction
        self.imu_vecs = [self.ax.plot([],[],[],color='magenta')[0] for imuID in range(2)]


    def plot_init(self):
        self.ax.set_xlim(-2*L, 2*L)
        self.ax.set_ylim(-2*L, 2*L)
        self.ax.set_zlim(-2*L, 2*L)
        self.ax.set_xlabel("x (mm)")
        self.ax.set_ylabel("y (mm)")
        self.ax.set_zlabel("z (mm)")
        self.ax.view_init(90,-90)
        return self.rscat

    def update(self, nodes):
    	# reconstruction
        self.rdata = [[node[0],node[1],node[2]] for node in nodes.tolist()]

    def update_plot(self, frame):

        # state reconstruction
        data = np.array(self.rdata)
        self.rscat._offsets3d = (data[:,0],data[:,1],data[:,2])

        # node labels
        for i,tex in enumerate(self.texts):
        	tex.set_x(data[i,0])
        	tex.set_y(data[i,1])
        	tex.set_3d_properties(z=data[i,2],zdir=None)

        # bars and sensors
        for i in range(len(self.lines)):
        	pair = inPairs_3[i]
        	lin = self.lines[i]
        	# x and y
        	lin.set_data(np.array([[data[pair[0],0],data[pair[1],0]],[data[pair[0],1],data[pair[1],1]]]))
        	# z
        	lin.set_3d_properties(np.array([[data[pair[0],2],data[pair[1],2]]]))

        # # raw imu direction
        # data = np.array(self.idata)
        # for i in range(len(self.imu_vecs)):
        #     vec = self.imu_vecs[i]
        #     # x and y
        #     vec.set_data(np.array([[0,data[i,0]],[0,data[i,1]]]))
        #     # z
        #     vec.set_3d_properties(np.array([[0,data[i,2]]]))

if __name__ == '__main__':

    ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init)
    plt.show(block=True) 