import numpy as np
from scipy import interpolate
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
import copy

class CartesianDMP():
    
    def __init__(self,N_bf=300,alphaz=4.0,betaz=1.0,orientation=False):
        
        self.alphax = 1.0
        self.alphaz = alphaz
        self.betaz = betaz
        self.N_bf = N_bf # number of basis functions
        self.tau = 1.0 # temporal scaling

        self.phase = 1.0 # initialize phase variable

       

    def imitate(self, pose_demo, sampling_rate=100, oversampling=True):
        
        self.T = pose_demo.shape[0] / sampling_rate

        print(self.T)        
        if not oversampling:
            self.N = pose_demo.shape[0]
            self.dt = self.T / self.N
            self.x = pose_demo[:,:3]
            
        else:
            self.N = 10*pose_demo.shape[0] # 10-fold oversample
            print(self.N) 
            self.dt = self.T / self.N
            print(f"self.T is: {self.T}; self.dt is:{self.dt}")

            t = np.linspace(0.0,self.T,pose_demo[:,0].shape[0])
            self.x_des = np.zeros([self.N,3])
            for d in range(3):
                x_interp = interpolate.interp1d(t,pose_demo[:,d])
                for n in range(self.N):
                    self.x_des[n,d] = x_interp(n * self.dt)
                
        # Centers of basis functions 
        self.c = np.ones(self.N_bf) 
        c_ = np.linspace(0,self.T,self.N_bf)
        for i in range(self.N_bf):
            self.c[i] = np.exp(-self.alphax *c_[i])

        # Widths of basis functions 
        # (as in https://github.com/studywolf/pydmps/blob/80b0a4518edf756773582cc5c40fdeee7e332169/pydmps/dmp_discrete.py#L37)
        self.h = np.ones(self.N_bf) * self.N_bf**1.5 / self.c / self.alphax

        self.dx_des = np.gradient(self.x_des,axis=0)/self.dt
        self.ddx_des = np.gradient(self.dx_des,axis=0)/self.dt

        # Initial and final orientation
        self.x0 = self.x_des[0,:]
        self.dx0 = self.dx_des[0,:] 
        self.ddx0 = self.ddx_des[0,:]
        self.xT = self.x_des[-1,:]

        # Initialize the DMP
        self.x = copy.deepcopy(self.x0)
        self.dx = copy.deepcopy(self.dx0)
        self.ddx = copy.deepcopy(self.ddx0)

        # Evaluate the phase variable
        # self.phase = np.exp(-self.alphax*np.linspace(0.0,self.T,self.N))

        # Evaluate the forcing term
        forcing_target_pos = self.tau*self.ddx_des - self.alphaz*(self.betaz*(self.xT-self.x_des) - self.dx_des)
        self.fit_dmp(forcing_target_pos)
        
        return self.x_des
    
    def RBF(self, phase):

        if type(phase) is np.ndarray:
            return np.exp(-self.h*(phase[:,np.newaxis]-self.c)**2)
        else:
            return np.exp(-self.h*(phase-self.c)**2)

    def forcing_function_approx(self,weights,phase,xT=1,x0=0):

        BF = self.RBF(phase)
        if type(phase) is np.ndarray:
            return np.dot(BF,weights)*phase/np.sum(BF,axis=1)
        else:
            return np.dot(BF,weights)*phase/np.sum(BF)
    
    def fit_dmp(self,forcing_target):

        phase = np.exp(-self.alphax*np.linspace(0.0,self.T,self.N))
        BF = self.RBF(phase)
        X = BF*phase[:,np.newaxis]/np.sum(BF,axis=1)[:,np.newaxis]

        self.weights_pos = np.zeros([self.N_bf,3])

        # for d in range(3):
        #     self.weights_pos[:,d] = np.dot(np.linalg.pinv(X),forcing_target[:,d])

        regcoef = 0.01
        for d in range(3):        
            self.weights_pos[:,d] = np.dot(np.dot(np.linalg.pinv(np.dot(X.T,(X)) + \
                                    regcoef * np.eye(X.shape[1])),X.T),forcing_target[:,d].T) 

    def reset(self):
        
        self.phase = 1.0
        self.x = copy.deepcopy(self.x0)
        self.dx = copy.deepcopy(self.dx0)
        self.ddx = copy.deepcopy(self.ddx0)

        self.min_x = [0,0,-10]
        self.max_x= [20,20,10]
        self.d0 = [2,2,2]
        self.k = [1,2,20]



    def step(self, disturbance=None):
        
        disturbance_pos = np.zeros(3)
        disturbance_ori = np.zeros(3)

        if disturbance is None:
            disturbance = np.zeros(6)
        else:
            disturbance_pos = disturbance[:3]
            disturbance_ori = disturbance[3:]
        
        self.phase += (-self.alphax * self.tau * self.phase) * (self.T/self.N)
        forcing_term_pos = self.forcing_function_approx(self.weights_pos,self.phase)

        self.ddx = self.alphaz * (self.betaz * (self.xT - self.x) - self.dx) + forcing_term_pos + disturbance_pos
        self.dx += self.ddx * self.dt * self.tau
        self.x += self.dx * self.dt * self.tau

      
        return copy.deepcopy(self.x), copy.deepcopy(self.dx), copy.deepcopy(self.ddx)

    def rollout(self,tau=1.0,xT=None):

        x_rollout = np.zeros([self.N,3])
        dx_rollout = np.zeros([self.N,3])
        ddx_rollout = np.zeros([self.N,3])


        
        xT = self.xT

        x_rollout[0,:] = self.x0
        dx_rollout[0,:] = self.dx0
        ddx_rollout[0,:] = self.ddx0

        if xT is None:
            xT = self.xT
        
        phase = np.exp(-self.alphax*tau*np.linspace(0.0,self.T,self.N))

        # Position forcing term
        forcing_term_pos = np.zeros([self.N,3])
        print(forcing_term_pos.shape)
        for d in range(3):
            forcing_term_pos[:,d] = self.forcing_function_approx(
                self.weights_pos[:,d],phase,xT[d],self.x0[d])
            

        for d in range(3):
            for n in range(1,self.N):
            
                ddx_rollout[n,d] = self.alphaz*(self.betaz*(xT[d]-x_rollout[n-1,d]) - \
                                               dx_rollout[n-1,d]) + forcing_term_pos[n,d] 
                dx_rollout[n,d] = dx_rollout[n-1,d] + tau*ddx_rollout[n-1,d]*self.dt
                x_rollout[n,d] = x_rollout[n-1,d] + tau*dx_rollout[n-1,d]*self.dt
        
        # Get orientation rollout

        return x_rollout,dx_rollout,ddx_rollout
    
    def rollout_ECDMP(self,tau=1.0,x0=None,xT=None):

        x_rollout = np.zeros([self.N,3])
        dx_rollout = np.zeros([self.N,3])
        ddx_rollout = np.zeros([self.N,3])

        self.x0  = x0
        
        if xT is None:
            xT = self.xT

        x_rollout[0,:] = self.x0
        dx_rollout[0,:] = self.dx0
        ddx_rollout[0,:] = self.ddx0


        
        phase = np.exp(-self.alphax*tau*np.linspace(0.0,self.T,self.N))

        # Position forcing term
        forcing_term_pos = np.zeros([self.N,3])
        for d in range(3):
            forcing_term_pos[:,d] = self.forcing_function_approx(
                self.weights_pos[:,d],phase,xT[d],self.x0[d])
        print(forcing_term_pos[:,d].shape)
            
        min_x = [0,0,-10]
        max_x= [20,20,10]
        d0 = [2,2,2]
        k = [1,2,20]
        for d in range(3):
            for n in range(1,self.N):

                if x_rollout[n-1,d]-min_x[d] < d0[d]:
                    u = k[d] * (1/(x_rollout[n-1,d]-min_x[d])-1/d0[d])*(1/(x_rollout[n-1,d]-min_x[d])**2)
                elif max_x[d]-x_rollout[n-1,d] < d0[d]:
                    u = k[d] * (1/d0[d]-1/(max_x[d]-x_rollout[n-1,d]))*(1/(max_x[d]-x_rollout[n-1,d])**2)
                else:
                    u = 0

                ddx_rollout[n,d] = self.alphaz*(self.betaz*(xT[d]-x_rollout[n-1,d]) - \
                                               dx_rollout[n-1,d]) + forcing_term_pos[n,d]+u
                dx_rollout[n,d] = dx_rollout[n-1,d] + tau*ddx_rollout[n-1,d]*self.dt
                x_rollout[n,d] = x_rollout[n-1,d] + tau*dx_rollout[n-1,d]*self.dt
        
        # Get orientation rollout

        return x_rollout,dx_rollout,ddx_rollout

# Test

if __name__ == "__main__":

    import matplotlib.pyplot as plt
    import matplotlib as mpl

    plt.rcParams['axes.unicode_minus'] = False#使用上标小标小一字号
    plt.rcParams['font.sans-serif']=['Times New Roman']

    mpl.rcParams['font.sans-serif'] = ['STZhongsong']    # 指定默认字体：解决plot不能显示中文问题
    mpl.rcParams['axes.unicode_minus'] = False  

    plt.rcParams['axes.unicode_minus'] = False#使用上标小标小一字号
    plt.rcParams['font.sans-serif']=['Times New Roman']

    font1={'family': 'Times New Roman', 'weight': 'light', 'size': 12}
    plt.rcParams['xtick.direction'] = 'in'
    plt.rcParams['ytick.direction'] = 'in'




    X = np.load(r'D:\大论文\实验代码\experiment3\X.npy')
    Y = np.load(r'D:\大论文\实验代码\experiment3\Y.npy')

    data_len = 200
    demo = np.zeros([data_len,3])
    t = np.linspace(0,1,data_len,endpoint=True)
    demo[:,0] = t
    demo[:,1] = 20*np.cos(6*t+0.9*np.exp(-2*t)+1.6*t**2)
    demo[:,2] = np.ones(data_len)




    dmp = CartesianDMP(alphaz=10,betaz=10/4.0,N_bf=5,orientation=False)
    x_des = dmp.imitate(demo)
    x_rollout, dx_rollout, ddx_rollout = dmp.rollout()

    dmp2 = CartesianDMP(alphaz=20,betaz=20/4.0,N_bf=5,orientation=False)
    x_des = dmp2.imitate(demo)
    x_rollout2, dx_rollout2, ddx_rollout2 = dmp2.rollout()
    

    plt.figure(figsize=(3.5, 2.5))
    # x_major = MultipleLocator(1)
    # plt.gca().xaxis.set_major_locator(x_major)
    colors = ['red','blue','green']
    # plt.ticklabel_format(axis='both',style='sci',scilimits=(-1,2))#sci文章的风格
    plt.plot(x_des[:,1],color='black',label=f"demon", alpha=0.8,lw=2)
    plt.plot(x_rollout[:,1],color=colors[1],label=f"DMP1",lw=2,linestyle='--')
    plt.plot(x_rollout2[:,1],color=colors[2],label=f"DMP2",lw=2,linestyle='--')
    plt.title(f"DMP leaning", fontsize=10)
    plt.xlabel("Time(stamps)", fontsize=10)
    plt.ylabel("Position", fontsize=10)
    plt.legend()
    # plt.ylim(bottom=0.)
    plt.subplots_adjust(left=0.15, bottom=0.2, right=0.95, top=0.9,hspace=0.05,wspace=0.05)
    # plt.savefig(f'C:/Users/dsr/Desktop/dataset/dmp-tpe.png',dpi=600)
    
    plt.figure(figsize=(3.5, 2.5))
    # x_major = MultipleLocator(1)
    # plt.gca().xaxis.set_major_locator(x_major)
    colors = ['red','blue','green']
    # plt.ticklabel_format(axis='both',style='sci',scilimits=(-1,2))#sci文章的风格
    # plt.plot(x_des[:,0], x_des[:,1],color='black',label=f"demon", alpha=0.8,lw=2)
    error = x_rollout[:,1]-x_des[:,1]
    error2 = x_rollout2[:,1]-x_des[:,1]

    plt.plot(np.abs(error),color=colors[1],label=f"DMP1",lw=2,linestyle='--')
    plt.plot(np.abs(error2),color=colors[2],label=f"DMP2",lw=2,linestyle='--')
    # plt.plot(x_rollout2[:,0],x_rollout2[:,1],color=colors[2],label=f"DMP2",lw=2,linestyle='--')
    plt.title(f"Error between demon", fontsize=10)
    plt.xlabel("Time(stamps)", fontsize=10)
    plt.ylabel("Position", fontsize=10)
    plt.legend()
    # plt.ylim(bottom=0.)
    plt.subplots_adjust(left=0.15, bottom=0.2, right=0.95, top=0.9,hspace=0.05,wspace=0.05)
    # plt.savefig(f'C:/Users/dsr/Desktop/dataset/dmp-error.png',dpi=600)

    colors = ['red','blue','green']


    from scipy import interpolate
    time = 1
    timestamps = np.linspace(0,1,len(error),endpoint=True)
    f_interpolate = interpolate.interp1d(timestamps,x_rollout[:,0])
    time_new5 = np.linspace(0, time, 300)
    dp_x_rollout = f_interpolate(time_new5)

    time = 1
    timestamps = np.linspace(0,1,len(x_rollout[:,0]),endpoint=True)
    f_interpolate = interpolate.interp1d(timestamps,error)
    time_new5 = np.linspace(0, time, 300)
    dp_error = f_interpolate(time_new5)

    # time = 1
    # timestamps = np.linspace(0,1,len(error2),endpoint=True)
    # f_interpolate = interpolate.interp1d(timestamps,x_rollout[:,0])
    # time_new5 = np.linspace(0, time, 200)
    # x_rollout[:,0] = f_interpolate(time_new5)

    time = 1
    timestamps = np.linspace(0,1,len(error2),endpoint=True)
    f_interpolate = interpolate.interp1d(timestamps,error2)
    time_new5 = np.linspace(0, time, 300)
    dp_error2 = f_interpolate(time_new5)



    fig = plt.figure(figsize=(3.5, 2.5))

    # 创建空的轨迹线，一个红色表示预测值，一个蓝色表示真值
    line_DMP1, = plt.plot([], [], marker='.',linestyle='--', markersize=4, color='g', label='DMP1', )
    line_DMP2, = plt.plot([], [], marker='*',linestyle='--', markersize=4, color='b', label='DMP2')
    
    # plt.plot(x_rollout[:,0],x_des[:,1],color='black',label=f"demon", alpha=0.8,lw=2)

    # 初始化函数，用于绘制空轨迹线
def init():
    line_DMP1.set_data([], [])
    line_DMP2.set_data([], [])

    return line_DMP1,line_DMP2

def update(frame):
    line_DMP1.set_data(dp_x_rollout[:frame], np.abs(dp_error)[:frame])
    line_DMP2.set_data(dp_x_rollout[:frame], np.abs(dp_error2)[:frame])

    # plt.set_xlim(min(x_rollout[:,0]) - 0.1, max(x_rollout[:,0]) + 0.1)
    # plt.set_ylim(min(np.abs(error)) - 0.5, max(np.abs(error)) + 0.5)

    plt.xlim((-0.01, 1.01))
    plt.ylim((-0.5, 6.5))
 
    return line_DMP1,line_DMP2
 


from matplotlib.animation import FuncAnimation, PillowWriter
# 创建动画对象
ani = FuncAnimation(fig, update, frames=len(dp_error), init_func=init, blit=True)
# plt.axis('off') 

plt.title(f"Error between demon", fontsize=10)
plt.xlabel("Time", fontsize=10)
plt.ylabel("Value", fontsize=10)
plt.legend()


# plt.ylim(bottom=0.)
plt.subplots_adjust(left=0.15, bottom=0.2, right=0.95, top=0.9,hspace=0.05,wspace=0.05)

# 创建一个文件名为animation.gif的视频文件，使用PillowWriter
ani.save('animation_gt2.gif', writer=PillowWriter(fps=120),dpi=300)


# 显示动画
plt.show()





