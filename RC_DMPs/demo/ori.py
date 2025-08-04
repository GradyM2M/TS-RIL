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
        self.max_x= [5,20,10]
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
    
    def rollout_ECDMP(self,tau=1.0,xT=None):

        x_rollout = np.zeros([self.N,3])
        dx_rollout = np.zeros([self.N,3])
        ddx_rollout = np.zeros([self.N,3])
        
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
        max_x= [5,20,10]
        d0 = [3,2,2]
        k = [1,200,20]
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
    plt.figure(figsize=(8, 6))
    plt.rcParams['xtick.direction'] = 'in'
    plt.rcParams['ytick.direction'] = 'in'



    data_len = 20
    x_trajectory = 5*np.sin(np.linspace(0,np.pi,data_len))
    y_trajectory=5*np.cos(np.linspace(0,np.pi,data_len))
    z_trajectory=np.ones(data_len)

    x_direction = np.zeros([data_len,3])
    y_direction = np.zeros([data_len,3])
    z_direction = np.zeros([data_len,3])


    x_direction[:,0] = 1
    y_direction[:,1] = 1
    z_direction[:,2] = 1


    demo = np.zeros([data_len,3])
    demo[:,0] = x_trajectory
    demo[:,1] = y_trajectory
    demo[:,2] = z_trajectory


    dmp = CartesianDMP(alphaz=25.0,betaz=25.0/4.0,orientation=False)
    x_des = dmp.imitate(demo)


    # 生成泛化轨迹
    x_rollout, dx_rollout, ddx_rollout = dmp.rollout()
    # x_rollout, dx_rollout, ddx_rollout = dmp.rollout(x0=[9,15.5,3],xT=[10.25,15.6,3])
    x_rollout2, dx_rollout2, ddx_rollout2 = dmp.rollout_ECDMP()
    demo = np.zeros([300,3])

    # 绘制三维图
    ax3 = plt.axes(projection='3d')
    
    # 二元函数定义域平面
    x = np.linspace(0, 20, 20)
    y = np.linspace(0, 20, 20)
    X, Y = np.meshgrid(x, y)

    ax3.plot_surface(X,
                Y,
                Z=X*0+3,
                color='white',
                alpha=0.3 )
    
    ax3.set_xlabel('X(cm)', fontsize=15)
    ax3.set_ylabel('Y(cm)', fontsize=15)
    ax3.set_zlabel('Z(cm)', fontsize=15) 

    ax3.grid(False)

    ax3.tick_params(axis='x', labelsize=12)
    ax3.tick_params(axis='y', labelsize=12)
    ax3.tick_params(axis='z', labelsize=12)
    ax3.set_zlim(2.9,3.1)

    z_demo = np.ones(200)*3
    # # 安全裕度
    ax3.plot(np.ones(200)*2, np.linspace(2,18,200), z_demo,'black', linestyle=':' ,linewidth=2, label='Safe Limits')
    ax3.plot(np.ones(200)*18, np.linspace(2,18,200), z_demo,'black', linestyle=':' ,linewidth=2)
    ax3.plot( np.linspace(2,18,200), np.ones(200)*2,z_demo,'black', linestyle=':' ,linewidth=2)
    ax3.plot( np.linspace(2,18,200),np.ones(200)*18, z_demo,'black', linestyle=':' ,linewidth=2)
    # 安全边界
    ax3.plot(np.ones(200)*0, np.linspace(0,20,200), z_demo, 'black', linewidth=2, label='Safe Margins')
    ax3.plot(np.ones(200)*20, np.linspace(0,20,200), z_demo, 'black' ,linewidth=2)
    ax3.plot( np.linspace(0,20,200), np.ones(200)*0,z_demo, 'black' ,linewidth=2)
    ax3.plot( np.linspace(0,20,200),np.ones(200)*20, z_demo, 'black' ,linewidth=2)
    # 演示轨迹与泛化轨迹 
    ax3.plot(x_des[:,0], x_des[:,1], x_des[:,2],'g',linewidth=3, label='demo')
    ax3.plot(x_rollout[:,0],x_rollout[:,1], x_rollout[:,2],'b',linewidth=3, label='DMP')
    ax3.plot(x_rollout2[:,0],x_rollout2[:,1], x_rollout2[:,2],'r',linewidth=3, label='EC-DMP')

    # ax3.set_xticks([0,2,10,18,20])
    # ax3.set_yticks([0,2,10,18,20])
    ax3.set_zticks([2.5,3,3.5])
    
    plt.legend(loc=2,prop = {'size':12},
        ncols=1, shadow=False, fancybox=False)
    ax3.view_init(elev=56, azim=-51)
    plt.subplots_adjust(left=0.0, bottom=0.05, right=0.9, top=1.0,hspace=0.1,wspace=0.1)
    plt.show()



