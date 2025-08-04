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

    def rollout(self,tau=1.0,x0=None,xT=None):

        x_rollout = np.zeros([self.N,3])
        dx_rollout = np.zeros([self.N,3])
        ddx_rollout = np.zeros([self.N,3])

        self.x0  = x0
        
        if xT is None:
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



    demo = np.zeros([300,3])
    t2 = np.linspace(0,1,100)


    # demo[:,0] = np.cos(8*t2)
    # demo[:,1] = np.sin(12*t2)
    # demo[:,2] = -np.sin(10*t2)

    tra1x = np.zeros(100)
    tra1y = np.linspace(0,5,100)

    tra2x = np.sin(np.linspace(0,np.pi,100,endpoint=True))
    tra2y = 4 + np.cos(np.linspace(0,np.pi,100,endpoint=True))  

    tra3x = np.linspace(0,1.25,100)
    tra3y = np.linspace(3,0.1,100)

    tra_x = np.hstack((tra1x,tra2x,tra3x))
    tra_y = np.hstack((tra1y,tra2y,tra3y))
    tra_z = np.ones(300)*3

    demo[:,0] = tra_x+9
    demo[:,1] = tra_y+7.5
    demo[:,2] = tra_z 

    rot = np.ones((2,2))
    print(rot)
    rot[0] = [np.cos(-np.pi/4),-np.sin(-np.pi/4)]
    rot[1] = [np.sin(-np.pi/4),np.cos(-np.pi/4)]

    x0 = demo[:,:2]
    print(x0.shape)

    x1 = np.ones((300,2))
    for n in range(300):
        print(rot,x0[n])
        x1[n] = np.matmul(rot,x0[n])

    demo[:,0] = x1[:,0]-2
    demo[:,1] = x1[:,1]+10


    dmp = CartesianDMP(alphaz=25.0,betaz=25.0/4.0,orientation=False)
    x_des = dmp.imitate(demo)

    # 生成泛化轨迹
    x_rollout, _, _ = dmp.rollout(x0=[16,17,3],xT=[17.7,16,3])
    x_rollout2, _, _ = dmp.rollout_ECDMP(x0=[16,17,3],xT=[17.7,16,3])
    
    plt.figure(num=1,figsize=(4,3))
    plt.tick_params(\
    axis='x',#设置x轴
    direction='in',# 小坐标方向，in、out
    which='both',      # 主标尺和小标尺一起显示，major、minor、both
    bottom=True,      #底部标尺打开
    top=False,         #上部标尺关闭
    labelbottom=True, #x轴标签打开
    labelsize=12) #x轴标签大小
    plt.tick_params(\
    axis='y',
    direction='in',
    which='both',     
    left=True,    
    right=False,  
    labelbottom=True,
    labelsize=12) 
    plt.minorticks_on()#开启小坐标
    plt.ticklabel_format(axis='both',style='sci')#sci文章的风格
    plt.subplots_adjust(left=0.1, bottom=0.1, right=0.95, top=0.9,hspace=0.1,wspace=0.1)
    plt.plot(x_des[:,1], 'g', linewidth=2, label='demo')
    plt.plot(x_rollout[:,1], 'b', linewidth=2, label='DMP')
    plt.plot(x_rollout2[:,1], 'r', linewidth=2, label='EC-DMP')
    plt.plot(np.ones(len(x_des[:,1]))*18, 'black', linestyle=':', linewidth=2)
    plt.plot(np.ones(len(x_des[:,1]))*20, 'black', linewidth=2, )
    plt.fill_between(np.linspace(0,3000,3000),np.ones(3000)*18,np.ones(3000)*20,facecolor = 'yellow', alpha = 0.3)

    plt.legend(prop = {'size':8},bbox_to_anchor=(0.6,0.7),ncols=1)
    
    ax=plt.gca();#获得坐标轴的句柄
    ax.spines['bottom'].set_linewidth(2);###设置底部坐标轴的粗细
    ax.spines['left'].set_linewidth(2);####设置左边坐标轴的粗细
    ax.spines['right'].set_linewidth(2);###设置右边坐标轴的粗细
    ax.spines['top'].set_linewidth(2);####设置上部坐标轴的粗细

    plt.rcParams['axes.unicode_minus'] = False#使用上标小标小一字号
    plt.rcParams['font.sans-serif']=['Times New Roman']
    #设置全局字体，可选择需要的字体替换掉‘Times New Roman’
    #使用黑体'SimHei'作为全局字体，可以显示中文
    #plt.rcParams['font.sans-serif']=['SimHei']
    font1={'family': 'Times New Roman', 'weight': 'light', 'size': 12}#设置字体模板，
    plt.title("Y-axis",fontdict = font1, fontsize=12)#标题
    plt.xlabel('time(timesteps)',fontdict=font1,fontsize=12)
    plt.ylabel('Position(cm)',fontdict=font1,fontsize=12)#$\mathregular{min^{-1}}$label的格式,^{-1}为上标
    # plt.xlabel('',fontdict=font1)
    plt.subplots_adjust(left=0.15, bottom=0.15, right=0.95, top=0.9,hspace=0.1,wspace=0.1)
    plt.savefig("case4-y.png",dpi=600)
    plt.show()

