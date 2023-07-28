import numpy as np
import matplotlib.pyplot as pt

class IDM_accel():
    def __init__(self,params):
        self.params = params
        self.a = params[0]
        self.b = params[1]
        self.v0 = params[2]
        self.delta = params[3]
        self.T = params[4]
        self.s0 = params[5]
        self.vehicle_length = 5.0
        
    def accel_func(self,s,v,dv):

        if abs(s) < 1e-3:
            s = 1e-3
        
        s_star = self.s0 + np.max([0,v *self.T - v*dv/(2*np.sqrt(self.a * self.b))])
            
        return self.a*(1- (v/self.v0)**self.delta - (s_star/s)**2)
    
    def steady_state(self,v=None):
        return (self.s0 + v*self.T)/np.sqrt((1 - (v/self.v0)**self.delta))
    
    def get_partials(self,s,v):

        eps = 1e-6
        da_ds = np.zeros(s.shape)
        da_dv = np.zeros(s.shape)
        da_dds = np.zeros(s.shape)
        
        num_samples = len(s)
        
        for i in range(num_samples):
            
            da_ds[i] = (self.accel_func(s[i]+eps,v[i],0.0)-self.accel_func(s[i]-eps,v[i],0.0))/(2*eps)
            da_dds[i] = (self.accel_func(s[i],v[i],eps)-self.accel_func(s[i],v[i],-eps))/(2*eps)
            da_dv[i] = (self.accel_func(s[i],v[i]+eps,0.0)-self.accel_func(s[i],v[i]-eps,0.0))/(2*eps)

        return [da_ds,da_dv,da_dds]
            
    def get_stability_vals(self,s,v):
        #Taken from an analysis by Benjamin Seibold:
        [da_ds,da_dv,da_dds] = self.get_partials(s,v)
        
        alpha1 = da_ds
        alpha2 = da_dds-da_dv
        alpha3 = da_dds
        
        stab = np.multiply(alpha2,alpha2)-np.multiply(alpha3,alpha3)-2*alpha1
        
        return stab


    def plot_FD(self,want_stability_region=True):
        
        v_eq = np.linspace(0.1,self.v0-.1,100)
        s_eq = self.steady_state(v_eq)
        
        
        rho = np.divide(1,s_eq+self.vehicle_length)
        q = np.multiply(rho,v_eq)
        
        q = q*3600.0
        
        pt.figure(figsize=[20,10])
        marker_size = 20.0
        
        col = []
        
        if(want_stability_region):
            stability_vals = self.get_stability_vals(s=s_eq,v=v_eq)
            col = np.where(stability_vals<0,'r','b')
            pt.scatter(rho,q,s=marker_size,c=col)
        else:
            pt.scatter(rho,q,s=marker_size,c='b')

        pt.title('Fundamental Diagram')
        pt.xlabel('Density [vehicles/meter]')
        pt.ylabel('Flow [vehicles/hour]')

        return q,rho            
        
    def plot_speed_vs_flow(self,want_stability_region=True):
        v_eq = np.linspace(0.1,self.v0-.1,100)
        s_eq = self.steady_state(v_eq)

        rho = np.divide(1,s_eq+self.vehicle_length)
        q = np.multiply(rho,v_eq)
        
        q = q*3600.0
        
        pt.figure(figsize=[20,10])
        marker_size = 20.0
        
        col = []
        
        if(want_stability_region):
            stability_vals = self.get_stability_vals(s=s_eq,v=v_eq)
            col = np.where(stability_vals<0,'r','b')
            pt.scatter(q,v_eq,s=marker_size,c=col)
        else:
            pt.scatter(q,v_eq,s=marker_size,c='b')

        pt.title('Flow vs. Speed')
        pt.xlabel('Speed [meter/second]')
        pt.ylabel('Flow [vehicles/hour]')

#%%
class OVMFTL_accel():
    def __init__(self,params):
        self.params = params
        self.alpha = params[0]
        self.beta = params[1]
        self.vm = params[2]
        self.s0 = params[3]
        self.s_star = params[4]
        self.vehicle_length = 5.0
        
    def accel_func(self,s,v,dv):
        V = self.steady_state(s=s)
        accel = self.alpha*(V-v) + self.beta*(dv/(s**2))
        return accel
        
    def steady_state(self,v=None,s=None):
        v_eq = self.vm*(np.tanh(s/self.s0-self.s_star)+np.tanh(self.s_star))/(1+np.tanh(self.s_star))
        
        return v_eq
        
    
    def get_partials(self,s,v):

        eps = 1e-6
        da_ds = np.zeros(s.shape)
        da_dv = np.zeros(s.shape)
        da_dds = np.zeros(s.shape)
        
        num_samples = len(s)
        
        for i in range(num_samples):
            
            da_ds[i] = (self.accel_func(s[i]+eps,v[i],0.0)-self.accel_func(s[i]-eps,v[i],0.0))/(2*eps)
            da_dds[i] = (self.accel_func(s[i],v[i],eps)-self.accel_func(s[i],v[i],-eps))/(2*eps)
            da_dv[i] = (self.accel_func(s[i],v[i]+eps,0.0)-self.accel_func(s[i],v[i]-eps,0.0))/(2*eps)

        return [da_ds,da_dv,da_dds]
            
    def get_stability_vals(self,s,v):
        #Taken from an analysis by Benjamin Seibold:
        [da_ds,da_dv,da_dds] = self.get_partials(s,v)
        
        alpha1 = da_ds
        alpha2 = da_dds-da_dv
        alpha3 = da_dds
        
        stab = np.multiply(alpha2,alpha2)-np.multiply(alpha3,alpha3)-2*alpha1
        
        return stab


    def plot_FD(self,want_stability_region=True):
        
        v_eq = np.linspace(0.1,self.v0-.1,100)
        s_eq = self.steady_state(v_eq)
        
        
        rho = np.divide(1,s_eq+self.vehicle_length)
        q = np.multiply(rho,v_eq)
        
        q = q*3600.0
        
        pt.figure(figsize=[20,10])
        marker_size = 20.0
        
        col = []
        
        if(want_stability_region):
            stability_vals = self.get_stability_vals(s=s_eq,v=v_eq)
            col = np.where(stability_vals<0,'r','b')
            pt.scatter(rho,q,s=marker_size,c=col)
        else:
            pt.scatter(rho,q,s=marker_size,c='b')

        pt.title('Fundamental Diagram')
        pt.xlabel('Density [vehicles/meter]')
        pt.ylabel('Flow [vehicles/hour]')

        return q,rho            
        
    def plot_speed_vs_flow(self,want_stability_region=True):
        s_eq = np.linspace(0.1,30,100)
        v_eq = self.steady_state(s=s_eq)


        rho = np.divide(1,s_eq+self.vehicle_length)
        q = np.multiply(rho,v_eq)
        
        q = q*3600.0
        
        pt.figure(figsize=[20,10])
        marker_size = 20.0
        
        col = []
        
        if(want_stability_region):
            stability_vals = self.get_stability_vals(s=s_eq,v=v_eq)
            col = np.where(stability_vals<0,'r','b')
            pt.scatter(q,v_eq,s=marker_size,c=col)
        else:
            pt.scatter(q,v_eq,s=marker_size,c='b')

        pt.title('Flow vs. Speed')
        pt.xlabel('Speed [meter/second]')
        pt.ylabel('Flow [vehicles/hour]')
        pts.show()

    def plot_spacing_vs_speed(self,want_stability_region=True):

        s_eq = np.linspace(0.1,30,100)
        v_eq = self.steady_state(s=s_eq)

        pt.figure(figsize=[20,10])
        marker_size = 20.0

        if(want_stability_region):
            stability_vals = self.get_stability_vals(s=s_eq,v=v_eq)
            col = np.where(stability_vals<0,'r','b')
            pt.scatter(s_eq,v_eq,s=marker_size,c=col)
            pt.grid()
        else:
            pt.scatter(s_eq,v_eq,s=marker_size,c='b')
            pt.grid()

        pt.title('Spacing vs. Speed')
        pt.xlabel('Spacing [meter]')
        pt.ylabel('Speed [meters/second]')
        pt.show()