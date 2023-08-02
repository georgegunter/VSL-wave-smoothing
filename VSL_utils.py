import numpy as np
import matplotlib.pyplot as pt
from copy import deepcopy
from car_following_utils import OVMFTL_accel
from ring_sim_with_VSL import simulate_ring_with_VSL

class VSL_no_change():

	def __init__(self,HV_accel_func_list):

		self.HV_accel_func_list_original = deepcopy(HV_accel_func_list)

		self.HV_accel_func_list_current = deepcopy(HV_accel_func_list)

	def HV_update(self,HV_accel_func_list,X,V,sim_env):
		return HV_accel_func_list



class VSL_constant_vel():
	def __init__(self,HV_accel_func_list,v_des):

		self.HV_accel_func_list_original = deepcopy(HV_accel_func_list)

		self.HV_accel_func_list_current = deepcopy(HV_accel_func_list)

		self.updated_CFM_with_VSL = deepcopy(HV_accel_func_list[0])

		# NOTE: this is very specific to the CFM used in this particular case
		self.updated_CFM_with_VSL.vm = v_des

		self.HV_accel_func_list_current[0] = self.updated_CFM_with_VSL

	def HV_update(self,HV_accel_func_list,X,V,sim_env):
		return self.HV_accel_func_list_current





class VSL_moving_average_RDS():

	def __init__(self,HV_accel_func_list,RDS_speed_ave_window=30.0):

		self.HV_accel_func_list_original = deepcopy(HV_accel_func_list)

		self.HV_accel_func_list_current = deepcopy(HV_accel_func_list)

		self.updated_CFM_with_VSL = deepcopy(HV_accel_func_list[0])

		self.RDS_speed_ave_window = RDS_speed_ave_window

		self.speed_commands_history = []

	def update_cfm_with_v_des(self,v_des):
		# NOTE: this is very specific to the CFM used in this particular case
		self.updated_CFM_with_VSL.vm = v_des

		self.HV_accel_func_list_current[0] = self.updated_CFM_with_VSL

	def get_ave_speed(self,sim_env):
		T = sim_env.RDS_time_counts
		V  = sim_env.RDS_speed_measurements
		curr_sim_time = sim_env.curr_sim_time

		speed_mesurements = [V[-1]]

		i = -2
		while(T[i] > (curr_sim_time-self.RDS_speed_ave_window)):
			speed_mesurements.append(V[i])
			i -= 1

		return np.mean(speed_mesurements)


	def HV_update(self,HV_accel_func_list,X,V,sim_env):
		v_des = self.get_ave_speed(sim_env)
		self.update_cfm_with_v_des(v_des)
		self.speed_commands_history.append(v_des)
		return self.HV_accel_func_list_current

