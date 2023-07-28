import numpy as np
import matplotlib.pyplot as pt
from copy import deepcopy
from car_following_utils import OVMFTL_accel
from ring_sim_with_VSL import simulate_ring_with_VSL

class VSL_no_change():

	def __init__(self,HV_accel_func_list):

		self.HV_accel_func_list_original = deepcopy(HV_accel_func_list)

		self.HV_accel_func_list_current = deepcopy(HV_accel_func_list)

	def HV_update(self,HV_accel_func_list,X,V):
		return HV_accel_func_list



class VSL_constant_vel():
	def __init__(self,HV_accel_func_list,v_des):

		self.HV_accel_func_list_original = deepcopy(HV_accel_func_list)

		self.HV_accel_func_list_current = deepcopy(HV_accel_func_list)

		self.updated_CFM_with_VSL = deepcopy(HV_accel_func_list[0])

		# NOTE: this is very specific to the CFM used in this particular case
		self.updated_CFM_with_VSL.vm = v_des

		self.HV_accel_func_list_current[0] = self.updated_CFM_with_VSL

	def HV_update(self,HV_accel_func_list,X,V):
		return self.HV_accel_func_list_current




if __name__ == '__main__':
	HV_sim_params = [0.8,20.0,9.8,2.0,2.0]
	HV_accel_func = OVMFTL_accel(HV_sim_params)

	num_vehicles = 22

	# create list of CFM function objects:
	HV_accel_func_list = []
	for i in range(num_vehicles):
		HV_accel_func_list.append(HV_accel_func)

	ring_length = 100.0

	total_sim_time = 300.0

	# VSL_update_function = VSL_no_change(HV_accel_func_list)

	v_des = 7.0

	VSL_update_function = VSL_constant_vel(HV_accel_func_list,v_des = v_des)

	ring_sim = simulate_ring_with_VSL(HV_accel_func_list,
		VSL_update_function,
		ring_length=ring_length,
		total_sim_time=total_sim_time,
		added_noise_time=50,
		added_noise_std=0.05,
		VSL_switch_on_time=100,
		dt=0.1)

	ring_sim.run_sim()


	V = ring_sim.V.T


	pt.figure()

	
	pt.plot(V[:,1:],'b')
	pt.plot(V[:,0],'r')


	pt.show()


