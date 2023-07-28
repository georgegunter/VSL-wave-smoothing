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