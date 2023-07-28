import numpy as np
from copy import deepcopy

class simulate_ring_with_VSL():

	def __init__(self,
		HV_accel_func_list,
		VSL_update_function,
		ring_length=300,
		total_sim_time=600,
		added_noise_time=50,
		added_noise_std=0.2,
		VSL_switch_on_time=300,
		dt=0.1):

		# initialize geometry and driver functions:
		self.ring_length = ring_length
		self.dt = dt

		self.HV_accel_func_list_original = HV_accel_func_list
		self.HV_accel_func_list = deepcopy(HV_accel_func_list)

		self.VSL_update_function = VSL_update_function
		self.VSL_switch_on_time = VSL_switch_on_time
		self.num_vehicles = len(HV_accel_func_list)
		self.num_steps = int(total_sim_time/dt)
		self.added_noise_time = added_noise_time
		self.added_noise_std = added_noise_std
		self.total_sim_time = total_sim_time
		self.curr_sim_step = 1
		self.curr_sim_time = self.dt


		# initialize history for storing simulation data:
		self.X = np.zeros([self.num_vehicles,self.num_steps])
		self.V = np.zeros([self.num_vehicles,self.num_steps])
		self.S = np.zeros([self.num_vehicles,self.num_steps])
		self.DS_DT = np.zeros([self.num_vehicles,self.num_steps])
		self.DV_Dt = np.zeros([self.num_vehicles,self.num_steps])

		# all vehicles begin at even spacing:
		s_init = self.ring_length/self.num_vehicles
		self.S[:,0] = np.ones_like(self.S[:,0])*s_init
		x_init = np.arange(0,self.ring_length,s_init)
		self.X[:,0] = x_init


	def step_sim(self,want_added_noise=False):
		#vehicle 0 follows 1, 1 follows 2, ... n follows 0:
		for i in range(0,self.num_vehicles):

			s = self.S[i,self.curr_sim_step-1]
			x = self.X[i,self.curr_sim_step-1]
			v = self.V[i,self.curr_sim_step-1]

			ds_dt = self.DS_DT[i,self.curr_sim_step-1]

			# get CFM for the correct vehicle:
			dv_dt = self.HV_accel_func_list[i].accel_func(s,v,ds_dt)

			# if(self.curr_sim_time > self.VSL_switch_on_time):
			# 	if(i == 0):
			# 		print('Adjusted CFM v_max: '+str(self.HV_accel_func_list[i].params[2]))
			# 		print('Commanded accel : '+str(dv_dt))


			v_new = v + dv_dt*self.dt

			if(want_added_noise):
				v_new += np.random.randn()*self.added_noise_std

			s_new = s + ds_dt*self.dt
			x_new = x + v*self.dt

			if(s_new < 0.0):
				print('Collision occurred with vehicle '+str(i))

			v_new = np.max([v_new,0.0])
			s_new = np.max([s_new,0.0])

			# fill in to simulation record:
			self.S[i,self.curr_sim_step] = s_new
			self.X[i,self.curr_sim_step] = x_new
			self.V[i,self.curr_sim_step] = v_new

		DS_DT_new = np.zeros_like(self.DS_DT[:,self.curr_sim_step])

		DS_DT_new[0:self.num_vehicles-1] = np.diff(self.V[:,self.curr_sim_step])

		DS_DT_new[-1] = self.V[0,self.curr_sim_step] - self.V[-1,self.curr_sim_step]

		self.DS_DT[:,self.curr_sim_step] = DS_DT_new

		# advance simulation
		self.curr_sim_step += 1
		self.curr_sim_time += self.dt


	def initialize_sim_with_waves(self):
		while(self.curr_sim_time < 3.0):
			self.step_sim()
		while(self.curr_sim_time < self.added_noise_time):
			self.step_sim(want_added_noise=True)

	def run_sim(self):
		# run for some period to initialize waves:
		self.initialize_sim_with_waves()
		# run for the remaining duration of the simulation:

		# steps_left_to = self.curr_sim_step

		# for i in range()

		while(self.curr_sim_step < self.num_steps):
			# check to see if VSL should switch on:
			if(self.curr_sim_time > self.VSL_switch_on_time):


				# Get potential states that could be used to VSL calculation:
				X_for_VSL = self.X[:,self.curr_sim_step-1]
				V_for_VSL = self.V[:,self.curr_sim_step-1]
				# update the current accel funcs to allow for VSL changes:
				self.HV_accel_func_list = self.VSL_update_function.HV_update(
					self.HV_accel_func_list_original,
					X_for_VSL,
					V_for_VSL)

				# print('Adjusted CFM v_max: '+str(self.HV_accel_func_list[0].params[2]))

			self.step_sim()

		print('Simulation finished at time: '+str(self.curr_sim_time))



	


