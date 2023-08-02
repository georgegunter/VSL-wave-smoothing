import numpy as np
import matplotlib.pyplot as pt
from copy import deepcopy
from car_following_utils import OVMFTL_accel
from ring_sim_with_VSL import simulate_ring_with_VSL

from VSL_utils import VSL_moving_average_RDS



if __name__ == '__main__':
	HV_sim_params = [0.8,20.0,9.8,2.0,2.0]
	HV_accel_func = OVMFTL_accel(HV_sim_params)

	num_vehicles = 22

	# create list of CFM function objects:
	HV_accel_func_list = []
	for i in range(num_vehicles):
		HV_accel_func_list.append(HV_accel_func)

	ring_length = 100.0

	VSL_switch_on_time = 300.0

	total_sim_time = VSL_switch_on_time + 600.0

	# VSL_update_function = VSL_no_change(HV_accel_func_list)

	# v_des = 7.0

	# VSL_update_function = VSL_constant_vel(HV_accel_func_list,v_des = v_des)


	RDS_speed_ave_window = 100.0

	VSL_update_function = VSL_moving_average_RDS(HV_accel_func_list,
		RDS_speed_ave_window =RDS_speed_ave_window)


	ring_sim = simulate_ring_with_VSL(HV_accel_func_list,
		VSL_update_function,
		ring_length=ring_length,
		total_sim_time=total_sim_time,
		added_noise_time=50,
		added_noise_std=0.05,
		VSL_switch_on_time=VSL_switch_on_time,
		dt=0.1)

	ring_sim.run_sim()



	V = ring_sim.V.T

	sim_times = np.linspace(0,total_sim_time,len(V[:,1:]))


	RDS_time_measurements = ring_sim.RDS_time_counts
	RDS_speed_measurements = ring_sim.RDS_speed_measurements

	VSL_speed_commands = VSL_update_function.speed_commands_history

	VSL_times = np.linspace(VSL_switch_on_time,
		total_sim_time,
		len(VSL_speed_commands))


	fig = pt.figure()
	pt.subplot(3,1,1)
	pt.title('Vehicular speeds',fontsize=10)
	pt.plot(sim_times,V[:,1:],'b')
	pt.plot(sim_times,V[:,0],'r')
	pt.xlim([0,total_sim_time])
	pt.subplot(3,1,2)
	pt.plot(VSL_times,VSL_speed_commands)
	pt.title('Commanded speed',fontsize=10)
	pt.xlim([0,total_sim_time])
	pt.subplot(3,1,3)
	pt.plot(RDS_time_measurements,RDS_speed_measurements,'-o',markersize=5)
	pt.xlim([0,total_sim_time])
	pt.title('RDS speed measurements',fontsize=10)

	fig.suptitle('VSL commanding ave RDS speed, window: '+str(RDS_speed_ave_window),fontsize=20)

	pt.show()

