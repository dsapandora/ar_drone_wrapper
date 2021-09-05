# ar_drone_wrapper
ROS Package for PARROT AR DRONE 1.0

I will explain soon how it works,
this ros package is intend to be used for ROS MELODIC. and is based in https://github.com/venthur/python-ardrone proje

To install go to your ROS src folder an clone de REPOSITORY. 

Proceed to create a virtualenv env and source from it

Then install the python requeriments, After that you can compile the project

This project will need to be in the virtual env to work properly

The library  ffmpeg need to be installed to be able to receive the Drone Camera Image
```
sudo apt install  ffmpeg
```
#TODO I still don't know why is not publishing the navdata, but i will continue
working on it the next weekend.

## Navdata structure
```
{
	0: {
		'vy': 0.0,
		'phi': -3,
		'psi': -115,
		'num_frames': 0,
		'battery': 79,
		'altitude': 0,
		'ctrl_state': 1,
		'vx': 0.0,
		'theta': -1,
		'vz': 0.0
	},
	'drone_state': {
		'acq_thread_on': 1,
		'angles_out_of_range': 0,
		'ctrl_watchdog_mask': 0,
		'video_mask': 0,
		'com_watchdog_mask': 0,
		'fw_ver_mask': 0,
		'video_thread_on': 1,
		'adc_watchdog_mask': 0,
		'com_lost_mask': 0,
		'control_mask': 0,
		'user_el': 0,
		'atcodec_thread_on': 1,
		'command_mask': 1,
		'user_feedback_start': 0,
		'altitude_mask': 1,
		'fw_file_mask': 1,
		'navdata_thread_on': 1,
		'vbat_low': 0,
		'fly_mask': 0,
		'vision_mask': 0,
		'ultrasound_mask': 0,
		'emergency_mask': 0,
		'fw_upd_mask': 0,
		'pic_version_mask': 1,
		'cutout_mask': 0,
		'navdata_bootstrap': 0,
		'motors_mask': 0,
		'navdata_demo_mask': 1,
		'timer_elapsed': 0
	},
	'vision_flag': 0,
	'seq_nr': 23636,
	16: ['\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00'],
	'header': 1432778632,
	65535: ['A', '\x1e', '\x00', '\x00']
}
```
