#!/usr/bin/env python
import rospy
import Jetson.GPIO as GPIO

from std_msgs.msg import Float32
from std_msgs.msg import Int16

from grayson_sensors.msg import wheel_encoder_data

class Wheel_Encoder():
	def __init__(self):
		RIGHT_CHANNEL = 'GPIO_PZ0'
		LEFT_CHANNEL = 'LCD_BL_PW'
	
		# Pull private parameters only for Wheel Encoder
		channel = rospy.get_param('~channel')
		period = rospy.get_param('~period')
		
		# boolean for checking if the wheel encoder is for right/left wheel
		is_right = channel == RIGHT_CHANNEL

		# initialize GPIO bus mode
		if GPIO.getmode() != GPIO.TEGRA_SOC:
			GPIO.setmode(GPIO.TEGRA_SOC) 

		# Set publisher to publish on wheel encoder channel
		topic = 'rwheel'
		if channel == LEFT_CHANNEL:
			topic = 'lwheel'

		pub1 = rospy.Publisher(topic, Int16, queue_size = 5)
		pub2 = rospy.Publisher(channel, wheel_encoder_data, queue_size = 5)

		GPIO.setup(channel, GPIO.IN)

		deltaT = 0.0
		current_time = rospy.get_time()
		previous_time = current_time

		he_input = GPIO.input(channel)
		tick_count = 0
		total_tick_count = 0
		rospy.loginfo(rospy.get_caller_id() + ' began wheel encoder ' + channel)
		
		while not rospy.is_shutdown():
			current_time = rospy.get_time()
			
			new_input = GPIO.input(channel)
			
			if he_input != new_input:
				he_input = new_input
				tick_count += 1
				total_tick_count += 1
			
			deltaT += current_time - previous_time

			if deltaT >= period:
				msg1 = Int16()
				msg1.data = total_tick_count
				pub1.publish(msg1)
				
				msg2 = wheel_encoder_data()
				msg2.is_right = is_right
				msg2.total_tick_count = total_tick_count
				msg2.tick_count = tick_count
				msg2.delta_t = deltaT
				msg2.stamp.secs = current_time
				msg2.is_ready = True
				
				pub2.publish(msg2)
				
				deltaT = 0
				tick_count = 0
			previous_time = current_time
		GPIO.cleanup()
				
				

# initialization
if __name__ == '__main__':
	rospy.init_node('wheel_encoder')

	try:
		rospy.loginfo('Attempting to start wheel encoder')
		wc = Wheel_Encoder()
	except rospy.ROSInterruptException: pass
