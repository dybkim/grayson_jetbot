#!/usr/bin/env python
import rospy
import board
import busio
import adafruit_bno055

from std_msgs.msg import Float32

from grayson_sensors.msg import imu_data

class BNO055():
	def __init__(self):
		# Pull private parameters only for BNO055
		period = rospy.get_param('~period')
		rate = rospy.Rate(1/period) #default 100 hz
		
		i2c = busio.I2C(board.SCL_1, board.SDA_1)
		sensor = adafruit_bno055.BNO055(i2c)

		pub = rospy.Publisher('BNO055', wheel_encoder_data, queue_size = 10)

		deltaT = 0.0
		current_time = rospy.get_time()
		previous_time = current_time

		tick_count = 0
		total_tick_count = 0
		rospy.loginfo(rospy.get_caller_id() + ' began IMU Sensor:')
		
		while not rospy.is_shutdown():
			current_time = rospy.get_time()
			
			acceleration = sensor.linear_acceleration
			x_accel = acceleration[0]
			y_accel = acceleration[1]
			yaw = sensor.euler[2]
			
			deltaT += current_time - previous_time

			msg = imu_data()
			msg.linear_acceleration.x = x_accel
			msg.linear_acceleration.y = y_accel
			msg.angular_velocity.z = yaw
			msg.delta_t = deltaT
			msg.stamp = current_time
			msg.is_ready = True

			rospy.loginfo(rospy.get_caller_id() + ' x_accel: ' + str(x_accel) + ' y_accel: ' + str(y_accel) + ' yaw: ' + yaw)
			
			pub.publish(msg)
			
			deltaT = 0

			previous_time = current_time
			rate.sleep()
		
				
# initialization
if __name__ == '__main__':
	rospy.init_node('bno055_imu')

	try:
		rospy.loginfo('Attempting to start wheel encoder')
		imu = BNO055()
	except rospy.ROSInterruptException: pass
