#!/usr/bin/python
import movement

move = movement.KhanMove()
dist = movement.IRdistance()

selection = input('1 forward, 2 reverse, 3 right turn, 4 left turn, 5 stop, 6 print distance, 7 speed, 8 quit ')
speed = input('Input speed 0-100 ')

while selection!=8:
	
	if selection ==1:
		move.forward(speed)
	elif selection ==2:
		move.reverse(speed)
	elif selection ==3:
		move.right(speed)
	elif selection ==4:
		move.left(speed)
	elif selection ==5:
		move.stop()
	elif selection ==6:
		print (dist.readdist())
	elif selection ==7:
		speed = input('Input speed 0-100 ')

	selection = input('1 forward, 2 reverse, 3 right turn, 4 left turn, 5 stop, 6 print distance, 7 speed, 8 quit')