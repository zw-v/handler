#!/usr/bin/python3
import threading, time, curses, serial, json
import RPi.GPIO as GPIO

settings_fname = 'settings.json'
settings = []
flag_quit = 0

arduino = serial.Serial('/dev/ttyUSB0', 115200, bytesize=8, parity='E', stopbits=2, timeout=None)  # open serial port
recv_buff = [ 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 ]

recv_pos = 0
arduino_ready = 0

STP_STEPS_REV = 3200 #1600 * belt
Z_STP_STEPS_REV = 800
Z_CMS_PER_REV = 8.08
ENC_STEPS_REV = 4096

A_pos_drv_raw = 0
A_pos_enc = 0
A_pos_enc_raw = 0

B_pos_drv_raw = 0
B_pos_enc = 0
B_pos_enc_raw = 0

Z_pos_drv_raw = 0
Z_mag = 0


main_menu = [ 'Load', 'Unload', 'Go to', 'Setup', 'Quit' ]
menu_load = [ '1', '2', '3', '4', '5', '6', '7', '8', '9', '10', '11' ]
menu_goto = [ 'z:top', 'z:bottom', 'z:grab', 'a:hole', 'b:180deg', '1', '2', '3', '4', '5', '6', '7', '8', '9', '10', '11' ]
menu_unload = [ 'previous', '1', '2', '3', '4', '5', '6', '7', '8', '9', '10', '11' ]

m0 = 1			# wybrana pozycja glownego menu
m1 = 0			# wybrana pozycja podmenu	( 0 = zadna )
level = 0		# poziom menu na ktorym jestesmy

log = ['', '', '', '', '']

current = -1


def daemon_serial(name):
	global recv_buff, recv_pos, A_pos_enc_raw, B_pos_enc_raw, A_pos_enc, B_pos_enc, A_pos_drv_raw, B_pos_drv_raw, Z_pos_drv_raw, Z_mag, arduino_ready
	 
	while(1):
		if flag_quit:
			arduino.close()
			return
		
		recv_buff[recv_pos] = arduino.read()[0]
		if recv_pos == 0:
			if ( recv_buff[0] == 0xFA ):		# advance only on 0xFA
				recv_pos += 1
		elif recv_pos > 0:
			if recv_pos == 13:					# last byte
				if recv_buff[13] == 0xFF: 		# ...and a stop byte
					A_pos_drv_raw = 0
					A_pos_drv_raw = recv_buff[1]
					A_pos_drv_raw <<= 8
					A_pos_drv_raw |= recv_buff[2]

					A_pos_enc_raw = recv_buff[3]
					A_pos_enc_raw <<= 8
					A_pos_enc_raw |= recv_buff[4]
					
					B_pos_drv_raw = 0
					B_pos_drv_raw = recv_buff[5]
					B_pos_drv_raw <<= 8
					B_pos_drv_raw |= recv_buff[6]
					
					B_pos_enc_raw = recv_buff[7]
					B_pos_enc_raw <<= 8
					B_pos_enc_raw |= recv_buff[8]				
					
					Z_pos_drv_raw = recv_buff[9]
					Z_pos_drv_raw <<= 8
					Z_pos_drv_raw |= recv_buff[10]
					
					Z_mag = recv_buff[11]
					
					
					A_pos_enc = A_pos_enc_raw/16384*360
					B_pos_enc = B_pos_enc_raw/16384*360
					
					recv_pos = 0
					arduino_ready = 1

				else:							# ...last byte is wrong!
					#print_message("last byte is wrong")
					recv_pos = 0
			else:								# not yet a last byte
				recv_pos += 1
def angle2steps( angle ): return int(angle*STP_STEPS_REV/360)
def steps2angle( steps ): return float(steps*360/STP_STEPS_REV)
def steps2cms( steps ): return float(steps*Z_CMS_PER_REV/Z_STP_STEPS_REV)
def cms2steps( cms): return int(cms*Z_STP_STEPS_REV/Z_CMS_PER_REV)
def send_cmd( jog, a_target, b_target, z_target, z_mag):
	packet = bytearray()
	# START
	packet.append(0xFA)
	# FIRST
	if jog == "a+":
		packet.append(0xEA)
	elif jog == "a-":
		packet.append(0xEB)
	elif jog == "b+":
		packet.append(0xEC)
	elif jog == "b-":
		packet.append(0xED)
	elif jog == "z+":
		packet.append(0xEE)
	elif jog == "z-":
		packet.append(0xEF)
	else:
		packet.append(0xF0)
			
	# 2ND, 3RD
	if a_target != -1:
		a_target_H = a_target
		a_target_L = a_target
		a_target_H >>= 8
		a_target_L &= 0xFF
		packet.append(a_target_H)
		packet.append(a_target_L)
	else:
		packet.append(0xF0)
		packet.append(0xF0)
	# 4TH, 5TH
	if b_target != -1:
		b_target_H = b_target
		b_target_L = b_target
		b_target_H >>= 8
		b_target_L &= 0xFF
		packet.append(b_target_H)
		packet.append(b_target_L)
	else:
		packet.append(0xF0)
		packet.append(0xF0)
	# 6TH
	if z_target != -1:
		z_target_H = z_target
		z_target_L = z_target
		z_target_H >>= 8
		z_target_L &= 0xFF
		packet.append(z_target_H)
		packet.append(z_target_L)
	else:
		packet.append(0xF0)
		packet.append(0xF0)
	# 8TH
	if z_mag != -1:
		if z_mag == True:
			packet.append(0x01)
		else:
			packet.append(0x00)
	else:
		packet.append(0xF0)
	# 9TH
	packet.append(0xFF)

	arduino.write(packet)
def gotoangle( target_angle ): send_cmd(angle2steps(target_angle), -1, -1, -1)
def wait4arduino():
	global arduino_ready
	time.sleep(0.3)
	arduino_ready = 0
	
	while arduino_ready != 1:
		time.sleep(0.1)
	return
def load( slot ):
	global current, arduino_ready
	
	if ( current != -1 ):
		unload()
	
	if ( slot > 0 and slot < 12 ):
		print_message("Loading material from slot # " + str(slot))
		
		send_cmd(-1,settings[slot]['pos_a'],settings[slot]['pos_b'],settings[0]['z_grab_height'],1)
		wait4arduino()
		send_cmd(-1,-1,-1,0,-1)
		wait4arduino()
		send_cmd(-1,settings[0]['a_hole_position'],-1,-1,-1)
		wait4arduino()
		send_cmd(-1,-1,-1,cms2steps(5),-1)
		wait4arduino()
		
		print_message("Loaded")
		
		current = slot
	else:
		print_message("critical error")
	return
def unload():
	global current
	if ( current != -1):
		print_message("Unloading material to original slot # " + str(current))
		
		send_cmd(-1,-1,-1,0,-1)
		wait4arduino()
		send_cmd(-1,settings[current]['pos_a'],settings[current]['pos_b'],settings[0]['z_grab_height'],-1)
		wait4arduino()
		send_cmd(-1,-1,-1,-1,0)
		time.sleep(0.1)
		send_cmd(-1,-1,-1,settings[0]['z_grab_height']-5,-1)
		wait4arduino()
		send_cmd(-1,settings[current]['pos_a']-30,settings[current]['pos_b'],settings[0]['z_grab_height'],-1)
		wait4arduino()
		send_cmd(-1,-1,-1,0,-1)
		wait4arduino()
		print_message("Unloaded")
		current = -1
	else:
		print_message("Nothing to unload!")
	return
def print_message(message):
	global main, w_log, log

	w_log.box()
	
	
	x = 1
	y = 1
	if message is not "":
		w_log.clear()
		w_log.box()
		temp = time.strftime("%H:%M:%S - ") + message
		#temp = message
		if len(temp) < 77:
		
			log[4] = log[3]
			log[3] = log[2]
			log[2] = log[1]
			log[1] = log[0]
			log[0] = temp
	
	
	w_log.addstr(y,x,log[4])
	w_log.addstr(y+1,x,log[3])
	w_log.addstr(y+2,x,log[2])
	w_log.addstr(y+3,x,log[1])
	w_log.addstr(y+4,x,log[0])
	w_log.refresh()
def draw_mag():
	global w_mag
	
	w_mag.box()
	
	for i in range(1,12):
		w_mag.addstr(i, 2, str(i)+': '+settings[i]['name'])
		
		if settings[i]['pos_b'] is not -1 and settings[i]['pos_a'] is not -1:
			w_mag.addstr(i, 15, "x")
			
	w_mag.addstr(13,2, 'z_grab: ' + str(settings[0]['z_grab_height']))
	w_mag.addstr(14,2, 'a_hole: ' + str(settings[0]['a_hole_position']))
	w_mag.refresh()
def draw_status():
	global main, w_stat
	
	w_stat.box()
	x_h = 1
	x = 2
	main.addstr(0,10," Material handler by Dayport. 2020 ")
		
	w_stat.addstr(1,x_h,"-- Arm:      Axis A")
	w_stat.addstr(2,11,"raw")
	w_stat.addstr(2,18,"deg")
	w_stat.addstr(3,x, "drv:                 ")
	w_stat.addstr(4,x, "enc:                 ")
	w_stat.addstr(3,11,str(A_pos_drv_raw)[0:5])
	w_stat.addstr(3,18,str(steps2angle(A_pos_drv_raw))[0:5])
	w_stat.addstr(4,11,str(A_pos_enc_raw)[0:5])
	w_stat.addstr(4,18,str(A_pos_enc)[0:5])
	
	w_stat.addstr(7,x_h,"-- Bank:     Axis B")
	w_stat.addstr(8,11,"raw")
	w_stat.addstr(8,18,"deg")
	w_stat.addstr(9,x, "drv:                 ")
	w_stat.addstr(10,x, "enc:                 ")
	w_stat.addstr(9,11,str(B_pos_drv_raw)[0:5])
	w_stat.addstr(9,18,str(steps2angle(B_pos_drv_raw))[0:5])
	w_stat.addstr(10,11,str(B_pos_enc_raw)[0:5])
	w_stat.addstr(10,18,str(B_pos_enc)[0:5])
	
	w_stat.addstr(13,x_h,"-- Mag:      Axis Z")
	w_stat.addstr(14,11,"raw")
	w_stat.addstr(14,18,"cm")
	w_stat.addstr(15,x, "drv:                 ")
	w_stat.addstr(15,11,str(Z_pos_drv_raw)[0:5])
	w_stat.addstr(15,18,str(steps2cms(Z_pos_drv_raw))[0:5])
	w_stat.addstr(16,x, "mag:                 ")
	if Z_mag == 1:
		w_stat.addstr(16,11,"ON")
	else:
		w_stat.addstr(16,11,"OFF")
		
	w_stat.addstr(18,2,"Loaded: ")
	if current == -1:
		w_stat.addstr(18,10, "nothing")
	else:
		w_stat.addstr(18,10, settings[current]['name'])

	w_stat.refresh()
def draw_main_menu(level, m0, m1 ):
	global main
	w = main.subwin(len(main_menu)+2,10,2,2)
	w.box()
	# if level == 0 or m1 == 0: #clear submenu
		# x = 15
		# y = 2
		# for row in range(len(menu_unload)):
			# menu.addstr(y + row, x, "                  ")
	
	w.refresh()
	
	for idx, entry in enumerate(main_menu):
		x = 1
		y = 1 + idx
		if idx+1 == m0:
			w.attron(curses.color_pair(1))
			w.addstr(y,x,entry)
			w.attroff(curses.color_pair(1))
		else:
			w.addstr(y,x,entry)

		if level == 1:
		
		
			if m0 == 1: #load
				sub = main.subwin(len(menu_load)+2,10,2,12)
			elif m0 == 2: #unload
				sub = main.subwin(len(menu_unload)+2,10,2,12)
			elif m0 == 3: #goto
				sub = main.subwin(len(menu_goto)+2,10,2,12)
				
			sub.box()
			x = 1
			y = 1
			sub.refresh()
			#time.sleep(30)
			if m0 == 1: #load
				for idx, row in enumerate(menu_load):
					y = 1 + idx
					
					if idx+1 == m1:
						sub.attron(curses.color_pair(1))
						sub.addstr(y,x,row)
						sub.attroff(curses.color_pair(1))
					else:
						sub.addstr(y,x,row)
			elif m0 == 2: #unload
				for idx, row in enumerate(menu_unload):
					y = 1 + idx
					
					if idx+1 == m1:
						sub.attron(curses.color_pair(1))
						sub.addstr(y,x,row)
						sub.attroff(curses.color_pair(1))
					else:
						sub.addstr(y,x,row)

			elif m0 == 3: #go to
				for idx, row in enumerate(menu_goto):
					y = 1 + idx
					
					if idx+1 == m1:
						sub.attron(curses.color_pair(1))
						sub.addstr(y,x,row)
						sub.attroff(curses.color_pair(1))
					else:
						sub.addstr(y,x,row)
def read_config():
    global settings
    with open(settings_fname, 'r') as f:
        try:
            settings = json.load(f)
        except:
            print("Failed to load settings.json")
def save_as():
	global settings
	q = main.subwin(5,29,10,22)
	q.box()
	q.refresh()
	
	
	selected = 1
	
	while True:
		
		for i in range(1,12):
			if i == selected:
				if i < 11:
					q.attron(curses.color_pair(1))
					q.addstr(1,i*2, str(i))
				else:
					q.addstr(1,i*2, " ")
					q.attron(curses.color_pair(1))
					q.addstr(1,i*2+1, str(i))
				q.attroff(curses.color_pair(1))
			else:
				if i < 11:
					q.addstr(1,i*2, str(i))
				else:
					q.addstr(1,i*2, " "+str(i))
					
		if selected == 12:
			q.attron(curses.color_pair(1))
		q.addstr(2,2, "z grab height")
		q.attroff(curses.color_pair(1))
				
		if selected == 13:
			q.attron(curses.color_pair(1))
		q.addstr(3,2, "a axis hole pos.")
		q.attroff(curses.color_pair(1))
				
				
				
				
		x = main.getch()
		
		if x == curses.KEY_LEFT:							
			if selected > 1:
				selected -= 1
		elif x == curses.KEY_RIGHT:		
			if selected < 13:
				selected += 1
		elif x == curses.KEY_BACKSPACE:		
			q.erase()
			
			return 0
		elif x == 10:
			q.erase()
			if selected < 12:
				settings[selected]['pos_a'] = A_pos_drv_raw
				settings[selected]['pos_b'] = B_pos_drv_raw
				print_message("Saved as slot "+str(selected)+" |  A_pos: "+str(A_pos_drv_raw)+" B_pos: "+str(B_pos_drv_raw))
			elif selected == 12:
				settings[0]['z_grab_height'] = Z_pos_drv_raw
				print_message("Saved as grab height: "+str(Z_pos_drv_raw))
			elif selected == 13:
				settings[0]['a_hole_position'] = A_pos_drv_raw
				print_message("Saved as hole position (A axis): "+str(A_pos_drv_raw))
			
			return
		
		q.refresh()
def setup_axi(axi):
	global main, level, w_stat, Z_mag
	
	sub = main.subwin(8,29,2,22)
	
	sub.box()
	sub.addstr(1,1, "Move arm with LEFT / RIGHT")
	sub.addstr(2,1, "Move bank with UP / DOWN")
	sub.addstr(3,1, "Move Z with PGUP / PGDN")
	sub.addstr(4,1, "Switch mag with END")
	sub.addstr(6,1, "ENTER to save pos. as:")
	
	sub.refresh()
	
	while True:
		x = main.getch()
		draw_status()
		draw_mag()
		
		if x == curses.KEY_LEFT:							
			#print_message("pressed left")
			send_cmd("a-",-1, -1, -1, -1)
		elif x == curses.KEY_RIGHT:		
			#print_message("pressed right")
			send_cmd("a+",-1, -1, -1, -1)
		elif x == curses.KEY_UP:							
			#print_message("pressed up")
			send_cmd("b+",-1, -1, -1, -1)
		elif x == curses.KEY_DOWN:		
			#print_message("pressed down")
			send_cmd("b-",-1, -1, -1, -1)
		elif x == curses.KEY_NPAGE:							
			#print_message("pressed pgdown")
			send_cmd("z+",-1, -1, -1, -1)
		elif x == curses.KEY_PPAGE:		
			#print_message("pressed pgup")
			send_cmd("z-",-1, -1, -1, -1)
		elif x == curses.KEY_END:		
			#print_message("pressed end")
			Z_mag = not Z_mag
			send_cmd(-1,-1, -1, -1, Z_mag)
		elif x == curses.KEY_BACKSPACE:			
			level -= 1
			sub.erase()
			return
		elif x == 10:
			save_as()
def main_window(stdscr):
	global level, m0, m1, main, w_stat, w_log, w_mag
	
	num_rows, num_cols = stdscr.getmaxyx()
	if ( num_rows < 30 or num_cols < 100 ):
		raise Exception("Terminal size is: " + str(num_rows) + "x" + str(num_cols) + ", must be 30x100 chars minimum")

	main = stdscr.subwin(30,100,0,0)

	main.clear()
	curses.start_color()
	curses.noecho()
	curses.cbreak()
	main.nodelay(True)
	main.keypad(True)
	curses.curs_set(0)
	curses.init_pair(1, curses.COLOR_BLACK, curses.COLOR_WHITE)

	w_stat = main.subwin(20,26,2,54)
	w_log = main.subwin(7,96,22,2)
	w_mag = main.subwin(20,18,2,80)


	print_message("Init")
	thread_serial = threading.Thread(target=daemon_serial, args=(1,), daemon=True)
	thread_serial.start()
	print_message("serial started")
	read_config()
	print_message("settings loaded from settings.json")
	
	
	while(1):
		
		main.border(0)
		draw_status()
		draw_main_menu(level,m0,m1)
		draw_mag()
		print_message("")
		#main.refresh()	
		#w_stat.refresh()
		
		
		
		
		x = main.getch()

# UP & DOWN IN THE MENUS, LEFT & RIGHT IN SETUP SUBMENU

		if x == curses.KEY_UP:							# UP
			if level == 0 and m0 > 1:
				m0 -= 1
			elif level == 1 and m1 > 1:
				m1 -= 1
		elif x == curses.KEY_DOWN:						# DOWN
			if level == 0 and m0 < len(main_menu):
				m0 += 1
			elif level == 1 and m0 == 1 and m1 < len(menu_load):
				m1 += 1
			elif level == 1 and m0 == 3 and m1 < len(menu_goto):
				m1 += 1
			

		
		
		
		
		
		elif x == 10:									# ENTER
			if level == 0 and m0 == 1: 					# load menu
				level += 1
				m1 = 1
				
			
			elif level == 0 and m0 == 2: 				# unload menu
				unload()
				
				
			elif level == 0 and m0 == 3:				# go to
				level += 1
				m1 = 1	
				
			elif level == 0 and m0 == 4: 				# setup
				#level += 1
				#m1 = 1
				setup_axi(1)
				level = 0
				m1 = 0
				
			elif level == 0 and m0 == 5: 				# quit
				unload()
				print_message("Moving to safe position...")
				send_cmd(-1,-1,-1,0,-1)
				wait4arduino()
				send_cmd(-1,angle2steps(270),-1,-1,-1)
				wait4arduino()
				flag_quit = 1
				
				
				curses.nocbreak()
				curses.echo()
				curses.endwin()
				
				
				with open(settings_fname, 'w') as f:
					json.dump(settings, f, sort_keys=True, indent=4)
				
				break
			
			elif level == 1 and m0 == 1:				# load sample
				load(m1)
				level = 0
				m1 = 0
				
			elif level == 1 and m0 == 2:				# unload sample
				unload(m1-1)
				level = 0
				m1 = 0
			
			elif level == 1 and m0 == 3:				# go to
				if m1 == 1:								# z top
					send_cmd(-1, -1, -1, 0, -1)
				elif m1 == 2:							# z bottom
					send_cmd(-1, -1, -1, cms2steps(51), -1)
				elif m1 == 3:							# z grab
					send_cmd(-1, -1, -1, settings[0]['z_grab_height'], -1)
				elif m1 == 4:							# a hole
					#send_cmd(-1, -1, -1, 0, -1)
					#wait4arduino()
					send_cmd(-1, settings[0]['a_hole_position'], -1, -1, -1)
				elif m1 == 5:							# b 180
				
					send_cmd(-1, -1, angle2steps(180), -1, -1)
				else:
					#send_cmd(-1, -1, -1, 0, -1)
					#wait4arduino()
					send_cmd(-1, settings[m1-5]['pos_a'], settings[m1-5]['pos_b'], -1, -1)
				m1 = 0
				level = 0
				
				#level = 1
			main.erase()
				
				
		elif x == curses.KEY_BACKSPACE:					# BACKSPACE
			if level > 0:
				level -= 1
				m1 = 0
				main.erase()
		
		

curses.wrapper(main_window)
