from serial import Serial as ser

p = ser('/dev/ttyAMA0', 115200, timeout=1)

if p.read() == b'':
	print("LIDAR.py: no LIDAR serial connection.")

def getFrame():
	frame = []
	header = b''
	while True:
		header = p.read(2)
		if int.from_bytes(header, byteorder='big') == 22873:
			for a in range(7):
				frame.append(int.from_bytes(p.read(), byteorder='big'))
			if frame[6] == (sum(frame[0:6]) + 89 + 89) % 256:
				return frame
			else:
				frame = []

def getDist(frame=None):
	if frame is None:
		frame = getFrame()
	dist = bytes([frame[3], frame[2]])
	return int.from_bytes(dist, byteorder='big')

