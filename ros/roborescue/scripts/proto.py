from bitstring import Bits, BitArray, BitStream

class ControlPacket:

	PACKET_SIZE = 5				# packet size in bytes
	BYTE_SIZE = 8

	def __init__(self, data):
		self.ba = Bits(bytes=data)	
		if self.ba.length != self.PACKET_SIZE * self.BYTE_SIZE:
			raise PacketError('Incorrect packet size.')

		# first byte
		self.fwdbck = self.ba.bin[0]						# forward/backward
		self.fwdbckspeed = self.ba.bin[1:8]					# forward/backward speed

		# second byte
		self.leftright = self.ba.bin[8]						# left/right
		self.leftrightspeed = self.ba.bin[9:16]				# left/right speed

		# third byte
		self.frontflipper = self.ba.bin[16:18]				# front flipper arm
		self.backflipper = self.ba.bin[18:21]				# back flipper arm

		# fourth byte
		self.armcmd = self.ba.bin[24:26]					# manipulator arm cmd
		self.pan = self.ba.bin[26:28]						# camera pan cmd
		self.tilt = self.ba.bin[28:30]						# camera tilt cmd
		self.lights = self.ba.bin[30]						# lights on/off

		# fifth byte
		self.emergtoggle = self.ba.bin[32:34]				# emergency stop toggle
		self.robotshutdown = self.ba.bin[34:36]				# robot shutdown
		self.camone = self.ba.bin[36:38]					# camera one toggle
		self.camtwo = self.ba.bin[38:40]					# camera two toggle

	def __str__(self):
		return 'FOURTH BYTE: {0}\n' \
			   'LIGHTS: {1}'.format(self.ba.bin[24:32], self.lights)

class PacketError(Exception):
	def __init__(self, val):
		self.val = val
	def __str__(self):
		return repr(self.val)
