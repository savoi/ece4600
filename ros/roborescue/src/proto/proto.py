from bitstring import Bits, BitArray, BitStream

class ControlPacket:

	PACKET_SIZE = 5				# packet size in bytes
	BYTE_SIZE = 8

	def __init__(self, data):
		self.ba = Bits(bytes=data)	
		if ba.length != self.PACKET_SIZE * self.BYTE_SIZE:
			raise PacketError('Incorrect packet size.')

		# first byte
		self.fwdbck = ba.bin[0]							# forward/backward
		self.fwdbckspeed = ba.bin[1:8]					# forward/backward speed

		# second byte
		self.leftright = ba.bin[8]						# left/right
		self.leftrightspeed = ba.bin[9:16]				# left/right speed

		# third byte
		self.frontflipper = ba.bin[16:18]				# front flipper arm
		self.backflipper = ba.bin[18:21]				# back flipper arm

		# fourth byte
		self.armcmd = ba.bin[32:34]						# manipulator arm cmd
		self.pan = ba.bin[34:36]						# camera pan cmd
		self.tilt = ba.bin[36:38]						# camera tilt cmd
		self.lights = ba.bin[38]						# lights on/off

		# fifth byte
		self.emergtoggle = ba.bin[40:42]				# emergency stop toggle
		self.robotshutdown = ba.bin[42:44]				# robot shutdown
		self.camone = ba.bin[44:46]						# camera one toggle
		self.camtwo = ba.bin[46:48]						# camera two toggle

class PacketError(Exception):
	def __init__(self, val):
		self.val = val
	def __str__(self):
		return repr(self.val)
