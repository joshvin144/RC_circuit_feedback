

import serial
import threading
from config import config_set_flag, config_put_in_message_queue, config_put_in_sample_queue
from helpers import parse_by_delimiter


DEFAULT_PORT = "/dev/cu.usbmodem103"
DEFAULT_BAUDRATE = 115200 # Bits per second
DEFAULT_TIMEOUT = 20 # Seconds
DEFAULT_FNAME = "temp.csv"
DEFAULT_BATCH_SIZE = 258 # Bytes
DEFAULT_NBATCHES = 1032


class SerialConnection(serial.Serial):
	def __init__(self):
		super(SerialConnection, self).__init__()

	def set_properties(self, port = DEFAULT_PORT, baudrate = DEFAULT_BAUDRATE, timeout = DEFAULT_TIMEOUT):
		assert ( False == self.is_open ), ( "Serial connection was already open!" )
		self.port = port
		self.baudrate = baudrate
		self.timeout = timeout
		self.batch_size = DEFAULT_BATCH_SIZE
		self.nbatches = DEFAULT_NBATCHES
		self.fp = None

	def read_batch(self):
		byte_values = self.read(self.batch_size)
		serial_data = byte_values.decode('utf-8')
		return serial_data

	def save_batch(self, serial_data : str):
		self.fp.write(serial_data)

	def format_batch(self, serial_data : str):
		data_array = parse_by_delimiter(serial_data)
		return data_array

	def cat_batch(self, dest : list, src : list):
		return dest + src

	def read_save_and_queue_batches(self):
		serial_data = self.read_batch()
		self.save_batch(serial_data)
		data_array = self.format_batch(serial_data)

		dest = data_array
		dest_count = 1
		divisor = 1

		for batch_number in range(self.nbatches):
			serial_data = self.read_batch()
			self.save_batch(serial_data)
			data_array = self.format_batch(serial_data)
			dest = self.cat_batch(dest, data_array)
			dest_count += 1
			if( 0 == ( dest_count % divisor ) ):
				config_put_in_sample_queue([idx for idx in range(len(dest))], dest)
				dest = []
				dest_count = 0

	def check_and_open(self, fname = DEFAULT_FNAME):
		assert ( False == self.is_open ), ( "Serial connection was already open!" )
		# Open the file and write to it
		with open(fname, "w+") as self.fp:
			config_put_in_message_queue( "Opened output file..." )
			self.open()
			config_put_in_message_queue( "Opened serial connection..." )
			self.read_save_and_queue_batches()
			self.close()
			config_put_in_message_queue( "Closed serial connection..." )
		config_put_in_message_queue( "Closed output file..." )
		config_set_flag()


def serial_connection_thread_target(connection : SerialConnection):
	connection.check_and_open()


def serial_connection_create_and_start_thread(connection : SerialConnection):
	serial_connection_thread = threading.Thread(target = serial_connection_thread_target, args = (connection,))
	serial_connection_thread.start()

