

import argparse
from config import config_check_if_message_queue_empty, config_get_from_message_queue
from serial_connection import SerialConnection, serial_connection_create_and_start_thread
from real_time_plot import RealTimePlot, real_time_plot_create_and_start_thread


def create_argument_parser() -> argparse.ArgumentParser:
	parser = argparse.ArgumentParser()
	parser.add_argument("-f", "--fname", type = str, help = "Sets the filename; this is the file that the serial connection directs data to; if not specified, the filename defaults to temp.txt")
	parser.add_argument("-p", "--plot", action = "store_true", help = "Plots data in real time")
	return parser


def main(argc : int = None, argv : list = None) -> int:
	# Accept command line arguments
	parser = create_argument_parser()
	args = parser.parse_args()

	# Create serial connection
	connection = SerialConnection()
	connection.set_properties()
	# connection.check_and_open()

	# Spin off thread to receive data over the serial connection
	serial_connection_create_and_start_thread(connection)

	# Create plot
	rtp = RealTimePlot()
	rtp.run()

	# Spin off thread to plot data in real time
	# real_time_plot_create_and_start_thread(rtp)

    # Dump messages from message queue
	while( not config_check_if_message_queue_empty() ):
		print( config_get_from_message_queue() )

	# Exit success
	return 0


if __name__ == "__main__":
	_ = main()

