

import threading
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from config import config_flag_is_set, \
config_get_from_message_queue, \
config_check_if_sample_queue_is_empty, \
config_get_from_sample_queue


DEFAULT_REFRESH_RATE = 100
X_MIN = 0
X_MAX = 1000
Y_MIN = 0
Y_MAX = 5


class RealTimePlot(object):
	def __init__(self):
		self.fig = plt.figure()
		self.ax = self.fig.add_subplot(1, 1, 1)
		self.x_samples = []
		self.y_samples = []
		self.refresh_rate = DEFAULT_REFRESH_RATE

	def update_function(self, idx, x_samples, y_samples):
		queue_is_empty = config_check_if_sample_queue_is_empty()
		if( not queue_is_empty ):
			self.x_samples, self.y_samples = config_get_from_sample_queue()
			self.ax.clear()
			self.ax.plot(self.x_samples, self.y_samples)
			# plt.xlim(X_MIN, X_MAX)
			plt.ylim(Y_MIN, Y_MAX)

	def run(self):
		ani = animation.FuncAnimation(self.fig, self.update_function, fargs = (self.x_samples, self.y_samples), interval = self.refresh_rate)
		plt.show()


def real_time_plot_thread_target(rtp : RealTimePlot):
	rtp.run()


def real_time_plot_create_and_start_thread(rtp : RealTimePlot):
	rtp_thread = threading.Thread(target = real_time_plot_thread_target, args = (rtp,))
	rtp_thread.start()