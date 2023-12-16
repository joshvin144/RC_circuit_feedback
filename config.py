

import queue


MAX_QUEUE_SIZE = 256

flag_is_set = False
# For printing from inside a thread
message_queue = queue.Queue(maxsize = MAX_QUEUE_SIZE)
# For passing data from the serial connection thread to the real time plot
x_sample_queue = queue.Queue(maxsize = MAX_QUEUE_SIZE)
y_sample_queue = queue.Queue(maxsize = MAX_QUEUE_SIZE)


def config_set_flag():
	flag_is_set = True


def config_clear_flag():
	flag_is_set = False


def config_flag_is_set():
	return flag_is_set


def config_check_if_message_queue_empty():
	return message_queue.empty()


def config_put_in_message_queue(message : str):
	message_queue.put_nowait(message)


def config_get_from_message_queue():
	return message_queue.get_nowait()


def config_put_in_sample_queue(x_samples : list, y_samples: list):
	x_sample_queue.put_nowait(x_samples)
	y_sample_queue.put_nowait(y_samples)


def config_check_if_sample_queue_is_empty():
    return ( x_sample_queue.empty() or y_sample_queue.empty() )


def config_get_from_sample_queue():
	return x_sample_queue.get_nowait(), y_sample_queue.get_nowait()