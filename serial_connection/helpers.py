

DEFAULT_DELIMITER = ','


def parse_by_delimiter(char_array : str, delimiter = DEFAULT_DELIMITER) -> list:
	data_string = ""
	data_array = []
	num_chars = len( char_array )
	for char_idx in range( num_chars ):
		if( delimiter == char_array[char_idx] ):
			data_float = float( data_string )
			data_array.append( data_float )
			data_string = ""
		else:
			data_string = data_string + char_array[char_idx]
	return data_array