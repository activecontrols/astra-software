#ifndef PLATFORM_BRIDGE_TEMPLATE
#define PLATFORM_BRIDGE_TEMPLATE

#if defined(TARGET_TEENSY41)


#elif defined(TARGET_NATIVE)

template <typename T>
size_t usb_serial_class::print(T in) {
	std::ostringstream oss;
	oss << in;
	std::string str = oss.str();
	std::cout << in;
	return str.size();
}
template <typename T>
size_t usb_serial_class::println(T in) {
	std::ostringstream oss;
	oss << in;
	std::string str = oss.str();
	std::cout << in << std::endl;
	return str.size();
}


template <typename T>
size_t File::print(T in) {
	this->stream.seekp(0, std::ios::end);
	std::ostringstream oss;
	oss << in;
	std::string str = oss.str();
	this->stream << in;
	return str.size();
}

template <typename T>
size_t File::println(T in) {
	this->stream.seekp(0, std::ios::end);
	std::ostringstream oss;
	oss << in;
	std::string str = oss.str();
	this->stream << in << std::endl;
	return str.size();
}

#endif

#endif