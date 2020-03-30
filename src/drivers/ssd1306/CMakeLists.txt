px4_add_module(
	MODULE drivers__ssd1306
	MAIN ssd1306
	SRCS
		ssd1306_spi.cpp
		ssd1306.cpp
	DEPENDS
		px4_work_queue
	)
