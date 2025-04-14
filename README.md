# SDRAM_controller

Only 72 slices in a MachXo2 FPGA.

Simple SDRAM controller for two Winbond W9812G6KH-5I SDRAM chips in parallel.
It is designed to use two of these 2M word x 4 bank x 16 bits (8M words x 16 bits) chips in parallel to 
achieve a total of 8M x 32 bit words = 32MB RAM.

Default options
	CLK_FREQUENCY_MHZ = 80MHz
	CAS 3

Very simple CPU interface

	- The CPU sees RAM organized into 8 million 32-bit words and using the soc_side_wr_mask_port[4] signals, 
		it can select which of the 4 Bytes of each word to write to.

	- No burst support.
	
	- reset_n_port: Starts the initialization sequence for the SDRAM.

	- soc_side_busy_port: Indicate that the controller is busy during read/write operations, refresh cycles 
		and during the initialization sequence. The CPU must wait for soc_side_busy_port to be low before 
		sending commands to the SDRAM.
	
	- soc_side_addr_port: Address for read/write 32 bits data.

	- soc_side_wr_mask_port: 0000 --> No write/read memory.
							1111 --> Write 32 bits.
							1100 --> Write upper 16 bits.
							0011 --> Write lower 16 bits.
							0001 --> Write Byte 0.
							0010 --> Write Byte 1.
							0100 --> Write Byte 2.
							1000 --> Write Byte 3.

	Write data:
		- soc_side_wr_data_port: Data for writing, latched in on clk posedge if soc_side_wr_en_port is high.

		- soc_side_wr_en_port: On clk posedge, if soc_side_wr_en_port is high soc_side_wr_data_port will be 
		latched in, after a few clocks data will be written to the SDRAM.

		- soc_side_ready_port: This signal is used to notify the CPU when read data is available on the 
		soc_side_rd_data_port bus and also when a data write to memory has finished.

	Read data:
		- soc_side_rd_data_port: Data for reading, comes available a few clocks after 
		soc_side_rd_en_port and soc_side_addr_port are presented on the bus.

		- soc_side_rd_en_port: If asserted (high), soc_side_addr_port is sampled during the READ_CAS and 
		READ_BANK_ACTIVATE states. Data becomes available on soc_side_rd_data_port after a few clock cycles.
