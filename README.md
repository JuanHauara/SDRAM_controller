# SDRAM_controller

	Simple SDRAM controller for two Winbond W9812G6KH-5I SDRAM chips in parallel.
	It is designed to use two of these 2M word x 4 bank x 16 bits chips in parallel to 
	achieve a total of 32MB (4M words x 32 bits).

	Default options
		CLK_FREQUENCY_MHZ = 100MHz
		CAS 3

	Very simple CPU interface

		- No burst support.
		
		- soc_side_rst_n: Start init SDRAM process.

		- soc_side_busy: Indicate that the controller is busy during read/write operations 
		  and during refresh cycles. The CPU must wait for soc_side_busy to be low before 
		  sending commands to the SDRAM.
		
		- soc_side_addr: Address for read/write 32 bits of data.

		- soc_side_wr_mask: 0000 --> No write/read memory.
							1111 --> Write 32 bits.
							1100 --> Write upper 16 bits.
							0011 --> Write lower 16 bits.
							0001 --> Write Byte 0.
							0010 --> Write Byte 1.
							0100 --> Write Byte 2.
							1000 --> Write Byte 3.

		- soc_side_wr_data: Data for writing, latched in when soc_side_wr_enable is high.

		- soc_side_wr_enable: On clk posedge soc_side_addr and soc_side_wr_data will be 
		  latched in, after a few clocks data will be written to the SDRAM.

		- soc_side_rd_data: Data for reading, comes available a few clocks after 
		  soc_side_rd_enable and soc_side_addr are presented on the bus.

		- soc_side_rd_enable: On clk posedge soc_side_addr will be latched in, after a 
		  few clocks data will be available on the soc_side_rd_data port.

		- soc_side_rd_ready: This signal is used to notify the CPU when the read data is 
		  available on the soc_side_rd_data bus.
