/*
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
 */


module sdram_controller # (
	/* Address parameters */
	parameter ROW_WIDTH = 12,
	parameter COL_WIDTH = 9,
	parameter BANK_WIDTH = 2,		// 2 bits wide for 4 banks.
	
	parameter SOC_SIDE_ADDR_WIDTH = BANK_WIDTH + ROW_WIDTH + COL_WIDTH,
	parameter SDRAM_ADDR_WIDTH = (ROW_WIDTH > COL_WIDTH ? ROW_WIDTH : COL_WIDTH),

	/* Timing parameters */
	parameter CLK_FREQUENCY_MHZ = 100,	// Clock frequency [MHz].
	parameter REFRESH_TIME_MS = 64,		// Refresh period [ms].
	parameter REFRESH_COUNT = 4096		// Number of refresh cycles per refresh period.
) (
	input wire clk,

	/* SOC interface */
	input wire soc_side_rst_n,
	output reg soc_side_busy,

	// Address
	input wire [SOC_SIDE_ADDR_WIDTH - 1 : 0] soc_side_addr,
	input wire [3:0] soc_side_wr_mask,		// These are the mem_wstrb signals in the PicoRV32 SOC.

	// Write
	input wire [31:0] soc_side_wr_data,
	input wire soc_side_wr_enable,

	// Read
	output wire [31:0] soc_side_rd_data,
	input wire soc_side_rd_enable,
	output wire soc_side_rd_ready,

	/* SDRAM side */
	output wire [SDRAM_ADDR_WIDTH - 1 : 0] ram_side_addr,	// SDRAM chips 0 and 1, A0 to A11 pins.
	output wire [BANK_WIDTH - 1 : 0] ram_side_bank_addr,	// SDRAM chips 0 and 1, BS0 and BS1 pins.

	output wire ram_side_ldqm_pin_chip0,	// SDRAM chip 0, LDQM pin.
	output wire ram_side_udqm_pin_chip0,	// SDRAM chip 0, UDQM pin.
	inout wire [15:0] ram_side_data_chip0,	// SDRAM chip 0, DQ0 to DQ15 pins.

	output wire ram_side_ldqm_pin_chip1,	// SDRAM chip 1, LDQM pin.
	output wire ram_side_udqm_pin_chip1,	// SDRAM chip 1, UDQM pin.
	inout wire [15:0] ram_side_data_chip1,	// SDRAM chip 1, DQ0 to DQ15 pins.

	output wire ram_side_clock_enable,		// SDRAM chips 0 and 1, CKE pin.
	output wire ram_side_cs_n,				// SDRAM chips 0 and 1, CS pin.
	output wire ram_side_ras_n,				// SDRAM chips 0 and 1, RAS pin.
	output wire ram_side_cas_n,				// SDRAM chips 0 and 1, CAS pin.
	output wire ram_side_wr_enable			// SDRAM chips 0 and 1, WE pin.
);


/* 
	Local Parameters. 
*/

localparam INIT_PAUSE_US = 200;
localparam INIT_PAUSE_CYCLES = INIT_PAUSE_US * CLK_FREQUENCY_MHZ;

localparam CYCLES_BETWEEN_REFRESH = (CLK_FREQUENCY_MHZ * 1_000 * REFRESH_TIME_MS) / REFRESH_COUNT;

// STATES
localparam	IDLE = 5'b00000;

			// SDRAM initialization
localparam	INIT_NOP1 = 5'b01000,
			INIT_PRE1 = 5'b01001,
			INIT_NOP1_1 = 5'b00101,
			INIT_REF1 = 5'b01010,
			INIT_NOP2 = 5'b01011,
			INIT_REF2 = 5'b01100,
			INIT_NOP3 = 5'b01101,
			INIT_LOAD = 5'b01110,
			INIT_NOP4 = 5'b01111;

			// SDRAM refresh
localparam	REF_PRE  = 5'b00001,
			REF_NOP1 = 5'b00010,
			REF_REF  = 5'b00011,
			REF_NOP2 = 5'b00100;

			// Read SDRAM
localparam	READ_ACT  = 5'b10000,
			READ_NOP1 = 5'b10001,
			READ_CAS  = 5'b10010,
			READ_NOP2 = 5'b10011,
			READ_READ = 5'b10100;

			// Write SDRAM
localparam	WRIT_ACT  = 5'b11000,
			WRIT_NOP1 = 5'b11001,
			WRIT_CAS  = 5'b11010,
			WRIT_NOP2 = 5'b11011;

// SDRAM Commands        CCRCWBBA
//                       ESSSE100
localparam	CMD_PALL = 8'b10010001,
			CMD_REF  = 8'b10001000,
			CMD_NOP  = 8'b10111000,
			CMD_MRS  = 8'b1000000x,
			CMD_BACT = 8'b10011xxx,
			CMD_READ = 8'b10101xx1,
			CMD_WRIT = 8'b10100xx1;


/* 
	Internal Registers and Wires. 
*/

wire in_refresh_cycle;

// Internal registers for CPU interface.
reg [31:0] wr_data_r;
reg [31:0] rd_data_r;
reg rd_ready_r;

// Internal registers for SDRAM address generation.
reg [SDRAM_ADDR_WIDTH-1:0] addr_r;
reg [BANK_WIDTH-1:0] bank_addr_r;

// Internal registers for Byte mask signals.
reg [3:0] sdram_side_wr_mask_reg;

// Internal state machine signals.
reg [14:0] init_pause_counter;  // Counter for 200us init delay.
reg [3:0] state_cnt;
reg [9:0] refresh_cnt;
reg [7:0] command;
reg [4:0] state;
reg [7:0] command_nxt;
reg [3:0] state_cnt_nxt;
reg [4:0] next;


// Signal to detect refresh cycle states.
assign in_refresh_cycle = (state == REF_PRE) || (state == REF_NOP1) || (state == REF_REF) || (state == REF_NOP2);

/* Output assignments. */
assign {ram_side_clock_enable, ram_side_cs_n, ram_side_ras_n, ram_side_cas_n, ram_side_wr_enable} = command[7:3];
// state[4] is set if mode is read/write.
assign ram_side_bank_addr = (state[4]) ? bank_addr_r : command[2:1];
assign ram_side_addr = (state[4] | state == INIT_LOAD) ? addr_r : { {(SDRAM_ADDR_WIDTH - 11){1'b0}}, command[0], 10'd0 };

assign soc_side_rd_data = rd_data_r;
assign soc_side_rd_ready = rd_ready_r;

assign ram_side_ldqm_pin_chip0 = sdram_side_wr_mask_reg[0];
assign ram_side_udqm_pin_chip0 = sdram_side_wr_mask_reg[1];
assign ram_side_ldqm_pin_chip1 = sdram_side_wr_mask_reg[2];
assign ram_side_udqm_pin_chip1 = sdram_side_wr_mask_reg[3];

assign ram_side_data_chip0 = (state == WRIT_CAS) ? wr_data_r[15:0] : 16'bz;
assign ram_side_data_chip1 = (state == WRIT_CAS) ? wr_data_r[31:16] : 16'bz;


/* Host interface. All registered on positive edge clock. */
always @ (posedge clk)
begin
	if (~soc_side_rst_n)
		begin
			state <= INIT_NOP1;
			command <= CMD_NOP;
			state_cnt <= 4'hf;
			wr_data_r <= 32'b0;
			rd_data_r <= 32'b0;
			soc_side_busy <= 1'b0;
		end
	else
		begin
			state <= next;
			command <= command_nxt;

			if (!state_cnt)
				state_cnt <= state_cnt_nxt;
			else
				state_cnt <= state_cnt - 1'b1;

			if (soc_side_wr_enable)
				wr_data_r <= soc_side_wr_data;

			if (state == READ_READ)
				begin
					rd_data_r <= {ram_side_data_chip1, ram_side_data_chip0};
					rd_ready_r <= 1'b1;
				end
			else
				rd_ready_r <= 1'b0;

            // Indicate that the controller is busy during read/write operations and during refresh cycles.
            soc_side_busy <= state[4] || in_refresh_cycle;
		end
end

/* Handle refresh counter. */
always @ (posedge clk)
begin
	if (~soc_side_rst_n)
		refresh_cnt <= 10'b0;
	else if (state == REF_NOP2)
		refresh_cnt <= 10'b0;
	else
		refresh_cnt <= refresh_cnt + 1'b1;
end

/* Handle logic for sending addresses to SDRAM based on current state. */
always @*
begin
	// Set write mask signals based on state.
	if (state[4])	// Checks if mode is read/write.
		/*
			PicoRV32 SOC, mem_wstrb signals:
				soc_side_wr_mask = 0000 --> No write. Read memory.
				soc_side_wr_mask = 1111 --> Write 32 bits.
				soc_side_wr_mask = 1100 --> Write upper 16 bits.
				soc_side_wr_mask = 0011 --> Write lower 16 bits.
				soc_side_wr_mask = 0001 --> Write Byte 0.
				soc_side_wr_mask = 0010 --> Write Byte 1.
				soc_side_wr_mask = 0100 --> Write Byte 2.
				soc_side_wr_mask = 1000 --> Write Byte 3.

			The mem_wstrb signals of the SoC are inverted because the LDQM and UDQM pins of the 
			SDRAM set the input/output buffers to HIGH-Z with 1 and enable them with 0.
		*/
		if (soc_side_wr_mask)								// If the soc wants to write in the memory (soc_side_wr_mask diferent from 0000).
			sdram_side_wr_mask_reg = ~soc_side_wr_mask;		// Use the 4-bit write mask from the soc (mem_wstrb signals).
		else
			sdram_side_wr_mask_reg = 4'b0000;				// If the soc wants to read from the memory, enable the buffers.
	else
		sdram_side_wr_mask_reg = 4'b1111;	// If mode is not read/write, mask all Bytes (DQM high) so that SDRAM drives buffers to HIGH-Z.

	bank_addr_r = 2'b00;
	addr_r = {SDRAM_ADDR_WIDTH{1'b0}};

	if (state == READ_ACT | state == WRIT_ACT)
		begin
			bank_addr_r = soc_side_addr[ SOC_SIDE_ADDR_WIDTH - 1 : SOC_SIDE_ADDR_WIDTH - BANK_WIDTH ];
			addr_r = soc_side_addr[ SOC_SIDE_ADDR_WIDTH - (BANK_WIDTH + 1) : SOC_SIDE_ADDR_WIDTH - (BANK_WIDTH + ROW_WIDTH) ];
		end
	else if (state == READ_CAS | state == WRIT_CAS)
		begin
			// Send Column Address
			// Set bank to bank to precharge
			bank_addr_r = soc_side_addr[ SOC_SIDE_ADDR_WIDTH - 1 : SOC_SIDE_ADDR_WIDTH - BANK_WIDTH ];

			/*
				Examples for math:
										 BANK     ROW      COL
					SOC_SIDE_ADDR_WIDTH   2    +   12   +   9   = 23 
					SDRAM_ADDR_WIDTH 13

					Set CAS address to:
					0s,
					1 (A10 is always for auto precharge),
					0s,
					column address
			*/
			addr_r = { {(SDRAM_ADDR_WIDTH - 11){1'b0}},
						1'b1,	/* A10 */
						{(10 - COL_WIDTH){1'b0}},
						soc_side_addr[COL_WIDTH - 1 : 0]
					};
		end
	else if (state == INIT_LOAD)
		begin
			// Program mode register during load cycle
			//                                       B  C  SB
			//                                       R  A  EUR
			//                                       S  S-3Q ST
			//                                       T  654L210
			addr_r = { {(SDRAM_ADDR_WIDTH - 10){1'b0}}, 10'b1000110000 };
		end
end

/* Next state logic. */
always @*
begin
	state_cnt_nxt = 4'd0;
	command_nxt = CMD_NOP;

	if (state == IDLE)
		begin
			// Monitor for refresh or hold
			if (refresh_cnt >= CYCLES_BETWEEN_REFRESH)
				begin
					next = REF_PRE;
					command_nxt = CMD_PALL;
				end
			else if (soc_side_rd_enable)
				begin
					next = READ_ACT;
					command_nxt = CMD_BACT;
				end
			else if (soc_side_wr_enable)
				begin
					next = WRIT_ACT;
					command_nxt = CMD_BACT;
				end
			else
				begin
					// HOLD
					next = IDLE;
				end
		end
	else if (!state_cnt)

		case (state)
			// INIT ENGINE --------------
			INIT_NOP1:
				begin
					next = INIT_PRE1;
					command_nxt = CMD_PALL;
				end

			INIT_PRE1:
				begin
					next = INIT_NOP1_1;
				end

			INIT_NOP1_1:
				begin
					next = INIT_REF1;
					command_nxt = CMD_REF;
				end

			INIT_REF1:
				begin
					next = INIT_NOP2;
					state_cnt_nxt = 4'd7;
				end

			INIT_NOP2:
				begin
					next = INIT_REF2;
					command_nxt = CMD_REF;
				end

			INIT_REF2:
				begin
					next = INIT_NOP3;
					state_cnt_nxt = 4'd7;
				end

			INIT_NOP3:
				begin
					next = INIT_LOAD;
					command_nxt = CMD_MRS;
				end

			INIT_LOAD:
				begin
					next = INIT_NOP4;
					state_cnt_nxt = 4'd1;
				end
			// INIT_NOP4: default - IDLE

			// REFRESH --------------
			REF_PRE:
				begin
					next = REF_NOP1;
				end

			REF_NOP1:
				begin
					next = REF_REF;
					command_nxt = CMD_REF;
				end

			REF_REF:
				begin
					next = REF_NOP2;
					state_cnt_nxt = 4'd7;
				end
			// REF_NOP2: default - IDLE

			// WRITE --------------
			WRIT_ACT:
				begin
					next = WRIT_NOP1;
					state_cnt_nxt = 4'd1;
				end

			WRIT_NOP1:
				begin
					next = WRIT_CAS;
					command_nxt = CMD_WRIT;
				end

			WRIT_CAS:
				begin
					next = WRIT_NOP2;
					state_cnt_nxt = 4'd1;
				end
			// WRIT_NOP2: default - IDLE

			// READ --------------
			READ_ACT:
				begin
					next = READ_NOP1;
					state_cnt_nxt = 4'd1;
				end

			READ_NOP1:
				begin
					next = READ_CAS;
					command_nxt = CMD_READ;
				end

			READ_CAS:
				begin
					next = READ_NOP2;
					state_cnt_nxt = 4'd1;
				end

			READ_NOP2:
				begin
					next = READ_READ;
				end
			// READ_READ: default - IDLE

			default:
				begin
					next = IDLE;
				end
		endcase
		
	else
		begin
			// Counter Not Reached - HOLD
			next = state;
			command_nxt = command;
		end
end


endmodule

