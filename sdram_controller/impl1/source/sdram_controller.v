/*
	Simple SDRAM controller for two Winbond W9812G6KH-5I SDRAM chips in parallel.
	It is designed to use two of these 2M word x 4 bank x 16 bits (8M words x 16 bits) chips in parallel to 
	achieve a total of 8M x 32 bit words = 32MB RAM.

	The address bus must be 25 bits to address a total of 32MB.

	Default options
		L_CLK_FREQUENCY_MHZ = 80MHz
		CAS 3

	Very simple CPU interface

		- The CPU sees RAM organized into 8 million 32-bit words and using the soc_side_wmask[4] signals, 
		  it can select which of the 4 Bytes of each word to write to.

		- No burst support.
		
		- reset_n: Starts the initialization sequence for the SDRAM.

		- soc_side_busy: Indicate that the controller is busy during read/write operations, refresh cycles 
		  and during the initialization sequence. The CPU must wait for soc_side_busy to be low before 
		  sending commands to the SDRAM.
		
		- soc_side_byte_addr_bus: Byte-oriented address. 25 bits to address a total of 32MB.

		- soc_side_wmask: 0000 --> No write/read memory.
								1111 --> Write 32 bits.
								1100 --> Write upper 16 bits.
								0011 --> Write lower 16 bits.
								0001 --> Write Byte 0.
								0010 --> Write Byte 1.
								0100 --> Write Byte 2.
								1000 --> Write Byte 3.

		Write data:
			- soc_side_wdata: Data for writing, latched in on clk posedge if soc_side_wen is high.

			- soc_side_wen: On clk posedge, if soc_side_wen is high soc_side_wdata will be 
			latched in, after a few clocks data will be written to the SDRAM.

			- soc_side_ready: This signal is used to notify the CPU when a data write to memory has 
			finished or read data is available on the soc_side_rdata bus.

		Read data:
			- soc_side_rdata: Data for reading, comes available a few clocks after 
			soc_side_ren and soc_side_word_addr_bus are presented on the bus.

			- soc_side_ren: If asserted (high), soc_side_word_addr_bus is sampled during the READ_CAS and 
			READ_BANK_ACTIVATE states. Data becomes available on soc_side_rdata after a few clock cycles.
 */


module sdram_controller (
	input wire clk,
	input wire reset_n,

	/* SOC interface */
	output wire soc_side_busy,
	output wire soc_side_ready,

	// Address.
	input wire [24:0] soc_side_byte_addr_bus,

	// Read data.
	output wire [31:0] soc_side_rdata,
	input wire soc_side_ren,

	// Write data.
	input wire [31:0] soc_side_wdata,
	input wire [3:0] soc_side_wmask,	// These are the mem_wstrb signals in the PicoRV32 SOC.
	input wire soc_side_wen,

	/* SDRAM interface */
	output wire [11:0] ram_side_addr_bus,	// SDRAM chips 0 and 1, A0 to A11 pins.
	output wire [1:0] ram_side_bank_addr,	// SDRAM chips 0 and 1, BS0 and BS1 pins.

	output wire ram_side_chip0_ldqm,			// SDRAM chip 0, LDQM pin.
	output wire ram_side_chip0_udqm,			// SDRAM chip 0, UDQM pin.
	inout wire [15:0] ram_side_chip0_data_bus,	// SDRAM chip 0, DQ0 to DQ15 pins.

	output wire ram_side_chip1_ldqm,			// SDRAM chip 1, LDQM pin.
	output wire ram_side_chip1_udqm,			// SDRAM chip 1, UDQM pin.
	inout wire [15:0] ram_side_chip1_data_bus,	// SDRAM chip 1, DQ0 to DQ15 pins.

	output wire ram_side_cs_n,		// SDRAM chips 0 and 1, CS pin.
	output wire ram_side_ras_n,		// SDRAM chips 0 and 1, RAS pin.
	output wire ram_side_cas_n,		// SDRAM chips 0 and 1, CAS pin.
	output wire ram_side_wen,		// SDRAM chips 0 and 1, WE pin.

	/*
		During normal access mode, CKE must be held high, enabling the clock.
		This pin is only used in Power Down mode, Suspend mode, or Self 
		Refresh mode, this controller does not implement these modes, so CKE 
		is always held high.
	*/
	output wire ram_side_cken	// SDRAM chips 0 and 1, CKE pin.
);


	//-------------------------------
	// Local parameters
	//-------------------------------

	/* Timing parameters */
	localparam L_CLK_FREQUENCY_MHZ = 80;	// Clock frequency [MHz].
	localparam L_REFRESH_TIME_MS = 64;		// Refresh period [ms].
	localparam L_REFRESH_COUNT = 4096;		// Number of refresh cycles per refresh period.
	localparam L_CYCLES_BETWEEN_REFRESH = (L_CLK_FREQUENCY_MHZ * 1_000 * L_REFRESH_TIME_MS) / L_REFRESH_COUNT;

	/* Address parameters */
	localparam L_ROW_WIDTH = 12;
	localparam L_COL_WIDTH = 9;
	localparam L_BANK_ADDR_WIDTH = 2;		// 2 bits wide for 4 banks.
	localparam L_RAM_ADDR_BUS_WIDTH = L_ROW_WIDTH + L_COL_WIDTH + L_BANK_ADDR_WIDTH;  // 23 bits bus to address 8 million 32 bit words = 32MB RAM.
	localparam L_SDRAM_ADDR_WIDTH = L_ROW_WIDTH;


	// ------------------------------------------------------------------------
	// Times: Table "9.5 AC Characteristics and Operating Condition", page 15.
	// ------------------------------------------------------------------------

	// Timings for SDRAM initialization.
	// -------------------------------------------
	localparam L_INIT_PAUSE_US = 300;  // Initial pause of 200us + safety margin.
	localparam L_INIT_PAUSE_CYCLES = L_INIT_PAUSE_US * L_CLK_FREQUENCY_MHZ;

	localparam L_TRP_NS = 18;  // Precharge to Active Command Period, tRP: 15ns + safety margin. Wait time after issuing the precharge all banks command.
	localparam L_TRP_CYCLES = ((L_TRP_NS * L_CLK_FREQUENCY_MHZ) + 999) / 1000;  // Rounded up number of wait cycles to meet tRP time.

	localparam L_INIT_REFRESH_COUNT = 8;  // Number of consecutive refresh cycles required during initialization.

	localparam L_TRC_NS = 65;  // Ref/Active to Ref/Active Command Period, tRC: 55-65ns. Wait time after issuing the auto refresh command.
	localparam L_TRC_CYCLES = ((L_TRC_NS * L_CLK_FREQUENCY_MHZ) + 999) / 1000;  // Rounded up number of wait cycles to meet tRC time.

	localparam L_TRSC_CYCLES = 2;  // Mode Register Set Cycle Time, tRSC: 2 clock cycles. Wait time after issuing the L_CMD_MRS Mode Register Set (MRS) command.

	// Write and read timings.
	// -------------------------------
	localparam L_TRCD_NS = 18;  // Active to Read/Write Command Delay Time, tRCD: 15ns + safety margin. Wait time after issuing the L_CMD_BANK_ACTIVATE command.
	localparam L_TRCD_CYCLES = ((L_TRCD_NS * L_CLK_FREQUENCY_MHZ) + 999) / 1000;  // Rounded up number of wait cycles to meet tRCD time.

	localparam L_TWR_CYCLES = 2;  // Write Recovery Time, tWR: 2 clock cycles. Wait time after issuing the L_CMD_WRITE command.

	localparam L_CAS_LATENCY = 3;
	// ------------------------------------------------------------------------


	//-------------------------------
	// State Definitions
	//-------------------------------
	/*
		Gray encoding is used.

		Advantages of Gray Encoding:
			- Glitch reduction: Since only one bit changes between adjacent states, it minimizes
			the possibility of unwanted transient states.
			- Lower power consumption: Fewer bits change during transitions, reducing
			switching activity.
			- Higher noise immunity: Reduces the probability of incorrect state transitions
			due to noise.

		5-bit state register: 2^5 = 32 maximum states.
	*/

	// SDRAM initialization.
	localparam INIT						= 5'b00000;  // Initial state.
	localparam INIT_WAIT				= 5'b00001;  // Initial pause of at least 200us.
	localparam INIT_PRECHARGE_ALL		= 5'b00011;  // Precharge all banks.
	localparam INIT_WAIT_TRP			= 5'b00010;  // Wait tRP nanoseconds after precharge all.
	localparam INIT_AUTO_REFRESH		= 5'b00110;  // 8 Auto Refresh cycles.
	localparam INIT_WAIT_TRC			= 5'b00111;  // Wait tRC after each of the 8 auto refresh cycles.
	localparam INIT_MRS					= 5'b00101;  // Mode Register Set (MRS).
	localparam INIT_WAIT_TRSC			= 5'b00100;  // Wait tRSC nanoseconds after MRS.

	// States for the Auto refresh.
	localparam REFRESH_PRECHARGE_ALL	= 5'b01100;  // Precharge all banks before refresh.
	localparam REFRESH_WAIT_TRP			= 5'b01101;  // Wait tRP nanoseconds after precharge all.
	localparam REFRESH_AUTO_REFRESH		= 5'b01111;  // Issue Auto Refresh command.
	localparam REFRESH_WAIT_TRC			= 5'b01110;  // Wait tRC after Auto Refresh.

	// States for write operation.
	localparam WRITE_BANK_ACTIVATE		= 5'b01010;  // Activate row/bank (Bank Activate Command).
	localparam WRITE_WAIT_TRCD			= 5'b01011;  // NOP after activation (wait tRCD).
	localparam WRITE_CAS				= 5'b01001;  // Write command and data (Write Command).
	localparam WRITE_WAIT_TWR			= 5'b01000;  // NOP after write (wait tWR).
	//localparam WRITE_PRECHARGE		= 5'b11000;  // Precharge bank (if auto-precharge is not used).
	//localparam WRITE_WAIT_TRP			= 5'b11001;  // NOP after precharge (wait tRP).

	// States for read operation.
	localparam READ_BANK_ACTIVATE		= 5'b11011;  // Activate row/bank (Bank Activate Command).
	localparam READ_WAIT_TRCD			= 5'b11010;  // NOP after activation (wait tRCD).
	localparam READ_CAS					= 5'b11110;  // Read command (Read Command).
	localparam READ_WAIT_CAS_LATENCY	= 5'b11111;  // Wait CAS Latency (3 cycles).
	localparam READ_DATA				= 5'b11101;  // Capture data from DQ pins.
	//localparam READ_PRECHARGE			= 5'b11100;  // Precharge bank (if auto-precharge is not used).
	//localparam READ_WAIT_TRP			= 5'b10100;  // NOP after precharge (wait tRP).

	// IDLE state
	localparam IDLE						= 5'b10101;


	//-------------------------------
	// SDRAM Command Definitions
	//-------------------------------
	/*
		See datasheet, Table 1, pag. 12.

		7 bits commands: (CS, RAS, CAS, WE, BS1, BS0, A10)

		Bits [6:3] = comando real (CS, RAS, CAS, WE)
		Bits [2:1] = selección de banco (BS1, BS0)
		Bit 0 = A10 (para auto-precharge)
	*/
	localparam L_CMD_PRECHARGE_ALL	= 7'b0010001;  // Precharge All. CS=0, RAS=0, CAS=1, WE=0, BS1=x, BS0=x, A10=1.
	localparam L_CMD_AUTO_REFRESH	= 7'b0001000;  // Auto Refresh. CS=0, RAS=0, CAS=0, WE=1, BS1=x, BS0=x, A10=x.
	localparam L_CMD_NOP			= 7'b0111000;  // No Operation. CS=0, RAS=1, CAS=1, WE=1, BS1=x, BS0=x, A10=x.
	localparam L_CMD_MRS			= 7'b0000000;  // Mode Register Set (MRS). CS=0, RAS=0, CAS=0, WE=0, BS1=0, BS0=0, A10=0. See 10.4 Mode Register Set Cycle, datasheet, pag. 20.

	// Commands for read/write operations.
	localparam L_CMD_BANK_ACTIVATE	= 7'b0011000;  // Activate bank/row. CS=0, RAS=0, CAS=1, WE=1, BS1=x, BS0=x, A10=x. Bank address comes from the SOC address.
	localparam L_CMD_WRITE			= 7'b0100000;  // Write command. CS=0, RAS=1, CAS=0, WE=0, BS1=x, BS0=x, A10=0. Bank address comes from the SOC address.
	localparam L_CMD_READ			= 7'b0101000;  // Read command. CS=0, RAS=1, CAS=0, WE=1, BS1=x, BS0=x, A10=0. Bank address comes from the SOC address.
	//localparam L_CMD_PRECHARGE_BANK	= 7'b0010000;  // Precharge specific bank. CS=0, RAS=0, CAS=1, WE=0, BS1=x, BS0=x, A10=0. Bank address comes from the SOC address.


	//-------------------------------
	// Internal Registers and Wires
	//-------------------------------

	// Internal state machine registers.
	reg [4:0] current_state, next_state;
	reg [6:0] current_command, next_command;  // SDRAM command.

	// Counter for time delays.
	reg [14:0] delay_counter;
	reg reset_delay_counter;

	// Counts the 8 auto-refresh cycles needed for initialization.
	reg [3:0] init_counter;
	reg init_counter_count_one_cycle;

	// Counter to track the timing of the next refresh.
	reg [15:0] refresh_counter;
	reg reset_refresh_counter;

	// Internal registers for SDRAM address generation.
	reg [L_SDRAM_ADDR_WIDTH - 1: 0] ram_addr_reg;
	reg [L_BANK_ADDR_WIDTH - 1: 0] ram_bank_addr_reg;

	reg busy_signal;
	reg ready_signal;
	reg [31:0] rdata_reg;
	reg [31:0] wdata_reg;
	reg [3:0] sdram_side_wr_mask_reg;  // Byte mask signals.

	wire [22:0] soc_side_word_addr_bus;
	wire in_initialization_cycle;
	wire in_write_cycle;
	wire in_read_cycle;


	//-------------------------------
	// Assignments
	//-------------------------------

	/*
		Translate from Byte address to 32 bits word address.
		The PicoRV32 CPU uses Byte-oriented addresses but the RAM is organized in 32-bit word. 
		The controller have to select the correct 32-bit word within the RAM according to the 
		Byte address requested by the CPU.
		This is achieved by discarding the 2 least significant bits to obtain a 23-bit address 
		(for the case of a 8M 32bits words RAM) from the 25-bit address (32MB) issued by the CPU.
	*/
	assign soc_side_word_addr_bus = soc_side_byte_addr_bus[24:2];

	assign ram_side_cken = 1'b1;
	// Assigns current_command bits to outputs.
	assign {ram_side_cs_n, ram_side_ras_n, ram_side_cas_n, ram_side_wen} = current_command[6:3];
	assign ram_side_bank_addr = (in_write_cycle || in_read_cycle)? ram_bank_addr_reg : current_command[2:1];
	assign ram_side_addr_bus = (in_write_cycle || in_read_cycle || current_state == INIT_MRS)? ram_addr_reg : { {(L_SDRAM_ADDR_WIDTH - 11){1'b0}}, current_command[0], 10'd0 };

	assign soc_side_busy = busy_signal;
	assign soc_side_ready = ready_signal;

	// Read data: From ram_side_chip0_data_bus and ram_side_chip1_data_bus --> rdata_reg --> soc_side_rdata.
	assign soc_side_rdata = rdata_reg;

	// Write data: From soc_side_wdata --> wdata_reg --> ram_side_chip0_data_bus and ram_side_chip1_data_bus.
	assign ram_side_chip0_data_bus = (current_state == WRITE_CAS)? wdata_reg[15:0] : 16'bz;		// Lower 16 bits in chip 0.
	assign ram_side_chip1_data_bus = (current_state == WRITE_CAS)? wdata_reg[31:16] : 16'bz;		// Upper 16 bits in chip 1.

	assign ram_side_chip0_ldqm = sdram_side_wr_mask_reg[0];
	assign ram_side_chip0_udqm = sdram_side_wr_mask_reg[1];
	assign ram_side_chip1_ldqm = sdram_side_wr_mask_reg[2];
	assign ram_side_chip1_udqm = sdram_side_wr_mask_reg[3];

	// Signal to detect SDRAM initialization cycle states.
	assign in_initialization_cycle = (current_state == INIT) || (current_state == INIT_WAIT) || 
									(current_state == INIT_PRECHARGE_ALL) || (current_state == INIT_WAIT_TRP) || 
									(current_state == INIT_AUTO_REFRESH) || (current_state == INIT_WAIT_TRC) || 
									(current_state == INIT_MRS) || (current_state == INIT_WAIT_TRSC);

	// Signal to detect write cycle states.
	assign in_write_cycle = (current_state == WRITE_BANK_ACTIVATE) || (current_state == WRITE_WAIT_TRCD) || 
							(current_state == WRITE_CAS) || (current_state == WRITE_WAIT_TWR);
							/*(current_state == WRITE_PRECHARGE) || (current_state == WRITE_WAIT_TRP);*/

	// Signal to detect read cycle states.
	assign in_read_cycle = (current_state == READ_BANK_ACTIVATE) || (current_state == READ_WAIT_TRCD)  ||
						(current_state == READ_CAS) || (current_state == READ_WAIT_CAS_LATENCY)  ||
						(current_state == READ_DATA);
						/*(current_state == READ_PRECHARGE)  || (current_state == READ_WAIT_TRP);*/


	//-------------------------------------------------------
	// Sequential block for updating state machine registers.
	//-------------------------------------------------------
	always @(posedge clk) 
	begin
		if (~reset_n)  // Synchronous reset.
			begin
				current_state <= INIT;
				current_command <= L_CMD_NOP;
				
				// Data.
				wdata_reg <= 32'b0;
				rdata_reg <= 32'b0;
			end
		else 
			begin
				// Update state and current_command.
				// ----------------------------
				current_state <= next_state;
				current_command <= next_command;
				
				// Update write data register.
				// ----------------------------
				// Write data: From soc_side_wdata --> wdata_reg --> ram_side_chip0_data_bus and ram_side_chip1_data_bus.
				if (soc_side_wen)
					wdata_reg <= soc_side_wdata;  // Update the data to be written from the SOC.

				// Update read data register.
				// ----------------------------
				// Read data: From ram_side_chip0_data_bus and ram_side_chip1_data_bus --> rdata_reg --> soc_side_rdata.
				if (current_state == READ_DATA)
					rdata_reg <= {ram_side_chip1_data_bus, ram_side_chip0_data_bus};  // Updates the data that the SOC will read.
			end
	end

	//-------------------------
	// Counter for time delays.
	//-------------------------
	always @(posedge clk) 
	begin
		if (reset_delay_counter)
			delay_counter <= 0;
		else 
			delay_counter <= delay_counter + 1'b1;
	end

	//---------------------------------------------
	// Counter for the 8 refresh cycles at startup.
	//---------------------------------------------
	always @(posedge clk) 
	begin
		if (~reset_n)
			init_counter <= 0;
		else if (init_counter_count_one_cycle)
			init_counter <= init_counter + 1'b1;
	end

	//---------------------------
	// Counter for refresh cycle.
	//---------------------------
	always @(posedge clk) 
	begin
		if (reset_refresh_counter)
			refresh_counter <= 0;
		else 
			refresh_counter <= refresh_counter + 1'b1;
	end


	//------------------------------------------
	// Combinational block for next-state logic.
	//------------------------------------------
	/*
		Combinational logic of the state machine.

		Reglas para la implementación de los estados:

			// Valores por defecto para evitar latches en las salidas/señales.
			// De esta manera aunque las salidas se declaren como registros, el compilador las 
			// asignará inmediatamente sin tener que esperar al siguiente estado.
			output_a = 1'b1;
			output_b = 1'b1;
			output_c = 1'b0;
			output_d = 1'b0;

			// Lógica de la máquina de estados.
			case (current_state)

				ESTADO_1:
					begin
						// ---- Outputs ----
						// Comprobar entradas u otras señales y generar salidas del estado 1.

						output_a = 1'b0;

						if (input_a)
							begin
								output_b = 1'b1;
								output_c = 1'b0;
							end
						else
							begin
								output_b = 1'b0;
								output_c = 1'b1;
							end


						// ---- Transitions ----
						next_state = ESTADO_2;
						next_command = COMANDO_DURANTE_EL_ESTADO_2;

						// TODO: Actualizar notas sobre uso del contador de retardos.
						// Inicializar next_delay_counter si el siguiente estado necesita cumplir con una espera de tiempo de 'n' ciclos de reloj.
						next_delay_counter = n - 1;
					end

				ESTADO_2:
					begin
						// ---- Outputs ----
						// Comprobar entradas u otras señales y generar salidas del estado 2.

						output_a = 1'b1;

						if ( !input_a && (input_b || input_c) )
							begin
								output_b = 1'b1;
								output_c = 1'b1;
							end
						else
							begin
								output_b = 1'b0;
								output_c = 1'b0;
							end


						// ---- Transitions ----
						// Espera 'n' ciclos de reloj y luego transiciona al estado 3.
						if (delay_counter != 0)
							next_delay_counter = delay_counter - 1'b1;
						else 
							begin
								next_state = ESTADO_3;
								next_command = COMANDO_DURANTE_EL_ESTADO_3;

								// Inicializar next_delay_counter si el estado 3 necesita cumplir con una espera de tiempo de 'm' ciclos de reloj.
								next_delay_counter = m - 1;
							end
					end

					.
					.
					.

		Con esta estructura, cuando la máquina se encuentra en ESTADO_ACTUAL, ya está emitiendo el comando apropiado 
		para ese estado (asignado en el estado anterior).
	*/
	always @(*) 
	begin
		// Default values to prevent unwanted latches.
		next_state = current_state;
		next_command = L_CMD_NOP;

		reset_delay_counter = 1'b0;
		init_counter_count_one_cycle = 1'b0;
		reset_refresh_counter = 1'b0;

		/*
			In this case, even though the signals are declared as registers, the compiler will assign them
			immediately in each state without having to wait for the next state.
			That is, the signals will not be delayed by one clock cycle.
		*/
		busy_signal = 1'b1;  // Controller busy by default, it is only free when the state machine is in IDLE.
		ready_signal = 1'b0;
		
		case (current_state)

			// ---- Initialization Sequence ----
			/*
				Initialization Sequence
				-----------------------

				The implemented sequence follows the requirements of the W9812G6KH-5I datasheet.

				1. INIT: Enable delay counter for the next state.

				2. INIT_WAIT: Wait at least 200us after power-up.
					- During this pause, DQM and CKE are kept high to prevent data contention.

				3. INIT_PRECHARGE_ALL: Issues the precharge all banks current_command.
					- This current_command prepares all banks for subsequent operations.

				4. INIT_WAIT_TRP: Wait tRP nanoseconds after precharge.
					- Time required for the precharge to complete internally.

				5. INIT_AUTO_REFRESH + INIT_WAIT_TRC: Executes 8 Auto Refresh cycles.
					- Each cycle issues an AUTO_REFRESH current_command and waits tRC nanoseconds.
					- All 8 cycles are specifically required by the datasheet for initialization.

				6. INIT_MRS: Mode Register configuration.
					- Configures: Burst Length=1, Sequential type, CAS Latency=3, Standard operation.

				7. INIT_WAIT_TRSC: Wait tRSC before transitioning to IDLE.
					- Time required for the register configuration to complete.
			*/
			INIT:
				begin
					// ---- Outputs ----
					reset_delay_counter = 1'b1;
					next_command = L_CMD_NOP;  // Emit L_CMD_NOP command during timeouts.

					// ---- Transitions ----
					next_state = INIT_WAIT;
				end
			
			INIT_WAIT: 
				begin
					// ---- Outputs ----

					// ---- Transitions ----
					if (delay_counter == L_INIT_PAUSE_CYCLES - 1)  // Wait at least 200us before starting initialization.
						begin
							next_command = L_CMD_PRECHARGE_ALL;
							
							next_state = INIT_PRECHARGE_ALL;
						end
				end
			
			INIT_PRECHARGE_ALL: 
				begin
					// ---- Outputs ----
					// Precharge all banks.
					// L_CMD_PRECHARGE_ALL command issued in the previous state.

					reset_delay_counter = 1'b1;		// Enable the delay counter to start counting in the next state.
					next_command = L_CMD_NOP;		// Issue L_CMD_NOP command during wait times.

					// ---- Transitions ----
					next_state = INIT_WAIT_TRP;
				end
			
			INIT_WAIT_TRP: 
				begin
					// ---- Outputs ----

					// ---- Transitions ----
					if (delay_counter == L_TRP_CYCLES - 1)	// Wait tRP nanoseconds after issuing the precharge all command.
						begin
							next_command = L_CMD_AUTO_REFRESH;

							next_state = INIT_AUTO_REFRESH;
						end
				end
			
			INIT_AUTO_REFRESH: 
				begin
					// ---- Outputs ----
					/*
						Auto Refresh.

						L_CMD_AUTO_REFRESH command issued in the previous state.
						The L_CMD_AUTO_REFRESH command is active for a single clock cycle, immediately after 
						the SDRAM waits tRC with a NOP command.
					*/

					reset_delay_counter = 1'b1;
					next_command = L_CMD_NOP;  // Issue L_CMD_NOP command during wait times.

					// ---- Transitions ----
					next_state = INIT_WAIT_TRC;  // Wait refresh cycle time tRC after sending the auto refresh command.
				end
			
			INIT_WAIT_TRC: 
				begin
					// ---- Outputs ----

					// ---- Transitions ----
					if (delay_counter == L_TRC_CYCLES - 1)  // Wait tRC.
						begin
							// After waiting tRC, check if the 8 refresh cycles have been completed.
							if (init_counter == L_INIT_REFRESH_COUNT - 1) 
								begin
									// Once the 8 cycles are completed, set the mode register.
									next_command = L_CMD_MRS;

									next_state = INIT_MRS;
								end
							else 
								begin
									// Still not completed the 8 Auto Refresh cycles.
									init_counter_count_one_cycle = 1'b1;  // Count one completed refresh cycle.
									next_command = L_CMD_AUTO_REFRESH;

									next_state = INIT_AUTO_REFRESH;
								end
						end
				end
			
			INIT_MRS: 
				begin
					// ---- Outputs ----
					// Mode Register Set.
					// L_CMD_MRS command issued in the previous state.

					reset_delay_counter = 1'b1;
					next_command = L_CMD_NOP;  // Issue L_CMD_NOP command during wait times.

					// ---- Transitions ----
					next_state = INIT_WAIT_TRSC;
				end
			
			INIT_WAIT_TRSC: 
				begin
					// ---- Outputs ----

					// ---- Transitions ----
					if (delay_counter == L_TRSC_CYCLES - 1)	// Wait tRSC (Mode Register Set Cycle Time) after the Mode Register Set
						begin
							reset_refresh_counter = 1'b1;	// Starts periodic refresh cycles.
							next_command = L_CMD_NOP;		// The No Operation Command should be used in cases when the SDRAM is in an idle or a wait state.

							next_state = IDLE;				// Initialization completed.
						end
				end
			
			
			// ---- IDLE State ----
			IDLE:
				begin
					// ---- Outputs ----
					busy_signal = 1'b0;
					ready_signal = 1'b1;

					// ---- Transitions ----
					if (refresh_counter == L_CYCLES_BETWEEN_REFRESH - 1)
						begin
							/*
								Starts auto-refresh sequence.
								The auto-refresh counter always increments and resets at the end
								of the auto-refresh sequence, in the REFRESH_WAIT_TRC state.
							*/
							next_command = L_CMD_PRECHARGE_ALL;

							next_state = REFRESH_PRECHARGE_ALL;
						end
					else if (soc_side_wen)
						begin
							// Starts write sequence.
							next_command = L_CMD_BANK_ACTIVATE;

							next_state = WRITE_BANK_ACTIVATE;
						end
					else if (soc_side_ren)
						begin
							// Starts read sequence.
							next_command = L_CMD_BANK_ACTIVATE;

							next_state = READ_BANK_ACTIVATE;
						end
				end


			// ---- Auto Refresh Sequence ----
			/*
				Auto Refresh Sequence
				---------------------

				The automatic refresh cycle is activated periodically to maintain data in SDRAM:

				1. REFRESH_PRECHARGE_ALL: Precharges all banks before refresh.
					- Necessary because the AUTO_REFRESH current_command requires all banks to be inactive.

				2. REFRESH_WAIT_TRP: Wait tRP nanoseconds after precharge.
					- Time required for the precharge to complete internally.

				3. REFRESH_AUTO_REFRESH: Issues the AUTO_REFRESH current_command.
					- Refreshes all rows in all banks simultaneously.

				4. REFRESH_WAIT_TRC: Wait tRC nanoseconds after AUTO_REFRESH.
					- Time required for the refresh cycle to complete internally.
					- Upon completion, the refresh counter is reset to start a new period.

				The refresh frequency is calculated to ensure all 4096 rows are refreshed 
				within the 64ms retention period specified by the datasheet.
			*/
			REFRESH_PRECHARGE_ALL: 
				begin
					// ---- Outputs ----
					// Precharge all banks.
					// L_CMD_PRECHARGE_ALL command issued in the previous IDLE state.

					reset_delay_counter = 1'b1;
					next_command = L_CMD_NOP;  // Issue L_CMD_NOP command during wait times.

					// ---- Transitions ----
					next_state = REFRESH_WAIT_TRP;
				end

			REFRESH_WAIT_TRP: 
				begin
					// ---- Outputs ----

					// ---- Transitions ----
					if (delay_counter == L_TRP_CYCLES - 1)  // Wait tRP nanoseconds after issuing the precharge all command.
						begin
							next_command = L_CMD_AUTO_REFRESH;

							next_state = REFRESH_AUTO_REFRESH;
						end
				end

			REFRESH_AUTO_REFRESH:
				begin
					// ---- Outputs ----
					/*
						Auto Refresh.

						L_CMD_AUTO_REFRESH command issued in the previous state.
						The L_CMD_AUTO_REFRESH command is active for a single clock cycle, immediately after which 
						the SDRAM waits tRC with the NOP command.
					*/

					reset_delay_counter = 1'b1;
					next_command = L_CMD_NOP;  // Issue L_CMD_NOP command during wait times.
					
					// ---- Transitions ----
					next_state = REFRESH_WAIT_TRC;
				end

			REFRESH_WAIT_TRC:
				begin
					// ---- Outputs ----

					// ---- Transitions ----
					if (delay_counter == L_TRC_CYCLES - 1)  // Wait tRC nanoseconds after issuing the L_CMD_AUTO_REFRESH command.
						begin
							
							reset_refresh_counter = 1'b1;	// Reset the refresh counter.
							next_command = L_CMD_NOP;		// The No Operation Command should be used in cases when the SDRAM is in an idle or a wait state.

							next_state = IDLE;  // Return to IDLE state.
						end
				end


			// ---- Write Sequence ----
			/*
				Write Sequence
				--------------

				The write cycle follows the required sequence to write data to SDRAM:

				1. WRITE_BANK_ACTIVATE: Activates the specific bank and row.
					- Selects the memory row containing the address to write.

				2. WRITE_WAIT_TRCD: Wait tRCD nanoseconds after activation.
					- Time required between activation and column current_command (Row to Column Delay).

				3. WRITE_CAS: Issues the write current_command and sends data.
					- Column address and data are presented simultaneously.
					- A10 is configured for auto-precharge if enabled.

				4. WRITE_WAIT_TWR: Wait tWR nanoseconds after write.
					- Time required for data to be fully written to the memory array.

				5. WRITE_PRECHARGE/WRITE_WAIT_TRP (optional if auto-precharge not used):
					- Precharges the specific bank and waits tRP before new operations.
			*/
			WRITE_BANK_ACTIVATE:
				begin
					// ---- Outputs ----
					// Activate the specified bank and row.
					// L_CMD_BANK_ACTIVATE command issued in the previous state 'IDLE'.

					// See:
					//----------------------------------------------------------
					// Combinational logic block for generating bank address and
					// word address for the SDRAM based on the current state.
					//----------------------------------------------------------

					reset_delay_counter = 1'b1;
					next_command = L_CMD_NOP;  // Issue L_CMD_NOP command during wait times.

					// ---- Transitions ----
					next_state = WRITE_WAIT_TRCD;
				end

			WRITE_WAIT_TRCD:
				begin
					// ---- Outputs ----

					// ---- Transitions ----
					if (delay_counter == L_TRCD_CYCLES - 1)  // Wait for tRCD.
						begin
							next_command = L_CMD_WRITE;

							next_state = WRITE_CAS;
						end
				end

			WRITE_CAS:
				begin
					// ---- Outputs ----
					/*
						WRITE_CAS command emitted in the previous state.
						The data to be written is already in the wdata_reg register; it is captured
						in wdata_reg in the sequential block with the rising edge of clk.
					*/

					// See:
					//----------------------------------------------------------
					// Combinational logic block for generating bank address and
					// word address for the SDRAM based on the current state.
					//----------------------------------------------------------

					reset_delay_counter = 1'b1;
					next_command = L_CMD_NOP;  // Issue L_CMD_NOP command during wait times.

					// ---- Transitions ----
					next_state = WRITE_WAIT_TWR;
				end

			WRITE_WAIT_TWR:
				begin
					// ---- Outputs ----
					// After waiting tWR nanoseconds the data has already been written to the SDRAM memory.

					// ---- Transitions ----
					if (delay_counter == L_TWR_CYCLES - 1)  // Wait tWR cycles.
						begin
							next_command = L_CMD_NOP;  // The No Operation Command should be used in cases when the SDRAM is in a idle or a wait state.

							next_state = IDLE;  // Return directly to IDLE.
						end
				end

			// WRITE_WAIT_TWR: 
			// 	begin
			// 		// ---- Outputs ----
			// 		// After waiting tWR nanoseconds the data has already been written to the SDRAM memory.

			// 		// ---- Transitions ----
			// 		if (delay_counter == L_TWR_CYCLES - 1)  // Wait tWR nanoseconds.
			// 			begin
			// 				if (ram_addr_reg[10]) 
			// 					begin
			// 						/*
			// 							Si A10=1 durante el comando WRITE, auto-precharge está activo
			// 							y no necesitamos enviar un comando de precharge explícito.
			// 							Actualmente A10=1 siempre pero se mantienen los siguientes dos
			// 							estados WRITE_PRECHARGE y WRITE_WAIT_TRP para futuras mejoras 
			// 							del controlador.
			// 						*/

			// 						next_command = L_CMD_NOP;  // The No Operation Command should be used in cases when the SDRAM is in a idle or a wait state.

			//                      next_state = IDLE;  // Volver a IDLE directamente.
			// 					end
			// 				else 
			// 					begin
			// 						next_command = L_CMD_PRECHARGE_BANK;

			//                      next_state = WRITE_PRECHARGE;
			// 					end
			// 			end
			// 	end

			// WRITE_PRECHARGE: 
			// 	begin
			// 		// ---- Outputs ----
			// 		// Precargar el banco específico.
			// 		// Comando L_CMD_PRECHARGE_BANK emitido en el estado anterior.

			// 		reset_delay_counter = 1'b1;
			//         next_command = L_CMD_NOP;  // Emitir comando L_CMD_NOP durante los tiempos de espera.

			// 		// ---- Transitions ----
			// 		next_state = WRITE_WAIT_TRP;
			// 	end

			// WRITE_WAIT_TRP: 
			// 	begin
			// 		// ---- Outputs ----

			// 		// ---- Transitions ----
			// 		if (delay_counter == L_TRP_CYCLES - 1)  // Esperar tRP.
			// 			begin
			// 				next_command = L_CMD_NOP;		// The No Operation Command should be used in cases when the SDRAM is in a idle or a wait state.

			//                 next_state = IDLE;			// Volver a IDLE.
			// 			end
			// 	end


			// ---- Read Sequence ----
			/*
				Read Sequence
				-------------

				The read cycle follows the required sequence to read data from SDRAM:

				1. READ_BANK_ACTIVATE: Activates the specific bank and row.
					- Selects the memory row containing the address to read.
					- Bank address is provided via BS0, BS1.
					- Row address is provided via A0-A11.

				2. READ_WAIT_TRCD: Wait tRCD nanoseconds after activation.
					- Time required between row activation and column current_command (Row to Column Delay).
					- tRCD minimum is 15ns for -5/-5I/-5J grade parts.

				3. READ_CAS: Issues the read current_command.
					- Column address is presented and CS, CAS are set low, RAS, WE are set high.
					- A10 is configured for auto-precharge if enabled.

				4. READ_WAIT_CAS_LATENCY: Wait for CAS Latency cycles.
					- CAS Latency is 3 clock cycles as configured in Mode Register.
					- This delay is necessary for the SDRAM to retrieve data from the memory array.

				5. READ_DATA: Capture the data appearing on DQ pins.
					- Data becomes available exactly after CAS Latency.
					- The data is held valid for tOH (data output hold time) after the next rising clock edge.

				6. READ_PRECHARGE/READ_WAIT_TRP (optional if auto-precharge not used):
					- Precharges the specific bank and waits tRP before new operations.
					- tRP minimum is 15ns for -5/-5I/-5J grade parts.
					- Auto-precharge eliminates the need for an explicit precharge current_command.
			*/
			READ_BANK_ACTIVATE:
				begin
					// ---- Outputs ----
					// Activate the required bank and row.
					// current_command = L_CMD_BANK_ACTIVATE, issued in the previous IDLE state.

					// See:
					//----------------------------------------------------------
					// Combinational logic block for generating bank address and
					// word address for the SDRAM based on the current state.
					//----------------------------------------------------------

					reset_delay_counter = 1'b1;
					next_command = L_CMD_NOP;	// Issue L_CMD_NOP command during wait times.

					// ---- Transitions ----
					next_state = READ_WAIT_TRCD;
				end

			READ_WAIT_TRCD:
				begin
					// ---- Outputs ----
					// current_command = L_CMD_NOP, issued in the previous state.

					// ---- Transitions ----
					if (delay_counter == L_TRCD_CYCLES - 1)	// Wait tRCD after activating the bank.
						begin
							next_command = L_CMD_READ;

							next_state = READ_CAS;
						end
				end

			READ_CAS:
				begin
					// ---- Outputs ----
					// Issue read command.
					// current_command = L_CMD_READ, issued in the previous state.

					// See:
					//----------------------------------------------------------
					// Combinational logic block for generating bank address and
					// word address for the SDRAM based on the current state.
					//----------------------------------------------------------

					reset_delay_counter = 1'b1;
					next_command = L_CMD_NOP;	// Issue L_CMD_NOP command during wait times.

					// ---- Transitions ----
					next_state = READ_WAIT_CAS_LATENCY;
				end

			READ_WAIT_CAS_LATENCY:
				begin
					// ---- Outputs ----

					// ---- Transitions ----
					if (delay_counter == L_CAS_LATENCY - 1)	// Wait L_CAS_LATENCY cycles before data is available.
						begin
							next_command = L_CMD_NOP;

							next_state = READ_DATA;
						end
				end

			READ_DATA:
				begin
					// ---- Outputs ----
					/*
						After waiting L_CAS_LATENCY cycles, the data is captured in the sequential block
						in the rdata_reg register on the rising edge of clk.
					*/

					// ---- Transitions ----
					next_command = L_CMD_NOP;	// The No Operation Command should be used in cases when the SDRAM is in a idle or a wait state.

					next_state = IDLE;			// Return to IDLE.
				end

			// READ_DATA:
			// 	begin
			// 		// ---- Outputs ----
			// 		/*
			// 			Luego de esperar L_CAS_LATENCY ciclos los datos se capturan en el bloque secuencial 
			// 			en el registro rdata_reg con el flanco de subida de clk.
			// 		*/

			// 		// ---- Transitions ----
			// 		if (ram_addr_reg[10])  // Si A10=1 durante el comando READ, auto-precharge está activo.
			// 			begin
			// 				/*
			// 					Si A10=1 durante el comando READ, auto-precharge está activo
			// 					y no necesitamos enviar un comando de precharge explícito.
			// 					Actualmente A10=1 siempre pero se mantienen los siguientes dos
			// 					estados READ_PRECHARGE y READ_WAIT_TRP para futuras mejoras del 
			// 					controlador.
			// 				*/

			// 				next_command = L_CMD_NOP;	// The No Operation Command should be used in cases when the SDRAM is in a idle or a wait state.

			//              next_state = IDLE;			// Volver a IDLE directamente.
			// 			end
			// 		else
			// 			begin
			// 				next_command = L_CMD_PRECHARGE_BANK;

			//                 next_state = READ_PRECHARGE;
			// 			end
			// 	end

			// READ_PRECHARGE:
			// 	begin
			// 		// ---- Outputs ----
			// 		// Precargar el banco específico.
			// 		// Comando CMD_PRECHARGE_BANK emitido en el estado anterior.

			// 		reset_delay_counter = 1'b1;
			//         next_command = L_CMD_NOP;  // Emitir comando L_CMD_NOP durante los tiempos de espera.

			// 		// ---- Transitions ----
			// 		next_state = READ_WAIT_TRP;
			// 	end

			// READ_WAIT_TRP:
			// 	begin
			// 		// ---- Outputs ----

			// 		// ---- Transitions ----
			// 		if (delay_counter == L_TRP_CYCLES - 1)  // Esperar tRP después de precargar.
			// 			begin
			// 				next_command = L_CMD_NOP;		// The No Operation Command should be used in cases when the SDRAM is in a idle or a wait state.

			//                 next_state = IDLE;			// Volver a IDLE.
			// 			end
			// 	end

			
			default: 
				begin
					// ---- Outputs ----
					next_command = L_CMD_NOP;  // The No Operation Command should be used in cases when the SDRAM is in a idle or a wait state.

					// ---- Transitions ----
					next_state = IDLE;
				end

		endcase
	end

	//--------------------------------------------------------
	// Combinational logic block to set the writing mask based 
	// on the current state.
	//--------------------------------------------------------
	always @(*) 
	begin
		/*
			PicoRV32 SOC, mem_wstrb signals:
				soc_side_wmask = 0000 --> No write. Read operation.
				soc_side_wmask = 1111 --> Write 32 bits.
				soc_side_wmask = 1100 --> Write upper 16 bits.
				soc_side_wmask = 0011 --> Write lower 16 bits.
				soc_side_wmask = 0001 --> Write Byte 0.
				soc_side_wmask = 0010 --> Write Byte 1.
				soc_side_wmask = 0100 --> Write Byte 2.
				soc_side_wmask = 1000 --> Write Byte 3.
		*/
		if (in_initialization_cycle)
			// Disable buffers during initialization. Mask all bits high so that the SDRAM drives the input/output buffers to HIGH-Z.
			sdram_side_wr_mask_reg = 4'b1111;
		else if (in_write_cycle)
			/*
				PicoRV32 SOC, mem_wstrb signals:
					soc_side_wmask = 0000 --> No write. Read memory.
					soc_side_wmask = 1111 --> Write 32 bits.
					soc_side_wmask = 1100 --> Write upper 16 bits.
					soc_side_wmask = 0011 --> Write lower 16 bits.
					soc_side_wmask = 0001 --> Write Byte 0.
					soc_side_wmask = 0010 --> Write Byte 1.
					soc_side_wmask = 0100 --> Write Byte 2.
					soc_side_wmask = 1000 --> Write Byte 3.

				The mem_wstrb signals of the SoC are inverted because the LDQM and UDQM pins of the 
				SDRAM set the input/output buffers to HIGH-Z with 1 and enable them with 0.
			*/
			sdram_side_wr_mask_reg = ~soc_side_wmask;  // Use the 4-bit write mask from the soc (mem_wstrb signals).
		else if (in_read_cycle)
			// During read, enable all buffers so data can be read.
			sdram_side_wr_mask_reg = 4'b0000;  // DQM = 0 enables data output during read.
		else
			// If the state is not read or write, mask all bits high so that the SDRAM drives the input/output buffers to HIGH-Z.
			sdram_side_wr_mask_reg = 4'b1111;
	end

	//----------------------------------------------------------
	// Combinational logic block for generating bank address and 
	// word address for the SDRAM based on the current state.
	//----------------------------------------------------------
	always @(*) 
	begin
		ram_bank_addr_reg = 2'b00;
		ram_addr_reg = {L_SDRAM_ADDR_WIDTH{1'b0}};
		
		if (current_state == READ_BANK_ACTIVATE || current_state == WRITE_BANK_ACTIVATE)
			begin
				ram_bank_addr_reg = soc_side_word_addr_bus[L_RAM_ADDR_BUS_WIDTH - 1: L_RAM_ADDR_BUS_WIDTH - L_BANK_ADDR_WIDTH];
				ram_addr_reg = soc_side_word_addr_bus[L_RAM_ADDR_BUS_WIDTH - (L_BANK_ADDR_WIDTH + 1): L_RAM_ADDR_BUS_WIDTH - (L_BANK_ADDR_WIDTH + L_ROW_WIDTH)];
			end
		else if (current_state == READ_CAS || current_state == WRITE_CAS)
			begin
				ram_bank_addr_reg = soc_side_word_addr_bus[L_RAM_ADDR_BUS_WIDTH - 1: L_RAM_ADDR_BUS_WIDTH - L_BANK_ADDR_WIDTH];

				/*
											BANK	 ROW	 COL
					L_RAM_ADDR_BUS_WIDTH	 2	  +  12   +   9   = 23 
					L_SDRAM_ADDR_WIDTH 13
				*/
				ram_addr_reg = { {(L_SDRAM_ADDR_WIDTH - 11){1'b0}},			/* 0s */
								1'b1,										/* 1 (A10 is always for auto precharge) */
								{(10 - L_COL_WIDTH){1'b0}},					/* 0s */
								soc_side_word_addr_bus[L_COL_WIDTH - 1: 0]	/* column address */
							};
			end
		else if (current_state == INIT_MRS)
			begin
				/*
					See datasheet, pag. 20.

					Mode Register Set (MRS) configuration during initialization:

						Burst Length of 1 (no burst)	-> bits 0-2 = 000
						Sequential burst type			-> bit 3    = 0
						CAS Latency of 3				-> bits 4-6 = 011
						Standard operation mode			-> bits 7-8 = 00
						Burst Read and Single Write		-> bit 9    = 1

						= 10'b 1 00 011 0 000
				*/
				ram_addr_reg = { {(L_SDRAM_ADDR_WIDTH - 10){1'b0}}, 10'b1000110000 };
			end
	end

endmodule

