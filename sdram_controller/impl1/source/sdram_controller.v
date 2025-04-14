/*
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
 */


module sdram_controller # (
	/* Timing parameters */
	parameter CLK_FREQUENCY_MHZ = 80,	// Clock frequency [MHz].
	parameter REFRESH_TIME_MS = 64,		// Refresh period [ms].
	parameter REFRESH_COUNT = 4096,		// Number of refresh cycles per refresh period.

	/* Address parameters */
	parameter ROW_WIDTH = 12,
	parameter COL_WIDTH = 9,
	parameter BANK_ADDR_WIDTH = 2,		// 2 bits wide for 4 banks.
	
	parameter SOC_SIDE_ADDR_WIDTH = ROW_WIDTH + COL_WIDTH + BANK_ADDR_WIDTH,  // 23 bits bus to address 8 million 32 bit words = 32MB RAM.
	parameter SDRAM_ADDR_WIDTH = ROW_WIDTH
) (
	input wire clk,
	input wire reset_n_port,

	/* SOC interface */
	output wire soc_side_busy_port,
	output wire soc_side_ready_port,

	// Address.
	input wire [SOC_SIDE_ADDR_WIDTH - 1: 0] soc_side_addr_port,

	// Read data.
	output wire [31:0] soc_side_rd_data_port,
	input wire soc_side_rd_en_port,

	// Write data.
	input wire [31:0] soc_side_wr_data_port,
	input wire [3:0] soc_side_wr_mask_port,  // These are the mem_wstrb signals in the PicoRV32 SOC.
	input wire soc_side_wr_en_port,

	/* SDRAM interface */
	output wire [SDRAM_ADDR_WIDTH - 1: 0] ram_side_addr_port,		// SDRAM chips 0 and 1, A0 to A11 pins.
	output wire [BANK_ADDR_WIDTH - 1: 0] ram_side_bank_addr_port,	// SDRAM chips 0 and 1, BS0 and BS1 pins.

	output wire ram_side_chip0_ldqm_port,		// SDRAM chip 0, LDQM pin.
	output wire ram_side_chip0_udqm_port,		// SDRAM chip 0, UDQM pin.
	inout wire [15:0] ram_side_chip0_data_port,	// SDRAM chip 0, DQ0 to DQ15 pins.

	output wire ram_side_chip1_ldqm_port,		// SDRAM chip 1, LDQM pin.
	output wire ram_side_chip1_udqm_port,		// SDRAM chip 1, UDQM pin.
	inout wire [15:0] ram_side_chip1_data_port,	// SDRAM chip 1, DQ0 to DQ15 pins.

	output wire ram_side_cs_n_port,		// SDRAM chips 0 and 1, CS pin.
	output wire ram_side_ras_n_port,	// SDRAM chips 0 and 1, RAS pin.
	output wire ram_side_cas_n_port,	// SDRAM chips 0 and 1, CAS pin.
	output wire ram_side_wr_en_port,	// SDRAM chips 0 and 1, WE pin.

	/*
		During normal access mode, CKE must be held high, enabling the clock.
		This pin is only used in Power Down mode, Suspend mode, or Self 
		Refresh mode, this controller does not implement these modes, so CKE 
		is always held high.
	*/
	output wire ram_side_ck_en_port		// SDRAM chips 0 and 1, CKE pin.
);


//-------------------------------
// Parameters
//-------------------------------

localparam CYCLES_BETWEEN_REFRESH = (CLK_FREQUENCY_MHZ * 1_000 * REFRESH_TIME_MS) / REFRESH_COUNT;

// ------------------------------------------------------------------------
// Times: Table "9.5 AC Characteristics and Operating Condition", page 15.
// ------------------------------------------------------------------------

// Timings for SDRAM initialization.
// -------------------------------------------
localparam INIT_PAUSE_US = 300;  // Initial pause of 200us + safety margin.
localparam INIT_PAUSE_CYCLES = INIT_PAUSE_US * CLK_FREQUENCY_MHZ;

localparam TRP_NS = 18;  // Precharge to Active Command Period, tRP: 15ns + safety margin. Wait time after issuing the precharge all banks command.
localparam TRP_CYCLES = ((TRP_NS * CLK_FREQUENCY_MHZ) + 999) / 1000;  // Number of wait cycles to meet tRP time. Rounded up.

localparam INIT_REFRESH_COUNT = 8;  // Number of consecutive refresh cycles required during initialization.

localparam TRC_NS = 65;  // Ref/Active to Ref/Active Command Period, tRC: 55-65ns. Wait time after issuing the auto refresh command.
localparam TRC_CYCLES = ((TRC_NS * CLK_FREQUENCY_MHZ) + 999) / 1000;  // Number of wait cycles to meet tRC time. Rounded up.

localparam TRSC_CYCLES = 2;  // Mode Register Set Cycle Time, tRSC: 2 clock cycles. Wait time after issuing the CMD_MRS Mode Register Set (MRS) command.

// Write and read timings.
// -------------------------------
localparam TRCD_NS = 18;  // Active to Read/Write Command Delay Time, tRCD: 15ns + safety margin. Wait time after issuing the CMD_BANK_ACTIVATE command.
localparam TRCD_CYCLES = ((TRCD_NS * CLK_FREQUENCY_MHZ) + 999) / 1000;  // Number of wait cycles to meet tRCD time. Rounded up.

localparam TWR_CYCLES = 2;  // Write Recovery Time, tWR: 2 clock cycles. Wait time after issuing the CMD_WRITE command.

localparam CAS_LATENCY = 3;
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
localparam INIT_PAUSE				= 5'b00001;  // Initial pause of at least 200us.
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

localparam CMD_PRECHARGE_ALL	= 7'b0010001;  // Precharge All.
localparam CMD_AUTO_REFRESH		= 7'b0001000;  // Auto Refresh.
localparam CMD_NOP				= 7'b0111000;  // No Operation.
localparam CMD_MRS				= 7'b000000x;  // Mode Register Set (MRS).

// Commands for read/write operations.
localparam CMD_BANK_ACTIVATE	= 7'b0011xxx;  // Activate bank/row (x = bank address bits).
localparam CMD_WRITE			= 7'b0100xx1;  // Write command (x = bank address bits).
localparam CMD_PRECHARGE_BANK	= 7'b0010x00;  // Precharge specific bank (x = a10 for auto-precharge).
localparam CMD_READ				= 7'b0101xx1;  // CS=L, RAS=H, CAS=L, WE=H


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
reg [SDRAM_ADDR_WIDTH - 1: 0] ram_addr_reg;
reg [BANK_ADDR_WIDTH - 1: 0] ram_bank_addr_reg;

reg busy_signal;
reg ready_signal;
reg [31:0] rd_data_reg;
reg [31:0] wr_data_reg;
reg [3:0] sdram_side_wr_mask_reg;  // Byte mask signals.

wire in_initialization_cycle;
wire in_write_cycle;
wire in_read_cycle;


//-------------------------------
// Assignments
//-------------------------------

// Assigns current_command bits to outputs.
assign ram_side_ck_en_port = 1'b1;
assign {ram_side_cs_n_port, ram_side_ras_n_port, ram_side_cas_n_port, ram_side_wr_en_port} = current_command[6:3];
assign ram_side_bank_addr_port = (in_write_cycle || in_read_cycle)? ram_bank_addr_reg : current_command[2:1];
assign ram_side_addr_port = (in_write_cycle || in_read_cycle || current_state == INIT_MRS)? ram_addr_reg : { {(SDRAM_ADDR_WIDTH - 11){1'b0}}, current_command[0], 10'd0 };

assign soc_side_busy_port = busy_signal;
assign soc_side_ready_port = ready_signal;

// Read data: From ram_side_chip0_data_port and ram_side_chip1_data_port --> rd_data_reg --> soc_side_rd_data_port.
assign soc_side_rd_data_port = rd_data_reg;

// Write data: From soc_side_wr_data_port --> wr_data_reg --> ram_side_chip0_data_port and ram_side_chip1_data_port.
assign ram_side_chip0_data_port = (current_state == WRITE_CAS)? wr_data_reg[15:0] : 16'bz;		// Lower 16 bits in chip 0.
assign ram_side_chip1_data_port = (current_state == WRITE_CAS)? wr_data_reg[31:16] : 16'bz;		// Upper 16 bits in chip 1.

assign ram_side_chip0_ldqm_port = sdram_side_wr_mask_reg[0];
assign ram_side_chip0_udqm_port = sdram_side_wr_mask_reg[1];
assign ram_side_chip1_ldqm_port = sdram_side_wr_mask_reg[2];
assign ram_side_chip1_udqm_port = sdram_side_wr_mask_reg[3];

// Signal to detect SDRAM initialization cycle states.
assign in_initialization_cycle = (current_state == INIT) || (current_state == INIT_PAUSE) || 
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
	if (~reset_n_port)  // Reset síncrono.
		begin
			current_state <= INIT;
			current_command <= CMD_NOP;
			
			// Data.
			wr_data_reg <= 32'b0;
			rd_data_reg <= 32'b0;
		end
	else 
		begin
			// Update state and current_command.
			// ----------------------------
			current_state <= next_state;
			current_command <= next_command;
			
			// Update write data register.
			// ----------------------------
			// Write data: From soc_side_wr_data_port --> wr_data_reg --> ram_side_chip0_data_port and ram_side_chip1_data_port.
			if (soc_side_wr_en_port)
				wr_data_reg <= soc_side_wr_data_port;  // Update the data to be written from the SOC.

			// Update read data register.
			// ----------------------------
			// Read data: From ram_side_chip0_data_port --> rd_data_reg --> soc_side_rd_data_port.
			if (current_state == READ_DATA)
				rd_data_reg <= {ram_side_chip1_data_port, ram_side_chip0_data_port};  // Updates the data that the SOC will read.
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
	if (~reset_n_port)
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
    next_command = CMD_NOP;

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

			2. INIT_PAUSE: Wait at least 200us after power-up.
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
                next_command = CMD_NOP;  // Emitir comando CMD_NOP durante los tiempos de espera.

				// ---- Transitions ----
				next_state = INIT_PAUSE;
			end

		INIT_PAUSE: 
			begin
				// ---- Outputs ----

				// ---- Transitions ----
				if (delay_counter == INIT_PAUSE_CYCLES - 1)  // Esperar al menos 200us antes de comenzar la inicialización.
					begin
                        next_command = CMD_PRECHARGE_ALL;
                        
						next_state = INIT_PRECHARGE_ALL;
					end
			end
		
		INIT_PRECHARGE_ALL: 
			begin
				// ---- Outputs ----
				// Precarga de todos los bancos.
				// Comando CMD_PRECHARGE_ALL emitido en el estado anterior.

				reset_delay_counter = 1'b1;  // Habilitar el contador de retardos para iniciar la cuenta en el siguiente estado.
                next_command = CMD_NOP;  // Emitir comando CMD_NOP durante los tiempos de espera.

				// ---- Transitions ----
				next_state = INIT_WAIT_TRP;
			end
		
		INIT_WAIT_TRP: 
			begin
				// ---- Outputs ----

				// ---- Transitions ----
				if (delay_counter == TRP_CYCLES - 1)  // Esperar tRP nanosegundos después de emitir el comando precharge all.
					begin
                        next_command = CMD_AUTO_REFRESH;

						next_state = INIT_AUTO_REFRESH;
					end
			end
		
		INIT_AUTO_REFRESH: 
			begin
				// ---- Outputs ----
				/*
					Auto Refresh.

					Comando CMD_AUTO_REFRESH emitido en el estado anterior.
					El comando CMD_AUTO_REFRESH se activa durante un solo ciclo de reloj, inmediatamente después 
					la SDRAM espera tRC con comando NOP.
				*/

                reset_delay_counter = 1'b1;
				next_command = CMD_NOP;			// Emitir comando CMD_NOP durante los tiempos de espera.

				// ---- Transitions ----
				next_state = INIT_WAIT_TRC;		// Esperar tiempo de ciclo de refresco tRC luego de enviar el comando auto refresh.
			end
		
		INIT_WAIT_TRC: 
			begin
				// ---- Outputs ----

				// ---- Transitions ----
				if (delay_counter == TRC_CYCLES - 1)  // Esperar tRC.
					begin
						// Luego de esperar tRC, comprobar si se han completado los 8 ciclos de refresco.
						if (init_counter == INIT_REFRESH_COUNT - 1) 
							begin
								// Completados los 8 ciclos, configurar registro de modo.
                                next_command = CMD_MRS;

								next_state = INIT_MRS;
							end
						else 
							begin
								// Aún no completa los 8 ciclos de Auto Refresh.
                                init_counter_count_one_cycle = 1'b1;  // Contar un ciclo de refresco completado.
								next_command = CMD_AUTO_REFRESH;

								next_state = INIT_AUTO_REFRESH;
							end
					end
			end
		
		INIT_MRS: 
			begin
				// ---- Outputs ----
				// Mode Register Set.
				// Comando CMD_MRS emitido en el estado anterior.

				reset_delay_counter = 1'b1;
                next_command = CMD_NOP;  // Emitir comando CMD_NOP durante los tiempos de espera.

				// ---- Transitions ----
				next_state = INIT_WAIT_TRSC;
			end
		
		INIT_WAIT_TRSC: 
			begin
				// ---- Outputs ----

				// ---- Transitions ----
				if (delay_counter == TRSC_CYCLES - 1)  // Esperar tRSC (Mode Register Set Cycle Time) después del Mode Register Set
					begin
						reset_refresh_counter = 1'b1;   // Starts periodic refresh cycles.
                        next_command = CMD_NOP;         // The No Operation Command should be used in cases when the SDRAM is in a idle or a wait state.

						next_state = IDLE;			// Inicialización completada.
					end
			end
		

		// ---- IDLE State ----
		IDLE: 
			begin
				// ---- Outputs ----
				busy_signal = 1'b0;
				ready_signal = 1'b1;

				// ---- Transitions ----
				if (refresh_counter == CYCLES_BETWEEN_REFRESH - 1) 
					begin
						/*
							Inicia secuencia de auto refresh.
							El contador de auto refresco se incrementa siempre y se resetea al finalizar 
							la secuencia de auto refresh, en el estado REFRESH_WAIT_TRC.
						*/
						next_command = CMD_PRECHARGE_ALL;

                        next_state = REFRESH_PRECHARGE_ALL;
					end
				else if (soc_side_wr_en_port) 
					begin
						// Inicia secuencia de escritura.
						next_command = CMD_BANK_ACTIVATE;

                        next_state = WRITE_BANK_ACTIVATE;
					end
				else if (soc_side_rd_en_port) 
					begin
						// Inicia secuencia de lectura.
						next_command = CMD_BANK_ACTIVATE;

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
				// Precarga de todos los bancos.
				// Comando CMD_PRECHARGE_ALL emitido en el estado anterior IDLE.

				reset_delay_counter = 1'b1;
                next_command = CMD_NOP;  // Emitir comando CMD_NOP durante los tiempos de espera.

				// ---- Transitions ----
				next_state = REFRESH_WAIT_TRP;
			end

		REFRESH_WAIT_TRP: 
			begin
				// ---- Outputs ----

				// ---- Transitions ----
				if (delay_counter == TRP_CYCLES - 1)  // Esperar tRP nanosegundos después de emitir el comando precharge all.
					begin
						next_command = CMD_AUTO_REFRESH;

                        next_state = REFRESH_AUTO_REFRESH;
					end
			end

		REFRESH_AUTO_REFRESH:
			begin
				// ---- Outputs ----
				/*
					Auto Refresh.

					Comando CMD_AUTO_REFRESH emitido en el estado anterior.
					El comando CMD_AUTO_REFRESH se activa durante un solo ciclo de reloj, inmediatamente después 
					la SDRAM espera tRC con comando NOP.
				*/

				reset_delay_counter = 1'b1;
                next_command = CMD_NOP;  // Emitir comando CMD_NOP durante los tiempos de espera.
				
				// ---- Transitions ----
				next_state = REFRESH_WAIT_TRC;
			end

		REFRESH_WAIT_TRC:
			begin
				// ---- Outputs ----

				// ---- Transitions ----
				if (delay_counter == TRC_CYCLES - 1)  // Esperar tRC nanosegundos después de emitir el comando CMD_AUTO_REFRESH.
					begin
						
						reset_refresh_counter = 1'b1;   // Resetear el contador de refresco.
                        next_command = CMD_NOP;         // The No Operation Command should be used in cases when the SDRAM is in a idle or a wait state.

						next_state = IDLE;			// Volver al estado IDLE.
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
				// Activar el banco y fila especificados.
				// Comando CMD_BANK_ACTIVATE emitido en el estado anterior IDLE.

				// See:
				//----------------------------------------------------------
				// Combinational logic block for generating bank address and 
				// word address for the SDRAM based on the current state.
				//----------------------------------------------------------

				reset_delay_counter = 1'b1;
                next_command = CMD_NOP;  // Emitir comando CMD_NOP durante los tiempos de espera.

				// ---- Transitions ----
				next_state = WRITE_WAIT_TRCD;
			end

		WRITE_WAIT_TRCD: 
			begin
				// ---- Outputs ----

				// ---- Transitions ----
				if (delay_counter == TRCD_CYCLES - 1)  // Esperar tRCD.
					begin
                        next_command = CMD_WRITE;

						next_state = WRITE_CAS;
					end
			end

		WRITE_CAS: 
			begin
				// ---- Outputs ----
				/*
					WRITE_CAS command emitted in the previous state.
					The data to be written is already in the wr_data_reg register; it is captured 
					in wr_data_reg in the sequential block with the rising edge of clk.
				*/

				// See:
				//----------------------------------------------------------
				// Combinational logic block for generating bank address and 
				// word address for the SDRAM based on the current state.
				//----------------------------------------------------------

				reset_delay_counter = 1'b1;
                next_command = CMD_NOP;  // Emitir comando CMD_NOP durante los tiempos de espera.

				// ---- Transitions ----
				next_state = WRITE_WAIT_TWR;
			end

		WRITE_WAIT_TWR: 
			begin
				// ---- Outputs ----
				// After waiting tWR nanoseconds the data has already been written to the SDRAM memory.

				// ---- Transitions ----
				if (delay_counter == TWR_CYCLES - 1)  // Wait tWR nanoseconds.
					begin
						next_command = CMD_NOP;  // The No Operation Command should be used in cases when the SDRAM is in a idle or a wait state.

						next_state = IDLE;  // Volver a IDLE directamente.
					end
			end

		// WRITE_WAIT_TWR: 
		// 	begin
		// 		// ---- Outputs ----
		// 		// After waiting tWR nanoseconds the data has already been written to the SDRAM memory.

		// 		// ---- Transitions ----
		// 		if (delay_counter == TWR_CYCLES - 1)  // Wait tWR nanoseconds.
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

		// 						next_command = CMD_NOP;  // The No Operation Command should be used in cases when the SDRAM is in a idle or a wait state.

        //                      next_state = IDLE;  // Volver a IDLE directamente.
		// 					end
		// 				else 
		// 					begin
		// 						next_command = CMD_PRECHARGE_BANK;

        //                      next_state = WRITE_PRECHARGE;
		// 					end
		// 			end
		// 	end

		// WRITE_PRECHARGE: 
		// 	begin
		// 		// ---- Outputs ----
		// 		// Precargar el banco específico.
		// 		// Comando CMD_PRECHARGE_BANK emitido en el estado anterior.

		// 		reset_delay_counter = 1'b1;
        //         next_command = CMD_NOP;  // Emitir comando CMD_NOP durante los tiempos de espera.

		// 		// ---- Transitions ----
		// 		next_state = WRITE_WAIT_TRP;
		// 	end

		// WRITE_WAIT_TRP: 
		// 	begin
		// 		// ---- Outputs ----

		// 		// ---- Transitions ----
		// 		if (delay_counter == TRP_CYCLES - 1)  // Esperar tRP.
		// 			begin
		// 				next_command = CMD_NOP;		// The No Operation Command should be used in cases when the SDRAM is in a idle or a wait state.

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
				// Activar el banco y fila necesarios.
				// current_command = CMD_BANK_ACTIVATE, emitido en el estado anterior IDLE.

				// See:
				//----------------------------------------------------------
				// Combinational logic block for generating bank address and 
				// word address for the SDRAM based on the current state.
				//----------------------------------------------------------

				reset_delay_counter = 1'b1;
                next_command = CMD_NOP;  // Emitir comando CMD_NOP durante los tiempos de espera.

				// ---- Transitions ----
				next_state = READ_WAIT_TRCD;
			end
		
		READ_WAIT_TRCD:
			begin
				// ---- Outputs ----
				// current_command = CMD_NOP, emitido en el estado anterior.

				// ---- Transitions ----
				if (delay_counter == TRCD_CYCLES - 1)  // Esperar tRCD después de activar el banco.
					begin
						next_command = CMD_READ;

                        next_state = READ_CAS;
					end
			end

		READ_CAS:
			begin
				// ---- Outputs ----
				// Emitir comando de lectura.
				// current_command = CMD_READ, emitido en el estado anterior.

				// See:
				//----------------------------------------------------------
				// Combinational logic block for generating bank address and 
				// word address for the SDRAM based on the current state.
				//----------------------------------------------------------

				reset_delay_counter = 1'b1;
                next_command = CMD_NOP;  // Emitir comando CMD_NOP durante los tiempos de espera.

				// ---- Transitions ----
				next_state = READ_WAIT_CAS_LATENCY;
			end

		READ_WAIT_CAS_LATENCY:
			begin
				// ---- Outputs ----

				// ---- Transitions ----
				if (delay_counter == CAS_LATENCY - 1)  // Esperar CAS_LATENCY ciclos antes de que los datos estén disponibles.
					begin
						next_command = CMD_NOP;

                        next_state = READ_DATA;
					end
			end

		READ_DATA:
			begin
				// ---- Outputs ----
				/*
					Luego de esperar CAS_LATENCY ciclos los datos se capturan en el bloque secuencial 
					en el registro rd_data_reg con el flanco de subida de clk.
				*/

				// ---- Transitions ----
				next_command = CMD_NOP;		// The No Operation Command should be used in cases when the SDRAM is in a idle or a wait state.

				next_state = IDLE;			// Volver a IDLE.
			end

		// READ_DATA:
		// 	begin
		// 		// ---- Outputs ----
		// 		/*
		// 			Luego de esperar CAS_LATENCY ciclos los datos se capturan en el bloque secuencial 
		// 			en el registro rd_data_reg con el flanco de subida de clk.
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

		// 				next_command = CMD_NOP;		// The No Operation Command should be used in cases when the SDRAM is in a idle or a wait state.

        //              next_state = IDLE;			// Volver a IDLE directamente.
		// 			end
		// 		else
		// 			begin
		// 				next_command = CMD_PRECHARGE_BANK;

        //                 next_state = READ_PRECHARGE;
		// 			end
		// 	end

		// READ_PRECHARGE:
		// 	begin
		// 		// ---- Outputs ----
		// 		// Precargar el banco específico.
		// 		// Comando CMD_PRECHARGE_BANK emitido en el estado anterior.

		// 		reset_delay_counter = 1'b1;
        //         next_command = CMD_NOP;  // Emitir comando CMD_NOP durante los tiempos de espera.

		// 		// ---- Transitions ----
		// 		next_state = READ_WAIT_TRP;
		// 	end

		// READ_WAIT_TRP:
		// 	begin
		// 		// ---- Outputs ----

		// 		// ---- Transitions ----
		// 		if (delay_counter == TRP_CYCLES - 1)  // Esperar tRP después de precargar.
		// 			begin
		// 				next_command = CMD_NOP;		// The No Operation Command should be used in cases when the SDRAM is in a idle or a wait state.

        //                 next_state = IDLE;			// Volver a IDLE.
		// 			end
		// 	end

		
		default: 
			begin
				next_command = CMD_NOP;  // The No Operation Command should be used in cases when the SDRAM is in a idle or a wait state.

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
			soc_side_wr_mask_port = 0000 --> No write. Read operation.
			soc_side_wr_mask_port = 1111 --> Write 32 bits.
			soc_side_wr_mask_port = 1100 --> Write upper 16 bits.
			soc_side_wr_mask_port = 0011 --> Write lower 16 bits.
			soc_side_wr_mask_port = 0001 --> Write Byte 0.
			soc_side_wr_mask_port = 0010 --> Write Byte 1.
			soc_side_wr_mask_port = 0100 --> Write Byte 2.
			soc_side_wr_mask_port = 1000 --> Write Byte 3.
	*/
	if (in_initialization_cycle)
		// Disable buffers during initialization. Mask all bits high so that the SDRAM drives the input/output buffers to HIGH-Z.
		sdram_side_wr_mask_reg = 4'b1111;
	else if (in_write_cycle)
		/*
			PicoRV32 SOC, mem_wstrb signals:
				soc_side_wr_mask_port = 0000 --> No write. Read memory.
				soc_side_wr_mask_port = 1111 --> Write 32 bits.
				soc_side_wr_mask_port = 1100 --> Write upper 16 bits.
				soc_side_wr_mask_port = 0011 --> Write lower 16 bits.
				soc_side_wr_mask_port = 0001 --> Write Byte 0.
				soc_side_wr_mask_port = 0010 --> Write Byte 1.
				soc_side_wr_mask_port = 0100 --> Write Byte 2.
				soc_side_wr_mask_port = 1000 --> Write Byte 3.

			The mem_wstrb signals of the SoC are inverted because the LDQM and UDQM pins of the 
			SDRAM set the input/output buffers to HIGH-Z with 1 and enable them with 0.
		*/
		sdram_side_wr_mask_reg = ~soc_side_wr_mask_port;  // Use the 4-bit write mask from the soc (mem_wstrb signals).
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
	ram_addr_reg = {SDRAM_ADDR_WIDTH{1'b0}};
	
	if (current_state == READ_BANK_ACTIVATE || current_state == WRITE_BANK_ACTIVATE)
		begin
			ram_bank_addr_reg = soc_side_addr_port[SOC_SIDE_ADDR_WIDTH - 1: SOC_SIDE_ADDR_WIDTH - BANK_ADDR_WIDTH];
			ram_addr_reg = soc_side_addr_port[SOC_SIDE_ADDR_WIDTH - (BANK_ADDR_WIDTH + 1): SOC_SIDE_ADDR_WIDTH - (BANK_ADDR_WIDTH + ROW_WIDTH)];
		end
	else if (current_state == READ_CAS || current_state == WRITE_CAS)
		begin
			ram_bank_addr_reg = soc_side_addr_port[SOC_SIDE_ADDR_WIDTH - 1: SOC_SIDE_ADDR_WIDTH - BANK_ADDR_WIDTH];

			/*
									 BANK	 ROW	 COL
				SOC_SIDE_ADDR_WIDTH   2	  +  12   +   9   = 23 
				SDRAM_ADDR_WIDTH 13
			*/
			ram_addr_reg = { {(SDRAM_ADDR_WIDTH - 11){1'b0}},		/* 0s */
							1'b1,									/* 1 (A10 is always for auto precharge) */
							{(10 - COL_WIDTH){1'b0}},				/* 0s */
							soc_side_addr_port[COL_WIDTH - 1: 0]	/* column address */
						};
		end
	else if (current_state == INIT_MRS)
		begin
            /*
                Mode Register Set (MRS) configuration during initialization:

                    Burst Length of 1 (no burst)      -> bits 0-2 = 000
                    Sequential burst type             -> bit 3    = 0
                    CAS Latency of 3                  -> bits 4-6 = 011
                    Standard operation mode           -> bits 7-8 = 00
                    "Burst Read/Single Write" setting -> bit 9    = 1

                    = 10'b 1 00 011 0 000
            */
			ram_addr_reg = { {(SDRAM_ADDR_WIDTH - 10){1'b0}}, 10'b1000110000 };
		end
end


endmodule

