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


//-------------------------------
// Par�metros de Inicializaci�n 
//-------------------------------

localparam INIT_PAUSE_US = 200;  // Pausa inicial de 200us.
localparam INIT_PAUSE_CYCLES = INIT_PAUSE_US * CLK_FREQUENCY_MHZ;

// N�mero de ciclos de refresco requeridos durante la inicializaci�n.
localparam INIT_REFRESH_COUNT = 8;

localparam CYCLES_BETWEEN_REFRESH = (CLK_FREQUENCY_MHZ * 1_000 * REFRESH_TIME_MS) / REFRESH_COUNT;


//-------------------------------
// Definici�n de Estados
//-------------------------------

/*
	Se utiliza codificaci�n Grey.

	Ventajas de la Codificaci�n Gray:
		- Reducci�n de glitches: Como solo un bit cambia entre estados adyacentes, minimiza 
		  la posibilidad de estados transitorios no deseados.
		- Menor consumo de energ�a: Menos bits cambian durante las transiciones, reduciendo 
		  la actividad de conmutaci�n.
		- Mayor inmunidad al ruido: Reduce la probabilidad de transiciones de estado 
		  incorrectas debido a ruido.

	Registro de estados de 5 bits: 2^5 = 32 estados m�ximo.
*/

localparam IDLE				= 5'b00000;

// SDRAM initialization.
localparam INIT_PAUSE		= 5'b00001;  // Pausa inicial de 200us.
localparam INIT_PRECHARGE	= 5'b00011;  // Precargar todos los bancos.
localparam INIT_NOP1		= 5'b00010;  // NOP despu�s de precarga.
localparam INIT_REFRESH		= 5'b00110;  // Auto Refresh (repetido 8 veces).
localparam INIT_NOP2		= 5'b00111;  // NOP despu�s de auto refresh.
localparam INIT_MRS			= 5'b00101;  // Mode Register Set (MRS).
localparam INIT_NOP3		= 5'b00100;  // NOP despu�s de MRS.


//-------------------------------
// Definici�n de Comandos SDRAM
//-------------------------------

localparam CMD_PALL	= 8'b10010001;  // Precharge All
localparam CMD_REF	= 8'b10001000;  // Auto Refresh
localparam CMD_NOP	= 8'b10111000;  // No Operation
localparam CMD_MRS	= 8'b1000000x;  // Mode Register Set


//-------------------------------
// Internal Registers and Wires
//-------------------------------

// Internal state machine signals
reg [4:0] state, next_state;
reg [14:0] delay_counter, next_delay_counter;		// Para retardos de tiempo entre comandos.
reg [3:0] refresh_counter, next_refresh_counter;	// Cuenta los ciclos de auto-refresh.
reg [7:0] command, next_command;					// Comando SDRAM.

// Internal registers for CPU interface.
reg [31:0] wr_data_r;
reg [31:0] rd_data_r;
reg rd_ready_r;

// Internal registers for SDRAM address generation.
reg [SDRAM_ADDR_WIDTH-1:0] addr_r;
reg [BANK_WIDTH-1:0] bank_addr_r;

// Internal registers for Byte mask signals.
reg [3:0] sdram_side_wr_mask_reg;

wire in_refresh_cycle;


//-------------------------------
// Output assignments
//-------------------------------

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

// Signal to detect refresh cycle states.
assign in_refresh_cycle = (state == REF_PRE) || (state == REF_NOP1) || (state == REF_REF) || (state == REF_NOP2);


//-------------------------------
// L�gica Secuencial
//-------------------------------

always @(posedge clk) 
begin
	if (~soc_side_rst_n) 
		begin
			state <= INIT_PAUSE;
			command <= CMD_NOP;

			// Counters.
			delay_counter <= INIT_PAUSE_CYCLES;  // Inicializar para la pausa de 200us.
			refresh_counter <= 0;

			// Ports.
			wr_data_r <= 32'b0;
			rd_data_r <= 32'b0;
			soc_side_busy <= 1'b1;  // El controlador est� ocupado durante inicializaci�n.
		end
	else 
		begin
			state <= next_state;
			delay_counter <= next_delay_counter;
			refresh_counter <= next_refresh_counter;
			command <= next_command;
			
			if (soc_side_wr_enable)
				wr_data_r <= soc_side_wr_data;

			if (state == READ_READ)
				begin
					rd_data_r <= {ram_side_data_chip1, ram_side_data_chip0};
					rd_ready_r <= 1'b1;
				end
			else
				rd_ready_r <= 1'b0;

			// Indicar que el controlador est� ocupado durante inicializaci�n, operaciones
			// de lectura/escritura y ciclos de refresco.
			soc_side_busy <= (state == INIT_PAUSE || state == INIT_PRECHARGE || 
							state == INIT_NOP1 || state == INIT_REFRESH || 
							state == INIT_NOP2 || state == INIT_MRS || 
							state == INIT_NOP3 || state[5] || in_refresh_cycle);
		end
end

/* Handle refresh counter. */
always @ (posedge clk)
begin
	if (~soc_side_rst_n)
		refresh_counter <= 10'b0;
	else if (state == REF_NOP2)
		refresh_counter <= 10'b0;
	else
		refresh_counter <= refresh_counter + 1'b1;
end


//-------------------------------
// L�gica Combinacional 
//-------------------------------

// Next state logic.
always @* 
begin
	// Valores por defecto para evitar latches.
	next_state = state;
	next_delay_counter = delay_counter;
	next_refresh_counter = refresh_counter;
	next_command = CMD_NOP;
	
	// Manejo de las se�ales DQM - always high during initialization
	if (state == INIT_PAUSE || state == INIT_PRECHARGE || state == INIT_NOP1 || state == INIT_REFRESH || 
		state == INIT_NOP2 || state == INIT_MRS || state == INIT_NOP3)
		sdram_side_wr_mask_reg = 4'b1111; // DQM high (deshabilita buffers)
	else if (state[5]) // Estados de lectura/escritura
		if (soc_side_wr_mask)
			sdram_side_wr_mask_reg = ~soc_side_wr_mask;
		else
			sdram_side_wr_mask_reg = 4'b0000;
	else
		sdram_side_wr_mask_reg = 4'b1111; // Por defecto, deshabilita buffers
		
	// L�gica de la m�quina de estados
	case (state)
		IDLE: 
		begin
			// L�gica para salir de IDLE (sin cambios de tu implementaci�n original)
			if (refresh_counter >= CYCLES_BETWEEN_REFRESH) 
			begin
				next_state = REF_PRE;
				next_command = CMD_PALL;
			end
			else if (soc_side_rd_enable) 
			begin
				next_state = READ_ACT;
				next_command = CMD_BACT;
			end
			else if (soc_side_wr_enable) 
			begin
				next_state = WRIT_ACT;
				next_command = CMD_BACT;
			end
		end
		
		//---------- Estados de Inicializaci�n ----------
		
		INIT_PAUSE: 
		begin
			// Esperar 200us antes de comenzar inicializaci�n
			if (init_pause_counter != 0)
				next_init_pause_counter = init_pause_counter - 1'b1;
			else 
			begin
				next_state = INIT_PRECHARGE;
				next_command = CMD_PALL;
			end
		end
		
		INIT_PRECHARGE: 
		begin
			// Precarga de todos los bancos
			next_state = INIT_NOP1;
			next_delay_counter = 4'd2; // Esperar tRP (precharge to active/refresh)
		end
		
		INIT_NOP1: 
		begin
			// Esperar despu�s de precarga
			if (delay_counter != 0)
				next_delay_counter = delay_counter - 1'b1;
			else 
			begin
				next_state = INIT_REFRESH;
				next_command = CMD_REF;
				next_refresh_counter = 0; // Iniciar los 8 ciclos de refresh
			end
		end
		
		INIT_REFRESH: 
		begin
			// Auto Refresh - ciclo actual
			next_state = INIT_NOP2;
			next_delay_counter = 4'd7; // Esperar tRFC (refresh cycle time)
		end
		
		INIT_NOP2: 
		begin
			// Esperar despu�s del Auto Refresh
			if (delay_counter != 0)
				next_delay_counter = delay_counter - 1'b1;
			else 
			begin
				if (refresh_counter < INIT_REFRESH_COUNT - 1) 
				begin
					// A�n faltan ciclos de Auto Refresh
					next_state = INIT_REFRESH;
					next_command = CMD_REF;
					next_refresh_counter = refresh_counter + 1'b1;
				end
				else 
				begin
					// Completados los 8 ciclos, configurar registro de modo
					next_state = INIT_MRS;
					next_command = CMD_MRS;
				end
			end
		end
		
		INIT_MRS: 
		begin
			// Mode Register Set
			next_state = INIT_NOP3;
			next_delay_counter = 4'd2; // Esperar tMRD (mode register set cycle time)
		end
		
		INIT_NOP3: 
		begin
			// Esperar despu�s del Mode Register Set
			if (delay_counter != 0)
				next_delay_counter = delay_counter - 1'b1;
			else 
			begin
				next_state = IDLE; // Inicializaci�n completada
			end
		end
		
		// Otros estados.....
		
		default: 
			next_state = IDLE;

	endcase
end

// TODO: nuevo bloque, verifcar con el bloque anterio: Handle logic for sending addresses to SDRAM based on current state.
// L�gica para configuraci�n del registro de modo.
always @* 
begin
	bank_addr_r = 2'b00;
	addr_r = {SDRAM_ADDR_WIDTH{1'b0}};
	
	if (state == READ_ACT || state == WRIT_ACT)
		begin
			// Tu implementaci�n original para lectura/escritura
			bank_addr_r = soc_side_addr[SOC_SIDE_ADDR_WIDTH-1:SOC_SIDE_ADDR_WIDTH-BANK_WIDTH];
			addr_r = soc_side_addr[SOC_SIDE_ADDR_WIDTH-(BANK_WIDTH+1):SOC_SIDE_ADDR_WIDTH-(BANK_WIDTH+ROW_WIDTH)];
		end
	else if (state == READ_CAS || state == WRIT_CAS)
		begin
			// Tu implementaci�n original para operaciones CAS
			bank_addr_r = soc_side_addr[SOC_SIDE_ADDR_WIDTH-1:SOC_SIDE_ADDR_WIDTH-BANK_WIDTH];
			addr_r = {{(SDRAM_ADDR_WIDTH-11){1'b0}}, 1'b1, {(10-COL_WIDTH){1'b0}}, 
					  soc_side_addr[COL_WIDTH-1:0]};
		end
	else if (state == INIT_MRS)
		begin
			// Configuraci�n del registro de modo durante inicializaci�n
			//											B  C  SB
			//											R  A  EUR
			//											S  S-3Q ST
			//											T  654L210
			addr_r = {{(SDRAM_ADDR_WIDTH-10){1'b0}}, 10'b1000110000};  // CAS=3, BL=1, Sequential
		end
end

// Handle logic for sending addresses to SDRAM based on current state.
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
										 BANK	 ROW	 COL
					SOC_SIDE_ADDR_WIDTH   2	  +  12   +   9   = 23 
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
			//									   B  C  SB
			//									   R  A  EUR
			//									   S  S-3Q ST
			//									   T  654L210
			addr_r = { {(SDRAM_ADDR_WIDTH - 10){1'b0}}, 10'b1000110000 };
		end
end


endmodule

