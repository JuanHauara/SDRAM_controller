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
// Parámetros de Inicialización 
//-------------------------------

localparam CYCLES_BETWEEN_REFRESH = (CLK_FREQUENCY_MHZ * 1_000 * REFRESH_TIME_MS) / REFRESH_COUNT;

// Tiempos para la inicialización de la SDRAM.
localparam INIT_PAUSE_US = 200;  // Pausa inicial de 200us.
localparam INIT_PAUSE_CYCLES = INIT_PAUSE_US * CLK_FREQUENCY_MHZ;

localparam TRP_NS = 15;  // Esperar Tiempo mínimo de precarga tRP (15ns), luego de emitir el comando precharge all banks.
localparam TRP_CYCLES = ((TRP_NS * CLK_FREQUENCY_MHZ) + 999) / 1000;  // Cantidad de ciclos de espera para cumplir con el tiempo tRP. Redondeo hacia arriba.

localparam INIT_REFRESH_COUNT = 8;  // Número de ciclos de refresco requeridos durante la inicialización.

localparam TRFC_NS = 70;  // Esperar tiempo de ciclo de refresco tRFC (~60-70ns), luego de emitir el comando auto refresh.
localparam TRFC_CYCLES = ((TRFC_NS * CLK_FREQUENCY_MHZ) + 999) / 1000;  // Cantidad de ciclos de espera para cumplir con el tiempo tRFC. Redondeo hacia arriba.

localparam TMRD_NS = 20;  // Esperar mode register set cycle time tMRD (), luego de emitir el comando mode register set (MRS).
localparam TMRD_CYCLES = ((TMRD_NS * CLK_FREQUENCY_MHZ) + 999) / 1000;  // Cantidad de ciclos de espera para cumplir con el tiempo tMRD. Redondeo hacia arriba.

// Tiempos para escritura.
localparam TRCD_NS = 15;  // Esperar tRCD (15ns), luego de emitir el comando CMD_ACTIVATE.
localparam TRCD_CYCLES = ((TRCD_NS * CLK_FREQUENCY_MHZ) + 999) / 1000;  // Cantidad de ciclos de espera para cumplir con el tiempo tRCD. Redondeo hacia arriba.

localparam TWR_NS = 15;  // Esperar tWR (15ns), luego de emitir el comando CMD_WRITE.
localparam TWR_CYCLES = ((TWR_NS * CLK_FREQUENCY_MHZ) + 999) / 1000;  // Cantidad de ciclos de espera para cumplir con el tiempo tWR. Redondeo hacia arriba.


//-------------------------------
// Definición de Estados
//-------------------------------

/*
	Se utiliza codificación Grey.

	Ventajas de la Codificación Gray:
		- Reducción de glitches: Como solo un bit cambia entre estados adyacentes, minimiza 
		  la posibilidad de estados transitorios no deseados.
		- Menor consumo de energía: Menos bits cambian durante las transiciones, reduciendo 
		  la actividad de conmutación.
		- Mayor inmunidad al ruido: Reduce la probabilidad de transiciones de estado 
		  incorrectas debido a ruido.

	Registro de estados de 5 bits: 2^5 = 32 estados máximo.
*/

localparam IDLE						= 5'b00000;

// SDRAM initialization.
localparam INIT_PAUSE				= 5'b00001;  // Pausa inicial de 200us.
localparam INIT_PRECHARGE_ALL		= 5'b00011;  // Precargar todos los bancos.
localparam INIT_WAIT_TRP			= 5'b00010;  // Esperar tRP nanosegundos después de precharge all.
localparam INIT_AUTO_REFRESH		= 5'b00110;  // 8 ciclos de Auto Refresh.
localparam INIT_WAIT_TRFC			= 5'b00111;  // Esperar tRFC después de cada uno de los 8 ciclos de auto refresh.
localparam INIT_MRS					= 5'b00101;  // Mode Register Set (MRS).
localparam INIT_WAIT_TMRD			= 5'b00100;  // Esperar tMRD nanosegundos después de MRS.

// States for the Auto refresh.
localparam REFRESH_PRECHARGE_ALL	= 5'b01100;  // Precargar todos los bancos antes del refresco.
localparam REFRESH_WAIT_TRP			= 5'b01101;  // Esperar tRP nanosegundos después de precharge all.
localparam REFRESH_AUTO_REFRESH		= 5'b01111;  // Emitir comando Auto Refresh.
localparam REFRESH_WAIT_TRFC		= 5'b01110;  // Esperar tRFC después de Auto Refresh.

// Estados para operación de escritura.
localparam WRITE_BANK_ACTIVATE		= 5'b01010;  // Activar fila/banco (Bank Activate Command).
localparam WRITE_WAIT_TRCD			= 5'b01011;  // NOP después de activación (esperar tRCD).
localparam WRITE_CAS				= 5'b01001;  // Comando de escritura y datos (Write Command).
localparam WRITE_WAIT_TWR			= 5'b01000;  // NOP después de escritura (esperar tWR).
localparam WRITE_PRECHARGE			= 5'b11000;  // Precargar banco (si no se usa auto-precharge).
localparam WRITE_WAIT_TRP			= 5'b11001;  // NOP después de precharge (esperar tRP).


//-------------------------------
// Definición de Comandos SDRAM
//-------------------------------

localparam CMD_PRECHARGE_ALL	= 8'b10010001;  // Precharge All.
localparam CMD_AUTO_REFRESH		= 8'b10001000;  // Auto Refresh.
localparam CMD_NOP				= 8'b10111000;  // No Operation.
localparam CMD_MRS				= 8'b1000000x;  // Mode Register Set (MRS).

// Comandos para operaciones de lectura/escritura.
localparam CMD_ACTIVATE			= 8'b10011xxx;  // Activar banco/fila (x = bank address bits).
localparam CMD_WRITE			= 8'b10100xx1;  // Comando de escritura (x = bank address bits).
localparam CMD_PRECHARGE_BANK	= 8'b10010x00;  // Precargar banco específico (x = a10 para auto-precharge).


//-------------------------------
// Internal Registers and Wires
//-------------------------------

// Internal state machine signals
reg [4:0] state, next_state;
reg [14:0] delay_counter, next_delay_counter;				// Para retardos de tiempo entre comandos.
reg [3:0] init_refresh_counter, next_init_refresh_counter;	// Cuenta los 8 ciclos de auto-refresh necesarios para la inicialización.
reg [15:0] refresh_counter, next_refresh_counter;			// Contador para seguimiento del momento del próximo refresco.
reg [7:0] command, next_command;  // Comando SDRAM.

// Internal registers for CPU interface.
reg [31:0] wr_data_reg;
reg [31:0] rd_data_reg;
reg read_ready_reg;

// Internal registers for SDRAM address generation.
reg [SDRAM_ADDR_WIDTH - 1: 0] addr_reg;
reg [BANK_WIDTH - 1: 0] bank_addr_reg;

// Internal registers for Byte mask signals.
reg [3:0] sdram_side_wr_mask_reg;

wire in_refresh_cycle;
wire is_read_write_state;


//-------------------------------
// Assignments
//-------------------------------

assign {ram_side_clock_enable, ram_side_cs_n, ram_side_ras_n, ram_side_cas_n, ram_side_wr_enable} = command[7:3];

assign ram_side_bank_addr = is_read_write_state ? bank_addr_reg : command[2:1];
assign ram_side_addr = (is_read_write_state /*|| state == INIT_LOAD*/) ? addr_reg : { {(SDRAM_ADDR_WIDTH - 11){1'b0}}, command[0], 10'd0 };  // TODO: Ver que era el estado INIT_LOAD en el código original.

assign soc_side_rd_data = rd_data_reg;
assign soc_side_rd_ready = read_ready_reg;

assign ram_side_ldqm_pin_chip0 = sdram_side_wr_mask_reg[0];
assign ram_side_udqm_pin_chip0 = sdram_side_wr_mask_reg[1];
assign ram_side_ldqm_pin_chip1 = sdram_side_wr_mask_reg[2];
assign ram_side_udqm_pin_chip1 = sdram_side_wr_mask_reg[3];

assign ram_side_data_chip0 = (state == WRITE_CAS) ? wr_data_reg[15:0] : 16'bz;
assign ram_side_data_chip1 = (state == WRITE_CAS) ? wr_data_reg[31:16] : 16'bz;

// Signal to detect refresh cycle states.
assign in_refresh_cycle = (state == REFRESH_PRECHARGE_ALL) || (state == REFRESH_WAIT_TRP) || 
						(state == REFRESH_AUTO_REFRESH) || (state == REFRESH_WAIT_TRFC);

// Signal to detect read or write cycle states.
assign is_read_write_state = (state == WRITE_BANK_ACTIVATE) || (state == WRITE_WAIT_TRCD) || (state == WRITE_CAS) ||
							(state == WRITE_WAIT_TWR) || (state == WRITE_PRECHARGE) || (state == WRITE_WAIT_TRP);  // TODO: Agregar estados de lectura de memoria.


//-------------------------------
// Lógica Secuencial
//-------------------------------

always @(posedge clk) 
begin
	if (~soc_side_rst_n) 
		begin
			state <= INIT_PAUSE;
			command <= CMD_NOP;

			// Counters.
			delay_counter <= INIT_PAUSE_CYCLES;  // Inicializar contador para la pausa de 200us.
			init_refresh_counter <= 0;
			refresh_counter <= 0;

			// Ports.
			wr_data_reg <= 32'b0;
			rd_data_reg <= 32'b0;
			soc_side_busy <= 1'b1;  // El controlador está ocupado durante inicialización.
		end
	else 
		begin
			state <= next_state;

			delay_counter <= next_delay_counter;
			init_refresh_counter <= next_init_refresh_counter;
			refresh_counter <= next_refresh_counter;

			command <= next_command;
			
			if (soc_side_wr_enable)
				wr_data_reg <= soc_side_wr_data;  // Update write data.

			// Update read data and read ready signal.
			/*if (state == READ_READ)  TODO: Descomentar esto cuando implemente la lectura.
				begin
					rd_data_reg <= {ram_side_data_chip1, ram_side_data_chip0};
					read_ready_reg <= 1'b1;
				end
			else
				read_ready_reg <= 1'b0;*/

			// Indicar que el controlador está ocupado durante inicialización, operaciones
			// de lectura/escritura y ciclos de refresco.
			soc_side_busy <= (state == INIT_PAUSE || state == INIT_PRECHARGE_ALL || state == INIT_WAIT_TRP || 
							state == INIT_AUTO_REFRESH || state == INIT_WAIT_TRFC || state == INIT_MRS || 
							state == INIT_WAIT_TMRD || in_refresh_cycle || is_read_write_state);
		end
end


//-------------------------------
// Lógica Combinacional 
//-------------------------------

/*
	Next state logic for the state machine.

	Lógica en la implementación de los estados:

	ESTADO_ACTUAL:
		begin
			// Comprobar entradas y otras condiciones.

			next_state = SIGUIENTE_ESTADO;
			next_command = COMANDO_PARA_SIGUIENTE_ESTADO;

			// Inicializar next_delay_counter si el siguiente estado necesita cumplir con una espera de tiempo.
			// Otras asignaciones.
		end

	Con esta estructura, cuando la máquina se encuentra en ESTADO_ACTUAL, ya está emitiendo el comando apropiado 
	para ese estado (asignado en el estado anterior).
*/
always @* 
begin
	// Valores por defecto para evitar latches.
	next_state = state;
	next_command = CMD_NOP;

	next_delay_counter = delay_counter;
	next_init_refresh_counter = init_refresh_counter;
	next_refresh_counter = refresh_counter + 1'b1;  // Siempre incrementa, se resetea en REF_NOP2
	
	// Lógica de la máquina de estados.
	case (state)
		//---------- Estados de Inicialización ----------
		/*
			Initialization Sequence.

			The implemented sequence follows the requirements of the W9812G6KH-5I datasheet:
				- INIT_PAUSE: Wait 200us.
				- INIT_PRECHARGE_ALL: Emmits precharge all banks command.
				- INIT_WAIT_TRP: Wait tRP nanoseconds after precharge all.
				- INIT_AUTO_REFRESH + INIT_WAIT_TRFC: Emmits Auto Refresh command and Wait tRFC nanoseconds, repeat 8 times.
				- INIT_MRS: Mode register configuration.
				- INIT_WAIT_TMRD: Final wait before transitioning to IDLE.
		*/
		INIT_PAUSE: 
			begin
				// Esperar 200us antes de comenzar la inicialización.

				if (delay_counter != 0)
					next_delay_counter = delay_counter - 1'b1;
				else 
					begin
						next_state = INIT_PRECHARGE_ALL;
						next_command = CMD_PRECHARGE_ALL;
					end
			end
		
		INIT_PRECHARGE_ALL: 
			begin
				// Precarga de todos los bancos.
				// Comando CMD_PRECHARGE_ALL emitido en el estado anterior.

				next_state = INIT_WAIT_TRP;
				next_command = CMD_NOP;  // Emitir comando CMD_NOP durante los tiempos de espera.

				next_delay_counter = TRP_CYCLES;  // Inicializar contador para esperar tRP nanosegundos en el siguiente estado.
			end
		
		INIT_WAIT_TRP: 
			begin
				// Esperar tRP nanosegundos después de emitir el comando precharge all.

				if (delay_counter != 0)
					next_delay_counter = delay_counter - 1'b1;
				else 
					begin
						next_state = INIT_AUTO_REFRESH;
						next_command = CMD_AUTO_REFRESH;

						next_init_refresh_counter = 0; // Cuenta los 8 ciclos de auto refresh requeridos para la inicialización.
					end
			end
		
		INIT_AUTO_REFRESH: 
			begin
				/*
					Auto Refresh.

					Comando CMD_AUTO_REFRESH emitido en el estado anterior.
					El comando CMD_AUTO_REFRESH se activa durante un solo ciclo de reloj, inmediatamente después 
					la SDRAM espera tRFC con comando NOP.
				*/

				next_state = INIT_WAIT_TRFC;	// Esperar tiempo de ciclo de refresco tRFC luego de enviar el comando auto refresh.
				next_command = CMD_NOP;			// Emitir comando CMD_NOP durante los tiempos de espera.
				
				next_delay_counter = TRFC_CYCLES;  // Esperar tRFC nanosegundos.
			end
		
		INIT_WAIT_TRFC: 
			begin
				if (delay_counter != 0)  // Esperar tRFC después de emitir el comando CMD_AUTO_REFRESH.
					next_delay_counter = delay_counter - 1'b1;
				else 
					begin
						if (init_refresh_counter < INIT_REFRESH_COUNT - 1) 
							begin
								// Aún no completa los 8 ciclos de Auto Refresh.
								next_state = INIT_AUTO_REFRESH;
								next_command = CMD_AUTO_REFRESH;

								next_init_refresh_counter = init_refresh_counter + 1'b1;
							end
						else 
							begin
								// Completados los 8 ciclos, configurar registro de modo.
								next_state = INIT_MRS;
								next_command = CMD_MRS;
							end
					end
			end
		
		INIT_MRS: 
			begin
				// Mode Register Set.
				// Comando CMD_MRS emitido en el estado anterior.

				next_state = INIT_WAIT_TMRD;
				next_command = CMD_NOP;  // Emitir comando CMD_NOP durante los tiempos de espera.

				next_delay_counter = TMRD_CYCLES;  // Esperar tMRD (mode register set cycle time) en el siguiente estado.
			end
		
		INIT_WAIT_TMRD: 
			begin
				// Esperar después del Mode Register Set
				if (delay_counter != 0)
					next_delay_counter = delay_counter - 1'b1;
				else 
					next_state = IDLE;			// Inicialización completada.
					next_command = CMD_NOP;		// The No Operation Command should be used in cases when the SDRAM is in a idle or a wait state.
			end
		

		//---------- Estado IDLE ----------
		IDLE: 
			begin
				// Lógica para salir de IDLE.
				if (refresh_counter >= CYCLES_BETWEEN_REFRESH) 
					begin
						// Inicia secuencia de auto refresh.
						next_state = REFRESH_PRECHARGE_ALL;
						next_command = CMD_PRECHARGE_ALL;
						// No resetear refresh_counter aquí, se hace al final de la secuencia de auto refresco.
					end
				else if (soc_side_rd_enable) 
					begin
						/*next_state = READ_ACT;
						next_command = CMD_BACT;*/
					end
				else if (soc_side_wr_enable) 
					begin
						next_state = WRITE_BANK_ACTIVATE;
						next_command = CMD_ACTIVATE;
					end
			end


		//---------- Estados para el auto refresh ----------
		REFRESH_PRECHARGE_ALL: 
			begin
				// Precarga de todos los bancos.
				// Comando CMD_PRECHARGE_ALL emitido en el estado anterior IDLE.

				next_state = REFRESH_WAIT_TRP;
				next_command = CMD_NOP;  // Emitir comando CMD_NOP durante los tiempos de espera.

				next_delay_counter = TRP_CYCLES;  // Inicializar contador para esperar tRP nanosegundos en el siguiente estado.
			end

		REFRESH_WAIT_TRP: 
			begin
				// Esperar tRP nanosegundos después de emitir el comando precharge all.

				if (delay_counter != 0)
					next_delay_counter = delay_counter - 1'b1;
				else 
					begin
						next_state = REFRESH_AUTO_REFRESH;
						next_command = CMD_AUTO_REFRESH;
					end
			end

		REFRESH_AUTO_REFRESH:
			begin
				/*
					Auto Refresh.

					Comando CMD_AUTO_REFRESH emitido en el estado anterior.
					El comando CMD_AUTO_REFRESH se activa durante un solo ciclo de reloj, inmediatamente después 
					la SDRAM espera tRFC con comando NOP.
				*/
				
				next_state = REFRESH_WAIT_TRFC;
				next_command = CMD_NOP;  // Emitir comando CMD_NOP durante los tiempos de espera.

				next_delay_counter = TRFC_CYCLES;  // Esperar tRFC nanosegundos.
			end

		REFRESH_WAIT_TRFC:
			begin
				// Esperar tRFC después de emitir el comando CMD_AUTO_REFRESH.

				if (delay_counter != 0)
					next_delay_counter = delay_counter - 1'b1;
				else 
					begin
						next_state = IDLE;			// Volver al estado IDLE.
						next_command = CMD_NOP;		// The No Operation Command should be used in cases when the SDRAM is in a idle or a wait state.

						next_refresh_counter = 0;  // Resetear el contador de refresco.
					end
			end


		//---------- Estados para escritura ----------
		WRITE_BANK_ACTIVATE: 
			begin
				// Activar el banco y fila especificados.
				// Comando CMD_ACTIVATE emitido en el estado anterior IDLE.

				next_state = WRITE_WAIT_TRCD;
				next_command = CMD_NOP;  // Emitir comando CMD_NOP durante los tiempos de espera.

				next_delay_counter = TRCD_CYCLES;  // Esperar tRCD.
			end

		WRITE_WAIT_TRCD: 
			begin
				// Esperar tRCD.
				
				if (delay_counter != 0)
					next_delay_counter = delay_counter - 1'b1;
				else 
					begin
						next_state = WRITE_CAS;
						next_command = CMD_WRITE;
						// Los datos ya están en wr_data_reg
					end
			end

		WRITE_CAS: 
			begin
				// Comando de escritura emitido en el estado anterior, datos enviados.

				next_state = WRITE_WAIT_TWR;
				next_command = CMD_NOP;  // Emitir comando CMD_NOP durante los tiempos de espera.

				next_delay_counter = TWR_CYCLES;  // Esperar tWR.
			end

		WRITE_WAIT_TWR: 
			begin
				if (delay_counter != 0)
					next_delay_counter = delay_counter - 1'b1;
				else 
					begin
						// Si A10=1 durante el comando WRITE, auto-precharge está activo
						// y no necesitamos enviar un comando de precharge explícito
						if (addr_reg[10]) 
							begin
								next_state = IDLE;			// Volver a IDLE directamente.
								next_command = CMD_NOP;		// The No Operation Command should be used in cases when the SDRAM is in a idle or a wait state.
							end
						else 
							begin
								next_state = WRITE_PRECHARGE;
								next_command = CMD_PRECHARGE_BANK;
							end
					end
			end

		WRITE_PRECHARGE: 
			begin
				// Precargar el banco específico.
				// Comando CMD_PRECHARGE_BANK emitido en el estado anterior.

				next_state = WRITE_WAIT_TRP;
				next_delay_counter = TRP_CYCLES;  // Esperar tRP.
			end

		WRITE_WAIT_TRP: 
			begin
				if (delay_counter != 0)
					next_delay_counter = delay_counter - 1'b1;
				else 
					begin
						next_state = IDLE;			// Volver a IDLE.
						next_command = CMD_NOP;		// The No Operation Command should be used in cases when the SDRAM is in a idle or a wait state.
					end
			end

		
		default: 
			begin
				next_state = IDLE;
				next_command = CMD_NOP;  // The No Operation Command should be used in cases when the SDRAM is in a idle or a wait state.
			end

	endcase
end

always @* 
begin
	// Handle logic for sending addresses to SDRAM based on the current state.
	// ------------------------------------------------------------------------
	bank_addr_reg = 2'b00;
	addr_reg = {SDRAM_ADDR_WIDTH{1'b0}};
	
	if (/*state == READ_ACT || */state == WRITE_BANK_ACTIVATE)  // TODO: Descomentar esto cuando implemente la lectura y ver si WRITE_ACT en el código original corresponde a WRITE_BANK_ACTIVATE ahora.
		begin
			bank_addr_reg = soc_side_addr[SOC_SIDE_ADDR_WIDTH - 1: SOC_SIDE_ADDR_WIDTH - BANK_WIDTH];
			addr_reg = soc_side_addr[SOC_SIDE_ADDR_WIDTH - (BANK_WIDTH + 1): SOC_SIDE_ADDR_WIDTH - (BANK_WIDTH + ROW_WIDTH)];
		end
	else if (/*state == READ_CAS || */state == WRITE_CAS)  // TODO: Descomentar esto cuando implemente la lectura.
		begin
			bank_addr_reg = soc_side_addr[SOC_SIDE_ADDR_WIDTH - 1: SOC_SIDE_ADDR_WIDTH - BANK_WIDTH];

			/*
									 BANK	 ROW	 COL
				SOC_SIDE_ADDR_WIDTH   2	  +  12   +   9   = 23 
				SDRAM_ADDR_WIDTH 13
			*/
			addr_reg = { {(SDRAM_ADDR_WIDTH - 11){1'b0}},		/* 0s */
						1'b1,									/* 1 (A10 is always for auto precharge) */
						{(10 - COL_WIDTH){1'b0}},				/* 0s */
						soc_side_addr[COL_WIDTH - 1: 0]			/* column address */
					};
		end
	else if (state == INIT_MRS)
		begin
			/*
				Configuración del registro de modo (MRS) durante inicialización.

				10'b1000110000:
					bits 0-2: 000 -> Burst Length de 1 (sin ráfagas).
					bit 3: 0 -> Tipo de burst secuencial.
					bits 4-6: 011 -> CAS Latency de 3.
					bit 7-8: 00 -> Modo de operación estándar.
					bit 9: 1 -> Ajuste de "Burst Read/Single Write".
			*/
			addr_reg = { {(SDRAM_ADDR_WIDTH - 10){1'b0}}, 
						10'b1000110000
					};
		end
	// ------------------------------------------------------------------------


	// Set write mask signals based on the current state.
	// -----------------------------------------
	if (state == INIT_PAUSE || state == INIT_PRECHARGE_ALL || state == INIT_WAIT_TRP || state == INIT_AUTO_REFRESH || 
		state == INIT_WAIT_TRFC || state == INIT_MRS || state == INIT_WAIT_TMRD)

		// Deshabilita buffers during initialization.
		sdram_side_wr_mask_reg = 4'b1111;  // Mask all Bytes (DQM high) so that SDRAM drives buffers to HIGH-Z.

	else if (is_read_write_state)  // If this is a read or write state.
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
		sdram_side_wr_mask_reg = 4'b1111;	// If mode is not read/write mask all Bytes (DQM high) so that SDRAM drives buffers to HIGH-Z.
	// -----------------------------------------
end


endmodule

