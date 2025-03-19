/*
	Simple SDRAM controller for two Winbond W9812G6KH-5I SDRAM chips in parallel.
	It is designed to use two of these 2M word x 4 bank x 16 bits chips in parallel to 
	achieve a total of 32MB (4M words x 32 bits).

	Default options
		CLK_FREQUENCY_MHZ = 100MHz
		CAS 3

	Very simple CPU interface

		- No burst support.
		
		- soc_side_rst_n_pin: Start init SDRAM process.

		- soc_side_busy_pin: Indicate that the controller is busy during read/write operations 
		  and during refresh cycles. The CPU must wait for soc_side_busy_pin to be low before 
		  sending commands to the SDRAM.
		
		- soc_side_addr_pin: Address for read/write 32 bits of data.

		- soc_side_wr_mask_pin: 0000 --> No write/read memory.
							1111 --> Write 32 bits.
							1100 --> Write upper 16 bits.
							0011 --> Write lower 16 bits.
							0001 --> Write Byte 0.
							0010 --> Write Byte 1.
							0100 --> Write Byte 2.
							1000 --> Write Byte 3.

		- soc_side_wr_data_pin: Data for writing, latched in when soc_side_wr_en_pin is high.

		- soc_side_wr_en_pin: On clk posedge soc_side_addr_pin and soc_side_wr_data_pin will be 
		  latched in, after a few clocks data will be written to the SDRAM.

		- soc_side_rd_data_pin: Data for reading, comes available a few clocks after 
		  soc_side_rd_en_pin and soc_side_addr_pin are presented on the bus.

		- soc_side_rd_en_pin: On clk posedge soc_side_addr_pin will be latched in, after a 
		  few clocks data will be available on the soc_side_rd_data_pin port.

		- soc_side_rd_ready_pin: This signal is used to notify the CPU when the read data is 
		  available on the soc_side_rd_data_pin bus.
 */


module sdram_controller # (
	/* Address parameters */
	parameter ROW_WIDTH = 12,
	parameter COL_WIDTH = 9,
	parameter BANK_ADDR_WIDTH = 2,		// 2 bits wide for 4 banks.
	
	parameter SOC_SIDE_ADDR_WIDTH = BANK_ADDR_WIDTH + ROW_WIDTH + COL_WIDTH,
	parameter SDRAM_ADDR_WIDTH = (ROW_WIDTH > COL_WIDTH) ? ROW_WIDTH : COL_WIDTH,

	/* Timing parameters */
	parameter CLK_FREQUENCY_MHZ = 100,	// Clock frequency [MHz].
	parameter REFRESH_TIME_MS = 64,		// Refresh period [ms].
	parameter REFRESH_COUNT = 4096		// Number of refresh cycles per refresh period.
) (
	input wire clk,

	/* SOC interface */
	input wire soc_side_rst_n_pin,
	output reg soc_side_busy_pin,

	// Address
	input wire [SOC_SIDE_ADDR_WIDTH - 1: 0] soc_side_addr_pin,
	input wire [3:0] soc_side_wr_mask_pin,	// These are the mem_wstrb signals in the PicoRV32 SOC.

	// Write
	input wire [31:0] soc_side_wr_data_pin,
	input wire soc_side_wr_en_pin,

	// Read
	output wire [31:0] soc_side_rd_data_pin,
	input wire soc_side_rd_en_pin,
	output wire soc_side_rd_ready_pin,

	/* SDRAM side */
	output wire [SDRAM_ADDR_WIDTH - 1: 0] ram_side_addr_pin,		// SDRAM chips 0 and 1, A0 to A11 pins.
	output wire [BANK_ADDR_WIDTH - 1: 0] ram_side_bank_addr_pin,	// SDRAM chips 0 and 1, BS0 and BS1 pins.

	output wire ram_side_chip0_ldqm_pin,		// SDRAM chip 0, LDQM pin.
	output wire ram_side_chip0_udqm_pin,		// SDRAM chip 0, UDQM pin.
	inout wire [15:0] ram_side_chip0_data_pin,	// SDRAM chip 0, DQ0 to DQ15 pins.

	output wire ram_side_chip1_ldqm_pin,		// SDRAM chip 1, LDQM pin.
	output wire ram_side_chip1_udqm_pin,		// SDRAM chip 1, UDQM pin.
	inout wire [15:0] ram_side_chip1_data_pin,	// SDRAM chip 1, DQ0 to DQ15 pins.

	output wire ram_side_cs_n_pin,		// SDRAM chips 0 and 1, CS pin.
	output wire ram_side_ras_n_pin,		// SDRAM chips 0 and 1, RAS pin.
	output wire ram_side_cas_n_pin,		// SDRAM chips 0 and 1, CAS pin.
	output wire ram_side_wr_en_pin,		// SDRAM chips 0 and 1, WE pin.

	/*
		During normal access mode, CKE must be held high enabling the clock.
		Este pin solo es utilizado en Power Down mode, Suspend mode or Self 
		Refresh mode. Este controlador no implementa estos modos por lo que
		se mantiene siempre en high.
	*/
	output wire ram_side_ck_en_pin		// SDRAM chips 0 and 1, CKE pin.
);


//-------------------------------
// Parámetros
//-------------------------------

localparam CYCLES_BETWEEN_REFRESH = (CLK_FREQUENCY_MHZ * 1_000 * REFRESH_TIME_MS) / REFRESH_COUNT;

localparam CAS_LATENCY = 3;

// ------------------------------------------------------------------------
// Times: Table "9.5 AC Characteristics and Operating Condition", page 15.
// ------------------------------------------------------------------------

// Tiempos para la inicialización de la SDRAM.
// -------------------------------------------
localparam INIT_PAUSE_US = 300;  // Pausa inicial de 200us + margen de seguridad.
localparam INIT_PAUSE_CYCLES = INIT_PAUSE_US * CLK_FREQUENCY_MHZ;

localparam TRP_NS = 18;  // Precharge to Active Command Period, tRP: 15ns + margen de seguridad. Tiempo de espera luego de emitir el comando precharge all banks.
localparam TRP_CYCLES = ((TRP_NS * CLK_FREQUENCY_MHZ) + 999) / 1000;  // Cantidad de ciclos de espera para cumplir con el tiempo tRP. Redondeo hacia arriba.

localparam INIT_REFRESH_COUNT = 8;  // Número de ciclos de refresco consecutivos requeridos durante la inicialización.

localparam TRC_NS = 65;  // Ref/Active to Ref/Active Command Period, tRC: 55-65ns. Tiempo de espera luego de emitir el comando auto refresh.
localparam TRC_CYCLES = ((TRC_NS * CLK_FREQUENCY_MHZ) + 999) / 1000;  // Cantidad de ciclos de espera para cumplir con el tiempo tRC. Redondeo hacia arriba.

localparam TRSC_CYCLES = 2;  // Mode Register Set Cycle Time, tRSC: 2 clock cycles. Tiempo de espera luego de emitir el comando CMD_MRS Mode Register Set (MRS).

// Tiempos para escritura.
// -----------------------
localparam TRCD_NS = 18;  // Active to Read/Write Command Delay Time, tRCD: 15ns + margen de seguridad. Tiempo de espera luego de emitir el comando CMD_BANK_ACTIVATE.
localparam TRCD_CYCLES = ((TRCD_NS * CLK_FREQUENCY_MHZ) + 999) / 1000;  // Cantidad de ciclos de espera para cumplir con el tiempo tRCD. Redondeo hacia arriba.

localparam TWR_CYCLES = 2;  // Write Recovery Time, tWR: 2 clock cycles. Tiempo de espera luego de emitir el comando CMD_WRITE.
// ------------------------------------------------------------------------


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
localparam INIT_WAIT_TRC			= 5'b00111;  // Esperar tRC después de cada uno de los 8 ciclos de auto refresh.
localparam INIT_MRS					= 5'b00101;  // Mode Register Set (MRS).
localparam INIT_WAIT_TRSC			= 5'b00100;  // Esperar tRSC nanosegundos después de MRS.

// States for the Auto refresh.
localparam REFRESH_PRECHARGE_ALL	= 5'b01100;  // Precargar todos los bancos antes del refresco.
localparam REFRESH_WAIT_TRP			= 5'b01101;  // Esperar tRP nanosegundos después de precharge all.
localparam REFRESH_AUTO_REFRESH		= 5'b01111;  // Emitir comando Auto Refresh.
localparam REFRESH_WAIT_TRC			= 5'b01110;  // Esperar tRC después de Auto Refresh.

// Estados para operación de escritura.
localparam WRITE_BANK_ACTIVATE		= 5'b01010;  // Activar fila/banco (Bank Activate Command).
localparam WRITE_WAIT_TRCD			= 5'b01011;  // NOP después de activación (esperar tRCD).
localparam WRITE_CAS				= 5'b01001;  // Comando de escritura y datos (Write Command).
localparam WRITE_WAIT_TWR			= 5'b01000;  // NOP después de escritura (esperar tWR).
localparam WRITE_PRECHARGE			= 5'b11000;  // Precargar banco (si no se usa auto-precharge).
localparam WRITE_WAIT_TRP			= 5'b11001;  // NOP después de precharge (esperar tRP).

// Estados para operación de lectura.
localparam READ_BANK_ACTIVATE		= 5'b11011;  // Activar fila/banco (Bank Activate Command).
localparam READ_WAIT_TRCD			= 5'b11010;  // NOP después de activación (esperar tRCD).
localparam READ_CAS					= 5'b11110;  // Comando de lectura (Read Command).
localparam READ_WAIT_CAS_LATENCY	= 5'b11111;  // Esperar CAS Latency (3 ciclos).
localparam READ_DATA				= 5'b11101;  // Capturar datos de los pines DQ.
localparam READ_PRECHARGE			= 5'b11100;  // Precargar banco (si no se usa auto-precharge).
localparam READ_WAIT_TRP			= 5'b10100;  // NOP después de precharge (esperar tRP).


//-------------------------------
// Definición de Comandos SDRAM
//-------------------------------

localparam CMD_PRECHARGE_ALL	= 8'b10010001;  // Precharge All.
localparam CMD_AUTO_REFRESH		= 8'b10001000;  // Auto Refresh.
localparam CMD_NOP				= 8'b10111000;  // No Operation.
localparam CMD_MRS				= 8'b1000000x;  // Mode Register Set (MRS).

// Comandos para operaciones de lectura/escritura.
localparam CMD_BANK_ACTIVATE	= 8'b10011xxx;  // Activar banco/fila (x = bank address bits).
localparam CMD_WRITE			= 8'b10100xx1;  // Comando de escritura (x = bank address bits).
localparam CMD_PRECHARGE_BANK	= 8'b10010x00;  // Precargar banco específico (x = a10 para auto-precharge).
localparam CMD_READ				= 8'b10101xx1;  // CS=L, RAS=H, CAS=L, WE=H


//-------------------------------
// Internal Registers and Wires
//-------------------------------

// Internal state machine signals
reg [4:0] state, next_state;
reg [14:0] delay_counter, next_delay_counter;				// Para retardos de tiempo entre comandos.
reg [3:0] init_refresh_counter, next_init_refresh_counter;	// Cuenta los 8 ciclos de auto-refresh necesarios para la inicialización.
reg [15:0] refresh_counter, next_refresh_counter;			// Contador para seguimiento del momento del próximo refresco.
reg [2:0] cas_counter, next_cas_counter;
reg [7:0] command, next_command;  // Comando SDRAM.

// Internal registers for CPU interface.
reg [31:0] wr_data_reg;
reg [31:0] rd_data_reg;
reg read_ready_reg;

// Internal registers for SDRAM address generation.
reg [SDRAM_ADDR_WIDTH - 1: 0] addr_reg;
reg [BANK_ADDR_WIDTH - 1: 0] bank_addr_reg;

reg [3:0] sdram_side_wr_mask_reg;  // Internal registers for Byte mask signals.

wire in_initialization_cycle;
wire in_refresh_cycle;
wire in_read_write_cycle;


//-------------------------------
// Assignments
//-------------------------------

// Assigns command bits to outputs.
assign {ram_side_ck_en_pin, ram_side_cs_n_pin, ram_side_ras_n_pin, ram_side_cas_n_pin, ram_side_wr_en_pin} = command[7:3];
assign ram_side_bank_addr_pin = in_read_write_cycle ? bank_addr_reg : command[2:1];
assign ram_side_addr_pin = (in_read_write_cycle || state == INIT_MRS) ? addr_reg : { {(SDRAM_ADDR_WIDTH - 11){1'b0}}, command[0], 10'd0 };

assign soc_side_rd_data_pin = rd_data_reg;
assign soc_side_rd_ready_pin = read_ready_reg;

assign ram_side_chip0_ldqm_pin = sdram_side_wr_mask_reg[0];
assign ram_side_chip0_udqm_pin = sdram_side_wr_mask_reg[1];
assign ram_side_chip1_ldqm_pin = sdram_side_wr_mask_reg[2];
assign ram_side_chip1_udqm_pin = sdram_side_wr_mask_reg[3];

assign ram_side_chip0_data_pin = (state == WRITE_CAS) ? wr_data_reg[15:0] : 16'bz;
assign ram_side_chip1_data_pin = (state == WRITE_CAS) ? wr_data_reg[31:16] : 16'bz;

// Signal to detect SDRAM initialization cycle states.
assign in_initialization_cycle = (state == INIT_PAUSE) || (state == INIT_PRECHARGE_ALL) || 
								(state == INIT_WAIT_TRP) || (state == INIT_AUTO_REFRESH) || 
								(state == INIT_WAIT_TRC) || (state == INIT_MRS) || 
								(state == INIT_WAIT_TRSC);

// Signal to detect refresh cycle states.
assign in_refresh_cycle = (state == REFRESH_PRECHARGE_ALL) || (state == REFRESH_WAIT_TRP) || 
						(state == REFRESH_AUTO_REFRESH) || (state == REFRESH_WAIT_TRC);

// Signal to detect read or write cycle states.
assign in_read_write_cycle = (state == WRITE_BANK_ACTIVATE) || (state == WRITE_WAIT_TRCD) || 
							(state == WRITE_CAS) || (state == WRITE_WAIT_TWR) || 
							(state == WRITE_PRECHARGE) || (state == WRITE_WAIT_TRP);  // TODO: Agregar estados de lectura de memoria.


//-------------------------------
// Lógica Secuencial
//-------------------------------

always @(posedge clk) 
begin
	if (~soc_side_rst_n_pin) 
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
			soc_side_busy_pin <= 1'b1;  // El controlador está ocupado durante inicialización.
		end
	else 
		begin
			state <= next_state;

			delay_counter <= next_delay_counter;
			init_refresh_counter <= next_init_refresh_counter;
			refresh_counter <= next_refresh_counter;

			command <= next_command;
			
			if (soc_side_wr_en_pin)
				wr_data_reg <= soc_side_wr_data_pin;  // Update write data.

			// Update read data and read ready signal.
			// TODO: Descomentar esto cuando implemente la lectura.
			/*if (state == READ_READ)
				begin
					rd_data_reg <= {ram_side_chip1_data_pin, ram_side_chip0_data_pin};
					read_ready_reg <= 1'b1;
				end
			else
				read_ready_reg <= 1'b0;*/

			/*
				Indicar que el controlador está ocupado durante el ciclo de inicialización de la SDRAM, 
				los ciclos de refresco y los ciclos de lectura o escritura.
			*/
			soc_side_busy_pin <= in_initialization_cycle || in_refresh_cycle || in_read_write_cycle;
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

		// ---- Initialization Sequence ----
		/*
			Initialization Sequence
			------------------------

			The implemented sequence follows the requirements of the W9812G6KH-5I datasheet.

			1. INIT_PAUSE: Wait 200us after power-up.
				- During this pause, DQM and CKE are kept high to prevent data contention.

			2. INIT_PRECHARGE_ALL: Issues the precharge all banks command.
				- This command prepares all banks for subsequent operations.

			3. INIT_WAIT_TRP: Wait tRP nanoseconds after precharge.
				- Time required for the precharge to complete internally.

			4. INIT_AUTO_REFRESH + INIT_WAIT_TRC: Executes 8 Auto Refresh cycles.
				- Each cycle issues an AUTO_REFRESH command and waits tRC nanoseconds.
				- All 8 cycles are specifically required by the datasheet for initialization.

			5. INIT_MRS: Mode Register configuration.
				- Configures: Burst Length=1, Sequential type, CAS Latency=3, Standard operation.

			6. INIT_WAIT_TRSC: Wait tRSC before transitioning to IDLE.
				- Time required for the register configuration to complete.
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
					la SDRAM espera tRC con comando NOP.
				*/

				next_state = INIT_WAIT_TRC;	// Esperar tiempo de ciclo de refresco tRC luego de enviar el comando auto refresh.
				next_command = CMD_NOP;			// Emitir comando CMD_NOP durante los tiempos de espera.
				
				next_delay_counter = TRC_CYCLES;  // Esperar tRC nanosegundos.
			end
		
		INIT_WAIT_TRC: 
			begin
				if (delay_counter != 0)  // Esperar tRC después de emitir el comando CMD_AUTO_REFRESH.
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

				next_state = INIT_WAIT_TRSC;
				next_command = CMD_NOP;  // Emitir comando CMD_NOP durante los tiempos de espera.

				next_delay_counter = TRSC_CYCLES;  // Esperar tRSC (Mode Register Set Cycle Time) en el siguiente estado.
			end
		
		INIT_WAIT_TRSC: 
			begin
				// Esperar después del Mode Register Set
				if (delay_counter != 0)
					next_delay_counter = delay_counter - 1'b1;
				else 
					next_state = IDLE;			// Inicialización completada.
					next_command = CMD_NOP;		// The No Operation Command should be used in cases when the SDRAM is in a idle or a wait state.
			end
		

		// ---- IDLE State ----
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
				else if (soc_side_wr_en_pin) 
					begin
						// Inicia secuencia de escritura.
						next_state = WRITE_BANK_ACTIVATE;
						next_command = CMD_BANK_ACTIVATE;
					end
				else if (soc_side_rd_en_pin) 
					begin
						// Inicia secuencia de lectura.
						/*next_state = READ_ACT;
						next_command = CMD_BACT;*/
					end
			end


		// ---- Auto Refresh Cycle ----
		/*
			Auto Refresh Cycle
			-------------------

			The automatic refresh cycle is activated periodically to maintain data in SDRAM:

			1. REFRESH_PRECHARGE_ALL: Precharges all banks before refresh.
				- Necessary because the AUTO_REFRESH command requires all banks to be inactive.

			2. REFRESH_WAIT_TRP: Wait tRP nanoseconds after precharge.
				- Time required for the precharge to complete internally.

			3. REFRESH_AUTO_REFRESH: Issues the AUTO_REFRESH command.
				- Refreshes all rows in all banks simultaneously.

			4. REFRESH_WAIT_TRC: Wait tRC nanoseconds after AUTO_REFRESH.
				- Time required for the refresh cycle to complete internally.
				- Upon completion, the refresh counter is reset to start a new period.

			The refresh frequency is calculated to ensure all 4096 rows are refreshed 
			within the 64ms retention period specified by the datasheet.
		*/
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
					la SDRAM espera tRC con comando NOP.
				*/
				
				next_state = REFRESH_WAIT_TRC;
				next_command = CMD_NOP;  // Emitir comando CMD_NOP durante los tiempos de espera.

				next_delay_counter = TRC_CYCLES;  // Esperar tRC nanosegundos.
			end

		REFRESH_WAIT_TRC:
			begin
				// Esperar tRC después de emitir el comando CMD_AUTO_REFRESH.

				if (delay_counter != 0)
					next_delay_counter = delay_counter - 1'b1;
				else 
					begin
						next_state = IDLE;			// Volver al estado IDLE.
						next_command = CMD_NOP;		// The No Operation Command should be used in cases when the SDRAM is in a idle or a wait state.

						next_refresh_counter = 0;  // Resetear el contador de refresco.
					end
			end


		// ---- Write Cycle ----
		/*
			Write Cycle
			------------

			The write cycle follows the required sequence to write data to SDRAM:

			1. WRITE_BANK_ACTIVATE: Activates the specific bank and row.
				- Selects the memory row containing the address to write.

			2. WRITE_WAIT_TRCD: Wait tRCD nanoseconds after activation.
				- Time required between activation and column command (Row to Column Delay).

			3. WRITE_CAS: Issues the write command and sends data.
				- Column address and data are presented simultaneously.
				- A10 is configured for auto-precharge if enabled.

			4. WRITE_WAIT_TWR: Wait tWR nanoseconds after write.
				- Time required for data to be fully written to the memory array.

			5. WRITE_PRECHARGE/WRITE_WAIT_TRP (optional if auto-precharge not used):
				- Precharges the specific bank and waits tRP before new operations.
		*/
		WRITE_BANK_ACTIVATE: 
			begin
				// Activar el banco y fila especificados.
				// Comando CMD_BANK_ACTIVATE emitido en el estado anterior IDLE.

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
						if (addr_reg[10]) 
							begin
								/*
									Si A10=1 durante el comando WRITE, auto-precharge está activo
									y no necesitamos enviar un comando de precharge explícito.
									Actualmente A10=1 siempre pero se mantienen los siguientes dos
									estados para futuras mejoras del controlador.
								*/

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

		// ---- Read Cycle ----

		
		default: 
			begin
				next_state = IDLE;
				next_command = CMD_NOP;  // The No Operation Command should be used in cases when the SDRAM is in a idle or a wait state.
			end

	endcase
end

always @* 
begin
	// Send addresses to SDRAM based on the current state.
	// -----------------------------------------------------
	bank_addr_reg = 2'b00;
	addr_reg = {SDRAM_ADDR_WIDTH{1'b0}};
	
	if (/*state == READ_ACT || */state == WRITE_BANK_ACTIVATE)  // TODO: Descomentar esto cuando implemente la lectura.
		begin
			bank_addr_reg = soc_side_addr_pin[SOC_SIDE_ADDR_WIDTH - 1: SOC_SIDE_ADDR_WIDTH - BANK_ADDR_WIDTH];
			addr_reg = soc_side_addr_pin[SOC_SIDE_ADDR_WIDTH - (BANK_ADDR_WIDTH + 1): SOC_SIDE_ADDR_WIDTH - (BANK_ADDR_WIDTH + ROW_WIDTH)];
		end
	else if (/*state == READ_CAS || */state == WRITE_CAS)  // TODO: Descomentar esto cuando implemente la lectura.
		begin
			bank_addr_reg = soc_side_addr_pin[SOC_SIDE_ADDR_WIDTH - 1: SOC_SIDE_ADDR_WIDTH - BANK_ADDR_WIDTH];

			/*
									 BANK	 ROW	 COL
				SOC_SIDE_ADDR_WIDTH   2	  +  12   +   9   = 23 
				SDRAM_ADDR_WIDTH 13
			*/
			addr_reg = { {(SDRAM_ADDR_WIDTH - 11){1'b0}},		/* 0s */
						1'b1,									/* 1 (A10 is always for auto precharge) */
						{(10 - COL_WIDTH){1'b0}},				/* 0s */
						soc_side_addr_pin[COL_WIDTH - 1: 0]			/* column address */
					};
		end
	else if (state == INIT_MRS)
		begin
			/*
				Configuración del registro de modo (MRS) durante inicialización:

					Burst Length de 1 (sin ráfagas)		-> bits 0-2	= 000
					Tipo de burst secuencial			-> bit 3	= 0
					CAS Latency de 3					-> bits 4-6	= 011
					Modo de operación estándar			-> bits 7-8	= 00
					Ajuste de "Burst Read/Single Write"	-> bit 9	= 1

					= 10'b 1 00 011 0 000
			*/
			addr_reg = { {(SDRAM_ADDR_WIDTH - 10){1'b0}}, 10'b1000110000 };
		end
	// -----------------------------------------------------


	// Set the writing mask based on the current state.
	// -------------------------------------------------
	if (in_initialization_cycle)
		// Deshabilita buffers during initialization. Mask all bits high so that the SDRAM drives the input/output buffers to HIGH-Z.
		sdram_side_wr_mask_reg = 4'b1111;
	else if (in_read_write_cycle)
		/*
			PicoRV32 SOC, mem_wstrb signals:
				soc_side_wr_mask_pin = 0000 --> No write. Read memory.
				soc_side_wr_mask_pin = 1111 --> Write 32 bits.
				soc_side_wr_mask_pin = 1100 --> Write upper 16 bits.
				soc_side_wr_mask_pin = 0011 --> Write lower 16 bits.
				soc_side_wr_mask_pin = 0001 --> Write Byte 0.
				soc_side_wr_mask_pin = 0010 --> Write Byte 1.
				soc_side_wr_mask_pin = 0100 --> Write Byte 2.
				soc_side_wr_mask_pin = 1000 --> Write Byte 3.

			The mem_wstrb signals of the SoC are inverted because the LDQM and UDQM pins of the 
			SDRAM set the input/output buffers to HIGH-Z with 1 and enable them with 0.
		*/
		if (soc_side_wr_mask_pin)								// If the soc wants to write in the memory (soc_side_wr_mask_pin diferent from 0000).
			sdram_side_wr_mask_reg = ~soc_side_wr_mask_pin;		// Use the 4-bit write mask from the soc (mem_wstrb signals).
		else 
			sdram_side_wr_mask_reg = 4'b0000;	// If the SOC needs to read from memory, enable buffers.
	else
		// If the state is not read or write, mask all bits high so that the SDRAM drives the input/output buffers to HIGH-Z.
		sdram_side_wr_mask_reg = 4'b1111;
	// -------------------------------------------------
end


endmodule

