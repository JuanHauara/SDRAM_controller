/*
	Simple SDRAM controller for two Winbond W9812G6KH-5I SDRAM chips in parallel.
	It is designed to use two of these 2M word x 4 bank x 16 bits (8M words x 16 bits) chips in parallel to 
	achieve a total of 8M x 32 bit words = 32MB RAM.

	Default options
		CLK_FREQUENCY_MHZ = 85MHz
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

			// TODO: Corregir esta documentación sobre cuando se muestrea la dirección de memoria.

			- soc_side_wr_en_port: On clk posedge, if soc_side_wr_en_port is high soc_side_wr_data_port and 
			soc_side_addr_port will be latched in, after a few clocks data will be written to the SDRAM.

			- soc_side_ready_port: This signal is used to notify the CPU when read data is available on the 
			soc_side_rd_data_port bus and also when a data write to memory has finished.

		Read data:
			- soc_side_rd_data_port: Data for reading, comes available a few clocks after 
			soc_side_rd_en_port and soc_side_addr_port are presented on the bus.

			- soc_side_rd_en_port: On clk posedge soc_side_addr_port will be latched in, after a 
			few clocks data will be available on the soc_side_rd_data_port port.
 */


module sdram_controller # (
	/* Timing parameters */
	parameter CLK_FREQUENCY_MHZ = 85,	// Clock frequency [MHz].
	parameter REFRESH_TIME_MS = 64,		// Refresh period [ms].
	parameter REFRESH_COUNT = 4096,		// Number of refresh cycles per refresh period.

	/* Address parameters */
	parameter ROW_WIDTH = 12,
	parameter COL_WIDTH = 9,
	parameter BANK_ADDR_WIDTH = 2,		// 2 bits wide for 4 banks.
	
	parameter SOC_SIDE_ADDR_WIDTH = ROW_WIDTH + COL_WIDTH + BANK_ADDR_WIDTH,
	parameter SDRAM_ADDR_WIDTH = (ROW_WIDTH > COL_WIDTH)? ROW_WIDTH : COL_WIDTH
) (
	input wire clk,
	input wire reset_n_port,

	/* SOC interface */
	output wire soc_side_busy_port,
	output wire soc_side_ready_port,

	// Address.
	input wire [SOC_SIDE_ADDR_WIDTH - 1: 0] soc_side_addr_port,  // 23 bits bus to address 8 million 32 bit words = 32MB RAM.

	// Read data.
	output wire [31:0] soc_side_rd_data_port,
	input wire soc_side_rd_en_port,

	// Write data.
	input wire [31:0] soc_side_wr_data_port,
	input wire [3:0] soc_side_wr_mask_port,  // These are the mem_wstrb signals in the PicoRV32 SOC.
	input wire soc_side_wr_en_port,

	/* SDRAM interface */
	output wire [SDRAM_ADDR_WIDTH - 1: 0] ram_side_addr_port,		// SDRAM chips 0 and 1, A0 to A11 pins.  TODO: Ver que el ancho de este bus sea correcto.
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
		During normal access mode, CKE must be held high enabling the clock.
		Este pin solo es utilizado en Power Down mode, Suspend mode or Self 
		Refresh mode. Este controlador no implementa estos modos por lo que
		CKE se mantiene siempre en high.
	*/
	output wire ram_side_ck_en_port		// SDRAM chips 0 and 1, CKE pin.
);


//-------------------------------
// Parámetros
//-------------------------------

localparam CYCLES_BETWEEN_REFRESH = (CLK_FREQUENCY_MHZ * 1_000 * REFRESH_TIME_MS) / REFRESH_COUNT;

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

// Tiempos de escritura y lectura.
// -------------------------------
localparam TRCD_NS = 18;  // Active to Read/Write Command Delay Time, tRCD: 15ns + margen de seguridad. Tiempo de espera luego de emitir el comando CMD_BANK_ACTIVATE.
localparam TRCD_CYCLES = ((TRCD_NS * CLK_FREQUENCY_MHZ) + 999) / 1000;  // Cantidad de ciclos de espera para cumplir con el tiempo tRCD. Redondeo hacia arriba.

localparam TWR_CYCLES = 2;  // Write Recovery Time, tWR: 2 clock cycles. Tiempo de espera luego de emitir el comando CMD_WRITE.

localparam CAS_LATENCY = 3;
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

// SDRAM initialization.
localparam INIT						= 5'b00000;  // Initial state.
localparam INIT_PAUSE				= 5'b00001;  // Pausa inicial de al menos 200us.
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

// IDLE state
localparam IDLE						= 5'b10101;


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

// Internal state machine registers.
reg [4:0] current_state, next_state;
reg [7:0] command, next_command;  // Comando SDRAM.

// Contador para retardos de tiempo.
reg [14:0] delay_counter, delay_cycles;

// Cuenta los 8 ciclos de auto-refresh necesarios para la inicialización.
reg [3:0] init_counter;
reg reset_init_counter;
reg init_counter_count_one_cycle;

// Contador para seguimiento del momento del próximo refresco.
reg [15:0] refresh_counter;
reg reset_refresh_counter;

// Internal registers for SDRAM address generation.
reg [SDRAM_ADDR_WIDTH - 1: 0] ram_addr_reg;
reg [BANK_ADDR_WIDTH - 1: 0] ram_bank_addr_reg;

reg busy_signal;
reg ready_signal;
reg [31:0] rd_data_reg;
reg [31:0] wr_data_reg;
reg [3:0] sdram_side_wr_mask_reg;  // Internal registers for Byte mask signals.

wire in_initialization_cycle;
wire in_write_cycle;
wire in_read_cycle;


//-------------------------------
// Assignments
//-------------------------------

// Assigns command bits to outputs.
assign {ram_side_ck_en_port, ram_side_cs_n_port, ram_side_ras_n_port, ram_side_cas_n_port, ram_side_wr_en_port} = command[7:3];
assign ram_side_bank_addr_port = (in_write_cycle || in_read_cycle)? ram_bank_addr_reg : command[2:1];
assign ram_side_addr_port = (in_write_cycle || in_read_cycle || current_state == INIT_MRS)? ram_addr_reg : { {(SDRAM_ADDR_WIDTH - 11){1'b0}}, command[0], 10'd0 };

assign soc_side_busy_port = busy_signal;
assign soc_side_ready_port = ready_signal;

// Read data: From ram_side_chip0_data_port --> rd_data_reg --> soc_side_rd_data_port.
assign soc_side_rd_data_port = rd_data_reg;

// Write data: From soc_side_wr_data_port --> wr_data_reg --> ram_side_chip0_data_port.
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
						(current_state == WRITE_CAS) || (current_state == WRITE_WAIT_TWR) || 
						(current_state == WRITE_PRECHARGE) || (current_state == WRITE_WAIT_TRP);

// Signal to detect read cycle states.
assign in_read_cycle = (current_state == READ_BANK_ACTIVATE) || (current_state == READ_WAIT_TRCD)  ||
					(current_state == READ_CAS) || (current_state == READ_WAIT_CAS_LATENCY)  ||
					(current_state == READ_DATA) || (current_state == READ_PRECHARGE)  ||
					(current_state == READ_WAIT_TRP);


//-------------------------------
// Bloque Secuencial para el registro de estado
//-------------------------------
always @(posedge clk) 
begin
	if (~reset_n_port)  // Reset síncrono.
		begin
			current_state <= INIT;
			command <= CMD_NOP;
			
			// Data.
			wr_data_reg <= 32'b0;
			rd_data_reg <= 32'b0;
		end
	else 
		begin
			// Update state and command.
			// ----------------------------
			current_state <= next_state;
			command <= next_command;
			
			// Update write data register.
			// ----------------------------
			// Write data: From soc_side_wr_data_port --> wr_data_reg --> ram_side_chip0_data_port.
			if (soc_side_wr_en_port)
				wr_data_reg <= soc_side_wr_data_port;  // Update the data to be written from the SOC.

			// Update read data register.
			// ----------------------------
			// Read data: From ram_side_chip0_data_port --> rd_data_reg --> soc_side_rd_data_port.
			if (current_state == READ_DATA)
				rd_data_reg <= {ram_side_chip1_data_port, ram_side_chip0_data_port};  // Updates the data that the SOC will read.
		end
end

//-------------------------------
// Contador para retardos de tiempo.
//-------------------------------
always @(posedge clk) 
begin
	if (~reset_n_port)
		delay_counter <= 0;
	else if (delay_cycles != 0)
		delay_counter <= delay_cycles;
	else if (delay_counter != 0)
		delay_counter <= delay_counter - 1'b1;
end

//-------------------------------
// Contador para los 8 ciclos de refresco al inicio.
//-------------------------------
always @(posedge clk) 
begin
	if (reset_init_counter)
		init_counter <= INIT_REFRESH_COUNT - 1;
	else if (init_counter_count_one_cycle)
		init_counter <= init_counter - 1'b1;
end

//-------------------------------
// Contador para ciclo de refresco.
//-------------------------------
always @(posedge clk) 
begin
	if (~reset_n_port || reset_refresh_counter)
		refresh_counter <= CYCLES_BETWEEN_REFRESH - 1;
	else if (refresh_counter != 0)
		refresh_counter <= refresh_counter - 1'b1;
end


//-------------------------------
// Bloque Combinacional.
//-------------------------------
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
	// Valores por defecto para evitar latches no deseados.
	next_state = current_state;
	next_command = CMD_NOP;

	delay_cycles = 0;
	reset_init_counter = 1'b0;
	init_counter_count_one_cycle = 1'b0;
	reset_refresh_counter = 1'b0;

	/*
		En este caso aunque las señales se declaren como registros, el compilador las asignará 
		inmediatamente en cada estado sin tener que esperar al siguiente estado. 
		Esto es, las señales no estaran retrasadas un ciclo de reloj.
	*/
	busy_signal = 1'b1;  // Controlador ocupado por defecto, solo está desocupado cuando la máquina de estados está en IDLE.
	ready_signal = 1'b0;
	
	// Lógica de la máquina de estados.
	case (current_state)

		// ---- Initialization Sequence ----
		/*
			Initialization Sequence
			-----------------------

			The implemented sequence follows the requirements of the W9812G6KH-5I datasheet.

			1. INIT: Enable delay counter for the next state.

			2. INIT_PAUSE: Wait at least 200us after power-up.
				- During this pause, DQM and CKE are kept high to prevent data contention.

			3. INIT_PRECHARGE_ALL: Issues the precharge all banks command.
				- This command prepares all banks for subsequent operations.

			4. INIT_WAIT_TRP: Wait tRP nanoseconds after precharge.
				- Time required for the precharge to complete internally.

			5. INIT_AUTO_REFRESH + INIT_WAIT_TRC: Executes 8 Auto Refresh cycles.
				- Each cycle issues an AUTO_REFRESH command and waits tRC nanoseconds.
				- All 8 cycles are specifically required by the datasheet for initialization.

			6. INIT_MRS: Mode Register configuration.
				- Configures: Burst Length=1, Sequential type, CAS Latency=3, Standard operation.

			7. INIT_WAIT_TRSC: Wait tRSC before transitioning to IDLE.
				- Time required for the register configuration to complete.
		*/
		INIT:
			begin
				// ---- Outputs ----
				delay_cycles = INIT_PAUSE_CYCLES - 1;
				next_command = CMD_NOP;  // Emitir comando CMD_NOP durante los tiempos de espera.

				// ---- Transitions ----
				next_state = INIT_PAUSE;
			end

		INIT_PAUSE: 
			begin
				// ---- Outputs ----

				// ---- Transitions ----
				if (delay_counter == 0)  // Esperar al menos 200us antes de comenzar la inicialización.
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

				delay_cycles = TRP_CYCLES - 1;
				next_command = CMD_NOP;  // Emitir comando CMD_NOP durante los tiempos de espera.

				// ---- Transitions ----
				next_state = INIT_WAIT_TRP;
			end
		
		INIT_WAIT_TRP: 
			begin
				// ---- Outputs ----

				// ---- Transitions ----
				if (delay_counter == 0)  // Esperar tRP nanosegundos después de emitir el comando precharge all.
					begin
						reset_init_counter = 1'b1;
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
				delay_cycles = TRC_CYCLES - 1;
				next_command = CMD_NOP;		// Emitir comando CMD_NOP durante los tiempos de espera.

				// ---- Transitions ----
				next_state = INIT_WAIT_TRC;		// Esperar tiempo de ciclo de refresco tRC luego de enviar el comando auto refresh.
			end
		
		INIT_WAIT_TRC: 
			begin
				// ---- Outputs ----

				// ---- Transitions ----
				if (delay_counter == 0)  // Esperar tRC.
					begin
						if (init_counter == 0)  // Comprobar si se han completado los 8 ciclos de refresco.
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

				delay_cycles = TRSC_CYCLES - 1;
				next_command = CMD_NOP;  // Emitir comando CMD_NOP durante los tiempos de espera.

				// ---- Transitions ----
				next_state = INIT_WAIT_TRSC;
			end
		
		INIT_WAIT_TRSC: 
			begin
				// ---- Outputs ----

				// ---- Transitions ----
				if (delay_counter == 0)  // Esperar tRSC (Mode Register Set Cycle Time) después del Mode Register Set
					begin
						reset_refresh_counter = 1'b1;	// Starts periodic refresh cycles.
						next_command = CMD_NOP;			// The No Operation Command should be used in cases when the SDRAM is in a idle or a wait state.

						next_state = IDLE;	// Inicialización completada.
					end
			end
		

		// ---- IDLE State ----
		IDLE: 
			begin
				// ---- Outputs ----
				busy_signal = 1'b0;
				ready_signal = 1'b1;

				// ---- Transitions ----
				if (refresh_counter == 0) 
					begin
						/*
							Inicia secuencia de auto refresh.
							El contador de auto refresco se incrementa siempre y se resetea al finalizar 
							la secuencia de auto refresh, en el estado REFRESH_WAIT_TRC.
						*/
						next_state = REFRESH_PRECHARGE_ALL;
						next_command = CMD_PRECHARGE_ALL;
					end
				else if (soc_side_wr_en_port) 
					begin
						// Inicia secuencia de escritura.
						next_state = WRITE_BANK_ACTIVATE;
						next_command = CMD_BANK_ACTIVATE;
					end
				else if (soc_side_rd_en_port) 
					begin
						// Inicia secuencia de lectura.
						next_state = READ_BANK_ACTIVATE;
						next_command = CMD_BANK_ACTIVATE;
					end
			end


		// ---- Auto Refresh Sequence ----
		/*
			Auto Refresh Sequence
			---------------------

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
				// ---- Outputs ----
				// Precarga de todos los bancos.
				// Comando CMD_PRECHARGE_ALL emitido en el estado anterior IDLE.

				delay_cycles = TRP_CYCLES - 1;
				next_command = CMD_NOP;  // Emitir comando CMD_NOP durante los tiempos de espera.

				// ---- Transitions ----
				next_state = REFRESH_WAIT_TRP;
			end

		REFRESH_WAIT_TRP: 
			begin
				// ---- Outputs ----

				// ---- Transitions ----
				if (delay_counter == 0)  // Esperar tRP nanosegundos después de emitir el comando precharge all.
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
				delay_cycles = TRC_CYCLES - 1;
				next_command = CMD_NOP;  // Emitir comando CMD_NOP durante los tiempos de espera.
				
				// ---- Transitions ----
				next_state = REFRESH_WAIT_TRC;
			end

		REFRESH_WAIT_TRC:
			begin
				// ---- Outputs ----

				// ---- Transitions ----
				if (delay_counter == 0)  // Esperar tRC nanosegundos después de emitir el comando CMD_AUTO_REFRESH.
					begin
						// Resetear el contador de refresco.
						reset_refresh_counter = 1'b1;
						next_command = CMD_NOP;		// The No Operation Command should be used in cases when the SDRAM is in a idle or a wait state.

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
				// ---- Outputs ----
				// Activar el banco y fila especificados.
				// Comando CMD_BANK_ACTIVATE emitido en el estado anterior IDLE.

				delay_cycles = TRCD_CYCLES - 1;
				next_command = CMD_NOP;  // Emitir comando CMD_NOP durante los tiempos de espera.

				// ---- Transitions ----
				next_state = WRITE_WAIT_TRCD;
			end

		WRITE_WAIT_TRCD: 
			begin
				// ---- Outputs ----

				// ---- Transitions ----
				if (delay_counter == 0)  // Esperar tRCD.
					begin
						next_command = CMD_WRITE;

						next_state = WRITE_CAS;
					end
			end

		WRITE_CAS: 
			begin
				// ---- Outputs ----
				/*
					Write command issued in the previous state.
					The data to be written is already in the wr_data_reg register; it is captured 
					in wr_data_reg in the sequential block with the rising edge of clk.
				*/

				delay_cycles = TWR_CYCLES - 1;
				next_command = CMD_NOP;  // Emitir comando CMD_NOP durante los tiempos de espera.

				// ---- Transitions ----
				next_state = WRITE_WAIT_TWR;
			end

		WRITE_WAIT_TWR: 
			begin
				// ---- Outputs ----
				// After waiting tWR nanoseconds the data has already been written to the SDRAM memory.

				// ---- Transitions ----
				if (delay_counter == 0)  // Wait tWR nanoseconds.
					begin
						if (ram_addr_reg[10]) 
							begin
								/*
									Si A10=1 durante el comando WRITE, auto-precharge está activo
									y no necesitamos enviar un comando de precharge explícito.
									Actualmente A10=1 siempre pero se mantienen los siguientes dos
									estados WRITE_PRECHARGE y WRITE_WAIT_TRP para futuras mejoras 
									del controlador.
								*/
								next_command = CMD_NOP;		// The No Operation Command should be used in cases when the SDRAM is in a idle or a wait state.

								next_state = IDLE;			// Volver a IDLE directamente.
							end
						else 
							begin
								next_command = CMD_PRECHARGE_BANK;

								next_state = WRITE_PRECHARGE;
							end
					end
			end

		WRITE_PRECHARGE: 
			begin
				// ---- Outputs ----
				// Precargar el banco específico.
				// Comando CMD_PRECHARGE_BANK emitido en el estado anterior.

				// ---- Transitions ----

				delay_cycles = TRP_CYCLES - 1;
				next_command = CMD_NOP;  // Emitir comando CMD_NOP durante los tiempos de espera.

				next_state = WRITE_WAIT_TRP;
			end

		WRITE_WAIT_TRP: 
			begin
				// ---- Outputs ----

				// ---- Transitions ----
				if (delay_counter == 0)  // Esperar tRP.
					begin
						next_command = CMD_NOP;  // The No Operation Command should be used in cases when the SDRAM is in a idle or a wait state.

						next_state = IDLE;  // Volver a IDLE.
					end
			end


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
				- Time required between row activation and column command (Row to Column Delay).
				- tRCD minimum is 15ns for -5/-5I/-5J grade parts.

			3. READ_CAS: Issues the read command.
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
				- Auto-precharge eliminates the need for an explicit precharge command.
		*/
		READ_BANK_ACTIVATE:
			begin
				// ---- Outputs ----
				// Activar el banco y fila especificados.
				// command = CMD_BANK_ACTIVATE, emitido en el estado anterior IDLE.

				delay_cycles = TRCD_CYCLES - 1;
				next_command = CMD_NOP;  // Emitir comando CMD_NOP durante los tiempos de espera.

				// ---- Transitions ----
				next_state = READ_WAIT_TRCD;
			end
		
		READ_WAIT_TRCD:
			begin
				// ---- Outputs ----
				// command = CMD_NOP, emitido en el estado anterior READ_BANK_ACTIVATE.

				// ---- Transitions ----
				if (delay_counter == 0)  // Esperar tRCD después de activar el banco.
					begin
						next_command = CMD_READ;

						next_state = READ_CAS;
					end
			end

		READ_CAS:
			begin
				// ---- Outputs ----
				// Emitir comando de lectura.
				// command = CMD_READ, emitido en el estado anterior READ_WAIT_TRCD.

				delay_cycles = CAS_LATENCY - 1;
				next_command = CMD_NOP;  // Emitir comando CMD_NOP durante los tiempos de espera.

				// ---- Transitions ----
				next_state = READ_WAIT_CAS_LATENCY;
			end

		READ_WAIT_CAS_LATENCY:
			begin
				// ---- Outputs ----

				// ---- Transitions ----
				if (delay_counter == 0)  // Esperar CAS_LATENCY ciclos antes de que los datos estén disponibles.
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
				if (ram_addr_reg[10])  // Si A10=1 durante el comando READ, auto-precharge está activo.
					begin
						/*
							Si A10=1 durante el comando READ, auto-precharge está activo
							y no necesitamos enviar un comando de precharge explícito.
							Actualmente A10=1 siempre pero se mantienen los siguientes dos
							estados READ_PRECHARGE y READ_WAIT_TRP para futuras mejoras del 
							controlador.
						*/

						next_command = CMD_NOP;		// The No Operation Command should be used in cases when the SDRAM is in a idle or a wait state.

						next_state = IDLE;			// Volver a IDLE directamente.
					end
				else
					begin
						next_command = CMD_PRECHARGE_BANK;

						next_state = READ_PRECHARGE;
					end
			end

		READ_PRECHARGE:
			begin
				// ---- Outputs ----
				// Precargar el banco específico.
				// Comando CMD_PRECHARGE_BANK emitido en el estado anterior.

				delay_cycles = TRP_CYCLES - 1;
				next_command = CMD_NOP;  // Emitir comando CMD_NOP durante los tiempos de espera.

				// ---- Transitions ----
				next_state = READ_WAIT_TRP;
			end

		READ_WAIT_TRP:
			begin
				// ---- Outputs ----

				// ---- Transitions ----
				if (delay_counter == 0)  // Esperar tRP después de precargar.
					begin
						next_command = CMD_NOP;		// The No Operation Command should be used in cases when the SDRAM is in a idle or a wait state.

						next_state = IDLE;			// Volver a IDLE.
					end
			end

		
		default: 
			begin
				next_command = CMD_NOP;  // The No Operation Command should be used in cases when the SDRAM is in a idle or a wait state.

				next_state = IDLE;
			end

	endcase
end


// Set the writing mask based on the current state.
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
		// Deshabilita buffers during initialization. Mask all bits high so that the SDRAM drives the input/output buffers to HIGH-Z.
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
        // Durante la lectura, habilitar todos los buffers para que los datos puedan leerse.
        sdram_side_wr_mask_reg = 4'b0000;  // DQM = 0 habilita la salida de datos en lectura.
	else
		// If the state is not read or write, mask all bits high so that the SDRAM drives the input/output buffers to HIGH-Z.
		sdram_side_wr_mask_reg = 4'b1111;
end


// Send addresses to SDRAM based on the current state.
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
				Configuración del registro de modo (MRS) durante inicialización:

					Burst Length de 1 (sin ráfagas)		-> bits 0-2	= 000
					Tipo de burst secuencial			-> bit 3	= 0
					CAS Latency de 3					-> bits 4-6	= 011
					Modo de operación estándar			-> bits 7-8	= 00
					Ajuste de "Burst Read/Single Write"	-> bit 9	= 1

					= 10'b 1 00 011 0 000
			*/
			ram_addr_reg = { {(SDRAM_ADDR_WIDTH - 10){1'b0}}, 10'b1000110000 };
		end
end


endmodule

