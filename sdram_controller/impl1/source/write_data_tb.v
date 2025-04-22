`timescale 1ns/1ps

module write_data_tb;
    // Parameters
    parameter CLK_PERIOD_NS = 12.5;  // 80MHz clock
    
    // Test bench signals
    reg clk;
    reg reset_n;
    wire soc_side_busy;
    wire soc_side_ready;
    
    // Additional signals required for module instantiation
    // Using defaults for signals we're not actively testing
    reg [22:0] soc_side_addr;  // 23-bit address bus (8M x 32-bit words)

    reg [31:0] soc_side_wr_data;
    reg [3:0] soc_side_wr_mask;
    reg soc_side_wr_en;
    
    wire [31:0] soc_side_rd_data;
	reg soc_side_rd_en;
    
    wire [11:0] ram_side_addr;  // Assuming max of ROW_WIDTH (12 bits) for SDRAM_ADDR_WIDTH
    wire [1:0] ram_side_bank_addr;
    
    wire ram_side_chip0_ldqm;
    wire ram_side_chip0_udqm;
    wire [15:0] ram_side_chip0_data;
    
    wire ram_side_chip1_ldqm;
    wire ram_side_chip1_udqm;
    wire [15:0] ram_side_chip1_data;
    
    wire ram_side_cs_n;
    wire ram_side_ras_n;
    wire ram_side_cas_n;
    wire ram_side_wr_en;
    wire ram_side_ck_en;
    
    // Bidirectional data bus handling
    assign ram_side_chip0_data = 16'hzzzz; // High impedance for data bus
    assign ram_side_chip1_data = 16'hzzzz; // High impedance for data bus
    
    // Instantiate the SDRAM controller
    sdram_controller #(
        .CLK_FREQUENCY_MHZ(80),  // 80MHz clock
        .REFRESH_TIME_MS(64),
        .REFRESH_COUNT(4096),
        .ROW_WIDTH(12),
        .COL_WIDTH(9),
        .BANK_ADDR_WIDTH(2)
    ) dut (
        .clk(clk),
        .reset_n_port(reset_n),
        
        // SOC interface
        .soc_side_busy_port(soc_side_busy),
        .soc_side_ready_port(soc_side_ready),
        .soc_side_addr_port(soc_side_addr),
        .soc_side_wr_data_port(soc_side_wr_data),
        .soc_side_wr_mask_port(soc_side_wr_mask),
        .soc_side_wr_en_port(soc_side_wr_en),
        .soc_side_rd_data_port(soc_side_rd_data),
        .soc_side_rd_en_port(soc_side_rd_en),
        
        // SDRAM interface
        .ram_side_addr_port(ram_side_addr),
        .ram_side_bank_addr_port(ram_side_bank_addr),
        .ram_side_chip0_ldqm_port(ram_side_chip0_ldqm),
        .ram_side_chip0_udqm_port(ram_side_chip0_udqm),
        .ram_side_chip0_data_port(ram_side_chip0_data),
        .ram_side_chip1_ldqm_port(ram_side_chip1_ldqm),
        .ram_side_chip1_udqm_port(ram_side_chip1_udqm),
        .ram_side_chip1_data_port(ram_side_chip1_data),
        .ram_side_cs_n_port(ram_side_cs_n),
        .ram_side_ras_n_port(ram_side_ras_n),
        .ram_side_cas_n_port(ram_side_cas_n),
        .ram_side_wr_en_port(ram_side_wr_en),
        .ram_side_ck_en_port(ram_side_ck_en)
    );
    
    // Clock generation.
    initial begin
        clk = 0;
        forever #(CLK_PERIOD_NS / 2) clk = ~clk;
    end
    
    // Test sequence.
    initial begin
        // Initialize inputs.
        reset_n = 0;           // Assert reset (active low).

        soc_side_addr = 0;
        soc_side_wr_data = 0;
        soc_side_wr_mask = 0;
        soc_side_wr_en = 0;
        soc_side_rd_en = 0;
        
        // Wait 10 clock cycles plus a quarter cycle and then deassert the reset.
        #(CLK_PERIOD_NS * 10 + CLK_PERIOD_NS / 4);
        reset_n = 1;  // Deassert reset.
        
        #(400000);  // Wait 400us for SDRAM to initialize.

        soc_side_addr = 23'd8086;  // 0000 0000 0000 0000 0001 1111 1001 0110
        soc_side_wr_data = 32'b11001100111100001111000011110001;  // 3438342385
        soc_side_wr_mask = 4'b1111;
		
        soc_side_wr_en = 1;		// Init write cycle.
		#(CLK_PERIOD_NS * 2)	// Wait 2 clock cycles.
		soc_side_wr_en = 0;

		#(1000);  // Wait 1us.
        
        // End simulation
        $display("Simulation complete at time %t", $time);

        $finish;
    end

    // Monitor reset signal.
    /*initial begin
        $monitor("Time: %0.3fns, reset_n: %b", $realtime, reset_n);
    end*/

    // Monitor state machine.
    initial begin
        $monitor("Time: %0.3fus, current_state: %b, current_command: %b", $realtime / 1000, dut.current_state, dut.current_command);
    end
    
endmodule