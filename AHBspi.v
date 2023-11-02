`timescale 1ns / 1ns
module AHBspi(
			// Bus signals
			input wire HCLK,			// bus clock
			input wire HRESETn,			// bus reset, active low
			input wire HSEL,			// selects this slave
			input wire HREADY,			// indicates previous transaction completing
			input wire [31:0] HADDR,	// address
			input wire [1:0] HTRANS,	// transaction type (only bit 1 used)
			input wire HWRITE,			// write transaction
			input wire [31:0] HWDATA,	// write data
			output wire [31:0] HRDATA,	// read data from slave
			output wire HREADYOUT,		// ready output from slave
			// SPI signals
			input MISO,				// 
			output MOSI,			// 
			output CS,
			output DCLK
    );
	
	// Registers to hold signals from address phase
	reg rHADDR;			// only need two bits of address
	reg rWrite, rRead;	// write enable signals

	// Internal signals
	reg [7:0]  readData;		// 8-bit data from read multiplexer
	wire [7:0] miso_dout;
	reg spi_command;		// transmitter status signal
    reg [31:0] mosi_din;
    reg [2:0] control;

 	// Capture bus signals in address phase
	always @(posedge HCLK)
		if(!HRESETn)
			begin
				rHADDR <= 1'b0;
				rWrite <= 1'b0;
				rRead  <= 1'b0;
			end
		else if(HREADY)
		 begin
			rHADDR <= HADDR[1];         // capture address bits for for use in data phase
			rWrite <= HSEL & HWRITE & HTRANS[1];	// slave selected for write transfer       
			rRead <= HSEL & ~HWRITE & HTRANS[1];	// slave selected for read transfer 
		 end
    always @(posedge HCLK)
    begin
        if(!HRESETn)
			begin
				mosi_din <= 32'b0;
				spi_command <= 1'b0;
				control  <= 3'b0;
			end
		else if(rWrite) begin
		  mosi_din = {8'b0, HWDATA[23:0]};
		  spi_command = HWDATA[24];
		  control = HWDATA[31:29];
		end
    end
	// Control register
	
    always @(miso_dout, rHADDR)
        case (rHADDR)		// select on word address (stored from address phase)
            1'h0:		readData = miso_dout;	// read from rx fifo - oldest received byte
            1'h1:		readData = 16'b0;	// read of tx register gives oldest byte in queue		
        endcase

	assign HRDATA = rRead ? {24'b0, readData} : 32'b0;	// extend with 0 bits for bus read

// Options on ready signal - can wait on write when full, or read when empty 
	assign HREADYOUT = 1'b1;	// always ready - transaction never delayed
spi spi2(
    .clk (HCLK),			// 50 MHz clock
    .rst(~HRESETn),			// asynchronous reset
    .mosi_din(mosi_din),	// 24-bit data to be transmitted
    .control(control),		// 010 for 16-bit data send and then 8 bit read, 011 for 24-bit data send
    .miso_dout(miso_dout),	// 8-bit received data
    
    .newcommand(spi_command), //The first time input bit is high, and an operation is performed for each level shift
    
	.DCLK(DCLK),
	.MOSI(MOSI),
	.CS(CS),
	.MISO(MISO)
    );
endmodule

