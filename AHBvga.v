`timescale 1ns / 1ps
module AHBvga(
			// Bus signals
			input wire HCLK,			// bus clock
			input wire HRESETn,			// bus reset, active low
			input wire HSEL,			// selects this slave
			input wire HREADY,			// indicates previous transaction completing
			input wire [31:0] HADDR,	// address
			input wire [1:0] HTRANS,	// transaction type (only bit 1 used)
			input wire HWRITE,			// write transaction
			input wire [31:0] HWDATA,	// write data
			output wire HREADYOUT,		// ready output from slave
			// VGA signals
            output [3:0] vgaR,// Red signal output
            output [3:0] vgaG,// Green signal output
            output [3:0] vgaB,// Blue signal output
            output vgaHS,// line synchronization output
            output vgaVS// trailing synchronization output	
    );

	// Registers to hold signals from address phase
	reg rHADDR;			// only need two bits of address
	reg rWrite, rRead;	// write enable signals

	// Internal signals
    reg [31:0] position;
    wire [3:0] RGBoutR, RGBoutG, RGBoutB;
    wire VgaH, VgaV;
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
                position <= 0;
			end
		else if(rWrite) begin
		  position <= HWDATA[31:0];// Registers hold bus data
		end
    end

// Options on ready signal - can wait on write when full, or read when empty 
	assign HREADYOUT = 1'b1;	// always ready - transaction never delayed

vga vga2(
    .clk(HCLK),
    .rst(~HRESETn),
    .position(position),
    .vgaR(vgaR),
    .vgaG(vgaG),
    .vgaB(vgaB),
    .vgaHS(vgaHS),
    .vgaVS(vgaVS)
    );
endmodule
