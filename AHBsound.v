`timescale 1ns / 1ps

module AHBsound(
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
            output reg Audiout			// Audio output
    );

	// Registers to hold signals from address phase
	reg rHADDR;			// only need two bits of address
	reg rWrite, rRead;	// write enable signals

	// Internal signals
    reg [2:0] freqselect;
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
   
    assign HRDATA = {16'b0,frq};

    always @(posedge HCLK)
    begin
        if(!HRESETn)
			begin
                freqselect <= 0;
			end
		else if(rWrite) begin
		  freqselect <= HWDATA[2:0];
		end
    end
    
    reg [15:0] counter, frq;
    
    always @(posedge HCLK)//frequency division counter
    begin
        if(!HRESETn) begin
           counter <= 0;
           Audiout <= 0;
		end
        if(counter > frq) begin
            Audiout <= ~Audiout;
            counter <= 0;
		end
		else begin
		  counter = counter + 1;
		end
    end
    
    always @(*)
    begin
        case(freqselect)//different frequency options
            3'b001: frq = 16'd6250; 
            3'b010: frq = 16'd12500;
            3'b011: frq = 16'd25000;
            3'b100: frq = 16'd50000;
            default:  frq = 0;
        endcase
    end

// Options on ready signal - can wait on write when full, or read when empty 
	assign HREADYOUT = 1'b1;	// always ready - transaction never delayed

endmodule