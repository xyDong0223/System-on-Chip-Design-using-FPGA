`timescale 1ns / 1ns
//////////////////////////////////////////////////////////////////////////////////
// Company: UCD School of Electrical and Electronic Engineering
// Engineer: Brian Mulkeen
// 
// Design Name:   Cortex-M0 DesignStart system for Digilent Nexys4 board
// Module Name:   AHBDCD 
// Description:   Address decoder for AHB Lite bus, incomplete
//          Examines the 8 MSBs of the address signal.
//			Outputs ten individual slave select signals and a 4-bit signal
//          to tell the multiplexers which slave is active.
//
//////////////////////////////////////////////////////////////////////////////////

module AHBDCD(
    input [31:0] HADDR,       // AHB bus address  
    output reg HSEL_S0,       // slave select line 0
    output reg HSEL_S1,
    output reg HSEL_S2,
    output reg HSEL_S3,
    output reg HSEL_S4,
    output reg HSEL_S5,
    output reg HSEL_S6,
    output reg HSEL_S7,
    output reg HSEL_S8,
    output reg HSEL_S9,       // slave select line 9
    output reg HSEL_NOMAP,    // indicates invalid address  
    output reg [3:0] MUX_SEL  // multiplexer control signal
    );  // end of port list


// Address decoding logic to implement the address map: 
// decide which slave is active by checking the 8 MSBs of the address.
always @ (HADDR)
    begin
        HSEL_S0 = 1'b0;         // all slave select outputs will be 0
        HSEL_S1 = 1'b0;         // unless one of them is set to 1 below
        HSEL_S2 = 1'b0;
        HSEL_S3 = 1'b0;
        HSEL_S4 = 1'b0;
        HSEL_S5 = 1'b0;
        HSEL_S6 = 1'b0;
        HSEL_S7 = 1'b0;
        HSEL_S8 = 1'b0;
        HSEL_S9 = 1'b0;
        HSEL_NOMAP = 1'b0;
        
// Logic to select one slave, and also output the slave number to the multiplexers
// ## As you add more slaves, you need to extend this logic to select those slaves
        case(HADDR[31:24])      // Use the top 8 bits of the address to select
            8'h00: 				// Address range 0x0000_0000 to 0x00FF_FFFF  16MB
                begin
                    HSEL_S0 = 1'b1;     // activate slave select 0 output
                    MUX_SEL = 4'd0;     // send slave number 0 to multiplexers
                end
                
            8'h20: 				// Address range 0x2000_0000 to 0x20FF_FFFF  16MB
                begin
                    HSEL_S1 = 1'b1;     // activate slave select 1 output
                    MUX_SEL = 4'd1;     // send slave number 1 to multiplexers
                end
            
             8'h50: 				// Address range 0x2000_0000 to 0x20FF_FFFF  16MB
                    begin
                        HSEL_S2 = 1'b1;     // activate slave select 1 output
                        MUX_SEL = 4'd2;     // send slave number 1 to multiplexers
                    end
                        
             8'h51: 				// Address range 0x5100_0000 to 0x51FF_FFFF  16MB
                    begin
                        HSEL_S3 = 1'b1;     // activate slave select 1 output
                        MUX_SEL = 4'd3;     // send slave number 1 to multiplexers
                    end
             8'h52: 				// Address range 0x5200_0000 to 0x52FF_FFFF  16MB
                   begin
                        HSEL_S4 = 1'b1;     // activate slave select 1 output
                        MUX_SEL = 4'd4;     // send slave number 1 to multiplexers
                   end   
             8'h53: 				// Address range 0x5300_0000 to 0x53FF_FFFF  16MB
                   begin
                        HSEL_S5 = 1'b1;     // activate slave select 1 output
                        MUX_SEL = 4'd5;     // send slave number 1 to multiplexers
                   end
             8'h54: 				// Address range 0x5400_0000 to 0x54FF_FFFF  16MB
                   begin
                        HSEL_S6 = 1'b1;     // activate slave select 1 output
                        MUX_SEL = 4'd6;     // send slave number 1 to multiplexers
                   end  
             8'h55: 				// Address range 0x5500_0000 to 0x55FF_FFFF  16MB
                   begin
                        HSEL_S7 = 1'b1;     // activate slave select 1 output
                        MUX_SEL = 4'd7;     // send slave number 1 to multiplexers
                   end      
            default: 			// Address not mapped to any slave
                begin
                    HSEL_NOMAP = 1'b1;   // activate the NOMAP output
                    MUX_SEL = 4'd15;     // send dummy slave number 15 to multiplexers 
                end
        endcase
    end  // end of always block
    
endmodule
