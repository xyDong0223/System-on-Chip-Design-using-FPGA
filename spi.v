module spi(
    input clk,					// main clock, drives all logic
    input rst,					// asynchronous reset
    input [31:0] mosi_din,		// 8-bit data to be transmitted
    input [2:0] control,		// 010 for 16-bit data send and then 8 bit read, 011 for 24-bit data send
    output reg [7:0] miso_dout,	// 8-bit received data
    
	//spi signals
    input newcommand, //The first time input bit is high, and an operation is performed for each level shift
	output reg DCLK,
	output reg MOSI,
	output reg CS,
	input MISO
    );

// State machine states
parameter IDLE = 3'b000;
parameter START = 3'b001;
parameter SEND16 = 3'b010;
parameter SEND24 = 3'b011;
parameter RECV = 3'b100;
parameter FINISH = 3'b101;

reg oldcommand = 0;
reg [2:0] next_state, state;
reg [3:0] counter, miso_reg, mosi_reg, sendcounter;
reg [2:0] bitcounter = 3'b111;

always @(posedge clk) 
begin
	if(rst)
	begin
	counter <= 4'b0;
	end
		else 
			if (state == SEND16 || state == SEND24 || state == RECV) begin
				counter <= counter + 1;
			end
			else begin
			    counter <= 4'b0;
			end
end

always @(posedge clk)
begin
	if(rst)
	begin
	bitcounter <= 3'b111;
	sendcounter <= 0;
	DCLK <= 0;
	miso_dout <= 8'b0;
	end
	if((counter < 4'b1011) && (counter > 4'b0010)) begin
			DCLK <= 1'b1;
		end
		else begin
			DCLK <= 1'b0;
		end
		if(counter == 4'b1101) begin
			bitcounter <= bitcounter - 1'b1;
		end
		if((state == RECV) && (counter == 4'b0110))begin
            miso_dout[bitcounter] <= MISO;
        end
		if(counter == 4'b1111 && bitcounter == 3'b111 && (state == SEND16 || state == SEND24 || state == RECV)) begin
		    sendcounter <= sendcounter + 1'b1;
		end
		else if(state == FINISH) begin
		      sendcounter <= 0;
		end
end

always @ (bitcounter or mosi_din or sendcounter)
	case (bitcounter)
		4'd0:		MOSI = mosi_din[0 + 8 * sendcounter];	// LSB
		4'd1:		MOSI = mosi_din[1 + 8 * sendcounter];
		4'd2:		MOSI = mosi_din[2 + 8 * sendcounter];
		4'd3:		MOSI = mosi_din[3 + 8 * sendcounter];
		4'd4:		MOSI = mosi_din[4 + 8 * sendcounter];
		4'd5:		MOSI = mosi_din[5 + 8 * sendcounter];
		4'd6:		MOSI = mosi_din[6 + 8 * sendcounter];
		4'd7:		MOSI = mosi_din[7 + 8 * sendcounter];	// MSB
		default:	MOSI = 1'b0;
endcase

always @(posedge clk)
begin
    if(rst)
    begin
        state <= IDLE;
    end
    state <= next_state;
end

always @(posedge clk)
begin
        if(rst) begin
        oldcommand <= 1'b0;
        CS <= 1;
        end
        case(state)
        IDLE:   begin
                if(oldcommand == ~newcommand) begin
                    oldcommand <= newcommand;
                    next_state <= START;
                    end
                CS <= 1;
                end
        START:  begin
                CS <= 0;
                case(control)
                        SEND16: next_state <= SEND16;
                        SEND24: next_state <= SEND24;
						default: next_state <= IDLE;
                endcase
                end
        SEND16: if(sendcounter > 3'd1)
                begin
                    next_state <= RECV;
                    CS <= 0;
                end
        SEND24: if(sendcounter > 3'd2)
                begin
                    next_state <= FINISH;
                    CS <= 0;
                end
        RECV:   if((sendcounter > 3'd2))
                begin
                    next_state <= FINISH;
					CS <= 0;
                end
        FINISH: begin
                    next_state <= IDLE;
                    CS <= 1;
                end
        default: begin
                    next_state <= IDLE;
                    CS <= 1;
                 end
        endcase
end
endmodule