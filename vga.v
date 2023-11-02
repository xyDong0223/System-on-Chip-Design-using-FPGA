module vga(
    input clk,// system clock
    input rst,// asynchronous reset
    input [31:0] position,// Regular hexagon position and color signal
    output reg [3:0] vgaR,// Red signal output
    output reg [3:0] vgaG,// Green signal output
    output reg [3:0] vgaB,// Blue signal output
    output reg vgaHS,// line synchronization output
    output reg vgaVS// trailing synchronization output
);

reg clk_2 = 0;
reg [9:0] Vcounter, Hcounter;

parameter  H_SYNC   =  10'd96;    //line synchronization
parameter  H_BACK   =  10'd48;    //line display back edge
parameter  H_DISP   =  10'd640;   //line valid data
parameter  H_FRONT  =  10'd16;    //line display leading edge
parameter  H_TOTAL  =  10'd800;   //line scan cycle
parameter  V_SYNC   =  10'd2;     //trailing synchronization
parameter  V_BACK   =  10'd33;    //trailing edge of field display
parameter  V_DISP   =  10'd480;   //trailing valid data
parameter  V_FRONT  =  10'd10;    //trailing display leading edge
parameter  V_TOTAL  =  10'd525;   //trailing scan cycle

always @(posedge clk) //System Clock Halved
begin
    if(rst) begin
    clk_2 = 1'b0;
    end else
        begin
        clk_2 <= ~clk_2;
    end
end

always @(posedge clk)//line synchronization
begin
    if(rst) begin
    Hcounter <= 16'b0;
    end 
    else if((clk_2 == 1) && (Hcounter < H_TOTAL - 1'b1)) begin
        Hcounter <= Hcounter + 1'b1;
        end
        else if(clk_2 == 1) begin
            Hcounter <= 16'b0;
            end
end

always @(posedge clk)//trailing synchronization
begin
    if(rst) begin
    Vcounter <= 16'b0;
    end 
    else if((clk_2 == 1) && (Hcounter == H_TOTAL - 1'b1)) begin
        if(Vcounter < V_TOTAL - 1'b1) begin
            Vcounter <= Vcounter + 1'b1;
            end
            else
            Vcounter <= 16'b0;
    end
end

wire [9:0] Hpot, Vpot;

assign Hpot = position[31] ? 9'd269 +  position[23:16] : 9'd269 -  position[23:16]; //Determine the vertex coordinates of the upper right corner of the octagon
assign Vpot = position[30] ? 9'd189 +  position[15:8] : 9'd189 -  position[15:8]; //Determine the vertex coordinates of the upper right corner of the octagon

always @(*)//regular octagon drawing
begin
    if(Vcounter > Vpot) begin//top trapezoid
        if((Vcounter - Vpot) < 7'd30) begin
            if((Hcounter > (H_SYNC + H_BACK - 1'b1 + Hpot - (Vcounter - Vpot - 7'd29))) && (Hcounter < (H_SYNC + H_BACK - 1'b1 + Hpot + 7'd72 + (Vcounter - Vpot)))) begin
                    vgaR = 4'b1111;
                    vgaG = position[7:4];
                    vgaB = position[3:0];
                end
            else begin
                vgaR = 4'b0;
                vgaG = 4'b0;
                vgaB = 4'b0;
            end
        end
        else if(((Vcounter - Vpot) <= 7'd71) && ((Vcounter - Vpot)> 7'd29)) begin//middle rectangle
                if((Hcounter > (H_SYNC + H_BACK - 1'b1 + Hpot)) && (Hcounter < (H_SYNC + H_BACK - 1'b1 + Hpot + 7'd102))) begin
                    vgaR = 4'b1111;
                    vgaG = position[7:4];
                    vgaB = position[3:0];
                end
             else begin
                vgaR = 4'b0;
                vgaG = 4'b0;
                vgaB = 4'b0;
            end
        end
        else if((Vcounter - Vpot) <= 7'd102 && (Vcounter - Vpot)> 7'd71) begin//down trapezoid
                if((Hcounter > (H_SYNC + H_BACK - 1'b1 + Hpot + (Vcounter - Vpot - 7'd72))) && (Hcounter < (H_SYNC + H_BACK - 1'b1 + Hpot + 7'd102 - (Vcounter - Vpot - 7'd72)))) begin
                    vgaR = 4'b1111;
                    vgaG = position[7:4];
                    vgaB = position[3:0];
                end
                else begin
                vgaR = 4'b0;
                vgaG = 4'b0;
                vgaB = 4'b0;
                end
            end
        else begin
                vgaR = 4'b0;
                vgaG = 4'b0;
                vgaB = 4'b0;
        end   
    end
    else begin
                vgaR = 4'b0;
                vgaG = 4'b0;
                vgaB = 4'b0;      
    end

    vgaHS  = (Hcounter < H_SYNC) ? 1'b0 : 1'b1;//line synchronization output
    vgaVS  = (Vcounter < V_SYNC) ? 1'b0 : 1'b1;//trailing synchronization output
end

endmodule