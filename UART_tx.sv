module UART_tx(clk,rst_n,TX,trmt,tx_data,tx_done);

/////////////////////////////////////////////////////////////////
// Intentionally obfuscated model of UART_tx with 115200 baud //
///////////////////////////////////////////////////////////////
input clk,rst_n;		// clock and active low reset
input trmt;				// trmt tells TX section to transmit tx_data
input [7:0] tx_data;	// byte to transmit
output TX, tx_done;		// tx_done asserted when transmission complete

reg n001,n002;
reg [8:0] n003;
reg [3:0] n004;
reg [8:0] n005;
reg tx_done;

reg n007, n008;

wire n009;

localparam N010 = 1'b0;
localparam N011 = 1'b1;

////////////////////////////
// Infer n001 flop next //
//////////////////////////
always @(posedge clk or negedge rst_n)
  if (!rst_n)
    n001 <= N010;
  else
    n001 <= n002;


/////////////////////////
// Infer n004 next //
///////////////////////
always @(posedge clk or negedge rst_n)
  if (!rst_n)
    n004 <= 4'b0000;
  else if (n007)
    n004 <= 4'b0000;
  else if (n009)
    n004 <= n004+1;

//////////////////////////
// Infer n005 next //
////////////////////////
always @(posedge clk or negedge rst_n)
  if (!rst_n)
    n005 <= 434;
  else if (n007 || n009)
    n005 <= 434;
  else if (n008)
    n005 <= n005-1;

////////////////////////////////
// Infer n009 register next //
//////////////////////////////
always @(posedge clk or negedge rst_n)
  if (!rst_n)
    n003 <= 9'h1FF;
  else if (n007)
    n003 <= {tx_data,1'b0};
  else if (n009)
    n003 <= {1'b1,n003[8:1]};

///////////////////////////////////////////////
// Easiest to make tx_done a set/reset flop //
/////////////////////////////////////////////
always @(posedge clk or negedge rst_n)
  if (!rst_n)
    tx_done <= 1'b0;
  else if (trmt)
    tx_done <= 1'b0;
  else if (n004==4'b1010)
    tx_done <= 1'b1;


always @(n001,trmt,n009,n004)
  begin
    //////////////////////////////////////
    // Default assign all output of SM //
    ////////////////////////////////////
    n007         = 0;
    n008 = 0;
    n002    = N010;
    
    case (n001)
      N010 : begin
        if (trmt)
          begin
            n002 = N011;
            n007 = 1;
          end
        else n002 = N010;
      end
      default : begin
        if (n004==4'b1010)
          n002 = N010;
        else
          n002 = N011;
        n008 = 1;
      end
    endcase
  end


assign n009 = ~|n005;
assign TX = n003[0];

endmodule

