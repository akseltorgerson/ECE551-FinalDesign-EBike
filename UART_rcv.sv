module UART_rcv(clk,rst_n,RX,rdy,rx_data,clr_rdy);

input clk,rst_n;			// clock and active low reset
input RX;					// rx is the asynch serial input (need to double flop)
input clr_rdy;			// rdy can be cleared by this or n007 of new byte
output rdy;				// signifies to core a byte has been received
output [7:0] rx_data;		// data that was received

typedef enum reg {IDLE, RX_STATE} asdf;
asdf n100, n001;

reg [8:0] n002;
reg [3:0] n003;
reg [8:0] n004;
reg rdy;
reg n005, n006;

logic n007, n008, n009;

wire shift;


always_ff @(posedge clk or negedge rst_n)
  if (!rst_n)
    n100 <= IDLE;
  else
    n100 <= n001;


always_ff @(posedge clk or negedge rst_n)
  if (!rst_n)
    n003 <= 4'b0000;
  else if (n007)
    n003 <= 4'b0000;
  else if (shift)
    n003 <= n003+1;


always_ff @(posedge clk or negedge rst_n)
  if (!rst_n)
    n004 <= 217;			
  else if (n007)
    n004 <= 217;			
  else if (shift)
    n004 <= 434;			
  else if (n009)
    n004 <= n004-1;		


always_ff @(posedge clk)
  if (shift)
    n002 <= {n006,n002[8:1]};


always @(posedge clk or negedge rst_n)
  if (!rst_n)
    rdy <= 1'b0;
  else if (n007 || clr_rdy)
    rdy <= 1'b0;
  else if (n008)
    rdy <= 1'b1;

always_ff @(posedge clk or negedge rst_n)
  if (!rst_n)
    begin
      n005 <= 1'b1;
      n006 <= 1'b1;
    end
  else
    begin
      n005 <= RX;
      n006 <= n005;
    end


always_comb
  begin
    n007         = 0;
    n008    	  = 0;
    n009     = 0;
    n001     = IDLE;
    
    case (n100)
      IDLE : begin
        if (!n006)
          begin
            n001 = RX_STATE;
            n007 = 1;
          end
        else n001 = IDLE;
      end
      default : begin
        if (n003==4'b1010)
          begin
            n008 = 1;
            n001 = IDLE;
          end
        else
          n001 = RX_STATE;
        n009 = 1;
      end
    endcase
  end

///////////////////////////////////
// Continuous assignment follow //
/////////////////////////////////
assign shift = ~|n004;
assign rx_data = n002[7:0];

endmodule
