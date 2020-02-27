module eBikePhysics(clk,RST_n,SS_n,SCLK,MISO,MOSI,INT,yaw_rt,
	            highGrn,lowGrn,highYlw,lowYlw,highBlu,
		    lowBlu,hallGrn,hallYlw,hallBlu,avg_curr);

  //////////////////////////////////////////////////
  // Model reponse of inertial sensor given a  //
  // yaw_rt input.  The AY is set to "agree"  //
  // with incline derived from yaw_rt.       //
  ///////////////////////////////////////////////
  // This code is a terrible hack.  Remember  //
  // do as I say, not as I do.  I am the     //
  // instructor and I judge you.  Please    //
  // don't judge me.                       //
  //////////////////////////////////////////
  input clk;				// same 50MHz clock you give to Segway.v
  input RST_n;				// unsynchronized raw reset input
  input SS_n;				// active low slave select to inertial sensor
  input SCLK;				// Serial clock
  input MOSI;				// serial data in from master
  input [15:0] yaw_rt;			// rate eBike pitching up hill at
  input highGrn,lowGrn;			// FET controls green coil
  input highYlw,lowYlw;			// FET controls yellow coil
  input highBlu,lowBlu;			// FET controls Blue coil
  output hallGrn,hallYlw,hallBlu;	// hall effect sensor outputs 
  output [11:0] avg_curr;
  output MISO;				// serial data out to master
  output reg INT;			// interrupt output, goes high when inertial sensor has data
  
  localparam FRICTION = 13'h070;
  
  typedef enum reg[1:0] {IDLE,HALF1,HALF2} state_t;
  ////////////////////////////////////////////////////////////////////
  // Registers needed in modeling of inertial sensor declared next //
  //////////////////////////////////////////////////////////////////
  state_t state,nstate;
  reg [15:0] shft_reg_tx;	// SPI shift register for transmitted data (falling edge)
  reg [15:0] shft_reg_rx;	// SPI shift register for received data (rising edge)
  reg [3:0] bit_cnt;		// Needed to know when to interpret R/Wn and address for tx_data
  reg write_reg;		// Used as sentinel to mark that command is write, so write is 
                            // completed at end of transaction
  reg POR_n;			// Power On Reset to inertial sensor active low
  //////// Need array to hold all possible registers of iNEMO ////////
  reg [7:0]registers[0:127];
  reg internal_clk;				// 12.5MHz (represents internal clock of inertial sensor)
  reg [12:0] update_period;		// around 3050Hz (which is much faster than actual rate)
  reg clr_INT;
  
  ////////////////////////////////////////////
  // Registers needed for modeling physics //
  //////////////////////////////////////////
  reg signed [15:0] incline;
  reg signed [15:0] ay;						// computed based on yaw rate
  
  /////////////////////////////////////////////
  // SM outputs declared as type logic next //
  ///////////////////////////////////////////
  logic ld_tx_reg, shft_tx, init;
  logic set_write,clr_write;
  logic [7:0] tx_data;
  
  wire calc_physics;
  wire NEMO_setup;		// once registers setup it will start the measurement cycle of inertial sensor
	
  ///////////////////////////////////////////////////////////////////////////////
  // Now for register and wire defs of what used to be in hub_wheel_models.sv //
  // //////////////////////////////////////////////////////////////////////////
  reg signed [12:0] alpha;		// angular acceleration
  reg signed [19:0] omega;		// angular velocity
  reg [14:0] theta;				// angular position (0 to 359)*64
  reg [16:0] avg_curr_accum;
  reg vld;
  
  wire signed [12:0] coilGY,coilYB,coilBG;
  wire [11:0] abs_coilGY,abs_coilYB,abs_coilBG;
  
  wire [2:0] rot_state; 		// {hallGrn,hallYlw,hallBlu} as a vector
  wire [8:0] pos;				// scaling of angular position used for hall outputs
  wire signed [15:0] new_theta;	// intermediate term for calculating theta
  logic signed [12:0] raw_torque;
  wire signed [12:0] net_torque;
  wire [18:0] omega_abs;
  wire [12:0] back_emf;			// opposing electrical voltage subtracted from coil voltage
  wire [21:0] avg_curr_prod;
  wire [11:0] sum_coil_v;
  wire [23:0] curr_omega_factor;
  wire [14:0] curr_factor; 

  /////////////////////////////////////////////////////////////////////
  // Now for the register & wire defs that were a part of coil_volt //
  ///////////////////////////////////////////////////////////////////
  reg [10:0] PWM_per_cnt;
  reg [10:0] HG_cnt,LG_cnt,HY_cnt,LY_cnt,HB_cnt,LB_cnt;
  reg [10:0] HG_dty,LG_dty,HY_dty,LY_dty,HB_dty,LB_dty;
  reg HG_ff1,LG_ff1,HY_ff1,LY_ff1,HB_ff1,LB_ff1; 
  
  wire HG_rise,HG_fall,LG_rise,LG_fall,HY_rise,HY_fall,HB_rise,HB_fall;
  wire period_over;
  
  //////////////////////////////////////////////
  // Next is modeling of the inertial sensor //
  ////////////////////////////////////////////  
  
  //// Infer main SPI shift register ////
  always_ff @(negedge SCLK, negedge POR_n)
    if (!POR_n)
	  shft_reg_tx <= 16'h0000;
	else if (init)
	  shft_reg_tx <= 16'h0000;
	else if (ld_tx_reg)			// occurs at beginning and middle of 16-bit transaction
	  shft_reg_tx <= {tx_data,8'h00};
	else if (shft_tx)
	  shft_reg_tx <= {shft_reg_tx[14:0],1'b0};

  //// Infer main SPI shift register ////
  always_ff @(posedge SCLK, negedge POR_n)
    if (!POR_n)
	  shft_reg_rx <= 16'h0000;
	else if (!SS_n)
	  shft_reg_rx <= {shft_reg_rx[14:0],MOSI};
	  
  always_ff @(negedge SCLK)
    if (init)
	  bit_cnt <= 4'b0000;
	else if (shft_tx)
	  bit_cnt <= bit_cnt + 1;
	  
  always_ff @(negedge SCLK, negedge POR_n)
    if (!POR_n)
	  write_reg <= 1'b0;
	else if (set_write)
	  write_reg <= 1'b1;
	else if (write_reg)		// can only be high for one SCLK period
	  write_reg <= 1'b0;
	 
  ///////////////////////////////////////////////////
  // At end of SPI transaction, if it was a write //
  // the register being written is updated       //
  ////////////////////////////////////////////////
  always_ff @(posedge SS_n)
    if (write_reg)
      registers[shft_reg_rx[14:8]] <= shft_reg_rx[7:0];
	
  //////////////////////////////////////////////////
  // model update_period for ODR of inert sensor //
  ////////////////////////////////////////////////
  always_ff @(posedge internal_clk, negedge POR_n)
    if (!POR_n)
	  update_period <= 13'h0000;
	else if (NEMO_setup)
	  update_period <= update_period + 1;

  assign calc_physics = &update_period;

	
  always_ff @(posedge internal_clk, negedge POR_n)
    if (!POR_n)
	  INT <= 1'b0;
	else if (clr_INT)
	  INT <= 1'b0;
	else if (&update_period)
	  INT <= 1'b1;
	
  //// Infer state register next ////
  always @(negedge SCLK, negedge POR_n)
    if (!POR_n)
	  state <= IDLE;
	else
	  state <= nstate;

  ///////////////////////////////////////
  // Implement state transition logic //
  /////////////////////////////////////
  always_comb
    begin
      //////////////////////
      // Default outputs //
      ////////////////////
      ld_tx_reg = 0;
      shft_tx = 0;
      init = 0;
      tx_data = 16'h0000;
      set_write = 0;
      nstate = IDLE;	  

      case (state)
        IDLE : begin
          if (!SS_n) begin
		    init = 1;
            nstate = HALF1;
          end
        end
		HALF1 : begin
		  shft_tx = 1;
		  if (bit_cnt==4'b0111) begin
		    ld_tx_reg = 1;
			tx_data = response(shft_reg_rx[7:0]);		// response if function of first 8-bits received
		    nstate = HALF2;
	      end else
		    nstate = HALF1;
		end
		HALF2 : begin
		  shft_tx = 1;		
		  if (bit_cnt==4'b1110) begin
		    set_write = ~shft_reg_rx[14];				// if it is a write set the write sentinel
		    nstate = IDLE;
		  end else
		    nstate = HALF2;
		end
      endcase
    end
	
  ///// MISO is shift_reg[15] with a tri-state ///////////
  assign MISO = (SS_n) ? 1'bz : shft_reg_tx[15];

  ///// MISO is shift_reg[15] with a tri-state ///////////
  assign MISO = (SS_n) ? 1'bz : shft_reg_tx[15];

  initial begin
    POR_n = 0;
	clr_INT = 0;
	internal_clk = 0;
	incline = 16'h0000;
	ay = 16'h0000;
	@(negedge internal_clk);
	POR_n = 1;
  end
  
  always
    #40 internal_clk = ~internal_clk;	// generate 12.5MHz internal clock
  
  function [7:0] response (input [7:0] in_byte);
    if (in_byte[7])	begin		// if it is a read respond with requested register
	  case (in_byte[6:0])
            7'h24 : begin response = 8'h00; clr_INT=1; end
	    7'h25 : begin response = 8'h00; clr_INT=0; end
	    7'h26 : begin response = yaw_rt[7:0]; clr_INT=1; end
 	    7'h27 : begin response = yaw_rt[15:8]; clr_INT=0; end
	    7'h2A : response = ay[7:0];
	    7'h2B : response = ay[15:8];
	    7'h2C : response = 8'h00;
	    7'h2D : response = 8'h00;
	    default : response = registers[in_byte[6:0]];	// case it is just a generic register
	  endcase
	end else					// it is a write
	  response = 8'hA5;			// respond with 0xA5
  endfunction
  
  assign NEMO_setup = ((registers[7'h0d]===8'h02) && (registers[7'h11]===8'h50) &&
	               (registers[7'h10]===8'h53)) ? 1'b1 : 1'b0;

  always @(posedge calc_physics) begin
    incline = calc_incline(incline,yaw_rt);
    ay = accel(incline);
  end
  
  //////////////////////////////////////////////////////
  // functions used in "physics" computations follow //
  ////////////////////////////////////////////////////
  function signed [15:0] calc_incline (input signed [15:0] theta1,omega);
    calc_incline = theta1 + {{7{omega[15]}},omega[15:7]};
  endfunction
  
  function signed [15:0] accel (input signed [15:0] incline1);
    reg signed [15:0] temp;
    temp = incline + {{3{incline1[15]}},incline1[15:3]};
    accel = -temp;
  endfunction 

  //////////// What follows is the contents of hub_wheel_model.sv //////////// 
  
  ////////////////////////////////////////////////////////////////////////////////////
  // what follows is the instance of coil_volt.sv which was inside hub_wheel_model //
  //////////////////////////////////////////////////////////////////////////////////
  
  ///////////////////////////////////////////////
  // Implement period counter for inverse PWM //
  /////////////////////////////////////////////
  always_ff @(posedge clk, negedge RST_n)
    if (!RST_n)
	  PWM_per_cnt <= 11'h000;
	else
	  PWM_per_cnt <= PWM_per_cnt + 1;
	  
  assign period_over = &PWM_per_cnt;
  
  ////////////////////////////////////////////////////
  // implement duty cycle counters for all 6 gates //
  //////////////////////////////////////////////////
  always_ff @(posedge clk, negedge RST_n)
    if (!RST_n)
	  HG_cnt <= 11'h000;
	else if (period_over)
	  HG_cnt <= 11'h000;
	else if (highGrn)
	  HG_cnt <= HG_cnt + 1;

  always_ff @(posedge clk, negedge RST_n)
    if (!RST_n)
	  LG_cnt <= 11'h000;
	else if (period_over)
	  LG_cnt <= 11'h000;
	else if (lowGrn)
	  LG_cnt <= LG_cnt + 1;

  always_ff @(posedge clk, negedge RST_n)
    if (!RST_n)
	  HY_cnt <= 11'h000;
	else if (period_over)
	  HY_cnt <= 11'h000;
	else if (highYlw)
	  HY_cnt <= HY_cnt + 1;

  always_ff @(posedge clk, negedge RST_n)
    if (!RST_n)
	  LY_cnt <= 11'h000;
	else if (period_over)
	  LY_cnt <= 11'h000;
	else if (lowYlw)
	  LY_cnt <= LY_cnt + 1;

  always_ff @(posedge clk, negedge RST_n)
    if (!RST_n)
	  HB_cnt <= 11'h000;
	else if (period_over)
	  HB_cnt <= 11'h000;
	else if (highBlu)
	  HB_cnt <= HB_cnt + 1;

  always_ff @(posedge clk, negedge RST_n)
    if (!RST_n)
	  LB_cnt <= 11'h000;
	else if (period_over)
	  LB_cnt <= 11'h000;
	else if (lowBlu)
	  LB_cnt <= LB_cnt + 1;	 

  //////////////////////////////////////////////////////////
  // Implement capture registers to get duty as a vector // 
  ////////////////////////////////////////////////////////
  always_ff @(posedge clk, negedge RST_n)
    if (!RST_n)
      HG_dty <= 11'h000;
    else if (period_over)
      HG_dty <= HG_cnt;

  always_ff @(posedge clk, negedge RST_n)
    if (!RST_n)
      LG_dty <= 11'h000;
    else if (period_over)
      LG_dty <= LG_cnt;

  always_ff @(posedge clk, negedge RST_n)
    if (!RST_n)
      HY_dty <= 11'h000;
    else if (period_over)
      HY_dty <= HY_cnt;

  always_ff @(posedge clk, negedge RST_n)
    if (!RST_n)
      LY_dty <= 11'h000;
    else if (period_over)
      LY_dty <= LY_cnt;

  always_ff @(posedge clk, negedge RST_n)
    if (!RST_n)
      HB_dty <= 11'h000;
    else if (period_over)
      HB_dty <= HB_cnt;

  always_ff @(posedge clk, negedge RST_n)
    if (!RST_n)
      LB_dty <= 11'h000;
    else if (period_over)
      LB_dty <= LB_cnt;	  
	  
  ///////////////////////////////////////////////////////
  // New readings are valid a cycle after period_over //
  /////////////////////////////////////////////////////
  always_ff @(posedge clk, negedge RST_n)
    if (!RST_n)
	  vld <= 1'b0;
	else
	  vld <= period_over;

  ///////////////////////////////////////////////////////////
  // Now based on gate drives establish each coil voltage //
  /////////////////////////////////////////////////////////
  assign coilGY = (((HG_dty<11'h3FF) && (LG_dty<11'h3FF)) |  	// coil undriven at one end
	               ((HY_dty<11'h3FF) && (LY_dty<11'h3FF))) ? 13'h000 :
				  ({2'b00,HG_dty} - {2'b00,LG_dty}) - ({2'b00,HY_dty} - {2'b00,LY_dty});

  assign coilYB = (((HY_dty<11'h3FF) && (LY_dty<11'h3FF)) |  	// coil undriven at one end
	               ((HB_dty<11'h3FF) && (LB_dty<11'h3FF))) ? 13'h000 :
				  ({2'b00,HY_dty} - {2'b00,LY_dty}) - ({2'b00,HB_dty} - {2'b00,LB_dty});

  assign coilBG = (((HB_dty<11'h3FF) && (LB_dty<11'h3FF)) |  	// coil undriven at one end
	               ((HG_dty<11'h3FF) && (LG_dty<11'h3FF))) ? 13'h000 :
				  ({2'b00,HB_dty} - {2'b00,LB_dty}) - ({2'b00,HG_dty} - {2'b00,LG_dty});
				  	

				  
  assign abs_coilGY = (coilGY[12]) ? -coilGY : coilGY;
  assign abs_coilYB = (coilYB[12]) ? -coilYB : coilYB;
  assign abs_coilBG = (coilBG[12]) ? -coilBG : coilBG;  
  //assign avg_curr = abs_coilGY + abs_coilYB + abs_coilBG;	
  assign avg_curr_prod = 31*avg_curr_accum;
  assign sum_coil_v = abs_coilGY + abs_coilYB + abs_coilBG;
  assign curr_omega_factor = sum_coil_v * (11'h200 + omega_abs[18:8]);
  assign curr_factor = curr_omega_factor[23:9];
  always @(posedge vld, negedge RST_n)
    if (!RST_n)
      avg_curr_accum <= 17'h00000;
    else
      avg_curr_accum <= avg_curr_prod[21:5] + curr_factor; 
  assign avg_curr = avg_curr_accum[16:5];

  //////////////////////////////////////////////////////////////////////////
  // assign hall outputs according to pos(ition) which is based on theta //
  ////////////////////////////////////////////////////////////////////////
  assign pos = theta[14:6];
  assign rot_state = ((pos>=0) && (pos<60)) ? 3'b101 : 		// Green rise state
                     ((pos>=60) && (pos<120)) ? 3'b100 :	// 2nd state of Green
					 ((pos>=120) && (pos<180)) ? 3'b110 : 	// Yellow rise state
					 ((pos>=180) && (pos<240)) ? 3'b010 :	// 2nd state of Yellow
					 ((pos>=240) && (pos<300)) ? 3'b011 :	// Blue rise state
					 ((pos>=300) && (pos<360)) ? 3'b001 :	// 2nd state of Blue
                     3'bxxx;								// this should never happen
					 
  assign hallGrn = rot_state[2];
  assign hallYlw = rot_state[1];
  assign hallBlu = rot_state[0];
  
  /////////////////////////////////////////////////////////
  // Calculate angular position as integration of omega //
  // but with rollover from "359" to "0" ////////////////
  ////////////////////////////////////////
  assign new_theta = {1'b0,theta} + {{4{omega[19]}},omega[19:8]};
  
  always_ff @(posedge clk, negedge RST_n)
    if (!RST_n)
	  theta <= 15'd1920;		// start at 30*64 degrees (middle of Green rise state)
	else if (vld)
	  if (new_theta[15])		// if negative wrap the other way
	    theta <= 15'd23040 + new_theta[14:0];
	  else if (new_theta>=16'd23040)	// if greater than full circle modulo it.
	    theta <= new_theta[14:0] - 15'd23040;
	  else
	    theta <= new_theta[14:0];

  /////////////////////////////////////////////////////////
  // Calculate angular velocity as integration of alpha //
  ///////////////////////////////////////////////////////
  always_ff @(posedge clk, negedge RST_n)
    if (!RST_n)
	  omega <= 20'h000;
	else if (vld)
	  omega <= omega + {{9{alpha[12]}},alpha[12:2]};
	  
  //////////////////////////////////////////////////////////////////////////////
  // Calculate torque based on current position and coil voltages & back_emf //
  ////////////////////////////////////////////////////////////////////////////
  assign omega_abs = (omega[19]) ? -omega : omega;
  assign back_emf = omega_abs[17:5];
  
  always_comb begin
    case (rot_state)
	  3'b101 : begin
	    if (coilGY>=0)
		  raw_torque = coilGY - back_emf;
		else if (coilGY<0)
		  raw_torque = coilGY + back_emf;
		else
		  raw_torque = 12'h000;
	  end
	  3'b100 : begin
	    if (coilBG<=0)
		  raw_torque = -coilBG - back_emf;
		else if (coilYB>0)
		  raw_torque = -coilYB + back_emf;
		else
		  raw_torque = 12'h000;
	  end
	  3'b110 : begin
	    if (coilYB>=0)
		  raw_torque = coilYB - back_emf;
		else if (coilBG<0)
		  raw_torque = coilBG + back_emf;
		else
		  raw_torque = 12'h000;
	  end
	  3'b010 : begin
	    if (coilGY<=0)
		  raw_torque = -coilGY - back_emf;
		else if (coilGY>0)
		  raw_torque = -coilGY + back_emf;
		else
		  raw_torque = 12'h000;
	  end
	  3'b011 : begin
	    if (coilBG>=0)
		  raw_torque = coilBG - back_emf;
		else if (coilYB<0)
		  raw_torque = coilYB + back_emf;
		else
		  raw_torque = 12'h000;
	  end
	  3'b001 : begin
	    if (coilYB<=0)
		  raw_torque = -coilYB - back_emf;
		else if (coilBG>0)
		  raw_torque = -coilBG + back_emf;
		else
		  raw_torque = 12'h000;
	  end	  
	endcase
  end
  
  ///////////////////////////////////////////////////////////
  // Now modify raw_torque based on friction and back_emf //
  /////////////////////////////////////////////////////////
  assign net_torque = (raw_torque > FRICTION) ? raw_torque - FRICTION :
                      (raw_torque < -FRICTION) ? raw_torque + FRICTION :
					  (omega>20'h00200) ? -FRICTION :
					  (omega<-20'h00200) ? FRICTION :
					  13'h000;
	
  assign alpha = {{1{net_torque[12]}},net_torque[12:1]};	
					  
  
endmodule  
