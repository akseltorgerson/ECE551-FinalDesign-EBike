module inertial_integrator(clk,rst_n,vld,roll_rt,yaw_rt,AY,AZ,incline);

						   
  input clk, rst_n;
  input vld;							// goes high for 1 clock cycle when new data valid
  input signed [15:0] roll_rt;		// raw gyro rate reading from inert_intf
  input signed [15:0] yaw_rt;			// raw gyro rate reading from inert_intf
  input signed [15:0] AY;				// raw AX reading from inert_intf
  input signed [15:0] AZ;				// AZ & roll are needed to tell if bike is leaning side to side

  output signed [12:0] incline;
  reg signed [25:0] roll_acc_product;		// used in fusion calculations
  reg vld_ff,vld_ff2;					// vld delayed by 1 clock
  wire signed [12:0] roll_acc;				// roll purely from accelerometer, used in fusion
  reg signed [25:0] incline_acc_product;	// used in fusion calculations
  wire signed [12:0] incline_acc;			// incline exclusively from accel (used in fusion)
  wire signed [23:0] fusion_roll_offset;	// fusion term added or subtracte for leaking to roll_acc
  wire signed [23:0] fusion_incline_offset;	// fusion term added or subtracted for leaking to incline_acc
  wire signed [15:0] yaw_rt_comp;			// offset compensated version of yaw_rt
  wire signed [12:0] incline_internal;	   // internal verions of incline (no roll zeroing applied)
  wire signed [12:0] roll;						// signed version of roll
  wire [11:0] roll_abs;							// absolute value of roll

  //////////////////////////////
  // Define needed registers //
  ////////////////////////////
  reg signed [23:0] incline_int;						// angle integrator
  reg signed [23:0] roll_int;
  
  localparam YAW_RT_OFFSET = 16'h0054;		// offset of this inertial sensor for yaw rate
  localparam ROLL_THRES = 12'h0098;		   // Threshold of too much lean for when incline set to zero
  

  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n) begin
      vld_ff <= 1'b0;
      vld_ff2 <= 1'b0;
    end else begin
      vld_ff <= vld;
      vld_ff2 <= vld_ff;
    end

  assign yaw_rt_comp = yaw_rt - YAW_RT_OFFSET;
  
  //////////////////////////////////
  // incline integrator register //
  ////////////////////////////////
  always @(posedge clk, negedge rst_n)
    if (!rst_n)
	  incline_int <= 0;
    else if (vld_ff2)
	  ///////////////////////////////////////////////////////////////////
	  // Integrate angular rate to get position but "leak" it toward  //
	  // incline as obtained through accel (i.e. fusion)               //
	  ////////////////////////////////////////////////////////////////
	  incline_int <= incline_int + {{8{yaw_rt_comp[15]}},yaw_rt_comp} + fusion_incline_offset;          
  
    ///////////////////////////////
    // roll integrator register //
    /////////////////////////////
    always @(posedge clk, negedge rst_n)
      if (!rst_n)
	     roll_int <= 0;
      else if (vld_ff2)
	    ///////////////////////////////////////////////////////////////////
	    // Integrate angular rate to get position but "leak" it toward  //
	    // roll as obtained through accel (i.e. fusion)               //
	    ////////////////////////////////////////////////////////////////
	    roll_int <= roll_int + {{8{roll_rt[15]}},roll_rt} + fusion_roll_offset;	  
	  
	//////////////////////
	// Divide by 2^11. //
	///////////////////////////////////////////////
	// This is just a scaling factor.  Somewhat //
	// arbitrary, choosen by trial and error.  //
	////////////////////////////////////////////
	assign incline_internal = incline_int[23:11];
	assign roll = roll_int[23:11];
	assign roll_abs = (roll[12]) ? -roll : roll;
	assign incline = (roll_abs>ROLL_THRES) ? 13'h0000 : incline_internal;

	////////////////////////////////////////////////////////
	// Now calculate roll & incline from G readings only //
	////////////////////////////////////////////////////////////////////////
	// Where did that 327 number come from?  Trial/error and observation //
	//////////////////////////////////////////////////////////////////////
	always @(posedge clk) begin
	  incline_acc_product <= AY*$signed(-10'd327);
	  roll_acc_product <= AZ*$signed(10'd327);
	end

	assign incline_acc = incline_acc_product[25:13];
	assign roll_acc = roll_acc_product[25:13];
	
    /////////////////////////////////////////////////////////////////
	// "leak" the integrator positive if incline_acc>incline, and //
    // leak negative	if incline_acc<incline.  So "DC" reading //
	// approaches that of what is calculated by the accel alone.//
	/////////////////////////////////////////////////////////////
	assign fusion_incline_offset = (incline_acc>incline_internal) ? 24'h000400 : 24'hFFFC00;
	assign fusion_roll_offset = (roll_acc>roll) ? 24'h000400 : 24'hFFFC00;
	
	
endmodule
