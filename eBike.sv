 module eBike(clk,RST_n,A2D_SS_n,A2D_MOSI,A2D_SCLK,
             A2D_MISO,hallGrn,hallYlw,hallBlu,highGrn,
			 lowGrn,highYlw,lowYlw,highBlu,lowBlu,
			 inertSS_n,inertSCLK,inertMOSI,inertMISO,
			 inertINT,cadence,TX,tgglMd,setting);
			 
  input logic clk;				// 50MHz clk
  input logic RST_n;				// active low RST_n from push button
  output logic A2D_SS_n;			// Slave select to A2D on DE0
  output logic  A2D_SCLK;			// SPI clock to A2D on DE0
  output logic A2D_MOSI;			// serial output to A2D (what channel to read)
  input logic A2D_MISO;			// serial input from A2D
  input logic hallGrn;			// hall position input for "Green" phase
  input logic hallYlw;			// hall position input for "Yellow" phase
  input logic hallBlu;			// hall position input for "Blue" phase
  output logic highGrn;			// high side gate drive for "Green" phase
  output logic lowGrn;			// low side gate drive for "Green" phase
  output logic highYlw;			// high side gate drive for "Yellow" phase
  output logic lowYlw;			// low side gate drive for "Yellow" phas
  output logic highBlu;			// high side gate drive for "Blue" phase
  output logic lowBlu;			// low side gate drive for "Blue" phase
  output logic inertSS_n;			// Slave select to inertial (tilt) sensor
  output logic inertSCLK;			// SCLK signal to inertial (tilt) sensor
  output logic inertMOSI;			// Serial out to inertial (tilt) sensor  
  input logic inertMISO;			// Serial in from inertial (tilt) sensor
  input logic inertINT;			// Alerts when inertial sensor has new reading
  input logic cadence;			// pulse input from pedal cadence sensor
  input logic tgglMd;				// used to select setting[1:0] (from PB switch)
  output reg [1:0] setting;	// 11 => easy, 10 => normal, 01 => hard, 00 => off
  output logic TX;				// serial output of measured batt,curr,torque
  
  ///////////////////////////////////////////////
  // Declare any needed internal signals here //
  /////////////////////////////////////////////
  wire rst_n;									// global reset from reset_synch
  logic tgglMd_F1, tgglMd_F2, tgglMd_F3; // used to detect a rising edge of tgglMd
  logic [11:0] batt, curr, torque, brake;
  logic [12:0] incline;
  logic [12:0] error;
  logic not_pedaling;
  logic [11:0] drv_mag;
  logic brake_n;
  logic [10:0] duty;
  logic [1:0] selGrn;
  logic [1:0] selYlw;
  logic [1:0] selBlu;
  logic vld;
  
  ///////// Any needed macros follow /////////
  parameter FAST_SIM = 0;
  
  
  /////////////////////////////////////
  // Instantiate reset synchronizer //
  ///////////////////////////////////
	rst_synch iRst_synch(.RST_n(RST_n), .clk(clk), .rst_n(rst_n));
  
  ///////////////////////////////////////////////////////
  // Instantiate A2D_intf to read torque & batt level //
  /////////////////////////////////////////////////////
	A2D_intf iA2D_intf(.clk(clk), .rst_n(rst_n), .MISO(A2D_MISO), .batt(batt), .curr(curr), .brake(brake), .torque(torque), .SS_n(A2D_SS_n), .SCLK(A2D_SCLK), .MOSI(A2D_MOSI));
				 
  ////////////////////////////////////////////////////////////
  // Instantiate SensorCondition block to filter & average //
  // readings and provide cadence_vec, and zero_cadence   //
  // Don't forget to pass FAST_SIM parameter!!           //
  ////////////////////////////////////////////////////////
	sensorCondition #(FAST_SIM) iSensorCondition(.clk(clk), .rst_n(rst_n), .torque(torque), .cadence(cadence), .curr(curr), .incline(incline), .setting(setting), .batt(batt), .error(error), .not_pedaling(not_pedaling), .TX(TX));

  ///////////////////////////////////////////////////
  // Instantiate PID to determine drive magnitude //
  // Don't forget to pass FAST_SIM parameter!!   //
  ////////////////////////////////////////////////	
	PID #(FAST_SIM) iPID(.clk(clk), .rst_n(rst_n), .error(error), .not_pedaling(not_pedaling), .drv_mag(drv_mag));
  
  ////////////////////////////////////////////////
  // Instantiate brushless DC motor controller //
  //////////////////////////////////////////////
	brushless iBrushless(.clk(clk), .drv_mag(drv_mag), .hallGrn(hallGrn), .hallYlw(hallYlw), .hallBlu(hallBlu), .brake_n(brake_n), .duty(duty), .selGrn(selGrn), .selYlw(selYlw), .selBlu(selBlu));

  ///////////////////////////////
  // Instantiate motor driver //
  /////////////////////////////
	mtr_drv iMtr_drv(.clk(clk), .rst_n(rst_n), .duty(duty), .selGrn(selGrn), .selYlw(selYlw), .selBlu(selBlu), .highGrn(highGrn), .lowGrn(lowGrn), .highYlw(highYlw), .lowYlw(lowYlw), .highBlu(highBlu), .lowBlu(lowBlu));


  /////////////////////////////////////////////////////////////
  // Instantiate inertial sensor to measure incline (pitch) //
  ///////////////////////////////////////////////////////////
	inert_intf iInert_intf(.clk(clk), .rst_n(rst_n), .incline(incline), .vld(vld), .INT(inertINT), .SS_n(inertSS_n), .SCLK(inertSCLK), .MOSI(inertMOSI), .MISO(inertMISO));
					
  ////////////////////////////////////////////////////////
  // Instantiate (or infer) tggleMd/setting[1:0] logic //
  //////////////////////////////////////////////////////
  
  //Rise edge detect
	always @(posedge clk, negedge rst_n) begin
		if(!rst_n) begin
			tgglMd_F1 <= 0;
			tgglMd_F2 <= 0;
			tgglMd_F3 <= 0;
		end
		else begin
			tgglMd_F1 <= tgglMd;
			tgglMd_F2 <= tgglMd_F1;
			tgglMd_F3 <= tgglMd_F2;
		end
	end
  //setting counter
	always @(posedge clk, negedge rst_n) begin
		if(!rst_n)
			setting <= 2'b10;
		else if(tgglMd_F3 & ~tgglMd_F2)
			setting <= setting + 1;
	end
  ///////////////////////////////////////////////////////////////////////
  // brake_n should be asserted if brake A2D reading lower than 0x800 //
  /////////////////////////////////////////////////////////////////////
	assign brake_n = (brake < 12'h800) ? 0 : 1;

endmodule























