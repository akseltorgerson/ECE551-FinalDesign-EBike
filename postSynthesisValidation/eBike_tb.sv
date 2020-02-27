`timescale 1ns/1ps
module eBike_tb();

	reg clk,RST_n;
	reg [11:0] BATT;				// analog values you apply to AnalogModel
	reg [11:0] BRAKE,TORQUE;		// analog values
	reg cadence;					// you have to have some way of applying a cadence signal
	reg tgglMd;	
	reg [15:0] YAW_RT;			// models angular rate of incline
  
	wire A2D_SS_n,A2D_MOSI,A2D_SCLK,A2D_MISO;		// A2D SPI interface
	wire highGrn,lowGrn,highYlw;					// FET control
	wire lowYlw,highBlu,lowBlu;					//   PWM signals
	wire hallGrn,hallBlu,hallYlw;					// hall sensor outputs
	wire inertSS_n,inertSCLK,inertMISO,inertMOSI,inertINT;	// Inert sensor SPI bus
  
	wire [1:0] setting;		// drive LEDs on real design
	wire [11:0] curr;			// comes from eBikePhysics back to AnalogModel
		
	//Variables for omega testing
	reg signed [19:0] omega_temp1;
	reg signed [19:0] omega_temp2;
	//////////////////////////////////////////////////
	// Instantiate model of analog input circuitry //
	////////////////////////////////////////////////
	AnalogModel iANLG(.clk(clk),.rst_n(RST_n),.SS_n(A2D_SS_n),.SCLK(A2D_SCLK),
		.MISO(A2D_MISO),.MOSI(A2D_MOSI),.BATT(BATT),
		.CURR(curr),.BRAKE(BRAKE),.TORQUE(TORQUE));

	////////////////////////////////////////////////////////////////
	// Instantiate model inertial sensor used to measure incline //
	//////////////////////////////////////////////////////////////
	eBikePhysics iPHYS(.clk(clk),.RST_n(RST_n),.SS_n(inertSS_n),.SCLK(inertSCLK),
		.MISO(inertMISO),.MOSI(inertMOSI),.INT(inertINT),
		.yaw_rt(YAW_RT),.highGrn(highGrn),.lowGrn(lowGrn),
		.highYlw(highYlw),.lowYlw(lowYlw),.highBlu(highBlu),
		.lowBlu(lowBlu),.hallGrn(hallGrn),.hallYlw(hallYlw),
		     .hallBlu(hallBlu),.avg_curr(curr));

	//////////////////////
	// Instantiate DUT //
	////////////////////
	eBike iDUT(.clk(clk),.RST_n(RST_n),.A2D_SS_n(A2D_SS_n),.A2D_MOSI(A2D_MOSI),
		.A2D_SCLK(A2D_SCLK),.A2D_MISO(A2D_MISO),.hallGrn(hallGrn),
		.hallYlw(hallYlw),.hallBlu(hallBlu),.highGrn(highGrn),
		.lowGrn(lowGrn),.highYlw(highYlw),.lowYlw(lowYlw),
		.highBlu(highBlu),.lowBlu(lowBlu),.inertSS_n(inertSS_n),
		.inertSCLK(inertSCLK),.inertMOSI(inertMOSI),
		.inertMISO(inertMISO),.inertINT(inertINT),
		.cadence(cadence),.tgglMd(tgglMd),.TX(TX),
		.setting(setting));

	///////////////////////////////////////////////////////////
	// Instantiate Something to monitor telemetry output??? //
	/////////////////////////////////////////////////////////
	initial begin
		
	 	clk = 0; // only need to do this once
	 	
	 	//***************************** Test Suite ******************************************
	 	
		BATT = 0;		// analog values you apply to AnalogModel
		BRAKE = 0;
		TORQUE = 0;		// analog values
		cadence = 0;		// you have to have some way of applying a cadence signal
		tgglMd = 0;
		YAW_RT = 0;
	
	 	// reset everything
	 	RST_n = 0;
		@(posedge clk);
		@(negedge clk);
		RST_n = 1;
		
	 	//TEST 1 CHECKS THAT WHEN TORQUE INCREASES SO SHOULD OMEGA AND CURR
	 		//force iDUT.setting = 2'b10;
			BATT = 12'hB11; // battery not low
			BRAKE = 12'h900; // brake not asserted (LOW)
			TORQUE = 12'h400; // torque low
			YAW_RT = 16'h2000; // uphill
				
			// alternate the cadence signal to simulate pedaling
			// this is what takes a long ass time	
			repeat(10)begin
				cadence = 0;
				repeat(2048)@(posedge clk);
				cadence = 1;
				repeat(2048)@(posedge clk);
				end

				omega_temp1 = iPHYS.omega;

			//Test with high torque and uphill at default setting
			TORQUE = 12'h983; // torque high
			// alternate the cadence signal to simulate pedaling
			// this is what takes a long ass time	
			repeat(10)begin
				cadence = 0;
				repeat(2048)@(posedge clk);
				cadence = 1;
				repeat(2048)@(posedge clk);
				end

			omega_temp2 = iPHYS.omega;
				
			//Check that the omega and curr did increase because the torque increased
			if(!(omega_temp1 < omega_temp2)) begin
				$display("Test #1: Omega did not increase when torque increased.");
				$stop;
			end
				
		// All tests passed if none failed :)
		$display("YAHOO TEST PASSED!");
		$stop();
	end

	// alternate the clock
  	always
		#2 clk = ~clk;
		

endmodule

