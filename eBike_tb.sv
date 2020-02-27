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
	
	localparam FAST_SIM = 1;
	
	//Variables for omega testing
	reg signed [19:0] omega_temp1;
	reg signed [19:0] omega_temp2;
	logic [11:0] avg_curr_temp1;
	logic [11:0] avg_curr_temp2;
  
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
	eBike #(FAST_SIM) iDUT(.clk(clk),.RST_n(RST_n),.A2D_SS_n(A2D_SS_n),.A2D_MOSI(A2D_MOSI),
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
	//UART_rcv iUART(.clk(clk),.rst_n(rst_n),.RX,rdy,rx_data,clr_rdy);
	
	initial begin
	
		integer testcase = 3; //CHANGE THIS GUY TO CHOOSE YOUR TEST
		localparam errorThreshold = 50;
		
	 	clk = 0; // only need to do this once
	 	
	 	//***************************** Test Suite ******************************************
	 	
		//for(testcase = 1; testcase < 8; testcase = testcase + 1) begin

	 	// reset everything
	 	RST_n = 0;
		@(posedge clk);
		@(negedge clk);
		RST_n = 1;
		
	 	//TEST 1 CHECKS THAT WHEN TORQUE INCREASES SO SHOULD OMEGA AND CURR
		
	 	if (testcase ==  1) begin
	 		force iDUT.setting = 2'b10;
			BATT = 12'hB11; // battery not low
			BRAKE = 12'h900; // brake not asserted (LOW)
			TORQUE = 12'h400; // torque low
			YAW_RT = 16'h2000; // uphill
				
			// alternate the cadence signal to simulate pedaling
			// this is what takes a long ass time	
			repeat(2048)begin
				cadence = 0;
				repeat(2048)@(posedge clk);
				cadence = 1;
				repeat(2048)@(posedge clk);
				end
				
				//Get the current omega value when before changing anything
				if(iDUT.iSensorCondition.error > errorThreshold || iDUT.iSensorCondition.error < -errorThreshold) begin
					omega_temp1 = iPHYS.omega;
					avg_curr_temp1 = iDUT.iSensorCondition.avg_curr;
					$display("Error value: %d", iDUT.iSensorCondition.error); 
					$display("Test #%d: Avg current never approached the target current.", testcase);
					$stop;
				end
				
			//Test with high torque and uphill at default setting
			TORQUE = 12'h983; // torque high
			// alternate the cadence signal to simulate pedaling
			// this is what takes a long ass time	
			repeat(2048)begin
				cadence = 0;
				repeat(2048)@(posedge clk);
				cadence = 1;
				repeat(2048)@(posedge clk);
				end
				
				//Get the current omega after the change
			if(iDUT.iSensorCondition.error > errorThreshold || iDUT.iSensorCondition.error < -errorThreshold) begin
				omega_temp2 = iPHYS.omega;
				avg_curr_temp2 = iDUT.iSensorCondition.avg_curr;
				$display("Error value: %d", iDUT.iSensorCondition.error); 
				$display("Test #%d: Avg current never approached the target current.", testcase);
				$stop;
			end
			
			//Check that the omega and curr did increase because the torque increased
			if(!(omega_temp1 < omega_temp2)) begin
				$display("Test #%d: Omega did not increase when torque increased.", testcase);
				$stop;
			end
			
			if(!(avg_curr_temp1 < avg_curr_temp2)) begin
				$display("Test #%d: Avg_curr did not increase when torque increased.", testcase);
				$stop;
			end
			
		end		
			
			
			
			
			
			//TEST 2 CHECKS THAT IF CADENCE GOES TO ZERO THE OMEGA SHOULD DECREASE TO NEARLY ZERO
			
			//Test with high torque uphill with cadence
		if (testcase ==  2) begin
			force iDUT.setting = 2'b10;
			BATT = 12'hB11; // battery not low
			BRAKE = 12'h900; // brake not asserted (LOW)
			TORQUE = 12'h983; // torque high
			YAW_RT = 16'h2000; // uphill
				
			// alternate the cadence signal to simulate pedaling
			// this is what takes a long ass time	
		
				repeat(2048)begin
					cadence = 0;
					repeat(2048)@(posedge clk);
					cadence = 1; // CADENCE HERE
					repeat(2048)@(posedge clk);
					end
					
					//Get the current omega value when before changing anything
				if(iDUT.iSensorCondition.error > errorThreshold || iDUT.iSensorCondition.error < -errorThreshold) begin
					omega_temp1 = iPHYS.omega;
					avg_curr_temp1 = iDUT.iSensorCondition.avg_curr;
					$display("Error value: %d", iDUT.iSensorCondition.error); 
					$display("Test #%d: Avg current never approached the target current.", testcase);
					$stop;
				end
			
			//Test with high torque uphill and NO CADENCE
			// alternate the cadence signal to simulate pedaling
			// this is what takes a long ass time	
			
				repeat(2048)begin
					cadence = 0;
					repeat(2048)@(posedge clk);
					cadence = 0; // NO CADENCE HERE
					repeat(2048)@(posedge clk);
					end
					
						//Get the current omega after the change
			if(iDUT.iSensorCondition.error > errorThreshold || iDUT.iSensorCondition.error < -errorThreshold) begin
				omega_temp2 = iPHYS.omega;
				avg_curr_temp2 = iDUT.iSensorCondition.avg_curr;
				$display("Error value: %d", iDUT.iSensorCondition.error); 
				$display("Test #%d: Avg current never approached the target current.", testcase);
				$stop;
			end
			
			//Check that the omega did decrease because the cadence went to zero
			if(!(omega_temp1 > omega_temp2)) begin
				$display("Test #%d: Omega did not decrease when cadence was zero.", testcase);
				$stop;
			end
			
			if(!(avg_curr_temp1 > avg_curr_temp2)) begin
				$display("Test #%d: Avg_curr did not decrease when cadence became zero.", testcase);
				$stop;
			end
			
				
		end	
			
			
			
			//TEST 3 CHECKS WHEN GOING FROM UPHILL TO DOWNHILL IF OMEGA DECREASES
			
		if (testcase ==  3) begin
	 		force iDUT.setting = 2'b10;
			BATT = 12'hB11; // battery not low
			BRAKE = 12'h900; // brake not asserted (LOW)
			TORQUE = 12'h600; // torque normal
			YAW_RT = 16'h2000; // uphill
				
			// alternate the cadence signal to simulate pedaling
			// this is what takes a long ass time	
			
				repeat(2048)begin
					cadence = 0;
					repeat(2048)@(posedge clk);
					cadence = 1;
					repeat(2048)@(posedge clk);
					end
					
					//Get the current omega value when before changing anything
				if(iDUT.iSensorCondition.error > errorThreshold || iDUT.iSensorCondition.error < -errorThreshold) begin
					omega_temp1 = iPHYS.omega;
					avg_curr_temp1 = iDUT.iSensorCondition.avg_curr;
					$display("Error value: %d", iDUT.iSensorCondition.error); 
					$display("Test #%d: Avg current never approached the target current.", testcase);
					$stop;
				end
		
			
			//Now lets go downhill
			YAW_RT = 16'hA800; // downhill?
			repeat(3000000) @(posedge clk); //wait 3,000,000 clks
			YAW_RT = 16'h0000;
				
			// alternate the cadence signal to simulate pedaling
			// this is what takes a long ass time	
				repeat(2048)begin
					cadence = 0;
					repeat(2048)@(posedge clk);
					cadence = 1;
					repeat(2048)@(posedge clk);
					end
					
						//Get the current omega and avg_curr after the change
			if(iDUT.iSensorCondition.error > errorThreshold || iDUT.iSensorCondition.error < -errorThreshold) begin
				omega_temp2 = iPHYS.omega;
				avg_curr_temp2 = iDUT.iSensorCondition.avg_curr;
				$display("Error value: %d", iDUT.iSensorCondition.error); 
				$display("Test #%d: Avg current never approached the target current.", testcase);
				$stop;
			end
			
			//Check that the omega decreased because now its going downhill
			if(!(omega_temp1 > omega_temp2)) begin
				$display("Test #%d: Omega did not decrease when going downhill.", testcase);
				$stop;
			end
			
			if(!(avg_curr_temp1 > avg_curr_temp2)) begin
				$display("Test #%d: Avg_curr did not decrease when going from uphill to downhill.", testcase);
				$stop;
			end
			
					
		end





		//TEST 4 CHECKS THAT WHEN YOU INCREASE SETTING THE CURRENT AND OMEGA INCREASE
		if (testcase ==  4) begin
	 		force iDUT.setting = 2'b01;
			BATT = 12'hB11; // battery not low
			BRAKE = 12'h900; // brake not asserted (LOW)
			TORQUE = 12'h600; // torque normal
			YAW_RT = 16'h2000; // uphill
				
			// alternate the cadence signal to simulate pedaling
			// this is what takes a long ass time	
			
				repeat(2048)begin
					cadence = 0;
					repeat(2048)@(posedge clk);
					cadence = 1;
					repeat(2048)@(posedge clk);
					end
					
					//Get the current omega value when before changing anything
				if(iDUT.iSensorCondition.error > errorThreshold || iDUT.iSensorCondition.error < -errorThreshold) begin
					omega_temp1 = iPHYS.omega;
					avg_curr_temp1 = iDUT.iSensorCondition.avg_curr;
					$display("Error value: %d", iDUT.iSensorCondition.error); 
					$display("Test #%d: Avg current never approached the target current.", testcase);
					$stop;
				end
		
			
			//Time to go fast
				force iDUT.setting = 2'b11;
			// alternate the cadence signal to simulate pedaling
			// this is what takes a long ass time	
				repeat(2048)begin
					cadence = 0;
					repeat(2048)@(posedge clk);
					cadence = 1;
					repeat(2048)@(posedge clk);
					end
					
						//Get the current omega after the change
			if(iDUT.iSensorCondition.error > errorThreshold || iDUT.iSensorCondition.error < -errorThreshold) begin
				omega_temp2 = iPHYS.omega;
				avg_curr_temp2 = iDUT.iSensorCondition.avg_curr;
				$display("Error value: %d", iDUT.iSensorCondition.error); 
				$display("Test #%d: Avg current never approached the target current.", testcase);
				$stop;
			end
			
			//Check that the omega increases because setting increased
			if(!(omega_temp1 < omega_temp2)) begin
				$display("Test #%d: Omega did not increase when setting increased.", testcase);
				$stop;
			end
			
			if(!(avg_curr_temp1 < avg_curr_temp2)) begin
				$display("Test #%d: Avg_curr did not increase when setting increased.", testcase);
				$stop;
			end
			
					
		end
		
		
		
		
		
		
		//TEST 5 CHECKS IF WHEN THE INCLINE INCREASES SO DOES THE OMEGA AND CURR
		
		if (testcase ==  5) begin
	 		force iDUT.setting = 2'b01;
			BATT = 12'hB11; // battery not low
			BRAKE = 12'h900; // brake not asserted (LOW)
			TORQUE = 12'h600; // torque normal
			YAW_RT = 16'h2000; // uphill
				
			// alternate the cadence signal to simulate pedaling
			// this is what takes a long ass time	
			
				repeat(2048)begin
					cadence = 0;
					repeat(2048)@(posedge clk);
					cadence = 1;
					repeat(2048)@(posedge clk);
					end
					
					//Get the current omega value when before changing anything
				if(iDUT.iSensorCondition.error > errorThreshold || iDUT.iSensorCondition.error < -errorThreshold) begin
					omega_temp1 = iPHYS.omega;
					avg_curr_temp1 = iDUT.iSensorCondition.avg_curr;
					$display("Error value: %d", iDUT.iSensorCondition.error); 
					$display("Test #%d: Avg current never approached the target current.", testcase);
					$stop;
				end
		
			
			//Time to go up a big hill
				YAW_RT = 16'h8000;
				
			// alternate the cadence signal to simulate pedaling
			// this is what takes a long ass time	
				repeat(2048)begin
					cadence = 0;
					repeat(2048)@(posedge clk);
					cadence = 1;
					repeat(2048)@(posedge clk);
					end
					
						//Get the current omega after the change
			if(iDUT.iSensorCondition.error > errorThreshold || iDUT.iSensorCondition.error < -errorThreshold) begin
				omega_temp2 = iPHYS.omega;
				avg_curr_temp2 = iDUT.iSensorCondition.avg_curr;
				$display("Error value: %d", iDUT.iSensorCondition.error); 
				$display("Test #%d: Avg current never approached the target current.", testcase);
				$stop;
			end
			
			//Check that the omega increased because its going uphill
			if(!(omega_temp1 < omega_temp2)) begin
				$display("Test #%d: Omega did not increase when incline increased.", testcase);
				$stop;
			end
			
			if(!(avg_curr_temp1 < avg_curr_temp2)) begin
				$display("Test #%d: Avg_curr did not increase when incline increased.", testcase);
				$stop;
			end
					
		end




	//TEST 6 CHECKS IF THE OMEGA INCREASES WHEN GOING FROM DOWNHILL TO UPHILL

	if(testcase == 6) begin
		force iDUT.setting = 2'b10;
			BATT = 12'hB11; // battery not low
			BRAKE = 12'h900; // brake not asserted (LOW)
			TORQUE = 12'h600; // torque normal
			YAW_RT = 16'hA800; // downhill?
			repeat(3000000) @(posedge clk); //wait 3,000,000 clks
			YAW_RT = 16'h0000;
				
			// alternate the cadence signal to simulate pedaling
			// this is what takes a long ass time	
			
				repeat(2048)begin
					cadence = 0;
					repeat(2048)@(posedge clk);
					cadence = 1;
					repeat(2048)@(posedge clk);
					end
					
					//Get the current omega value when before changing anything
				if(iDUT.iSensorCondition.error > errorThreshold || iDUT.iSensorCondition.error < -errorThreshold) begin
					omega_temp1 = iPHYS.omega;
					avg_curr_temp1 = iDUT.iSensorCondition.avg_curr;
					$display("Error value: %d", iDUT.iSensorCondition.error); 
					$display("Test #%d: Avg current never approached the target current.", testcase);
					$stop;
				end
		
			
			//Now lets go uphill
			YAW_RT = 16'h8000;
				
			// alternate the cadence signal to simulate pedaling
			// this is what takes a long ass time	
				repeat(2048)begin
					cadence = 0;
					repeat(2048)@(posedge clk);
					cadence = 1;
					repeat(2048)@(posedge clk);
					end
					
						//Get the current omega and avg_curr after the change
			if(iDUT.iSensorCondition.error > errorThreshold || iDUT.iSensorCondition.error < -errorThreshold) begin
				omega_temp2 = iPHYS.omega;
				avg_curr_temp2 = iDUT.iSensorCondition.avg_curr;
				$display("Error value: %d", iDUT.iSensorCondition.error); 
				$display("Test #%d: Avg current never approached the target current.", testcase);
				$stop;
			end
			
			//Check that the omega increased because now its going uphill
			if(!(omega_temp1 < omega_temp2)) begin
				$display("Test #%d: Omega did not increase when going uphill.", testcase);
				$stop;
			end
			
			if(!(avg_curr_temp1 < avg_curr_temp2)) begin
				$display("Test #%d: Avg_curr did not increase when going from downhill to uphill.", testcase);
				$stop;
			end
		end
			
		//TEST 7 CHECKS IF WHEN THE CADENCE RATE IS INCREASED SO DOES THE OMEGA AND CURR
			
			if(testcase == 7) begin
			force iDUT.setting = 2'b10;
			BATT = 12'hB11; // battery not low
			BRAKE = 12'h900; // brake not asserted (LOW)
			TORQUE = 12'h600; // torque normal
			YAW_RT = 16'hA800; // downhill?
			repeat(3000000) @(posedge clk); //wait 3,000,000 clks
			YAW_RT = 16'h0000;
				
			// alternate the cadence signal to simulate pedaling
			// this is what takes a long ass time	
			
				repeat(2048)begin
					cadence = 0;
					repeat(2048)@(posedge clk);
					cadence = 1;
					repeat(2048)@(posedge clk);
					end
					
					//Get the current omega value when before changing anything
				if(iDUT.iSensorCondition.error > errorThreshold || iDUT.iSensorCondition.error < -errorThreshold) begin
					omega_temp1 = iPHYS.omega;
					avg_curr_temp1 = iDUT.iSensorCondition.avg_curr;
					$display("Error value: %d", iDUT.iSensorCondition.error); 
					$display("Test #%d: Avg current never approached the target current.", testcase);
					$stop;
				end
		
			
			//Now lets go change the cadence rate
				
			// alternate the cadence signal to simulate pedaling
			// this is what takes a long ass time	
				repeat(2048)begin
					cadence = 0;
					repeat(1024)@(posedge clk);
					cadence = 1;
					repeat(1024)@(posedge clk);
					end
					
						//Get the current omega and avg_curr after the change
			if(iDUT.iSensorCondition.error > errorThreshold || iDUT.iSensorCondition.error < -errorThreshold) begin
				omega_temp2 = iPHYS.omega;
				avg_curr_temp2 = iDUT.iSensorCondition.avg_curr;
				$display("Error value: %d", iDUT.iSensorCondition.error); 
				$display("Test #%d: Avg current never approached the target current.", testcase);
				$stop;
			end
			
			//Check that the omega increased because cadence rate increased
			if(!(omega_temp1 < omega_temp2)) begin
				$display("Test #%d: Omega did not increase when cadence increased.", testcase);
				$stop;
			end
			//Check that avg_curr also increased as cadence rate increased
			if(!(avg_curr_temp1 < avg_curr_temp2)) begin
				$display("Test #%d: Avg_curr did not increase when cadence increased.", testcase);
				$stop;
			end
		end
		
		
		//TEST 8 CHECKS IF WHEN THE BRAKES ARE APPLIED THE OMEGA AND CURR DROP
		
		if(testcase == 8) begin
			force iDUT.setting = 2'b10;
			BATT = 12'hB11; // battery not low
			BRAKE = 12'h900; // brake not asserted (LOW)
			TORQUE = 12'h600; // torque normal
			YAW_RT = 16'h2000; // uphill
				
			// alternate the cadence signal to simulate pedaling
			// this is what takes a long ass time	
			
				repeat(2048)begin
					cadence = 0;
					repeat(2048)@(posedge clk);
					cadence = 1;
					repeat(2048)@(posedge clk);
					end
					
				//Get the current omega value when before changing anything
				omega_temp1 = iPHYS.omega;
				avg_curr_temp1 = iDUT.iSensorCondition.avg_curr;
		
			//Now apply the brakes!
			force iDUT.brake_n = 0;
				
			// alternate the cadence signal to simulate pedaling
			// this is what takes a long ass time	
				repeat(2048)begin
					cadence = 0;
					repeat(2048)@(posedge clk);
					cadence = 1;
					repeat(2048)@(posedge clk);
					end

				//Get the current omega and avg_curr after the change
				omega_temp2 = iPHYS.omega;
				avg_curr_temp2 = iDUT.iSensorCondition.avg_curr;

			
			//Check that the omega decreased
			if(!(omega_temp1 > omega_temp2)) begin
				$display("Test #%d: Omega did not decrease when brakes applied.", testcase);
				$stop;
			end
			//Check that avg_curr also decreased
			if(!(avg_curr_temp1 > avg_curr_temp2)) begin
				$display("Test #%d: Avg_curr did not decrease when brakes applied.", testcase);
				$stop;
			end
		end


		//end
			
		
	
		// All tests passed if none failed :)
		$display("YAHOO TEST PASSED!");
		$stop();
	end

	// alternate the clock
  	always
		#10 clk = ~clk;
		

endmodule

