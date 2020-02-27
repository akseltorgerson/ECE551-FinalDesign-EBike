module sensorCondition(clk, rst_n, torque, cadence, curr, incline, setting, batt, error, not_pedaling, TX);
	input logic clk, rst_n;
	input logic [11:0] torque; //raw torque signal from A2D_intf
	input logic cadence; //raw (unfiltered) cadence signal
	input logic [11:0] curr; //raw measurement of current through the motor
	input logic [12:0] incline; //positive for uphill, negative for downhill
	input logic [1:0] setting; //assist level setting (off, low, med, high)
	input logic [11:0] batt; //raw battery voltage
	localparam LOW_BATT_THRES = 12'hA98;
	output logic signed [12:0] error; //SIGNED error signal to PID
	output logic not_pedaling; //asserted when cadence_vec < 2;
	output logic TX; //output from telemetry module
	
	//Cadence signals
	logic [24:0] cadence_per; // 25 bit count used for the timer
	logic [4:0] cadence_cnt; //Used to count the rising edges of cadence_filt
	logic clr; //??active high?? used to clear the cadence count and the final flop (comes from cadence_per timer)
	logic cadence_filt; //output of the cadence_filt module
	logic cadence_filt_flopped; //flop the cadence filt once
	logic cadence_rise; //high if rising edge of cadence_filt
	logic not_pedaling_flopped;
	logic not_pedaling_fall; //high if falling edge of not_pedaling
	logic [4:0] cadence_vec; //count the rising edges of cadence_filt
	
	//accumulator signals
	logic [11:0] avg_curr; //calculated avg_curr using the accumulator
	logic [13:0] curr_accum;
	logic curr_accum_full;
	logic [21:0] avg_curr_smpl;
	logic [16:0] torque_accum;
	logic [11:0] avg_torque;
	
	logic [11:0] target_curr;
	
	// FAST SIM
	parameter FAST_SIM = 0;


/********************
* CADENCE VEC STUFF *
*********************/

//cadence_per timer(free running timer that can time out (.67sec period)
	always @(posedge clk, negedge rst_n) begin
		if(!rst_n)
			cadence_per <= 0;
		else
			cadence_per <= cadence_per + 1;
	end
	
	//assign clr = &cadence_per;
	
	/***********
	* FAST_SIM *
	***********/
	generate if (FAST_SIM)
		assign clr = &cadence_per[15:0];
	else
		assign clr = &cadence_per;
	endgenerate
	

	//Produce the cadence_filt signal (CHANGE THE FAST SIM FOR TESTING)
	cadence_filt #(FAST_SIM) iCadence_Filt(.clk(clk), .rst_n(rst_n), .cadence(cadence), .cadence_filt(cadence_filt));
	
	//Flop the cadence filt for rising edge detection
	
	always @(posedge clk, negedge rst_n) begin
		if(!rst_n)
			cadence_filt_flopped <= 0;
		else
			cadence_filt_flopped <= cadence_filt;
	end
	assign cadence_rise = (cadence_filt & ~cadence_filt_flopped);

	//cadence_cnt block (5-bit saturating counter with ASYNCH and SYNCH reset, enabled at rise of filtered cadence signal
	always @(posedge clk, negedge rst_n) begin
		if(!rst_n)
			cadence_cnt <= 0;
		else if(clr)
			cadence_cnt <= 0;
		else if(cadence_rise & !(&cadence_cnt)) //increment count on rising edge of cadence_filt and if the count isn't full
			cadence_cnt <= cadence_cnt + 1;
	end
	
	//cadence_vec flop
	always @(posedge clk, negedge rst_n) begin
		if(!rst_n)
			cadence_vec <= 0;
		else if(clr)
			cadence_vec <= cadence_cnt;
	end
	
	//Assign the not_pedaling signal if cadence_vec < 2
	assign not_pedaling = (cadence_vec < 5'h02) ? 1 : 0;

	//Falling edge detector for not pedaling
	always @(posedge clk, negedge rst_n) begin
		if(!rst_n)
			not_pedaling_flopped <= 0;
		else
			not_pedaling_flopped <= not_pedaling;
	end
	assign not_pedaling_fall = (~not_pedaling & not_pedaling_flopped);
	
	
/************************
* EXPONENTIAL AVG STUFF *
*************************/

	//Current Exponential Average
	
	//counter for sampling curr every 2^22 clocks
	always @(posedge clk, negedge rst_n) begin
		if(!rst_n)
			avg_curr_smpl <= 0;
		else
			avg_curr_smpl <= avg_curr_smpl + 1;
	end
	
	//accumulator for calculating sample average of curr
	always @(posedge clk, negedge rst_n) begin
		if(!rst_n)
			curr_accum <= 0;
		else if (curr_accum_full)
			curr_accum <= ((curr_accum * 3) / 4) + curr;
	end
	generate if (FAST_SIM)
		assign curr_accum_full = &cadence_per[14:0]; 
	else
		assign curr_accum_full = &cadence_per;
	endgenerate
	assign avg_curr = (curr_accum / 4);
	
	//Torque Exponential Average
	
	//accumulator for calculating sample average of torque and seed the accumulator based off of falling edge of not_pedaling
	always @(posedge clk, negedge rst_n) begin
		if(!rst_n)
			torque_accum <= 0;
		else if(not_pedaling_fall)
			torque_accum <= {1'b0, torque, 4'b0000};
		else if (cadence_rise)
			torque_accum <= ((torque_accum * 31) / 32) + torque;
	end
	assign avg_torque = (torque_accum / 32);
	
	
	
/************************
*  DESIRED DRIVE STUFF  *
*************************/
	
	//Instantiate desiredDrive to acquire target_curr
	desiredDrive iDesired(.avg_torque(avg_torque), .cadence_vec(cadence_vec), .incline(incline), .setting(setting), .target_curr(target_curr), .clk(clk));
	
	assign error = ((batt < LOW_BATT_THRES) || not_pedaling) ? 0 : (target_curr - avg_curr);
	
	
	
	//Instantiate telemetry
	telemetry iTelemetry(.batt_v(batt), .avg_curr(avg_curr), .avg_torque(avg_torque), .clk(clk), .rst_n(rst_n), .TX(TX));
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	

endmodule