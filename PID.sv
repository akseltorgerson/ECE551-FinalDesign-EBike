module PID(clk, rst_n, error, not_pedaling, drv_mag);
	input logic clk, rst_n, not_pedaling;
	input logic [12:0] error;
	output logic [11:0] drv_mag;
	
	parameter FAST_SIM = 0;
	
	// pterm stuff
	logic [13:0] pterm, pterm_flopped;
	
	// iterm stuff
	logic [11:0] iterm;
	logic [17:0] errorSext;
	logic [17:0] integrator;
	logic [17:0] runningSum;
	logic [17:0] negCheck;
	logic [17:0] overflowCheck;
	logic [17:0] decimatedSignal;
	logic [17:0] pedalCheck;
	
	// decimator stuff
	logic [19:0] decimatorCounter;
	logic decimator_full;
	
	// dterm stuff
	logic  [9:0] dterm;
	logic [12:0] firstReading, firstReadingFlopped;
	logic [12:0] secondReading, secondReadingFlopped; 
	logic [12:0] thirdReading, prev_err;
	logic [12:0] D_diff;
	logic [8:0] D_diffSat;
	
	// putting it together
	logic [13:0] itermZext, itermZext_flopped;
	logic [13:0] dtermSext, dtermSext_flopped;
	logic [13:0] PIDsum, PIDsum_preFlop;
	logic [11:0] PIDsat;
		
	/****************************** PTERM *************************************************/
	
	assign pterm = {{error[12]}, error};
	
	/****************************** ITERM *************************************************/
	
	// sign extends error to be 18 bits
	assign errorSext = {{5{error[12]}}, error};
	// sums the error and the previous value
	assign runningSum = integrator + errorSext;
	// checks the MSB to see if it is negative, if it is, set it to 0
	assign negCheck = runningSum[17] ? 18'h00000 : runningSum;
	// check to see if the result has overflowed, if so, saturate the value at 1FFFF
		// TODO IS THIS SUPPOSED TO BE [17] & [16] here?? Does this mean overflow?
	assign overflowCheck = runningSum[17] & integrator[16] ? 18'h1FFFF : negCheck;
	// we must wait to pass the new signal to when our decimatorCounter is full
	assign decimatedSignal = decimator_full ? overflowCheck : integrator; 
	// check if pedaling is happening or not
	assign pedalCheck = not_pedaling ? 18'h00000 : decimatedSignal;
	
	// use a flop to propogate the value through on a clock edge
	always @(posedge clk, negedge rst_n) begin
		if (~rst_n)
			integrator <= 18'h00000;
		else 
			integrator <= pedalCheck;
	end
	
	// assign iterm to be bits [16:5] of the integrator
	assign iterm = integrator[16:5];		
	
	/***************************** DTERM *****************************************************/
	
	assign firstReading = decimator_full ? error : firstReadingFlopped;
	
	always @(posedge clk, negedge rst_n) begin
		if (~rst_n)
			firstReadingFlopped <= 0;
		else
			firstReadingFlopped <= firstReading;
	end
	
	assign secondReading = decimator_full ? firstReadingFlopped : secondReadingFlopped;
	
	always @(posedge clk, negedge rst_n) begin
		if (~rst_n)
			secondReadingFlopped <= 0;
		else
			secondReadingFlopped <= secondReading;
	end
	
	assign thirdReading = decimator_full ? secondReadingFlopped : prev_err;
	
	always @(posedge clk, negedge rst_n) begin
		if (~rst_n)
			prev_err <= 0;
		else
			prev_err <= thirdReading;
	end
	
	assign D_diff = error - prev_err;
	
	//assign D_diffSat = |D_diff[12:9] ? 9'b111111111 : D_diff[8:0]; // TODO We changed from this to a signed saturation
	assign D_diffSat = (~D_diff[12]) ? (|D_diff[11:8] ? 9'b0111_1111_1 : D_diff[9:0]) : (&D_diff[11:8] ? D_diff[8:0] : 9'b1000_0000_0);
	
	assign dterm = {D_diffSat, 1'b0};
	
	/***************************** DECIMATOR **************************************************/
	
	// decimator
	// pulse once every 1/48th of a second
	// corresponds to a 20 bit counter reaching max val @ 50MHz clk
	always @(posedge clk, negedge rst_n) begin
		if (~rst_n)
			decimatorCounter <= 0;
		else
			decimatorCounter <= decimatorCounter + 1;
	end
	
	generate if (FAST_SIM)
		assign decimator_full = &decimatorCounter[14:0];
	else
		assign decimator_full = &decimatorCounter;
	endgenerate
	
	/***************************** PUTTING IT TOGETHER ***************************************/
	
	assign itermZext = {{2{1'b0}}, iterm};
	assign dtermSext = {{4{dterm[9]}}, dterm};
	assign PIDsum_preFlop = pterm + (itermZext + dtermSext);
	always @(posedge clk)
		PIDsum <= PIDsum_preFlop;
	
	assign PIDsat = PIDsum[12] ? 12'hFFF : PIDsum[11:0];
	
	assign drv_mag = PIDsum[13] ? 12'h000 : PIDsat;
		
	
endmodule
