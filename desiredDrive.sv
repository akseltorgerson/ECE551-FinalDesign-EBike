module desiredDrive(avg_torque, cadence_vec, incline, setting, target_curr, clk);
	input [11:0] avg_torque; // Unsignednumber representing the toque the rider is putting on the cranks (force of their pedaling)
	input [4:0] cadence_vec; // Unsignednumber representing the speed of the rider’s pedaling.
	input signed [12:0] incline; // Signed number
	input [1:0] setting; // unsigned.Represents level of assist motor provides11 => a lot of assist, 10 => medium, 01 => little
	output logic [11:0] target_curr; // Unsignedoutput setting the target current the motor should be running at.  This will go to the PID controller to eventually form the duty cycle the motor driver is run at.
	input logic clk;
	
	logic [11:0] target_curr_preflop;
	logic [13:0] mult_stg_1, mult_stg_1_flopped;
	logic [14:0] mult_stg_2, mult_stg_2_flopped;
	
	
	wire [10:0] incline_factor;
	wire [8:0] incline_lim;
	wire signed [5:0] cadence_factor; 
	wire [12:0] torque_off;
	wire [11:0] torque_pos;
	wire [28:0] assist_prod;
	wire signed [9:0] incline_sat;
	localparam TORQUE_MIN = 12'h380;
	assign incline_sat = (~incline[12]) ? (|incline[11:9] ? 10'b0111_1111_11 :incline[9:0]) : (&incline[11:9] ? incline[9:0] : 10'b1000_0000_00);
	assign incline_factor = incline_sat + 9'd256;
	
	assign incline_lim = (incline_factor[10]) ? incline_factor[8:0] : ~incline_factor[9] ? incline_factor[8:0] :  incline_factor[8] ? 9'b0 : 9'd511;
	assign cadence_factor  =  |cadence_vec[4:1] ?  cadence_vec + 32 : 6'b0000_00;
	assign torque_off = avg_torque - TORQUE_MIN;
	assign torque_pos = torque_off[12] ? 12'h000: torque_off[11:0];
	
	
	assign mult_stg_1 = (torque_pos * setting);
	assign mult_stg_2 = (incline_lim * cadence_factor);
	
	
	assign assist_prod = mult_stg_1_flopped * mult_stg_2_flopped;
	
	
	assign target_curr_preflop = |assist_prod[28:26] ? 12'hFFF : assist_prod[25:14];
	
	always @(posedge clk) begin
		target_curr <= target_curr_preflop;	
		mult_stg_1_flopped <= mult_stg_1;
		mult_stg_2_flopped <= mult_stg_2;		
	end
	
endmodule
	
