module brushless(clk, drv_mag, hallGrn, hallYlw, hallBlu, brake_n, duty, selGrn, selYlw, selBlu);
input clk, hallBlu,hallGrn, hallYlw, brake_n;
input [11:0] drv_mag;
output [10:0] duty;
output logic[1:0] selGrn, selBlu, selYlw;

logic [2:0] rotation_state;
//Assign the different hall sensor values to be part of a single register for ease of programming
always @(posedge clk) begin
	rotation_state[0] <= hallBlu;
	rotation_state[1] <= hallYlw;
	rotation_state[2] <= hallGrn;
end
localparam HIGH_Z = 2'b00;
localparam rev_curr = 2'b01;
localparam frwd_curr = 2'b10;
localparam brake = 2'b11;
// Always comb block for each case of the inputs
always_comb begin
    // Brake is the exception to the case statements
	if (~brake_n) begin
		selGrn = brake;
		selYlw = brake;
		selBlu = brake;
	end
	else begin	
	case(rotation_state)
		3'b101: begin
			selGrn = frwd_curr;
			selYlw = rev_curr;
			selBlu = HIGH_Z;
		end
		3'b100: begin
			selGrn = frwd_curr;
			selYlw = HIGH_Z;
			selBlu = rev_curr;
		end
		3'b110: begin
			selGrn = HIGH_Z;
			selYlw = frwd_curr;
			selBlu = rev_curr;
		end
		3'b010: begin
			selGrn = rev_curr;
			selYlw = frwd_curr;
			selBlu = HIGH_Z;
		end
		3'b011: begin
			selGrn = rev_curr;
			selYlw = HIGH_Z;
			selBlu = frwd_curr;
		end
		3'b001: begin
			selGrn = HIGH_Z;
			selYlw = rev_curr;
			selBlu = frwd_curr;
		end
		//The default case should just be assumed HIGH_Z for safety
		default: begin
			selGrn = HIGH_Z;
			selYlw = HIGH_Z;
			selBlu = HIGH_Z;
		end
	endcase
	end
end
// Assign the duty that will drive future module using PWM
assign duty = brake_n ? 11'h400 + drv_mag[11:2] : 11'h600;

endmodule