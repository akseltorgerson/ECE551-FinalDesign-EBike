module nonoverlap(clk, rst_n,highIn, lowIn,highOut, lowOut);
	input clk, rst_n, highIn, lowIn;
	output reg highOut, lowOut;
	reg highInFlopped, lowInFlopped;
	logic changed, clrCnt, outputsLow;
	logic timeExpired;
	reg [4:0] counter;
	
	//flop highIn
	always @(posedge clk) begin
		highInFlopped <= highIn;
	end
	//flop lowIn
	always @(posedge clk) begin
		lowInFlopped <= lowIn;
	end

	//Counter
	always @(posedge clk) begin
		if (clrCnt)
			counter <= 5'b0;
		else if (&counter)
			timeExpired <= 1;
		else
			counter <= counter + 1;
		end
	
	assign changed = (highIn ^ highInFlopped) || (lowIn ^ lowInFlopped) ? 1 : 0;

	typedef enum logic{IDLE, LOW} state_t;
	state_t currState, nxt_state;

	always @(posedge clk, negedge rst_n) begin
		if (!rst_n)
			currState <= IDLE;
		else 
			currState <= nxt_state;
		end

	always @(posedge clk, negedge rst_n) begin
		if(!rst_n)
			highOut <= 1'b0;
		else if(outputsLow)
			highOut <= 1'b0;
		else 
			highOut <= highIn;
		end
		
	always @(posedge clk, negedge rst_n) begin
		if(!rst_n)
			lowOut <= 1'b0;
		else if(outputsLow)
			lowOut <= 1'b0;
		else 
			lowOut <= lowIn;
		end

	always_comb begin
		outputsLow = 0;
		clrCnt = 0;
		nxt_state = currState;
		
		case(currState) 
			IDLE: if(changed) 
					nxt_state = LOW;
				else 
					clrCnt = 1;
			LOW: if(~timeExpired) 
					outputsLow = 1;
				else if(changed) begin
					outputsLow = 1;
					clrCnt = 1;
					end
				else if (timeExpired) begin
					nxt_state = IDLE;
					clrCnt = 1;
				end
			default:
				nxt_state = IDLE;
			endcase
		
		end
endmodule	
	