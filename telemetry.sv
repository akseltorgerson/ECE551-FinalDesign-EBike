module telemetry(batt_v, avg_curr,  avg_torque, clk, rst_n, TX);
	input clk, rst_n;
	input [11:0] batt_v, avg_curr, avg_torque;
	logic [7:0] tx_data;
	logic tx_done, trmt;
	output TX;
	reg [19:0] counter;
	typedef enum reg[3:0] {IDLE, DELIM1, PAYLOAD1, PAYLOAD2,PAYLOAD3,PAYLOAD4,PAYLOAD5,PAYLOAD6, WAIT} state_t;
	state_t state, nxt_state;
	UART_tx UART_tx(.clk(clk),.rst_n(rst_n),.TX(TX),.trmt(trmt),.tx_data(tx_data),.tx_done(tx_done));
	always_ff @(posedge clk, negedge rst_n) begin
		if (!rst_n)
			state <= IDLE;
		else	
			state <= nxt_state;
	end
	always @(posedge clk, negedge rst_n) begin
		if(!rst_n)
			counter <= 20'b0;
		else
			counter <= counter + 1;
	end
	
	always_comb begin
		tx_data = 8'b0;
		nxt_state = state;
		trmt = 0;
		case(state) 
			IDLE: begin
				tx_data = 8'hAA;
				trmt = 1;
				nxt_state = DELIM1;
				end
			DELIM1:if (tx_done) begin
			
				tx_data = 8'h55;
				trmt = 1;
				nxt_state = PAYLOAD1;
			end
			PAYLOAD1: if (tx_done)begin
				tx_data = {4'b0, batt_v[11:8]};
				trmt = 1;
				
				nxt_state = PAYLOAD2;
				end
			PAYLOAD2: if (tx_done) begin
				tx_data = batt_v[7:0];
				trmt = 1;
				nxt_state = PAYLOAD3;
				end
			PAYLOAD3: if (tx_done)begin
				tx_data = {4'h0, avg_curr[11:8]};
				trmt = 1;
				nxt_state = PAYLOAD4;
				end
			PAYLOAD4: if (tx_done)begin
				tx_data = avg_curr[7:0];
				trmt = 1;
				nxt_state = PAYLOAD5;
				end
			PAYLOAD5: if (tx_done)begin
				tx_data = {4'h0, avg_torque[11:8]};
				trmt = 1;
				nxt_state = PAYLOAD6;
				end
			PAYLOAD6: if (tx_done)begin
				tx_data = avg_torque[7:0];
				trmt = 1;
				nxt_state = WAIT;
				end
			WAIT: begin 
				if (&counter) 
					nxt_state = IDLE;
					end
			default:
				nxt_state = IDLE;
			endcase
	end
endmodule
			