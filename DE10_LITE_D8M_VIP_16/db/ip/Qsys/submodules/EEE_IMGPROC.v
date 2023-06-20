  module EEE_IMGPROC(
	// global clock & reset
	clk,
	reset_n,
	
	// mm slave
	s_chipselect,
	s_read,
	s_write,
	s_readdata,
	s_writedata,
	s_address,

	// stream sink
	sink_data,
	sink_valid,
	sink_ready,
	sink_sop,
	sink_eop,
	
	// streaming source
	source_data,
	source_valid,
	source_ready,
	source_sop,
	source_eop,
	
	// conduit
	mode
	
);


// global clock & reset
input	clk;
input	reset_n;

// mm slave
input							s_chipselect;
input							s_read;
input							s_write;
output	reg	[31:0]	s_readdata;
input	[31:0]				s_writedata;
input	[2:0]					s_address;


// streaming sink
input	[23:0]            	sink_data;
input								sink_valid;
output							sink_ready;
input								sink_sop;
input								sink_eop;

// streaming source
output	[23:0]			  	   source_data;
output								source_valid;
input									source_ready;
output								source_sop;
output								source_eop;

// conduit export
input                         mode;

////////////////////////////////////////////////////////////////////////
//
parameter IMAGE_W = 11'd640;
parameter IMAGE_H = 11'd480;
parameter MESSAGE_BUF_MAX = 256;
parameter MSG_INTERVAL = 20;
parameter BB_COL_DEFAULT = 24'h00ff00;


wire [7:0]   red, green, blue, grey;
wire [7:0]   red_out, green_out, blue_out;

wire         sop, eop, in_valid, out_ready;
////////////////////////////////////////////////////////////////////////

//converting rgb to hsv
reg [7:0] max_val, min_val, delta;
reg [7:0] hue, saturation, value;
reg [7:0] l_hue, l_saturation, l_value;
reg [7:0] ll_hue, ll_saturation, ll_value;

always @(*) begin
	max_val = (red > green) ? ((red > blue) ? red : blue) : ((green > blue) ? green : blue);
   min_val = (red < green) ? ((red < blue) ? red : blue) : ((green < blue) ? green : blue);
   delta = max_val - min_val;

   // Calculate hue component
   if (delta == 0) begin
       hue = 0;
   end else if (max_val == red) begin
       hue = ((green < blue) ? (green - blue + 256) : (green - blue)) * 43 / delta;
   end else if (max_val == green) begin
       hue = ((blue < red ? (blue - red + 256) : (blue - red)) * 43 / delta) + 85;
   end else begin
       hue = ((red < green ? (red - green + 256) : (red - green)) * 43 / delta) + 171;
   end

   // Calculate saturation component
   if (max_val == 0) begin
       saturation = 0;
   end else begin
       saturation = (delta * 255) / max_val;
   end

	// Calculate value component
   value = max_val;
end
   
always@(posedge clk) begin
	ll_hue <= l_hue;
	ll_saturation <= l_saturation;
	ll_value <= l_value;
	l_hue <= hue;
	l_saturation <= saturation;
	l_value <= value;	
end


// Detect red areas
wire red_detect;
wire yellow_detect;
wire green_detect;
wire white_detect;
wire [7:0] avg_hue, avg_saturation, avg_value;
assign avg_hue = ll_hue[7:2]+l_hue[7:1]+hue[7:2];
assign avg_saturation = ll_saturation[7:2]+l_saturation[7:1]+saturation[7:2];
assign avg_value = ll_value[7:2]+l_value[7:1]+value[7:2];
wire detected;
/*
assign red_detect = (avg_hue <= 8'd13) & (avg_saturation > 8'd120) & (avg_value > 8'd70);
assign yellow_detect = (avg_hue > 8'd13) & (avg_hue < 8'd35) & (avg_saturation > 8'd110) & (avg_value > 8'd50);
assign green_detect =  (avg_hue > 8'd150) & (avg_hue < 8'd240) & (avg_saturation > 8'd90) & (avg_value > 8'd50);
*/
assign red_detect = (hue <= 8'd13) & (saturation > 8'd120) & (value > 8'd70);
assign yellow_detect = (hue > 8'd13) & (hue < 8'd35) & (saturation > 8'd110) & (value > 8'd50);
assign green_detect =  (hue > 8'd140) & (hue < 8'd240) & (saturation > 8'd90) & (value > 8'd50);
//assign green_detect = (red < 8'd70) & (green < 8'd70) & (blue > 8'd100);

//assign white_detect = (red > 8'd35 & red < 8'd140)&(green>8'd45 & green < 8'd170)&(blue>8'd18&blue<8'd100);
assign white_detect = (red > 8'd30 & red < 8'd150) &  (green > 8'd40 & green <8'd210) & (blue > 8'd30 & blue < 8'd110);
//assign white_detect = 1'b0;
assign detected = red_detect | yellow_detect | green_detect | white_detect;

// Find boundary of cursor box

// Highlight detected areas
wire [23:0] red_high;
assign grey = green[7:1] + red[7:2] + blue[7:2]; //Grey = green/2 + red/4 + blue/4
//assign red_high  =  red_detect ? {8'hff, 8'hff, 8'hff} : {8'h00, 8'h00, 8'h00}
assign red_high = (red_detect) ? {8'hff, 8'h00, 8'h00} :
                  (yellow_detect) ? {8'hff, 8'hff, 8'h00} :
                  (green_detect) ? {8'h00, 8'h00, 8'hff} :
                  (white_detect) ? {8'hff, 8'hff, 8'hff} :
                  {grey, grey, grey};
						
//assign red_high = (x_line_active|y_line_active) ? {8'h00, 8'h00, 8'h00} : {8'hff, 8'hff, 8'hff};
						
// Show bounding box
wire [23:0] new_image;
wire bb_active_r;
wire bb_active_y;
wire bb_active_g;
//wire bb_active_w;
assign bb_active_r = (x == left) | (x == right) | (y == top) | (y == bottom) | (x == mid);
assign bb_active_y = (x == y_left) | (x == y_right) | (y == y_top) | (y == y_bottom) | ( x == y_mid );
assign bb_active_g = (x == g_left) | (x == g_right) | (y == g_top) | (y == g_bottom) | (x == g_mid);
//assign bb_active_w = (x == wl_left) | (x == wl_right) | (y == wl_top) | (y == wl_bottom) | (x == wr_left) | (x == wr_right);
//assign bb_active_wl = (x == 11'd10) | ( y == 11'd10 );
//assign new_image = (bb_active_r | bb_active_y | bb_active_g) ? 24'hffffff : red_high;//bb_col

//detect white lines
reg [479:0] y_w_detect;
reg [639:0] x_w_detect;
reg x_line_active;
reg y_line_active;

always@(posedge clk) begin
	if (white_detect == 1'b1 & x_w_detect[x] == 1'b0 & y >= 11'd270 & ( x > 11'd5 & x < 11'd635) & x[3:0] == 4'b0001) begin
		x_w_detect[x] <= 1'b1;
		x_line_active <= 1'b1;
	end
	else if (white_detect == 1'b1 & x_w_detect[x] == 1'b1) begin
		x_line_active <= 1'b0;
	end
	else if (white_detect == 1'b0 | x[1:0] != 2'b01) begin
		x_w_detect[x] <= 1'b0;
		x_line_active <= 1'b0;
	end
end

always@(posedge clk) begin
	if (white_detect == 1'b1 & y_w_detect[y] == 1'b0 & y >= 11'd270 & ( x > 11'd5 & x < 11'd635) & y[3:0] == 4'b0001) begin
		y_w_detect[y] <= 1'b1;
		y_line_active <= 1'b1;
	end
	else if (white_detect == 1'b1 & y_w_detect[y] == 1'b1) begin
		y_line_active <= 1'b0;
	end
	else if (white_detect == 1'b0 | y[1:0] != 2'b01) begin
		y_w_detect[y] <= 1'b0;
		y_line_active <= 1'b0;
	end
end

assign new_image = bb_active_r ? 24'hff0000 :
						 bb_active_y ? 24'hffff00 :
						 bb_active_g ? 24'h0000ff :
						 (x_line_active | y_line_active) ? 24'h00ff00 :
						 red_high;
						 
//assign new_image = (x_line_active|y_line_active) ? {8'h00, 8'h00, 8'h00} : {8'hff, 8'hff, 8'hff};

// Switch output pixels depending on mode switch
// Don't modify the start-of-packet word - it's a packet discriptor
// Don't modify data in non-video packets
assign {red_out, green_out, blue_out} = (~mode & ~sop & packet_video) ? new_image : {red,green,blue};
//assign {red_out, green_out, blue_out} = (mode & ~sop & packet_video) ? new_image : 8'hffffff;

//Count valid pixels to tget the image coordinates. Reset and detect packet type on Start of Packet.
reg [10:0] x, y;
reg packet_video;
always@(posedge clk) begin
	if (sop) begin
		x <= 11'h0;
		y <= 11'h0;
		packet_video <= (blue[3:0] == 3'h0);
	end
	else if (in_valid) begin
		if (x == IMAGE_W-1) begin
			x <= 11'h0;
			y <= y + 11'h1;
		end
		else begin
			x <= x + 11'h1;
		end
	end
end

reg [3:0] c_counter, r_counter, g_counter, y_counter, w_counter, counter;
always@(posedge clk) begin
	c_counter <= c_counter + 4'b1;
	if (red_detect & !white_detect) r_counter <= r_counter + 4'b1;
	if (yellow_detect & !white_detect) y_counter <= y_counter + 4'b1;
	if (green_detect & !white_detect) g_counter <= g_counter + 4'b1;
	if (white_detect) w_counter <= w_counter + 4'b1;
	if (detected) counter <= counter + 4'b1;
	if (c_counter == 4'b0) begin
		r_counter <= 4'b0;
		y_counter <= 4'b0;
		g_counter <= 4'b0;
		w_counter <= 4'b0;
		counter <= 4'b0;
	end
end

//Find first and last red pixels
reg [10:0] x_min, y_min, x_max, y_max, x_mid;
always@(posedge clk) begin
	if (red_detect & !white_detect & in_valid & (r_counter > 4'hd | ((4'hf - counter > 4'h8)&r_counter > 4'h5)) ) begin	//Update bounds when the pixel is red
		if (x < x_min) x_min <= x;
		if (x > x_max) x_max <= x;
		if (y < y_min) y_min <= y;
		if (y > y_max) y_max <= y;
		//x_mid <= x_min[10:1] + x_min[10:1];
	end
	if (sop & in_valid) begin	//Reset bounds on start of packet
		x_min <= IMAGE_W-11'h1;
		x_max <= 0;
		y_min <= IMAGE_H-11'h1;
		y_max <= 0;
		x_mid <= 0;
	end
end

//yellow pixels
reg [10:0] y_x_min, y_y_min, y_x_max, y_y_max, y_x_mid;
always@(posedge clk) begin
	if (yellow_detect & !white_detect & in_valid & (y_counter > 4'hd| ((4'hf - counter > 4'h8)&y_counter > 4'h5)) ) begin	//Update bounds when the pixel is red
		if (x < y_x_min) y_x_min <= x;
		if (x > y_x_max) y_x_max <= x;
		if (y < y_y_min) y_y_min <= y;
		if (y > y_y_max) y_y_max <= y;
		//y_x_mid <= y_x_min[10:1] + y_x_max[10:1];
	end
	if (sop & in_valid) begin	//Reset bounds on start of packet
		y_x_min <= IMAGE_W-11'h1;
		y_x_max <= 0;
		y_y_min <= IMAGE_H-11'h1;
		y_y_max <= 0;
		y_x_mid <= 0;
	end
end

//green pixels
reg [10:0] g_x_min, g_y_min, g_x_max, g_y_max, g_x_mid;
always@(posedge clk) begin
	if (green_detect & !white_detect & in_valid & (g_counter > 4'hd| ((4'hf - counter > 4'h8)&g_counter > 4'h5)) ) begin	//Update bounds when the pixel is red
		if (x < g_x_min) g_x_min <= x;
		if (x > g_x_max) g_x_max <= x;
		if (y < g_y_min) g_y_min <= y;
		if (y > g_y_max) g_y_max <= y;
		//g_x_mid <= g_x_min[10:1] + g_x_max[10:1];
	end
	if (sop & in_valid) begin	//Reset bounds on start of packet
		g_x_min <= IMAGE_W-11'h1;
		g_x_max <= 0;
		g_y_min <= IMAGE_H-11'h1;
		g_y_max <= 0;
		g_x_mid <= 0;
	end
end

//Process bounding box at the end of the frame.
reg [10:0] msg_state;
reg [10:0] left, right, top, bottom, mid;
reg [10:0] y_left, y_right, y_top, y_bottom, y_mid;
reg [10:0] g_left, g_right, g_top, g_bottom, g_mid;
//reg [10:0] wl_left, wl_right, wl_top, wl_bottom;
//reg [10:0] wr_left, wr_right, wr_top, wr_bottom;
reg [8:0] frame_count;

reg [31:0] point_mem [127:0];
reg [6:0] point_pointer;
reg [6:0] send_pointer;
reg [1:0] clk_pointer;
reg [3:0] send_counter;
always@(posedge clk) begin
	if (eop & in_valid & packet_video) begin  //Ignore non-video packets
		
		//Latch edges for display overlay on next frame
		left <= x_min;
		right <= x_max;
		top <= y_min;
		bottom <= y_max;
		mid <= left[10:1] + right[10:1];
		//if (right <= 11'h0 & left >= 11'h027f) mid <= 11'd0;
		
		y_left <= y_x_min;
		y_right <= y_x_max;
		y_top <= y_y_min;
		y_bottom <= y_y_max;
		y_mid <= y_left[10:1]+y_right[10:1];
		//if (y_right <= 11'h0 & y_left >= 11'h027f) mid <= 11'd0;
		
		g_left <= g_x_min;
		g_right <= g_x_max;
		g_top <= g_y_min;
		g_bottom <= g_y_max;
		g_mid <= g_left[10:1]+g_right[10:1];		
		//if (g_right <= 11'h0 & g_left >= 11'h027f) mid <= 11'd0;
		
		
		//Start message writer FSM once every MSG_INTERVAL frames, if there is room in the FIFO
		frame_count <= frame_count - 1;
		if (frame_count == 0 && msg_buf_size < (MESSAGE_BUF_MAX - 60)) begin
			point_counter <= 7'b0;
			msg_state <= 11'd1;
			frame_count <= MSG_INTERVAL-1;
			send_counter <= send_counter + 4'b1;
		end
		if (send_counter == 4'hf) begin
			x_tracker <= 11'b0;
			y_tracker <= 11'b0;
		end
	end
	if(point_pointer == 7'd127) begin
		x_tracker <= x;
		y_tracker <= point_mem[110];
	end
	//Cycle through message writer states once started
	if (msg_state != 11'd0 & msg_state <= 11'd11) begin
		msg_state <= msg_state + 11'b1;
		white_wr <= 1'b0;
	end  
	if (msg_state > 11'd11 & msg_state < 11'd46 && point_pointer == 7'd127) begin
	//if (msg_state >= 11'd10 & msg_state < 11'd15) begin
		msg_state <= msg_state + 11'b1;
		send_pointer <= send_pointer + 7'b1;
		white_wr <= 1'b1;
	end
	else begin
		white_wr <= 1'b0;
	end
end



always@(posedge clk) begin
	//if((x_line_active||y_line_active)&&(y>y_tracker || (y==y_tracker && x > x_tracker))) begin
	if((x_line_active||y_line_active)&&(y>y_tracker)) begin
		point_mem[point_pointer] <= {5'b0,x,5'b0,y};
		if(point_pointer < 7'd127) point_pointer <= point_pointer + 7'b1;
	end
	if(send_pointer >= 7'd127) begin
		point_pointer <= 7'b0;
	end
end

//Generate output messages for CPU
reg [31:0] msg_buf_in; 
wire [31:0] msg_buf_out;
reg msg_buf_wr;
wire msg_buf_rd, msg_buf_flush;
wire [7:0] msg_buf_size;
wire msg_buf_empty;
reg [1:0] det; 
reg [20:0] green_dot_counter;
reg count_state;
reg white_wr;

`define RED_BOX_MSG_ID "RBB"
`define YELLOW_BOX_MSG_ID "YBB"
`define GREEN_BOX_MSG_ID "GBB"
`define WHITE_BOX_MSG_ID "WBB"
`define END_MSG_ID "END"

reg[6:0] point_counter;
reg[10:0] x_tracker;
reg[10:0] y_tracker;
reg[10:0] pre_msg_state;
/*
always@(posedge clk) begin
	case(msg_state)
		11'd0: begin
			msg_buf_in <= 32'b0;
			msg_buf_wr <= 1'b0;
		end
		11'd1: begin
			msg_buf_in <= `RED_BOX_MSG_ID;
			msg_buf_wr <= 1'b1;
		end
		11'd2: begin
			msg_buf_in <= {5'b0, x_min, 5'b0, y_min};
			//msg_buf_in <= {32'h0000000f};
			msg_buf_wr <= 1'b1;
		end
		11'd3: begin
			msg_buf_in <= {5'b0, x_max, 5'b0, y_max};
			//msg_buf_in <= {32'h000000f0};
			msg_buf_wr <= 1'b1;
		end
		11'd4: begin
			msg_buf_in <= `YELLOW_BOX_MSG_ID;
			msg_buf_wr <= 1'b1;
		end
		11'd5: begin
			msg_buf_in <= {5'b0, y_x_min, 5'b0, y_y_min};
			//msg_buf_in <= {32'h00000f00};
			msg_buf_wr <= 1'b1;
		end
		11'd6: begin
			msg_buf_in <= {5'b0, y_x_max, 5'b0, y_y_max};
			//msg_buf_in <= {32'h0000f000};
			msg_buf_wr <= 1'b1;
		end
		11'd7: begin
			msg_buf_in <= `GREEN_BOX_MSG_ID;;
			msg_buf_wr <= 1'b1;
		end
		11'd8: begin
			msg_buf_in <= {5'b0, g_x_min, 5'b0, g_y_min};
			//msg_buf_in <= {32'h000f0000};
			msg_buf_wr <= 1'b1;
		end
		11'd9: begin
			msg_buf_in <= {5'b0, g_x_max, 5'b0, g_y_max};
			//msg_buf_in <= {32'h00f00000};
			msg_buf_wr <= 1'b1;
		end
		11'd10: begin
			msg_buf_in <= `WHITE_BOX_MSG_ID;
			msg_buf_wr <= 1'b1;
		end
		default: begin
			if(msg_state == 11'd46) begin
				//msg_buf_in <= `END_MSG_ID;
				;
			end
			else begin
				msg_buf_in <= point_mem[send_pointer];
				//msg_buf_in <= {5'b0,point_mem[send_pointer][10:0],5'b0, y_tracker};
				//msg_buf_in <= {24'b0, msg_buf_size};
			end	
			msg_buf_wr <= white_wr;
		end 
	endcase
end*/

always@(posedge clk) begin
	case(msg_state)
		11'd0: begin
			msg_buf_in <= 32'b0;
			msg_buf_wr <= 1'b0;
		end
		11'd1: begin
			msg_buf_in <= `END_MSG_ID;
			msg_buf_wr <= 1'b1;
		end
		11'd2: begin
			msg_buf_in <= `RED_BOX_MSG_ID;
			msg_buf_wr <= 1'b1;
		end
		11'd3: begin
			msg_buf_in <= {5'b0, x_min, 5'b0, y_min};
			//msg_buf_in <= {32'h0000000f};
			msg_buf_wr <= 1'b1;
		end
		11'd4: begin
			msg_buf_in <= {5'b0, x_max, 5'b0, y_max};
			//msg_buf_in <= {32'h000000f0};
			msg_buf_wr <= 1'b1;
		end
		11'd5: begin
			msg_buf_in <= `YELLOW_BOX_MSG_ID;
			msg_buf_wr <= 1'b1;
		end
		11'd6: begin
			msg_buf_in <= {5'b0, y_x_min, 5'b0, y_y_min};
			//msg_buf_in <= {32'h00000f00};
			msg_buf_wr <= 1'b1;
		end
		11'd7: begin
			msg_buf_in <= {5'b0, y_x_max, 5'b0, y_y_max};
			//msg_buf_in <= {32'h0000f000};
			msg_buf_wr <= 1'b1;
		end
		11'd8: begin
			msg_buf_in <= `GREEN_BOX_MSG_ID;;
			msg_buf_wr <= 1'b1;
		end
		11'd9: begin
			msg_buf_in <= {5'b0, g_x_min, 5'b0, g_y_min};
			//msg_buf_in <= {32'h000f0000};
			msg_buf_wr <= 1'b1;
		end
		11'd10: begin
			msg_buf_in <= {5'b0, g_x_max, 5'b0, g_y_max};
			//msg_buf_in <= {32'h00f00000};
			msg_buf_wr <= 1'b1;
		end
		11'd11: begin
			msg_buf_in <= `WHITE_BOX_MSG_ID;
			msg_buf_wr <= 1'b1;
		end
		default: begin
			if(msg_state == 11'd46) begin
				//msg_buf_in <= `END_MSG_ID;
				;
			end
			else begin
				msg_buf_in <= point_mem[send_pointer];
				//msg_buf_in <= {5'b0,point_mem[send_pointer][10:0],5'b0, y_tracker};
				//msg_buf_in <= {24'b0, msg_buf_size};
			end	
			msg_buf_wr <= white_wr;
		end 
	endcase
end




/*
always@(*) begin	//Write words to FIFO as state machine advances
	case(msg_state)
		2'b00: begin
			msg_buf_in = 32'hffffffff;
			msg_buf_wr = 1'b0
			;
		end
		2'b01: begin
			msg_buf_in = `RED_BOX_MSG_ID;	//Message ID
			msg_buf_wr = 1'b1;
		end
		2'b10: begin
			msg_buf_in = {32'hf000000f};	//Top left coordinate
			msg_buf_wr = 1'b1;
		end
		2'b11: begin
			msg_buf_in = {32'h0000ffff}; //Bottom right coordinate
			msg_buf_wr = 1'b1;
		end
	endcase
end*/



//Output message FIFO
MSG_FIFO	MSG_FIFO_inst (
	.clock (clk),
	.data (msg_buf_in),
	.rdreq (msg_buf_rd),
	.sclr (~reset_n | msg_buf_flush),
	.wrreq (msg_buf_wr),
	.q (msg_buf_out),
	.usedw (msg_buf_size),
	.empty (msg_buf_empty)
	);


//Streaming registers to buffer video signal
STREAM_REG #(.DATA_WIDTH(26)) in_reg (
	.clk(clk),
	.rst_n(reset_n),
	.ready_out(sink_ready),
	.valid_out(in_valid),
	.data_out({red,green,blue,sop,eop}),
	.ready_in(out_ready),
	.valid_in(sink_valid),
	.data_in({sink_data,sink_sop,sink_eop})
);

STREAM_REG #(.DATA_WIDTH(26)) out_reg (
	.clk(clk),
	.rst_n(reset_n),
	.ready_out(out_ready),
	.valid_out(source_valid),
	.data_out({source_data,source_sop,source_eop}),
	.ready_in(source_ready),
	.valid_in(in_valid),
	.data_in({red_out, green_out, blue_out, sop, eop})
);


/////////////////////////////////
/// Memory-mapped port		 /////
/////////////////////////////////

// Addresses
`define REG_STATUS    			0
`define READ_MSG    				1
`define READ_ID    				2
`define REG_BBCOL					3

//Status register bits
// 31:16 - unimplemented
// 15:8 - number of words in message buffer (read only)
// 7:5 - unused
// 4 - flush message buffer (write only - read as 0)
// 3:0 - unused


// Process write

reg  [7:0]   reg_status;
reg	[23:0]	bb_col;

always @ (posedge clk)
begin
	if (~reset_n)
	begin
		reg_status <= 8'b0;
		//bb_col <= BB_COL_DEFAULT;
		bb_col <= 24'hffffff;
	end
	else begin
		if(s_chipselect & s_write) begin
		   if      (s_address == `REG_STATUS)	reg_status <= s_writedata[7:0];
		   if      (s_address == `REG_BBCOL)	bb_col <= s_writedata[23:0];
		end
	end
end


//Flush the message buffer if 1 is written to status register bit 4
assign msg_buf_flush =  (s_chipselect & s_write & (s_address == `REG_STATUS) & s_writedata[4]);


// Process reads
reg read_d; //Store the read signal for correct updating of the message buffer

// Copy the requested word to the output port when there is a read.
always @ (posedge clk)
begin
   if (~reset_n) begin
	   s_readdata <= {32'b0};
		read_d <= 1'b0;
	end
	
	else if (s_chipselect & s_read) begin
		if   (s_address == `REG_STATUS) s_readdata <= {16'b0,msg_buf_size,reg_status};
		if   (s_address == `READ_MSG) s_readdata <= {msg_buf_out};
		if   (s_address == `READ_ID) s_readdata <= 32'h1234EEE2;
		if   (s_address == `REG_BBCOL) s_readdata <= {8'h0, bb_col};
	end
	
	read_d <= s_read;
end

//Fetch next word from message buffer after read from READ_MSG
assign msg_buf_rd = s_chipselect & s_read & ~read_d & ~msg_buf_empty & (s_address == `READ_MSG);
						


endmodule

