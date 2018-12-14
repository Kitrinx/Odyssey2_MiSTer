// FPGA Videopac
//
// $Id: jop_vp.vhd,v 1.11 2007/04/10 21:29:02 arnim Exp $
// $Name: videopac_rel_1_0 $
//
// Toplevel of the Cyclone port for 
//   https://github.com/wsoltys/mist-cores
//
//-----------------------------------------------------------------------------
//
// Copyright (c) 2007, Arnim Laeuger (arnim.laeuger@gmx.net)
//
// All rights reserved
//
// Redistribution and use in source and synthezised forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// Redistributions in synthesized form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// Neither the name of the author nor the names of other contributors may
// be used to endorse or promote products derived from this software without
// specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Please report bugs to the author, but before you do so, please
// make sure that this is not a derivative work and that
// you have the latest version of this file.
//
// Based off MiST port by wsoltys in 2014.
//
// Adapted for MiSTer by Kitrinx in 2018

module emu
(
	//Master input clock
	input         CLK_50M,

	//Async reset from top-level module.
	//Can be used as initial reset.
	input         RESET,

	//Must be passed to hps_io module
	inout  [44:0] HPS_BUS,

	//Base video clock. Usually equals to CLK_SYS.
	output        CLK_VIDEO,

	//Multiple resolutions are supported using different CE_PIXEL rates.
	//Must be based on CLK_VIDEO
	output        CE_PIXEL,

	//Video aspect ratio for HDMI. Most retro systems have ratio 4:3.
	output  [7:0] VIDEO_ARX,
	output  [7:0] VIDEO_ARY,

	output  [7:0] VGA_R,
	output  [7:0] VGA_G,
	output  [7:0] VGA_B,
	output        VGA_HS,
	output        VGA_VS,
	output        VGA_DE,    // = ~(VBlank | HBlank)
	output        VGA_F1,
	output [1:0]  VGA_SL,

	output        LED_USER,  // 1 - ON, 0 - OFF.

	// b[1]: 0 - LED status is system status OR'd with b[0]
	//       1 - LED status is controled solely by b[0]
	// hint: supply 2'b00 to let the system control the LED.
	output  [1:0] LED_POWER,
	output  [1:0] LED_DISK,

	output [15:0] AUDIO_L,
	output [15:0] AUDIO_R,
	output        AUDIO_S, // 1 - signed audio samples, 0 - unsigned
	output  [1:0] AUDIO_MIX, // 0 - no mix, 1 - 25%, 2 - 50%, 3 - 100% (mono)
	input         TAPE_IN,

	// SD-SPI
	output        SD_SCK,
	output        SD_MOSI,
	input         SD_MISO,
	output        SD_CS,
	input         SD_CD,

	//High latency DDR3 RAM interface
	//Use for non-critical time purposes
	output        DDRAM_CLK,
	input         DDRAM_BUSY,
	output  [7:0] DDRAM_BURSTCNT,
	output [28:0] DDRAM_ADDR,
	input  [63:0] DDRAM_DOUT,
	input         DDRAM_DOUT_READY,
	output        DDRAM_RD,
	output [63:0] DDRAM_DIN,
	output  [7:0] DDRAM_BE,
	output        DDRAM_WE,
	output [1:0]  BUFFERMODE,

	//SDRAM interface with lower latency
	output        SDRAM_CLK,
	output        SDRAM_CKE,
	output [12:0] SDRAM_A,
	output  [1:0] SDRAM_BA,
	inout  [15:0] SDRAM_DQ,
	output        SDRAM_DQML,
	output        SDRAM_DQMH,
	output        SDRAM_nCS,
	output        SDRAM_nCAS,
	output        SDRAM_nRAS,
	output        SDRAM_nWE,

	input         UART_CTS,
	output        UART_RTS,
	input         UART_RXD,
	output        UART_TXD,
	output        UART_DTR,
	input         UART_DSR
);

assign {UART_RTS, UART_TXD, UART_DTR} = 0;

assign AUDIO_S   = 1;
assign AUDIO_MIX = 0;

assign LED_USER  = ioctl_download;
assign LED_DISK  = 0;
assign LED_POWER = 0;

assign VIDEO_ARX = status[8] ? 8'd16 : 8'd64;
assign VIDEO_ARY = status[8] ? 8'd9  : 8'd48;

assign {SDRAM_DQ, SDRAM_A, SDRAM_BA, SDRAM_CKE, SDRAM_DQML, SDRAM_DQMH, SDRAM_nWE, SDRAM_nCAS, SDRAM_nRAS, SDRAM_nCS} = 'Z;
assign {DDRAM_CLK, DDRAM_BURSTCNT, DDRAM_ADDR, DDRAM_DIN, DDRAM_BE, DDRAM_RD, DDRAM_WE} = 0;
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;

assign SDRAM_CLK = clk_sys;


////////////////////////////  HPS I/O  //////////////////////////////////


`include "build_id.v"
parameter CONF_STR = {
	"ODYSSEY2;;",
	"-;",
	"F,BIN;",
	"-;",
	"OE,Video Region,NTSC,PAL;",
	"O8,Aspect ratio,4:3,16:9;",
	"O9B,Scandoubler Fx,None,HQ2x,CRT 25%,CRT 50%,CRT 75%;",
	"OGH,Buffering,Triple,Single,Low Latency;",
	"-;",
	"O7,Swap Joysticks,No,Yes;",
	"-;",
	"R0,Reset;",
	"J1,Action, T0;",
	"V,v",`BUILD_DATE
};

wire  [1:0] buttons;
wire [31:0] status;
wire        forced_scandoubler;

wire        ioctl_download;
wire [24:0] ioctl_addr;
wire [15:0] ioctl_dout;
wire        ioctl_wait;
wire        ioctl_wr;

wire [11:0] joystick_0,joystick_1;
wire [24:0] ps2_mouse;

hps_io #(.STRLEN($size(CONF_STR)>>3)) hps_io
(
	.clk_sys(clk_sys),
	.HPS_BUS(HPS_BUS),

	.conf_str(CONF_STR),

	.ioctl_download(ioctl_download),
	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout),
	.ioctl_wait(ioctl_wait),

	.forced_scandoubler(forced_scandoubler),
	
	.buttons(buttons),
	.status(status),

	.joystick_0(joystick_0),
	.joystick_1(joystick_1)
);

assign BUFFERMODE = (status[17:16] == 2'b00) ? 2'b01 : (status[17:16] == 2'b01) ? 2'b00 : 2'b10;

wire       PAL = status[15];
wire       joy_swap = status[7];

wire [11:0] joya = joy_swap ? joystick_1 : joystick_0;
wire [11:0] joyb = joy_swap ? joystick_0 : joystick_1;


///////////////////////  CLOCK/RESET  ///////////////////////////////////


wire clock_locked;

wire clk_sys_ntsc;
wire clk_sys_pal;
wire clk_sys = PAL ? clk_sys_pal : clk_sys_ntsc;

wire clk_vga_ntsc;
wire clk_vga_pal;
wire clk_vga = PAL ? clk_vga_pal : clk_vga_ntsc;

wire clk_cpu = (clk_cpu_ctr == div_cpu);
wire clk_vdc = (clk_vdc_ctr == div_vdc);

reg [4:0] clk_cpu_ctr;
reg [4:0] clk_vdc_ctr;

wire div_vdc = PAL ? 4'd5 : 4'd3;
wire div_cpu = PAL ? 4'd6 : 4'd4;

pll pll
(
	.refclk(CLK_50M),
	.rst(0),
	.outclk_0(clk_sys_ntsc),
	.outclk_1(clk_sys_pal),
	.outclk_2(clk_vga_ntsc),
	.outclk_3(clk_vga_pal),
	.locked(clock_locked)
);

// hold machine in reset until first download starts
reg init_reset_n = 0;
reg old_pal = 0;
always @(posedge clk_sys) begin
	old_pal <= PAL;
	if(RESET) init_reset_n <= 0;
	else if(ioctl_download) init_reset_n <= 1;
end

wire reset = ~init_reset_n | buttons[1] | status[0] | ioctl_download | (old_pal != PAL);


// Clocks:
// Standard    NTSC         PAL
// Main clock  21.477 MHz   35.469 MHz
// VDC divider 3            5
// VDC clock   7.159 MHz    7.094 MHz
// CPU divider 4            6
// CPU clock   5.369 MHz    5.911 MHz


always @(posedge clk_sys or posedge reset) begin
	if (reset) begin
		clk_cpu_ctr <= 4'd1;
		clk_vdc_ctr <= 4'd1;
	end else begin
		if (clk_cpu_ctr >= div_cpu)
			clk_cpu_ctr <= 4'd1;
		else
			clk_cpu_ctr <= clk_cpu_ctr + 4'd1;
			
		if (clk_vdc_ctr >= div_vdc)
			clk_vdc_ctr <= 4'd1;
		else
			clk_vdc_ctr <= clk_vdc_ctr + 4'd1;
	end
end


////////////////////////////  SYSTEM  ///////////////////////////////////


vp_console #(0) vp
(
	// System
	.clk_i          (clk_sys),   // 21.5mhz
	.clk_cpu_en_i   (clk_cpu), // CPU 21.5mhz / 4
	.clk_vdc_en_i   (clk_vdc), // VDC 21.5mhz / 3
	
	.res_n_i        (~reset), // low to reset

	// Cart Data
	.cart_cs_o      (),
	.cart_cs_n_o    (),
	.cart_wr_n_o    (),
	.cart_a_o       (cart_addr),
	.cart_d_i       (cart_do),
	.cart_bs0_o     (cart_bank_0),
	.cart_bs1_o     (cart_bank_1),
	.cart_psen_n_o  (cart_rd),
	.cart_t0_i      (joy_gnd), // Some kind of switch/key?
	.cart_t0_o      (),
	.cart_t0_dir_o  (),

	// Input
	.joy_up_n_i     (joy_up), //-- idx = 0 : left joystick -- idx = 1 : right joystick
	.joy_down_n_i   (joy_down),
	.joy_left_n_i   (joy_left),
	.joy_right_n_i  (joy_right),
	.joy_action_n_i (joy_action),

	.keyb_dec_o     (kb_dec),
	.keyb_enc_i     (kb_enc),

	// Video
	.r_o            (R),
	.g_o            (G),
	.b_o            (B),
	.l_o            (luma),
	.hsync_n_o      (HSync),
	.vsync_n_o      (VSync),
	.hbl_o          (HBlank),
	.vbl_o          (VBlank),
	
	// Sound
	.snd_o          (snd_m),
	.snd_vec_o      (snd)
);


////////////////////////////  SOUND  ////////////////////////////////////


wire snd_m;
wire [3:0] snd;
reg [3:0] snd_out;

always @(posedge clk_sys) begin
		snd_out <= snd;
end

assign AUDIO_L = {snd_out, 12'b0};// + {2'b0, snd_m, 13'b0};
assign AUDIO_R = {snd_out, 12'b0};// + {2'b0, snd_m, 13'b0};


////////////////////////////  VIDEO  ////////////////////////////////////


wire R;
wire G;
wire B;
wire luma;

wire HSync;
wire VSync;
wire HBlank;
wire VBlank;

wire ce_pix = 1;

wire [23:0] colors = color_lut[{R, G, B, luma}];

assign CLK_VIDEO = clk_vdc;
assign VGA_SL = sl[1:0];
assign VGA_F1 = 0;


wire [2:0] scale = status[11:9];
wire [2:0] sl = scale ? scale - 1'd1 : 3'd0;
wire       scandoubler =  (scale || forced_scandoubler);

video_mixer #(.LINE_LENGTH(267)) video_mixer
(
	.*,
	.HBlank(HBlank),
	.VBlank(VBlank),
	.HSync(~HSync),
	.VSync(~VSync),
	.clk_sys(CLK_VIDEO),
	.ce_pix_out(CE_PIXEL),

	.scanlines(0),
	.hq2x(scale==1),
	.mono(0),

	.R(colors[23:16]),
	.G(colors[15:8]),
	.B(colors[7:0])
);

// always @(posedge clk) begin
// 	reg [8:0] old_count_v;

// 	if(pix_ce_n) begin
// 		if((old_count_v == 511) && (count_v == 0)) begin
// 			h <= 0;
// 			v <= 0;
// 			free_sync <= 0;
// 		end else begin
// 			if(h == 340) begin
// 				h <= 0;
// 				if(v == 261) begin
// 					v <= 0;
// 					if(~&free_sync) free_sync <= free_sync + 1'd1;
// 				end else begin
// 					v <= v + 1'd1;
// 				end
// 			end else begin
// 				h <= h + 1'd1;
// 			end
// 		end

// 		old_count_v <= count_v;
// 	end

// 	if(pix_ce) begin
// 		if(hide_overscan) begin
// 			HBlank <= (hc > (256-8)) || (hc<10);
// 			VBlank <= (vc > (240-10)) || (vc<6);
// 		end else begin
// 			HBlank <= (hc >= 256);
// 			VBlank <= (vc >= 240);
// 		end
// 		HSync  <= ((hc >= 277) && (hc <  318));
// 		VSync  <= ((vc >= 245) && (vc <  254));
// 	end
// end

////////////////////////////  INPUT  ////////////////////////////////////


// XXX: Add keyboard

// [5]  = B
// [4]  = A
// [3]  = UP
// [2]  = DOWN
// [1]  = LEFT
// [0]  = RIGHT

wire [7:0] kb_dec;
wire [15:0] kb_enc;


// Joystick wires are low when pressed
// Passed as a vector bit 1 = left bit 0 = right
wire [1:0] joy_up     = {~joya[3], ~joyb[3]};
wire [1:0] joy_down   = {~joya[2], ~joyb[2]};
wire [1:0] joy_left   = {~joya[1], ~joyb[1]};
wire [1:0] joy_right  = {~joya[0], ~joyb[0]};
wire [1:0] joy_action = {~joya[4], ~joyb[4]};
wire       joy_gnd    = {joya[5]};


////////////////////////////  MEMORY  ///////////////////////////////////


wire [11:0] cart_addr;
wire [7:0] cart_di;
wire [7:0] cart_do;
wire cart_bank_0;
wire cart_bank_1;
wire cart_rd;
wire cart_gnd;
reg [15:0] cart_size;

dpram #(14) rom
(
	.clock(clk_sys),
	.address_a(ioctl_download ? ioctl_addr[13:0] : rom_addr),
	.data_a(ioctl_dout),
	.wren_a(ioctl_wr),
	.q_a(cart_do)
);

reg old_download = 0;

always @(posedge clk_sys) begin
	old_download <= ioctl_download;
	
	if (~old_download & ioctl_download)
		cart_size <= 16'd0;
	else if (ioctl_download & ioctl_wr)
		cart_size <= cart_size + 16'd1;
end

wire [12:0] rom_addr = 
	{(cart_size >= 16'h2000) ? cart_bank_1 : 1'b0, (cart_size >= 16'h1000) ? 
	cart_bank_0 : 1'b0, cart_addr[11], cart_addr[9:0]};


// LUT using calibrated palette
wire [23:0] color_lut[16] = '{
	24'h000000, 24'h676767,
	24'h1a37be, 24'h5c80f6,
	24'h006d07, 24'h56c469,
	24'h2aaabe, 24'h77e6eb,
	24'h790000, 24'hc75151,
	24'h94309f, 24'hdc84e8,
	24'h77670b, 24'hc6b86a,
	24'hcecece, 24'hffffff
};

endmodule

