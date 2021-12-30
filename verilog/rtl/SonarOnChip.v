/*
Sonar on Chip top level module based on user project example
Files:
defines.v - macroodefinitions (come vith Caravel)
*/
`include "defines.v"

`define BUS_WIDTH 16 

module SonarOnChip
   (
  `ifdef USE_POWER_PINS
    inout wire vdda1,	// User area 1 3.3V supply
    inout wire vdda2,	// User area 2 3.3V supply
    inout wire vssa1,	// User area 1 analog ground
    inout wire vssa2,	// User area 2 analog ground
    inout wire vccd1,	// User area 1 1.8V supply
    inout wire vccd2,	// User area 2 1.8v supply
    inout wire vssd1,	// User area 1 digital ground
    inout wire vssd2,	// User area 2 digital ground
  `endif

    // Wishbone Slave ports (WB MI A)
    input wire wb_clk_i,
    input wire wb_rst_i,
    input wire wb_valid_i,
    input wire  [3:0] wbs_adr_i,
    input wire  [`BUS_WIDTH-1:0] wbs_dat_i,
    input  wire  wbs_strb_i,
    output                    wbs_ack_o,
    output   [`BUS_WIDTH-1:0] wbs_dat_o,

    /* Design specific ports*/
    /* 4.8 MHz clock input */
    input wire  ce_pdm,
    /* PCM pace signal */
    input wire  ce_pcm,
    /* External microphone PDM data */
    input wire  pdm_data_i,
    /* Master clear */
    input wire  mclear,
    /* compare otupt signal*/    
    output wire  cmp
	);

  /*----------------------------- Register map begins ---------------------*/

  localparam  CONTROL_ADDR 		=  'd0;
  localparam  A0_ADDR 			=  'd1;
  localparam  A1_ADDR 			=  'd2; 
  localparam  A2_ADDR 			=  'd3; 
  localparam  B1_ADDR 			=  'd4; 
  localparam  B2_ADDR 			=  'd5; 
  localparam  AMP_ADDR 			=  'd6; 
  localparam  THRESHOLD_ADDR 	=  'd7;
  localparam  TIMER_ADDR 	    =  'd8;
  localparam  PCM_ADDR 	        =  'd9;
  localparam  PCM_LOAD_ADDR     =  'd10;
  localparam  FB0_ADDR 	 	    =  'd11;
  localparam  FB1_ADDR	    	=  'd12;
 
  /*----------------------------- Register Map ends ---------------------*/

  /* clock and reset signals*/
  wire clk;
  wire rst;
  wire we_pcm;  
   /* Compare module wires*/
  wire [`BUS_WIDTH-1:0] maf_o;
  wire compare_out;
  /* PCM inputs from GPIO, will come from PDM */	
  wire [`BUS_WIDTH-1:0] pcm_reg_i;
  /* ABS  output signal*/
  wire [`BUS_WIDTH-1:0] pcm_abs;
  /* Multiplier  output */
  wire [`BUS_WIDTH-1:0] mul_o;
 /** Wishbone Slave Interface **/
  reg wbs_done;
  wire wb_valid;
  wire [3:0] wstrb;
  reg [7:0] control;
  reg [7:0] amp;
  reg signed [`BUS_WIDTH-1:0] pcm;
  reg signed [`BUS_WIDTH-1:0] pcm_load;
  reg [`BUS_WIDTH-1:0] timer;
  //---- IIR COEFF ---- //
  reg signed [`BUS_WIDTH-1:0] a0;
  reg signed [`BUS_WIDTH-1:0] a1;
  reg signed [`BUS_WIDTH-1:0] a2;
  reg signed [`BUS_WIDTH-1:0] b1;
  reg signed [`BUS_WIDTH-1:0] b2;
	//---- FIR COEFF ---- //
  reg signed [`BUS_WIDTH-1:0] fb0;
  reg signed [`BUS_WIDTH-1:0] fb1;
  reg [`BUS_WIDTH-1:0] threshold;
  wire timer_we;
  wire srlatchQ;
  wire srlatchQbar;
  reg [`BUS_WIDTH-1:0] rdata;
  wire [11:0] cic_out;
  wire [`BUS_WIDTH-1:0] fir_out;
  wire [`BUS_WIDTH-1:0] fir_in;
  wire  [`BUS_WIDTH-1:0] mul_i;
  wire  [`BUS_WIDTH-1:0] iir_data;

  assign wbs_ack_o = wbs_done;
  assign wbs_dat_o =  rdata;
  assign clk = wb_clk_i;
  assign rst = wb_rst_i;



/* register mapping and slave interface  */

	always@(posedge clk) begin
		if(rst) begin
        wbs_done <= 0;
		a0 <= 16'h0000;
		a1 <= 16'h0000;
		a2 <= 16'h0000;
		b1 <= 16'h0000;
		b2 <= 16'h0000;
        fb0 <= 0;
        fb1 <= 0;
		fb0 <= 16'h0FFF;
		fb1 <= 16'h7FFF; 
        amp <= 8'h00;
        threshold <= 0;
        control   <= 0; 
        threshold <= 16'h0400;
        control   <= 16'h04; 
		pcm_load <= 16'h0000;
        rdata <= 16'h0000;

		end
		else begin
            wbs_done <= 0;
			if (wb_valid_i) begin     
				case(wbs_adr_i)   
					CONTROL_ADDR: 
 						begin
                  rdata <= control;
                   		if(wbs_strb_i)
       						      control <= wbs_dat_i;
                      end
					A0_ADDR:
 						begin
                  rdata <= a0;
                   	if(wbs_strb_i)
       						    a0 <= wbs_dat_i;
            end
					A1_ADDR:
 						begin
                  rdata <= a1;
                   	if(wbs_strb_i)
       						    a1 <= wbs_dat_i;
            end
					A2_ADDR:
 						begin
                  rdata <= a2;
                   		if(wbs_strb_i)
       						a2 <= wbs_dat_i;
                        end
					B1_ADDR:
 						begin
                  rdata <= b1;
                   	if(wbs_strb_i)
       						    b1 <= wbs_dat_i;
            end
					B2_ADDR:
 						begin
                  rdata <= b2;
                   		if(wbs_strb_i)
       						      b2 <= wbs_dat_i;
            end
					FB0_ADDR:
 						begin
                   	         rdata <= fb0;
                   		if(wbs_strb_i)
       						fb0 <= wbs_dat_i;
                        end
					FB1_ADDR:
 						begin
                   	         rdata <= fb1;
                   		if(wbs_strb_i)
       						fb1 <= wbs_dat_i;
                        end

					PCM_ADDR:
 						begin
                   	         rdata <= pcm;
                        end
					TIMER_ADDR:
 						begin
                   	         rdata <= timer;
                        end
					PCM_LOAD_ADDR:
 						begin
                   	         rdata <= pcm_load;
                   		if(wbs_strb_i)
       						 pcm_load <= wbs_dat_i;
                        end
					AMP_ADDR:
 						begin
                   	         rdata <= amp;
                   		if(wbs_strb_i)
       						amp <= wbs_dat_i;
                        end
					THRESHOLD_ADDR:
 						begin
                   	         rdata <= threshold;
                   		if(wbs_strb_i)
       						threshold <= wbs_dat_i;
                        end
                    default:   rdata <= 0; 
				endcase
                wbs_done  <= 1;
			end
		end
   end 
  


/* timer register block */
/*-----------------------------------------------------------------------------*/
  always @(posedge clk)
    begin
      if (rst)
            timer <= 0;
        else if (mclear)
            timer <= 0;
        else if (timer_we)
            timer <= timer + 1'b1;
    end
   
   assign  timer_we = we_pcm & (srlatchQbar); 
   /*-----------------------------------------------------------------------------*/

   /* select wheater the PCM clock is derived from main clock or the PCM   
   datapath can be clocked manually*/
  assign we_pcm = ce_pcm; 


  /*-------------------------Structural modelling ----------------------------*/
  
  /*------------------------  PDM starts   -----------------------------------*/
  cic  cicmodule(clk, rst, ce_pdm, pdm_data_i, cic_out);
  /*------------------------   PDM ends    -----------------------------------*/
  
  /*------------------------   FIR start    -----------------------------------*/

 /* extend the 12 bit signal from PDM demodulator to 16 bit*/
  assign fir_in = {{4{cic_out[11]}}, cic_out };
// FIR fir_filter(clk, rst, we_pcm, fir_in, fb0, fb1, fir_out);
  
  /*------------------------   FIR ends    -----------------------------------*/
  
  /*------------------------  PCM starts   -----------------------------------*/
  
  assign pcm_reg_i = control[3] ? pcm_load :fir_out;

/* pcm register block */
  always@(posedge clk) begin
  	if(rst) 
		pcm <= 0;
    else if (we_pcm)
        pcm <= pcm_reg_i;
  end
  /*------------------------   PCM ends    -----------------------------------*/
  
  /*------------------------  MUL starts   -----------------------------------*/
  assign mul_i = control[2] ? pcm : iir_data; 
  multiplier mul(mul_i, amp, mul_o);
  /*------------------------   MUL ends    -----------------------------------*/
    
  /*------------------------  ABS starts   -----------------------------------*/
  Abs  abs(mul_o, pcm_abs);
  /*------------------------   ABS ends    -----------------------------------*/
  
  /*------------------------  IIR starts   -----------------------------------*/
	/*IIR_Filter u_Filter(
    .clk(clk),
    .rst(rst),
    .en(we_pcm),
    .X(pcm),
    .a0(a0),
    .a1(a1),
    .a2(a2),
    .b1(b1),
    .b2(b2),
    .Y(iir_data)
		);
*/

  /*------------------------   IIR ends    -----------------------------------*/
  
  /*------------------------  MAMOV starts   ---------------------------------*/
 //MAF_FILTER maf(clk, rst, we_pcm, pcm_abs, maf_o);
  /*------------------------   MAMOV ends    ---------------------------------*/
  
  /*------------------------  COMP starts   ----------------------------------*/
  
  comparator comp(maf_o, threshold, compare_out);
  SR_latch sr(clk, rst, mclear, compare_out, srlatchQ, srlatchQbar);
  assign cmp = srlatchQ;
  
  /*------------------------   COMP ends    ----------------------------------*/
  


 Filters  filt( .clk(clk),
                .rst(rst),
                .en(we_pcm),
                .X_fir(fir_in),
                .b0_fir(fb0),
                .b1_fir(fb1),
                .Y_fir(fir_out),  
                .X_iir(pcm), 
                .a0_iir(a0),
                .a1_iir(a1), 
                .a2_iir(a2), 
                .b1_iir(b1), 
                .b2_iir(b2), 
                .Y_iir(iir_data),
                .X_maf(pcm_abs),
                .Y_maf(maf_o)
);


endmodule

// Module Name:    Abs 
// Luis Osses Gutierrez

module Abs
  #(  parameter n=16)( 
  input signed  [n-1:0] data_in,
  output        [n-1:0] data_out);
	
  assign data_out = ~data_in[n-1] ? data_in : ~data_in; 

endmodule

// Module Name:    Comparator
//maximiliano cerda cid

module comparator #(parameter n=16) (data_i,threshold,compare_o);
  input wire    [n-1:0] data_i, threshold; //maf filter output and treshold value  
  output wire           compare_o; //output of the compare block
  //reg compare_o;

  assign compare_o = (data_i > threshold) ? 1'b1:1'b0;

endmodule

//////////////////////////////////////////////////////////////////////////////////
// Module Name:    MULTIPLICADOR POR CONSTANTE.
// Luis Osses Gutierrez
//////////////////////////////////////////////////////////////////////////////////
module multiplier   #(parameter n =16)  (data_i, amplify, multiplier_o);
  
  input   wire [n-1:0]   data_i;
  input   wire [7:0]     amplify; 
  output  wire [n-1:0]   multiplier_o;
    
  assign multiplier_o = data_i <<< amplify; 
  
endmodule

/*
SR type letch
*/
module SR_latch(
	input clk, 
	input rst, 
	input r, 
	input s, 
	output reg q, 
	output reg qbar); 


always@(posedge clk) begin

    if(rst)	begin
		q <= 0;
		qbar <= 1;
	end
      
	else if(s)	begin
		q <= 1;
		qbar <= 0;
	end

	else if(r) begin
		q <= 0;
		qbar <= 1;
	end

	else if(s == 0 & r == 0) begin
			q <= q;
			qbar <= qbar;
	end

	end

endmodule 

// ---------------------------------------------------- //
//        	     DEMODULATOR FILTER RSS
//         Mauricio Montanares, Luis Osses, Max Cerda 2021
// ---------------------------------------------------- //     
//This file is completly based on the paper:
// FPGA-BASED REAL-TIME ACOUSTIC CAMERA USING PDM MEMS MICROPHONES WITH A CUSTOM DEMODULATION FILTER

`define OverSample 32

// =========== Top module START =========== //
module cic #(parameter N =- 2)(
  input clk, rst, we,
  input                       data_in,
  output wire signed [11:0] data_out2
);
  integer i;
  wire signed [6:0] sum1; 
  wire signed [1:0] data_1_in;
  reg         [1:0] ff1[`OverSample];
  wire        [1:0] ff1_last;
  reg         [6:0] ff1out;
  wire        [6:0] dinext1;
  wire        [6:0] ffext1;
  

  wire signed [11:0] sum2;   
  reg         [6:0] ff2[`OverSample];
  wire        [6:0] ff2_last;
  reg         [11:0] ff2out;
  wire        [11:0] dinext2;
  wire        [11:0] ffext2;


   /* input 2'complement extension */
   assign data_1_in = (data_in == 1'b0) ? 2'b11 : 2'b01;
   assign dinext1 =  {{6{data_1_in[1]}},data_1_in};
   assign ff1_last = ff1[`OverSample-1];
   assign ffext1 =  {{5{ff1_last[1]}}, ff1_last };
   assign sum1 = dinext1-ffext1;


   assign dinext2 =  {{5{ff1out[6]}},ff1out};
   assign ff2_last =ff2[`OverSample-1];
   assign ffext2 =  {{5{ff2_last[6]}}, ff2_last };
   assign sum2 = dinext2 - ffext2;
   assign data_out2 = ff2out;

   always @(posedge clk) begin
      if (rst) begin
         ff1out <=0;
         ff2out <=0;
		 for(i = 0; i<`OverSample; i=i+1 ) begin
		     ff1[i] <= 0;
		     ff2[i] <= 0;
		 end
      end  
      else if (we) begin
    	ff1[0] <= data_1_in;   
		ff2[0] <= ff1out; 	
		for(i = 1; i<`OverSample; i=i+1 ) begin
			ff1[i] <= ff1[i-1];       
            ff2[i] <= ff2[i-1];    
        end
        /*  integrator */
        ff1out <= ff1out + sum1;
        ff2out <= ff2out + sum2;   
      end
    
   end
endmodule


// ---------------------------------------------------- //
//        	       FILTERS 
// ---------------------------------------------------- //  

// phase state: FIR 0-4 | IIR 5-10 | MAF 11-15

`define USE_POWER_PINS

module Filters  #(parameter N  = 16)
  ( 
    input         clk,
    input         rst,
    input          en,
    // --- FIR in/outs begin --- //
    input signed [N-1:0] X_fir,
    input signed [N-1:0] b0_fir,
    input signed [N-1:0] b1_fir,
    output signed [N-1:0]   Y_fir,
    // --- FIR in/outs end --- //

    // --- IIR in/outs begin --- //
    input signed [N-1:0] X_iir,
    input signed [N-1:0] a0_iir,
    input signed [N-1:0] a1_iir,
    input signed [N-1:0] a2_iir,
    input signed [N-1:0] b1_iir,
    input signed [N-1:0] b2_iir,
    output signed  [N-1:0] Y_iir,
    // --- IIR in/outs end --- //

    // --- MAF in/outs begin --- //
    input [N-1:0] X_maf,
    output [N-1:0] Y_maf 
    // --- MAF in/outs end --- //
    );


//circuit architecture
reg signed [N-1:0] mux_coeff;
reg signed [N-1:0] mux_xy;
wire signed [2*N-1:0] product;
wire [N-1:0] aritmetic_shift;
reg signed [N-1:0] add1;
reg signed [N-1:0] result;

//fir REGs
reg signed [N-1:0] X1_fir, X2_fir, X3_fir, Yt_fir;

//iir REGs
reg signed [N-1:0] X1_iir, X2_iir;
reg signed [N-1:0] Yt_iir, Y1_iir, Y2_iir;
 
//maf REGs
reg  [N-1:0] X1_maf, X2_maf, X3_maf, Yt_maf;

//out REGs 
assign Y_fir = Yt_fir;
assign Y_iir = Yt_iir;
assign Y_maf = (Yt_maf >> 2);



// -------  fir, iir and maf samples flow start ------- //
always@(posedge clk) begin
    if(rst) begin

        //output regs
        Yt_fir <= 0;
        Yt_iir <= 0;
        Yt_maf <= 0;

        //fir
        X1_fir <= 0;
        X2_fir <= 0;
        X3_fir <= 0;

        //iir
        Y1_iir <= 0;
        Y2_iir <= 0;
        X1_iir <= 0;
        X2_iir <= 0;

        //maf
        X1_maf <= 0;
        X2_maf <= 0;
        X3_maf <= 0;

    end

    else if(en == 1) begin
        //fir 
        X1_fir <= X_fir;
        X2_fir <= X1_fir;
        X3_fir <= X2_fir;



        //iir
        Y1_iir <= Yt_iir; 
        Y2_iir <= Y1_iir;
        X1_iir <= X_iir;
        X2_iir <= X1_iir;

        //maf
        X1_maf <= X_maf;
        X2_maf <= X1_maf;
        X3_maf <= X2_maf;
    end

   else if(phase == phase4)
        Yt_fir <= result;
   else if(phase == phase10)
        Yt_iir <= result;
   else if(phase == phase15)
        Yt_maf <= result; 

end

// -------  fir, iir and maf samples flow end ------- //



// ================== FSMs start ================== //

reg [4:0] phase; //state
reg [4:0] next_phase; //next_state

parameter phase0 = 0; parameter phase1 = 1;
parameter phase2 = 2; parameter phase3 = 3;
parameter phase4 = 4; parameter phase5 = 5;
parameter phase6 = 6; parameter phase7 = 7;
parameter phase8 = 8; parameter phase9 = 9;

parameter phase10 = 10; parameter phase11 = 11;
parameter phase12 = 12; parameter phase13 = 13;
parameter phase14 = 14;
parameter phase15 = 15;
parameter phase16 = 16;


always @(posedge clk) begin 
        if (rst) 
         phase <= phase0;
        else if(en == 1 )
           phase <= phase0;
        else 
           phase <= next_phase;
end

// ------- MUX's coeffs - Samples FIR/IIR start ------- //
always @(phase or en or aritmetic_shift or X1_maf or  X2_maf or X3_maf) begin



        case(phase)

        //fir Yt = X*b0 + X1*b1 + X2*b1 + X3*b0  
            phase0    : begin
                mux_coeff = b0_fir;
                mux_xy = X_fir;
                add1 = aritmetic_shift;
                next_phase = phase1;
            end

            phase1    : begin
                mux_coeff = b1_fir;
                mux_xy = X1_fir;
                add1 = aritmetic_shift; 
                next_phase = phase2;
            end

            phase2    : begin
                mux_coeff = b1_fir;
                mux_xy = X2_fir;
                add1 = aritmetic_shift;
                next_phase = phase3;   
            end

            phase3    : begin
                mux_coeff = b0_fir;
                mux_xy = X3_fir;
                add1 = aritmetic_shift;
                next_phase = phase4;
            end


            phase4    : begin 
                next_phase = phase5;
            end

             //iir  Y <= X*a0 + X1*a1 + X2*a2 - Y1*b1 - Y2*b2;
            phase5    : begin
                mux_coeff = a0_iir;
                mux_xy = X_iir;
                add1 = aritmetic_shift; 
                next_phase = phase6;
            end

            phase6    : begin
                mux_coeff = a1_iir;
                mux_xy = X1_iir;
                add1 = aritmetic_shift;
                next_phase = phase7;
            end  	

            phase7     : begin
                mux_coeff = a2_iir;
                mux_xy = X2_iir;
                add1 = aritmetic_shift;
                next_phase = phase8;
            end

            phase8    : begin
                mux_coeff = b1_iir;
                mux_xy = -Y1_iir;
                add1 = aritmetic_shift;
                next_phase = phase9;
            end

            phase9    : begin
                mux_coeff = b2_iir;
                mux_xy = -Y2_iir;
                add1 = aritmetic_shift;
                next_phase = phase10; 
            end

            phase10    : begin

                next_phase = phase11; 
            end

            //MAF
            phase11    : begin
                add1 = X_maf;
                next_phase = phase12;
            end

            phase12    : begin
                add1 = X1_maf;
                next_phase = phase13;
            end

            phase13    : begin
                add1 = X2_maf;
                next_phase = phase14;
            end

            phase14    : begin
                add1 = X3_maf;
                next_phase = phase15;
            end
            phase15    : begin
                next_phase = phase16;
            end

             default : begin
                add1 = aritmetic_shift;                
             end  
        endcase
end

// ------- MUX's coeffs - Samples FIR/IIR end ------- //


// ================== FSMs end ================== //


// ------- Math operations begin ------- //
assign product = mux_coeff * mux_xy;
assign aritmetic_shift = product >>> 15;
// ------- Math operations end ------- //


// ------ Acumulacion start -------- //

always @(posedge clk) begin

    if(rst ==1'b1) 
        begin
            result <= 0;
        end
    else  if (en == 1)
            result <= 0;
    else  if (phase == phase4) 
            result <= 0;
    else  if (phase == phase10) 
            result <= 0;
    else
    	result <= result + add1;  //acumulacion

end

// ------ Acumulacion end -------- //

endmodule

