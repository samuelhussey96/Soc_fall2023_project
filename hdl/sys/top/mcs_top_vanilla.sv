module mcs_top_vanilla
#(parameter BRG_BASE = 32'hc000_0000)	
(
   input logic clk,
   input logic reset_n,
   // switches and LEDs
   input logic [15:0] sw,
   output logic [15:0] led,
   // uart
   input logic rx,
   output logic tx,
   // sseg
   output logic [7:0] sseg,
   output logic [7:0] an, 
   // SPI
   output logic acl_sclk,
   output logic acl_mosi,
   input  logic acl_miso,
   output logic acl_ss_n,
   // I2C
   output tri tmp_i2c_scl,
   inout  tri tmp_i2c_sda, 
   // XADC analog input pairs
   input logic [3:0] adc_p, adc_n,
   // DDFS audio
   output logic audio_on,audio_pdm,
   // PMOD JA (divided into top row and bottom row)
   output logic [4:1] ja_top,
   output logic [10:7] ja_btm
   
);

   // declaration
   logic clk_100M;
   logic reset_sys;
   // MCS IO bus
   logic io_addr_strobe;
   logic io_read_strobe;
   logic io_write_strobe;
   logic [3:0] io_byte_enable;
   logic [31:0] io_address;
   logic [31:0] io_write_data;
   logic [31:0] io_read_data;
   logic io_ready;
   // fpro bus 
   logic fp_mmio_cs; 
   logic fp_wr;      
   logic fp_rd;     
   logic [20:0] fp_addr;       
   logic [31:0] fp_wr_data;    
   logic [31:0] fp_rd_data;    
   // ddfs/audio pdm
   logic pdm, ddfs_sq_wave;
   // pwm 
   logic [7:0] pwm;

   // body
   assign clk_100M = clk;                  // 100 MHz external clock
   assign reset_sys = !reset_n;
   // audio
   assign audio_pdm = pdm;
   assign audio_on = 1'b1;
   // PMOD JA  
   assign ja_top[1] = ddfs_sq_wave;
   assign ja_top[2] = pdm;
   assign ja_top[4:3] = pwm[7:6];
   assign ja_btm = 4'b0000;
   //assign ja_top[4:1] = pwm[3:0];
   //assign ja_btm[10:7] = pwm[7:4];
   
   //instantiate uBlaze MCS
   cpu cpu_unit (
    .Clk(clk_100M),                     // input wire Clk
    .Reset(reset_sys),                  // input wire Reset
    .IO_addr_strobe(io_addr_strobe),    // output wire IO_addr_strobe
    .IO_address(io_address),            // output wire [31 : 0] IO_address
    .IO_byte_enable(io_byte_enable),    // output wire [3 : 0] IO_byte_enable
    .IO_read_data(io_read_data),        // input wire [31 : 0] IO_read_data
    .IO_read_strobe(io_read_strobe),    // output wire IO_read_strobe
    .IO_ready(io_ready),                // input wire IO_ready
    .IO_write_data(io_write_data),      // output wire [31 : 0] IO_write_data
    .IO_write_strobe(io_write_strobe)   // output wire IO_write_strobe
   );
    
   // instantiate bridge
   chu_mcs_bridge #(.BRG_BASE(BRG_BASE)) bridge_unit (.*, .fp_video_cs());
    
   // instantiated i/o subsystem
   mmio_sys_vanilla #(.N_SW(16),.N_LED(16)) mmio_unit (
   .clk(clk),
   .reset(reset_sys),
   .mmio_cs(fp_mmio_cs),
   .mmio_wr(fp_wr),
   .mmio_rd(fp_rd),
   .mmio_addr(fp_addr), 
   .mmio_wr_data(fp_wr_data),
   .mmio_rd_data(fp_rd_data),
   .sw(sw),
   .led(led),
   .rx(rx),
   .tx(tx),
   .sseg(sseg),
   .an(an),
   .spi_sclk(acl_sclk),
   .spi_mosi(acl_mosi),
   .spi_miso(acl_miso),
   .spi_ss_n(acl_ss_n),
   .scl(tmp_i2c_scl),
   .sda(tmp_i2c_sda),
   .adc_p(adc_p),
   .adc_n(adc_n),
   .pwm(pwm),
   .ddfs_sq_wave(ddfs_sq_wave), // square wave output
   .pdm(pdm) // 1-bit dac
  );   
endmodule    

