//Copyright (C)2014-2025 Gowin Semiconductor Corporation.
//All rights reserved.
//File Title: IP file
//Tool Version: V1.9.12 (64-bit)
//Part Number: GW5AT-LV138PG484AC1/I0
//Device: GW5AT-138
//Device Version: B
//Created Time: Wed Oct 28 17:06:23 2026

module Gowin_pROM (dout, clk, oce, ce, reset, ad);

output [7:0] dout;
input clk;
input oce;
input ce;
input reset;
input [7:0] ad;

wire [23:0] prom_inst_0_dout_w;
wire gw_gnd;

assign gw_gnd = 1'b0;

pROM prom_inst_0 (
    .DO({prom_inst_0_dout_w[23:0],dout[7:0]}),
    .CLK(clk),
    .OCE(oce),
    .CE(ce),
    .RESET(reset),
    .AD({gw_gnd,gw_gnd,gw_gnd,ad[7:0],gw_gnd,gw_gnd,gw_gnd})
);

defparam prom_inst_0.READ_MODE = 1'b0;
defparam prom_inst_0.BIT_WIDTH = 8;
defparam prom_inst_0.RESET_MODE = "SYNC";
defparam prom_inst_0.INIT_RAM_00 = 256'h6664615F5C595654514E4B4845423E3B3835312E2A27231F1C1814100C080400;
defparam prom_inst_0.INIT_RAM_01 = 256'hA5A4A2A19F9E9C9A9997959392908E8C8A88868482807E7C79777573706E6B69;
defparam prom_inst_0.INIT_RAM_02 = 256'hCAC9C8C7C7C6C5C4C3C2C1C0BFBDBCBBBAB9B8B7B5B4B3B2B0AFAEACABAAA8A7;
defparam prom_inst_0.INIT_RAM_03 = 256'hE0DFDFDEDEDDDDDCDBDBDADAD9D8D8D7D6D6D5D4D4D3D2D1D1D0CFCECECDCCCB;
defparam prom_inst_0.INIT_RAM_04 = 256'hECECECECEBEBEBEAEAEAE9E9E8E8E8E7E7E7E6E6E5E5E5E4E4E3E3E2E2E1E1E0;
defparam prom_inst_0.INIT_RAM_05 = 256'hF4F4F4F3F3F3F3F3F2F2F2F2F2F1F1F1F1F0F0F0F0F0EFEFEFEEEEEEEEEDEDED;
defparam prom_inst_0.INIT_RAM_06 = 256'hF8F8F8F8F8F8F8F8F7F7F7F7F7F7F7F7F6F6F6F6F6F6F6F5F5F5F5F5F5F4F4F4;
defparam prom_inst_0.INIT_RAM_07 = 256'hFBFBFBFBFBFBFAFAFAFAFAFAFAFAFAFAFAFAFAFAF9F9F9F9F9F9F9F9F9F9F9F8;

endmodule //Gowin_pROM
