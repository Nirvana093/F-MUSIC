//Copyright (C)2014-2025 Gowin Semiconductor Corporation.
//All rights reserved.
//File Title: Template file for instantiation
//Tool Version: V1.9.12 (64-bit)
//Part Number: GW5AT-LV138PG484AC1/I0
//Device: GW5AT-138
//Device Version: B
//Created Time: Wed Oct 28 17:12:06 2026

//Change the instance name and port connections to the signal names
//--------Copy here to design--------

	Gowin_GoRV32_Top your_instance_name(
		.HCLK(HCLK), //input HCLK
		.PCLK(PCLK), //input PCLK
		.POR_N(POR_N), //input POR_N
		.EXT_GPIO_CLK(EXT_GPIO_CLK), //input EXT_GPIO_CLK
		.GPIO_IN(GPIO_IN), //input [31:0] GPIO_IN
		.GPIO_OE(GPIO_OE), //output [31:0] GPIO_OE
		.GPIO_OUT(GPIO_OUT), //output [31:0] GPIO_OUT
		.UART1_CTSN(UART1_CTSN), //input UART1_CTSN
		.UART1_DCDN(UART1_DCDN), //input UART1_DCDN
		.UART1_DSRN(UART1_DSRN), //input UART1_DSRN
		.UART1_RIN(UART1_RIN), //input UART1_RIN
		.UART1_RXD(UART1_RXD), //input UART1_RXD
		.UART1_DTRN(UART1_DTRN), //output UART1_DTRN
		.UART1_OUT1N(UART1_OUT1N), //output UART1_OUT1N
		.UART1_OUT2N(UART1_OUT2N), //output UART1_OUT2N
		.UART1_RTSN(UART1_RTSN), //output UART1_RTSN
		.UART1_TXD(UART1_TXD), //output UART1_TXD
		.FLASH_QSPI_CSN(FLASH_QSPI_CSN), //inout FLASH_QSPI_CSN
		.FLASH_QSPI_MISO(FLASH_QSPI_MISO), //inout FLASH_QSPI_MISO
		.FLASH_QSPI_MOSI(FLASH_QSPI_MOSI), //inout FLASH_QSPI_MOSI
		.FLASH_QSPI_CLK(FLASH_QSPI_CLK), //inout FLASH_QSPI_CLK
		.FLASH_QSPI_HOLDN(FLASH_QSPI_HOLDN), //inout FLASH_QSPI_HOLDN
		.FLASH_QSPI_WPN(FLASH_QSPI_WPN), //inout FLASH_QSPI_WPN
		.I2C_SCL_IN(I2C_SCL_IN), //input I2C_SCL_IN
		.I2C_SDA_IN(I2C_SDA_IN), //input I2C_SDA_IN
		.I2C_SCL_OUT(I2C_SCL_OUT), //output I2C_SCL_OUT
		.I2C_SDA_OUT(I2C_SDA_OUT), //output I2C_SDA_OUT
		.EXT_INT(EXT_INT), //input [7:0] EXT_INT
		.EXTS_HRDATA(EXTS_HRDATA), //input [31:0] EXTS_HRDATA
		.EXTS_HREADYIN(EXTS_HREADYIN), //input EXTS_HREADYIN
		.EXTS_HRESP(EXTS_HRESP), //input [1:0] EXTS_HRESP
		.EXTS_HREADYOUT(EXTS_HREADYOUT), //output EXTS_HREADYOUT
		.EXTS_HADDR(EXTS_HADDR), //output [31:0] EXTS_HADDR
		.EXTS_HBURST(EXTS_HBURST), //output [2:0] EXTS_HBURST
		.EXTS_HPROT(EXTS_HPROT), //output [3:0] EXTS_HPROT
		.EXTS_HSEL(EXTS_HSEL), //output EXTS_HSEL
		.EXTS_HSIZE(EXTS_HSIZE), //output [2:0] EXTS_HSIZE
		.EXTS_HTRANS(EXTS_HTRANS), //output [1:0] EXTS_HTRANS
		.EXTS_HWDATA(EXTS_HWDATA), //output [31:0] EXTS_HWDATA
		.EXTS_HWRITE(EXTS_HWRITE), //output EXTS_HWRITE
		.EXTS_HCLK(EXTS_HCLK), //output EXTS_HCLK
		.EXTS_HRSTN(EXTS_HRSTN), //output EXTS_HRSTN
		.JTAG_TMS(JTAG_TMS), //input JTAG_TMS
		.JTAG_TDI(JTAG_TDI), //input JTAG_TDI
		.JTAG_TDO(JTAG_TDO), //output JTAG_TDO
		.JTAG_TCK(JTAG_TCK) //input JTAG_TCK
	);

//--------Copy end-------------------
