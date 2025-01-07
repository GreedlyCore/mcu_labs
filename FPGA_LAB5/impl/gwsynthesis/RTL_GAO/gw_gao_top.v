module gw_gao(
    data,
    ss,
    \d[4] ,
    \d[3] ,
    \d[2] ,
    \d[1] ,
    \d[0] ,
    clkoutd,
    \bit_index[4] ,
    \bit_index[3] ,
    \bit_index[2] ,
    \bit_index[1] ,
    \bit_index[0] ,
    \showing_number[31] ,
    \showing_number[30] ,
    \showing_number[29] ,
    \showing_number[28] ,
    \showing_number[27] ,
    \showing_number[26] ,
    \showing_number[25] ,
    \showing_number[24] ,
    \showing_number[23] ,
    \showing_number[22] ,
    \showing_number[21] ,
    \showing_number[20] ,
    \showing_number[19] ,
    \showing_number[18] ,
    \showing_number[17] ,
    \showing_number[16] ,
    \showing_number[15] ,
    \showing_number[14] ,
    \showing_number[13] ,
    \showing_number[12] ,
    \showing_number[11] ,
    \showing_number[10] ,
    \showing_number[9] ,
    \showing_number[8] ,
    \showing_number[7] ,
    \showing_number[6] ,
    \showing_number[5] ,
    \showing_number[4] ,
    \showing_number[3] ,
    \showing_number[2] ,
    \showing_number[1] ,
    \showing_number[0] ,
    \count_sec[15] ,
    \count_sec[14] ,
    \count_sec[13] ,
    \count_sec[12] ,
    \count_sec[11] ,
    \count_sec[10] ,
    \count_sec[9] ,
    \count_sec[8] ,
    \count_sec[7] ,
    \count_sec[6] ,
    \count_sec[5] ,
    \count_sec[4] ,
    \count_sec[3] ,
    \count_sec[2] ,
    \count_sec[1] ,
    \count_sec[0] ,
    \bcd0[3] ,
    \bcd0[2] ,
    \bcd0[1] ,
    \bcd0[0] ,
    \bcd1[3] ,
    \bcd1[2] ,
    \bcd1[1] ,
    \bcd1[0] ,
    clk,
    tms_pad_i,
    tck_pad_i,
    tdi_pad_i,
    tdo_pad_o
);

input data;
input ss;
input \d[4] ;
input \d[3] ;
input \d[2] ;
input \d[1] ;
input \d[0] ;
input clkoutd;
input \bit_index[4] ;
input \bit_index[3] ;
input \bit_index[2] ;
input \bit_index[1] ;
input \bit_index[0] ;
input \showing_number[31] ;
input \showing_number[30] ;
input \showing_number[29] ;
input \showing_number[28] ;
input \showing_number[27] ;
input \showing_number[26] ;
input \showing_number[25] ;
input \showing_number[24] ;
input \showing_number[23] ;
input \showing_number[22] ;
input \showing_number[21] ;
input \showing_number[20] ;
input \showing_number[19] ;
input \showing_number[18] ;
input \showing_number[17] ;
input \showing_number[16] ;
input \showing_number[15] ;
input \showing_number[14] ;
input \showing_number[13] ;
input \showing_number[12] ;
input \showing_number[11] ;
input \showing_number[10] ;
input \showing_number[9] ;
input \showing_number[8] ;
input \showing_number[7] ;
input \showing_number[6] ;
input \showing_number[5] ;
input \showing_number[4] ;
input \showing_number[3] ;
input \showing_number[2] ;
input \showing_number[1] ;
input \showing_number[0] ;
input \count_sec[15] ;
input \count_sec[14] ;
input \count_sec[13] ;
input \count_sec[12] ;
input \count_sec[11] ;
input \count_sec[10] ;
input \count_sec[9] ;
input \count_sec[8] ;
input \count_sec[7] ;
input \count_sec[6] ;
input \count_sec[5] ;
input \count_sec[4] ;
input \count_sec[3] ;
input \count_sec[2] ;
input \count_sec[1] ;
input \count_sec[0] ;
input \bcd0[3] ;
input \bcd0[2] ;
input \bcd0[1] ;
input \bcd0[0] ;
input \bcd1[3] ;
input \bcd1[2] ;
input \bcd1[1] ;
input \bcd1[0] ;
input clk;
input tms_pad_i;
input tck_pad_i;
input tdi_pad_i;
output tdo_pad_o;

wire data;
wire ss;
wire \d[4] ;
wire \d[3] ;
wire \d[2] ;
wire \d[1] ;
wire \d[0] ;
wire clkoutd;
wire \bit_index[4] ;
wire \bit_index[3] ;
wire \bit_index[2] ;
wire \bit_index[1] ;
wire \bit_index[0] ;
wire \showing_number[31] ;
wire \showing_number[30] ;
wire \showing_number[29] ;
wire \showing_number[28] ;
wire \showing_number[27] ;
wire \showing_number[26] ;
wire \showing_number[25] ;
wire \showing_number[24] ;
wire \showing_number[23] ;
wire \showing_number[22] ;
wire \showing_number[21] ;
wire \showing_number[20] ;
wire \showing_number[19] ;
wire \showing_number[18] ;
wire \showing_number[17] ;
wire \showing_number[16] ;
wire \showing_number[15] ;
wire \showing_number[14] ;
wire \showing_number[13] ;
wire \showing_number[12] ;
wire \showing_number[11] ;
wire \showing_number[10] ;
wire \showing_number[9] ;
wire \showing_number[8] ;
wire \showing_number[7] ;
wire \showing_number[6] ;
wire \showing_number[5] ;
wire \showing_number[4] ;
wire \showing_number[3] ;
wire \showing_number[2] ;
wire \showing_number[1] ;
wire \showing_number[0] ;
wire \count_sec[15] ;
wire \count_sec[14] ;
wire \count_sec[13] ;
wire \count_sec[12] ;
wire \count_sec[11] ;
wire \count_sec[10] ;
wire \count_sec[9] ;
wire \count_sec[8] ;
wire \count_sec[7] ;
wire \count_sec[6] ;
wire \count_sec[5] ;
wire \count_sec[4] ;
wire \count_sec[3] ;
wire \count_sec[2] ;
wire \count_sec[1] ;
wire \count_sec[0] ;
wire \bcd0[3] ;
wire \bcd0[2] ;
wire \bcd0[1] ;
wire \bcd0[0] ;
wire \bcd1[3] ;
wire \bcd1[2] ;
wire \bcd1[1] ;
wire \bcd1[0] ;
wire clk;
wire tms_pad_i;
wire tck_pad_i;
wire tdi_pad_i;
wire tdo_pad_o;
wire tms_i_c;
wire tck_i_c;
wire tdi_i_c;
wire tdo_o_c;
wire [9:0] control0;
wire gao_jtag_tck;
wire gao_jtag_reset;
wire run_test_idle_er1;
wire run_test_idle_er2;
wire shift_dr_capture_dr;
wire update_dr;
wire pause_dr;
wire enable_er1;
wire enable_er2;
wire gao_jtag_tdi;
wire tdo_er1;

IBUF tms_ibuf (
    .I(tms_pad_i),
    .O(tms_i_c)
);

IBUF tck_ibuf (
    .I(tck_pad_i),
    .O(tck_i_c)
);

IBUF tdi_ibuf (
    .I(tdi_pad_i),
    .O(tdi_i_c)
);

OBUF tdo_obuf (
    .I(tdo_o_c),
    .O(tdo_pad_o)
);

GW_JTAG  u_gw_jtag(
    .tms_pad_i(tms_i_c),
    .tck_pad_i(tck_i_c),
    .tdi_pad_i(tdi_i_c),
    .tdo_pad_o(tdo_o_c),
    .tck_o(gao_jtag_tck),
    .test_logic_reset_o(gao_jtag_reset),
    .run_test_idle_er1_o(run_test_idle_er1),
    .run_test_idle_er2_o(run_test_idle_er2),
    .shift_dr_capture_dr_o(shift_dr_capture_dr),
    .update_dr_o(update_dr),
    .pause_dr_o(pause_dr),
    .enable_er1_o(enable_er1),
    .enable_er2_o(enable_er2),
    .tdi_o(gao_jtag_tdi),
    .tdo_er1_i(tdo_er1),
    .tdo_er2_i(1'b0)
);

gw_con_top  u_icon_top(
    .tck_i(gao_jtag_tck),
    .tdi_i(gao_jtag_tdi),
    .tdo_o(tdo_er1),
    .rst_i(gao_jtag_reset),
    .control0(control0[9:0]),
    .enable_i(enable_er1),
    .shift_dr_capture_dr_i(shift_dr_capture_dr),
    .update_dr_i(update_dr)
);

ao_top u_ao_top(
    .control(control0[9:0]),
    .data_i({data,ss,\d[4] ,\d[3] ,\d[2] ,\d[1] ,\d[0] ,clkoutd,\bit_index[4] ,\bit_index[3] ,\bit_index[2] ,\bit_index[1] ,\bit_index[0] ,\showing_number[31] ,\showing_number[30] ,\showing_number[29] ,\showing_number[28] ,\showing_number[27] ,\showing_number[26] ,\showing_number[25] ,\showing_number[24] ,\showing_number[23] ,\showing_number[22] ,\showing_number[21] ,\showing_number[20] ,\showing_number[19] ,\showing_number[18] ,\showing_number[17] ,\showing_number[16] ,\showing_number[15] ,\showing_number[14] ,\showing_number[13] ,\showing_number[12] ,\showing_number[11] ,\showing_number[10] ,\showing_number[9] ,\showing_number[8] ,\showing_number[7] ,\showing_number[6] ,\showing_number[5] ,\showing_number[4] ,\showing_number[3] ,\showing_number[2] ,\showing_number[1] ,\showing_number[0] ,\count_sec[15] ,\count_sec[14] ,\count_sec[13] ,\count_sec[12] ,\count_sec[11] ,\count_sec[10] ,\count_sec[9] ,\count_sec[8] ,\count_sec[7] ,\count_sec[6] ,\count_sec[5] ,\count_sec[4] ,\count_sec[3] ,\count_sec[2] ,\count_sec[1] ,\count_sec[0] ,\bcd0[3] ,\bcd0[2] ,\bcd0[1] ,\bcd0[0] ,\bcd1[3] ,\bcd1[2] ,\bcd1[1] ,\bcd1[0] }),
    .clk_i(clk)
);

endmodule
