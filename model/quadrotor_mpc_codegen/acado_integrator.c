/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#include "acado_common.h"


real_t rk_dim22_swap;

/** Column vector of size: 22 */
real_t rk_dim22_bPerm[ 22 ];

real_t rk_ttt;

/** Row vector of size: 30 */
real_t rk_xxx[ 30 ];

/** Matrix of size: 11 x 2 (row major format) */
real_t rk_kkk[ 22 ];

/** Matrix of size: 22 x 22 (row major format) */
real_t rk_A[ 484 ];

/** Column vector of size: 22 */
real_t rk_b[ 22 ];

/** Row vector of size: 22 */
int rk_dim22_perm[ 22 ];

/** Column vector of size: 11 */
real_t rk_rhsTemp[ 11 ];

/** Matrix of size: 2 x 187 (row major format) */
real_t rk_diffsTemp2[ 374 ];

/** Matrix of size: 11 x 2 (row major format) */
real_t rk_diffK[ 22 ];

/** Matrix of size: 11 x 17 (row major format) */
real_t rk_diffsNew2[ 187 ];

#pragma omp threadprivate( auxVar, rk_ttt, rk_xxx, rk_kkk, rk_diffK, rk_rhsTemp, rk_dim22_perm, rk_A, rk_b, rk_diffsNew2, rk_diffsTemp2, rk_dim22_swap, rk_dim22_bPerm )

void acado_rhs(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 11;

/* Compute outputs: */
out[0] = xd[7];
out[1] = xd[8];
out[2] = xd[9];
out[3] = ((real_t)(5.0000000000000000e-01)*(((((real_t)(0.0000000000000000e+00)-u[1])*xd[4])-(u[2]*xd[5]))-(u[3]*xd[6])));
out[4] = ((real_t)(5.0000000000000000e-01)*(((u[1]*xd[3])+(u[3]*xd[5]))-(u[2]*xd[6])));
out[5] = ((real_t)(5.0000000000000000e-01)*(((u[2]*xd[3])-(u[3]*xd[4]))+(u[1]*xd[6])));
out[6] = ((real_t)(5.0000000000000000e-01)*(((u[3]*xd[3])+(u[2]*xd[4]))-(u[1]*xd[5])));
out[7] = (((real_t)(2.0000000000000000e+00)*((xd[3]*xd[5])+(xd[4]*xd[6])))*u[0]);
out[8] = (((real_t)(2.0000000000000000e+00)*((xd[5]*xd[6])-(xd[3]*xd[4])))*u[0]);
out[9] = (((((real_t)(1.0000000000000000e+00)-(((real_t)(2.0000000000000000e+00)*xd[4])*xd[4]))-(((real_t)(2.0000000000000000e+00)*xd[5])*xd[5]))*u[0])-(real_t)(9.8065999999999995e+00));
out[10] = (((real_t)(9.9999999999999995e-08)*u[4])+((real_t)(9.9999999999999995e-08)*u[5]));
}



void acado_diffs(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 11;

/* Compute outputs: */
out[0] = (real_t)(0.0000000000000000e+00);
out[1] = (real_t)(0.0000000000000000e+00);
out[2] = (real_t)(0.0000000000000000e+00);
out[3] = (real_t)(0.0000000000000000e+00);
out[4] = (real_t)(0.0000000000000000e+00);
out[5] = (real_t)(0.0000000000000000e+00);
out[6] = (real_t)(0.0000000000000000e+00);
out[7] = (real_t)(1.0000000000000000e+00);
out[8] = (real_t)(0.0000000000000000e+00);
out[9] = (real_t)(0.0000000000000000e+00);
out[10] = (real_t)(0.0000000000000000e+00);
out[11] = (real_t)(0.0000000000000000e+00);
out[12] = (real_t)(0.0000000000000000e+00);
out[13] = (real_t)(0.0000000000000000e+00);
out[14] = (real_t)(0.0000000000000000e+00);
out[15] = (real_t)(0.0000000000000000e+00);
out[16] = (real_t)(0.0000000000000000e+00);
out[17] = (real_t)(0.0000000000000000e+00);
out[18] = (real_t)(0.0000000000000000e+00);
out[19] = (real_t)(0.0000000000000000e+00);
out[20] = (real_t)(0.0000000000000000e+00);
out[21] = (real_t)(0.0000000000000000e+00);
out[22] = (real_t)(0.0000000000000000e+00);
out[23] = (real_t)(0.0000000000000000e+00);
out[24] = (real_t)(0.0000000000000000e+00);
out[25] = (real_t)(1.0000000000000000e+00);
out[26] = (real_t)(0.0000000000000000e+00);
out[27] = (real_t)(0.0000000000000000e+00);
out[28] = (real_t)(0.0000000000000000e+00);
out[29] = (real_t)(0.0000000000000000e+00);
out[30] = (real_t)(0.0000000000000000e+00);
out[31] = (real_t)(0.0000000000000000e+00);
out[32] = (real_t)(0.0000000000000000e+00);
out[33] = (real_t)(0.0000000000000000e+00);
out[34] = (real_t)(0.0000000000000000e+00);
out[35] = (real_t)(0.0000000000000000e+00);
out[36] = (real_t)(0.0000000000000000e+00);
out[37] = (real_t)(0.0000000000000000e+00);
out[38] = (real_t)(0.0000000000000000e+00);
out[39] = (real_t)(0.0000000000000000e+00);
out[40] = (real_t)(0.0000000000000000e+00);
out[41] = (real_t)(0.0000000000000000e+00);
out[42] = (real_t)(0.0000000000000000e+00);
out[43] = (real_t)(1.0000000000000000e+00);
out[44] = (real_t)(0.0000000000000000e+00);
out[45] = (real_t)(0.0000000000000000e+00);
out[46] = (real_t)(0.0000000000000000e+00);
out[47] = (real_t)(0.0000000000000000e+00);
out[48] = (real_t)(0.0000000000000000e+00);
out[49] = (real_t)(0.0000000000000000e+00);
out[50] = (real_t)(0.0000000000000000e+00);
out[51] = (real_t)(0.0000000000000000e+00);
out[52] = (real_t)(0.0000000000000000e+00);
out[53] = (real_t)(0.0000000000000000e+00);
out[54] = (real_t)(0.0000000000000000e+00);
out[55] = ((real_t)(5.0000000000000000e-01)*((real_t)(0.0000000000000000e+00)-u[1]));
out[56] = ((real_t)(5.0000000000000000e-01)*((real_t)(0.0000000000000000e+00)-u[2]));
out[57] = ((real_t)(5.0000000000000000e-01)*((real_t)(0.0000000000000000e+00)-u[3]));
out[58] = (real_t)(0.0000000000000000e+00);
out[59] = (real_t)(0.0000000000000000e+00);
out[60] = (real_t)(0.0000000000000000e+00);
out[61] = (real_t)(0.0000000000000000e+00);
out[62] = (real_t)(0.0000000000000000e+00);
out[63] = ((real_t)(5.0000000000000000e-01)*(((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00))*xd[4]));
out[64] = ((real_t)(5.0000000000000000e-01)*((real_t)(0.0000000000000000e+00)-xd[5]));
out[65] = ((real_t)(5.0000000000000000e-01)*((real_t)(0.0000000000000000e+00)-xd[6]));
out[66] = (real_t)(0.0000000000000000e+00);
out[67] = (real_t)(0.0000000000000000e+00);
out[68] = (real_t)(0.0000000000000000e+00);
out[69] = (real_t)(0.0000000000000000e+00);
out[70] = (real_t)(0.0000000000000000e+00);
out[71] = ((real_t)(5.0000000000000000e-01)*u[1]);
out[72] = (real_t)(0.0000000000000000e+00);
out[73] = ((real_t)(5.0000000000000000e-01)*u[3]);
out[74] = ((real_t)(5.0000000000000000e-01)*((real_t)(0.0000000000000000e+00)-u[2]));
out[75] = (real_t)(0.0000000000000000e+00);
out[76] = (real_t)(0.0000000000000000e+00);
out[77] = (real_t)(0.0000000000000000e+00);
out[78] = (real_t)(0.0000000000000000e+00);
out[79] = (real_t)(0.0000000000000000e+00);
out[80] = ((real_t)(5.0000000000000000e-01)*xd[3]);
out[81] = ((real_t)(5.0000000000000000e-01)*((real_t)(0.0000000000000000e+00)-xd[6]));
out[82] = ((real_t)(5.0000000000000000e-01)*xd[5]);
out[83] = (real_t)(0.0000000000000000e+00);
out[84] = (real_t)(0.0000000000000000e+00);
out[85] = (real_t)(0.0000000000000000e+00);
out[86] = (real_t)(0.0000000000000000e+00);
out[87] = (real_t)(0.0000000000000000e+00);
out[88] = ((real_t)(5.0000000000000000e-01)*u[2]);
out[89] = ((real_t)(5.0000000000000000e-01)*((real_t)(0.0000000000000000e+00)-u[3]));
out[90] = (real_t)(0.0000000000000000e+00);
out[91] = ((real_t)(5.0000000000000000e-01)*u[1]);
out[92] = (real_t)(0.0000000000000000e+00);
out[93] = (real_t)(0.0000000000000000e+00);
out[94] = (real_t)(0.0000000000000000e+00);
out[95] = (real_t)(0.0000000000000000e+00);
out[96] = (real_t)(0.0000000000000000e+00);
out[97] = ((real_t)(5.0000000000000000e-01)*xd[6]);
out[98] = ((real_t)(5.0000000000000000e-01)*xd[3]);
out[99] = ((real_t)(5.0000000000000000e-01)*((real_t)(0.0000000000000000e+00)-xd[4]));
out[100] = (real_t)(0.0000000000000000e+00);
out[101] = (real_t)(0.0000000000000000e+00);
out[102] = (real_t)(0.0000000000000000e+00);
out[103] = (real_t)(0.0000000000000000e+00);
out[104] = (real_t)(0.0000000000000000e+00);
out[105] = ((real_t)(5.0000000000000000e-01)*u[3]);
out[106] = ((real_t)(5.0000000000000000e-01)*u[2]);
out[107] = ((real_t)(5.0000000000000000e-01)*((real_t)(0.0000000000000000e+00)-u[1]));
out[108] = (real_t)(0.0000000000000000e+00);
out[109] = (real_t)(0.0000000000000000e+00);
out[110] = (real_t)(0.0000000000000000e+00);
out[111] = (real_t)(0.0000000000000000e+00);
out[112] = (real_t)(0.0000000000000000e+00);
out[113] = (real_t)(0.0000000000000000e+00);
out[114] = ((real_t)(5.0000000000000000e-01)*((real_t)(0.0000000000000000e+00)-xd[5]));
out[115] = ((real_t)(5.0000000000000000e-01)*xd[4]);
out[116] = ((real_t)(5.0000000000000000e-01)*xd[3]);
out[117] = (real_t)(0.0000000000000000e+00);
out[118] = (real_t)(0.0000000000000000e+00);
out[119] = (real_t)(0.0000000000000000e+00);
out[120] = (real_t)(0.0000000000000000e+00);
out[121] = (real_t)(0.0000000000000000e+00);
out[122] = (((real_t)(2.0000000000000000e+00)*xd[5])*u[0]);
out[123] = (((real_t)(2.0000000000000000e+00)*xd[6])*u[0]);
out[124] = (((real_t)(2.0000000000000000e+00)*xd[3])*u[0]);
out[125] = (((real_t)(2.0000000000000000e+00)*xd[4])*u[0]);
out[126] = (real_t)(0.0000000000000000e+00);
out[127] = (real_t)(0.0000000000000000e+00);
out[128] = (real_t)(0.0000000000000000e+00);
out[129] = (real_t)(0.0000000000000000e+00);
out[130] = ((real_t)(2.0000000000000000e+00)*((xd[3]*xd[5])+(xd[4]*xd[6])));
out[131] = (real_t)(0.0000000000000000e+00);
out[132] = (real_t)(0.0000000000000000e+00);
out[133] = (real_t)(0.0000000000000000e+00);
out[134] = (real_t)(0.0000000000000000e+00);
out[135] = (real_t)(0.0000000000000000e+00);
out[136] = (real_t)(0.0000000000000000e+00);
out[137] = (real_t)(0.0000000000000000e+00);
out[138] = (real_t)(0.0000000000000000e+00);
out[139] = (((real_t)(2.0000000000000000e+00)*((real_t)(0.0000000000000000e+00)-xd[4]))*u[0]);
out[140] = (((real_t)(2.0000000000000000e+00)*((real_t)(0.0000000000000000e+00)-xd[3]))*u[0]);
out[141] = (((real_t)(2.0000000000000000e+00)*xd[6])*u[0]);
out[142] = (((real_t)(2.0000000000000000e+00)*xd[5])*u[0]);
out[143] = (real_t)(0.0000000000000000e+00);
out[144] = (real_t)(0.0000000000000000e+00);
out[145] = (real_t)(0.0000000000000000e+00);
out[146] = (real_t)(0.0000000000000000e+00);
out[147] = ((real_t)(2.0000000000000000e+00)*((xd[5]*xd[6])-(xd[3]*xd[4])));
out[148] = (real_t)(0.0000000000000000e+00);
out[149] = (real_t)(0.0000000000000000e+00);
out[150] = (real_t)(0.0000000000000000e+00);
out[151] = (real_t)(0.0000000000000000e+00);
out[152] = (real_t)(0.0000000000000000e+00);
out[153] = (real_t)(0.0000000000000000e+00);
out[154] = (real_t)(0.0000000000000000e+00);
out[155] = (real_t)(0.0000000000000000e+00);
out[156] = (real_t)(0.0000000000000000e+00);
out[157] = (((real_t)(0.0000000000000000e+00)-(((real_t)(2.0000000000000000e+00)*xd[4])+((real_t)(2.0000000000000000e+00)*xd[4])))*u[0]);
out[158] = (((real_t)(0.0000000000000000e+00)-(((real_t)(2.0000000000000000e+00)*xd[5])+((real_t)(2.0000000000000000e+00)*xd[5])))*u[0]);
out[159] = (real_t)(0.0000000000000000e+00);
out[160] = (real_t)(0.0000000000000000e+00);
out[161] = (real_t)(0.0000000000000000e+00);
out[162] = (real_t)(0.0000000000000000e+00);
out[163] = (real_t)(0.0000000000000000e+00);
out[164] = (((real_t)(1.0000000000000000e+00)-(((real_t)(2.0000000000000000e+00)*xd[4])*xd[4]))-(((real_t)(2.0000000000000000e+00)*xd[5])*xd[5]));
out[165] = (real_t)(0.0000000000000000e+00);
out[166] = (real_t)(0.0000000000000000e+00);
out[167] = (real_t)(0.0000000000000000e+00);
out[168] = (real_t)(0.0000000000000000e+00);
out[169] = (real_t)(0.0000000000000000e+00);
out[170] = (real_t)(0.0000000000000000e+00);
out[171] = (real_t)(0.0000000000000000e+00);
out[172] = (real_t)(0.0000000000000000e+00);
out[173] = (real_t)(0.0000000000000000e+00);
out[174] = (real_t)(0.0000000000000000e+00);
out[175] = (real_t)(0.0000000000000000e+00);
out[176] = (real_t)(0.0000000000000000e+00);
out[177] = (real_t)(0.0000000000000000e+00);
out[178] = (real_t)(0.0000000000000000e+00);
out[179] = (real_t)(0.0000000000000000e+00);
out[180] = (real_t)(0.0000000000000000e+00);
out[181] = (real_t)(0.0000000000000000e+00);
out[182] = (real_t)(0.0000000000000000e+00);
out[183] = (real_t)(0.0000000000000000e+00);
out[184] = (real_t)(0.0000000000000000e+00);
out[185] = (real_t)(9.9999999999999995e-08);
out[186] = (real_t)(9.9999999999999995e-08);
}



void acado_solve_dim22_triangular( real_t* const A, real_t* const b )
{

b[21] = b[21]/A[483];
b[20] -= + A[461]*b[21];
b[20] = b[20]/A[460];
b[19] -= + A[439]*b[21];
b[19] -= + A[438]*b[20];
b[19] = b[19]/A[437];
b[18] -= + A[417]*b[21];
b[18] -= + A[416]*b[20];
b[18] -= + A[415]*b[19];
b[18] = b[18]/A[414];
b[17] -= + A[395]*b[21];
b[17] -= + A[394]*b[20];
b[17] -= + A[393]*b[19];
b[17] -= + A[392]*b[18];
b[17] = b[17]/A[391];
b[16] -= + A[373]*b[21];
b[16] -= + A[372]*b[20];
b[16] -= + A[371]*b[19];
b[16] -= + A[370]*b[18];
b[16] -= + A[369]*b[17];
b[16] = b[16]/A[368];
b[15] -= + A[351]*b[21];
b[15] -= + A[350]*b[20];
b[15] -= + A[349]*b[19];
b[15] -= + A[348]*b[18];
b[15] -= + A[347]*b[17];
b[15] -= + A[346]*b[16];
b[15] = b[15]/A[345];
b[14] -= + A[329]*b[21];
b[14] -= + A[328]*b[20];
b[14] -= + A[327]*b[19];
b[14] -= + A[326]*b[18];
b[14] -= + A[325]*b[17];
b[14] -= + A[324]*b[16];
b[14] -= + A[323]*b[15];
b[14] = b[14]/A[322];
b[13] -= + A[307]*b[21];
b[13] -= + A[306]*b[20];
b[13] -= + A[305]*b[19];
b[13] -= + A[304]*b[18];
b[13] -= + A[303]*b[17];
b[13] -= + A[302]*b[16];
b[13] -= + A[301]*b[15];
b[13] -= + A[300]*b[14];
b[13] = b[13]/A[299];
b[12] -= + A[285]*b[21];
b[12] -= + A[284]*b[20];
b[12] -= + A[283]*b[19];
b[12] -= + A[282]*b[18];
b[12] -= + A[281]*b[17];
b[12] -= + A[280]*b[16];
b[12] -= + A[279]*b[15];
b[12] -= + A[278]*b[14];
b[12] -= + A[277]*b[13];
b[12] = b[12]/A[276];
b[11] -= + A[263]*b[21];
b[11] -= + A[262]*b[20];
b[11] -= + A[261]*b[19];
b[11] -= + A[260]*b[18];
b[11] -= + A[259]*b[17];
b[11] -= + A[258]*b[16];
b[11] -= + A[257]*b[15];
b[11] -= + A[256]*b[14];
b[11] -= + A[255]*b[13];
b[11] -= + A[254]*b[12];
b[11] = b[11]/A[253];
b[10] -= + A[241]*b[21];
b[10] -= + A[240]*b[20];
b[10] -= + A[239]*b[19];
b[10] -= + A[238]*b[18];
b[10] -= + A[237]*b[17];
b[10] -= + A[236]*b[16];
b[10] -= + A[235]*b[15];
b[10] -= + A[234]*b[14];
b[10] -= + A[233]*b[13];
b[10] -= + A[232]*b[12];
b[10] -= + A[231]*b[11];
b[10] = b[10]/A[230];
b[9] -= + A[219]*b[21];
b[9] -= + A[218]*b[20];
b[9] -= + A[217]*b[19];
b[9] -= + A[216]*b[18];
b[9] -= + A[215]*b[17];
b[9] -= + A[214]*b[16];
b[9] -= + A[213]*b[15];
b[9] -= + A[212]*b[14];
b[9] -= + A[211]*b[13];
b[9] -= + A[210]*b[12];
b[9] -= + A[209]*b[11];
b[9] -= + A[208]*b[10];
b[9] = b[9]/A[207];
b[8] -= + A[197]*b[21];
b[8] -= + A[196]*b[20];
b[8] -= + A[195]*b[19];
b[8] -= + A[194]*b[18];
b[8] -= + A[193]*b[17];
b[8] -= + A[192]*b[16];
b[8] -= + A[191]*b[15];
b[8] -= + A[190]*b[14];
b[8] -= + A[189]*b[13];
b[8] -= + A[188]*b[12];
b[8] -= + A[187]*b[11];
b[8] -= + A[186]*b[10];
b[8] -= + A[185]*b[9];
b[8] = b[8]/A[184];
b[7] -= + A[175]*b[21];
b[7] -= + A[174]*b[20];
b[7] -= + A[173]*b[19];
b[7] -= + A[172]*b[18];
b[7] -= + A[171]*b[17];
b[7] -= + A[170]*b[16];
b[7] -= + A[169]*b[15];
b[7] -= + A[168]*b[14];
b[7] -= + A[167]*b[13];
b[7] -= + A[166]*b[12];
b[7] -= + A[165]*b[11];
b[7] -= + A[164]*b[10];
b[7] -= + A[163]*b[9];
b[7] -= + A[162]*b[8];
b[7] = b[7]/A[161];
b[6] -= + A[153]*b[21];
b[6] -= + A[152]*b[20];
b[6] -= + A[151]*b[19];
b[6] -= + A[150]*b[18];
b[6] -= + A[149]*b[17];
b[6] -= + A[148]*b[16];
b[6] -= + A[147]*b[15];
b[6] -= + A[146]*b[14];
b[6] -= + A[145]*b[13];
b[6] -= + A[144]*b[12];
b[6] -= + A[143]*b[11];
b[6] -= + A[142]*b[10];
b[6] -= + A[141]*b[9];
b[6] -= + A[140]*b[8];
b[6] -= + A[139]*b[7];
b[6] = b[6]/A[138];
b[5] -= + A[131]*b[21];
b[5] -= + A[130]*b[20];
b[5] -= + A[129]*b[19];
b[5] -= + A[128]*b[18];
b[5] -= + A[127]*b[17];
b[5] -= + A[126]*b[16];
b[5] -= + A[125]*b[15];
b[5] -= + A[124]*b[14];
b[5] -= + A[123]*b[13];
b[5] -= + A[122]*b[12];
b[5] -= + A[121]*b[11];
b[5] -= + A[120]*b[10];
b[5] -= + A[119]*b[9];
b[5] -= + A[118]*b[8];
b[5] -= + A[117]*b[7];
b[5] -= + A[116]*b[6];
b[5] = b[5]/A[115];
b[4] -= + A[109]*b[21];
b[4] -= + A[108]*b[20];
b[4] -= + A[107]*b[19];
b[4] -= + A[106]*b[18];
b[4] -= + A[105]*b[17];
b[4] -= + A[104]*b[16];
b[4] -= + A[103]*b[15];
b[4] -= + A[102]*b[14];
b[4] -= + A[101]*b[13];
b[4] -= + A[100]*b[12];
b[4] -= + A[99]*b[11];
b[4] -= + A[98]*b[10];
b[4] -= + A[97]*b[9];
b[4] -= + A[96]*b[8];
b[4] -= + A[95]*b[7];
b[4] -= + A[94]*b[6];
b[4] -= + A[93]*b[5];
b[4] = b[4]/A[92];
b[3] -= + A[87]*b[21];
b[3] -= + A[86]*b[20];
b[3] -= + A[85]*b[19];
b[3] -= + A[84]*b[18];
b[3] -= + A[83]*b[17];
b[3] -= + A[82]*b[16];
b[3] -= + A[81]*b[15];
b[3] -= + A[80]*b[14];
b[3] -= + A[79]*b[13];
b[3] -= + A[78]*b[12];
b[3] -= + A[77]*b[11];
b[3] -= + A[76]*b[10];
b[3] -= + A[75]*b[9];
b[3] -= + A[74]*b[8];
b[3] -= + A[73]*b[7];
b[3] -= + A[72]*b[6];
b[3] -= + A[71]*b[5];
b[3] -= + A[70]*b[4];
b[3] = b[3]/A[69];
b[2] -= + A[65]*b[21];
b[2] -= + A[64]*b[20];
b[2] -= + A[63]*b[19];
b[2] -= + A[62]*b[18];
b[2] -= + A[61]*b[17];
b[2] -= + A[60]*b[16];
b[2] -= + A[59]*b[15];
b[2] -= + A[58]*b[14];
b[2] -= + A[57]*b[13];
b[2] -= + A[56]*b[12];
b[2] -= + A[55]*b[11];
b[2] -= + A[54]*b[10];
b[2] -= + A[53]*b[9];
b[2] -= + A[52]*b[8];
b[2] -= + A[51]*b[7];
b[2] -= + A[50]*b[6];
b[2] -= + A[49]*b[5];
b[2] -= + A[48]*b[4];
b[2] -= + A[47]*b[3];
b[2] = b[2]/A[46];
b[1] -= + A[43]*b[21];
b[1] -= + A[42]*b[20];
b[1] -= + A[41]*b[19];
b[1] -= + A[40]*b[18];
b[1] -= + A[39]*b[17];
b[1] -= + A[38]*b[16];
b[1] -= + A[37]*b[15];
b[1] -= + A[36]*b[14];
b[1] -= + A[35]*b[13];
b[1] -= + A[34]*b[12];
b[1] -= + A[33]*b[11];
b[1] -= + A[32]*b[10];
b[1] -= + A[31]*b[9];
b[1] -= + A[30]*b[8];
b[1] -= + A[29]*b[7];
b[1] -= + A[28]*b[6];
b[1] -= + A[27]*b[5];
b[1] -= + A[26]*b[4];
b[1] -= + A[25]*b[3];
b[1] -= + A[24]*b[2];
b[1] = b[1]/A[23];
b[0] -= + A[21]*b[21];
b[0] -= + A[20]*b[20];
b[0] -= + A[19]*b[19];
b[0] -= + A[18]*b[18];
b[0] -= + A[17]*b[17];
b[0] -= + A[16]*b[16];
b[0] -= + A[15]*b[15];
b[0] -= + A[14]*b[14];
b[0] -= + A[13]*b[13];
b[0] -= + A[12]*b[12];
b[0] -= + A[11]*b[11];
b[0] -= + A[10]*b[10];
b[0] -= + A[9]*b[9];
b[0] -= + A[8]*b[8];
b[0] -= + A[7]*b[7];
b[0] -= + A[6]*b[6];
b[0] -= + A[5]*b[5];
b[0] -= + A[4]*b[4];
b[0] -= + A[3]*b[3];
b[0] -= + A[2]*b[2];
b[0] -= + A[1]*b[1];
b[0] = b[0]/A[0];
}

real_t acado_solve_dim22_system( real_t* const A, real_t* const b, int* const rk_perm )
{
real_t det;

int i;
int j;
int k;

int indexMax;

int intSwap;

real_t valueMax;

real_t temp;

for (i = 0; i < 22; ++i)
{
rk_perm[i] = i;
}
det = 1.0000000000000000e+00;
for( i=0; i < (21); i++ ) {
	indexMax = i;
	valueMax = fabs(A[i*22+i]);
	for( j=(i+1); j < 22; j++ ) {
		temp = fabs(A[j*22+i]);
		if( temp > valueMax ) {
			indexMax = j;
			valueMax = temp;
		}
	}
	if( indexMax > i ) {
for (k = 0; k < 22; ++k)
{
	rk_dim22_swap = A[i*22+k];
	A[i*22+k] = A[indexMax*22+k];
	A[indexMax*22+k] = rk_dim22_swap;
}
	rk_dim22_swap = b[i];
	b[i] = b[indexMax];
	b[indexMax] = rk_dim22_swap;
	intSwap = rk_perm[i];
	rk_perm[i] = rk_perm[indexMax];
	rk_perm[indexMax] = intSwap;
	}
	det *= A[i*22+i];
	for( j=i+1; j < 22; j++ ) {
		A[j*22+i] = -A[j*22+i]/A[i*22+i];
		for( k=i+1; k < 22; k++ ) {
			A[j*22+k] += A[j*22+i] * A[i*22+k];
		}
		b[j] += A[j*22+i] * b[i];
	}
}
det *= A[483];
det = fabs(det);
acado_solve_dim22_triangular( A, b );
return det;
}

void acado_solve_dim22_system_reuse( real_t* const A, real_t* const b, int* const rk_perm )
{

rk_dim22_bPerm[0] = b[rk_perm[0]];
rk_dim22_bPerm[1] = b[rk_perm[1]];
rk_dim22_bPerm[2] = b[rk_perm[2]];
rk_dim22_bPerm[3] = b[rk_perm[3]];
rk_dim22_bPerm[4] = b[rk_perm[4]];
rk_dim22_bPerm[5] = b[rk_perm[5]];
rk_dim22_bPerm[6] = b[rk_perm[6]];
rk_dim22_bPerm[7] = b[rk_perm[7]];
rk_dim22_bPerm[8] = b[rk_perm[8]];
rk_dim22_bPerm[9] = b[rk_perm[9]];
rk_dim22_bPerm[10] = b[rk_perm[10]];
rk_dim22_bPerm[11] = b[rk_perm[11]];
rk_dim22_bPerm[12] = b[rk_perm[12]];
rk_dim22_bPerm[13] = b[rk_perm[13]];
rk_dim22_bPerm[14] = b[rk_perm[14]];
rk_dim22_bPerm[15] = b[rk_perm[15]];
rk_dim22_bPerm[16] = b[rk_perm[16]];
rk_dim22_bPerm[17] = b[rk_perm[17]];
rk_dim22_bPerm[18] = b[rk_perm[18]];
rk_dim22_bPerm[19] = b[rk_perm[19]];
rk_dim22_bPerm[20] = b[rk_perm[20]];
rk_dim22_bPerm[21] = b[rk_perm[21]];
rk_dim22_bPerm[1] += A[22]*rk_dim22_bPerm[0];

rk_dim22_bPerm[2] += A[44]*rk_dim22_bPerm[0];
rk_dim22_bPerm[2] += A[45]*rk_dim22_bPerm[1];

rk_dim22_bPerm[3] += A[66]*rk_dim22_bPerm[0];
rk_dim22_bPerm[3] += A[67]*rk_dim22_bPerm[1];
rk_dim22_bPerm[3] += A[68]*rk_dim22_bPerm[2];

rk_dim22_bPerm[4] += A[88]*rk_dim22_bPerm[0];
rk_dim22_bPerm[4] += A[89]*rk_dim22_bPerm[1];
rk_dim22_bPerm[4] += A[90]*rk_dim22_bPerm[2];
rk_dim22_bPerm[4] += A[91]*rk_dim22_bPerm[3];

rk_dim22_bPerm[5] += A[110]*rk_dim22_bPerm[0];
rk_dim22_bPerm[5] += A[111]*rk_dim22_bPerm[1];
rk_dim22_bPerm[5] += A[112]*rk_dim22_bPerm[2];
rk_dim22_bPerm[5] += A[113]*rk_dim22_bPerm[3];
rk_dim22_bPerm[5] += A[114]*rk_dim22_bPerm[4];

rk_dim22_bPerm[6] += A[132]*rk_dim22_bPerm[0];
rk_dim22_bPerm[6] += A[133]*rk_dim22_bPerm[1];
rk_dim22_bPerm[6] += A[134]*rk_dim22_bPerm[2];
rk_dim22_bPerm[6] += A[135]*rk_dim22_bPerm[3];
rk_dim22_bPerm[6] += A[136]*rk_dim22_bPerm[4];
rk_dim22_bPerm[6] += A[137]*rk_dim22_bPerm[5];

rk_dim22_bPerm[7] += A[154]*rk_dim22_bPerm[0];
rk_dim22_bPerm[7] += A[155]*rk_dim22_bPerm[1];
rk_dim22_bPerm[7] += A[156]*rk_dim22_bPerm[2];
rk_dim22_bPerm[7] += A[157]*rk_dim22_bPerm[3];
rk_dim22_bPerm[7] += A[158]*rk_dim22_bPerm[4];
rk_dim22_bPerm[7] += A[159]*rk_dim22_bPerm[5];
rk_dim22_bPerm[7] += A[160]*rk_dim22_bPerm[6];

rk_dim22_bPerm[8] += A[176]*rk_dim22_bPerm[0];
rk_dim22_bPerm[8] += A[177]*rk_dim22_bPerm[1];
rk_dim22_bPerm[8] += A[178]*rk_dim22_bPerm[2];
rk_dim22_bPerm[8] += A[179]*rk_dim22_bPerm[3];
rk_dim22_bPerm[8] += A[180]*rk_dim22_bPerm[4];
rk_dim22_bPerm[8] += A[181]*rk_dim22_bPerm[5];
rk_dim22_bPerm[8] += A[182]*rk_dim22_bPerm[6];
rk_dim22_bPerm[8] += A[183]*rk_dim22_bPerm[7];

rk_dim22_bPerm[9] += A[198]*rk_dim22_bPerm[0];
rk_dim22_bPerm[9] += A[199]*rk_dim22_bPerm[1];
rk_dim22_bPerm[9] += A[200]*rk_dim22_bPerm[2];
rk_dim22_bPerm[9] += A[201]*rk_dim22_bPerm[3];
rk_dim22_bPerm[9] += A[202]*rk_dim22_bPerm[4];
rk_dim22_bPerm[9] += A[203]*rk_dim22_bPerm[5];
rk_dim22_bPerm[9] += A[204]*rk_dim22_bPerm[6];
rk_dim22_bPerm[9] += A[205]*rk_dim22_bPerm[7];
rk_dim22_bPerm[9] += A[206]*rk_dim22_bPerm[8];

rk_dim22_bPerm[10] += A[220]*rk_dim22_bPerm[0];
rk_dim22_bPerm[10] += A[221]*rk_dim22_bPerm[1];
rk_dim22_bPerm[10] += A[222]*rk_dim22_bPerm[2];
rk_dim22_bPerm[10] += A[223]*rk_dim22_bPerm[3];
rk_dim22_bPerm[10] += A[224]*rk_dim22_bPerm[4];
rk_dim22_bPerm[10] += A[225]*rk_dim22_bPerm[5];
rk_dim22_bPerm[10] += A[226]*rk_dim22_bPerm[6];
rk_dim22_bPerm[10] += A[227]*rk_dim22_bPerm[7];
rk_dim22_bPerm[10] += A[228]*rk_dim22_bPerm[8];
rk_dim22_bPerm[10] += A[229]*rk_dim22_bPerm[9];

rk_dim22_bPerm[11] += A[242]*rk_dim22_bPerm[0];
rk_dim22_bPerm[11] += A[243]*rk_dim22_bPerm[1];
rk_dim22_bPerm[11] += A[244]*rk_dim22_bPerm[2];
rk_dim22_bPerm[11] += A[245]*rk_dim22_bPerm[3];
rk_dim22_bPerm[11] += A[246]*rk_dim22_bPerm[4];
rk_dim22_bPerm[11] += A[247]*rk_dim22_bPerm[5];
rk_dim22_bPerm[11] += A[248]*rk_dim22_bPerm[6];
rk_dim22_bPerm[11] += A[249]*rk_dim22_bPerm[7];
rk_dim22_bPerm[11] += A[250]*rk_dim22_bPerm[8];
rk_dim22_bPerm[11] += A[251]*rk_dim22_bPerm[9];
rk_dim22_bPerm[11] += A[252]*rk_dim22_bPerm[10];

rk_dim22_bPerm[12] += A[264]*rk_dim22_bPerm[0];
rk_dim22_bPerm[12] += A[265]*rk_dim22_bPerm[1];
rk_dim22_bPerm[12] += A[266]*rk_dim22_bPerm[2];
rk_dim22_bPerm[12] += A[267]*rk_dim22_bPerm[3];
rk_dim22_bPerm[12] += A[268]*rk_dim22_bPerm[4];
rk_dim22_bPerm[12] += A[269]*rk_dim22_bPerm[5];
rk_dim22_bPerm[12] += A[270]*rk_dim22_bPerm[6];
rk_dim22_bPerm[12] += A[271]*rk_dim22_bPerm[7];
rk_dim22_bPerm[12] += A[272]*rk_dim22_bPerm[8];
rk_dim22_bPerm[12] += A[273]*rk_dim22_bPerm[9];
rk_dim22_bPerm[12] += A[274]*rk_dim22_bPerm[10];
rk_dim22_bPerm[12] += A[275]*rk_dim22_bPerm[11];

rk_dim22_bPerm[13] += A[286]*rk_dim22_bPerm[0];
rk_dim22_bPerm[13] += A[287]*rk_dim22_bPerm[1];
rk_dim22_bPerm[13] += A[288]*rk_dim22_bPerm[2];
rk_dim22_bPerm[13] += A[289]*rk_dim22_bPerm[3];
rk_dim22_bPerm[13] += A[290]*rk_dim22_bPerm[4];
rk_dim22_bPerm[13] += A[291]*rk_dim22_bPerm[5];
rk_dim22_bPerm[13] += A[292]*rk_dim22_bPerm[6];
rk_dim22_bPerm[13] += A[293]*rk_dim22_bPerm[7];
rk_dim22_bPerm[13] += A[294]*rk_dim22_bPerm[8];
rk_dim22_bPerm[13] += A[295]*rk_dim22_bPerm[9];
rk_dim22_bPerm[13] += A[296]*rk_dim22_bPerm[10];
rk_dim22_bPerm[13] += A[297]*rk_dim22_bPerm[11];
rk_dim22_bPerm[13] += A[298]*rk_dim22_bPerm[12];

rk_dim22_bPerm[14] += A[308]*rk_dim22_bPerm[0];
rk_dim22_bPerm[14] += A[309]*rk_dim22_bPerm[1];
rk_dim22_bPerm[14] += A[310]*rk_dim22_bPerm[2];
rk_dim22_bPerm[14] += A[311]*rk_dim22_bPerm[3];
rk_dim22_bPerm[14] += A[312]*rk_dim22_bPerm[4];
rk_dim22_bPerm[14] += A[313]*rk_dim22_bPerm[5];
rk_dim22_bPerm[14] += A[314]*rk_dim22_bPerm[6];
rk_dim22_bPerm[14] += A[315]*rk_dim22_bPerm[7];
rk_dim22_bPerm[14] += A[316]*rk_dim22_bPerm[8];
rk_dim22_bPerm[14] += A[317]*rk_dim22_bPerm[9];
rk_dim22_bPerm[14] += A[318]*rk_dim22_bPerm[10];
rk_dim22_bPerm[14] += A[319]*rk_dim22_bPerm[11];
rk_dim22_bPerm[14] += A[320]*rk_dim22_bPerm[12];
rk_dim22_bPerm[14] += A[321]*rk_dim22_bPerm[13];

rk_dim22_bPerm[15] += A[330]*rk_dim22_bPerm[0];
rk_dim22_bPerm[15] += A[331]*rk_dim22_bPerm[1];
rk_dim22_bPerm[15] += A[332]*rk_dim22_bPerm[2];
rk_dim22_bPerm[15] += A[333]*rk_dim22_bPerm[3];
rk_dim22_bPerm[15] += A[334]*rk_dim22_bPerm[4];
rk_dim22_bPerm[15] += A[335]*rk_dim22_bPerm[5];
rk_dim22_bPerm[15] += A[336]*rk_dim22_bPerm[6];
rk_dim22_bPerm[15] += A[337]*rk_dim22_bPerm[7];
rk_dim22_bPerm[15] += A[338]*rk_dim22_bPerm[8];
rk_dim22_bPerm[15] += A[339]*rk_dim22_bPerm[9];
rk_dim22_bPerm[15] += A[340]*rk_dim22_bPerm[10];
rk_dim22_bPerm[15] += A[341]*rk_dim22_bPerm[11];
rk_dim22_bPerm[15] += A[342]*rk_dim22_bPerm[12];
rk_dim22_bPerm[15] += A[343]*rk_dim22_bPerm[13];
rk_dim22_bPerm[15] += A[344]*rk_dim22_bPerm[14];

rk_dim22_bPerm[16] += A[352]*rk_dim22_bPerm[0];
rk_dim22_bPerm[16] += A[353]*rk_dim22_bPerm[1];
rk_dim22_bPerm[16] += A[354]*rk_dim22_bPerm[2];
rk_dim22_bPerm[16] += A[355]*rk_dim22_bPerm[3];
rk_dim22_bPerm[16] += A[356]*rk_dim22_bPerm[4];
rk_dim22_bPerm[16] += A[357]*rk_dim22_bPerm[5];
rk_dim22_bPerm[16] += A[358]*rk_dim22_bPerm[6];
rk_dim22_bPerm[16] += A[359]*rk_dim22_bPerm[7];
rk_dim22_bPerm[16] += A[360]*rk_dim22_bPerm[8];
rk_dim22_bPerm[16] += A[361]*rk_dim22_bPerm[9];
rk_dim22_bPerm[16] += A[362]*rk_dim22_bPerm[10];
rk_dim22_bPerm[16] += A[363]*rk_dim22_bPerm[11];
rk_dim22_bPerm[16] += A[364]*rk_dim22_bPerm[12];
rk_dim22_bPerm[16] += A[365]*rk_dim22_bPerm[13];
rk_dim22_bPerm[16] += A[366]*rk_dim22_bPerm[14];
rk_dim22_bPerm[16] += A[367]*rk_dim22_bPerm[15];

rk_dim22_bPerm[17] += A[374]*rk_dim22_bPerm[0];
rk_dim22_bPerm[17] += A[375]*rk_dim22_bPerm[1];
rk_dim22_bPerm[17] += A[376]*rk_dim22_bPerm[2];
rk_dim22_bPerm[17] += A[377]*rk_dim22_bPerm[3];
rk_dim22_bPerm[17] += A[378]*rk_dim22_bPerm[4];
rk_dim22_bPerm[17] += A[379]*rk_dim22_bPerm[5];
rk_dim22_bPerm[17] += A[380]*rk_dim22_bPerm[6];
rk_dim22_bPerm[17] += A[381]*rk_dim22_bPerm[7];
rk_dim22_bPerm[17] += A[382]*rk_dim22_bPerm[8];
rk_dim22_bPerm[17] += A[383]*rk_dim22_bPerm[9];
rk_dim22_bPerm[17] += A[384]*rk_dim22_bPerm[10];
rk_dim22_bPerm[17] += A[385]*rk_dim22_bPerm[11];
rk_dim22_bPerm[17] += A[386]*rk_dim22_bPerm[12];
rk_dim22_bPerm[17] += A[387]*rk_dim22_bPerm[13];
rk_dim22_bPerm[17] += A[388]*rk_dim22_bPerm[14];
rk_dim22_bPerm[17] += A[389]*rk_dim22_bPerm[15];
rk_dim22_bPerm[17] += A[390]*rk_dim22_bPerm[16];

rk_dim22_bPerm[18] += A[396]*rk_dim22_bPerm[0];
rk_dim22_bPerm[18] += A[397]*rk_dim22_bPerm[1];
rk_dim22_bPerm[18] += A[398]*rk_dim22_bPerm[2];
rk_dim22_bPerm[18] += A[399]*rk_dim22_bPerm[3];
rk_dim22_bPerm[18] += A[400]*rk_dim22_bPerm[4];
rk_dim22_bPerm[18] += A[401]*rk_dim22_bPerm[5];
rk_dim22_bPerm[18] += A[402]*rk_dim22_bPerm[6];
rk_dim22_bPerm[18] += A[403]*rk_dim22_bPerm[7];
rk_dim22_bPerm[18] += A[404]*rk_dim22_bPerm[8];
rk_dim22_bPerm[18] += A[405]*rk_dim22_bPerm[9];
rk_dim22_bPerm[18] += A[406]*rk_dim22_bPerm[10];
rk_dim22_bPerm[18] += A[407]*rk_dim22_bPerm[11];
rk_dim22_bPerm[18] += A[408]*rk_dim22_bPerm[12];
rk_dim22_bPerm[18] += A[409]*rk_dim22_bPerm[13];
rk_dim22_bPerm[18] += A[410]*rk_dim22_bPerm[14];
rk_dim22_bPerm[18] += A[411]*rk_dim22_bPerm[15];
rk_dim22_bPerm[18] += A[412]*rk_dim22_bPerm[16];
rk_dim22_bPerm[18] += A[413]*rk_dim22_bPerm[17];

rk_dim22_bPerm[19] += A[418]*rk_dim22_bPerm[0];
rk_dim22_bPerm[19] += A[419]*rk_dim22_bPerm[1];
rk_dim22_bPerm[19] += A[420]*rk_dim22_bPerm[2];
rk_dim22_bPerm[19] += A[421]*rk_dim22_bPerm[3];
rk_dim22_bPerm[19] += A[422]*rk_dim22_bPerm[4];
rk_dim22_bPerm[19] += A[423]*rk_dim22_bPerm[5];
rk_dim22_bPerm[19] += A[424]*rk_dim22_bPerm[6];
rk_dim22_bPerm[19] += A[425]*rk_dim22_bPerm[7];
rk_dim22_bPerm[19] += A[426]*rk_dim22_bPerm[8];
rk_dim22_bPerm[19] += A[427]*rk_dim22_bPerm[9];
rk_dim22_bPerm[19] += A[428]*rk_dim22_bPerm[10];
rk_dim22_bPerm[19] += A[429]*rk_dim22_bPerm[11];
rk_dim22_bPerm[19] += A[430]*rk_dim22_bPerm[12];
rk_dim22_bPerm[19] += A[431]*rk_dim22_bPerm[13];
rk_dim22_bPerm[19] += A[432]*rk_dim22_bPerm[14];
rk_dim22_bPerm[19] += A[433]*rk_dim22_bPerm[15];
rk_dim22_bPerm[19] += A[434]*rk_dim22_bPerm[16];
rk_dim22_bPerm[19] += A[435]*rk_dim22_bPerm[17];
rk_dim22_bPerm[19] += A[436]*rk_dim22_bPerm[18];

rk_dim22_bPerm[20] += A[440]*rk_dim22_bPerm[0];
rk_dim22_bPerm[20] += A[441]*rk_dim22_bPerm[1];
rk_dim22_bPerm[20] += A[442]*rk_dim22_bPerm[2];
rk_dim22_bPerm[20] += A[443]*rk_dim22_bPerm[3];
rk_dim22_bPerm[20] += A[444]*rk_dim22_bPerm[4];
rk_dim22_bPerm[20] += A[445]*rk_dim22_bPerm[5];
rk_dim22_bPerm[20] += A[446]*rk_dim22_bPerm[6];
rk_dim22_bPerm[20] += A[447]*rk_dim22_bPerm[7];
rk_dim22_bPerm[20] += A[448]*rk_dim22_bPerm[8];
rk_dim22_bPerm[20] += A[449]*rk_dim22_bPerm[9];
rk_dim22_bPerm[20] += A[450]*rk_dim22_bPerm[10];
rk_dim22_bPerm[20] += A[451]*rk_dim22_bPerm[11];
rk_dim22_bPerm[20] += A[452]*rk_dim22_bPerm[12];
rk_dim22_bPerm[20] += A[453]*rk_dim22_bPerm[13];
rk_dim22_bPerm[20] += A[454]*rk_dim22_bPerm[14];
rk_dim22_bPerm[20] += A[455]*rk_dim22_bPerm[15];
rk_dim22_bPerm[20] += A[456]*rk_dim22_bPerm[16];
rk_dim22_bPerm[20] += A[457]*rk_dim22_bPerm[17];
rk_dim22_bPerm[20] += A[458]*rk_dim22_bPerm[18];
rk_dim22_bPerm[20] += A[459]*rk_dim22_bPerm[19];

rk_dim22_bPerm[21] += A[462]*rk_dim22_bPerm[0];
rk_dim22_bPerm[21] += A[463]*rk_dim22_bPerm[1];
rk_dim22_bPerm[21] += A[464]*rk_dim22_bPerm[2];
rk_dim22_bPerm[21] += A[465]*rk_dim22_bPerm[3];
rk_dim22_bPerm[21] += A[466]*rk_dim22_bPerm[4];
rk_dim22_bPerm[21] += A[467]*rk_dim22_bPerm[5];
rk_dim22_bPerm[21] += A[468]*rk_dim22_bPerm[6];
rk_dim22_bPerm[21] += A[469]*rk_dim22_bPerm[7];
rk_dim22_bPerm[21] += A[470]*rk_dim22_bPerm[8];
rk_dim22_bPerm[21] += A[471]*rk_dim22_bPerm[9];
rk_dim22_bPerm[21] += A[472]*rk_dim22_bPerm[10];
rk_dim22_bPerm[21] += A[473]*rk_dim22_bPerm[11];
rk_dim22_bPerm[21] += A[474]*rk_dim22_bPerm[12];
rk_dim22_bPerm[21] += A[475]*rk_dim22_bPerm[13];
rk_dim22_bPerm[21] += A[476]*rk_dim22_bPerm[14];
rk_dim22_bPerm[21] += A[477]*rk_dim22_bPerm[15];
rk_dim22_bPerm[21] += A[478]*rk_dim22_bPerm[16];
rk_dim22_bPerm[21] += A[479]*rk_dim22_bPerm[17];
rk_dim22_bPerm[21] += A[480]*rk_dim22_bPerm[18];
rk_dim22_bPerm[21] += A[481]*rk_dim22_bPerm[19];
rk_dim22_bPerm[21] += A[482]*rk_dim22_bPerm[20];


acado_solve_dim22_triangular( A, rk_dim22_bPerm );
b[0] = rk_dim22_bPerm[0];
b[1] = rk_dim22_bPerm[1];
b[2] = rk_dim22_bPerm[2];
b[3] = rk_dim22_bPerm[3];
b[4] = rk_dim22_bPerm[4];
b[5] = rk_dim22_bPerm[5];
b[6] = rk_dim22_bPerm[6];
b[7] = rk_dim22_bPerm[7];
b[8] = rk_dim22_bPerm[8];
b[9] = rk_dim22_bPerm[9];
b[10] = rk_dim22_bPerm[10];
b[11] = rk_dim22_bPerm[11];
b[12] = rk_dim22_bPerm[12];
b[13] = rk_dim22_bPerm[13];
b[14] = rk_dim22_bPerm[14];
b[15] = rk_dim22_bPerm[15];
b[16] = rk_dim22_bPerm[16];
b[17] = rk_dim22_bPerm[17];
b[18] = rk_dim22_bPerm[18];
b[19] = rk_dim22_bPerm[19];
b[20] = rk_dim22_bPerm[20];
b[21] = rk_dim22_bPerm[21];
}



/** Matrix of size: 2 x 2 (row major format) */
static const real_t acado_Ah_mat[ 4 ] = 
{ 2.5000000000000001e-02, 5.3867513459481292e-02, 
-3.8675134594812867e-03, 2.5000000000000001e-02 };


/* Fixed step size:0.1 */
int acado_integrate( real_t* const rk_eta, int resetIntegrator )
{
int error;

int i;
int j;
int k;
int run;
int run1;
int tmp_index1;
int tmp_index2;

real_t det;

rk_ttt = 0.0000000000000000e+00;
rk_xxx[11] = rk_eta[198];
rk_xxx[12] = rk_eta[199];
rk_xxx[13] = rk_eta[200];
rk_xxx[14] = rk_eta[201];
rk_xxx[15] = rk_eta[202];
rk_xxx[16] = rk_eta[203];
rk_xxx[17] = rk_eta[204];
rk_xxx[18] = rk_eta[205];
rk_xxx[19] = rk_eta[206];
rk_xxx[20] = rk_eta[207];
rk_xxx[21] = rk_eta[208];
rk_xxx[22] = rk_eta[209];
rk_xxx[23] = rk_eta[210];
rk_xxx[24] = rk_eta[211];
rk_xxx[25] = rk_eta[212];
rk_xxx[26] = rk_eta[213];
rk_xxx[27] = rk_eta[214];
rk_xxx[28] = rk_eta[215];
rk_xxx[29] = rk_eta[216];

for (run = 0; run < 1; ++run)
{
if( resetIntegrator ) {
for (i = 0; i < 1; ++i)
{
for (run1 = 0; run1 < 2; ++run1)
{
for (j = 0; j < 11; ++j)
{
rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
rk_xxx[j] += + acado_Ah_mat[run1 * 2]*rk_kkk[tmp_index1 * 2];
rk_xxx[j] += + acado_Ah_mat[run1 * 2 + 1]*rk_kkk[tmp_index1 * 2 + 1];
}
acado_diffs( rk_xxx, &(rk_diffsTemp2[ run1 * 187 ]) );
for (j = 0; j < 11; ++j)
{
tmp_index1 = (run1 * 11) + (j);
rk_A[tmp_index1 * 22] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 187) + (j * 17)];
rk_A[tmp_index1 * 22 + 1] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 187) + (j * 17 + 1)];
rk_A[tmp_index1 * 22 + 2] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 187) + (j * 17 + 2)];
rk_A[tmp_index1 * 22 + 3] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 187) + (j * 17 + 3)];
rk_A[tmp_index1 * 22 + 4] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 187) + (j * 17 + 4)];
rk_A[tmp_index1 * 22 + 5] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 187) + (j * 17 + 5)];
rk_A[tmp_index1 * 22 + 6] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 187) + (j * 17 + 6)];
rk_A[tmp_index1 * 22 + 7] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 187) + (j * 17 + 7)];
rk_A[tmp_index1 * 22 + 8] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 187) + (j * 17 + 8)];
rk_A[tmp_index1 * 22 + 9] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 187) + (j * 17 + 9)];
rk_A[tmp_index1 * 22 + 10] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 187) + (j * 17 + 10)];
if( 0 == run1 ) rk_A[(tmp_index1 * 22) + (j)] -= 1.0000000000000000e+00;
rk_A[tmp_index1 * 22 + 11] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 187) + (j * 17)];
rk_A[tmp_index1 * 22 + 12] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 187) + (j * 17 + 1)];
rk_A[tmp_index1 * 22 + 13] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 187) + (j * 17 + 2)];
rk_A[tmp_index1 * 22 + 14] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 187) + (j * 17 + 3)];
rk_A[tmp_index1 * 22 + 15] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 187) + (j * 17 + 4)];
rk_A[tmp_index1 * 22 + 16] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 187) + (j * 17 + 5)];
rk_A[tmp_index1 * 22 + 17] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 187) + (j * 17 + 6)];
rk_A[tmp_index1 * 22 + 18] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 187) + (j * 17 + 7)];
rk_A[tmp_index1 * 22 + 19] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 187) + (j * 17 + 8)];
rk_A[tmp_index1 * 22 + 20] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 187) + (j * 17 + 9)];
rk_A[tmp_index1 * 22 + 21] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 187) + (j * 17 + 10)];
if( 1 == run1 ) rk_A[(tmp_index1 * 22) + (j + 11)] -= 1.0000000000000000e+00;
}
acado_rhs( rk_xxx, rk_rhsTemp );
rk_b[run1 * 11] = rk_kkk[run1] - rk_rhsTemp[0];
rk_b[run1 * 11 + 1] = rk_kkk[run1 + 2] - rk_rhsTemp[1];
rk_b[run1 * 11 + 2] = rk_kkk[run1 + 4] - rk_rhsTemp[2];
rk_b[run1 * 11 + 3] = rk_kkk[run1 + 6] - rk_rhsTemp[3];
rk_b[run1 * 11 + 4] = rk_kkk[run1 + 8] - rk_rhsTemp[4];
rk_b[run1 * 11 + 5] = rk_kkk[run1 + 10] - rk_rhsTemp[5];
rk_b[run1 * 11 + 6] = rk_kkk[run1 + 12] - rk_rhsTemp[6];
rk_b[run1 * 11 + 7] = rk_kkk[run1 + 14] - rk_rhsTemp[7];
rk_b[run1 * 11 + 8] = rk_kkk[run1 + 16] - rk_rhsTemp[8];
rk_b[run1 * 11 + 9] = rk_kkk[run1 + 18] - rk_rhsTemp[9];
rk_b[run1 * 11 + 10] = rk_kkk[run1 + 20] - rk_rhsTemp[10];
}
det = acado_solve_dim22_system( rk_A, rk_b, rk_dim22_perm );
for (j = 0; j < 2; ++j)
{
rk_kkk[j] += rk_b[j * 11];
rk_kkk[j + 2] += rk_b[j * 11 + 1];
rk_kkk[j + 4] += rk_b[j * 11 + 2];
rk_kkk[j + 6] += rk_b[j * 11 + 3];
rk_kkk[j + 8] += rk_b[j * 11 + 4];
rk_kkk[j + 10] += rk_b[j * 11 + 5];
rk_kkk[j + 12] += rk_b[j * 11 + 6];
rk_kkk[j + 14] += rk_b[j * 11 + 7];
rk_kkk[j + 16] += rk_b[j * 11 + 8];
rk_kkk[j + 18] += rk_b[j * 11 + 9];
rk_kkk[j + 20] += rk_b[j * 11 + 10];
}
}
}
for (i = 0; i < 5; ++i)
{
for (run1 = 0; run1 < 2; ++run1)
{
for (j = 0; j < 11; ++j)
{
rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
rk_xxx[j] += + acado_Ah_mat[run1 * 2]*rk_kkk[tmp_index1 * 2];
rk_xxx[j] += + acado_Ah_mat[run1 * 2 + 1]*rk_kkk[tmp_index1 * 2 + 1];
}
acado_rhs( rk_xxx, rk_rhsTemp );
rk_b[run1 * 11] = rk_kkk[run1] - rk_rhsTemp[0];
rk_b[run1 * 11 + 1] = rk_kkk[run1 + 2] - rk_rhsTemp[1];
rk_b[run1 * 11 + 2] = rk_kkk[run1 + 4] - rk_rhsTemp[2];
rk_b[run1 * 11 + 3] = rk_kkk[run1 + 6] - rk_rhsTemp[3];
rk_b[run1 * 11 + 4] = rk_kkk[run1 + 8] - rk_rhsTemp[4];
rk_b[run1 * 11 + 5] = rk_kkk[run1 + 10] - rk_rhsTemp[5];
rk_b[run1 * 11 + 6] = rk_kkk[run1 + 12] - rk_rhsTemp[6];
rk_b[run1 * 11 + 7] = rk_kkk[run1 + 14] - rk_rhsTemp[7];
rk_b[run1 * 11 + 8] = rk_kkk[run1 + 16] - rk_rhsTemp[8];
rk_b[run1 * 11 + 9] = rk_kkk[run1 + 18] - rk_rhsTemp[9];
rk_b[run1 * 11 + 10] = rk_kkk[run1 + 20] - rk_rhsTemp[10];
}
acado_solve_dim22_system_reuse( rk_A, rk_b, rk_dim22_perm );
for (j = 0; j < 2; ++j)
{
rk_kkk[j] += rk_b[j * 11];
rk_kkk[j + 2] += rk_b[j * 11 + 1];
rk_kkk[j + 4] += rk_b[j * 11 + 2];
rk_kkk[j + 6] += rk_b[j * 11 + 3];
rk_kkk[j + 8] += rk_b[j * 11 + 4];
rk_kkk[j + 10] += rk_b[j * 11 + 5];
rk_kkk[j + 12] += rk_b[j * 11 + 6];
rk_kkk[j + 14] += rk_b[j * 11 + 7];
rk_kkk[j + 16] += rk_b[j * 11 + 8];
rk_kkk[j + 18] += rk_b[j * 11 + 9];
rk_kkk[j + 20] += rk_b[j * 11 + 10];
}
}
for (run1 = 0; run1 < 2; ++run1)
{
for (j = 0; j < 11; ++j)
{
rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
rk_xxx[j] += + acado_Ah_mat[run1 * 2]*rk_kkk[tmp_index1 * 2];
rk_xxx[j] += + acado_Ah_mat[run1 * 2 + 1]*rk_kkk[tmp_index1 * 2 + 1];
}
acado_diffs( rk_xxx, &(rk_diffsTemp2[ run1 * 187 ]) );
for (j = 0; j < 11; ++j)
{
tmp_index1 = (run1 * 11) + (j);
rk_A[tmp_index1 * 22] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 187) + (j * 17)];
rk_A[tmp_index1 * 22 + 1] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 187) + (j * 17 + 1)];
rk_A[tmp_index1 * 22 + 2] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 187) + (j * 17 + 2)];
rk_A[tmp_index1 * 22 + 3] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 187) + (j * 17 + 3)];
rk_A[tmp_index1 * 22 + 4] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 187) + (j * 17 + 4)];
rk_A[tmp_index1 * 22 + 5] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 187) + (j * 17 + 5)];
rk_A[tmp_index1 * 22 + 6] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 187) + (j * 17 + 6)];
rk_A[tmp_index1 * 22 + 7] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 187) + (j * 17 + 7)];
rk_A[tmp_index1 * 22 + 8] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 187) + (j * 17 + 8)];
rk_A[tmp_index1 * 22 + 9] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 187) + (j * 17 + 9)];
rk_A[tmp_index1 * 22 + 10] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 187) + (j * 17 + 10)];
if( 0 == run1 ) rk_A[(tmp_index1 * 22) + (j)] -= 1.0000000000000000e+00;
rk_A[tmp_index1 * 22 + 11] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 187) + (j * 17)];
rk_A[tmp_index1 * 22 + 12] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 187) + (j * 17 + 1)];
rk_A[tmp_index1 * 22 + 13] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 187) + (j * 17 + 2)];
rk_A[tmp_index1 * 22 + 14] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 187) + (j * 17 + 3)];
rk_A[tmp_index1 * 22 + 15] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 187) + (j * 17 + 4)];
rk_A[tmp_index1 * 22 + 16] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 187) + (j * 17 + 5)];
rk_A[tmp_index1 * 22 + 17] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 187) + (j * 17 + 6)];
rk_A[tmp_index1 * 22 + 18] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 187) + (j * 17 + 7)];
rk_A[tmp_index1 * 22 + 19] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 187) + (j * 17 + 8)];
rk_A[tmp_index1 * 22 + 20] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 187) + (j * 17 + 9)];
rk_A[tmp_index1 * 22 + 21] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 187) + (j * 17 + 10)];
if( 1 == run1 ) rk_A[(tmp_index1 * 22) + (j + 11)] -= 1.0000000000000000e+00;
}
}
for (run1 = 0; run1 < 11; ++run1)
{
for (i = 0; i < 2; ++i)
{
rk_b[i * 11] = - rk_diffsTemp2[(i * 187) + (run1)];
rk_b[i * 11 + 1] = - rk_diffsTemp2[(i * 187) + (run1 + 17)];
rk_b[i * 11 + 2] = - rk_diffsTemp2[(i * 187) + (run1 + 34)];
rk_b[i * 11 + 3] = - rk_diffsTemp2[(i * 187) + (run1 + 51)];
rk_b[i * 11 + 4] = - rk_diffsTemp2[(i * 187) + (run1 + 68)];
rk_b[i * 11 + 5] = - rk_diffsTemp2[(i * 187) + (run1 + 85)];
rk_b[i * 11 + 6] = - rk_diffsTemp2[(i * 187) + (run1 + 102)];
rk_b[i * 11 + 7] = - rk_diffsTemp2[(i * 187) + (run1 + 119)];
rk_b[i * 11 + 8] = - rk_diffsTemp2[(i * 187) + (run1 + 136)];
rk_b[i * 11 + 9] = - rk_diffsTemp2[(i * 187) + (run1 + 153)];
rk_b[i * 11 + 10] = - rk_diffsTemp2[(i * 187) + (run1 + 170)];
}
if( 0 == run1 ) {
det = acado_solve_dim22_system( rk_A, rk_b, rk_dim22_perm );
}
 else {
acado_solve_dim22_system_reuse( rk_A, rk_b, rk_dim22_perm );
}
for (i = 0; i < 2; ++i)
{
rk_diffK[i] = rk_b[i * 11];
rk_diffK[i + 2] = rk_b[i * 11 + 1];
rk_diffK[i + 4] = rk_b[i * 11 + 2];
rk_diffK[i + 6] = rk_b[i * 11 + 3];
rk_diffK[i + 8] = rk_b[i * 11 + 4];
rk_diffK[i + 10] = rk_b[i * 11 + 5];
rk_diffK[i + 12] = rk_b[i * 11 + 6];
rk_diffK[i + 14] = rk_b[i * 11 + 7];
rk_diffK[i + 16] = rk_b[i * 11 + 8];
rk_diffK[i + 18] = rk_b[i * 11 + 9];
rk_diffK[i + 20] = rk_b[i * 11 + 10];
}
for (i = 0; i < 11; ++i)
{
rk_diffsNew2[(i * 17) + (run1)] = (i == run1-0);
rk_diffsNew2[(i * 17) + (run1)] += + rk_diffK[i * 2]*(real_t)5.0000000000000003e-02 + rk_diffK[i * 2 + 1]*(real_t)5.0000000000000003e-02;
}
}
for (run1 = 0; run1 < 6; ++run1)
{
for (i = 0; i < 2; ++i)
{
for (j = 0; j < 11; ++j)
{
tmp_index1 = (i * 11) + (j);
tmp_index2 = (run1) + (j * 17);
rk_b[tmp_index1] = - rk_diffsTemp2[(i * 187) + (tmp_index2 + 11)];
}
}
acado_solve_dim22_system_reuse( rk_A, rk_b, rk_dim22_perm );
for (i = 0; i < 2; ++i)
{
rk_diffK[i] = rk_b[i * 11];
rk_diffK[i + 2] = rk_b[i * 11 + 1];
rk_diffK[i + 4] = rk_b[i * 11 + 2];
rk_diffK[i + 6] = rk_b[i * 11 + 3];
rk_diffK[i + 8] = rk_b[i * 11 + 4];
rk_diffK[i + 10] = rk_b[i * 11 + 5];
rk_diffK[i + 12] = rk_b[i * 11 + 6];
rk_diffK[i + 14] = rk_b[i * 11 + 7];
rk_diffK[i + 16] = rk_b[i * 11 + 8];
rk_diffK[i + 18] = rk_b[i * 11 + 9];
rk_diffK[i + 20] = rk_b[i * 11 + 10];
}
for (i = 0; i < 11; ++i)
{
rk_diffsNew2[(i * 17) + (run1 + 11)] = + rk_diffK[i * 2]*(real_t)5.0000000000000003e-02 + rk_diffK[i * 2 + 1]*(real_t)5.0000000000000003e-02;
}
}
rk_eta[0] += + rk_kkk[0]*(real_t)5.0000000000000003e-02 + rk_kkk[1]*(real_t)5.0000000000000003e-02;
rk_eta[1] += + rk_kkk[2]*(real_t)5.0000000000000003e-02 + rk_kkk[3]*(real_t)5.0000000000000003e-02;
rk_eta[2] += + rk_kkk[4]*(real_t)5.0000000000000003e-02 + rk_kkk[5]*(real_t)5.0000000000000003e-02;
rk_eta[3] += + rk_kkk[6]*(real_t)5.0000000000000003e-02 + rk_kkk[7]*(real_t)5.0000000000000003e-02;
rk_eta[4] += + rk_kkk[8]*(real_t)5.0000000000000003e-02 + rk_kkk[9]*(real_t)5.0000000000000003e-02;
rk_eta[5] += + rk_kkk[10]*(real_t)5.0000000000000003e-02 + rk_kkk[11]*(real_t)5.0000000000000003e-02;
rk_eta[6] += + rk_kkk[12]*(real_t)5.0000000000000003e-02 + rk_kkk[13]*(real_t)5.0000000000000003e-02;
rk_eta[7] += + rk_kkk[14]*(real_t)5.0000000000000003e-02 + rk_kkk[15]*(real_t)5.0000000000000003e-02;
rk_eta[8] += + rk_kkk[16]*(real_t)5.0000000000000003e-02 + rk_kkk[17]*(real_t)5.0000000000000003e-02;
rk_eta[9] += + rk_kkk[18]*(real_t)5.0000000000000003e-02 + rk_kkk[19]*(real_t)5.0000000000000003e-02;
rk_eta[10] += + rk_kkk[20]*(real_t)5.0000000000000003e-02 + rk_kkk[21]*(real_t)5.0000000000000003e-02;
for (i = 0; i < 11; ++i)
{
for (j = 0; j < 11; ++j)
{
tmp_index2 = (j) + (i * 11);
rk_eta[tmp_index2 + 11] = rk_diffsNew2[(i * 17) + (j)];
}
for (j = 0; j < 6; ++j)
{
tmp_index2 = (j) + (i * 6);
rk_eta[tmp_index2 + 132] = rk_diffsNew2[(i * 17) + (j + 11)];
}
}
resetIntegrator = 0;
rk_ttt += 1.0000000000000000e+00;
}
for (i = 0; i < 11; ++i)
{
}
if( det < 1e-12 ) {
error = 2;
} else if( det < 1e-6 ) {
error = 1;
} else {
error = 0;
}
return error;
}



