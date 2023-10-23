/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2023, ITU/ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/** \file     Contexts.cpp
 *  \brief    Classes providing probability descriptions and contexts (also contains context initialization values)
 */

#include "Contexts.h"

#include <algorithm>
#include <cstring>
#include <limits>

const uint8_t ProbModelTables::m_RenormTable_32[32] =
{
  6,  5,  4,  4,
  3,  3,  3,  3,
  2,  2,  2,  2,
  2,  2,  2,  2,
  1,  1,  1,  1,
  1,  1,  1,  1,
  1,  1,  1,  1,
  1,  1,  1,  1
};
#if EC_HIGH_PRECISION
const BinFracBits ProbModelTables::m_binFracBits[512] = {
	{ { 46,327680 } },
	{ { 139,275744 } },
	{ { 231,251595 } },
	{ { 324,235689 } },
	{ { 417,223808 } },
	{ { 511,214321 } },
	{ { 604,206424 } },
	{ { 698,199659 } },
	{ { 791,193742 } },
	{ { 885,188484 } },
	{ { 980,183753 } },
	{ { 1074,179452 } },
	{ { 1168,175510 } },
	{ { 1263,171872 } },
	{ { 1358,168494 } },
	{ { 1453,165341 } },
	{ { 1549,162385 } },
	{ { 1644,159604 } },
	{ { 1740,156977 } },
	{ { 1836,154488 } },
	{ { 1932,152124 } },
	{ { 2028,149872 } },
	{ { 2125,147723 } },
	{ { 2221,145667 } },
	{ { 2318,143697 } },
	{ { 2415,141806 } },
	{ { 2512,139988 } },
	{ { 2610,138236 } },
	{ { 2708,136548 } },
	{ { 2805,134918 } },
	{ { 2904,133342 } },
	{ { 3002,131816 } },
	{ { 3100,130339 } },
	{ { 3199,128906 } },
	{ { 3298,127516 } },
	{ { 3397,126165 } },
	{ { 3496,124852 } },
	{ { 3596,123574 } },
	{ { 3696,122330 } },
	{ { 3796,121118 } },
	{ { 3896,119936 } },
	{ { 3996,118783 } },
	{ { 4097,117657 } },
	{ { 4197,116558 } },
	{ { 4298,115483 } },
	{ { 4400,114433 } },
	{ { 4501,113405 } },
	{ { 4603,112399 } },
	{ { 4705,111414 } },
	{ { 4807,110449 } },
	{ { 4909,109504 } },
	{ { 5012,108577 } },
	{ { 5114,107668 } },
	{ { 5217,106776 } },
	{ { 5321,105900 } },
	{ { 5424,105041 } },
	{ { 5528,104196 } },
	{ { 5632,103367 } },
	{ { 5736,102552 } },
	{ { 5840,101751 } },
	{ { 5945,100963 } },
	{ { 6050,100188 } },
	{ { 6155, 99425 } },
	{ { 6260, 98675 } },
	{ { 6365, 97936 } },
	{ { 6471, 97209 } },
	{ { 6577, 96493 } },
	{ { 6683, 95787 } },
	{ { 6790, 95092 } },
	{ { 6897, 94407 } },
	{ { 7004, 93731 } },
	{ { 7111, 93065 } },
	{ { 7218, 92409 } },
	{ { 7326, 91761 } },
	{ { 7434, 91122 } },
	{ { 7542, 90492 } },
	{ { 7650, 89870 } },
	{ { 7759, 89256 } },
	{ { 7868, 88650 } },
	{ { 7977, 88051 } },
	{ { 8087, 87461 } },
	{ { 8196, 86877 } },
	{ { 8306, 86300 } },
	{ { 8416, 85731 } },
	{ { 8527, 85168 } },
	{ { 8638, 84612 } },
	{ { 8749, 84062 } },
	{ { 8860, 83519 } },
	{ { 8971, 82982 } },
	{ { 9083, 82450 } },
	{ { 9195, 81925 } },
	{ { 9307, 81406 } },
	{ { 9420, 80892 } },
	{ { 9533, 80383 } },
	{ { 9646, 79880 } },
	{ { 9759, 79383 } },
	{ { 9873, 78890 } },
	{ { 9987, 78403 } },
	{ { 10101, 77921 } },
	{ { 10215, 77443 } },
	{ { 10330, 76970 } },
	{ { 10445, 76502 } },
	{ { 10560, 76039 } },
	{ { 10676, 75580 } },
	{ { 10792, 75125 } },
	{ { 10908, 74675 } },
	{ { 11025, 74229 } },
	{ { 11141, 73787 } },
	{ { 11258, 73349 } },
	{ { 11376, 72916 } },
	{ { 11493, 72486 } },
	{ { 11611, 72060 } },
	{ { 11729, 71638 } },
	{ { 11848, 71220 } },
	{ { 11967, 70805 } },
	{ { 12086, 70394 } },
	{ { 12205, 69986 } },
	{ { 12325, 69582 } },
	{ { 12445, 69182 } },
	{ { 12565, 68784 } },
	{ { 12686, 68390 } },
	{ { 12806, 68000 } },
	{ { 12928, 67612 } },
	{ { 13049, 67228 } },
	{ { 13171, 66847 } },
	{ { 13293, 66468 } },
	{ { 13416, 66093 } },
	{ { 13538, 65721 } },
	{ { 13662, 65352 } },
	{ { 13785, 64985 } },
	{ { 13909, 64622 } },
	{ { 14033, 64261 } },
	{ { 14157, 63903 } },
	{ { 14282, 63547 } },
	{ { 14407, 63194 } },
	{ { 14532, 62844 } },
	{ { 14658, 62497 } },
	{ { 14784, 62151 } },
	{ { 14911, 61809 } },
	{ { 15037, 61469 } },
	{ { 15164, 61131 } },
	{ { 15292, 60796 } },
	{ { 15420, 60463 } },
	{ { 15548, 60132 } },
	{ { 15676, 59804 } },
	{ { 15805, 59478 } },
	{ { 15934, 59154 } },
	{ { 16064, 58833 } },
	{ { 16194, 58513 } },
	{ { 16324, 58196 } },
	{ { 16454, 57881 } },
	{ { 16585, 57568 } },
	{ { 16717, 57257 } },
	{ { 16848, 56948 } },
	{ { 16980, 56641 } },
	{ { 17113, 56336 } },
	{ { 17246, 56033 } },
	{ { 17379, 55732 } },
	{ { 17512, 55432 } },
	{ { 17646, 55135 } },
	{ { 17781, 54840 } },
	{ { 17915, 54546 } },
	{ { 18050, 54254 } },
	{ { 18186, 53964 } },
	{ { 18322, 53676 } },
	{ { 18458, 53389 } },
	{ { 18594, 53105 } },
	{ { 18731, 52821 } },
	{ { 18869, 52540 } },
	{ { 19007, 52260 } },
	{ { 19145, 51982 } },
	{ { 19284, 51706 } },
	{ { 19423, 51431 } },
	{ { 19562, 51158 } },
	{ { 19702, 50886 } },
	{ { 19842, 50616 } },
	{ { 19983, 50347 } },
	{ { 20124, 50080 } },
	{ { 20266, 49815 } },
	{ { 20408, 49550 } },
	{ { 20550, 49288 } },
	{ { 20693, 49027 } },
	{ { 20836, 48767 } },
	{ { 20980, 48509 } },
	{ { 21124, 48252 } },
	{ { 21268, 47996 } },
	{ { 21413, 47742 } },
	{ { 21559, 47489 } },
	{ { 21705, 47238 } },
	{ { 21851, 46988 } },
	{ { 21998, 46739 } },
	{ { 22145, 46491 } },
	{ { 22293, 46245 } },
	{ { 22441, 46000 } },
	{ { 22590, 45756 } },
	{ { 22739, 45514 } },
	{ { 22889, 45273 } },
	{ { 23039, 45033 } },
	{ { 23189, 44794 } },
	{ { 23340, 44556 } },
	{ { 23492, 44320 } },
	{ { 23644, 44085 } },
	{ { 23796, 43851 } },
	{ { 23949, 43618 } },
	{ { 24103, 43386 } },
	{ { 24257, 43156 } },
	{ { 24411, 42926 } },
	{ { 24566, 42698 } },
	{ { 24722, 42470 } },
	{ { 24878, 42244 } },
	{ { 25034, 42019 } },
	{ { 25191, 41795 } },
	{ { 25349, 41572 } },
	{ { 25507, 41350 } },
	{ { 25666, 41129 } },
	{ { 25825, 40909 } },
	{ { 25985, 40691 } },
	{ { 26145, 40473 } },
	{ { 26306, 40256 } },
	{ { 26467, 40040 } },
	{ { 26629, 39825 } },
	{ { 26791, 39611 } },
	{ { 26954, 39398 } },
	{ { 27118, 39186 } },
	{ { 27282, 38975 } },
	{ { 27447, 38765 } },
	{ { 27612, 38556 } },
	{ { 27778, 38348 } },
	{ { 27944, 38140 } },
	{ { 28111, 37934 } },
	{ { 28279, 37728 } },
	{ { 28447, 37524 } },
	{ { 28616, 37320 } },
	{ { 28786, 37117 } },
	{ { 28956, 36915 } },
	{ { 29126, 36714 } },
	{ { 29298, 36514 } },
	{ { 29469, 36314 } },
	{ { 29642, 36115 } },
	{ { 29815, 35918 } },
	{ { 29989, 35721 } },
	{ { 30163, 35524 } },
	{ { 30339, 35329 } },
	{ { 30514, 35135 } },
	{ { 30691, 34941 } },
	{ { 30868, 34748 } },
	{ { 31045, 34556 } },
	{ { 31224, 34364 } },
	{ { 31403, 34174 } },
	{ { 31583, 33984 } },
	{ { 31763, 33795 } },
	{ { 31944, 33606 } },
	{ { 32126, 33419 } },
	{ { 32309, 33232 } },
	{ { 32492, 33046 } },
	{ { 32676, 32860 } },
	{ { 32860, 32676 } },
	{ { 33046, 32492 } },
	{ { 33232, 32309 } },
	{ { 33419, 32126 } },
	{ { 33606, 31944 } },
	{ { 33795, 31763 } },
	{ { 33984, 31583 } },
	{ { 34174, 31403 } },
	{ { 34364, 31224 } },
	{ { 34556, 31045 } },
	{ { 34748, 30868 } },
	{ { 34941, 30691 } },
	{ { 35135, 30514 } },
	{ { 35329, 30339 } },
	{ { 35524, 30163 } },
	{ { 35721, 29989 } },
	{ { 35918, 29815 } },
	{ { 36115, 29642 } },
	{ { 36314, 29469 } },
	{ { 36514, 29298 } },
	{ { 36714, 29126 } },
	{ { 36915, 28956 } },
	{ { 37117, 28786 } },
	{ { 37320, 28616 } },
	{ { 37524, 28447 } },
	{ { 37728, 28279 } },
	{ { 37934, 28111 } },
	{ { 38140, 27944 } },
	{ { 38348, 27778 } },
	{ { 38556, 27612 } },
	{ { 38765, 27447 } },
	{ { 38975, 27282 } },
	{ { 39186, 27118 } },
	{ { 39398, 26954 } },
	{ { 39611, 26791 } },
	{ { 39825, 26629 } },
	{ { 40040, 26467 } },
	{ { 40256, 26306 } },
	{ { 40473, 26145 } },
	{ { 40691, 25985 } },
	{ { 40909, 25825 } },
	{ { 41129, 25666 } },
	{ { 41350, 25507 } },
	{ { 41572, 25349 } },
	{ { 41795, 25191 } },
	{ { 42019, 25034 } },
	{ { 42244, 24878 } },
	{ { 42470, 24722 } },
	{ { 42698, 24566 } },
	{ { 42926, 24411 } },
	{ { 43156, 24257 } },
	{ { 43386, 24103 } },
	{ { 43618, 23949 } },
	{ { 43851, 23796 } },
	{ { 44085, 23644 } },
	{ { 44320, 23492 } },
	{ { 44556, 23340 } },
	{ { 44794, 23189 } },
	{ { 45033, 23039 } },
	{ { 45273, 22889 } },
	{ { 45514, 22739 } },
	{ { 45756, 22590 } },
	{ { 46000, 22441 } },
	{ { 46245, 22293 } },
	{ { 46491, 22145 } },
	{ { 46739, 21998 } },
	{ { 46988, 21851 } },
	{ { 47238, 21705 } },
	{ { 47489, 21559 } },
	{ { 47742, 21413 } },
	{ { 47996, 21268 } },
	{ { 48252, 21124 } },
	{ { 48509, 20980 } },
	{ { 48767, 20836 } },
	{ { 49027, 20693 } },
	{ { 49288, 20550 } },
	{ { 49550, 20408 } },
	{ { 49815, 20266 } },
	{ { 50080, 20124 } },
	{ { 50347, 19983 } },
	{ { 50616, 19842 } },
	{ { 50886, 19702 } },
	{ { 51158, 19562 } },
	{ { 51431, 19423 } },
	{ { 51706, 19284 } },
	{ { 51982, 19145 } },
	{ { 52260, 19007 } },
	{ { 52540, 18869 } },
	{ { 52821, 18731 } },
	{ { 53105, 18594 } },
	{ { 53389, 18458 } },
	{ { 53676, 18322 } },
	{ { 53964, 18186 } },
	{ { 54254, 18050 } },
	{ { 54546, 17915 } },
	{ { 54840, 17781 } },
	{ { 55135, 17646 } },
	{ { 55432, 17512 } },
	{ { 55732, 17379 } },
	{ { 56033, 17246 } },
	{ { 56336, 17113 } },
	{ { 56641, 16980 } },
	{ { 56948, 16848 } },
	{ { 57257, 16717 } },
	{ { 57568, 16585 } },
	{ { 57881, 16454 } },
	{ { 58196, 16324 } },
	{ { 58513, 16194 } },
	{ { 58833, 16064 } },
	{ { 59154, 15934 } },
	{ { 59478, 15805 } },
	{ { 59804, 15676 } },
	{ { 60132, 15548 } },
	{ { 60463, 15420 } },
	{ { 60796, 15292 } },
	{ { 61131, 15164 } },
	{ { 61469, 15037 } },
	{ { 61809, 14911 } },
	{ { 62151, 14784 } },
	{ { 62497, 14658 } },
	{ { 62844, 14532 } },
	{ { 63194, 14407 } },
	{ { 63547, 14282 } },
	{ { 63903, 14157 } },
	{ { 64261, 14033 } },
	{ { 64622, 13909 } },
	{ { 64985, 13785 } },
	{ { 65352, 13662 } },
	{ { 65721, 13538 } },
	{ { 66093, 13416 } },
	{ { 66468, 13293 } },
	{ { 66847, 13171 } },
	{ { 67228, 13049 } },
	{ { 67612, 12928 } },
	{ { 68000, 12806 } },
	{ { 68390, 12686 } },
	{ { 68784, 12565 } },
	{ { 69182, 12445 } },
	{ { 69582, 12325 } },
	{ { 69986, 12205 } },
	{ { 70394, 12086 } },
	{ { 70805, 11967 } },
	{ { 71220, 11848 } },
	{ { 71638, 11729 } },
	{ { 72060, 11611 } },
	{ { 72486, 11493 } },
	{ { 72916, 11376 } },
	{ { 73349, 11258 } },
	{ { 73787, 11141 } },
	{ { 74229, 11025 } },
	{ { 74675, 10908 } },
	{ { 75125, 10792 } },
	{ { 75580, 10676 } },
	{ { 76039, 10560 } },
	{ { 76502, 10445 } },
	{ { 76970, 10330 } },
	{ { 77443, 10215 } },
	{ { 77921, 10101 } },
	{ { 78403,  9987 } },
	{ { 78890,  9873 } },
	{ { 79383,  9759 } },
	{ { 79880,  9646 } },
	{ { 80383,  9533 } },
	{ { 80892,  9420 } },
	{ { 81406,  9307 } },
	{ { 81925,  9195 } },
	{ { 82450,  9083 } },
	{ { 82982,  8971 } },
	{ { 83519,  8860 } },
	{ { 84062,  8749 } },
	{ { 84612,  8638 } },
	{ { 85168,  8527 } },
	{ { 85731,  8416 } },
	{ { 86300,  8306 } },
	{ { 86877,  8196 } },
	{ { 87461,  8087 } },
	{ { 88051,  7977 } },
	{ { 88650,  7868 } },
	{ { 89256,  7759 } },
	{ { 89870,  7650 } },
	{ { 90492,  7542 } },
	{ { 91122,  7434 } },
	{ { 91761,  7326 } },
	{ { 92409,  7218 } },
	{ { 93065,  7111 } },
	{ { 93731,  7004 } },
	{ { 94407,  6897 } },
	{ { 95092,  6790 } },
	{ { 95787,  6683 } },
	{ { 96493,  6577 } },
	{ { 97209,  6471 } },
	{ { 97936,  6365 } },
	{ { 98675,  6260 } },
	{ { 99425,  6155 } },
	{ { 100188,  6050 } },
	{ { 100963,  5945 } },
	{ { 101751,  5840 } },
	{ { 102552,  5736 } },
	{ { 103367,  5632 } },
	{ { 104196,  5528 } },
	{ { 105041,  5424 } },
	{ { 105900,  5321 } },
	{ { 106776,  5217 } },
	{ { 107668,  5114 } },
	{ { 108577,  5012 } },
	{ { 109504,  4909 } },
	{ { 110449,  4807 } },
	{ { 111414,  4705 } },
	{ { 112399,  4603 } },
	{ { 113405,  4501 } },
	{ { 114433,  4400 } },
	{ { 115483,  4298 } },
	{ { 116558,  4197 } },
	{ { 117657,  4097 } },
	{ { 118783,  3996 } },
	{ { 119936,  3896 } },
	{ { 121118,  3796 } },
	{ { 122330,  3696 } },
	{ { 123574,  3596 } },
	{ { 124852,  3496 } },
	{ { 126165,  3397 } },
	{ { 127516,  3298 } },
	{ { 128906,  3199 } },
	{ { 130339,  3100 } },
	{ { 131816,  3002 } },
	{ { 133342,  2904 } },
	{ { 134918,  2805 } },
	{ { 136548,  2708 } },
	{ { 138236,  2610 } },
	{ { 139988,  2512 } },
	{ { 141806,  2415 } },
	{ { 143697,  2318 } },
	{ { 145667,  2221 } },
	{ { 147723,  2125 } },
	{ { 149872,  2028 } },
	{ { 152124,  1932 } },
	{ { 154488,  1836 } },
	{ { 156977,  1740 } },
	{ { 159604,  1644 } },
	{ { 162385,  1549 } },
	{ { 165341,  1453 } },
	{ { 168494,  1358 } },
	{ { 171872,  1263 } },
	{ { 175510,  1168 } },
	{ { 179452,  1074 } },
	{ { 183753,   980 } },
	{ { 188484,   885 } },
	{ { 193742,   791 } },
	{ { 199659,   698 } },
	{ { 206424,   604 } },
	{ { 214321,   511 } },
	{ { 223808,   417 } },
	{ { 235689,   324 } },
	{ { 251595,   231 } },
	{ { 275744,   139 } },
	{ { 327680,    46 } },
};
#else
const BinFracBits ProbModelTables::m_binFracBits[256] = {
  { { 0x0005c, 0x48000 } }, { { 0x00116, 0x3b520 } }, { { 0x001d0, 0x356cb } }, { { 0x0028b, 0x318a9 } },
  { { 0x00346, 0x2ea40 } }, { { 0x00403, 0x2c531 } }, { { 0x004c0, 0x2a658 } }, { { 0x0057e, 0x28beb } },
  { { 0x0063c, 0x274ce } }, { { 0x006fc, 0x26044 } }, { { 0x007bc, 0x24dc9 } }, { { 0x0087d, 0x23cfc } },
  { { 0x0093f, 0x22d96 } }, { { 0x00a01, 0x21f60 } }, { { 0x00ac4, 0x2122e } }, { { 0x00b89, 0x205dd } },
  { { 0x00c4e, 0x1fa51 } }, { { 0x00d13, 0x1ef74 } }, { { 0x00dda, 0x1e531 } }, { { 0x00ea2, 0x1db78 } },
  { { 0x00f6a, 0x1d23c } }, { { 0x01033, 0x1c970 } }, { { 0x010fd, 0x1c10b } }, { { 0x011c8, 0x1b903 } },
  { { 0x01294, 0x1b151 } }, { { 0x01360, 0x1a9ee } }, { { 0x0142e, 0x1a2d4 } }, { { 0x014fc, 0x19bfc } },
  { { 0x015cc, 0x19564 } }, { { 0x0169c, 0x18f06 } }, { { 0x0176d, 0x188de } }, { { 0x0183f, 0x182e8 } },
  { { 0x01912, 0x17d23 } }, { { 0x019e6, 0x1778a } }, { { 0x01abb, 0x1721c } }, { { 0x01b91, 0x16cd5 } },
  { { 0x01c68, 0x167b4 } }, { { 0x01d40, 0x162b6 } }, { { 0x01e19, 0x15dda } }, { { 0x01ef3, 0x1591e } },
  { { 0x01fcd, 0x15480 } }, { { 0x020a9, 0x14fff } }, { { 0x02186, 0x14b99 } }, { { 0x02264, 0x1474e } },
  { { 0x02343, 0x1431b } }, { { 0x02423, 0x13f01 } }, { { 0x02504, 0x13afd } }, { { 0x025e6, 0x1370f } },
  { { 0x026ca, 0x13336 } }, { { 0x027ae, 0x12f71 } }, { { 0x02894, 0x12bc0 } }, { { 0x0297a, 0x12821 } },
  { { 0x02a62, 0x12494 } }, { { 0x02b4b, 0x12118 } }, { { 0x02c35, 0x11dac } }, { { 0x02d20, 0x11a51 } },
  { { 0x02e0c, 0x11704 } }, { { 0x02efa, 0x113c7 } }, { { 0x02fe9, 0x11098 } }, { { 0x030d9, 0x10d77 } },
  { { 0x031ca, 0x10a63 } }, { { 0x032bc, 0x1075c } }, { { 0x033b0, 0x10461 } }, { { 0x034a5, 0x10173 } },
  { { 0x0359b, 0x0fe90 } }, { { 0x03693, 0x0fbb9 } }, { { 0x0378c, 0x0f8ed } }, { { 0x03886, 0x0f62b } },
  { { 0x03981, 0x0f374 } }, { { 0x03a7e, 0x0f0c7 } }, { { 0x03b7c, 0x0ee23 } }, { { 0x03c7c, 0x0eb89 } },
  { { 0x03d7d, 0x0e8f9 } }, { { 0x03e7f, 0x0e671 } }, { { 0x03f83, 0x0e3f2 } }, { { 0x04088, 0x0e17c } },
  { { 0x0418e, 0x0df0e } }, { { 0x04297, 0x0dca8 } }, { { 0x043a0, 0x0da4a } }, { { 0x044ab, 0x0d7f3 } },
  { { 0x045b8, 0x0d5a5 } }, { { 0x046c6, 0x0d35d } }, { { 0x047d6, 0x0d11c } }, { { 0x048e7, 0x0cee3 } },
  { { 0x049fa, 0x0ccb0 } }, { { 0x04b0e, 0x0ca84 } }, { { 0x04c24, 0x0c85e } }, { { 0x04d3c, 0x0c63f } },
  { { 0x04e55, 0x0c426 } }, { { 0x04f71, 0x0c212 } }, { { 0x0508d, 0x0c005 } }, { { 0x051ac, 0x0bdfe } },
  { { 0x052cc, 0x0bbfc } }, { { 0x053ee, 0x0b9ff } }, { { 0x05512, 0x0b808 } }, { { 0x05638, 0x0b617 } },
  { { 0x0575f, 0x0b42a } }, { { 0x05888, 0x0b243 } }, { { 0x059b4, 0x0b061 } }, { { 0x05ae1, 0x0ae83 } },
  { { 0x05c10, 0x0acaa } }, { { 0x05d41, 0x0aad6 } }, { { 0x05e74, 0x0a907 } }, { { 0x05fa9, 0x0a73c } },
  { { 0x060e0, 0x0a575 } }, { { 0x06219, 0x0a3b3 } }, { { 0x06354, 0x0a1f5 } }, { { 0x06491, 0x0a03b } },
  { { 0x065d1, 0x09e85 } }, { { 0x06712, 0x09cd4 } }, { { 0x06856, 0x09b26 } }, { { 0x0699c, 0x0997c } },
  { { 0x06ae4, 0x097d6 } }, { { 0x06c2f, 0x09634 } }, { { 0x06d7c, 0x09495 } }, { { 0x06ecb, 0x092fa } },
  { { 0x0701d, 0x09162 } }, { { 0x07171, 0x08fce } }, { { 0x072c7, 0x08e3e } }, { { 0x07421, 0x08cb0 } },
  { { 0x0757c, 0x08b26 } }, { { 0x076da, 0x089a0 } }, { { 0x0783b, 0x0881c } }, { { 0x0799f, 0x0869c } },
  { { 0x07b05, 0x0851f } }, { { 0x07c6e, 0x083a4 } }, { { 0x07dd9, 0x0822d } }, { { 0x07f48, 0x080b9 } },
  { { 0x080b9, 0x07f48 } }, { { 0x0822d, 0x07dd9 } }, { { 0x083a4, 0x07c6e } }, { { 0x0851f, 0x07b05 } },
  { { 0x0869c, 0x0799f } }, { { 0x0881c, 0x0783b } }, { { 0x089a0, 0x076da } }, { { 0x08b26, 0x0757c } },
  { { 0x08cb0, 0x07421 } }, { { 0x08e3e, 0x072c7 } }, { { 0x08fce, 0x07171 } }, { { 0x09162, 0x0701d } },
  { { 0x092fa, 0x06ecb } }, { { 0x09495, 0x06d7c } }, { { 0x09634, 0x06c2f } }, { { 0x097d6, 0x06ae4 } },
  { { 0x0997c, 0x0699c } }, { { 0x09b26, 0x06856 } }, { { 0x09cd4, 0x06712 } }, { { 0x09e85, 0x065d1 } },
  { { 0x0a03b, 0x06491 } }, { { 0x0a1f5, 0x06354 } }, { { 0x0a3b3, 0x06219 } }, { { 0x0a575, 0x060e0 } },
  { { 0x0a73c, 0x05fa9 } }, { { 0x0a907, 0x05e74 } }, { { 0x0aad6, 0x05d41 } }, { { 0x0acaa, 0x05c10 } },
  { { 0x0ae83, 0x05ae1 } }, { { 0x0b061, 0x059b4 } }, { { 0x0b243, 0x05888 } }, { { 0x0b42a, 0x0575f } },
  { { 0x0b617, 0x05638 } }, { { 0x0b808, 0x05512 } }, { { 0x0b9ff, 0x053ee } }, { { 0x0bbfc, 0x052cc } },
  { { 0x0bdfe, 0x051ac } }, { { 0x0c005, 0x0508d } }, { { 0x0c212, 0x04f71 } }, { { 0x0c426, 0x04e55 } },
  { { 0x0c63f, 0x04d3c } }, { { 0x0c85e, 0x04c24 } }, { { 0x0ca84, 0x04b0e } }, { { 0x0ccb0, 0x049fa } },
  { { 0x0cee3, 0x048e7 } }, { { 0x0d11c, 0x047d6 } }, { { 0x0d35d, 0x046c6 } }, { { 0x0d5a5, 0x045b8 } },
  { { 0x0d7f3, 0x044ab } }, { { 0x0da4a, 0x043a0 } }, { { 0x0dca8, 0x04297 } }, { { 0x0df0e, 0x0418e } },
  { { 0x0e17c, 0x04088 } }, { { 0x0e3f2, 0x03f83 } }, { { 0x0e671, 0x03e7f } }, { { 0x0e8f9, 0x03d7d } },
  { { 0x0eb89, 0x03c7c } }, { { 0x0ee23, 0x03b7c } }, { { 0x0f0c7, 0x03a7e } }, { { 0x0f374, 0x03981 } },
  { { 0x0f62b, 0x03886 } }, { { 0x0f8ed, 0x0378c } }, { { 0x0fbb9, 0x03693 } }, { { 0x0fe90, 0x0359b } },
  { { 0x10173, 0x034a5 } }, { { 0x10461, 0x033b0 } }, { { 0x1075c, 0x032bc } }, { { 0x10a63, 0x031ca } },
  { { 0x10d77, 0x030d9 } }, { { 0x11098, 0x02fe9 } }, { { 0x113c7, 0x02efa } }, { { 0x11704, 0x02e0c } },
  { { 0x11a51, 0x02d20 } }, { { 0x11dac, 0x02c35 } }, { { 0x12118, 0x02b4b } }, { { 0x12494, 0x02a62 } },
  { { 0x12821, 0x0297a } }, { { 0x12bc0, 0x02894 } }, { { 0x12f71, 0x027ae } }, { { 0x13336, 0x026ca } },
  { { 0x1370f, 0x025e6 } }, { { 0x13afd, 0x02504 } }, { { 0x13f01, 0x02423 } }, { { 0x1431b, 0x02343 } },
  { { 0x1474e, 0x02264 } }, { { 0x14b99, 0x02186 } }, { { 0x14fff, 0x020a9 } }, { { 0x15480, 0x01fcd } },
  { { 0x1591e, 0x01ef3 } }, { { 0x15dda, 0x01e19 } }, { { 0x162b6, 0x01d40 } }, { { 0x167b4, 0x01c68 } },
  { { 0x16cd5, 0x01b91 } }, { { 0x1721c, 0x01abb } }, { { 0x1778a, 0x019e6 } }, { { 0x17d23, 0x01912 } },
  { { 0x182e8, 0x0183f } }, { { 0x188de, 0x0176d } }, { { 0x18f06, 0x0169c } }, { { 0x19564, 0x015cc } },
  { { 0x19bfc, 0x014fc } }, { { 0x1a2d4, 0x0142e } }, { { 0x1a9ee, 0x01360 } }, { { 0x1b151, 0x01294 } },
  { { 0x1b903, 0x011c8 } }, { { 0x1c10b, 0x010fd } }, { { 0x1c970, 0x01033 } }, { { 0x1d23c, 0x00f6a } },
  { { 0x1db78, 0x00ea2 } }, { { 0x1e531, 0x00dda } }, { { 0x1ef74, 0x00d13 } }, { { 0x1fa51, 0x00c4e } },
  { { 0x205dd, 0x00b89 } }, { { 0x2122e, 0x00ac4 } }, { { 0x21f60, 0x00a01 } }, { { 0x22d96, 0x0093f } },
  { { 0x23cfc, 0x0087d } }, { { 0x24dc9, 0x007bc } }, { { 0x26044, 0x006fc } }, { { 0x274ce, 0x0063c } },
  { { 0x28beb, 0x0057e } }, { { 0x2a658, 0x004c0 } }, { { 0x2c531, 0x00403 } }, { { 0x2ea40, 0x00346 } },
  { { 0x318a9, 0x0028b } }, { { 0x356cb, 0x001d0 } }, { { 0x3b520, 0x00116 } }, { { 0x48000, 0x0005c } },
};
#endif
void BinProbModel_Std::init( int qp, int initId )
{
  int slope = (initId >> 3) - 4;
  int offset = ((initId & 7) * 18) + 1;
  int inistate = ((slope   * (qp - 16)) >> 1) + offset;
  int state_clip = inistate < 1 ? 1 : inistate > 127 ? 127 : inistate;
  const int p1 = (state_clip << 8);
  m_state[0]   = p1 & MASK_0;
  m_state[1]   = p1 & MASK_1;
#if JVET_Z0135_TEMP_CABAC_WIN_WEIGHT 
  m_stateUsed[0] = m_state[0];
  m_stateUsed[1] = m_state[1];
#endif
}




CtxSet::CtxSet( std::initializer_list<CtxSet> ctxSets )
{
  uint16_t  minOffset = std::numeric_limits<uint16_t>::max();
  uint16_t  maxOffset = 0;
  for( auto iter = ctxSets.begin(); iter != ctxSets.end(); iter++ )
  {
    minOffset = std::min<uint16_t>( minOffset, (*iter).Offset              );
    maxOffset = std::max<uint16_t>( maxOffset, (*iter).Offset+(*iter).Size );
  }
  Offset  = minOffset;
  Size    = maxOffset - minOffset;
}





const std::vector<uint8_t>& ContextSetCfg::getInitTable( unsigned initId )
{
  CHECK( initId >= (unsigned)sm_InitTables.size(),
         "Invalid initId (" << initId << "), only " << sm_InitTables.size() << " tables defined." );
  return sm_InitTables[initId];
}

CtxSet ContextSetCfg::addCtxSet( std::initializer_list<std::initializer_list<uint8_t>> initSet2d )
{
  const std::size_t startIdx  = sm_InitTables[0].size();
  const std::size_t numValues = ( *initSet2d.begin() ).size();
  std::size_t setId     = 0;
  for( auto setIter = initSet2d.begin(); setIter != initSet2d.end() && setId < sm_InitTables.size(); setIter++, setId++ )
  {
    const std::initializer_list<uint8_t>& initSet   = *setIter;
    std::vector<uint8_t>&           initTable = sm_InitTables[setId];
    CHECK( initSet.size() != numValues,
      "Number of init values do not match for all sets (" << initSet.size() << " != " << numValues << ")." );
    initTable.resize( startIdx + numValues );
    std::size_t elemId = startIdx;
    for( auto elemIter = ( *setIter ).begin(); elemIter != ( *setIter ).end(); elemIter++, elemId++ )
    {
      initTable[elemId] = *elemIter;
    }
  }
  return CtxSet( (uint16_t)startIdx, (uint16_t)numValues );
}

#define CNU 35
#if SLICE_TYPE_WIN_SIZE
#if JVET_Z0135_TEMP_CABAC_WIN_WEIGHT
std::vector<std::vector<uint8_t>> ContextSetCfg::sm_InitTables( NUMBER_OF_SLICE_TYPES * 3 + 2);
#else
std::vector<std::vector<uint8_t>> ContextSetCfg::sm_InitTables(NUMBER_OF_SLICE_TYPES << 1);
#endif
#else
std::vector<std::vector<uint8_t>> ContextSetCfg::sm_InitTables(NUMBER_OF_SLICE_TYPES + 1);
#endif

// clang-format off
#if JVET_Z0135_TEMP_CABAC_WIN_WEIGHT
const CtxSet ContextSetCfg::SplitFlag = ContextSetCfg::addCtxSet
({
  {  11,  20,  44,   4,   6,  30,  20,  15,  31, },
  {  11,  20,  52,  12,  21,  30,  13,  15,  31, },
  {  27,  36,  38,  35,  29,  38,  28,  38,  31, },
  {   5,   9,   4,   9,  13,  12,   9,   9,  12, },
  {  12,  12,   4,  13,  13,  12,   9,   9,  12, },
  {  12,  12,  13,   9,  13,  12,   5,   5,   9, },
  {   4,  11,  11,  11,   4,  11,   4,   4,  11, },
  {  11,  18,  11,  18,  11,   4,  11,   4,   4, },
  {  18,  11,  32,  11,  11,   4,  11,   4,   4, },
  { 131, 134, 142, 133, 238, 238, 110, 116, 137, },
  { 140, 237, 101, 202, 236, 227, 121, 125,  84, },
  });

const CtxSet ContextSetCfg::SplitQtFlag = ContextSetCfg::addCtxSet
({
  {  19,  14,  23,  11,  12,  13, },
  {   5,   7,  23,  11,  12,  13, },
  {   4,  13,   7,  33,  27,  22, },
  {   6,  12,  12,  12,   8,   8, },
  {   5,   8,   9,  12,  12,   8, },
  {   4,   5,   9,  12,  12,  12, },
  {   4,   4,  11,  11,   4,  18, },
  {   4,   4,   4,   4,  11,   4, },
  {  32,  18,  32,   4,   4,  18, },
  { 176, 107, 117, 124, 182, 148, },
  { 138, 238, 235, 238, 235, 181, },
  });

const CtxSet ContextSetCfg::SplitHvFlag = ContextSetCfg::addCtxSet
({
  {  43,  35,  44,  42,  37, },
  {  36,  35,  44,  34,  45, },
  {  43,  50,  29,  27,  52, },
  {  10,   9,   9,   8,   5, },
  {  10,   9,  13,   9,   6, },
  {   9,   9,   9,   4,   5, },
  {   4,  11,   4,  11,  11, },
  {  11,   4,  11,  11,   4, },
  {  18,  18,  18,   4,  18, },
  { 120, 119, 148, 211, 125, },
  { 133, 137, 157, 117, 119, },
  });

const CtxSet ContextSetCfg::Split12Flag = ContextSetCfg::addCtxSet
({
  {  36,  37,  21,  22, },
  {  36,  37,  36,  22, },
  {  44,  45,  44,  45, },
  {  12,  13,  12,  13, },
  {  12,  13,  12,  13, },
  {  12,  13,  12,  13, },
  {  11,  11,  11,   4, },
  {   4,   4,   4,   4, },
  {  18,  11,  18,  11, },
  { 237, 139, 186, 120, },
  { 102,  92, 110, 107, },
  });

#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
const CtxSet ContextSetCfg::ModeConsFlag = ContextSetCfg::addCtxSet
( {
  {   25, 20 },
  {   25, 12 },
  {   35, 35 },
  {   1,   0 },
  {   1,   0 },
  {   1,   0 }
  { DWE, DWE },
  { DWE, DWE },
  { DWE, DWE },
  { DWO, DWO },
  { DWO, DWO }
  } );
#endif

const CtxSet ContextSetCfg::SkipFlag = ContextSetCfg::addCtxSet
({
  {  56,  59,  60, },
  {  57,  59,  60, },
  {   0,  42,  36, },
  {   6,   6,  10, },
  {   6,   6,   9, },
  {   5,  10,   7, },
  {  18,  18,  18, },
  {  18,  11,  11, },
  {  18,  11,  11, },
  { 110, 117, 119, },
  { 123, 142, 228, },
  });

const CtxSet ContextSetCfg::MergeFlag = ContextSetCfg::addCtxSet
({
  {  14, },
  {   7, },
  {  19, },
  {   6, },
  {   6, },
  {   4, },
  {  18, },
  {  11, },
  {  18, },
  { 133, },
  { 105, },
  });

const CtxSet ContextSetCfg::RegularMergeFlag = ContextSetCfg::addCtxSet
({
  {  23,  14, },
  {  31,   6, },
  { CNU, CNU, },
  {   6,   5, },
  {   6,   6, },
  { DWS, DWS, },
  {   4,  18, },
  {   4,  18, },
  { DWE, DWE, },
  { 131, 110, },
  { 131, 117, },
  });

const CtxSet ContextSetCfg::MergeIdx = ContextSetCfg::addCtxSet
({
#if NON_ADJACENT_MRG_CAND
  {  26,  29,  44,  44,  29, CNU, CNU, CNU, CNU, CNU, },
  {   5,  29,  44,  37,  44, CNU, CNU, CNU, CNU, CNU, },
  {  33,  42,  43,  27,  18, CNU, CNU, CNU, CNU, CNU, },
  {   5,   5,   5,   5,   9, DWS, DWS, DWS, DWS, DWS, },
  {   6,   5,   5,   5,   9, DWS, DWS, DWS, DWS, DWS, },
  {   6,   7,   6,  13,  12, DWS, DWS, DWS, DWS, DWS, },
  {  18,  18,  18,  18,  11, DWE, DWE, DWE, DWE, DWE, },
  {  18,  18,  18,  18,  11, DWE, DWE, DWE, DWE, DWE, },
  {  18,  18,   4,   4,   4, DWE, DWE, DWE, DWE, DWE, },
  { 119, 134, 126, 126, 198, 119, 119, 119, 119, 119, },
  { 117, 105, 117, 116, 238, 119, 119, 119, 119, 119, },
#else
  {  26 },
  {   5 },
  {  33 },
  {   5 },
  {   6 },
  {   6 },
  {  18 },
  {  18 },
  {  18 },
  { 119 },
  { 117 },
#endif
  });

#if TM_MRG || (JVET_Z0084_IBC_TM && IBC_TM_MRG)
const CtxSet ContextSetCfg::TmMergeIdx = ContextSetCfg::addCtxSet
({
#if NON_ADJACENT_MRG_CAND
  {  20,  43,  42, CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
  {  21,  20,  42, CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
  { CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
  {   4,   4,   4, DWS, DWS, DWS, DWS, DWS, DWS, DWS, },
  {   4,   5,   5, DWS, DWS, DWS, DWS, DWS, DWS, DWS, },
  { DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, },
  {  18,  18,  18, DWE, DWE, DWE, DWE, DWE, DWE, DWE, },
  {  18,  18,  18, DWE, DWE, DWE, DWE, DWE, DWE, DWE, },
  { DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, },
  { 135, 118, 125, 119, 119, 119, 119, 119, 119, 119, },
  { 116, 121, 119, 119, 119, 119, 119, 119, 119, 119, },
#else
  {  20 },
  {  21 },
  { CNU },
  {   4 },
  {   4 },
  { DWS },
  {  18 },
  {  18 },
  { DWE },
  { 135 },
  { 116 },
#endif
  });
#endif

#if JVET_Y0065_GPM_INTRA
const CtxSet ContextSetCfg::GPMIntraFlag = ContextSetCfg::addCtxSet
({
  {  19, },
  {  26, },
  { CNU, },
  {   6, },
  {   6, },
  { DWS, },
  {  18, },
  {  18, },
  { DWE, },
  { 196, },
  { 121, },
  });
#endif

const CtxSet ContextSetCfg::MmvdFlag = ContextSetCfg::addCtxSet
({
#if JVET_AA0093_ENHANCED_MMVD_EXTENSION
  {  18, 18, 18, 18},
  {  18, 18, 18, 18},
  { CNU, CNU, CNU, CNU },
  {   6, 6, 6, 6 },
  {   5, 5, 5, 5},
  { DWS, DWS, DWS, DWS},
  {  18, 18, 18, 18},
  {  18, 18, 18, 18},
  { DWE, DWE, DWE, DWE},
  { 117, 117, 117, 117},
  { 117, 117, 117, 117},
#else
  {  18, },
  {  18, },
  { CNU, },
  {   6, },
  {   5, },
  { DWS, },
  {  18, },
  {  18, },
  { DWE, },
  { 117, },
  { 117, },
#endif
  });

const CtxSet ContextSetCfg::MmvdMergeIdx = ContextSetCfg::addCtxSet
({
#if JVET_AA0093_ENHANCED_MMVD_EXTENSION
  {  58,  58,  58,  58, },
  {  43,  43,  43,  43, },
  { CNU, CNU, CNU, CNU, },
  {   9,   9,   9,   9, },
  {  10,  10,  10,  10, },
  { DWS, DWS, DWS, DWS, },
  {  18,  18,  18,  18, },
  {  11,  11,  11,  11, },
  { DWE, DWE, DWE, DWE, },
  { 116, 116, 116, 116, },
  { 118, 118, 118, 118, },
#else
  {  58, },
  {  43, },
  { CNU, },
  {   9, },
  {  10, },
  { DWS, },
  {  18, },
  {  11, },
  { DWE, },
  { 116, },
  { 118, },
#endif
  });

const CtxSet ContextSetCfg::MmvdStepMvpIdx = ContextSetCfg::addCtxSet
({
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
  {  43,  36,  36,  28,  35, },
  {  28,  21,  36,  28,  43, },
  { CNU, CNU, CNU, CNU, CNU, },
  {   5,   4,   4,   4,   4, },
  {   5,   5,   5,   5,   4, },
  { DWS, DWS, DWS, DWS, DWS, },
  {  18,  11,  11,  11,  11, },
  {  18,  18,  18,  18,  11, },
  { DWE, DWE, DWE, DWE, DWE, },
  { 142, 157, 157, 142, 155, },
  { 116, 103, 104, 108, 117, },
#else
  {  59 },
  {  60 },
  {  35 },
  {   0 },
  {   0 },
  {   0 },
  { DWE },
  { DWE },
  { DWE },
  { DWO },
  { DWO },
#endif
  });

#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
const CtxSet ContextSetCfg::MmvdStepMvpIdxECM3 = ContextSetCfg::addCtxSet
({
  {  59 },
  {  60 },
  {  35 },
  {   0 },
  {   0 },
  {   0 },
  { DWE },
  { DWE },
  { DWE },
  { DWO },
  { DWO },
});
#endif

#if JVET_W0097_GPM_MMVD_TM
const CtxSet ContextSetCfg::GeoMmvdFlag = ContextSetCfg::addCtxSet
({
  {  33, },
  {  26, },
  { CNU, },
  {   3, },
  {   2, },
  { DWS, },
  {   4, },
  {   4, },
  { DWE, },
  { 109, },
  { 115, },
  });

const CtxSet ContextSetCfg::GeoMmvdStepMvpIdx = ContextSetCfg::addCtxSet
({
  {  59, },
  {  60, },
  { CNU, },
  {   1, },
  {   1, },
  { DWS, },
  {  11, },
  {  11, },
  { DWE, },
  { 126, },
  { 122, },
  });
#endif

#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
const CtxSet ContextSetCfg::GeoBldFlag = ContextSetCfg::addCtxSet
({
  { 59,  59,  59,  59 },
  { 60,  60,  60,  60 },
  { CNU, CNU, CNU, CNU },
  { 1,   1,   1,   1 },
  { 1,   1,   1,   1 },
  { DWS, DWS, DWS, DWS },
  { 11,  11,  11,  11 },
  { 11,  11,  11,  11 },
  { DWE, DWE, DWE, DWE },
  { 126, 126, 126, 126 },
  { 122, 122, 122, 122 },
});
#endif

#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
const CtxSet ContextSetCfg::GeoSubModeIdx = ContextSetCfg::addCtxSet
( {
  {  33,  28,  36,  36,  29, },
  {  20,  21,  29,  29,  29, },
  {  34,  43,  36,  35,  25, },
  {   4,   5,   4,   4,   8, },
  {   5,   5,   5,   4,   8, },
  {   5,   4,  10,  13,  13, },
  { DWE, DWE, DWE, DWE, DWE, },
  { DWE, DWE, DWE, DWE, DWE, },
  { DWE, DWE, DWE, DWE, DWE, },
  { DWO, DWO, DWO, DWO, DWO, },
  { DWO, DWO, DWO, DWO, DWO, }
  } );
#endif

#if AFFINE_MMVD
const CtxSet ContextSetCfg::AfMmvdFlag = ContextSetCfg::addCtxSet
({
  {  11, },
  {   4, },
  { CNU, },
  {   5, },
  {   5, },
  { DWS, },
  {  18, },
  {  25, },
  { DWE, },
  { 119, },
  { 132, },
  });

const CtxSet ContextSetCfg::AfMmvdIdx = ContextSetCfg::addCtxSet
({
#if JVET_AA0093_ENHANCED_MMVD_EXTENSION
  { CNU, CNU,CNU,CNU},
  { CNU, CNU,CNU,CNU},
  { CNU, CNU,CNU,CNU},
  { DWS, DWS,DWS,DWS},
  { DWS, DWS,DWS,DWS},
  { DWS, DWS,DWS,DWS},
  { DWE, DWE,DWE,DWE},
  { DWE, DWE,DWE,DWE},
  { DWE, DWE,DWE,DWE},
  { 119, 119,119,119},
  { 119, 119,119,119},
#else
  { CNU, },
  { CNU, },
  { CNU, },
  { DWS, },
  { DWS, },
  { DWS, },
  { DWE, },
  { DWE, },
  { DWE, },
  { 119, },
  { 119, },
#endif
  });

const CtxSet ContextSetCfg::AfMmvdOffsetStep = ContextSetCfg::addCtxSet
({
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
#if JVET_AA0093_ENHANCED_MMVD_EXTENSION
  {  21,  29,  29,  29,  44, CNU},
  {  13,  21,  36,  29,  44, CNU},
  { CNU, CNU, CNU, CNU, CNU, CNU},
  {   5,   5,   5,   4,   5, DWS},
  {   5,   5,   4,   5,   9, DWS},
  { DWS, DWS, DWS, DWS, DWS, DWS},
  {  18,  18,  18,  11,  11, DWE},
  {  18,  18,  11,  18,  25, DWE},
  { DWE, DWE, DWE, DWE, DWE, DWE},
  { 126, 141, 187, 219, 238, DWO},
  { 117, 102, 100, 100,  87, DWO},
#else
  {  21,  29,  29,  29,  44, },
  {  13,  21,  36,  29,  44, },
  { CNU, CNU, CNU, CNU, CNU, },
  {   5,   5,   5,   4,   5, },
  {   5,   5,   4,   5,   9, },
  { DWS, DWS, DWS, DWS, DWS, },
  {  18,  18,  18,  11,  11, },
  {  18,  18,  11,  18,  25, },
  { DWE, DWE, DWE, DWE, DWE, },
  { 126, 141, 187, 219, 238, },
  { 117, 102, 100, 100,  87, },
#endif
#else
  {  21 },
  {  13 },
  { CNU },
  {   5 },
  {   5 },
  { DWS },
  {  18 },
  {  18 },
  { DWE },
  { 126 },
  { 117 },
#endif
  });

#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
const CtxSet ContextSetCfg::AfMmvdOffsetStepECM3 = ContextSetCfg::addCtxSet
({
  {  21 },
  {  13 },
  { CNU },
  {   5 },
  {   5 },
  { DWS },
  {  18 },
  {  18 },
  { DWE },
  { 126 },
  { 117 },
});
#endif
#endif

#if JVET_AA0061_IBC_MBVD
const CtxSet ContextSetCfg::IbcMbvdFlag = ContextSetCfg::addCtxSet
({
#if JVET_AE0169_BIPREDICTIVE_IBC
  {  18,  35, },
  {  18,  35, },
  { CNU, CNU, },
  {   6,   6, },
  {   5,   5, },
  { DWS, DWS, },
  {  18,  18, },
  {  18,  18, },
  { DWE, DWE, },
  { 117, 119, },
  { 117, 119, },
#else
  {  18, },
  {  18, },
  { CNU, },
  {   6, },
  {   5, },
  { DWS, },
  {  18, },
  {  18, },
  { DWE, },
  { 117, },
  { 117, },
#endif
  });

const CtxSet ContextSetCfg::IbcMbvdMergeIdx = ContextSetCfg::addCtxSet
({
  {  58, },
  {  43, },
  { CNU, },
  {   9, },
  {  10, },
  { DWS, },
  {  18, },
  {  11, },
  { DWE, },
  { 116, },
  { 118, },
  });

const CtxSet ContextSetCfg::IbcMbvdStepMvpIdx = ContextSetCfg::addCtxSet
({
  {  43,  36,  36,  28,  35, },
  {  28,  21,  36,  28,  43, },
  { CNU, CNU, CNU, CNU, CNU, },
  {   5,   4,   4,   4,   4, },
  {   5,   5,   5,   5,   4, },
  { DWS, DWS, DWS, DWS, DWS, },
  {  18,  11,  11,  11,  11, },
  {  18,  18,  18,  18,  11, },
  { DWE, DWE, DWE, DWE, DWE, },
  { 142, 157, 157, 142, 155, },
  { 116, 103, 104, 108, 117, },
  });
#endif

#if TM_MRG || (JVET_Z0084_IBC_TM && IBC_TM_MRG)
const CtxSet ContextSetCfg::TMMergeFlag = ContextSetCfg::addCtxSet
({
#if JVET_Z0084_IBC_TM && IBC_TM_MRG
  {  25,  33 },
  {  33,  25 },
  { CNU,  35 },
  {   5,   5 },
  {   5,   5 },
  { DWS,   4 },
  {  18,  18 },
  {  18,  18 },
  { DWE, DWE },
  { 119, 119 },
  { 125, 125 },
#else
  {  25, },
  {  33, },
  { CNU, },
  {   5, },
  {   5, },
  { DWS, },
  {  18, },
  {  18, },
  { DWE, },
  { 119, },
  { 125, },
#endif
});
#endif

#if TM_MRG
#if JVET_X0141_CIIP_TIMD_TM
const CtxSet ContextSetCfg::CiipTMMergeFlag = ContextSetCfg::addCtxSet
({
  {  18, },
  {  26, },
  { CNU, },
  {   5, },
  {   5, },
  { DWS, },
  {  18, },
  {  11, },
  { DWE, },
  { 141, },
  { 118, },
  });
#endif
#endif

const CtxSet ContextSetCfg::PredMode = ContextSetCfg::addCtxSet
({
  {  25,  21, },
  {  40,  28, },
  { CNU, CNU, },
  {   6,   2, },
  {   6,   2, },
  { DWS, DWS, },
  {  18,  18, },
  {  18,  18, },
  { DWE, DWE, },
  { 106, 119, },
  { 117, 118, },
  });

const CtxSet ContextSetCfg::MultiRefLineIdx = ContextSetCfg::addCtxSet
({
#if JVET_Y0116_EXTENDED_MRL_LIST
#if JVET_W0123_TIMD_FUSION
  {  18,  51,  51,  51,  43,  25,  59, },
  {  25,  50,  50,  43,  50,  25,  58, },
  {  25,  59,  52,  44,  59,  25,  60, },
  {   6,   4,   5,   5,   6,   6,   4, },
  {   6,   4,   5,   4,   5,   6,   4, },
  {   6,   4,   8,   4,   4,   6,   8, },
  {  18,  11,  11,  18,  18,  11,  11, },
  {  11,  11,  18,   4,  11,  11,  18, },
  {  18,   4,  18,   4,   4,  18,  25, },
  { 123, 115, 109, 101, 100, 124, 117, },
  { 146, 150, 138, 139, 168, 121, 148, },
#else
  {  18,  51,  51,  51,  43 },
  {  25,  50,  50,  43,  50 },
  {  25,  59,  52,  44,  59 },
  {   6,   4,   5,   5,   6 },
  {   6,   4,   5,   4,   5 },
  {   6,   4,   8,   4,   4 },
  {  18,  11,  11,  18,  18 },
  {  11,  11,  18,   4,  11 },
  {  18,   4,  18,   4,   4 },
  { 123, 115, 109, 101, 100 },
  { 146, 150, 138, 139, 168 },
#endif
#else
#if JVET_W0123_TIMD_FUSION
  {  18,  51,  51,  51 },
  {  25,  50,  50,  43 },
  {  25,  59,  52,  44 },
  {   6,   4,   5,   5 },
  {   6,   4,   5,   4 },
  {   6,   4,   8,   4 },
  {  18,  11,  11,  18 },
  {  11,  11,  18,   4 },
  {  18,   4,  18,   4 },
  { 123, 115, 109, 101 },
  { 146, 150, 138, 139 },
#else
  {  18,  51 },
  {  25,  50 },
  {  25,  59 },
  {   6,   4 },
  {   6,   4 },
  {   6,   4 },
  {  18,  11 },
  {  11,  11 },
  {  18,   4 },
  { 123, 115 },
  { 146, 150 },
#endif
#endif
  });

const CtxSet ContextSetCfg::IntraLumaMpmFlag = ContextSetCfg::addCtxSet
({
  {  44, },
  {  29, },
  {  29, },
  {   6, },
  {   6, },
  {   6, },
  {  11, },
  {  11, },
  {  18, },
  { 117, },
  { 119, },
  });

#if SECONDARY_MPM
const CtxSet ContextSetCfg::IntraLumaSecondMpmFlag = ContextSetCfg::addCtxSet
({
  {  36, },
  {  36, },
  {  36, },
  {   6, },
  {  10, },
  {   6, },
  {   4, },
  {  11, },
  {  11, },
  { 119, },
  { 126, },
  });

const CtxSet ContextSetCfg::IntraLumaMPMIdx = ContextSetCfg::addCtxSet
({
  {  28,  44,  28, },
  {  21,  37,  28, },
  {  20,  44,  50, },
  {   1,   2,   6, },
  {   6,   2,   6, },
  {   1,   2,   6, },
  {  11,  11,  18, },
  {  11,   4,  11, },
  {  11,  11,  18, },
  { 121, 126, 119, },
  { 119, 109, 121, },
  });

#if JVET_AD0085_MPM_SORTING
const CtxSet ContextSetCfg::IntraLumaSecondMpmIdx = ContextSetCfg::addCtxSet
({
  { CNU, CNU, CNU, CNU, CNU },
  { CNU, CNU, CNU, CNU, CNU },
  { CNU, CNU, CNU, CNU, CNU },
  { DWS, DWS, DWS, DWS, DWS },
  { DWS, DWS, DWS, DWS, DWS },
  { DWS, DWS, DWS, DWS, DWS },
  { DWE, DWE, DWE, DWE, DWE },
  { DWE, DWE, DWE, DWE, DWE },
  { DWE, DWE, DWE, DWE, DWE },
  { 119, 119, 119, 119, 119 },
  { 119, 119, 119, 119, 119 },
  });
#endif
#endif

const CtxSet ContextSetCfg::IntraLumaPlanarFlag = ContextSetCfg::addCtxSet
({
#if JVET_AC0105_DIRECTIONAL_PLANAR
  {   6,   6,  CNU,  CNU, },
  {   6,  28,  CNU,  CNU, },
  {  23,  21,  CNU,  CNU, },
  {   1,   2,  DWS,  DWS, },
  {   1,   2,  DWS,  DWS, },
  {   1,   6,  DWS,  DWS, },
  {  18,  18,  DWE,  DWE, },
  {  18,  18,  DWE,  DWE, },
  {  11,  25,  DWE,  DWE, },
  { 125, 116,  DWO,  DWO, },
  { 116, 117,  DWO,  DWO, },
#else
  {   6,   6, },
  {   6,  28, },
  {  23,  21, },
  {   1,   2, },
  {   1,   2, },
  {   1,   6, },
  {  18,  18, },
  {  18,  18, },
  {  11,  25, },
  { 125, 116, },
  { 116, 117, },
#endif
  });

const CtxSet ContextSetCfg::CclmModeFlag = ContextSetCfg::addCtxSet
({
  {  26, },
  {  26, },
  {  59, },
  {   1, },
  {   5, },
  {   5, },
  {  18, },
  {  25, },
  {  25, },
  { 117, },
  { 147, },
  });

const CtxSet ContextSetCfg::CclmModeIdx = ContextSetCfg::addCtxSet
({
  {  26, },
  {  34, },
  {  12, },
  {   5, },
  {   5, },
  {   5, },
  {  11, },
  {  11, },
  {  11, },
  { 116, },
  { 131, },
  });

const CtxSet ContextSetCfg::IntraChromaPredMode = ContextSetCfg::addCtxSet
({
  {  25, },
  {  25, },
  {  34, },
  {   5, },
  {   5, },
  {   5, },
  {  18, },
  {  11, },
  {  18, },
  { 116, },
  { 118, },
  });

#if JVET_Z0050_DIMD_CHROMA_FUSION
#if ENABLE_DIMD
const CtxSet ContextSetCfg::DimdChromaMode = ContextSetCfg::addCtxSet
({
  { CNU, },
  { CNU, },
  { CNU, },
  { DWS, },
  { DWS, },
  { DWS, },
  { DWE, },
  { DWE, },
  { DWE, },
  { DWO, },
  { DWO, },
  });
#endif

const CtxSet ContextSetCfg::ChromaFusionMode = ContextSetCfg::addCtxSet
({
  { CNU, },
  { CNU, },
  { CNU, },
  { DWS, },
  { DWS, },
  { DWS, },
  { DWE, },
  { DWE, },
  { DWE, },
  { DWO, },
  { DWO, },
  });
#endif

#if JVET_AC0071_DBV
const CtxSet ContextSetCfg::DbvChromaMode = ContextSetCfg::addCtxSet
({
  { CNU, },
  { CNU, },
  { CNU, },
  { DWS, },
  { DWS, },
  { DWS, },
  { DWE, },
  { DWE, },
  { DWE, },
  { DWO, },
  { DWO, },
  });
#endif

const CtxSet ContextSetCfg::MipFlag = ContextSetCfg::addCtxSet
({
  {  48,  49,  42,  33, },
  {  48,  49,  50,  33, },
  {  48,  41,  42,  25, },
  {   9,  10,   8,   6, },
  {   9,   9,   8,   6, },
  {  10,  10,   8,   6, },
  {  18,  18,  18,  18, },
  {  11,  11,  18,  11, },
  {  18,  11,  11,  11, },
  { 104, 118, 118, 108, },
  { 117, 118, 120, 124, },
  });

#if JVET_V0130_INTRA_TMP
const CtxSet ContextSetCfg::TmpFlag = ContextSetCfg::addCtxSet
({
#if JVET_AD0086_ENHANCED_INTRA_TMP
  {  33,  42,   7,  33, CNU, CNU, CNU,},
  {   0,  25,  57,   0, CNU, CNU, CNU,},
  {  40,  19,  21,  33, CNU, CNU, CNU,},
  {   1,   4,   6,   1, DWS, DWS, DWS,},
  {   8,   8,   0,  12, DWS, DWS, DWS,},
  {   6,   5,   0,   2, DWS, DWS, DWS,},
  {  11,  32,  25,  18, DWE, DWE, DWE,},
  {   4,  25,  32,  11, DWE, DWE, DWE,},
  {  25,  32,   4,  25, DWE, DWE, DWE,},
  {  99, 101, 133, 115, DWO, DWO, DWO,},
  { 147, 161, 114, 131, DWO, DWO, DWO,},
#else
  {  33,  42,   7,  33, },
  {   0,  25,  57,   0, },
  {  40,  19,  21,  33, },
  {   1,   4,   6,   1, },
  {   8,   8,   0,  12, },
  {   6,   5,   0,   2, },
  {  11,  32,  25,  18, },
  {   4,  25,  32,  11, },
  {  25,  32,   4,  25, },
  {  99, 101, 133, 115, },
  { 147, 161, 114, 131, },
#endif
  });
#endif

#if JVET_AD0086_ENHANCED_INTRA_TMP
const CtxSet ContextSetCfg::TmpIdx = ContextSetCfg::addCtxSet
({
  { CNU, CNU, CNU,},
  { CNU, CNU, CNU,},
  { CNU, CNU, CNU,},
  { DWS, DWS, DWS,},
  { DWS, DWS, DWS,},
  { DWS, DWS, DWS,},
  { DWE, DWE, DWE,},
  { DWE, DWE, DWE,},
  { DWE, DWE, DWE,},
  { DWO, DWO, DWO,},
  { DWO, DWO, DWO,},
  });

const CtxSet ContextSetCfg::TmpFusion = ContextSetCfg::addCtxSet
({
  { CNU,CNU,CNU,CNU,CNU,CNU },
  { CNU,CNU,CNU,CNU,CNU,CNU },
  { CNU,CNU,CNU,CNU,CNU,CNU },
  { DWS,DWS,DWS,DWS,DWS,DWS },
  { DWS,DWS,DWS,DWS,DWS,DWS },
  { DWS,DWS,DWS,DWS,DWS,DWS },
  { DWE,DWE,DWE,DWE,DWE,DWE },
  { DWE,DWE,DWE,DWE,DWE,DWE },
  { DWE,DWE,DWE,DWE,DWE,DWE },
  { DWO,DWO,DWO,DWO,DWO,DWO },
  { DWO,DWO,DWO,DWO,DWO,DWO },
  });
#endif

#if MMLM
const CtxSet ContextSetCfg::MMLMFlag = ContextSetCfg::addCtxSet
({
  {  46, },
  {  46, },
  {  37, },
  {   5, },
  {   5, },
  {   5, },
  {   4, },
  {  18, },
  {  11, },
  { 109, },
  { 133, },
  });
#endif

const CtxSet ContextSetCfg::DeltaQP = ContextSetCfg::addCtxSet
({
  { 35, 35, },
  { 35, 35, },
  { 35, 35, },
  { 8,  8, },
  { 8,  8, },
  { 8,  8, },
  { DWE, DWE, },
  { DWE, DWE, },
  { DWE, DWE, },
  { 119, 119, },
  { 119, 119, },
  });

const CtxSet ContextSetCfg::InterDir = ContextSetCfg::addCtxSet
({
#if CTU_256
  {   7,  13,  12,   4,  18,   3,  10,   0, },
  {   7,   6,   5,   4,  11,  18,  10,  48, },
  { CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
  {   0,   0,   1,   1,   5,   5,   6,   1, },
  {   1,   1,   1,   1,   5,   9,   6,   1, },
  { DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, },
  {  18,  11,  11,   4,  18,  18,  18,  32, },
  {  32,  18,  11,   4,  11,  18,   4,  18, },
  { DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, },
  { 116, 116, 117, 117, 117, 103,  67, 228, },
  { 116, 148, 148, 124, 148, 148, 119, 116, },
#else
  {   7,  13,  12,   4,  18,   3,  10 },
  {   7,   6,   5,   4,  11,  18,  10 },
  { CNU, CNU, CNU, CNU, CNU, CNU, CNU },
  {   0,   0,   1,   1,   5,   5,   6 },
  {   1,   1,   1,   1,   5,   9,   6 },
  { DWS, DWS, DWS, DWS, DWS, DWS, DWS },
  {  18,  11,  11,   4,  18,  18,  18 },
  {  32,  18,  11,   4,  11,  18,   4 },
  { DWE, DWE, DWE, DWE, DWE, DWE, DWE },
  { 116, 116, 117, 117, 117, 103,  67 },
  { 116, 148, 148, 124, 148, 148, 119 },
#endif
  });

const CtxSet ContextSetCfg::RefPic = ContextSetCfg::addCtxSet
({
  {  20,  28, },
  {   5,  29, },
  { CNU, CNU, },
  {   1,   1, },
  {   1,   5, },
  { DWS, DWS, },
  {  11,   4, },
  {  18,  18, },
  { DWE, DWE, },
  { 122, 125, },
  { 124, 103, },
  });

#if JVET_Z0054_BLK_REF_PIC_REORDER
const CtxSet ContextSetCfg::RefPicLC = ContextSetCfg::addCtxSet
({
  { CNU, CNU, CNU },
  { CNU, CNU, CNU },
  { CNU, CNU, CNU },
  { DWS, DWS, DWS },
  { DWS, DWS, DWS },
  { DWS, DWS, DWS },
  { DWE, DWE, DWE },
  { DWE, DWE, DWE },
  { DWE, DWE, DWE },
  { DWO, DWO, DWO },
  { DWO, DWO, DWO },
  });
#endif

const CtxSet ContextSetCfg::SubblockMergeFlag = ContextSetCfg::addCtxSet
({
  {  25,  43,  30, },
  {  40,  34,  36, },
  { CNU, CNU, CNU, },
  {   6,   5,   5, },
  {   6,   5,   5, },
  { DWS, DWS, DWS, },
  {  18,  18,  18, },
  {  11,  18,  18, },
  { DWE, DWE, DWE, },
  { 121, 117, 118, },
  { 126, 133, 148, },
  });

#if JVET_X0049_ADAPT_DMVR
const CtxSet ContextSetCfg::BMMergeFlag = ContextSetCfg::addCtxSet
({
  {  48,  50,  50,  50, },
  {  56,  50,  43,  28, },
  { CNU, CNU, CNU, CNU, },
  {   5,   5,   9,   5, },
  {   5,   5,   5,   5, },
  { DWS, DWS, DWS, DWS, },
  {  18,  18,  25,  18, },
  {  18,  18,  11,  18, },
  { DWE, DWE, DWE, DWE, },
  { 126, 126, 181, 126, },
  { 117, 117, 110, 116, },
  });
#endif
#if JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
const CtxSet ContextSetCfg::affBMFlag = ContextSetCfg::addCtxSet
({
  {  48,  50, },
  {  56,  50, },
  { CNU, CNU, },
  {   5,   5, },
  {   5,   5, },
  { DWS, DWS, },
  {  18,  18, },
  {  18,  18, },
  { DWE, DWE, },
  { 126, 126, },
  { 117, 117, },
  });
#endif
#if JVET_AA0070_RRIBC
const CtxSet ContextSetCfg::rribcFlipType = ContextSetCfg::addCtxSet
({
  {  48,  50,  50,  50, },
  {  56,  50,  43,  28, },
  { CNU, CNU, CNU, CNU, },
  {   5,   5,   9,   5, },
  {   5,   5,   5,   5, },
  { DWS, DWS, DWS, DWS, },
  {  18,  18,  25,  18, },
  {  18,  18,  11,  18, },
  { DWE, DWE, DWE, DWE, },
  { 126, 126, 181, 126, },
  { 117, 117, 110, 116, },
  });
#endif

#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
const CtxSet ContextSetCfg::bvOneZeroComp = ContextSetCfg::addCtxSet
({
  {  48,  50,  50,  50, },
  {  56,  50,  43,  28, },
  { CNU, CNU, CNU, CNU, },
  {   5,   5,   9,   5, },
  {   5,   5,   5,   5, },
  { DWS, DWS, DWS, DWS, },
  {  18,  18,  25,  18, },
  {  18,  18,  11,  18, },
  { DWE, DWE, DWE, DWE, },
  { 126, 126, 181, 126, },
  { 117, 117, 110, 116, },
  });
#endif

const CtxSet ContextSetCfg::AffineFlag = ContextSetCfg::addCtxSet
({
  {  19,   6,   7, },
  {  19,   5,   6, },
  { CNU, CNU, CNU, },
  {   5,   1,   0, },
  {   5,   1,   1, },
  { DWS, DWS, DWS, },
  {  11,  11,   4, },
  {  11,  11,  18, },
  { DWE, DWE, DWE, },
  { 116, 119, 148, },
  { 117, 117, 116, },
  });

const CtxSet ContextSetCfg::AffineType = ContextSetCfg::addCtxSet
({
  {  19, },
  {  34, },
  { CNU, },
  {   1, },
  {   1, },
  { DWS, },
  {   4, },
  {   4, },
  { DWE, },
  { 102, },
  { 211, },
  });

#if JVET_AA0128_AFFINE_MERGE_CTX_INC
const CtxSet ContextSetCfg::AffMergeIdx = ContextSetCfg::addCtxSet
({
  {   4, CNU, CNU, },
  {   4, CNU, CNU, },
  { CNU, CNU, CNU, },
  {   1, DWS, DWS, },
  {   1, DWS, DWS, },
  { DWS, DWS, DWS, },
  {  18, DWE, DWE, },
  {  25, DWE, DWE, },
  { DWE, DWE, DWE, },
  { 118, DWO, DWO, },
  { 148, DWO, DWO, },
  });
#else
const CtxSet ContextSetCfg::AffMergeIdx = ContextSetCfg::addCtxSet
({
  {   4, },
  {   4, },
  { CNU, },
  {   1, },
  {   1, },
  { DWS, },
  {  18, },
  {  25, },
  { DWE, },
  { 118, },
  { 148, },
  });
#endif

#if INTER_LIC
const CtxSet ContextSetCfg::LICFlag = ContextSetCfg::addCtxSet
({
  {  12, },
  {   4, },
  { CNU, },
  {   5, },
  {   5, },
  { DWS, },
  {  11, },
  {   4, },
  { DWE, },
  { 132, },
  { 132, },
  });
#endif

const CtxSet ContextSetCfg::BcwIdx = ContextSetCfg::addCtxSet
({
  {   4, },
  {   5, },
  { CNU, },
  {   0, },
  {   0, },
  { DWS, },
  {   4, },
  {  11, },
  { DWE, },
  { 164, },
  { 118, },
  });

const CtxSet ContextSetCfg::Mvd = ContextSetCfg::addCtxSet
({
  {  44,  36, },
  {  44,  43, },
  {  21,  30, },
  {   6,   3, },
  {   6,   3, },
  {  13,   9, },
  {   4,  11, },
  {   4,  11, },
  {  25,   4, },
  { 141, 116, },
  { 126,  68, },
  });

#if JVET_Z0131_IBC_BVD_BINARIZATION
const CtxSet ContextSetCfg::Bvd = ContextSetCfg::addCtxSet
({
  { 53,  38,  38,  29,  20,  34,  27,  45,  37,  43,  34,  48  },
  { 38,  38,  38,  29,  28,  42,  27,  45,  44,  28,  42,  33  },
  { 38,  38,  38,  29,  28,  42,  27,  45,  44,  28,  42,  33  },
  {  1,  12,  8,   4,   2,   5,   3,   4,   0,   0,   5,   4   },
  {  6,  10,  9,   6,   7,   7,   5,   5,   4,   1,   2,   3   },
  {  6,  10,  9,   6,   7,   7,   5,   5,   4,   1,   2,   3   },
  { DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE },
  { DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE },
  { DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE },
  { DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO },
  { DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO }
});
#endif

#if JVET_AD0140_MVD_PREDICTION
const CtxSet ContextSetCfg::MvsdIdxMVDMSB = ContextSetCfg::addCtxSet
({
  {  34,  41,  49,  41,  34,  41,  49,  41,  34,  41,  49,  41,  34,  41,  49,  41,  34,  41,  49,  41,  34,  41,  49,  41,  34,  41,  49,  41,  34,  41,  49,  41},
  {  34,  41,  34,  41,  34,  41,  34,  41,  34,  41,  34,  41,  34,  41,  34,  41,  34,  41,  34,  41,  34,  41,  34,  41,  34,  41,  34,  41,  34,  41,  34,  41},
  { CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU},
  {  13,  13,  12,  12,  13,  13,  12,  12,  13,  13,  12,  12,  13,  13,  12,  12,  13,  13,  12,  12,  13,  13,  12,  12,  13,  13,  12,  12,  13,  13,  12,  12},
  {  13,  13,  12,  12,  13,  13,  12,  12,  13,  13,  12,  12,  13,  13,  12,  12,  13,  13,  12,  12,  13,  13,  12,  12,  13,  13,  12,  12,  13,  13,  12,  12},
  { DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS},
  {   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4},
  {   4,   4,  18,   4,   4,   4,  18,   4,   4,   4,  18,   4,   4,   4,  18,   4,   4,   4,  18,   4,   4,   4,  18,   4,   4,   4,  18,   4,   4,   4,  18,   4},
  { DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE},
  { 222,  83, 181,  94, 222,  83, 181,  94, 222,  83, 181,  94, 222,  83, 181,  94, 222,  83, 181,  94, 222,  83, 181,  94, 222,  83, 181,  94, 222,  83, 181,  94},
  { 100, 116, 100, 101, 100, 116, 100, 101, 100, 116, 100, 101, 100, 116, 100, 101, 100, 116, 100, 101, 100, 116, 100, 101, 100, 116, 100, 101, 100, 116, 100, 101},
});
#endif

#if JVET_AC0104_IBC_BVD_PREDICTION
const CtxSet ContextSetCfg::MvsdIdxBVDMSB = ContextSetCfg::addCtxSet
({
  {  34,  41,  49,  41,  34,  41,  49,  41,  34,  41,  49,  41, 34,   41,  49,  41, },
  {  34,  41,  34,  41,  34,  41,  34,  41,  34,  41,  34,  41, 34,   41,  34,  41, },
  { CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
  {  13,  13,  12,  12,  13,  13,  12,  12,  13,  13,  12,  12, 13,   13,  12,  12, },
  {  13,  13,  12,  12,  13,  13,  12,  12,  13,  13,  12,  12, 13,   13,  12,  12, },
  { DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, },
  {   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4, },
  {   4,   4,  18,   4,   4,   4,  18,   4,   4,   4,  18,   4,   4,   4,  18,   4, },
  { DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, },
  { 222,  83, 181,  94, 222,  83, 181,  94, 222,  83, 181,  94, 222,  83, 181,  94, },
  { 100, 116, 100, 101, 100, 116, 100, 101, 100, 116, 100, 101, 100, 116, 100, 101, },
  });
#endif


#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED || JVET_AC0104_IBC_BVD_PREDICTION
const CtxSet ContextSetCfg::MvsdIdx = ContextSetCfg::addCtxSet
({
  {  34,  41,  49,  41, },
  {  34,  41,  34,  41, },
  { CNU, CNU, CNU, CNU, },
  {  13,  13,  12,  12, },
  {  13,  13,  12,  12, },
  { DWS, DWS, DWS, DWS, },
  {   4,   4,   4,   4, },
  {   4,   4,  18,   4, },
  { DWE, DWE, DWE, DWE, },
  { 222,  83, 181,  94, },
  { 100, 116, 100, 101, },
  });
#endif


#if JVET_AD0140_MVD_PREDICTION && JVET_AC0104_IBC_BVD_PREDICTION
const CtxSet ContextSetCfg::MvsdIdxIBC = ContextSetCfg::addCtxSet
({
  {  34,  41 },
  {  34,  41 },
  { CNU, CNU },
  {  13,  13 },
  {  13,  13 },
  { DWS, DWS },
  {   4,   4 },
  {   4,   4 },
  { DWE, DWE },
  { 222,  83 },
  { 100, 116 },
  });
#endif

#if MULTI_HYP_PRED
const CtxSet ContextSetCfg::MultiHypothesisFlag = ContextSetCfg::addCtxSet
({
  {  17,  42,  51, },
  {  10,  42,  51, },
  { CNU, CNU, CNU, },
  {   2,   6,   5, },
  {   2,   5,   1, },
  { DWS, DWS, DWS, },
  {  11,  11,  18, },
  {  11,  11,   4, },
  { DWE, DWE, DWE, },
  { 116, 139, 118, },
  { 132, 109, 131, },
  });
#endif

const CtxSet ContextSetCfg::MHRefPic = ContextSetCfg::addCtxSet
({
  {  20,  43, },
  {  28,  19, },
  { CNU, CNU, },
  {   1,   1, },
  {   1,   0, },
  { DWS, DWS, },
  {  11,  11, },
  {  11,  11, },
  { DWE, DWE, },
  { 132, 148, },
  { 118, 117, },
  });

const CtxSet ContextSetCfg::MHWeight = ContextSetCfg::addCtxSet
({
  {  35, CNU, },
  {  35, CNU, },
  { CNU, CNU, },
  {   1, DWS, },
  {   1, DWS, },
  { DWS, DWS, },
  {  11, DWE, },
  {  11, DWE, },
  { DWE, DWE, },
  { 126, 119, },
  { 126, 119, },
  });

const CtxSet ContextSetCfg::BDPCMMode = ContextSetCfg::addCtxSet
({
  {  33,  14,   0,   4, },
  {   0,  21,   0,  12, },
  {  11,  50,   0,  27, },
  {   1,   1,   4,   3, },
  {   1,   4,   0,   8, },
  {   1,   4,   8,   3, },
  {  18,  25,  32,  32, },
  {   4,  32,  32,  32, },
  {  18,  25,  11,  25, },
  { 118, 133,  83, 156, },
  { 120, 132, 212, 147, },
  });

const CtxSet ContextSetCfg::QtRootCbf = ContextSetCfg::addCtxSet
({
  {   5, },
  {   5, },
  {   6, },
  {   1, },
  {   1, },
  {   6, },
  {   4, },
  {   4, },
  {  32, },
  { 118, },
  { 102, },
  });

const CtxSet ContextSetCfg::ACTFlag = ContextSetCfg::addCtxSet
({
  { 46, },
  { 46, },
  { 52, },
  { 1, },
  { 1, },
  { 1, },
  { DWE, },
  { DWE, },
  { DWE, },
  { 119, },
  { 119, },
  });

const CtxSet ContextSetCfg::QtCbf[] = 
{
  ContextSetCfg::addCtxSet
  ({
  {  23,  30,  13,  14, },
  {  31,   6,  13,   7, },
  {  15,  12,   5,   7, },
  {   6,   1,   9,   9, },
  {   6,   0,   5,   9, },
  {   5,   1,   5,   5, },
  {  18,  18,  18,  25, },
  {  18,  18,   4,  11, },
  {  18,  18,  11,   4, },
  { 117, 118, 100, 147, },
  { 116, 116, 179, 139, },
}),
ContextSetCfg::addCtxSet
({
  {  25,  38, },
  {  25,   5, },
  {  12,  14, },
  {   6,   2, },
  {   6,   0, },
  {   5,   3, },
  {  18,  32, },
  {  25,  32, },
  {  18,  32, },
  { 106, 117, },
  { 131, 132, },
}),
ContextSetCfg::addCtxSet
({
  {  17,  36,  22, },
  {  17,  36,  53, },
  {  26,  13,  37, },
  {   3,   2,  13, },
  {   2,   2,   2, },
  {   2,   2,   3, },
  {  18,  18,   4, },
  {  18,  18,  25, },
  {  18,  18,  32, },
  { 116, 118,  91, },
  { 121, 126, 130, },
}),
};

const CtxSet ContextSetCfg::SigCoeffGroup[] =
{
  ContextSetCfg::addCtxSet
  ({
  {  25,  38, },
  {  25,  45, },
  {  18,  31, },
  {   8,   5, },
  {   8,   5, },
  {   5,   5, },
  {  25,  18, },
  {  25,  18, },
  {  18,  18, },
  { 116, 124, },
  { 116, 119, },
}),
ContextSetCfg::addCtxSet
({
  {  17,  45, },
  {  25,  45, },
  {  25,   7, },
  {   5,   8, },
  {   9,  13, },
  {   5,   9, },
  {  18,  11, },
  {  11,  25, },
  {  18,  18, },
  {  93, 151, },
  { 116, 121, },
}),
};
#if JVET_AE0102_LFNST_CTX
const CtxSet ContextSetCfg::SigFlagL[] =
{
  ContextSetCfg::addCtxSet
  ({
  {  17,  41,  42,  29,   9,  42,  28,  37,  33,  44,  51,  45, },
  {  17,  41,  42,  29,  25,  42,  43,  37,  33,  44,  51,  30, },
  {  25,  19,  28,  14,  18,  28,  29,  30,  27,  45,  30,  38, },
  {  13,   9,   9,  10,   9,  10,   9,  10,   8,   9,   9,   9, },
  {  13,   9,   9,  10,   9,  10,  10,  10,   8,   9,   9,   9, },
  {  13,   9,   9,  10,  10,  10,  10,  13,   9,   9,  10,  10, },
  {  11,  11,  18,  18,  11,  18,  18,  11,  11,  18,  18,  11, },
  {  11,  11,  18,  18,  18,  18,  18,  11,  18,  18,  18,  11, },
  {  18,  11,  18,  11,  18,  11,  11,  11,  18,  18,  18,  11, },
  {  90, 119, 117, 121, 123, 119, 117, 126, 126, 132, 118, 132, },
  { 120, 133, 118, 125, 117, 126, 119, 120, 117, 117, 132, 108, },
}),
ContextSetCfg::addCtxSet
({
  {  25,  42,  43,  29,  41,  60,  60,  38, },
  {  18,  27,  28,  29,  34,  45,  45,  38, },
  {  25,  27,  28,  37,  27,  53,  53,  46, },
  {   8,  13,   9,  13,   5,   5,   8,  10, },
  {  13,  13,  13,  13,   5,   5,   8,   9, },
  {  12,  12,   9,  13,   5,   5,   9,   9, },
  {   4,  25,  11,  11,  18,  18,  18,  18, },
  {  18,  18,  18,  11,  11,  11,  18,  18, },
  {  18,  18,  11,  18,  11,  11,  11,  11, },
  { 119, 108, 119, 103, 125, 133, 132, 118, },
  { 109, 117, 126, 124, 119, 110, 126, 137, },
}),
ContextSetCfg::addCtxSet
({
  {  26,  45,  53,  46,  19,  54,  61,  39,  34,  39,  39,  39, },
  {  26,  38,  38,  46,  34,  54,  54,  39,  13,  39,  39,  39, },
  {  26,  38,  46,  54,  27,  39,  39,  39,  28,  39,  39,  39, },
  {   9,  13,  12,   8,   8,   8,   8,   4,   0,   0,   0,   0, },
  {   9,  13,  12,   8,   8,   8,   8,   5,   0,   0,   0,   0, },
  {   9,  13,  12,   8,   8,   8,   8,   4,   4,   0,   0,   0, },
  {  11,  11,  18,  18,  18,  25,  25,  18,   4,  32,  32,  32, },
  {  11,  11,  32,  11,  18,  25,  25,  25,  32,  32,  32,  32, },
  {  18,  11,  32,  11,  18,  32,  32,  18,  32,  32,  32,  32, },
  { 118, 238, 151, 158, 148, 182, 166, 142, 131, 168, 238, 138, },
  { 126, 236, 126,  91, 117, 131, 131,  98, 228, 116, 116, 116, },
}),
ContextSetCfg::addCtxSet
({
  {  41,  45,  38,  31,   4,  39,  39,  39, },
  {  34,  38,  53,  54,  44,  39,  39,  39, },
  {  19,  46,  38,  39,  44,  39,  39,  39, },
  {   8,  12,  12,   8,   4,   0,   0,   0, },
  {   8,  12,  12,   8,   4,   0,   0,   0, },
  {   8,  12,  12,   8,   0,   0,   0,   0, },
  {  18,  18,  25,  25,  25,  32,  32,  32, },
  {  11,  11,  25,  25,  25,  32,  32,  32, },
  {  18,  18,  18,  25,  32,  32,  32,  32, },
  { 132, 233, 221, 197, 116, 197, 214, 116, },
  { 117, 155,  98, 102, 149,  98, 116,  99, },
}),
ContextSetCfg::addCtxSet
({
  {  26,  54,  39,  39,  34,  39,  39,  39,   0,  39,  39,  39, },
  {  19,  39,  54,  39,  19,  39,  39,  39,  48,  39,  39,  39, },
  {  18,  39,  39,  39,  27,  39,  39,  39,   0,  39,  39,  39, },
  {   8,   8,   8,  12,   8,   4,   4,   8,   4,   0,   0,   0, },
  {   8,   8,   8,  12,   8,   4,   4,   8,   0,   0,   0,   0, },
  {   8,   8,  12,  12,   8,   0,   4,   4,   0,   0,   0,   0, },
  {  18,  25,  25,  32,  18,  18,  25,  32,  32,  32,  32,  32, },
  {  18,  25,  18,  32,  18,  18,  25,  32,  25,  32,  32,  32, },
  {  18,  25,  32,  32,  18,  25,  32,  32,  32,  32,  32,  32, },
  { 119, 190, 190, 171, 132, 155, 142, 139,  82, 117, 238, 229, },
  { 119, 115, 108,  92, 117,  99,  99,  82, 134, 116, 116, 116, },
}),
ContextSetCfg::addCtxSet
({
  {  26,  38,  54,  39,  26,  39,  39,  39, },
  {  34,  38,  62,  39,  26,  39,  39,  39, },
  {  11,  39,  39,  39,  26,  39,  39,  39, },
  {   8,  12,   8,   8,   0,   0,   0,   0, },
  {   8,   8,   8,   8,   0,   0,   0,   0, },
  {   8,   8,   8,   8,   4,   0,   0,   0, },
  {  25,  32,  25,  25,  18,  32,  32,  32, },
  {  18,  18,  25,  25,  18,  32,  32,  32, },
  {  18,  18,  25,  25,  25,  32,  32,  32, },
  { 227, 190, 168, 158, 115, 185, 229, 196, },
  { 117, 182, 118, 116, 134, 114, 114,  99, },
}),
};

const CtxSet ContextSetCfg::ParFlagL[] =
{
  ContextSetCfg::addCtxSet
  ({
  {  33,  40,  33,  26,  34,  42,  25,  33,  34,  34,  27,  25,  34,  42,  42,  35,  33,  27,  35,  42,  35, },
  {  33,  25,  33,  26,  34,  42,  25,  33,  34,  42,  27,  25,  34,  42,  42,  35,  26,  27,  42,  35,  35, },
  {  33,  25,  18,  26,  34,  27,  25,  26,  19,  42,  35,  33,  19,  27,  35,  35,  34,  42,  20,  43,  20, },
  {   8,   9,  13,  13,  13,  13,  13,  13,  13,  13,  13,  12,  13,  13,  13,  13,   9,  13,  13,  12,  13, },
  {   9,   9,   9,  12,  13,  13,  13,  13,  13,  13,  13,  12,  13,  13,  13,  13,  10,  13,  13,  13,  13, },
  {   8,   9,   9,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13, },
  {   4,  11,  11,  11,   4,   4,  11,  11,  11,  11,   4,  11,  18,  11,  11,   4,  11,  18,  11,  11,   4, },
  {  11,  11,   4,  11,   4,   4,  11,  11,  11,   4,   4,  11,  11,  11,   4,   4,  11,  11,  11,  11,   4, },
  {  11,   4,   4,   4,   4,   4,  11,  11,  11,  11,   4,  11,  11,  11,  11,   4,  11,  11,  11,  11,  11, },
  {  78,  90,  91, 102, 126, 126,  89, 119, 103, 158, 119, 171, 119, 126, 125, 123, 106, 135, 120, 119, 135, },
  { 133, 147, 117, 118, 126, 134, 120, 126, 140, 120, 140, 147, 126, 121, 133, 126, 147, 118, 126, 212, 119, },
}),
ContextSetCfg::addCtxSet
({
  {  33,  25,  26,  19,  42,  27,  33,  42,  35,  35,  35, },
  {  33,  25,  26,  19,  34,  27,  33,  42,  35,  35,  35, },
  {  33,  25,  26,  42,  19,  27,  26,  50,  35,  35,  35, },
  {   9,  13,  13,  13,  13,  13,   9,   9,  13,  13,  13, },
  {   8,  13,  13,  12,  13,  13,  13,  13,  13,  13,  13, },
  {  12,  13,  12,  12,  12,  13,  13,  13,  13,  12,  13, },
  {  11,  11,  18,  18,  11,   4,   4,   4,  18,  18,   4, },
  {   4,  11,  18,  11,  11,   4,  11,  18,  11,  18,   4, },
  {  11,  11,   4,  11,   4,   4,  11,  11,  11,  11,  11, },
  {  69, 123, 115, 100, 101, 119, 121, 117, 121, 117, 236, },
  { 198, 118, 120, 132, 137, 124, 116, 133, 138, 135, 104, },
}),
};

const CtxSet ContextSetCfg::GtxFlagL[] =
{
  ContextSetCfg::addCtxSet
  ({
  {  25,   0,  40,  25,  33,  26,   0,  17,  25,  33,  19,   9,  25,  26,  34,  20,  25,  18,  19,  20,  37, },
  {   1,   0,  17,  17,  25,  26,   0,   9,  25,  33,  34,   9,  25,  33,  34,  20,  25,  33,  19,  27,  29, },
  {  25,   1,  40,  25,  33,  19,   9,  25,  25,  18,  12,  17,  33,  26,  19,  13,  33,  19,  20,  28,  22, },
  {   1,   9,   9,   6,   6,   5,   9,   9,  10,   9,   6,   9,   9,   9,   5,   5,   6,   8,   5,   9,   9, },
  {   2,   8,   5,   9,   9,   5,  12,  12,   9,  13,   2,   9,   9,   9,   9,   5,   6,   5,   9,   9,   9, },
  {   4,   9,   6,   6,   6,   5,   9,  12,   9,   9,   6,  13,  10,  10,  10,  10,   6,   9,  10,  10,  10, },
  {  11,  11,  18,   4,   4,  18,   4,   4,  11,   4,  18,  11,  11,  18,   4,   4,  11,  18,  11,  18,  11, },
  {   4,   4,   4,   4,   4,  18,  11,   4,   4,  11,   4,   4,  11,  18,  18,   4,  11,   4,  18,  18,  11, },
  {  18,  11,   4,   4,   4,  18,  11,  18,  11,  11,  18,  18,  18,  18,  18,  18,  11,  18,  18,  18,  11, },
  {  99,  69,  83,  90,  78, 147,  68,  84, 124, 103, 116,  88, 105, 125, 117, 117, 105, 120, 121, 124, 125, },
  { 134, 163, 118, 117, 116, 117, 148, 119, 117, 131, 118, 117, 131, 117, 117, 115, 126, 116, 116, 116, 118,},
}),
ContextSetCfg::addCtxSet
({
  {  25,   9,  25,  33,  26,  12,  17,  33,  34,  28,  45, },
  {   1,   1,  25,  18,  11,  12,  17,  33,  19,  20,  22, },
  {  40,   1,  25,  18,  34,  12,  25,  34,  35,  36,  37, },
  {   1,   6,   9,   5,   5,   2,   6,   9,   8,   5,   8, },
  {   5,   8,   9,   5,   5,   2,   6,   9,   5,   5,   8, },
  {   2,   9,   9,   8,   8,   2,  10,   5,   8,   8,   9, },
  {  18,   4,   4,   4,   4,   4,   4,   4,  11,   4,  11, },
  {   4,   4,  11,  11,   4,   4,   4,   4,  11,  11,  11, },
  {  11,   4,  11,  18,  18,   4,  11,   4,  18,  18,  11, },
  {  99,  69,  76, 101, 100, 117,  99, 100, 102, 102, 116, },
  { 118, 117, 118, 119, 148, 116, 117, 119, 134, 141, 120, },
}),
ContextSetCfg::addCtxSet
({
  {   9,  17,  26,  27,  35,  21,  25,  34,  35,  36,  37,  33,  35,  36,  29,  30,  34,  36,  37,  45,  38, },
  {   1,  17,  26,  34,  35,  44,  25,  34,  35,  36,  37,  33,  20,  36,  29,  37,  34,  28,  37,  37,  38, },
  {  25,  25,  11,  27,  20,  29,  33,  12,  28,  21,  22,  34,  28,  29,  29,  30,  28,  29,  45,  30,  23, },
  {   9,   8,   6,   9,  10,   9,   9,  10,  13,  13,  10,   9,  10,   9,  10,  10,   6,   9,  10,  13,  13, },
  {   9,   9,   6,   9,  10,  10,   9,  10,  13,  13,  10,  10,  10,  10,  10,   9,   9,   9,  10,   9,  13, },
  {   9,   5,   6,  10,  10,  10,  13,  10,  13,  13,  13,   9,  10,  10,  10,  13,  10,  10,  10,  10,  13, },
  {  11,   4,   4,   4,   4,  18,  11,  11,  18,  11,   4,  18,  18,  11,  11,   4,  18,  11,  18,  18,  11, },
  {  11,  11,   4,   4,   4,  18,  11,  11,  18,  11,   4,  18,  18,  11,  11,   4,  18,  18,  11,  11,  11, },
  {  11,   4,   4,   4,   4,  18,  18,  11,  11,  11,  18,  11,  11,  11,  11,  11,  18,  18,  11,  11,  11, },
  {  88,  99, 107, 117, 117, 117,  92, 122, 117, 116, 110, 116, 123, 118, 122, 117, 116, 124, 126, 125, 124, },
  { 227, 117, 116, 100, 104, 117, 117, 126, 120, 156, 135, 118, 116, 124, 120, 102, 117, 116, 118, 116,  90, },
}),
ContextSetCfg::addCtxSet
({
  {   9,  25,  27,  36,  13,  37,  42,  37,  45,  38,  46, },
  {   9,  25,  35,  28,  21,  22,  35,  37,  30,  30,  23, },
  {  40,  41,  35,  36,  21,  37,  36,  37,  45,  38,  46, },
  {   9,   9,   9,   9,  13,  10,   5,   6,   5,   9,   9, },
  {   9,  13,  13,  13,  13,  10,   5,   9,   5,   5,   9, },
  {   9,  12,   9,   8,   8,   9,   9,  12,   8,  12,  13, },
  {  11,  11,  11,   4,  11,  11,  11,   4,   4,  18,  11, },
  {   4,  18,  18,  18,  18,  11,  11,  18,  11,  11,  11, },
  {  11,  18,  11,   4,   4,  11,  11,  18,   4,  18,  18, },
  {  74,  99, 109, 115, 115, 117, 117, 100, 164, 110, 118, },
  { 117, 115, 119, 138, 167, 121, 118, 221, 105, 164, 105, },
}),
};
#endif

const CtxSet ContextSetCfg::SigFlag[] =
{
  ContextSetCfg::addCtxSet
  ({
  {  17,  41,  42,  29,   9,  42,  28,  37,  33,  44,  51,  45, },
  {  17,  41,  42,  29,  25,  42,  43,  37,  33,  44,  51,  30, },
  {  25,  19,  28,  14,  18,  28,  29,  30,  27,  45,  30,  38, },
  {  13,   9,   9,  10,   9,  10,   9,  10,   8,   9,   9,   9, },
  {  13,   9,   9,  10,   9,  10,  10,  10,   8,   9,   9,   9, },
  {  13,   9,   9,  10,  10,  10,  10,  13,   9,   9,  10,  10, },
  {  11,  11,  18,  18,  11,  18,  18,  11,  11,  18,  18,  11, },
  {  11,  11,  18,  18,  18,  18,  18,  11,  18,  18,  18,  11, },
  {  18,  11,  18,  11,  18,  11,  11,  11,  18,  18,  18,  11, },
  {  90, 119, 117, 121, 123, 119, 117, 126, 126, 132, 118, 132, },
  { 120, 133, 118, 125, 117, 126, 119, 120, 117, 117, 132, 108, },
}),
ContextSetCfg::addCtxSet
({
  {  25,  42,  43,  29,  41,  60,  60,  38, },
  {  18,  27,  28,  29,  34,  45,  45,  38, },
  {  25,  27,  28,  37,  27,  53,  53,  46, },
  {   8,  13,   9,  13,   5,   5,   8,  10, },
  {  13,  13,  13,  13,   5,   5,   8,   9, },
  {  12,  12,   9,  13,   5,   5,   9,   9, },
  {   4,  25,  11,  11,  18,  18,  18,  18, },
  {  18,  18,  18,  11,  11,  11,  18,  18, },
  {  18,  18,  11,  18,  11,  11,  11,  11, },
  { 119, 108, 119, 103, 125, 133, 132, 118, },
  { 109, 117, 126, 124, 119, 110, 126, 137, },
}),
ContextSetCfg::addCtxSet
({
  {  26,  45,  53,  46,  19,  54,  61,  39,  34,  39,  39,  39, },
  {  26,  38,  38,  46,  34,  54,  54,  39,  13,  39,  39,  39, },
  {  26,  38,  46,  54,  27,  39,  39,  39,  28,  39,  39,  39, },
  {   9,  13,  12,   8,   8,   8,   8,   4,   0,   0,   0,   0, },
  {   9,  13,  12,   8,   8,   8,   8,   5,   0,   0,   0,   0, },
  {   9,  13,  12,   8,   8,   8,   8,   4,   4,   0,   0,   0, },
  {  11,  11,  18,  18,  18,  25,  25,  18,   4,  32,  32,  32, },
  {  11,  11,  32,  11,  18,  25,  25,  25,  32,  32,  32,  32, },
  {  18,  11,  32,  11,  18,  32,  32,  18,  32,  32,  32,  32, },
  { 118, 238, 151, 158, 148, 182, 166, 142, 131, 168, 238, 138, },
  { 126, 236, 126,  91, 117, 131, 131,  98, 228, 116, 116, 116, },
}),
ContextSetCfg::addCtxSet
({
  {  41,  45,  38,  31,   4,  39,  39,  39, },
  {  34,  38,  53,  54,  44,  39,  39,  39, },
  {  19,  46,  38,  39,  44,  39,  39,  39, },
  {   8,  12,  12,   8,   4,   0,   0,   0, },
  {   8,  12,  12,   8,   4,   0,   0,   0, },
  {   8,  12,  12,   8,   0,   0,   0,   0, },
  {  18,  18,  25,  25,  25,  32,  32,  32, },
  {  11,  11,  25,  25,  25,  32,  32,  32, },
  {  18,  18,  18,  25,  32,  32,  32,  32, },
  { 132, 233, 221, 197, 116, 197, 214, 116, },
  { 117, 155,  98, 102, 149,  98, 116,  99, },
}),
ContextSetCfg::addCtxSet
({
  {  26,  54,  39,  39,  34,  39,  39,  39,   0,  39,  39,  39, },
  {  19,  39,  54,  39,  19,  39,  39,  39,  48,  39,  39,  39, },
  {  18,  39,  39,  39,  27,  39,  39,  39,   0,  39,  39,  39, },
  {   8,   8,   8,  12,   8,   4,   4,   8,   4,   0,   0,   0, },
  {   8,   8,   8,  12,   8,   4,   4,   8,   0,   0,   0,   0, },
  {   8,   8,  12,  12,   8,   0,   4,   4,   0,   0,   0,   0, },
  {  18,  25,  25,  32,  18,  18,  25,  32,  32,  32,  32,  32, },
  {  18,  25,  18,  32,  18,  18,  25,  32,  25,  32,  32,  32, },
  {  18,  25,  32,  32,  18,  25,  32,  32,  32,  32,  32,  32, },
  { 119, 190, 190, 171, 132, 155, 142, 139,  82, 117, 238, 229, },
  { 119, 115, 108,  92, 117,  99,  99,  82, 134, 116, 116, 116, },
}),
ContextSetCfg::addCtxSet
({
  {  26,  38,  54,  39,  26,  39,  39,  39, },
  {  34,  38,  62,  39,  26,  39,  39,  39, },
  {  11,  39,  39,  39,  26,  39,  39,  39, },
  {   8,  12,   8,   8,   0,   0,   0,   0, },
  {   8,   8,   8,   8,   0,   0,   0,   0, },
  {   8,   8,   8,   8,   4,   0,   0,   0, },
  {  25,  32,  25,  25,  18,  32,  32,  32, },
  {  18,  18,  25,  25,  18,  32,  32,  32, },
  {  18,  18,  25,  25,  25,  32,  32,  32, },
  { 227, 190, 168, 158, 115, 185, 229, 196, },
  { 117, 182, 118, 116, 134, 114, 114,  99, },
}),
};

const CtxSet ContextSetCfg::ParFlag[] =
{
  ContextSetCfg::addCtxSet
  ({
  {  33,  40,  33,  26,  34,  42,  25,  33,  34,  34,  27,  25,  34,  42,  42,  35,  33,  27,  35,  42,  35, },
  {  33,  25,  33,  26,  34,  42,  25,  33,  34,  42,  27,  25,  34,  42,  42,  35,  26,  27,  42,  35,  35, },
  {  33,  25,  18,  26,  34,  27,  25,  26,  19,  42,  35,  33,  19,  27,  35,  35,  34,  42,  20,  43,  20, },
  {   8,   9,  13,  13,  13,  13,  13,  13,  13,  13,  13,  12,  13,  13,  13,  13,   9,  13,  13,  12,  13, },
  {   9,   9,   9,  12,  13,  13,  13,  13,  13,  13,  13,  12,  13,  13,  13,  13,  10,  13,  13,  13,  13, },
  {   8,   9,   9,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13,  13, },
  {   4,  11,  11,  11,   4,   4,  11,  11,  11,  11,   4,  11,  18,  11,  11,   4,  11,  18,  11,  11,   4, },
  {  11,  11,   4,  11,   4,   4,  11,  11,  11,   4,   4,  11,  11,  11,   4,   4,  11,  11,  11,  11,   4, },
  {  11,   4,   4,   4,   4,   4,  11,  11,  11,  11,   4,  11,  11,  11,  11,   4,  11,  11,  11,  11,  11, },
  {  78,  90,  91, 102, 126, 126,  89, 119, 103, 158, 119, 171, 119, 126, 125, 123, 106, 135, 120, 119, 135, },
  { 133, 147, 117, 118, 126, 134, 120, 126, 140, 120, 140, 147, 126, 121, 133, 126, 147, 118, 126, 212, 119, },
}),
ContextSetCfg::addCtxSet
({
  {  33,  25,  26,  19,  42,  27,  33,  42,  35,  35,  35, },
  {  33,  25,  26,  19,  34,  27,  33,  42,  35,  35,  35, },
  {  33,  25,  26,  42,  19,  27,  26,  50,  35,  35,  35, },
  {   9,  13,  13,  13,  13,  13,   9,   9,  13,  13,  13, },
  {   8,  13,  13,  12,  13,  13,  13,  13,  13,  13,  13, },
  {  12,  13,  12,  12,  12,  13,  13,  13,  13,  12,  13, },
  {  11,  11,  18,  18,  11,   4,   4,   4,  18,  18,   4, },
  {   4,  11,  18,  11,  11,   4,  11,  18,  11,  18,   4, },
  {  11,  11,   4,  11,   4,   4,  11,  11,  11,  11,  11, },
  {  69, 123, 115, 100, 101, 119, 121, 117, 121, 117, 236, },
  { 198, 118, 120, 132, 137, 124, 116, 133, 138, 135, 104, },
}),
};

const CtxSet ContextSetCfg::GtxFlag[] =
{
  ContextSetCfg::addCtxSet
  ({
  {  25,   0,  40,  25,  33,  26,   0,  17,  25,  33,  19,   9,  25,  26,  34,  20,  25,  18,  19,  20,  37, },
  {   1,   0,  17,  17,  25,  26,   0,   9,  25,  33,  34,   9,  25,  33,  34,  20,  25,  33,  19,  27,  29, },
  {  25,   1,  40,  25,  33,  19,   9,  25,  25,  18,  12,  17,  33,  26,  19,  13,  33,  19,  20,  28,  22, },
  {   1,   9,   9,   6,   6,   5,   9,   9,  10,   9,   6,   9,   9,   9,   5,   5,   6,   8,   5,   9,   9, },
  {   2,   8,   5,   9,   9,   5,  12,  12,   9,  13,   2,   9,   9,   9,   9,   5,   6,   5,   9,   9,   9, },
  {   4,   9,   6,   6,   6,   5,   9,  12,   9,   9,   6,  13,  10,  10,  10,  10,   6,   9,  10,  10,  10, },
  {  11,  11,  18,   4,   4,  18,   4,   4,  11,   4,  18,  11,  11,  18,   4,   4,  11,  18,  11,  18,  11, },
  {   4,   4,   4,   4,   4,  18,  11,   4,   4,  11,   4,   4,  11,  18,  18,   4,  11,   4,  18,  18,  11, },
  {  18,  11,   4,   4,   4,  18,  11,  18,  11,  11,  18,  18,  18,  18,  18,  18,  11,  18,  18,  18,  11, },
  {  99,  69,  83,  90,  78, 147,  68,  84, 124, 103, 116,  88, 105, 125, 117, 117, 105, 120, 121, 124, 125, },
  { 134, 163, 118, 117, 116, 117, 148, 119, 117, 131, 118, 117, 131, 117, 117, 115, 126, 116, 116, 116, 118,},
}),
ContextSetCfg::addCtxSet
({
  {  25,   9,  25,  33,  26,  12,  17,  33,  34,  28,  45, },
  {   1,   1,  25,  18,  11,  12,  17,  33,  19,  20,  22, },
  {  40,   1,  25,  18,  34,  12,  25,  34,  35,  36,  37, },
  {   1,   6,   9,   5,   5,   2,   6,   9,   8,   5,   8, },
  {   5,   8,   9,   5,   5,   2,   6,   9,   5,   5,   8, },
  {   2,   9,   9,   8,   8,   2,  10,   5,   8,   8,   9, },
  {  18,   4,   4,   4,   4,   4,   4,   4,  11,   4,  11, },
  {   4,   4,  11,  11,   4,   4,   4,   4,  11,  11,  11, },
  {  11,   4,  11,  18,  18,   4,  11,   4,  18,  18,  11, },
  {  99,  69,  76, 101, 100, 117,  99, 100, 102, 102, 116, },
  { 118, 117, 118, 119, 148, 116, 117, 119, 134, 141, 120, },
}),
ContextSetCfg::addCtxSet
({
  {   9,  17,  26,  27,  35,  21,  25,  34,  35,  36,  37,  33,  35,  36,  29,  30,  34,  36,  37,  45,  38, },
  {   1,  17,  26,  34,  35,  44,  25,  34,  35,  36,  37,  33,  20,  36,  29,  37,  34,  28,  37,  37,  38, },
  {  25,  25,  11,  27,  20,  29,  33,  12,  28,  21,  22,  34,  28,  29,  29,  30,  28,  29,  45,  30,  23, },
  {   9,   8,   6,   9,  10,   9,   9,  10,  13,  13,  10,   9,  10,   9,  10,  10,   6,   9,  10,  13,  13, },
  {   9,   9,   6,   9,  10,  10,   9,  10,  13,  13,  10,  10,  10,  10,  10,   9,   9,   9,  10,   9,  13, },
  {   9,   5,   6,  10,  10,  10,  13,  10,  13,  13,  13,   9,  10,  10,  10,  13,  10,  10,  10,  10,  13, },
  {  11,   4,   4,   4,   4,  18,  11,  11,  18,  11,   4,  18,  18,  11,  11,   4,  18,  11,  18,  18,  11, },
  {  11,  11,   4,   4,   4,  18,  11,  11,  18,  11,   4,  18,  18,  11,  11,   4,  18,  18,  11,  11,  11, },
  {  11,   4,   4,   4,   4,  18,  18,  11,  11,  11,  18,  11,  11,  11,  11,  11,  18,  18,  11,  11,  11, },
  {  88,  99, 107, 117, 117, 117,  92, 122, 117, 116, 110, 116, 123, 118, 122, 117, 116, 124, 126, 125, 124, },
  { 227, 117, 116, 100, 104, 117, 117, 126, 120, 156, 135, 118, 116, 124, 120, 102, 117, 116, 118, 116,  90, },
}),
ContextSetCfg::addCtxSet
({
  {   9,  25,  27,  36,  13,  37,  42,  37,  45,  38,  46, },
  {   9,  25,  35,  28,  21,  22,  35,  37,  30,  30,  23, },
  {  40,  41,  35,  36,  21,  37,  36,  37,  45,  38,  46, },
  {   9,   9,   9,   9,  13,  10,   5,   6,   5,   9,   9, },
  {   9,  13,  13,  13,  13,  10,   5,   9,   5,   5,   9, },
  {   9,  12,   9,   8,   8,   9,   9,  12,   8,  12,  13, },
  {  11,  11,  11,   4,  11,  11,  11,   4,   4,  18,  11, },
  {   4,  18,  18,  18,  18,  11,  11,  18,  11,  11,  11, },
  {  11,  18,  11,   4,   4,  11,  11,  18,   4,  18,  18, },
  {  74,  99, 109, 115, 115, 117, 117, 100, 164, 110, 118, },
  { 117, 115, 119, 138, 167, 121, 118, 221, 105, 164, 105, },
}),
};

const CtxSet ContextSetCfg::LastX[] =
{
  ContextSetCfg::addCtxSet
  ({
#if TU_256
  {  21,   6,  12,  14,   7,   4,  14,   7,   6,   4,  14,   7,  14,   6,   5,  14,   7,  14,  14,  22,  20,  21,  14,  29,   5, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
  {   6,  13,  12,   6,   6,  12,  14,   6,   5,  12,  29,  14,   6,   6,   6,  14,  14,   6,   6,  14,  53,  14,   6,  22,  54, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
  {  21,   5,   4,   6,   6,   4,   6,  14,  14,   4,  14,   7,  30,  14,   4,  22,  38,  15,  22,   6,   4,   7,  45,  22,   6, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
  {   5,   5,   5,   6,   5,   1,   6,   5,   2,   1,   6,   1,   1,   1,   1,   2,   2,   1,   1,   1,   0,   1,   4,   4,   0, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, },
  {   5,   5,   5,   6,   5,   1,   6,   5,   1,   0,   6,   1,   1,   1,   0,   1,   1,   1,   0,   0,   0,   1,   4,   0,   0, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, },
  {   9,   5,   5,   6,   5,   4,   5,   5,   1,   1,   5,   1,   1,   0,   0,   1,   1,   1,   0,   0,   0,   5,   4,   0,   0, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, },
  {  18,  18,  18,  18,  18,  11,  18,  18,  18,  18,  18,  11,  18,  25,  25,  11,  11,  18,  25,  32,  32,  18,  32,  32,  32, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, },
  {  18,  18,  18,  18,  18,  11,  18,  18,  18,  18,  18,  11,  18,  25,  32,  11,  11,  18,  18,  32,  32,  18,  18,  32,  32, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, },
  {  25,  18,  18,  18,  25,  25,  18,  25,  18,  32,  25,  18,  25,  18,  32,  18,  18,  25,  32,  32,  32,  32,  32,  32,  32, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, },
  { 116, 117, 118, 125, 119, 118, 122, 118, 126, 132, 118, 121, 118, 228, 116, 133, 126, 118, 148, 148, 116, 165, 237, 212, 116, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, },
  { 117, 118, 117, 132, 228, 103, 132, 228, 149, 117, 164, 149, 149, 229, 116, 126, 132, 227, 228, 228, 116, 132, 228, 228, 116, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, },
#else
  {  21,   6,  12,  14,   7,   4,  14,   7,   6,   4,  14,   7,  14,   6,   5,  14,   7,  14,  14,  22 },
  {   6,  13,  12,   6,   6,  12,  14,   6,   5,  12,  29,  14,   6,   6,   6,  14,  14,   6,   6,  14 },
  {  21,   5,   4,   6,   6,   4,   6,  14,  14,   4,  14,   7,  30,  14,   4,  22,  38,  15,  22,   6 },
  {   5,   5,   5,   6,   5,   1,   6,   5,   2,   1,   6,   1,   1,   1,   1,   2,   2,   1,   1,   1 },
  {   5,   5,   5,   6,   5,   1,   6,   5,   1,   0,   6,   1,   1,   1,   0,   1,   1,   1,   0,   0 },
  {   9,   5,   5,   6,   5,   4,   5,   5,   1,   1,   5,   1,   1,   0,   0,   1,   1,   1,   0,   0 },
  {  18,  18,  18,  18,  18,  11,  18,  18,  18,  18,  18,  11,  18,  25,  25,  11,  11,  18,  25,  32 },
  {  18,  18,  18,  18,  18,  11,  18,  18,  18,  18,  18,  11,  18,  25,  32,  11,  11,  18,  18,  32 },
  {  25,  18,  18,  18,  25,  25,  18,  25,  18,  32,  25,  18,  25,  18,  32,  18,  18,  25,  32,  32 },
  { 116, 117, 118, 125, 119, 118, 122, 118, 126, 132, 118, 121, 118, 228, 116, 133, 126, 118, 148, 148 },
  { 117, 118, 117, 132, 228, 103, 132, 228, 149, 117, 164, 149, 149, 229, 116, 126, 132, 227, 228, 228 },
#endif
}),
ContextSetCfg::addCtxSet
({
  {   4,  12,   3, },
  {  19,  26,  25, },
  {  12,   4,   3, },
  {   1,   4,   1, },
  {   6,   5,   5, },
  {   6,   4,   1, },
  {  11,  18,   4, },
  {  18,  18,  18, },
  {  18,  18,  11, },
  { 117, 118, 117, },
  { 132, 118, 117, },
}),
};

const CtxSet ContextSetCfg::LastY[] =
{
  ContextSetCfg::addCtxSet
  ({
#if TU_256
  {   5,   5,  20,   6,   6,   4,   6,  14,   5,  12,  14,   7,  13,   5,  20,  21,   7,   6,   5,  28,  43,  21,   6,  21,  57, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
  {   5,   5,   4,   6,   6,   4,  14,  14,   5,   4,  14,   7,  13,   5,  14,  21,   7,  13,  13,  14,  22,  14,   6,   6,  30, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
  {  13,   5,   4,  14,   6,  11,  14,  14,   5,  11,  14,   7,   6,   5,   3,  22,  38,  22,  14,   5,   4,  36,  30,   6,   4, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
  {   6,   5,   5,   6,   5,   5,   6,   6,   5,   1,   2,   6,   1,   1,   0,   2,   2,   1,   0,   0,   0,   1,   8,   4,   0, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, },
  {   5,   5,   5,   6,   6,   1,   6,   6,   5,   1,   6,   5,   1,   0,   0,   2,   2,   1,   0,   0,   0,   1,   8,   0,   0, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, },
  {   9,   5,   8,   6,   5,   4,   6,   5,   4,   0,   6,   6,   1,   4,   0,   1,   1,   1,   1,   0,   0,   0,   4,   4,   0, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, },
  {  18,  18,  18,  18,  18,  18,  18,  18,  18,  18,   4,  18,  11,  18,  25,  11,   4,  11,  11,  25,  32,  18,  32,  32,  32, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, },
  {   4,  11,  11,  18,  18,   4,  18,  18,  18,  18,  18,  18,  11,  18,  32,  18,   4,  18,  18,  32,  32,  18,  32,  32,  32, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, },
  {  18,  18,  18,  18,  18,  18,  18,  18,  18,  18,  25,  25,  18,  32,  32,  18,  18,  25,  32,  32,  32,  25,  32,  32,  32, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, },
  { 116, 120, 118, 119, 123, 118, 118, 124, 119, 132, 117, 116, 124, 134, 116, 126, 118, 119, 118, 196, 116, 196, 151, 232, 116, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, },
  { 116, 118, 118, 126, 228, 117, 116, 147, 230, 117, 132, 195, 165, 229, 116, 118, 123, 149, 228, 228, 116, 117, 230, 213, 116, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, },
#else
  {   5,   5,  20,   6,   6,   4,   6,  14,   5,  12,  14,   7,  13,   5,  20,  21,   7,   6,   5,  28 },
  {   5,   5,   4,   6,   6,   4,  14,  14,   5,   4,  14,   7,  13,   5,  14,  21,   7,  13,  13,  14 },
  {  13,   5,   4,  14,   6,  11,  14,  14,   5,  11,  14,   7,   6,   5,   3,  22,  38,  22,  14,   5 },
  {   6,   5,   5,   6,   5,   5,   6,   6,   5,   1,   2,   6,   1,   1,   0,   2,   2,   1,   0,   0 },
  {   5,   5,   5,   6,   6,   1,   6,   6,   5,   1,   6,   5,   1,   0,   0,   2,   2,   1,   0,   0 },
  {   9,   5,   8,   6,   5,   4,   6,   5,   4,   0,   6,   6,   1,   4,   0,   1,   1,   1,   1,   0 },
  {  18,  18,  18,  18,  18,  18,  18,  18,  18,  18,   4,  18,  11,  18,  25,  11,   4,  11,  11,  25 },
  {   4,  11,  11,  18,  18,   4,  18,  18,  18,  18,  18,  18,  11,  18,  32,  18,   4,  18,  18,  32 },
  {  18,  18,  18,  18,  18,  18,  18,  18,  18,  18,  25,  25,  18,  32,  32,  18,  18,  25,  32,  32 },
  { 116, 120, 118, 119, 123, 118, 118, 124, 119, 132, 117, 116, 124, 134, 116, 126, 118, 119, 118, 196 },
  { 116, 118, 118, 126, 228, 117, 116, 147, 230, 117, 132, 195, 165, 229, 116, 118, 123, 149, 228, 228 },
#endif
}),
ContextSetCfg::addCtxSet
({
  {  26,  20,  34, },
  {  26,  11,  25, },
  {  20,   4,   3, },
  {   2,   5,   0, },
  {   2,   6,   2, },
  {   6,   5,   2, },
  {  18,  32,  11, },
  {  11,  18,   4, },
  {  11,  18,   4, },
  { 117, 118, 117, },
  { 131, 148, 116, },
}),
};

const CtxSet ContextSetCfg::MVPIdx = ContextSetCfg::addCtxSet
({
  {  34, },
  {  34, },
  {  34, },
  {   6, },
  {   9, },
  {  12, },
  {   4, },
  {   4, },
  {   4, },
  { 122, },
  { 100, },
  });

#if JVET_X0083_BM_AMVP_MERGE_MODE
const CtxSet ContextSetCfg::amFlagState = ContextSetCfg::addCtxSet
({
  {  48, },
  {  41, },
  { CNU, },
  {   5, },
  {   5, },
  { DWS, },
  {  18, },
  {  11, },
  { DWE, },
  { 118, },
  { 118, },
  });
#endif

const CtxSet ContextSetCfg::SmvdFlag = ContextSetCfg::addCtxSet
({
  {  43, },
  {  28, },
  { CNU, },
  {   1, },
  {   4, },
  { DWS, },
  {  11, },
  {  11, },
  { DWE, },
  { 126, },
  { 102, },
  });

const CtxSet ContextSetCfg::SaoMergeFlag = ContextSetCfg::addCtxSet
({
  {  10, },
  {  61, },
  {  60, },
  {   1, },
  {   1, },
  {   0, },
  {  32, },
  {  18, },
  {  18, },
  { 116, },
  { 116, },
  });

const CtxSet ContextSetCfg::SaoTypeIdx = ContextSetCfg::addCtxSet
({
  {  10, },
  {   5, },
  {   5, },
  {   1, },
  {   8, },
  {   8, },
  {  18, },
  {  32, },
  {  32, },
  { 232, },
  { 197, },
  });

#if JVET_V0094_BILATERAL_FILTER
const CtxSet ContextSetCfg::BifCtrlFlags[] =
{
ContextSetCfg::addCtxSet
( {
  {  38, },
  {  38, },
  {  23, },
  {   6, },
  {   3, },
  {   3, },
  {   4, },
  {   4, },
  {  18, },
  { 115, },
  { 102, },
  } ),
#if JVET_X0071_CHROMA_BILATERAL_FILTER
  ContextSetCfg::addCtxSet
( {
  {  53, },
  {  37, },
  {  22, },
  {   2, },
  {   3, },
  {   1, },
  {  25, },
  {  18, },
  {  11, },
  { 116, },
  { 116, },
  } ),
  ContextSetCfg::addCtxSet
( {
  {  37, },
  {  30, },
  {  22, },
  {   3, },
  {   3, },
  {   2, },
  {  25, },
  {  18, },
  {  18, },
  { 116, },
  { 117, },
  } )
#endif
};
#endif

#if JVET_W0066_CCSAO
const CtxSet ContextSetCfg::CcSaoControlIdc = ContextSetCfg::addCtxSet
({
  {  11,  29,  38,  18,  29,  38,  18,  36,  45, },
  {   5,  30,  38,  19,  29,  45,  19,  21,  37, },
  {  50,  45,  38,  27,  37,  38,  35,  29,  30, },
  {   8,   0,   4,   5,   4,   4,   5,   4,   8, },
  {   5,   0,   4,   4,   4,   4,   4,   4,   4, },
  {   4,   0,   8,   0,   7,   4,   0,   2,   4, },
  {  18,   4,  11,   4,  18,  11,  11,  11,  32, },
  {  32,   4,  18,  18,  18,  18,  18,  18,  18, },
  {  32,   4,  32,  32,  32,  11,  25,  11,  18, },
  {  99, 165, 203,  99, 134, 237,  99, 148, 238, },
  { 198,  98,  99, 151,  98,  99, 141, 129,  99, },
  });
#endif

const CtxSet ContextSetCfg::LFNSTIdx = ContextSetCfg::addCtxSet
({
#if INTRA_TRANS_ENC_OPT
  {  51, CNU,  43,  42, },
  {  36, CNU,  36,  35, },
  { CNU,  51,  43,  42, },
  {  10, DWS,   5,  13, },
  {  10, DWS,   6,  13, },
  { DWS,  10,   5,  10, },
  {  11, DWE,  11,  11, },
  {  11, DWE,  11,  11, },
  { DWE,  11,   4,  11, },
  { 119, 126, 116, 116, },
  { 117, 117, 116, 125, },
#elif EXTENDED_LFNST || JVET_W0119_LFNST_EXTENSION
  { 58, 37, 42, 35 },
  { 43, 45, 42, 35 },
  { 28, 43, 42, 27 },
  {  9,  9,  9, 10 },
  {  9,  9,  6, 13 },
  {  9, 10,  9, 10 },
  {DWE, DWE, DWE, DWE},
  {DWE, DWE, DWE, DWE},
  {DWE, DWE, DWE, DWE},
  {DWO, DWO, DWO, DWO},
  {DWO, DWO, DWO, DWO},
#else
  { 58, 37, 42 },
  { 43, 45, 42 },
  { 28, 43, 42 },
  {  9,  9,  9 },
  {  9,  9,  6 },
  {  9, 10,  9 },
  {DWE, DWE, DWE},
  {DWE, DWE, DWE},
  {DWE, DWE, DWE},
  {DWO, DWO, DWO},
  {DWO, DWO, DWO},
#endif
  });

const CtxSet ContextSetCfg::PLTFlag = ContextSetCfg::addCtxSet
({
  { 17, },
  { 0, },
  { 25, },
  { 1, },
  { 1, },
  { 1, },
  { DWE, },
  { DWE, },
  { DWE, },
  { 119, },
  { 119, },
  });

const CtxSet ContextSetCfg::RotationFlag = ContextSetCfg::addCtxSet
({
  { 35, },
  { 42, },
  { 42, },
  { 5, },
  { 5, },
  { 5, },
  { DWE, },
  { DWE, },
  { DWE, },
  { 119, },
  { 119, },
  });

const CtxSet ContextSetCfg::RunTypeFlag = ContextSetCfg::addCtxSet
({
  { 50, },
  { 59, },
  { 42, },
  { 9, },
  { 9, },
  { 9, },
  { DWE, },
  { DWE, },
  { DWE, },
  { 119, },
  { 119, },
  });

const CtxSet ContextSetCfg::IdxRunModel = ContextSetCfg::addCtxSet
({
  { 58, 45, 45, 30, 38, },
  { 51, 30, 30, 38, 23, },
  { 50, 37, 45, 30, 46, },
  { 9,  6,  9, 10,  5, },
  { 9,  6,  9, 10,  5, },
  { 9,  6,  9, 10,  5, },
  { DWE, DWE, DWE, DWE, DWE, },
  { DWE, DWE, DWE, DWE, DWE, },
  { DWE, DWE, DWE, DWE, DWE, },
  { 119, 119, 119, 119, 119, },
  { 119, 119, 119, 119, 119, },
  });

const CtxSet ContextSetCfg::CopyRunModel = ContextSetCfg::addCtxSet
({
  { 45, 38, 46, },
  { 38, 53, 46, },
  { 45, 38, 46, },
  { 0,  9,  5, },
  { 0,  9,  5, },
  { 0,  9,  5, },
  { DWE, DWE, DWE, },
  { DWE, DWE, DWE, },
  { DWE, DWE, DWE, },
  { 119, 119, 119, },
  { 119, 119, 119, },
  });

const CtxSet ContextSetCfg::TransformSkipFlag = ContextSetCfg::addCtxSet
({
  {  25,   9, },
  {  25,   1, },
  {  25,   1, },
  {   2,   1, },
  {   2,   5, },
  {   1,   5, },
  {  18,  11, },
  {  18,  18, },
  {  11,  11, },
  { 119, 102, },
  { 147, 117, },
  });

const CtxSet ContextSetCfg::MTSIdx = ContextSetCfg::addCtxSet
({
  {  43,  46,  46, CNU, },
  {  36,  46,  46, CNU, },
  {  27,  38,  38, CNU, },
  {   8,   9,   9, DWS, },
  {  12,   8,   8, DWS, },
  {   9,   9,   5, DWS, },
  {  11,  11,  18, DWE, },
  {  18,  11,  18, DWE, },
  {  18,  18,   4, DWE, },
  { 104, 135, 133, DWO, },
  { 135,  99,  83, DWO, },
  });

const CtxSet ContextSetCfg::ISPMode = ContextSetCfg::addCtxSet
({
#if JVET_W0123_TIMD_FUSION
  {  26,  43,  33, },
  {  33,  43,  33, },
  {  33,  43,  33, },
  {   5,   2,   5, },
  {   6,   3,   8, },
  {   9,   1,   9, },
  {   4,  18,   4, },
  {  11,  11,  11, },
  {  18,  11,  18, },
  { 107, 119,  86, },
  { 116, 120, 133, },
#else
  {  26,  43 },
  {  33,  43 },
  {  33,  43 },
  {   5,   2 },
  {   6,   3 },
  {   9,   1 },
  {   4,  18 },
  {  11,  11 },
  {  18,  11 },
  { 107, 119 },
  { 116, 120 },
#endif
  });

const CtxSet ContextSetCfg::SbtFlag = ContextSetCfg::addCtxSet
({
  {  40,  49, },
  {  48,  49, },
  { CNU, CNU, },
  {   2,   6, },
  {   2,   6, },
  { DWS, DWS, },
  {  11,  18, },
  {  11,  11, },
  { DWE, DWE, },
  { 118, 110, },
  { 126, 126, },
  });

const CtxSet ContextSetCfg::SbtQuadFlag = ContextSetCfg::addCtxSet
({
  {  42, },
  {  42, },
  { CNU, },
  {  10, },
  {  10, },
  { DWS, },
  {  11, },
  {  11, },
  { DWE, },
  { 100, },
  { 133, },
  });

const CtxSet ContextSetCfg::SbtHorFlag = ContextSetCfg::addCtxSet
({
  {  20,  43,  20, },
  {  20,  58,  19, },
  { CNU, CNU, CNU, },
  {   5,   5,   2, },
  {   6,   5,   5, },
  { DWS, DWS, DWS, },
  {   4,  11,  11, },
  {  11,  11,  11, },
  { DWE, DWE, DWE, },
  { 117, 122, 119, },
  { 124, 117, 117, },
  });

const CtxSet ContextSetCfg::SbtPosFlag = ContextSetCfg::addCtxSet
({
  {  28, },
  {  28, },
  { CNU, },
  {  13, },
  {  13, },
  { DWS, },
  {  11, },
  {  11, },
  { DWE, },
  { 102, },
  { 110, },
  });

const CtxSet ContextSetCfg::ChromaQpAdjFlag = ContextSetCfg::addCtxSet
({
  { 35, },
  { 35, },
  { 35, },
  { 8, },
  { 8, },
  { 8, },
  { DWE, },
  { DWE, },
  { DWE, },
  { DWO, },
  { DWO, },
  });

#if ENABLE_DIMD
const CtxSet ContextSetCfg::DimdFlag = ContextSetCfg::addCtxSet
({
  {  40, CNU, CNU, },
  {  48, CNU, CNU, },
  {  25, CNU, CNU, },
  {   6, DWS, DWS, },
  {   6, DWS, DWS, },
  {   3, DWS, DWS, },
  {  11, DWE, DWE, },
  {  11, DWE, DWE, },
  {  11, DWE, DWE, },
  { 107, DWO, DWO, },
  { 131, DWO, DWO, },
  });
#endif

#if JVET_W0123_TIMD_FUSION
const CtxSet ContextSetCfg::TimdFlag = ContextSetCfg::addCtxSet
({
  {  41,  34,  42, },
  {  34,  34,  34, },
  {  42,  50,  58, },
  {   6,   6,   6, },
  {   7,   7,   5, },
  {   6,   6,   2, },
  {  11,  11,  18, },
  {   4,   4,   4, },
  {  11,  11,  11, },
  { 124, 126, 126, },
  { 126, 124, 117, },
  });
#endif

#if JVET_AB0155_SGPM
const CtxSet ContextSetCfg::SgpmFlag = ContextSetCfg::addCtxSet
({
  {  41,  34,  42, },
  {  34,  34,  34, },
  {  42,  50,  58, },
  {   6,   6,   6, },
  {   7,   7,   5, },
  {   6,   6,   2, },
  {  11,  11,  18, },
  {   4,   4,   4, },
  {  11,  11,  11, },
  { 124, 126, 126, },
  { 126, 124, 117, },
});
#endif
#if ENABLE_OBMC 
const CtxSet ContextSetCfg::ObmcFlag = ContextSetCfg::addCtxSet
({
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
  {  39,  53, },
  {  39,  53, },
  { CNU, CNU, },
  {   1,   7, },
  {   1,   4, },
  { DWS, DWS, },
  {  32,  25, },
  {  32,   4, },
  { DWE, DWE, },
  { 115, 102, },
  {  98,  68, },
#else
  {  39, },
  {  39, },
  { CNU, },
  {   1, },
  {   1, },
  { DWS, },
  {  32, },
  {  32, },
  { DWE, },
  { 115, },
  {  98, },
#endif
  });
#endif

const CtxSet ContextSetCfg::ChromaQpAdjIdc = ContextSetCfg::addCtxSet
({
  { 35, },
  { 35, },
  { 35, },
  { 8, },
  { 8, },
  { 8, },
  { DWE, },
  { DWE, },
  { DWE, },
  { DWO, },
  { DWO, },
  });

const CtxSet ContextSetCfg::ImvFlag = ContextSetCfg::addCtxSet
({
  {  59,  33,  50,  59,  53, },
  {  59,  33,  50,  59,  60, },
  { CNU,  34, CNU, CNU, CNU, },
  {   1,   5,   1,   0,   4, },
  {   1,   5,   1,   0,   5, },
  { DWS,   6, DWS, DWS, DWS, },
  {  11,  18,  11,  32,   4, },
  {  11,  11,  11,  32,  11, },
  { DWE,   4, DWE, DWE, DWE, },
  { 126,  92, 126, 116, 118, },
  { 117, 147, 117, 116, 119, },
  });

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
const CtxSet ContextSetCfg::ImvFlagIBC = ContextSetCfg::addCtxSet
({
  {  59,  33,  50,  59,  53, },
  {  59,  33,  50,  59,  60, },
  {  59,  33,  50,  59,  53, },
  {   1,   5,   1,   0,   4, },
  {   1,   5,   1,   0,   5, },
  {   1,   5,   1,   0,   4, },
  {  11,  18,  11,  32,   4, },
  {  11,  11,  11,  32,  11, },
  {  11,  18,  11,  32,   4, },
  { 126,  92, 126, 116, 118, },
  { 117, 147, 117, 116, 119, },
});
#endif

const CtxSet ContextSetCfg::ctbAlfFlag = ContextSetCfg::addCtxSet
({
  {  11,  23,  46,  18,  46,  54,  18,  46,  54, },
  {   6,  15,  31,  12,  46,  54,   5,  46,  54, },
  {  39,  39,  39,  31,  39,  39,  46,  39,  39, },
  {   8,   4,   5,  12,   0,   2,  12,   0,   5, },
  {   4,   4,   4,   4,   4,   4,   4,   0,   4, },
  {   0,   2,   1,   0,   6,   8,   0,   1,   8, },
  {  11,  25,  25,  32,  11,   4,  32,  11,  11, },
  {  18,  25,  18,  25,  32,  18,  32,  18,  18, },
  {  32,  32,  18,  32,  18,  25,  32,   4,  18, },
  {  85, 133, 135, 115, 212, 139,  85, 164, 136, },
  { 153, 237,  64, 167, 132,  64, 165, 227,  64, },
  });

const CtxSet ContextSetCfg::ctbAlfAlternative = ContextSetCfg::addCtxSet
({
#if ALF_IMPROVEMENT
  {  27,  19,  35, },
  {  20,  27,  27, },
  {  19,  27,  19, },
  {   1,   1,   1, },
  {   1,   1,   1, },
  {   0,   0,   0, },
  {  32,  32,  32, },
  {  25,  32,  32, },
  {  32,  18,  18, },
  { 231, 230, 229, },
  { 116, 116, 116, },
#else
  {  27,  19 },
  {  20,  27 },
  {  19,  27 },
  {   1,   1 },
  {   1,   1 },
  {   0,   0 },
  {  32,  32 },
  {  25,  32 },
  {  32,  18 },
  { 231, 230 },
  { 116, 116 },
#endif
  });

const CtxSet ContextSetCfg::AlfUseTemporalFilt = ContextSetCfg::addCtxSet
({
  {  39, },
  {  39, },
  {  39, },
  {   1, },
  {   1, },
  {   5, },
  {  11, },
  {   4, },
  {   4, },
  {  93, },
  {  67, },
  });

const CtxSet ContextSetCfg::CcAlfFilterControlFlag = ContextSetCfg::addCtxSet
({
  {  33,  44,  46,  18,  44,  46, },
  {   3,  37,  46,   3,  45,  46, },
  {  18,  37,  46,  10,  37,  46, },
  {   4,   1,   5,   4,   2,   8, },
  {   1,   1,   1,   4,   2,   4, },
  {   4,   3,   4,   4,   3,   5, },
  {   4,  18,   4,  11,  25,  18, },
  {   4,  18,   4,  11,  25,  11, },
  {  32,  11,  25,  18,  18,  25, },
  {  99, 117, 122, 100, 130, 135, },
  { 142, 130,  99, 135, 130,  99, },
  });

const CtxSet ContextSetCfg::CiipFlag = ContextSetCfg::addCtxSet
({
#if CIIP_PDPC
  {  56,  43, },
  {  57,  43, },
  { CNU, CNU, },
  {   1,   2, },
  {   2,   3, },
  { DWS, DWS, },
  {  11,  11, },
  {  11,  11, },
  { DWE, DWE, },
  { 126, 126, },
  { 117, 126, },
#else
  {  56 },
  {  57 },
  { CNU },
  {   1 },
  {   2 },
  { DWS },
  {  11 },
  {  11 },
  { DWE },
  { 126 },
  { 117 },
#endif
  });

const CtxSet ContextSetCfg::IBCFlag = ContextSetCfg::addCtxSet
({
  {  25,  43,  45, },
  {   0,  19,  36, },
  {  40,  27,  36, },
  {   5,   5,   6, },
  {   4,   7,   7, },
  {   6,   6,   9, },
  {  25,  18,  32, },
  {  18,  11,  18, },
  {  25,  18,  25, },
  { 117, 124, 119, },
  { 147, 117, 146, },
  });

#if JVET_AE0169_BIPREDICTIVE_IBC
const CtxSet ContextSetCfg::BiPredIbcFlag = ContextSetCfg::addCtxSet
({
  {   35, 35, },
  {   35, 35, },
  {   35, 35, },
  {    8,  8, },
  {    8,  8, },
  {    8,  8, },
  {   18, 18, },
  {   18, 18, },
  {   18, 18, },
  {  119,119, },
  {  119,119, },
  });
#endif

#if JVET_AC0112_IBC_CIIP
const CtxSet ContextSetCfg::IbcCiipFlag = ContextSetCfg::addCtxSet
({
  {  CNU, CNU, CNU, },
  {  CNU, CNU, CNU, },
  {  CNU, CNU, CNU, },
  {  DWS, DWS, DWS, },
  {  DWS, DWS, DWS, },
  {  DWS, DWS, DWS, },
  {  DWE, DWE, DWE, },
  {  DWE, DWE, DWE, },
  {  DWE, DWE, DWE, },
  {  DWO, DWO, DWO, },
  {  DWO, DWO, DWO, },
  });

const CtxSet ContextSetCfg::IbcCiipIntraIdx = ContextSetCfg::addCtxSet
({
  {  CNU, },
  {  CNU, },
  {  CNU, },
  {  DWS, },
  {  DWS, },
  {  DWS, },
  {  DWE, },
  {  DWE, },
  {  DWE, },
  {  DWO, },
  {  DWO, },
  });
#endif

#if JVET_AC0112_IBC_GPM
const CtxSet ContextSetCfg::IbcGpmFlag = ContextSetCfg::addCtxSet
({
  { CNU, },
  { CNU, },
  { CNU, },
  { DWS, },
  { DWS, },
  { DWS, },
  { DWE, },
  { DWE, },
  { DWE, },
  { DWO, },
  { DWO, },
  });

#if JVET_AE0169_GPM_IBC_IBC
const CtxSet ContextSetCfg::IbcGpmIntraFlag = ContextSetCfg::addCtxSet
({
  { CNU, CNU},
  { CNU, CNU},
  { CNU, CNU},
  { DWS, DWS},
  { DWS, DWS},
  { DWS, DWS},
  { DWE, DWE},
  { DWE, DWE},
  { DWE, DWE},
  { DWO, DWO},
  { DWO, DWO},
  });
#else
const CtxSet ContextSetCfg::IbcGpmIntraFlag = ContextSetCfg::addCtxSet
({
  { CNU, },
  { CNU, },
  { CNU, },
  { DWS, },
  { DWS, },
  { DWS, },
  { DWE, },
  { DWE, },
  { DWE, },
  { DWO, },
  { DWO, },
  });
#endif

const CtxSet ContextSetCfg::IbcGpmSplitDirSetFlag = ContextSetCfg::addCtxSet
({
  { CNU, },
  { CNU, },
  { CNU, },
  { DWS, },
  { DWS, },
  { DWS, },
  { DWE, },
  { DWE, },
  { DWE, },
  { DWO, },
  { DWO, },
  });

const CtxSet ContextSetCfg::IbcGpmBldIdx = ContextSetCfg::addCtxSet
({
  { CNU, CNU, CNU, CNU },
  { CNU, CNU, CNU, CNU },
  { CNU, CNU, CNU, CNU },
  { DWS, DWS, DWS, DWS },
  { DWS, DWS, DWS, DWS },
  { DWS, DWS, DWS, DWS },
  { DWE, DWE, DWE, DWE },
  { DWE, DWE, DWE, DWE },
  { DWE, DWE, DWE, DWE },
  { DWO, DWO, DWO, DWO },
  { DWO, DWO, DWO, DWO },
});
#endif

#if JVET_AC0112_IBC_LIC
const CtxSet ContextSetCfg::IbcLicFlag = ContextSetCfg::addCtxSet
({
#if JVET_AE0159_FIBC
  { CNU, CNU, CNU, CNU, },
  { CNU, CNU, CNU, CNU, },
  {  27,  25,  26,  12, },
  { DWS, DWS, DWS, DWS, },
  { DWS, DWS, DWS, DWS, },
  {   1,   6,   2,   0, },
  { DWE, DWE, DWE, DWE, },
  { DWE, DWE, DWE, DWE, },
  {  32,  32,  32,  32, },
  { DWO, DWO, DWO, DWO, },
  { DWO, DWO, DWO, DWO, },
#else
  { CNU, },
  { CNU, },
  { CNU, },
  { DWS, },
  { DWS, },
  { DWS, },
  { DWE, },
  { DWE, },
  { DWE, },
  { DWO, },
  { DWO, },
#endif
  });
#if JVET_AE0078_IBC_LIC_EXTENSION
const CtxSet ContextSetCfg::IbcLicIndex = ContextSetCfg::addCtxSet
({
  { CNU, CNU, },
  { CNU, CNU, },
  { CNU, CNU, },
  { DWS, DWS, },
  { DWS, DWS, },
  { DWS, DWS, },
  { DWE, DWE, },
  { DWE, DWE, },
  { DWE, DWE, },
  { DWO, DWO, },
  { DWO, DWO, },
  });
#endif
#endif

const CtxSet ContextSetCfg::JointCbCrFlag = ContextSetCfg::addCtxSet
({
  {  34,  28,  52, },
  {  27,  36,  52, },
  {  20,  29,  58, },
  {   1,   0,   1, },
  {   1,   0,   2, },
  {   1,   1,   2, },
  {  18,  18,  25, },
  {  18,  18,  25, },
  {  18,  18,  32, },
  { 117, 132, 117, },
  { 120, 117, 117, },
  });

const CtxSet ContextSetCfg::TsSigCoeffGroup = ContextSetCfg::addCtxSet
({
  {  18,  20,  37, },
  {  18,  27,  29, },
  {  18,  20,  38, },
  {   6,   9,   5, },
  {   7,   9,   5, },
  {   5,  10,  10, },
  {  11,  18,  18, },
  {  18,   4,   4, },
  {  18,  25,  25, },
  { 117, 125, 133, },
  { 227, 228, 123, },
  });

const CtxSet ContextSetCfg::TsSigFlag = ContextSetCfg::addCtxSet
({
  {  25,  35,  37, },
  {  40,  35,  44, },
  {  25,  28,  38, },
  {  13,  13,   5, },
  {  13,  13,   6, },
  {  13,  13,  10, },
  {   4,  18,  18, },
  {   4,  11,  11, },
  {   4,  11,  18, },
  {  90, 189, 110, },
  { 104, 238, 119, },
  });

const CtxSet ContextSetCfg::TsParFlag = ContextSetCfg::addCtxSet
({
  {   3, },
  {  10, },
  {  11, },
  {   5, },
  {   3, },
  {   6, },
  {  18, },
  {   4, },
  {  11, },
  { 116, },
  { 110, },
  });

const CtxSet ContextSetCfg::TsGtxFlag = ContextSetCfg::addCtxSet
({
  { CNU,  10,   3,   3,   4, },
  { CNU,  17,  10,   3,   3, },
  { CNU,  10,   3,   3,   3, },
  { DWS,   1,   2,   1,   3, },
  { DWS,   3,   2,   1,   0, },
  { DWS,   1,   1,   1,   1, },
  { DWE,  18,  25,  25,  32, },
  { DWE,  11,   4,   4,  11, },
  { DWE,  18,  18,  18,  18, },
  { DWO, 132, 226, 227, 227, },
  { DWO, 118, 118, 118, 116, },
  });

const CtxSet ContextSetCfg::TsLrg1Flag = ContextSetCfg::addCtxSet
({
  {  11,  11,   4,  14, },
  {  25,  11,   4,  21, },
  {   4,   5,  13,  14, },
  {   1,   1,   1,   2, },
  {   6,   3,   2,   2, },
  {   6,   2,   2,   7, },
  {  11,  18,  18,  18, },
  {  11,  18,  11,  11, },
  {  25,  18,  18,  18, },
  { 116, 117, 118, 145, },
  { 118, 117, 116, 102, },
  });

const CtxSet ContextSetCfg::TsResidualSign = ContextSetCfg::addCtxSet
({
  {   5,  10,  61,  28,  33,  38, },
  {  12,  10,  53,  27,  25,  46, },
  {   5,   2,  46,  28,  25,  46, },
  {   1,   5,   5,   7,   5,   8, },
  {   5,   6,   5,   2,   7,   7, },
  {   1,   5,   6,   5,   9,   8, },
  {  11,  18,  18,  18,  25,  32, },
  {  11,  18,  18,  18,  11,   4, },
  {  18,  32,  25,  11,  32,  25, },
  { 118, 164, 117, 226, 227, 132, },
  { 117, 117, 116, 115, 118, 195, },
  });

#if SIGN_PREDICTION
const CtxSet ContextSetCfg::signPred[] =
{
  ContextSetCfg::addCtxSet
  ({
  {  34,  34,  34,  26, },
  {  34,  34,  19,  26, },
  {  34,  34,  34,  18, },
  {  13,  10,  10,  10, },
  {  13,  13,  13,  10, },
  {  13,  10,   7,   7, },
  {  11,   4,   4,  11, },
  {   4,   4,  11,  11, },
  {  11,  11,   4,   4, },
  { 110, 102, 104, 100, },
  { 110, 110, 116, 116, },
}),
ContextSetCfg::addCtxSet
({
  {  34,  34,  34,  26, },
  {  41,  41,  26,  41, },
  {  34,  34, CNU, CNU, },
  {  13,  12,  10,   9, },
  {  13,  13,  10,   9, },
  {  10,  10, DWS, DWS, },
  {  11,  11,  11,  11, },
  {   4,   4,   4,   4, },
  {   4,   4, DWE, DWE, },
  { 100,  91,  99,  98, },
  { 116, 110, 117, 125, },
}),
};
#endif

#if JVET_Z0050_CCLM_SLOPE
const CtxSet ContextSetCfg::CclmDeltaFlags = ContextSetCfg::addCtxSet
({
  {  CNU, CNU, CNU, CNU, CNU, },
  {  CNU, CNU, CNU, CNU, CNU, },
  {  CNU, CNU, CNU, CNU, CNU, },
  {  DWS, DWS, DWS, DWS, DWS, },
  {  DWS, DWS, DWS, DWS, DWS, },
  {  DWS, DWS, DWS, DWS, DWS, },
  {  DWE, DWE, DWE, DWE, DWE, },
  {  DWE, DWE, DWE, DWE, DWE, },
  {  DWE, DWE, DWE, DWE, DWE, },
  {  DWO, DWO, DWO, DWO, DWO, },
  {  DWO, DWO, DWO, DWO, DWO, },
});
#endif

#if JVET_AA0126_GLM
const CtxSet ContextSetCfg::GlmFlags = ContextSetCfg::addCtxSet
({
  {  CNU, CNU, CNU, CNU, CNU, },
  {  CNU, CNU, CNU, CNU, CNU, },
  {  CNU, CNU, CNU, CNU, CNU, },
  {  DWS, DWS, DWS, DWS, DWS, },
  {  DWS, DWS, DWS, DWS, DWS, },
  {  DWS, DWS, DWS, DWS, DWS, },
  {  DWE, DWE, DWE, DWE, DWE, },
  {  DWE, DWE, DWE, DWE, DWE, },
  {  DWE, DWE, DWE, DWE, DWE, },
  {  DWO, DWO, DWO, DWO, DWO, },
  {  DWO, DWO, DWO, DWO, DWO, },
});
#endif

#if JVET_AA0057_CCCM
const CtxSet ContextSetCfg::CccmFlag = ContextSetCfg::addCtxSet
({
#if JVET_AC0147_CCCM_NO_SUBSAMPLING && JVET_AC0054_GLCCCM
  { CNU, CNU, CNU, },
  { CNU, CNU, CNU, },
  { CNU, CNU, CNU, },
  { DWS, DWS, DWS, },
  { DWS, DWS, DWS, },
  { DWS, DWS, DWS, },
  { DWE, DWE, DWE, },
  { DWE, DWE, DWE, },
  { DWE, DWE, DWE, },
  { DWO, DWO, DWO, },
  { DWO, DWO, DWO, },
#elif JVET_AC0147_CCCM_NO_SUBSAMPLING || JVET_AC0054_GLCCCM
  { CNU, CNU, },
  { CNU, CNU, },
  { CNU, CNU, },
  { DWS, DWS, },
  { DWS, DWS, },
  { DWS, DWS, },
  { DWE, DWE, },
  { DWE, DWE, },
  { DWE, DWE, },
  { DWO, DWO, },
  { DWO, DWO, },
#else
  { CNU, },
  { CNU, },
  { CNU, },
  { DWS, },
  { DWS, },
  { DWS, },
  { DWE, },
  { DWE, },
  { DWE, },
  { DWO, },
  { DWO, },
#endif
});

#if JVET_AD0202_CCCM_MDF
const CtxSet ContextSetCfg::CccmMpfFlag = ContextSetCfg::addCtxSet
({
  { CNU, CNU, CNU, },
  { CNU, CNU, CNU, },
  { CNU, CNU, CNU, },
  { DWS, DWS, DWS, },
  { DWS, DWS, DWS, },
  { DWS, DWS, DWS, },
  { DWE, DWE, DWE, },
  { DWE, DWE, DWE, },
  { DWE, DWE, DWE, },
  { DWO, DWO, DWO, },
  { DWO, DWO, DWO, },
});
#endif
#if JVET_AE0100_BVGCCCM
const CtxSet ContextSetCfg::BvgCccmFlag = ContextSetCfg::addCtxSet
({
  { CNU, },
  { CNU, },
  { CNU, },
  { DWS, },
  { DWS, },
  { DWS, },
  { DWE, },
  { DWE, },
  { DWE, },
  { DWO, },
  { DWO, },
});
#endif
#endif

#if JVET_AD0120_LBCCP
const CtxSet ContextSetCfg::CcInsideFilterFlag = ContextSetCfg::addCtxSet
({
  { CNU, },
  { CNU, },
  { CNU, },
  { DWS, },
  { DWS, },
  { DWS, },
  { DWE, },
  { DWE, },
  { DWE, },
  { DWO, },
  { DWO, },
});
#endif

#if JVET_AC0119_LM_CHROMA_FUSION
const CtxSet ContextSetCfg::ChromaFusionType = ContextSetCfg::addCtxSet
({
  { CNU, },
  { CNU, },
  { CNU, },
  { DWS, },
  { DWS, },
  { DWS, },
  { DWE, },
  { DWE, },
  { DWE, },
  { DWO, },
  { DWO, },
  });
const CtxSet ContextSetCfg::ChromaFusionCclm = ContextSetCfg::addCtxSet
({
  { CNU, },
  { CNU, },
  { CNU, },
  { DWS, },
  { DWS, },
  { DWS, },
  { DWE, },
  { DWE, },
  { DWE, },
  { DWO, },
  { DWO, },
  });
#endif

#if JVET_AB0157_TMRL
const CtxSet ContextSetCfg::TmrlDerive = ContextSetCfg::addCtxSet
({
  { CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
  { CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
  {  49,  50,  43,  43, CNU, CNU,  50, CNU, },
  {   8,   8,   9,   9,   9,   8,   9, DWS, },
  {   8,   8,   9,   9,   9,   8,   9, DWS, },
  {   8,   8,   9,   9,   9,   8,   9, DWS, },
  {  18, DWE, DWE, DWE, DWE, DWE, DWE, DWE, },
  {  18, DWE, DWE, DWE, DWE, DWE, DWE, DWE, },
  {  18, DWE, DWE, DWE, DWE, DWE, DWE, DWE, },
  { 119, DWO, DWO, DWO, DWO, DWO, DWO, DWO, },
  { 119, DWO, DWO, DWO, DWO, DWO, DWO, DWO, },
  });
#endif

#if JVET_AD0188_CCP_MERGE
const CtxSet ContextSetCfg::nonLocalCCP = ContextSetCfg::addCtxSet
({
  { CNU, },
  { CNU, },
  { CNU, },
  { DWS, },
  { DWS, },
  { DWS, },
  { DWE, },
  { DWE, },
  { DWE, },
  { DWO, },
  { DWO, },
});
#endif

#if JVET_AE0059_INTER_CCCM
const CtxSet ContextSetCfg::InterCccmFlag = ContextSetCfg::addCtxSet
({
  { CNU, },
  { CNU, },
  { CNU, },
  { DWS, },
  { DWS, },
  { DWS, },
  { DWE, },
  { DWE, },
  { DWE, },
  { DWO, },
  { DWO, },
  });
#endif

#elif SLICE_TYPE_WIN_SIZE
const CtxSet ContextSetCfg::SplitFlag = ContextSetCfg::addCtxSet
({
	{ 18, 20, 37, 11, 28, 45, 27, 22, 23 },
	{ 11, 35, 45, 12, 21, 30, 35, 15, 31 },
	{ 27, 36, 38, 35, 29, 38, 28, 38, 31 },
	{ 9,  9,  4,  8, 12, 12,  9,  9,  8 },
	{ 12, 12,  4, 12, 13, 12,  9,  9, 13 },
	{ 12, 13,  9,  9, 13, 13,  6,  9, 13 }
});

const CtxSet ContextSetCfg::SplitQtFlag = ContextSetCfg::addCtxSet
({
	{ 19, 36, 38, 18, 34,  6 },
	{ 27, 14, 23, 11, 12,  6 },
	{ 19, 13,  7, 33, 27, 22 },
	{ 3, 10, 13, 12,  8,  8 },
	{ 4,  8, 12, 12, 12,  8 },
	{ 0,  5,  7, 12, 13, 12 }
});

const CtxSet ContextSetCfg::SplitHvFlag = ContextSetCfg::addCtxSet
({
	{ 36, 35, 37, 42, 37 },
	{ 36, 35, 37, 34, 52 },
	{ 43, 50, 29, 27, 52 },
	{ 9,  8,  9,  8,  5 },
	{ 9,  9, 12,  8,  6 },
	{ 9,  9,  9,  8,  5 }
});

const CtxSet ContextSetCfg::Split12Flag = ContextSetCfg::addCtxSet
({
	{ 28, 29, 28, 29 },
	{ 43, 37, 28, 22 },
	{ 36, 45, 36, 45 },
	{ 12, 12, 12, 13 },
	{ 12, 13, 12, 13 },
	{ 12, 13, 12, 13 }
});
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
const CtxSet ContextSetCfg::ModeConsFlag = ContextSetCfg::addCtxSet
({
	{ 25, 20 },
	{ 25, 12 },
	{ 35, 35 },
	{ 1,  0 },
	{ 1,  0 },
	{ 1,  0 }
});
#endif
const CtxSet ContextSetCfg::SkipFlag = ContextSetCfg::addCtxSet
({
	{ 50, 60, 53 },
	{ 57, 59, 60 },
	{ 32, 34, 36 },
	{ 5,  4,  8 },
	{ 5,  5,  9 },
	{ 5, 10,  9 }
});

const CtxSet ContextSetCfg::MergeFlag = ContextSetCfg::addCtxSet
({
	{ 6 },
	{ 14 },
	{ 26 },
	{ 4 },
	{ 5 },
	{ 4 }
});

const CtxSet ContextSetCfg::RegularMergeFlag = ContextSetCfg::addCtxSet
({
	{ 31, 15 },
	{ 31, 14 },
	{ 35, 35 },
	{ 9,  4 },
	{ 9,  5 },
	{ 5,  5 }
});

const CtxSet ContextSetCfg::MergeIdx = ContextSetCfg::addCtxSet
({
#if NON_ADJACENT_MRG_CAND
	{ 33, 28, 36, 36, 29, 35, 35, 35, 35, 35 },
	{ 20, 21, 29, 29, 29, 35, 35, 35, 35, 35 },
	{ 34, 43, 36, 35, 25, 35, 35, 35, 35, 35 },
	{ 4,  5,  4,  4,  8,  4,  4,  4,  4,  4 },
	{ 5,  5,  5,  4,  8,  4,  4,  4,  4,  4 },
	{ 5,  4, 10, 13, 13,  4,  4,  4,  4,  4 }
#else
	{ 33 },
	{ 20 },
	{ 34 },
	{ 4  },
	{ 5  },
	{ 5  }
#endif
});

#if TM_MRG || (JVET_Z0084_IBC_TM && IBC_TM_MRG)
const CtxSet ContextSetCfg::TmMergeIdx = ContextSetCfg::addCtxSet
({
#if NON_ADJACENT_MRG_CAND
	{ 19, 35, 42, 35, 35, 35, 35, 35, 35, 35 },
	{ 13, 35, 42, 35, 35, 35, 35, 35, 35, 35 },
	{ 34, 35, 35, 35, 35, 35, 35, 35, 35, 35 },
	{ 4,  4,  4,  4,  4,  4,  4,  4,  4,  4 },
	{ 4,  4,  4,  4,  4,  4,  4,  4,  4,  4 },
	{ 4,  4,  4,  4,  4,  4,  4,  4,  4,  4 }
#else
	{ 19 },
	{ 13 },
	{ 34 },
	{ 4  },
	{ 4  },
	{ 4  }
#endif
});
#endif

#if JVET_Y0065_GPM_INTRA
const CtxSet ContextSetCfg::GPMIntraFlag = ContextSetCfg::addCtxSet
({
  {  35, },
  {  35, },
  {  35, },
  {   4, },
  {   4, },
  {   4, }
});
#endif

const CtxSet ContextSetCfg::MmvdFlag = ContextSetCfg::addCtxSet
({
	{ 25 },
	{ 33 },
	{ 35 },
	{ 5 },
	{ 4 },
	{ 4 }
});

const CtxSet ContextSetCfg::MmvdMergeIdx = ContextSetCfg::addCtxSet
({
	{ 58 },
	{ 43 },
	{ 35 },
	{ 9 },
	{ 10 },
	{ 10 }
});

const CtxSet ContextSetCfg::MmvdStepMvpIdx = ContextSetCfg::addCtxSet
({
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
  { 35, 35, 35, 35, 35},
  { 35, 35, 35, 35, 35},
  { 35, 35, 35, 35, 35},
  { 4,  4,  4,  4,  4 },
  { 4,  4,  4,  4,  4 },
  { 4,  4,  4,  4,  4 }
#else
  { 59 },
  { 60 },
  { 35 },
  { 0 },
  { 0 },
  { 0 }
#endif
});

#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
const CtxSet ContextSetCfg::MmvdStepMvpIdxECM3 = ContextSetCfg::addCtxSet
({
  { 59 },
  { 60 },
  { 35 },
  { 0 },
  { 0 },
  { 0 }
});
#endif

#if JVET_W0097_GPM_MMVD_TM
const CtxSet ContextSetCfg::GeoMmvdFlag = ContextSetCfg::addCtxSet
({
  { 25 },
  { 33 },
  { 35 },
  { 5 },
  { 4 },
  { 4 }
  });

const CtxSet ContextSetCfg::GeoMmvdStepMvpIdx = ContextSetCfg::addCtxSet
({
  { 59 },
  { 60 },
  { 35 },
  { 0 },
  { 0 },
  { 0 }
  });
#endif

#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
const CtxSet ContextSetCfg::GeoBldFlag = ContextSetCfg::addCtxSet
({
  { 59 },
  { 60 },
  { 35 },
  { 0 },
  { 0 },
  { 0 }
});
#endif

#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
const CtxSet ContextSetCfg::GeoSubModeIdx = ContextSetCfg::addCtxSet
({
  { 33, 28, 36, 36, 29, },
  { 20, 21, 29, 29, 29, },
  { 34, 43, 36, 35, 25, },
  {  4,  5,  4,  4,  8, },
  {  5,  5,  5,  4,  8, },
  {  5,  4, 10, 13, 13, }
});
#endif

#if AFFINE_MMVD
const CtxSet ContextSetCfg::AfMmvdFlag = ContextSetCfg::addCtxSet
({
	{ 18 },
	{ 11 },
	{ 35 },
	{ 4 },
	{ 4 },
	{ 4 }
});

const CtxSet ContextSetCfg::AfMmvdIdx = ContextSetCfg::addCtxSet
({
	{ 43 },
	{ 43 },
	{ 35 },
	{ 10 },
	{ 10 },
	{ 10 }
});

const CtxSet ContextSetCfg::AfMmvdOffsetStep = ContextSetCfg::addCtxSet
({
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
  { 35, 35, 35, 35, 35 },
  { 35, 35, 35, 35, 35 },
  { 35, 35, 35, 35, 35 },
  { 4,  4,  4,  4,  4 },
  { 4,  4,  4,  4,  4 },
  { 4,  4,  4,  4,  4 }
#else
  { 51 },
  { 60 },
  { 35 },
  { 0 },
  { 0 },
  { 0 }
#endif
});

#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
const CtxSet ContextSetCfg::AfMmvdOffsetStepECM3 = ContextSetCfg::addCtxSet
({
  { 51 },
  { 60 },
  { 35 },
  {  0 },
  {  0 },
  {  0 }
});
#endif
#endif

#if JVET_AA0061_IBC_MBVD
const CtxSet ContextSetCfg::IbcMbvdFlag = ContextSetCfg::addCtxSet
({
  { 25 },
  { 33 },
  { 35 },
  { 5 },
  { 4 },
  { 4 }
  });

const CtxSet ContextSetCfg::IbcMbvdMergeIdx = ContextSetCfg::addCtxSet
({
  { 58 },
  { 43 },
  { 35 },
  { 9 },
  { 10 },
  { 10 }
  });

const CtxSet ContextSetCfg::IbcMbvdStepMvpIdx = ContextSetCfg::addCtxSet
({
  { 35, 35, 35, 35, 35},
  { 35, 35, 35, 35, 35},
  { 35, 35, 35, 35, 35},
  { 4,  4,  4,  4,  4 },
  { 4,  4,  4,  4,  4 },
  { 4,  4,  4,  4,  4 }
  });
#endif

#if TM_MRG || (JVET_Z0084_IBC_TM && IBC_TM_MRG)
const CtxSet ContextSetCfg::TMMergeFlag = ContextSetCfg::addCtxSet
({
#if JVET_Z0084_IBC_TM && IBC_TM_MRG
  { 25, 33 },
  { 26, 25 },
  { 35, 35 },
  {  4,  5 },
  {  4,  5 },
  {  4,  4 }
#else
	{ 25 },
	{ 26 },
	{ 35 },
	{ 4 },
	{ 4 },
	{ 4 }
#endif
});
#endif

#if TM_MRG
#if JVET_X0141_CIIP_TIMD_TM
const CtxSet ContextSetCfg::CiipTMMergeFlag = ContextSetCfg::addCtxSet
({
  { 25 },
  { 26 },
  { 35 },
  { 4 },
  { 4 },
  { 4 }
  });
#endif
#endif

const CtxSet ContextSetCfg::PredMode = ContextSetCfg::addCtxSet
({
	{ 40, 35 },
	{ 40, 35 },
	{ 35, 35 },
	{ 5,  1 },
	{ 6,  2 },
	{ 5,  1 }
});

#if JVET_AB0157_TMRL
const CtxSet ContextSetCfg::TmrlDerive = ContextSetCfg::addCtxSet
({
  { CNU },
  { CNU },
  { CNU },
  { DWS },
  { DWS },
  { DWS }
  });
#endif

const CtxSet ContextSetCfg::MultiRefLineIdx = ContextSetCfg::addCtxSet
({
#if JVET_Y0116_EXTENDED_MRL_LIST
#if JVET_W0123_TIMD_FUSION
  { 25, 59, 59, 59, 59, 25, 59},
  { 25, 58, 58, 58, 58, 25, 58},
  { 25, 60, 60, 60, 60, 25, 60},
  { 6,  5,  5,  5,  5,  6,  5 },
  { 6,  5,  5,  5,  5,  6,  5 },
  { 6,  8,  8,  8,  8,  6,  8 }
#else
  { 25, 59, 59, 59, 59},
  { 25, 58, 58, 58, 58},
  { 25, 60, 60, 60, 60},
  { 6,  5,  5,  5,  5 },
  { 6,  5,  5,  5,  5 },
  { 6,  8,  8,  8,  8 }
#endif
#else
#if JVET_W0123_TIMD_FUSION
  { 25, 59, 25, 59 },
  { 25, 58, 25, 58 },
  { 25, 60, 25, 60 },
  { 6,  5,  6,  5 },
  { 6,  5,  6,  5 },
  { 6,  8,  6,  8 }
#else
	{ 25, 59 },
	{ 25, 58 },
	{ 25, 60 },
	{ 6,  5 },
	{ 6,  5 },
	{ 6,  8 }
#endif
#endif
});

const CtxSet ContextSetCfg::IntraLumaMpmFlag = ContextSetCfg::addCtxSet
({
	{ 37 },
	{ 29 },
	{ 37 },
	{ 6 },
	{ 6 },
	{ 6 }
});

#if SECONDARY_MPM
const CtxSet ContextSetCfg::IntraLumaSecondMpmFlag = ContextSetCfg::addCtxSet
({
	{ 36 },
	{ 36 },
	{ 44 },
	{ 10 },
	{ 10 },
	{ 7 }
});
#endif

const CtxSet ContextSetCfg::IntraLumaPlanarFlag = ContextSetCfg::addCtxSet
({
	{ 6, 14 },
	{ 13, 21 },
	{ 14, 29 },
	{ 1,  2 },
	{ 0,  2 },
	{ 1,  5 }
});

#if SECONDARY_MPM
const CtxSet ContextSetCfg::IntraLumaMPMIdx = ContextSetCfg::addCtxSet
({
	{ 20, 21, 13 },
	{ 5, 28, 13 },
	{ 20, 44, 35 },
	{ 1,  2,  5 },
	{ 4,  1,  6 },
	{ 2,  2,  6 }
});
#if JVET_AD0085_MPM_SORTING
const CtxSet ContextSetCfg::IntraLumaSecondMpmIdx = ContextSetCfg::addCtxSet
({
  { CNU, CNU, CNU, CNU, CNU },
  { CNU, CNU, CNU, CNU, CNU },
  { CNU, CNU, CNU, CNU, CNU },
  { DWS, DWS, DWS, DWS, DWS },
  { DWS, DWS, DWS, DWS, DWS },
  { DWS, DWS, DWS, DWS, DWS },
  });
#endif
#endif

const CtxSet ContextSetCfg::CclmModeFlag = ContextSetCfg::addCtxSet
({
	{ 26 },
	{ 41 },
	{ 59 },
	{ 1 },
	{ 4 },
	{ 4 }
});

const CtxSet ContextSetCfg::CclmModeIdx = ContextSetCfg::addCtxSet
({
	{ 34 },
	{ 34 },
	{ 12 },
	{ 8 },
	{ 8 },
	{ 9 }
});

const CtxSet ContextSetCfg::IntraChromaPredMode = ContextSetCfg::addCtxSet
({
	{ 25 },
	{ 25 },
	{ 34 },
	{ 5 },
	{ 5 },
	{ 5 }
});

#if JVET_Z0050_DIMD_CHROMA_FUSION
#if ENABLE_DIMD
const CtxSet ContextSetCfg::DimdChromaMode = ContextSetCfg::addCtxSet
( {
  { CNU, },
  { CNU, },
  { CNU, },
  { DWS, },
  { DWS, },
  { DWS, },
  } );
#endif

const CtxSet ContextSetCfg::ChromaFusionMode = ContextSetCfg::addCtxSet
( {
  { CNU, },
  { CNU, },
  { CNU, },
  { DWS, },
  { DWS, },
  { DWS, },
  } );
#endif

#if JVET_AC0071_DBV
const CtxSet ContextSetCfg::DbvChromaMode = ContextSetCfg::addCtxSet
({
  { CNU, },
  { CNU, },
  { CNU, },
  { DWS, },
  { DWS, },
  { DWS, },
  });
#endif

const CtxSet ContextSetCfg::MipFlag = ContextSetCfg::addCtxSet
({
	{ 56, 57, 50, 33 },
	{ 41, 57, 58, 26 },
	{ 33, 49, 50, 25 },
	{ 9,  9,  9,  6 },
	{ 9,  9,  8,  6 },
	{ 10, 10,  9,  6 }
});
#if JVET_V0130_INTRA_TMP
const CtxSet ContextSetCfg::TmpFlag = ContextSetCfg::addCtxSet
({
  {  CNU,  CNU,  CNU,  CNU, },
  {  CNU,  CNU,  CNU,  CNU, },
  {  CNU,  CNU,  CNU,  CNU, },
  {  DWS,  DWS,  DWS,  DWS, },
  {  DWS,  DWS,  DWS,  DWS, },
  {  DWS,  DWS,  DWS,  DWS, },
	});
#endif

#if MMLM
const CtxSet ContextSetCfg::MMLMFlag = ContextSetCfg::addCtxSet
({
	{ 46 },
	{ 46 },
	{ 53 },
	{ 8 },
	{ 4 },
	{ 8 }
});
#endif

const CtxSet ContextSetCfg::DeltaQP = ContextSetCfg::addCtxSet
({
	{ 35, 35 },
	{ 35, 35 },
	{ 35, 35 },
	{ 8,  8 },
	{ 8,  8 },
	{ 8,  8 }
});

const CtxSet ContextSetCfg::InterDir = ContextSetCfg::addCtxSet
({
#if CTU_256
	{ 7,  6,  5, 12, 11,  3, 10, 40 },
	{ 7, 21,  5, 12,  4, 18, 18, 48 },
	{ 35, 35, 35, 35, 35, 35, 35, 35 },
	{ 0,  0,  0,  1,  4,  4,  6,  0 },
	{ 0,  0,  0,  1,  4,  8,  5,  0 },
	{ 0,  0,  0,  1,  4,  4,  4,  0 }
#else
	{  6,  5, 12, 11,  3, 10, 40 },
	{ 21,  5, 12,  4, 18, 18, 48 },
	{ 35, 35, 35, 35, 35, 35, 35 },
	{  0,  0,  1,  4,  4,  6,  0 },
	{  0,  0,  1,  4,  8,  5,  0 },
	{  0,  0,  1,  4,  4,  4,  0 }
#endif
});

const CtxSet ContextSetCfg::RefPic = ContextSetCfg::addCtxSet
({
	{ 12, 20 },
	{ 27, 35 },
	{ 35, 35 },
	{ 0,  4 },
	{ 0,  4 },
	{ 0,  4 }
});

#if JVET_Z0054_BLK_REF_PIC_REORDER
const CtxSet ContextSetCfg::RefPicLC = ContextSetCfg::addCtxSet
({
  { CNU, CNU, CNU },
  { CNU, CNU, CNU },
  { CNU, CNU, CNU },
  { 0, 2, 4 },
  { 0, 2, 4 },
  { 0, 2, 4 }
});
#endif

const CtxSet ContextSetCfg::SubblockMergeFlag = ContextSetCfg::addCtxSet
({
	{ 25, 58, 52 },
	{ 48, 57, 44 },
	{ 35, 35, 35 },
	{ 5,  4,  4 },
	{ 5,  4,  4 },
	{ 4,  4,  4 }
});

#if JVET_X0049_ADAPT_DMVR
const CtxSet ContextSetCfg::BMMergeFlag = ContextSetCfg::addCtxSet
({
  { 25, CNU, CNU, CNU },
  { 26, CNU, CNU, CNU },
  { 35, CNU, CNU, CNU },
  { 4, 4, 4, 4 },
  { 4, 4, 4, 4 },
  { 4, 4, 4, 4 }
});
#endif

#if JVET_AA0070_RRIBC
const CtxSet ContextSetCfg::rribcFlipType = ContextSetCfg::addCtxSet
({
  { 25, CNU, CNU, CNU },
  { 26, CNU, CNU, CNU },
  { 35, CNU, CNU, CNU },
  { 4, 4, 4, 4 },
  { 4, 4, 4, 4 },
  { 4, 4, 4, 4 }
  });
#endif

const CtxSet ContextSetCfg::AffineFlag = ContextSetCfg::addCtxSet
({
	{ 34, 27,  6 },
	{ 26, 12,  6 },
	{ 35, 35, 35 },
	{ 4,  0,  0 },
	{ 5,  1,  0 },
	{ 4,  0,  0 }
});

const CtxSet ContextSetCfg::AffineType = ContextSetCfg::addCtxSet
({
	{ 35 },
	{ 42 },
	{ 35 },
	{ 4 },
	{ 4 },
	{ 4 }
});

const CtxSet ContextSetCfg::AffMergeIdx = ContextSetCfg::addCtxSet
({
	{ 4 },
	{ 12 },
	{ 35 },
	{ 0 },
	{ 0 },
	{ 0 }
});

#if INTER_LIC
const CtxSet ContextSetCfg::LICFlag = ContextSetCfg::addCtxSet
({
	{ 27 },
	{ 34 },
	{ 35 },
	{ 5 },
	{ 8 },
	{ 8 }
});
#endif

const CtxSet ContextSetCfg::BcwIdx = ContextSetCfg::addCtxSet
({
	{ 12 },
	{ 4 },
	{ 35 },
	{ 0 },
	{ 0 },
	{ 1 }
});

const CtxSet ContextSetCfg::Mvd = ContextSetCfg::addCtxSet
({
	{ 51, 43 },
	{ 51, 43 },
	{ 21, 30 },
	{ 9,  5 },
	{ 9,  6 },
	{ 9, 10 }
});

#if JVET_Z0131_IBC_BVD_BINARIZATION
const CtxSet ContextSetCfg::Bvd = ContextSetCfg::addCtxSet
({
  { 53, 38, 38, 29, 20, 34, 27, 45, 37, 43, 34, 48 },
  { 38, 38, 38, 29, 28, 42, 27, 45, 44, 28, 42, 33 },
  { 38, 38, 38, 29, 28, 42, 27, 45, 44, 28, 42, 33 },
  {  1, 12,  8,  4,  2,  5,  3,  4,  0,  0,  5,  4 },
  {  6, 10,  9,  6,  7,  7,  5,  5,  4,  1,  2,  3 },
  {  6, 10,  9,  6,  7,  7,  5,  5,  4,  1,  2,  3 }
});
#endif

#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED || JVET_AC0104_IBC_BVD_PREDICTION
const CtxSet ContextSetCfg::MvsdIdx = ContextSetCfg::addCtxSet
({
  { 34, 34, 34, 34,},
  { 34, 34, 34, 34,},
  { 34, 34, 34, 34,},
  { 13, 13, 13, 13,},
  { 13, 13, 13, 13,},
  { 13, 13, 13, 13,}
});
#endif
#if MULTI_HYP_PRED
const CtxSet ContextSetCfg::MultiHypothesisFlag = ContextSetCfg::addCtxSet
({
  { 3, 26, CNU, },
  { 3, 26, CNU, },
  { 35, 35, CNU, },
  { 1,  4, DWS, },
  { 1,  4, DWS, },
  { 4,  4, DWS, }
});

const CtxSet ContextSetCfg::MHRefPic = ContextSetCfg::addCtxSet
({
	{ 28, 50 },
	{ 28, 27 },
	{ 35, 35 },
	{ 1,  0 },
	{ 1,  0 },
	{ 4,  4 }
});

const CtxSet ContextSetCfg::MHWeight = ContextSetCfg::addCtxSet
({
	{ 50, 35 },
	{ 12, 35 },
	{ 35, 35 },
	{ 0,  4 },
	{ 0,  4 },
	{ 4,  4 }
});
#endif

const CtxSet ContextSetCfg::BDPCMMode = ContextSetCfg::addCtxSet
({
	{ 12, 21, 32, 36 },
	{ 40, 21, 32,  5 },
	{ 11, 50, 32, 19 },
	{ 0,  0,  1,  9 },
	{ 0,  0,  1,  0 },
	{ 1,  4,  8,  3 }
});

const CtxSet ContextSetCfg::QtRootCbf = ContextSetCfg::addCtxSet
({
	{ 5 },
	{ 5 },
	{ 13 },
	{ 4 },
	{ 4 },
	{ 4 }
});

const CtxSet ContextSetCfg::ACTFlag = ContextSetCfg::addCtxSet
({
	{ 46 },
	{ 46 },
	{ 52 },
	{ 1 },
	{ 1 },
	{ 1 }
});

const CtxSet ContextSetCfg::QtCbf[3] =
{
	ContextSetCfg::addCtxSet
	({
		{ 23, 14,  5, 14 },
		{ 23,  5, 20,  7 },
		{ 15, 12,  5,  7 },
		{ 6,  0,  8,  8 },
		{ 6,  0,  8, 10 },
		{ 5,  1,  8,  9 }
}),
ContextSetCfg::addCtxSet
({
	{ 25, 14 },
	{ 25,  6 },
	{ 12,  6 },
	{ 5,  0 },
	{ 5,  3 },
	{ 5,  0 }
}),
ContextSetCfg::addCtxSet
({
	{ 9, 44, 37 },
	{ 25, 29, 37 },
	{ 26, 13, 52 },
	{ 2,  1,  8 },
	{ 2,  1,  8 },
	{ 2,  2,  0 }
}),
};

const CtxSet ContextSetCfg::SigCoeffGroup[2] =
{
	ContextSetCfg::addCtxSet
	({
		{ 25, 45 },
		{ 25, 45 },
		{ 11, 31 },
		{ 8,  5 },
		{ 8,  5 },
		{ 5,  5 }
}),
ContextSetCfg::addCtxSet
({
	{ 25, 30 },
	{ 25, 45 },
	{ 25, 15 },
	{ 5,  9 },
	{ 9, 12 },
	{ 5,  9 }
}),
};

const CtxSet ContextSetCfg::SigFlag[6] =
{
	ContextSetCfg::addCtxSet
	({
		{ 17, 41, 49, 36,  1, 49, 50, 37, 48, 51, 58, 45 },
		{ 17, 41, 42, 29, 25, 49, 43, 37, 33, 51, 51, 30 },
		{ 25, 19, 28, 14, 18, 28, 29, 30, 19, 45, 30, 38 },
		{ 13,  9,  9,  9,  9,  9,  8, 10,  8,  8,  8,  9 },
		{ 13,  9,  9,  9,  9,  9,  9, 10,  8,  8,  8,  9 },
		{ 13,  9,  9, 10, 10, 10, 10, 13,  9,  9,  9, 10 }
}),
ContextSetCfg::addCtxSet
({
	{ 25, 34, 35, 29, 56, 52, 52, 38 },
	{ 25, 34, 35, 29, 34, 37, 52, 38 },
	{ 25, 27, 28, 37, 42, 53, 53, 46 },
	{ 12, 12,  9, 13,  4,  5,  8,  9 },
	{ 13, 13, 13, 13,  4,  5,  8,  8 },
	{ 12, 12, 10, 13,  5,  5,  9,  9 }
}),
ContextSetCfg::addCtxSet
({
	{ 26, 45, 53, 46, 34, 54, 61, 39, 27, 39, 39, 39 },
	{ 19, 38, 38, 46, 34, 54, 54, 39,  6, 39, 39, 39 },
	{ 11, 38, 46, 54, 27, 39, 39, 39, 36, 39, 39, 39 },
	{ 9, 13, 12,  8,  8,  8,  8,  5,  4,  0,  0,  0 },
	{ 9, 13,  8,  8,  8,  8,  8,  5,  0,  0,  0,  0 },
	{ 9, 13,  8,  8,  8,  8,  8,  5,  4,  0,  0,  0 }
}),
ContextSetCfg::addCtxSet
({
	{ 34, 45, 38, 31, 27, 39, 39, 39 },
	{ 42, 45, 53, 54, 44, 39, 39, 39 },
	{ 19, 46, 38, 39, 52, 39, 39, 39 },
	{ 8, 12, 12,  8,  4,  4,  0,  0 },
	{ 8, 12, 12,  8,  4,  4,  0,  0 },
	{ 8, 12, 12,  8,  0,  0,  0,  1 }
}),
ContextSetCfg::addCtxSet
({
	{ 19, 54, 39, 39, 42, 39, 39, 39, 32, 39, 39, 39 },
	{ 19, 39, 54, 39, 19, 39, 39, 39, 56, 39, 39, 39 },
	{ 18, 39, 39, 39, 27, 39, 39, 39, 32, 39, 39, 39 },
	{ 8,  8,  8,  8,  8,  4,  4,  8,  5,  0,  0,  0 },
	{ 8,  8,  8,  8,  8,  4,  4,  8,  0,  0,  0,  0 },
	{ 8,  8,  8,  8,  8,  0,  4,  4,  5,  0,  0,  0 }
}),
ContextSetCfg::addCtxSet
({
	{ 34, 38, 54, 39, 34, 39, 39, 39 },
	{ 34, 38, 62, 39, 34, 39, 39, 39 },
	{ 11, 39, 39, 39, 34, 39, 39, 39 },
	{ 8,  8,  8,  8,  0,  1,  1,  2 },
	{ 8,  8,  8,  8,  0,  1,  0,  0 },
	{ 8,  8,  8,  8,  4,  0,  2,  4 }
}),
};

const CtxSet ContextSetCfg::ParFlag[2] =
{
	ContextSetCfg::addCtxSet
	({
		{ 33, 40, 33, 26, 34, 42, 25, 33, 34, 34, 27, 25, 34, 42, 42, 35, 33, 27, 35, 42, 35 },
		{ 33, 25, 33, 26, 34, 42, 25, 33, 34, 42, 27, 25, 34, 42, 42, 35, 26, 27, 42, 35, 35 },
		{ 33, 25, 18, 26, 34, 27, 25, 26, 19, 42, 35, 33, 19, 27, 35, 35, 34, 42, 20, 43, 20 },
		{ 12,  9, 13, 13, 13, 13, 13, 13, 13, 13, 13, 12, 13, 13, 13, 13,  9, 13, 13, 12, 13 },
		{ 13,  9, 13, 13, 13, 13, 13, 13, 13, 13, 13, 12, 13, 13, 13, 13, 10, 13, 13, 12, 13 },
		{ 9, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13 }
}),
ContextSetCfg::addCtxSet
({
	{ 33, 25, 34, 34, 34, 27, 33, 42, 35, 35, 35 },
	{ 33, 25, 26, 11, 34, 27, 33, 42, 35, 35, 35 },
	{ 33, 25, 26, 42, 19, 27, 26, 50, 35, 35, 35 },
	{ 12, 13, 12, 12, 13, 13, 13, 13, 12, 13, 13 },
	{ 13, 13, 12, 12, 13, 13, 13, 12, 12, 12, 13 },
	{ 13, 13, 13, 13, 13, 13, 13, 13, 13, 12, 13 }
}),
};

const CtxSet ContextSetCfg::GtxFlag[4] =
{
	ContextSetCfg::addCtxSet
	({
		{ 25, 32, 40, 25, 33, 34, 32, 17, 25, 33, 27,  9, 25, 41, 34, 20, 25, 33, 34, 35, 37 },
		{ 40, 32, 40, 25, 33, 26, 32,  9, 25, 33, 34,  9, 25, 33, 34, 35, 25, 33, 34, 42, 29 },
		{ 25,  1, 40, 25, 33, 19, 17, 25, 25, 18, 12, 17, 33, 26, 19, 13, 33, 19, 20, 28, 22 },
		{ 1,  9,  9, 10, 13,  5,  9,  9, 10, 13,  6,  9,  9,  9,  9,  9,  6,  8,  5,  9, 10 },
		{ 0,  9,  5,  9, 13,  5, 12, 12, 12, 13,  5, 12,  9,  8,  9,  8,  6,  8,  8,  8,  9 },
		{ 4,  9, 10, 10, 10,  5,  9, 12, 10, 10,  6, 13, 10, 10, 10, 10,  7,  9, 10, 10, 10 }
}),
ContextSetCfg::addCtxSet
({
	{ 25,  1, 25, 33, 34, 12, 25, 33, 34,  5, 37 },
	{ 40,  9, 25,  3, 11,  4, 17, 33, 34, 27, 14 },
	{ 40,  1, 25, 18, 34, 27, 25, 34, 35, 36, 37 },
	{ 1,  9, 12,  8,  8,  5,  6, 12,  8,  8,  8 },
	{ 4,  8,  8,  4,  5,  5,  9, 12,  4,  6,  8 },
	{ 2,  9,  9,  8,  8,  6, 10,  9,  8,  8,  9 }
}),
ContextSetCfg::addCtxSet
({
	{ 32, 32, 41, 42, 35, 21, 25, 34, 35, 36, 37, 40, 42, 36, 29, 30, 34, 36, 37, 45, 38 },
	{ 32, 17, 26, 34, 35, 44, 25, 34, 35, 36, 37, 33, 27, 36, 29, 37, 34, 28, 37, 37, 38 },
	{ 25, 25, 11, 27, 20, 29, 33, 12, 28, 21, 22, 34, 28, 29, 29, 30, 36, 29, 45, 30, 23 },
	{ 9,  8, 10, 13, 13,  9,  9, 10, 13, 13, 13,  9,  9, 10, 10, 13,  5,  9, 10, 13, 13 },
	{ 9,  9,  9, 12, 13,  9,  9, 10, 12, 13, 13,  9,  9, 10, 10, 12,  8,  8, 10,  9, 13 },
	{ 9,  9, 10, 13, 13,  9, 13, 10, 13, 13, 13, 10, 10, 10, 10, 13,  9, 10, 10, 10, 13 }
}),
ContextSetCfg::addCtxSet
({
	{ 32, 40, 27, 36, 36, 37, 57, 37, 37, 30, 38 },
	{ 1, 25, 27, 20, 13, 14, 42, 37, 37, 37, 23 },
	{ 40, 41, 35, 36, 21, 37, 36, 37, 45, 38, 46 },
	{ 9,  9, 10, 12, 12, 10,  5,  9,  8,  9, 12 },
	{ 9, 12, 12, 12, 12, 10,  5,  8,  5,  6, 12 },
	{ 9, 12,  9, 12, 12, 10, 10, 12,  9, 13, 13 }
}),
};

const CtxSet ContextSetCfg::LastX[2] =
{
	ContextSetCfg::addCtxSet
	({
#if TU_256
		{  6,  6, 12, 14,  6,  4,  6,  7,  6,  4, 14,  7,  6,  6, 12, 21,  7,  6,  6, 37, 43, 35,  6, 15, 53, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35 }, // 36 contexts
		{  6, 13, 12,  6,  6, 12, 14,  6,  5, 12, 29, 14,  6,  6,  6, 21, 21, 28,  6,  6, 37,  6,  6, 15, 31, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26 },
		{ 13,  5,  4,  6,  6, 12,  6, 14, 29,  4, 14,  7, 22, 29,  4, 22, 38, 15, 22,  6, 19,  7, 37,  7, 13, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42 },
		{  5,  4,  4,  5,  4,  1,  5,  4,  1,  0,  5,  1,  0,  0,  0,  1,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
		{  5,  4,  4,  5,  4,  0,  5,  4,  0,  0,  5,  1,  0,  0,  0,  1,  1,  0,  0,  0,  0,  0,  4,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
		{  8,  5,  5,  6,  4,  4,  5,  4,  1,  0,  4,  1,  0,  0,  0,  1,  1,  1,  0,  0,  0,  1,  4,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 }
#else
		{  6,  6, 12, 14,  6,  4,  6,  7,  6,  4, 14,  7,  6,  6, 12, 21,  7,  6,  6, 37 }, // 20 contexts
		{  6, 13, 12,  6,  6, 12, 14,  6,  5, 12, 29, 14,  6,  6,  6, 21, 21, 28,  6,  6 },
		{ 13,  5,  4,  6,  6, 12,  6, 14, 29,  4, 14,  7, 22, 29,  4, 22, 38, 15, 22,  6 },
		{  5,  4,  4,  5,  4,  1,  5,  4,  1,  0,  5,  1,  0,  0,  0,  1,  1,  0,  0,  0 },
		{  5,  4,  4,  5,  4,  0,  5,  4,  0,  0,  5,  1,  0,  0,  0,  1,  1,  0,  0,  0 },
		{  8,  5,  5,  6,  4,  4,  5,  4,  1,  0,  4,  1,  0,  0,  0,  1,  1,  1,  0,  0 }
#endif
}),
ContextSetCfg::addCtxSet
({
	{ 26, 12, 11 },
	{ 19, 34, 33 },
	{ 12,  4,  3 },
	{ 4,  4,  4 },
	{ 5,  4,  4 },
	{ 6,  4,  4 }
}),
};

const CtxSet ContextSetCfg::LastY[2] =
{
	ContextSetCfg::addCtxSet
	({
#if TU_256
		{  5,  5, 20, 13, 13, 19,  6,  6, 12, 20, 14, 14,  5,  4, 20,  6,  7,  6, 12, 57, 51, 42, 13, 20,  5, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41 }, // 36 contexts
		{  5,  5, 12,  6,  6, 19,  6,  6,  5, 12, 14,  7, 13,  5, 37, 21,  7, 28, 20, 37, 37, 28, 13,  6, 15, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34 },
		{ 13,  5,  4,  6,  6, 11, 14, 14,  5, 11, 14,  7,  6,  5, 11, 22, 38, 22,  6, 20, 19, 51,  7, 14,  4, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34 },
		{  5,  4,  4,  5,  4,  4,  5,  5,  4,  0,  5,  5,  1,  0,  0,  1,  4,  1,  0,  0,  0,  0,  4,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
		{  8,  5,  4,  5,  5,  4,  6,  5,  4,  0,  5,  4,  1,  0,  0,  1,  5,  0,  0,  0,  0,  0,  4,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
		{  9,  5,  8,  6,  5,  4,  6,  5,  4,  0,  5,  5,  1,  0,  0,  1,  1,  0,  0,  0,  0,  0,  0,  4,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 }
#else
    {  5,  5, 20, 13, 13, 19,  6,  6, 12, 20, 14, 14,  5,  4, 20,  6,  7,  6, 12, 57 }, // 20 contexts
    {  5,  5, 12,  6,  6, 19,  6,  6,  5, 12, 14,  7, 13,  5, 37, 21,  7, 28, 20, 37 },
    { 13,  5,  4,  6,  6, 11, 14, 14,  5, 11, 14,  7,  6,  5, 11, 22, 38, 22,  6, 20 },
    {  5,  4,  4,  5,  4,  4,  5,  5,  4,  0,  5,  5,  1,  0,  0,  1,  4,  1,  0,  0 },
    {  8,  5,  4,  5,  5,  4,  6,  5,  4,  0,  5,  4,  1,  0,  0,  1,  5,  0,  0,  0 },
    {  9,  5,  8,  6,  5,  4,  6,  5,  4,  0,  5,  5,  1,  0,  0,  1,  1,  0,  0,  0 }
#endif
}),
ContextSetCfg::addCtxSet
({
	{ 26, 20, 34 },
	{ 26, 19, 18 },
	{ 20,  4,  3 },
	{ 2,  4,  4 },
	{ 5,  5,  5 },
	{ 6,  5,  5 }
}),
};

const CtxSet ContextSetCfg::MVPIdx = ContextSetCfg::addCtxSet
({
	{ 41 },
	{ 41 },
	{ 42 },
	{ 8 },
	{ 8 },
	{ 13 }
});

#if JVET_X0083_BM_AMVP_MERGE_MODE
const CtxSet ContextSetCfg::amFlagState = ContextSetCfg::addCtxSet
({
  {   34 },
  {   34 },
  {  CNU },
  {    4 },
  {    4 },
  {  DWS },
  });
#endif

const CtxSet ContextSetCfg::SmvdFlag = ContextSetCfg::addCtxSet
({
	{ 28 },
	{ 13 },
	{ 35 },
	{ 4 },
	{ 4 },
	{ 5 }
});

const CtxSet ContextSetCfg::SaoMergeFlag = ContextSetCfg::addCtxSet
({
	{ 2 },
	{ 60 },
	{ 59 },
	{ 0 },
	{ 0 },
	{ 1 }
});

const CtxSet ContextSetCfg::SaoTypeIdx = ContextSetCfg::addCtxSet
({
	{ 10 },
	{ 13 },
	{ 6 },
	{ 0 },
	{ 4 },
	{ 4 }
});

#if JVET_V0094_BILATERAL_FILTER
const CtxSet ContextSetCfg::BifCtrlFlags[] =
{
ContextSetCfg::addCtxSet
( {
  { 39 },
  { 39 },
  { 39 },
  { DWS },
  { DWS },
  { DWS }
} ),
#if JVET_X0071_CHROMA_BILATERAL_FILTER
ContextSetCfg::addCtxSet
( {
  { 39 },
  { 39 },
  { 39 },
  { DWS },
  { DWS },
  { DWS }
} ),
ContextSetCfg::addCtxSet
( {
  { 39 },
  { 39 },
  { 39 },
  { DWS },
  { DWS },
  { DWS }
} )
#endif
}
#endif

#if JVET_W0066_CCSAO
const CtxSet ContextSetCfg::CcSaoControlIdc = ContextSetCfg::addCtxSet
({
  { CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
  { CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
  { CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
  { DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, },
  { DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, },
  { DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, },
});
#endif

const CtxSet ContextSetCfg::LFNSTIdx = ContextSetCfg::addCtxSet
({
#if INTRA_TRANS_ENC_OPT
  { 51, CNU,  50,  35, },
  { 36, CNU,  43,  35, },
  { CNU,  51,  43,  42, },
  { 10, DWS,   8,  13, },
  { 10, DWS,   6,  13, },
  { DWS,  10,   9,  10, }
#elif EXTENDED_LFNST || JVET_W0119_LFNST_EXTENSION
  { 58, 37, 42, 35 },
  { 43, 45, 42, 35 },
  { 28, 43, 42, 27 },
  {  9,  9,  9, 10 },
  {  9,  9,  6, 13 },
  {  9, 10,  9, 10 }
#else
  { 58, 37, 42 },
  { 43, 45, 42 },
  { 28, 43, 42 },
  {  9,  9,  9 },
  {  9,  9,  6 },
  {  9, 10,  9 }
#endif
});

const CtxSet ContextSetCfg::PLTFlag = ContextSetCfg::addCtxSet
({
	{ 17 },
	{ 0 },
	{ 25 },
	{ 1 },
	{ 1 },
	{ 1 }
});

const CtxSet ContextSetCfg::RotationFlag = ContextSetCfg::addCtxSet
({
	{ 35 },
	{ 42 },
	{ 42 },
	{ 5 },
	{ 5 },
	{ 5 }
});

const CtxSet ContextSetCfg::RunTypeFlag = ContextSetCfg::addCtxSet
({
	{ 50 },
	{ 59 },
	{ 42 },
	{ 9 },
	{ 9 },
	{ 9 }
});

const CtxSet ContextSetCfg::IdxRunModel = ContextSetCfg::addCtxSet
({
	{ 58, 45, 45, 30, 38 },
	{ 51, 30, 30, 38, 23 },
	{ 50, 37, 45, 30, 46 },
	{ 9,  6,  9, 10,  5 },
	{ 9,  6,  9, 10,  5 },
	{ 9,  6,  9, 10,  5 }
});

const CtxSet ContextSetCfg::CopyRunModel = ContextSetCfg::addCtxSet
({
	{ 45, 38, 46 },
	{ 38, 53, 46 },
	{ 45, 38, 46 },
	{ 0,  9,  5 },
	{ 0,  9,  5 },
	{ 0,  9,  5 }
});

const CtxSet ContextSetCfg::TransformSkipFlag = ContextSetCfg::addCtxSet
({
	{ 25, 17 },
	{ 25,  1 },
	{ 25,  9 },
	{ 0,  1 },
	{ 1,  5 },
	{ 2,  5 }
});

#if JVET_W0103_INTRA_MTS
#if JVET_Y0142_ADAPT_INTRA_MTS
const CtxSet ContextSetCfg::MTSIdx = ContextSetCfg::addCtxSet
({
  { 43, 38, 46, 38 },
  { 36, 38, 46, 38 },
  { 35, 38, 38, 38 },
  {  8,  9,  9,  8 },
  { 12,  8,  8,  8 },
  {  9,  9,  9,  9 }
});
#elif INTRA_TRANS_ENC_OPT
const CtxSet ContextSetCfg::MTSIdx = ContextSetCfg::addCtxSet
({
  { 38,  42,  27,  45, },
  { 31,  27,  27,  38, },
  { 45,  28,  28,  37, },
  { 8,   8,   9,   8, },
  { 8,   8,   9,   8, },
  { 9,   9,  10,   8, }
});
#else
const CtxSet ContextSetCfg::MTSIdx = ContextSetCfg::addCtxSet
({
  { 45, 35, 20, 45, },
  { 38, 35, 35, 38, },
  { 37, 28, 28, 37, },
  { 8,  10, 10, 8,  },
  { 8,  10, 10, 8,  },
  { 9,  10, 10, 8,  }
  });
#endif
#else
const CtxSet ContextSetCfg::MTSIdx = ContextSetCfg::addCtxSet
({
	{ 37, 25, 34, 40 },
	{ 45, 40, 42, 32 },
	{ 36, 32, 28, 32 },
	{ 9,  0,  4,  0 },
	{ 9,  0, 10,  8 },
	{ 9,  1,  9,  0 }
});
#endif

const CtxSet ContextSetCfg::ISPMode = ContextSetCfg::addCtxSet
({
#if JVET_W0123_TIMD_FUSION
  { 33, 43, 33 },
  { 33, 43, 33 },
  { 33, 43, 33 },
  { 9,  2,  9 },
  { 9,  3,  9 },
  { 9,  2,  9 }
#else
	{ 33, 43 },
	{ 33, 43 },
	{ 33, 43 },
	{ 9,  2 },
	{ 9,  3 },
	{ 9,  2 }
#endif
});

const CtxSet ContextSetCfg::SbtFlag = ContextSetCfg::addCtxSet
({
	{ 33, 57 },
	{ 56, 57 },
	{ 35, 35 },
	{ 1,  5 },
	{ 1,  5 },
	{ 1,  5 }
});

const CtxSet ContextSetCfg::SbtQuadFlag = ContextSetCfg::addCtxSet
({
	{ 42 },
	{ 42 },
	{ 35 },
	{ 10 },
	{ 10 },
	{ 10 }
});

const CtxSet ContextSetCfg::SbtHorFlag = ContextSetCfg::addCtxSet
({
	{ 35, 51, 27 },
	{ 20, 58, 34 },
	{ 35, 35, 35 },
	{ 8,  5,  2 },
	{ 5,  4,  4 },
	{ 8,  4,  1 }
});

const CtxSet ContextSetCfg::SbtPosFlag = ContextSetCfg::addCtxSet
({
	{ 35 },
	{ 28 },
	{ 35 },
	{ 12 },
	{ 13 },
	{ 13 }
});

const CtxSet ContextSetCfg::ChromaQpAdjFlag = ContextSetCfg::addCtxSet
({
	{ 35 },
	{ 35 },
	{ 35 },
	{ 8 },
	{ 8 },
	{ 8 }
});

#if ENABLE_DIMD
const CtxSet ContextSetCfg::DimdFlag = ContextSetCfg::addCtxSet
({
#if INTRA_TRANS_ENC_OPT
  { 48, CNU, CNU, },
  { 48, CNU, CNU, },
  { 33, CNU, CNU, },
  { 6, DWS, DWS, },
  { 6, DWS, DWS, },
  { 3, DWS, DWS, }
#else
  { 48, 56, 56 },
  { 41, 49, 49 },
  { 33, 49, 49 },
  { 5,  1,  1 },
  { 5,  1,  1 },
  { 2,  1,  1 }
#endif
});
#endif


#if JVET_W0123_TIMD_FUSION
const CtxSet ContextSetCfg::TimdFlag = ContextSetCfg::addCtxSet
({
#if INTRA_TRANS_ENC_OPT
  { 41,  49,  49, },
  { 34,  34,  34, },
  { 34,  42,  50, },
  { 6,   6,   5, },
  { 7,   6,   5, },
  { 6,   5,   2, }
#else
  { 48, 56, 56 },
  { 41, 49, 49 },
  { 33, 49, 49 },
  { 5,  1,  1 },
  { 5,  1,  1 },
  { 2,  1,  1 }
#endif
});

#endif
#if ENABLE_OBMC 
const CtxSet ContextSetCfg::ObmcFlag = ContextSetCfg::addCtxSet
({
	{ 62 },
	{ 39 },
	{ 35 },
	{ 0 },
	{ 8 },
	{ 0 }
});
#endif

const CtxSet ContextSetCfg::ChromaQpAdjIdc = ContextSetCfg::addCtxSet
({
	{ 35 },
	{ 35 },
	{ 35 },
	{ 8 },
	{ 8 },
	{ 8 }
});

const CtxSet ContextSetCfg::ImvFlag = ContextSetCfg::addCtxSet
({
	{ 59, 26, 50, 60, 38 },
	{ 59, 48, 58, 60, 60 },
	{ 35, 34, 35, 35, 35 },
	{ 1,  4,  1,  0,  4 },
	{ 0,  5,  1,  0,  4 },
	{ 0, 10,  0,  0,  4 }
});

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
const CtxSet ContextSetCfg::ImvFlagIBC = ContextSetCfg::addCtxSet
({
  { 59, 26, 50, 60, 38 },
  { 59, 48, 58, 60, 60 },
  { 59, 26, 50, 60, 38 },
  { 1,  4,  1,  0,  4 },
  { 0,  5,  1,  0,  4 },
  { 1,  4,  1,  0,  4 },
});
#endif

const CtxSet ContextSetCfg::ctbAlfFlag = ContextSetCfg::addCtxSet
({
	{ 18, 37, 46, 25, 53, 54, 25, 46, 54 },
	{ 5, 23, 46,  4, 53, 46, 34, 46, 46 },
	{ 45, 39, 39, 22, 31, 39, 37, 31, 39 },
	{ 5,  0,  0, 10,  0,  3,  7,  0,  6 },
	{ 0,  0,  0,  4,  0,  0,  0,  0,  0 },
	{ 3,  3, 13,  0,  1, 10,  4,  1, 10 }
});

const CtxSet ContextSetCfg::ctbAlfAlternative = ContextSetCfg::addCtxSet
({
#if ALF_IMPROVEMENT
	{  4, 19, 19 },
	{ 35, 35, 20 },
	{ 26, 19, 19 },
	{ 0,  0,  0 },
	{ 0,  0,  0 },
	{ 0,  0,  0 }
#else
	{ 19, 19 },
	{ 35, 20 },
	{ 19, 19 },
	{  0,  0 },
	{  0,  0 },
	{  0,  0 }
#endif
});

const CtxSet ContextSetCfg::AlfUseTemporalFilt = ContextSetCfg::addCtxSet
({
	{ 46 },
	{ 53 },
	{ 31 },
	{ 0 },
	{ 0 },
	{ 1 }
});

const CtxSet ContextSetCfg::CcAlfFilterControlFlag = ContextSetCfg::addCtxSet
({
	{ 25, 20, 38, 25, 20, 38 },
	{ 18,  6, 38, 18,  6, 38 },
	{ 34, 38, 31, 34, 46, 31 },
	{ 5,  1,  4,  4,  3,  4 },
	{ 4,  0,  0,  4,  0,  0 },
	{ 0,  3,  8,  0,  0,  8 }
});

const CtxSet ContextSetCfg::CiipFlag = ContextSetCfg::addCtxSet
({
#if CIIP_PDPC
	{ 57, 21 },
	{ 50, 36 },
	{ 35, 35 },
	{ 0,  1 },
	{ 1,  2 },
	{ 1,  1 }
#else
	{ 57 },
	{ 50 },
	{ 35 },
	{  0 },
	{  1 },
	{  1 }
#endif
});

const CtxSet ContextSetCfg::IBCFlag = ContextSetCfg::addCtxSet
({
	{ 32, 50, 37 },
	{ 32, 42, 29 },
	{ 40, 27, 36 },
	{ 5,  4,  4 },
	{ 5,  5,  4 },
	{ 5,  6,  8 }
});

const CtxSet ContextSetCfg::JointCbCrFlag = ContextSetCfg::addCtxSet
({
	{ 42, 43, 45 },
	{ 27, 36, 45 },
	{ 20, 29, 58 },
	{ 1,  0,  0 },
	{ 1,  0,  0 },
	{ 1,  1,  0 }
});

const CtxSet ContextSetCfg::TsSigCoeffGroup = ContextSetCfg::addCtxSet
({
	{ 18, 35, 37 },
	{ 18, 12, 37 },
	{ 18, 20, 38 },
	{ 5,  8,  4 },
	{ 6,  9,  6 },
	{ 5,  8,  9 }
});

const CtxSet ContextSetCfg::TsSigFlag = ContextSetCfg::addCtxSet
({
	{ 25, 35, 37 },
	{ 40, 35, 44 },
	{ 25, 28, 38 },
	{ 13, 12,  5 },
	{ 13, 12,  5 },
	{ 13, 13, 10 }
});

const CtxSet ContextSetCfg::TsParFlag = ContextSetCfg::addCtxSet
({
	{ 11 },
	{ 18 },
	{ 11 },
	{ 5 },
	{ 6 },
	{ 10 }
});

const CtxSet ContextSetCfg::TsGtxFlag = ContextSetCfg::addCtxSet
({
	{ 35,  3,  4,  4,  5 },
	{ 35,  2, 18, 34, 11 },
	{ 35, 10,  3,  3,  3 },
	{ 8,  1,  0,  0,  0 },
	{ 8,  2,  1,  0,  0 },
	{ 8,  1,  1,  1,  1 }
});

const CtxSet ContextSetCfg::TsLrg1Flag = ContextSetCfg::addCtxSet
({
	{ 34, 11,  4, 14 },
	{ 18, 11,  4, 21 },
	{ 19,  5, 13, 14 },
	{ 1,  1,  1,  2 },
	{ 4,  2,  1,  0 },
	{ 5,  2,  2,  7 }
});

const CtxSet ContextSetCfg::TsResidualSign = ContextSetCfg::addCtxSet
({
	{ 28, 25, 46, 28, 33, 38 },
	{ 5, 10, 53, 50, 25, 46 },
	{ 20, 17, 46, 28, 25, 46 },
	{ 1,  4,  4,  6,  4,  8 },
	{ 5,  5,  4,  0,  8,  9 },
	{ 1,  4,  5,  9,  8,  8 }
});

#if SIGN_PREDICTION
const CtxSet ContextSetCfg::signPred[2] = 
{
#if JVET_Y0141_SIGN_PRED_IMPROVE
  ContextSetCfg::addCtxSet
  ( {
    { 34,  34,  19,  26, },
    { 34,  34,  34,  26, },
    { 34,  34,  34,  26, },
    { 13,  13,  13,  10, },
    { 13,  13,  13,  10, },
    { 13,  10,  10,  10, }
    } ),
  ContextSetCfg::addCtxSet
  ( {
    { 34,  34,  19,  26, },
    { 34,  41,  34,  41, },
    { 34,  34, CNU, CNU, },
    { 13,  13,   9,   8, },
    { 13,  13,  10,  10, },
    { 13,  13, DWS, DWS, }
    } )
#else
  ContextSetCfg::addCtxSet
  ( {
    { 34, 34, 34, 34 },
    { 34, 34, 34, 34 },
    { 34, 34, 34, 34 },
    {  9,  9,  9,  9 },
    {  9,  9,  9,  9 },
    { 10, 10, 10, 10 }
    } ),

  ContextSetCfg::addCtxSet
  ( {
    { 34, 34, 34, 34 },
    { 34, 34, 34, 34 },
    { 49, 49, 49, 49 },
    {  9,  9,  9,  9 },
    {  9,  9,  9,  9 },
    {  9,  9,  9,  9 }
    } )
#endif
};
#endif

#if JVET_Z0050_CCLM_SLOPE
const CtxSet ContextSetCfg::CclmDeltaFlags = ContextSetCfg::addCtxSet
({
  {  CNU, CNU, CNU, CNU, CNU, },
  {  CNU, CNU, CNU, CNU, CNU, },
  {  CNU, CNU, CNU, CNU, CNU, },
  {  DWS, DWS, DWS, DWS, DWS, },
  {  DWS, DWS, DWS, DWS, DWS, },
  {  DWS, DWS, DWS, DWS, DWS, },
});
#endif

#if JVET_AA0126_GLM
const CtxSet ContextSetCfg::GlmFlags = ContextSetCfg::addCtxSet
({
  {  CNU, CNU, CNU, CNU, CNU, },
  {  CNU, CNU, CNU, CNU, CNU, },
  {  CNU, CNU, CNU, CNU, CNU, },
  {  DWS, DWS, DWS, DWS, DWS, },
  {  DWS, DWS, DWS, DWS, DWS, },
  {  DWS, DWS, DWS, DWS, DWS, },
});
#endif

#if JVET_AA0057_CCCM
const CtxSet ContextSetCfg::CccmFlag = ContextSetCfg::addCtxSet
({
#if JVET_AC0147_CCCM_NO_SUBSAMPLING && JVET_AC0054_GLCCCM
  { CNU, CNU, CNU, },
  { CNU, CNU, CNU, },
  { CNU, CNU, CNU, },
  { DWS, DWS, DWS, },
  { DWS, DWS, DWS, },
  { DWS, DWS, DWS, },
#elif JVET_AC0147_CCCM_NO_SUBSAMPLING || JVET_AC0054_GLCCCM
  { CNU, CNU, },
  { CNU, CNU, },
  { CNU, CNU, },
  { DWS, DWS, },
  { DWS, DWS, },
  { DWS, DWS, },   
#else
  { CNU, },
  { CNU, },
  { CNU, },
  { DWS, },
  { DWS, },
  { DWS, },
#endif
});
#if JVET_AE0100_BVGCCCM
const CtxSet ContextSetCfg::BvgCccmFlag = ContextSetCfg::addCtxSet
({
  { CNU, },
  { CNU, },
  { CNU, },
  { DWS, },
  { DWS, },
  { DWS, },
});
#endif
#endif

#if JVET_AC0119_LM_CHROMA_FUSION
const CtxSet ContextSetCfg::ChromaFusionType = ContextSetCfg::addCtxSet
({
  { CNU, },
  { CNU, },
  { CNU, },
  { DWS, },
  { DWS, },
  { DWS, },
  });
const CtxSet ContextSetCfg::ChromaFusionCclm = ContextSetCfg::addCtxSet
({
  { CNU, },
  { CNU, },
  { CNU, },
  { DWS, },
  { DWS, },
  { DWS, },
  });
#endif

#else
const CtxSet ContextSetCfg::SplitFlag = ContextSetCfg::addCtxSet
({
  {  18,  27,  15,  18,  28,  45,  26,   7,  23, },
  {  11,  35,  53,  12,   6,  30,  13,  15,  31, },
  {  19,  28,  38,  27,  29,  38,  20,  30,  31, },
  {  12,  13,   8,   8,  13,  12,   5,   9,   9, },
});

const CtxSet ContextSetCfg::SplitQtFlag = ContextSetCfg::addCtxSet
({
  {  26,  36,  38,  18,  34,  21, },
  {  20,  14,  23,  18,  19,   6, },
  {  27,   6,  15,  25,  19,  37, },
  {   0,   8,   8,  12,  12,   8, },
});

const CtxSet ContextSetCfg::SplitHvFlag = ContextSetCfg::addCtxSet
({
  {  43,  42,  37,  42,  44, },
  {  43,  35,  37,  34,  52, },
  {  43,  42,  29,  27,  44, },
  {   9,   8,   9,   8,   5, },
});

const CtxSet ContextSetCfg::Split12Flag = ContextSetCfg::addCtxSet
({
  {  28,  29,  28,  29, },
  {  43,  37,  21,  22, },
  {  36,  45,  36,  45, },
  {  12,  13,  12,  13, },
});

#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
const CtxSet ContextSetCfg::ModeConsFlag = ContextSetCfg::addCtxSet
({
  {  25,  20, },
  {  25,  12, },
  { CNU, CNU, },
  {   1,   0, },
});
#endif

const CtxSet ContextSetCfg::SkipFlag = ContextSetCfg::addCtxSet
({
  {  57,  60,  46, },
  {  57,  59,  45, },
  {   0,  26,  28, },
  {   5,   4,   8, },
});

const CtxSet ContextSetCfg::MergeFlag = ContextSetCfg::addCtxSet
({
  {   6, },
  {  21, },
  {  26, },
  {   4, },
});

const CtxSet ContextSetCfg::RegularMergeFlag = ContextSetCfg::addCtxSet
({
  {  46,  15, },
  {  38,   7, },
  { CNU, CNU, },
  {   5,   5, },
});

const CtxSet ContextSetCfg::MergeIdx = ContextSetCfg::addCtxSet
({
#if NON_ADJACENT_MRG_CAND
	{ 33, 28, 36, 36, 29, 35, 35, 35, 35, 35 },
	{ 20, 21, 29, 29, 29, 35, 35, 35, 35, 35 },
	{ 34, 58, 28, 35, 25, 35, 35, 35, 35, 35 },
	{ 4,  5,  5,  4,  8,  4,  4,  4,  4,  4 }
#else
  {  18, },
  {  20, },
  {  34, },
  {   4, },
#endif
});

#if TM_MRG || (JVET_Z0084_IBC_TM && IBC_TM_MRG)
const CtxSet ContextSetCfg::TmMergeIdx = ContextSetCfg::addCtxSet
({
#if NON_ADJACENT_MRG_CAND
	{ 19, 35, 42, 35, 35, 35, 35, 35, 35, 35 },
	{ 13, 35, 42, 35, 35, 35, 35, 35, 35, 35 },
	{ 34, 35, 35, 35, 35, 35, 35, 35, 35, 35 },
	{ 4,  4,  4,  4,  4,  4,  4,  4,  4,  4 }
#else
  {  18, },
  {  20, },
  {  34, },
  {   4, },
#endif
});
#endif

const CtxSet ContextSetCfg::MmvdFlag = ContextSetCfg::addCtxSet
({
  {  25, },
  {  26, },
  { CNU, },
  {   4, },
});

const CtxSet ContextSetCfg::MmvdMergeIdx = ContextSetCfg::addCtxSet
({
  {  43, },
  {  43, },
  { CNU, },
  {  10, },
});

const CtxSet ContextSetCfg::MmvdStepMvpIdx = ContextSetCfg::addCtxSet
({
  {  59, },
  {  60, },
  { CNU, },
  {   0, },
});

#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
const CtxSet ContextSetCfg::MmvdStepMvpIdxECM3 = ContextSetCfg::addCtxSet
({
  {  59, },
  {  60, },
  { CNU, },
  {   0, },
});
#endif

#if JVET_W0097_GPM_MMVD_TM
const CtxSet ContextSetCfg::GeoMmvdFlag = ContextSetCfg::addCtxSet
({
  {  25, },
  {  26, },
  { CNU, },
  {   4, },
  });

const CtxSet ContextSetCfg::GeoMmvdStepMvpIdx = ContextSetCfg::addCtxSet
({
  {  59, },
  {  60, },
  { CNU, },
  {   0, },
  });
#endif

#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
const CtxSet ContextSetCfg::GeoSubModeIdx = ContextSetCfg::addCtxSet
({
  { 33, 28, 36, 36, 29, },
  { 20, 21, 29, 29, 29, },
  { 34, 58, 28, 35, 25, },
  {  4,  5,  5,  4,  8, }
});
#endif

#if AFFINE_MMVD
const CtxSet ContextSetCfg::AfMmvdFlag = ContextSetCfg::addCtxSet
({
	{ 18 },
	{ 11 },
	{ 35 },
	{ 4 }
});
const CtxSet ContextSetCfg::AfMmvdIdx = ContextSetCfg::addCtxSet
({
	{ 43 },
	{ 43 },
	{ 35 },
	{ 10 }
});
const CtxSet ContextSetCfg::AfMmvdOffsetStep = ContextSetCfg::addCtxSet
({
	{ 51 },
	{ 60 },
	{ 35 },
	{ 0 }
});
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
const CtxSet ContextSetCfg::AfMmvdOffsetStepECM3 = ContextSetCfg::addCtxSet
({
  { 51 },
  { 60 },
  { 35 },
  {  0 }
  });
#endif
#endif

#if JVET_AA0061_IBC_MBVD
const CtxSet ContextSetCfg::IbcMbvdFlag = ContextSetCfg::addCtxSet
({
  {  25, },
  {  26, },
  { CNU, },
  {   4, },
  });

const CtxSet ContextSetCfg::IbcMbvdMergeIdx = ContextSetCfg::addCtxSet
({
  {  43, },
  {  43, },
  { CNU, },
  {  10, },
  });

const CtxSet ContextSetCfg::IbcMbvdStepMvpIdx = ContextSetCfg::addCtxSet
({
  {  59, },
  {  60, },
  { CNU, },
  {   0, },
  });
#endif

#if TM_MRG || (JVET_Z0084_IBC_TM && IBC_TM_MRG)
const CtxSet ContextSetCfg::TMMergeFlag = ContextSetCfg::addCtxSet
({
#if JVET_Z0084_IBC_TM && IBC_TM_MRG
  {  25,  33 },
  {  26,  25 },
  { CNU, CNU },
  {   4,   5 }
#else
  {  25, },
  {  26, },
  { CNU, },
  {   4, },
#endif
});
#endif

#if TM_MRG
#if JVET_X0141_CIIP_TIMD_TM
const CtxSet ContextSetCfg::CiipTMMergeFlag = ContextSetCfg::addCtxSet
({
  {  25, },
  {  26, },
  { CNU, },
  {   4, },
  });
#endif
#endif

const CtxSet ContextSetCfg::PredMode = ContextSetCfg::addCtxSet
({
  {  40,  35, },
  {  40,  35, },
  { CNU, CNU, },
  {   5,   1, },
});

#if JVET_AB0157_TMRL
const CtxSet ContextSetCfg::TmrlDerive = ContextSetCfg::addCtxSet
({
  { CNU },
  { CNU },
  { CNU },
  { DWS },
  });
#endif

const CtxSet ContextSetCfg::MultiRefLineIdx = ContextSetCfg::addCtxSet
({
#if JVET_Y0116_EXTENDED_MRL_LIST
#if JVET_W0123_TIMD_FUSION
  { 25, 59, 59, 59, 59, 25, 59},
  { 25, 58, 58, 58, 58, 25, 58},
  { 25, 60, 60, 60, 60, 25, 60},
  { 5,  8,  8,  8,  8,  5,  8 },
#else
  { 25, 59, 59, 59, 59},
  { 25, 58, 58, 58, 58},
  { 25, 60, 60, 60, 60},
  { 5,  8,  8,  8,  8 },
#endif
#else
#if JVET_W0123_TIMD_FUSION
  {  25,  59,  25,  59, },
  {  25,  58,  25,  58, },
  {  25,  60,  25,  60, },
  {   5,   8,  5,   8, },
#else
  {  25,  59, },
  {  25,  58, },
  {  25,  60, },
  {   5,   8, },
#endif
#endif
});

const CtxSet ContextSetCfg::IntraLumaMpmFlag = ContextSetCfg::addCtxSet
({
  {  44, },
  {  36, },
  {  45, },
  {   6, },
});

#if SECONDARY_MPM
const CtxSet ContextSetCfg::IntraLumaSecondMpmFlag = ContextSetCfg::addCtxSet
({
	{ 36 },
	{ 36 },
	{ 44 },
	{ 7 }
  });
#endif

const CtxSet ContextSetCfg::IntraLumaPlanarFlag = ContextSetCfg::addCtxSet
({
  {  13,   6, },
  {  12,  20, },
  {  13,  28, },
  {   1,   5, },
});

#if SECONDARY_MPM
const CtxSet ContextSetCfg::IntraLumaMPMIdx = ContextSetCfg::addCtxSet
({
	{ 20, 21, 20 },
	{ 5, 28, 13 },
	{ 20, 44, 35 },
	{ 2,  2,  6 }
  });
#if JVET_AD0085_MPM_SORTING
const CtxSet ContextSetCfg::IntraLumaSecondMpmIdx = ContextSetCfg::addCtxSet
({
  { CNU, CNU, CNU, CNU, CNU },
  { CNU, CNU, CNU, CNU, CNU },
  { CNU, CNU, CNU, CNU, CNU },
  { DWS, DWS, DWS, DWS, DWS },
  });
#endif
#endif

const CtxSet ContextSetCfg::CclmModeFlag = ContextSetCfg::addCtxSet
({
  {  26, },
  {  34, },
  {  59, },
  {   4, },
});

const CtxSet ContextSetCfg::CclmModeIdx = ContextSetCfg::addCtxSet
({
  {  27, },
  {  27, },
  {  27, },
  {   9, },
});

const CtxSet ContextSetCfg::IntraChromaPredMode = ContextSetCfg::addCtxSet
({
  {  25, },
  {  25, },
  {  34, },
  {   5, },
});

const CtxSet ContextSetCfg::MipFlag = ContextSetCfg::addCtxSet
({
  {  56,  57,  50,  26, },
  {  41,  57,  58,  26, },
  {  33,  49,  50,  25, },
  {   9,  10,   9,   6, },
});
#if JVET_V0130_INTRA_TMP
const CtxSet ContextSetCfg::TmpFlag = ContextSetCfg::addCtxSet
({
  {  CNU,  CNU,  CNU,  CNU, },
  {  CNU,  CNU,  CNU,  CNU, },
  {  CNU,  CNU,  CNU,  CNU, },
  {  DWS,  DWS,  DWS,  DWS, },
	});
#endif


#if MMLM
const CtxSet ContextSetCfg::MMLMFlag = ContextSetCfg::addCtxSet
({
	{ 46 },
	{ 46 },
	{ 53 },
	{ 8 }
});
#endif

const CtxSet ContextSetCfg::DeltaQP = ContextSetCfg::addCtxSet
({
  { CNU, CNU, },
  { CNU, CNU, },
  { CNU, CNU, },
  { DWS, DWS, },
});

const CtxSet ContextSetCfg::InterDir = ContextSetCfg::addCtxSet
({
#if CTU_256
	{ 7,  6,  5, 12, 11,  3, 10, 40 },
	{ 7, 21,  5, 12,  4, 18, 18, 48 },
	{ 35, 35, 35, 35, 35, 35, 35, 35 },
	{ 0,  0,  0,  1,  4,  5,  5,  0 }
#else
  {  14,  13,   5,   4,   3,   3,  40, },
  {   7,   6,   5,  12,   4,   4,  40, },
  { CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
  {   0,   0,   1,   4,   4,   4,   0, },
#endif
});

const CtxSet ContextSetCfg::RefPic = ContextSetCfg::addCtxSet
({
  {   5,  35, },
  {  20,  35, },
  { CNU, CNU, },
  {   0,   4, },
});

const CtxSet ContextSetCfg::SubblockMergeFlag = ContextSetCfg::addCtxSet
({
  {  25,  58,  45, },
  {  48,  57,  44, },
  { CNU, CNU, CNU, },
  {   4,   4,   4, },
});

#if JVET_X0049_ADAPT_DMVR
const CtxSet ContextSetCfg::BMMergeFlag = ContextSetCfg::addCtxSet
({
  { 25, CNU, CNU, CNU },
  { 26, CNU, CNU, CNU },
  { CNU, CNU, CNU, CNU },
  { 4, 4, 4, 4 },
});
#endif

#if JVET_AA0070_RRIBC
const CtxSet ContextSetCfg::rribcFlipType = ContextSetCfg::addCtxSet
({
 { 25, CNU, CNU, CNU },
  { 26, CNU, CNU, CNU },
  { CNU, CNU, CNU, CNU },
  { 4, 4, 4, 4 },
  });
#endif

const CtxSet ContextSetCfg::AffineFlag = ContextSetCfg::addCtxSet
({
  {  19,  13,   6, },
  {  12,  13,  14, },
  { CNU, CNU, CNU, },
  {   4,   0,   0, },
});

const CtxSet ContextSetCfg::AffineType = ContextSetCfg::addCtxSet
({
  {  35, },
  {  35, },
  { CNU, },
  {   4, },
});

const CtxSet ContextSetCfg::AffMergeIdx = ContextSetCfg::addCtxSet
({
  {   4, },
  {   5, },
  { CNU, },
  {   0, },
});

#if INTER_LIC
const CtxSet ContextSetCfg::LICFlag = ContextSetCfg::addCtxSet
({
	{ 27 },
	{ 34 },
	{ 35 },
	{ 8 }
  });
#endif

const CtxSet ContextSetCfg::BcwIdx = ContextSetCfg::addCtxSet
({
  {   5, },
  {   4, },
  { CNU, },
  {   1, },
});

const CtxSet ContextSetCfg::Mvd = ContextSetCfg::addCtxSet
({
  {  51,  36, },
  {  44,  43, },
  {  14,  45, },
  {   9,   5, },
});

#if JVET_Z0131_IBC_BVD_BINARIZATION
const CtxSet ContextSetCfg::Bvd = ContextSetCfg::addCtxSet
({
  { 53, 38, 38, 29, 20, 34, 27, 45, 37, 43, 34, 48 },
  { 38, 38, 38, 29, 28, 42, 27, 45, 44, 28, 42, 33 },
  { 38, 38, 38, 29, 28, 42, 27, 45, 44, 28, 42, 33 },
  {  6, 10,  9,  6,  7,  7,  5,  5,  4,  1,  2,  3 }
});
#endif

#if MULTI_HYP_PRED
const CtxSet ContextSetCfg::MultiHypothesisFlag = ContextSetCfg::addCtxSet
({
  { 3, 26,CNU,},
  { 3, 26,CNU,},
  { 35, 35,CNU,},
  { 1,   4,CNU,},
  });

const CtxSet ContextSetCfg::MHRefPic = ContextSetCfg::addCtxSet
({
	{ 28, 50 },
	{ 28, 27 },
	{ 35, 35 },
	{ 1,  0 }
  });

const CtxSet ContextSetCfg::MHWeight = ContextSetCfg::addCtxSet
({
	{ 50, 35 },
	{ 12, 35 },
	{ 35, 35 },
	{ 0,  4 }
  });
#endif

const CtxSet ContextSetCfg::BDPCMMode = ContextSetCfg::addCtxSet
({
  {  19,  21,   0,  28, },
  {  40,  36,   0,  13, },
  {  19,  35,   1,  27, },
  {   1,   4,   1,   0, },
});

const CtxSet ContextSetCfg::QtRootCbf = ContextSetCfg::addCtxSet
({
  {  12, },
  {   5, },
  {   6, },
  {   4, },
});

const CtxSet ContextSetCfg::ACTFlag = ContextSetCfg::addCtxSet
({
  {  46, },
  {  46, },
  {  52, },
  {   1, },
});

const CtxSet ContextSetCfg::QtCbf[] =
{
  ContextSetCfg::addCtxSet
  ({
    {  15,   6,   5,  14, },
    {  23,   5,  20,   7, },
    {  15,  12,   5,   7, },
    {   5,   1,   8,   9, },
  }),
  ContextSetCfg::addCtxSet
  ({
    {  25,  37, },
    {  25,  28, },
    {  12,  21, },
    {   5,   0, },
  }),
  ContextSetCfg::addCtxSet
  ({
    {   9,  36,  45, },
    {  25,  29,  45, },
    {  33,  28,  36, },
    {   2,   1,   0, },
  })
};

const CtxSet ContextSetCfg::SigCoeffGroup[] =
{
  ContextSetCfg::addCtxSet
  ({
    {  25,  45, },
    {  25,  30, },
    {  18,  31, },
    {   8,   5, },
  }),
  ContextSetCfg::addCtxSet
  ({
    {  25,  14, },
    {  25,  45, },
    {  25,  15, },
    {   5,   8, },
  })
};

const CtxSet ContextSetCfg::SigFlag[] =
{
  ContextSetCfg::addCtxSet
  ({
    {  17,  41,  49,  36,   1,  49,  50,  37,  48,  51,  58,  45, },
    {  17,  41,  42,  29,  25,  49,  43,  37,  33,  58,  51,  30, },
    {  25,  19,  28,  14,  25,  20,  29,  30,  19,  37,  30,  38, },
    {  12,   9,   9,  10,   9,   9,   9,  10,   8,   8,   8,  10, },
  }),
  ContextSetCfg::addCtxSet
  ({
    {   9,  49,  50,  36,  48,  59,  59,  38, },
    {  17,  34,  35,  21,  41,  59,  60,  38, },
    {  25,  27,  28,  37,  34,  53,  53,  46, },
    {  12,  12,   9,  13,   4,   5,   8,   9, },
  }),
  ContextSetCfg::addCtxSet
  ({
    {  26,  45,  53,  46,  49,  54,  61,  39,  35,  39,  39,  39, },
    {  19,  38,  38,  46,  34,  54,  54,  39,   6,  39,  39,  39, },
    {  11,  38,  46,  54,  27,  39,  39,  39,  44,  39,  39,  39, },
    {   9,  13,   8,   8,   8,   8,   8,   5,   8,   0,   0,   0, },
  }),
  ContextSetCfg::addCtxSet
  ({
    {  34,  45,  38,  31,  58,  39,  39,  39, },
    {  35,  45,  53,  54,  44,  39,  39,  39, },
    {  19,  46,  38,  39,  52,  39,  39,  39, },
    {   8,  12,  12,   8,   4,   0,   0,   0, },
  }),
  ContextSetCfg::addCtxSet
  ({
    {  19,  54,  39,  39,  50,  39,  39,  39,   0,  39,  39,  39, },
    {  19,  39,  54,  39,  19,  39,  39,  39,  56,  39,  39,  39, },
    {  18,  39,  39,  39,  27,  39,  39,  39,   0,  39,  39,  39, },
    {   8,   8,   8,   8,   8,   0,   4,   4,   0,   0,   0,   0, },
  }),
  ContextSetCfg::addCtxSet
  ({
    {  34,  38,  54,  39,  41,  39,  39,  39, },
    {  34,  38,  62,  39,  26,  39,  39,  39, },
    {  11,  39,  39,  39,  19,  39,  39,  39, },
    {   8,   8,   8,   8,   4,   0,   0,   0, },
  })
};

const CtxSet ContextSetCfg::ParFlag[] =
{
  ContextSetCfg::addCtxSet
  ({
    {  33,  40,  25,  41,  26,  42,  25,  33,  26,  34,  27,  25,  41,  42,  42,  35,  33,  27,  35,  42,  43, },
    {  18,  17,  33,  18,  26,  42,  25,  33,  26,  42,  27,  25,  34,  42,  42,  35,  26,  27,  42,  20,  20, },
    {  33,  25,  18,  26,  34,  27,  25,  26,  19,  42,  35,  33,  19,  27,  35,  35,  34,  42,  20,  43,  20, },
    {   8,   9,  12,  13,  13,  13,  10,  13,  13,  13,  13,  13,  13,  13,  13,  13,  10,  13,  13,  13,  13, },
  }),
  ContextSetCfg::addCtxSet
  ({
    {  33,  25,  26,  34,  19,  27,  33,  42,  43,  35,  43, },
    {  25,  25,  26,  11,  19,  27,  33,  42,  35,  35,  43, },
    {  33,  25,  26,  42,  19,  27,  26,  50,  35,  20,  43, },
    {   8,  12,  12,  12,  13,  13,  13,  13,  13,  13,  13, },
  })
};

const CtxSet ContextSetCfg::GtxFlag[] =
{
  ContextSetCfg::addCtxSet
  ({
    {  25,   0,   0,  17,  25,  26,   0,   9,  25,  33,  19,   0,  25,  33,  26,  20,  25,  33,  27,  35,  22, },
    {  17,   0,   1,  17,  25,  18,   0,   9,  25,  33,  34,   9,  25,  18,  26,  20,  25,  18,  19,  27,  29, },
    {  25,   1,  40,  25,  33,  11,  17,  25,  25,  18,   4,  17,  33,  26,  19,  13,  33,  19,  20,  28,  22, },
    {   1,   5,   9,   9,   9,   6,   5,   9,  10,  10,   9,   9,   9,   9,   9,   9,   6,   8,   9,   9,  10, },
  }),
  ContextSetCfg::addCtxSet
  ({
    {  25,   1,  25,  33,  26,  12,  25,  33,  27,  28,  37, },
    {  17,   9,  25,  10,  18,   4,  17,  33,  19,  20,  29, },
    {  40,   9,  25,  18,  26,  35,  25,  26,  35,  28,  37, },
    {   1,   5,   8,   8,   9,   6,   6,   9,   8,   8,   9, },
  }),
  ContextSetCfg::addCtxSet
  ({
    {   0,   0,  33,  34,  35,  21,  25,  34,  35,  28,  29,  40,  42,  43,  29,  30,  49,  36,  37,  45,  38, },
    {   0,  17,  26,  19,  35,  21,  25,  34,  20,  28,  29,  33,  27,  28,  29,  22,  34,  28,  44,  37,  38, },
    {  25,  25,  11,  27,  20,  21,  33,  12,  28,  21,  22,  34,  28,  29,  29,  30,  36,  29,  45,  30,  23, },
    {   9,   5,  10,  13,  13,  10,   9,  10,  13,  13,  13,   9,  10,  10,  10,  13,   8,   9,  10,  10,  13, },
  }),
  ContextSetCfg::addCtxSet
  ({
    {   0,  40,  34,  43,  36,  37,  57,  52,  45,  38,  46, },
    {   0,  25,  19,  20,  13,  14,  57,  44,  30,  30,  23, },
    {  40,  33,  27,  28,  21,  37,  36,  37,  45,  38,  46, },
    {   8,   8,   9,  12,  12,  10,   5,   9,   9,   9,  13, },
  })
};

const CtxSet ContextSetCfg::LastX[] =
{
  ContextSetCfg::addCtxSet
  ({
#if TU_256
    {  6,  6, 12, 14,  6,  4,  6,  7,  6,  4, 14,  7,  6,  6, 12, 21,  7,  6,  6, 37, 43, 35,  6, 15, 53, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35 },
    {  6, 13, 12,  6,  6, 12, 14,  6, 13, 12, 29, 14,  6,  6,  6, 21, 21, 28,  6,  6, 37,  6,  6, 15, 31, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26 },
    { 13,  5,  4,  6,  6, 12,  6, 14, 29,  4, 14,  7, 22, 29,  4, 22, 38, 15, 22,  6, 19,  7, 37,  7, 13, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42 },
    {  5,  5,  4,  5,  4,  4,  5,  4,  1,  0,  5,  1,  0,  0,  0,  1,  1,  0,  0,  0,  0,  0,  4,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 }
#else
    {   6,   6,  12,  14,   6,   4,  14,   7,   6,   4,  29,   7,   6,   6,  12,  28,   7,  13,  13,  35, }, // 20 contexts
    {   6,  13,  12,   6,   6,  12,  14,  14,  13,  12,  29,   7,   6,  13,  36,  28,  14,  13,   5,  26, },
    {  13,   5,   4,  21,  14,   4,   6,  14,  21,  11,  14,   7,  14,   5,  11,  21,  30,  22,  13,  42, },
    {   8,   5,   4,   5,   4,   4,   5,   4,   1,   0,   4,   1,   0,   0,   0,   0,   1,   0,   0,   0, },
#endif
  }),
  ContextSetCfg::addCtxSet
  ({
    {  19,   5,   4, },
    {  12,   4,  18, },
    {  12,   4,   3, },
    {   5,   4,   4, },
  })
};

const CtxSet ContextSetCfg::LastY[] =
{
  ContextSetCfg::addCtxSet
  ({
#if TU_256
    {  5,  5, 20, 13, 13, 19,  6,  6, 12, 20, 14, 14,  5,  4, 20,  6,  7,  6, 12, 57, 51, 42, 13, 20,  5, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41, 41 },
    {  5,  5, 12,  6,  6, 19,  6,  6,  5, 12, 14,  7, 13,  5, 37, 21,  7, 28, 20, 37, 37, 28, 13,  6, 15, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34 },
    { 13,  5,  4,  6,  6, 11, 14, 14,  5, 11, 14,  7,  6,  5, 11, 22, 38, 22,  6, 20, 19, 51, 15,  6,  4, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34 },
    {  8,  5,  8,  5,  5,  4,  5,  5,  4,  0,  5,  5,  1,  0,  0,  1,  5,  0,  0,  0,  0,  0,  4,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 }
#else
    {   5,   5,  20,  13,  13,  19,  21,   6,  12,  12,  14,  14,   5,   4,  12,  13,   7,  13,  12,  41, }, // 20 contexts
    {   5,   5,  12,   6,   6,   4,   6,  14,   5,  12,  14,   7,  13,   5,  13,  21,  14,  20,  12,  34, },
    {  13,   5,   4,   6,  13,  11,  14,   6,   5,   3,  14,  22,   6,   4,   3,   6,  22,  29,  20,  34, },
    {   8,   5,   8,   5,   5,   4,   5,   5,   4,   0,   5,   4,   1,   0,   0,   1,   4,   0,   0,   0, },
#endif
  }),
  ContextSetCfg::addCtxSet
  ({
    {  11,   5,  27, },
    {  11,   4,  18, },
    {  12,   4,   3, },
    {   6,   5,   5, },
  })
};

const CtxSet ContextSetCfg::MVPIdx = ContextSetCfg::addCtxSet
({
  {  34, },
  {  34, },
  {  42, },
  {  12, },
});

const CtxSet ContextSetCfg::SmvdFlag = ContextSetCfg::addCtxSet
({
  {  28, },
  {  28, },
  { CNU, },
  {   5, },
});

const CtxSet ContextSetCfg::SaoMergeFlag = ContextSetCfg::addCtxSet
({
  {   2, },
  {  60, },
  {  60, },
  {   0, },
});

const CtxSet ContextSetCfg::SaoTypeIdx = ContextSetCfg::addCtxSet
({
  {   2, },
  {   5, },
  {  13, },
  {   4, },
});

#if JVET_V0094_BILATERAL_FILTER
const CtxSet ContextSetCfg::BifCtrlFlags[] =
{
ContextSetCfg::addCtxSet
( {
  { 39, },
  { 39, },
  { 39, },
  { DWS, },
} ),
#if JVET_X0071_CHROMA_BILATERAL_FILTER
ContextSetCfg::addCtxSet
( {
    { 39, },
    { 39, },
    { 39, },
    { DWS, },
} ),
ContextSetCfg::addCtxSet
( {
    { 39, },
    { 39, },
    { 39, },
    { DWS, },
} )
#endif
}
#endif

#if JVET_W0066_CCSAO
const CtxSet ContextSetCfg::CcSaoControlIdc = ContextSetCfg::addCtxSet
({
  { CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
  { CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
  { CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
  { DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, },
});
#endif

const CtxSet ContextSetCfg::LFNSTIdx = ContextSetCfg::addCtxSet
({
#if EXTENDED_LFNST || JVET_W0119_LFNST_EXTENSION
  { 58, 37, 42, 35 },
  { 43, 45, 42, 35 },
  { 28, 43, 42, 27 },
  {  9, 10,  9, 10 }
#else
  {  52,  37,  27, },
  {  37,  45,  27, },
  {  28,  52,  42, },
  {   9,   9,  10, },
#endif
});

const CtxSet ContextSetCfg::PLTFlag = ContextSetCfg::addCtxSet
({
  {  17, },
  {   0, },
  {  25, },
  {   1, },
});

const CtxSet ContextSetCfg::RotationFlag = ContextSetCfg::addCtxSet
({
  {  35, },
  {  42, },
  {  42, },
  {   5, },
});

const CtxSet ContextSetCfg::RunTypeFlag = ContextSetCfg::addCtxSet
({
  {  50, },
  {  59, },
  {  42, },
  {   9, },
});

const CtxSet ContextSetCfg::IdxRunModel = ContextSetCfg::addCtxSet
({
  {  58,  45,  45,  30,  38, },
  {  51,  30,  30,  38,  23, },
  {  50,  37,  45,  30,  46, },
  {   9,   6,   9,  10,   5, },
});

const CtxSet ContextSetCfg::CopyRunModel = ContextSetCfg::addCtxSet
({
  {  45,  38,  46, },
  {  38,  53,  46, },
  {  45,  38,  46, },
  {   0,   9,   5, },
});

const CtxSet ContextSetCfg::TransformSkipFlag = ContextSetCfg::addCtxSet
({
  {  25,  17, },
  {  25,   9, },
  {  25,   9, },
  {   1,   1, },
});
#if JVET_W0103_INTRA_MTS
const CtxSet ContextSetCfg::MTSIdx = ContextSetCfg::addCtxSet
({
  { 45, 35, 20, 45, },
  { 38, 35, 35, 38, },
  { 37, 28, 28, 37, },
  { 8,   9,  9, 8,  },
  });
#else
const CtxSet ContextSetCfg::MTSIdx = ContextSetCfg::addCtxSet
({
  {  45,  25,  27,   0, },
  {  45,  40,  27,   0, },
  {  29,   0,  28,   0, },
  {   8,   0,   9,   0, },
});
#endif
const CtxSet ContextSetCfg::ISPMode = ContextSetCfg::addCtxSet
({
#if JVET_W0123_TIMD_FUSION
  {  33,  43,  33, },
  {  33,  36,  33, },
  {  33,  43,  33, },
  {   9,   2,   9, },
#else
  {  33,  43, },
  {  33,  36, },
  {  33,  43, },
  {   9,   2, },
#endif
});

const CtxSet ContextSetCfg::SbtFlag = ContextSetCfg::addCtxSet
({
  {  41,  57, },
  {  56,  57, },
  { CNU, CNU, },
  {   1,   5, },
});

const CtxSet ContextSetCfg::SbtQuadFlag = ContextSetCfg::addCtxSet
({
  {  42, },
  {  42, },
  { CNU, },
  {  10, },
});

const CtxSet ContextSetCfg::SbtHorFlag = ContextSetCfg::addCtxSet
({
  {  35,  51,  27, },
  {  20,  43,  12, },
  { CNU, CNU, CNU, },
  {   8,   4,   1, },
});

const CtxSet ContextSetCfg::SbtPosFlag = ContextSetCfg::addCtxSet
({
  {  28, },
  {  28, },
  { CNU, },
  {  13, },
});

const CtxSet ContextSetCfg::ChromaQpAdjFlag = ContextSetCfg::addCtxSet
({
  { CNU, },
  { CNU, },
  { CNU, },
  { DWS, },
});
#if ENABLE_DIMD
const CtxSet ContextSetCfg::DimdFlag = ContextSetCfg::addCtxSet
({
  { 48, 56, 56 },
  { 41, 49, 49 },
  { 33, 49, 49 },
  {  5,  1,  1 }
  });
#endif

#if JVET_W0123_TIMD_FUSION
const CtxSet ContextSetCfg::TimdFlag = ContextSetCfg::addCtxSet
({
  { 48, 56, 56 },
  { 41, 49, 49 },
  { 33, 49, 49 },
  {  5,  1,  1 }
});
#endif

#if ENABLE_OBMC
const CtxSet ContextSetCfg::ObmcFlag = ContextSetCfg::addCtxSet
({
	{ 62 },
	{ 39 },
	{ 35 },
	{ 4 }
  });
#endif
const CtxSet ContextSetCfg::ChromaQpAdjIdc = ContextSetCfg::addCtxSet
({
  { CNU, },
  { CNU, },
  { CNU, },
  { DWS, },
});

const CtxSet ContextSetCfg::ImvFlag = ContextSetCfg::addCtxSet
({
  {  59,  26,  50,  60,  38, },
  {  59,  48,  58,  60,  60, },
  { CNU,  34, CNU, CNU, CNU, },
  {   0,   5,   0,   0,   4, },
});

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
const CtxSet ContextSetCfg::ImvFlagIBC = ContextSetCfg::addCtxSet
({
  {  59,  26,  50,  60,  38, },
  {  59,  48,  58,  60,  60, },
  {  59,  26,  50,  60,  38, },
  {   0,   5,   0,   0,   4, },
});
#endif

const CtxSet ContextSetCfg::ctbAlfFlag = ContextSetCfg::addCtxSet
({
  {  33,  52,  46,  25,  61,  54,  25,  61,  54, },
  {  13,  23,  46,   4,  61,  54,  19,  46,  54, },
  {  62,  39,  39,  54,  39,  39,  31,  39,  39, },
  {   0,   0,   0,   4,   0,   0,   1,   0,   0, },
});

const CtxSet ContextSetCfg::ctbAlfAlternative = ContextSetCfg::addCtxSet
({
#if ALF_IMPROVEMENT
  {  4, 19, 19 },
  { 35, 35, 20 },
  { 26, 19, 19 },
  {  0,  0,  0 }
#else
  {  11,  26, },
  {  20,  12, },
  {  11,  11, },
  {   0,   0, },
#endif
});

const CtxSet ContextSetCfg::AlfUseTemporalFilt = ContextSetCfg::addCtxSet
({
  {  46, },
  {  46, },
  {  46, },
  {   0, },
});

const CtxSet ContextSetCfg::CcAlfFilterControlFlag = ContextSetCfg::addCtxSet
({
  {  25,  35,  38,  25,  28,  38, },
  {  18,  21,  38,  18,  21,  38, },
  {  18,  30,  31,  18,  30,  31, },
  {   4,   1,   4,   4,   1,   4, },
});

const CtxSet ContextSetCfg::CiipFlag = ContextSetCfg::addCtxSet
({
#if CIIP_PDPC
  { 50, 21 },
  { 50, 36 },
  { 35, 35 },
  {  1,  2 }
#else
  {  57, },
  {  57, },
  { CNU, },
  {   1, },
#endif
});

const CtxSet ContextSetCfg::IBCFlag = ContextSetCfg::addCtxSet
({
  {   0,  43,  45, },
  {   0,  57,  44, },
  {  17,  42,  36, },
  {   1,   5,   8, },
});

const CtxSet ContextSetCfg::JointCbCrFlag = ContextSetCfg::addCtxSet
({
  {  42,  43,  52, },
  {  27,  36,  45, },
  {  12,  21,  35, },
  {   1,   1,   0, },
});

const CtxSet ContextSetCfg::TsSigCoeffGroup = ContextSetCfg::addCtxSet
({
  {  18,  35,  45, },
  {  18,  12,  29, },
  {  18,  20,  38, },
  {   5,   8,   8, },
});

const CtxSet ContextSetCfg::TsSigFlag = ContextSetCfg::addCtxSet
({
  {  25,  50,  37, },
  {  40,  35,  44, },
  {  25,  28,  38, },
  {  13,  13,   8, },
});

const CtxSet ContextSetCfg::TsParFlag = ContextSetCfg::addCtxSet
({
  {  11, },
  {   3, },
  {  11, },
  {   6, },
});

const CtxSet ContextSetCfg::TsGtxFlag = ContextSetCfg::addCtxSet
({
  { CNU,   3,   4,   4,   5, },
  { CNU,   2,  10,   3,   3, },
  { CNU,  10,   3,   3,   3, },
  { DWS,   1,   1,   1,   1, },
});

const CtxSet ContextSetCfg::TsLrg1Flag = ContextSetCfg::addCtxSet
({
  {  19,  11,   4,   6, },
  {  18,  11,   4,  28, },
  {  11,   5,   5,  14, },
  {   4,   2,   1,   6, },
});

const CtxSet ContextSetCfg::TsResidualSign = ContextSetCfg::addCtxSet
({
  {  35,  25,  46,  28,  33,  38, },
  {   5,  10,  53,  43,  25,  46, },
  {  12,  17,  46,  28,  25,  46, },
  {   1,   4,   4,   5,   8,   8, },
});

#if SIGN_PREDICTION
const CtxSet ContextSetCfg::signPred[2] =
{
  ContextSetCfg::addCtxSet
  ( {
    { 34, 34, 34, 34 },
    { 34, 34, 34, 34 },
    { 34, 34, 34, 34 },
    {  9,  9,  9,  9 }
    } ),

  ContextSetCfg::addCtxSet
  ( {
    { 34, 34, 34, 34 },
    { 34, 34, 34, 34 },
    { 49, 49, 49, 49 },
    {  9,  9,  9,  9 }
    } )
};
#endif

#if JVET_Z0050_CCLM_SLOPE
const CtxSet ContextSetCfg::CclmDeltaFlags = ContextSetCfg::addCtxSet
({
  {  CNU, CNU, CNU, CNU, CNU, },
  {  CNU, CNU, CNU, CNU, CNU, },
  {  CNU, CNU, CNU, CNU, CNU, },
  {  DWS, DWS, DWS, DWS, DWS, },
});
#endif

#if JVET_AA0126_GLM
const CtxSet ContextSetCfg::GlmFlags = ContextSetCfg::addCtxSet
({
  {  CNU, CNU, CNU, CNU, CNU, },
  {  CNU, CNU, CNU, CNU, CNU, },
  {  CNU, CNU, CNU, CNU, CNU, },
  {  DWS, DWS, DWS, DWS, DWS, },
});
#endif

#if JVET_AA0057_CCCM
const CtxSet ContextSetCfg::CccmFlag = ContextSetCfg::addCtxSet
({
#if JVET_AC0147_CCCM_NO_SUBSAMPLING && JVET_AC0054_GLCCCM
  { CNU, CNU, CNU, },
  { CNU, CNU, CNU, },
  { CNU, CNU, CNU, },
  { DWS, DWS, DWS, },
#elif JVET_AC0147_CCCM_NO_SUBSAMPLING || JVET_AC0054_GLCCCM
  { CNU, CNU, },
  { CNU, CNU, },
  { CNU, CNU, },
  { DWS, DWS, },
#else
  { CNU, },
  { CNU, },
  { CNU, },
  { DWS, },
#endif
});
#if JVET_AE0100_BVGCCCM
const CtxSet ContextSetCfg::BvgCccmFlag = ContextSetCfg::addCtxSet
({
  { CNU, },
  { CNU, },
  { CNU, },
  { DWS, },
});
#endif
#endif

#if JVET_AC0119_LM_CHROMA_FUSION
const CtxSet ContextSetCfg::ChromaFusionType = ContextSetCfg::addCtxSet
({
  { CNU, },
  { CNU, },
  { CNU, },
  { DWS, },
  });
const CtxSet ContextSetCfg::ChromaFusionCclm = ContextSetCfg::addCtxSet
({
  { CNU, },
  { CNU, },
  { CNU, },
  { DWS, },
  });
#endif

#endif
// clang-format on

const unsigned ContextSetCfg::NumberOfContexts = (unsigned)ContextSetCfg::sm_InitTables[0].size();


// combined sets
const CtxSet ContextSetCfg::Palette = { ContextSetCfg::RotationFlag, ContextSetCfg::RunTypeFlag, ContextSetCfg::IdxRunModel, ContextSetCfg::CopyRunModel };
const CtxSet ContextSetCfg::Sao = { ContextSetCfg::SaoMergeFlag, ContextSetCfg::SaoTypeIdx };

const CtxSet ContextSetCfg::Alf = { ContextSetCfg::ctbAlfFlag, ContextSetCfg::ctbAlfAlternative, ContextSetCfg::AlfUseTemporalFilt };
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
const CtxSet ContextSetCfg::Split = { ContextSetCfg::SplitFlag, ContextSetCfg::SplitQtFlag, ContextSetCfg::SplitHvFlag, ContextSetCfg::Split12Flag };
#else
const CtxSet ContextSetCfg::Split = { ContextSetCfg::SplitFlag, ContextSetCfg::SplitQtFlag, ContextSetCfg::SplitHvFlag, ContextSetCfg::Split12Flag, ContextSetCfg::ModeConsFlag };
#endif

template <class BinProbModel>
CtxStore<BinProbModel>::CtxStore()
  : m_ctxBuffer ()
  , m_ctx       ( nullptr )
{}

template <class BinProbModel>
CtxStore<BinProbModel>::CtxStore( bool dummy )
  : m_ctxBuffer ( ContextSetCfg::NumberOfContexts )
  , m_ctx       ( m_ctxBuffer.data() )
{}

template <class BinProbModel>
CtxStore<BinProbModel>::CtxStore( const CtxStore<BinProbModel>& ctxStore )
  : m_ctxBuffer ( ctxStore.m_ctxBuffer )
  , m_ctx       ( m_ctxBuffer.data() )
{}

template <class BinProbModel>
void CtxStore<BinProbModel>::init( int qp, int initId )
{
  const std::vector<uint8_t>& initTable = ContextSetCfg::getInitTable( initId );
  CHECK( m_ctxBuffer.size() != initTable.size(),
        "Size of init table (" << initTable.size() << ") does not match size of context buffer (" << m_ctxBuffer.size() << ")." );
#if SLICE_TYPE_WIN_SIZE
  const std::vector<uint8_t> &rateInitTable = ContextSetCfg::getInitTable(NUMBER_OF_SLICE_TYPES + initId);
#else
  const std::vector<uint8_t> &rateInitTable = ContextSetCfg::getInitTable(NUMBER_OF_SLICE_TYPES);
#endif
  CHECK(m_ctxBuffer.size() != rateInitTable.size(),
        "Size of rate init table (" << rateInitTable.size() << ") does not match size of context buffer ("
                                    << m_ctxBuffer.size() << ").");
#if JVET_Z0135_TEMP_CABAC_WIN_WEIGHT
  const std::vector<uint8_t> &weightInitTable = ContextSetCfg::getInitTable( (NUMBER_OF_SLICE_TYPES << 1) + initId );
  CHECK( m_ctxBuffer.size() != weightInitTable.size(),
         "Size of weight init table (" << weightInitTable.size() << ") does not match size of context buffer (" << m_ctxBuffer.size() << ")." );

  const std::vector<uint8_t> &rateOffsetInitTable0 = ContextSetCfg::getInitTable((NUMBER_OF_SLICE_TYPES * 3));
  const std::vector<uint8_t> &rateOffsetInitTable1 = ContextSetCfg::getInitTable((NUMBER_OF_SLICE_TYPES * 3) + 1);

  CHECK(m_ctxBuffer.size() != rateOffsetInitTable0.size(),
        "Size of weight init table (" << rateOffsetInitTable0.size() << ") does not match size of context buffer ("
                                      << m_ctxBuffer.size() << ").");
  CHECK(m_ctxBuffer.size() != rateOffsetInitTable1.size(),
        "Size of weight init table (" << rateOffsetInitTable1.size() << ") does not match size of context buffer ("
                                      << m_ctxBuffer.size() << ").");
#endif


  int clippedQP = Clip3( 0, MAX_QP, qp );
  for( std::size_t k = 0; k < m_ctxBuffer.size(); k++ )
  {
    m_ctxBuffer[k].init( clippedQP, initTable[k] );
    m_ctxBuffer[k].setLog2WindowSize(rateInitTable[k]);

#if JVET_Z0135_TEMP_CABAC_WIN_WEIGHT
    m_ctxBuffer[k].setAdaptRateOffset(rateOffsetInitTable0[k], 0);
    m_ctxBuffer[k].setAdaptRateOffset(rateOffsetInitTable1[k], 1);
    m_ctxBuffer[k].setAdaptRateWeight( weightInitTable[k] );
#endif
  }
}

#if JVET_Z0135_TEMP_CABAC_WIN_WEIGHT
template <class BinProbModel>
void CtxStore<BinProbModel>::saveWinSizes( std::vector<uint8_t>& windows ) const
{
  windows.resize( m_ctxBuffer.size(), uint8_t( 0 ) );

  for( std::size_t k = 0; k < m_ctxBuffer.size(); k++ )
  {
    windows[k] = m_ctxBuffer[k].getWinSizes();
  }
}

template <class BinProbModel>
void CtxStore<BinProbModel>::loadWinSizes( const std::vector<uint8_t>& windows )
{
  CHECK( m_ctxBuffer.size() != windows.size(),
         "Size of prob states table (" << windows.size() << ") does not match size of context buffer (" << m_ctxBuffer.size() << ")." );
  for( std::size_t k = 0; k < m_ctxBuffer.size(); k++ )
  {
    m_ctxBuffer[k].setWinSizes( windows[k] );
  }
}

template <class BinProbModel>
void CtxStore<BinProbModel>::loadWeights( const std::vector<uint8_t>& weights )
{
  CHECK( m_ctxBuffer.size() != weights.size(),
         "Size of prob states table (" << weights.size() << ") does not match size of context buffer (" << m_ctxBuffer.size() << ")." );
  for( std::size_t k = 0; k < m_ctxBuffer.size(); k++ )
  {
    m_ctxBuffer[k].setAdaptRateWeight( weights[k] );
  }
}

template <class BinProbModel>
void CtxStore<BinProbModel>::saveWeights( std::vector<uint8_t>& weights ) const
{
  weights.resize( m_ctxBuffer.size(), uint8_t( 0 ) );

  for( std::size_t k = 0; k < m_ctxBuffer.size(); k++ )
  {
    weights[k] = m_ctxBuffer[k].getAdaptRateWeight();
  }
}

template <class BinProbModel>
void CtxStore<BinProbModel>::loadPStates( const std::vector<std::pair<uint16_t, uint16_t>>& probStates )
{
  CHECK( m_ctxBuffer.size() != probStates.size(),
         "Size of prob states table (" << probStates.size() << ") does not match size of context buffer (" << m_ctxBuffer.size() << ")." );
  for( std::size_t k = 0; k < m_ctxBuffer.size(); k++ )
  {
    m_ctxBuffer[k].setState( probStates[k] );
  }
}

template <class BinProbModel>
void CtxStore<BinProbModel>::savePStates( std::vector<std::pair<uint16_t, uint16_t>>& probStates ) const
{
  probStates.resize( m_ctxBuffer.size(), std::pair<uint16_t, uint16_t>( 0, 0 ) );

  for( std::size_t k = 0; k < m_ctxBuffer.size(); k++ )
  {
    probStates[k] = m_ctxBuffer[k].getState();
  }
}
#else
template <class BinProbModel>
void CtxStore<BinProbModel>::loadPStates( const std::vector<uint16_t>& probStates )
{
  CHECK( m_ctxBuffer.size() != probStates.size(),
        "Size of prob states table (" << probStates.size() << ") does not match size of context buffer (" << m_ctxBuffer.size() << ")." );
  for( std::size_t k = 0; k < m_ctxBuffer.size(); k++ )
  {
    m_ctxBuffer[k].setState( probStates[k] );
  }
}

template <class BinProbModel>
void CtxStore<BinProbModel>::savePStates( std::vector<uint16_t>& probStates ) const
{
  probStates.resize( m_ctxBuffer.size(), uint16_t(0) );
  for( std::size_t k = 0; k < m_ctxBuffer.size(); k++ )
  {
    probStates[k] = m_ctxBuffer[k].getState();
  }
}
#endif



template class CtxStore<BinProbModel_Std>;





Ctx::Ctx()                                  : m_BPMType( BPM_Undefined )                        {}
Ctx::Ctx( const BinProbModel_Std*   dummy ) : m_BPMType( BPM_Std   ), m_ctxStore_Std  ( true )  {}

Ctx::Ctx( const Ctx& ctx )
  : m_BPMType         ( ctx.m_BPMType )
  , m_ctxStore_Std    ( ctx.m_ctxStore_Std    )
{
  ::memcpy( m_GRAdaptStats, ctx.m_GRAdaptStats, sizeof( unsigned ) * RExt__GOLOMB_RICE_ADAPTATION_STATISTICS_SETS );
}

