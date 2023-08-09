/**
  ******************************************************************************
  * @file    Libraries/Driver/include/LL/tx_phe_ll_fft.h
  * @author  HUGE-IC Application Team
  * @version V1.0.0
  * @date    01-08-2018
  * @brief   This file contains all the FFT LL firmware functions.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2018 HUGE-IC</center></h2>                                                      
  *
  * Description of the FFT module:
  *
  * 1. Windowing module:
  *    The FFT contains two windowed coefficient tables, the hanning window and the nuttallwin window. 
  * Of course, you can also pass the windowed coefficients to the FFT module via memory, but if you 
  * want to use the internal window coefficients, you can refer to the array fft_hanning_window and 
  * the array fft_nuttallwin_window. These two arrays contain specific coefficients, both of which 
  * are 1024 bytes.
  *
  * u16 fft_hanning_window[1024] = {
  *    0,     0,     1,     1,     2,     4,     6,     8,     10,    12,    15,    19,   
  *    22,    26,    30,    35,    39,    45,    50,    56,    62,    68,    75,    81,   
  *    89,    96,    104,   112,   121,   129,   138,   148,   157,   167,   178,   188,  
  *    199,   210,   222,   233,   246,   258,   271,   283,   297,   310,   324,   338,  
  *    353,   367,   383,   398,   413,   429,   446,   462,   479,   496,   513,   531,  
  *    549,   567,   586,   604,   624,   643,   663,   683,   703,   723,   744,   765,  
  *    787,   808,   830,   852,   875,   897,   920,   944,   967,   991,   1015,  1040, 
  *    1064,  1089,  1114,  1140,  1165,  1191,  1218,  1244,  1271,  1298,  1325,  1353, 
  *    1381,  1409,  1437,  1466,  1494,  1523,  1553,  1582,  1612,  1642,  1673,  1703, 
  *    1734,  1765,  1796,  1828,  1859,  1892,  1924,  1956,  1989,  2022,  2055,  2088, 
  *    2122,  2156,  2190,  2224,  2259,  2294,  2329,  2364,  2399,  2435,  2471,  2507, 
  *    2543,  2580,  2617,  2653,  2691,  2728,  2765,  2803,  2841,  2879,  2918,  2956, 
  *    2995,  3034,  3073,  3112,  3152,  3192,  3232,  3272,  3312,  3352,  3393,  3434, 
  *    3475,  3516,  3558,  3599,  3641,  3683,  3725,  3767,  3809,  3852,  3895,  3937, 
  *    3980,  4024,  4067,  4111,  4154,  4198,  4242,  4286,  4330,  4375,  4419,  4464, 
  *    4509,  4554,  4599,  4644,  4689,  4735,  4781,  4826,  4872,  4918,  4964,  5011, 
  *    5057,  5104,  5150,  5197,  5244,  5291,  5338,  5385,  5432,  5480,  5527,  5575, 
  *    5622,  5670,  5718,  5766,  5814,  5862,  5910,  5959,  6007,  6056,  6104,  6153, 
  *    6202,  6250,  6299,  6348,  6397,  6446,  6495,  6545,  6594,  6643,  6693,  6742, 
  *    6791,  6841,  6891,  6940,  6990,  7040,  7090,  7139,  7189,  7239,  7289,  7339, 
  *    7389,  7439,  7489,  7539,  7589,  7639,  7690,  7740,  7790,  7840,  7890,  7941, 
  *    7991,  8041,  8091,  8142,  8192,  8242,  8293,  8343,  8393,  8443,  8494,  8544, 
  *    8594,  8644,  8694,  8745,  8795,  8845,  8895,  8945,  8995,  9045,  9095,  9145, 
  *    9195,  9245,  9294,  9344,  9394,  9444,  9493,  9543,  9593,  9642,  9691,  9741, 
  *    9790,  9839,  9889,  9938,  9987,  10036, 10085, 10134, 10182, 10231, 10280, 10328,
  *    10377, 10425, 10474, 10522, 10570, 10618, 10666, 10714, 10762, 10809, 10857, 10904,
  *    10952, 10999, 11046, 11093, 11140, 11187, 11234, 11280, 11327, 11373, 11420, 11466,
  *    11512, 11558, 11603, 11649, 11695, 11740, 11785, 11830, 11875, 11920, 11965, 12009,
  *    12054, 12098, 12142, 12186, 12230, 12273, 12317, 12360, 12404, 12447, 12489, 12532,
  *    12575, 12617, 12659, 12701, 12743, 12785, 12826, 12868, 12909, 12950, 12991, 13032,
  *    13072, 13112, 13152, 13192, 13232, 13272, 13311, 13350, 13389, 13428, 13466, 13505,
  *    13543, 13581, 13619, 13656, 13693, 13731, 13767, 13804, 13841, 13877, 13913, 13949,
  *    13985, 14020, 14055, 14090, 14125, 14160, 14194, 14228, 14262, 14296, 14329, 14362,
  *    14395, 14428, 14460, 14492, 14525, 14556, 14588, 14619, 14650, 14681, 14711, 14742,
  *    14772, 14802, 14831, 14861, 14890, 14918, 14947, 14975, 15003, 15031, 15059, 15086,
  *    15113, 15140, 15166, 15193, 15219, 15244, 15270, 15295, 15320, 15344, 15369, 15393,
  *    15417, 15440, 15464, 15487, 15509, 15532, 15554, 15576, 15597, 15619, 15640, 15661,
  *    15681, 15701, 15721, 15741, 15760, 15780, 15798, 15817, 15835, 15853, 15871, 15888,
  *    15905, 15922, 15938, 15955, 15971, 15986, 16001, 16017, 16031, 16046, 16060, 16074,
  *    16087, 16101, 16113, 16126, 16138, 16151, 16162, 16174, 16185, 16196, 16206, 16217,
  *    16227, 16236, 16246, 16255, 16263, 16272, 16280, 16288, 16295, 16303, 16309, 16316,
  *    16322, 16328, 16334, 16339, 16345, 16349, 16354, 16358, 16362, 16365, 16369, 16372,
  *    16374, 16376, 16378, 16380, 16382, 16383, 16383, 16384, 16384, 16384, 16383, 16383,
  *    16382, 16380, 16378, 16376, 16374, 16372, 16369, 16365, 16362, 16358, 16354, 16349,
  *    16345, 16339, 16334, 16328, 16322, 16316, 16309, 16303, 16295, 16288, 16280, 16272,
  *    16263, 16255, 16246, 16236, 16227, 16217, 16206, 16196, 16185, 16174, 16162, 16151,
  *    16138, 16126, 16113, 16101, 16087, 16074, 16060, 16046, 16031, 16017, 16001, 15986,
  *    15971, 15955, 15938, 15922, 15905, 15888, 15871, 15853, 15835, 15817, 15798, 15780,
  *    15760, 15741, 15721, 15701, 15681, 15661, 15640, 15619, 15597, 15576, 15554, 15532,
  *    15509, 15487, 15464, 15440, 15417, 15393, 15369, 15344, 15320, 15295, 15270, 15244,
  *    15219, 15193, 15166, 15140, 15113, 15086, 15059, 15031, 15003, 14975, 14947, 14918,
  *    14890, 14861, 14831, 14802, 14772, 14742, 14711, 14681, 14650, 14619, 14588, 14556,
  *    14525, 14492, 14460, 14428, 14395, 14362, 14329, 14296, 14262, 14228, 14194, 14160,
  *    14125, 14090, 14055, 14020, 13985, 13949, 13913, 13877, 13841, 13804, 13767, 13731,
  *    13693, 13656, 13619, 13581, 13543, 13505, 13466, 13428, 13389, 13350, 13311, 13272,
  *    13232, 13192, 13152, 13112, 13072, 13032, 12991, 12950, 12909, 12868, 12826, 12785,
  *    12743, 12701, 12659, 12617, 12575, 12532, 12489, 12447, 12404, 12360, 12317, 12273,
  *    12230, 12186, 12142, 12098, 12054, 12009, 11965, 11920, 11875, 11830, 11785, 11740,
  *    11695, 11649, 11603, 11558, 11512, 11466, 11420, 11373, 11327, 11280, 11234, 11187,
  *    11140, 11093, 11046, 10999, 10952, 10904, 10857, 10809, 10762, 10714, 10666, 10618,
  *    10570, 10522, 10474, 10425, 10377, 10328, 10280, 10231, 10182, 10134, 10085, 10036,
  *    9987,  9938,  9889,  9839,  9790,  9741,  9691,  9642,  9593,  9543,  9493,  9444, 
  *    9394,  9344,  9294,  9245,  9195,  9145,  9095,  9045,  8995,  8945,  8895,  8845, 
  *    8795,  8745,  8694,  8644,  8594,  8544,  8494,  8443,  8393,  8343,  8293,  8242, 
  *    8192,  8142,  8091,  8041,  7991,  7941,  7890,  7840,  7790,  7740,  7690,  7639, 
  *    7589,  7539,  7489,  7439,  7389,  7339,  7289,  7239,  7189,  7139,  7090,  7040, 
  *    6990,  6940,  6891,  6841,  6791,  6742,  6693,  6643,  6594,  6545,  6495,  6446, 
  *    6397,  6348,  6299,  6250,  6202,  6153,  6104,  6056,  6007,  5959,  5910,  5862, 
  *    5814,  5766,  5718,  5670,  5622,  5575,  5527,  5480,  5432,  5385,  5338,  5291, 
  *    5244,  5197,  5150,  5104,  5057,  5011,  4964,  4918,  4872,  4826,  4781,  4735, 
  *    4689,  4644,  4599,  4554,  4509,  4464,  4419,  4375,  4330,  4286,  4242,  4198, 
  *    4154,  4111,  4067,  4024,  3980,  3937,  3895,  3852,  3809,  3767,  3725,  3683, 
  *    3641,  3599,  3558,  3516,  3475,  3434,  3393,  3352,  3312,  3272,  3232,  3192, 
  *    3152,  3112,  3073,  3034,  2995,  2956,  2918,  2879,  2841,  2803,  2765,  2728, 
  *    2691,  2653,  2617,  2580,  2543,  2507,  2471,  2435,  2399,  2364,  2329,  2294, 
  *    2259,  2224,  2190,  2156,  2122,  2088,  2055,  2022,  1989,  1956,  1924,  1892, 
  *    1859,  1828,  1796,  1765,  1734,  1703,  1673,  1642,  1612,  1582,  1553,  1523, 
  *    1494,  1466,  1437,  1409,  1381,  1353,  1325,  1298,  1271,  1244,  1218,  1191, 
  *    1165,  1140,  1114,  1089,  1064,  1040,  1015,  991,   967,   944,   920,   897,  
  *    875,   852,   830,   808,   787,   765,   744,   723,   703,   683,   663,   643,  
  *    624,   604,   586,   567,   549,   531,   513,   496,   479,   462,   446,   429,  
  *    413,   398,   383,   367,   353,   338,   324,   310,   297,   283,   271,   258,  
  *    246,   233,   222,   210,   199,   188,   178,   167,   157,   148,   138,   129,  
  *    121,   112,   104,   96,    89,    81,    75,    68,    62,    56,    50,    45,   
  *    39,    35,    30,    26,    22,    19,    15,    12,    10,    8,     6,     4,    
  *    2,     1,     1,     0,
  * };
  *
  * u16 fft_nuttallwin_window[1024] = { 
  *    6,     6,     6,     6,     6,     6,     6,     7,     7,     7,     7,     7,    
  *    8,     8,     8,     9,     9,     9,     10,    10,    11,    11,    12,    12,   
  *    13,    14,    14,    15,    16,    17,    17,    18,    19,    20,    21,    22,   
  *    23,    24,    25,    26,    27,    28,    29,    31,    32,    33,    35,    36,   
  *    38,    39,    41,    42,    44,    46,    47,    49,    51,    53,    55,    57,   
  *    59,    61,    64,    66,    68,    71,    73,    76,    78,    81,    84,    86,   
  *    89,    92,    95,    98,    101,   105,   108,   111,   115,   118,   122,   126,  
  *    130,   134,   138,   142,   146,   150,   155,   159,   164,   169,   173,   178,  
  *    183,   189,   194,   199,   205,   210,   216,   222,   228,   234,   240,   247,  
  *    253,   260,   266,   273,   280,   287,   295,   302,   310,   318,   325,   333,  
  *    342,   350,   359,   367,   376,   385,   394,   403,   413,   423,   432,   442,  
  *    453,   463,   474,   484,   495,   506,   518,   529,   541,   553,   565,   577,  
  *    590,   602,   615,   628,   642,   655,   669,   683,   697,   711,   726,   741,  
  *    756,   771,   787,   803,   819,   835,   851,   868,   885,   902,   920,   938,  
  *    956,   974,   992,   1011,  1030,  1049,  1069,  1089,  1109,  1129,  1150,  1171, 
  *    1192,  1213,  1235,  1257,  1280,  1302,  1325,  1348,  1372,  1395,  1419,  1444, 
  *    1468,  1493,  1519,  1544,  1570,  1596,  1623,  1649,  1676,  1704,  1731,  1759, 
  *    1788,  1816,  1845,  1875,  1904,  1934,  1964,  1995,  2026,  2057,  2089,  2121, 
  *    2153,  2185,  2218,  2251,  2285,  2319,  2353,  2388,  2423,  2458,  2493,  2529, 
  *    2565,  2602,  2639,  2676,  2714,  2752,  2790,  2829,  2868,  2907,  2947,  2987, 
  *    3027,  3068,  3109,  3150,  3192,  3234,  3276,  3319,  3362,  3405,  3449,  3493, 
  *    3538,  3582,  3628,  3673,  3719,  3765,  3811,  3858,  3905,  3953,  4001,  4049, 
  *    4097,  4146,  4195,  4245,  4295,  4345,  4395,  4446,  4497,  4548,  4600,  4652, 
  *    4704,  4757,  4810,  4863,  4917,  4971,  5025,  5079,  5134,  5189,  5244,  5300, 
  *    5356,  5412,  5469,  5525,  5583,  5640,  5697,  5755,  5813,  5872,  5931,  5989, 
  *    6049,  6108,  6168,  6228,  6288,  6348,  6409,  6470,  6531,  6592,  6654,  6715, 
  *    6777,  6840,  6902,  6965,  7027,  7090,  7154,  7217,  7280,  7344,  7408,  7472, 
  *    7536,  7601,  7665,  7730,  7795,  7860,  7925,  7990,  8055,  8121,  8187,  8252, 
  *    8318,  8384,  8450,  8516,  8582,  8649,  8715,  8782,  8848,  8915,  8981,  9048, 
  *    9115,  9182,  9248,  9315,  9382,  9449,  9516,  9583,  9650,  9717,  9784,  9851, 
  *    9917,  9984,  10051, 10118, 10185, 10251, 10318, 10385, 10451, 10517, 10584, 10650,
  *    10716, 10782, 10848, 10914, 10980, 11045, 11111, 11176, 11242, 11307, 11372, 11436,
  *    11501, 11565, 11630, 11694, 11757, 11821, 11885, 11948, 12011, 12074, 12136, 12199,
  *    12261, 12323, 12384, 12446, 12507, 12568, 12628, 12688, 12748, 12808, 12867, 12927,
  *    12985, 13044, 13102, 13160, 13217, 13274, 13331, 13387, 13443, 13499, 13554, 13609,
  *    13664, 13718, 13772, 13825, 13878, 13930, 13982, 14034, 14085, 14136, 14186, 14236,
  *    14286, 14335, 14383, 14431, 14479, 14526, 14573, 14619, 14664, 14709, 14754, 14798,
  *    14842, 14885, 14927, 14969, 15011, 15052, 15092, 15132, 15171, 15210, 15248, 15286,
  *    15323, 15359, 15395, 15430, 15465, 15499, 15533, 15566, 15598, 15630, 15661, 15691,
  *    15721, 15750, 15779, 15807, 15834, 15861, 15887, 15913, 15937, 15962, 15985, 16008,
  *    16030, 16052, 16073, 16093, 16113, 16131, 16150, 16167, 16184, 16200, 16216, 16231,
  *    16245, 16258, 16271, 16283, 16295, 16306, 16316, 16325, 16334, 16342, 16349, 16356,
  *    16362, 16367, 16371, 16375, 16378, 16381, 16383, 16384, 16384, 16384, 16383, 16381,
  *    16378, 16375, 16371, 16367, 16362, 16356, 16349, 16342, 16334, 16325, 16316, 16306,
  *    16295, 16283, 16271, 16258, 16245, 16231, 16216, 16200, 16184, 16167, 16150, 16131,
  *    16113, 16093, 16073, 16052, 16030, 16008, 15985, 15962, 15937, 15913, 15887, 15861,
  *    15834, 15807, 15779, 15750, 15721, 15691, 15661, 15630, 15598, 15566, 15533, 15499,
  *    15465, 15430, 15395, 15359, 15323, 15286, 15248, 15210, 15171, 15132, 15092, 15052,
  *    15011, 14969, 14927, 14885, 14842, 14798, 14754, 14709, 14664, 14619, 14573, 14526,
  *    14479, 14431, 14383, 14335, 14286, 14236, 14186, 14136, 14085, 14034, 13982, 13930,
  *    13878, 13825, 13772, 13718, 13664, 13609, 13554, 13499, 13443, 13387, 13331, 13274,
  *    13217, 13160, 13102, 13044, 12985, 12927, 12867, 12808, 12748, 12688, 12628, 12568,
  *    12507, 12446, 12384, 12323, 12261, 12199, 12136, 12074, 12011, 11948, 11885, 11821,
  *    11757, 11694, 11630, 11565, 11501, 11436, 11372, 11307, 11242, 11176, 11111, 11045,
  *    10980, 10914, 10848, 10782, 10716, 10650, 10584, 10517, 10451, 10385, 10318, 10251,
  *    10185, 10118, 10051, 9984,  9917,  9851,  9784,  9717,  9650,  9583,  9516,  9449, 
  *    9382,  9315,  9248,  9182,  9115,  9048,  8981,  8915,  8848,  8782,  8715,  8649, 
  *    8582,  8516,  8450,  8384,  8318,  8252,  8187,  8121,  8055,  7990,  7925,  7860, 
  *    7795,  7730,  7665,  7601,  7536,  7472,  7408,  7344,  7280,  7217,  7154,  7090, 
  *    7027,  6965,  6902,  6840,  6777,  6715,  6654,  6592,  6531,  6470,  6409,  6348, 
  *    6288,  6228,  6168,  6108,  6049,  5989,  5931,  5872,  5813,  5755,  5697,  5640, 
  *    5583,  5525,  5469,  5412,  5356,  5300,  5244,  5189,  5134,  5079,  5025,  4971, 
  *    4917,  4863,  4810,  4757,  4704,  4652,  4600,  4548,  4497,  4446,  4395,  4345, 
  *    4295,  4245,  4195,  4146,  4097,  4049,  4001,  3953,  3905,  3858,  3811,  3765, 
  *    3719,  3673,  3628,  3582,  3538,  3493,  3449,  3405,  3362,  3319,  3276,  3234, 
  *    3192,  3150,  3109,  3068,  3027,  2987,  2947,  2907,  2868,  2829,  2790,  2752, 
  *    2714,  2676,  2639,  2602,  2565,  2529,  2493,  2458,  2423,  2388,  2353,  2319, 
  *    2285,  2251,  2218,  2185,  2153,  2121,  2089,  2057,  2026,  1995,  1964,  1934, 
  *    1904,  1875,  1845,  1816,  1788,  1759,  1731,  1704,  1676,  1649,  1623,  1596, 
  *    1570,  1544,  1519,  1493,  1468,  1444,  1419,  1395,  1372,  1348,  1325,  1302, 
  *    1280,  1257,  1235,  1213,  1192,  1171,  1150,  1129,  1109,  1089,  1069,  1049, 
  *    1030,  1011,  992,   974,   956,   938,   920,   902,   885,   868,   851,   835,  
  *    819,   803,   787,   771,   756,   741,   726,   711,   697,   683,   669,   655,  
  *    642,   628,   615,   602,   590,   577,   565,   553,   541,   529,   518,   506,  
  *    495,   484,   474,   463,   453,   442,   432,   423,   413,   403,   394,   385,  
  *    376,   367,   359,   350,   342,   333,   325,   318,   310,   302,   295,   287,  
  *    280,   273,   266,   260,   253,   247,   240,   234,   228,   222,   216,   210,  
  *    205,   199,   194,   189,   183,   178,   173,   169,   164,   159,   155,   150,  
  *    146,   142,   138,   134,   130,   126,   122,   118,   115,   111,   108,   105,  
  *    101,   98,    95,    92,    89,    86,    84,    81,    78,    76,    73,    71,   
  *    68,    66,    64,    61,    59,    57,    55,    53,    51,    49,    47,    46,   
  *    44,    42,    41,    39,    38,    36,    35,    33,    32,    31,    29,    28,   
  *    27,    26,    25,    24,    23,    22,    21,    20,    19,    18,    17,    17,   
  *    16,    15,    14,    14,    13,    12,    12,    11,    11,    10,    10,    9,    
  *    9,     9,     8,     8,     8,     7,     7,     7,     7,     7,     6,     6,    
  *    6,     6,     6,     6, 
  * };  
  *
  ******************************************************************************
  */ 
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TX_PHE_LL_FFT_H
#define __TX_PHE_LL_FFT_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "tx_phe.h"
     
/** @addtogroup TX_PHE_StdPeriph_Driver TX_PHE Driver
  * @{
  */
     
/** @addtogroup fft_interface_gr FFT Driver
  * @ingroup  TX_PHE_StdPeriph_Driver
  * @{
  */ 

/** @addtogroup FFT_LL_Driver FFT LL Driver
  * @ingroup  fft_interface_gr
  * @brief Mainly the driver part of the FFT module, which includes \b FFT \b Register 
  * \b Constants, \b FFT \b Exported \b Constants, \b FFT \b Exported \b Struct, \b FFT
  * \b Data \b transfers \b functions, \b FFT \b Initialization \b and \b FFT \b Configuration 
  * \b And \b Interrupt \b Handle \b function.
  * @{
  */
     
/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
     
/** @defgroup FFT_LL_Register_Constants FFT LL Register Constants
  * @ingroup  FFT_LL_Driver
  * @brief    FFT LL register constant table definition
  *
  *
@verbatim   
  ===============================================================================
                                Register Constants
  ===============================================================================  
  
    Register Constants mainly encapsulates each bit in each group in the FFT 
    register. In the process of configuration, the macro definition can be directly 
    called to configure the FFT register, mainly for convenience. Understand the 
    configuration of the FFT.
    
@endverbatim
  *
  * @{
  */

/***** FFT_DMA_CON *****/
/*! Indicates whether the real or imaginary data after FFT windowing has at least one 
 *  data greater than (+-) (fft_maximum*2).
 */
#define LL_FFT_DMA_CON_FFT_OV2                    (1UL << 16)
/*! Indicates whether the real or imaginary data after FFT windowing has at least one 
 *  data greater than (+-) (fft_maximum).
 */
#define LL_FFT_DMA_CON_FFT_OV1                    (1UL << 15)
/*! FFT dma input data downsample select: The downsampling range is from 1 to 16.
 */
#define LL_FFT_DMA_CON_DOWN_SAMPLE(n)             (((n)&0xF) << 11)
/*! FFT DMA finish interrupt enable
 */
#define LL_FFT_DMA_CON_FFT_DMA_IE                 (1UL << 10)
/*! FFT DMA real and imag interface exchange
 */
#define LL_FFT_DMA_CON_REAL_IMAG_EXCHANGE         (1UL << 9)
/*! FFT DMA finish flag
 */
#define LL_FFT_DMA_CON_FFT_DMA_PEND               (1UL << 8)
/*! FFT DMA Window source select
 */
#define LL_FFT_DMA_CON_WINDOW_SRC(n)              (((n)&0x3) << 6)
/*! IMAGE input is zero select
 */
#define LL_FFT_DMA_CON_IMAGE_SEL(n)               (((n)&0x3) << 4)
/*! FFT DMA sample selection.
 */
#define LL_FFT_DMA_CON_FFT_MODE(n)                (((n)&0x7) << 1)
/*! FFT DMA enable. This bit will be cleared automatically after finish
 */
#define LL_FFT_DMA_CON_ENABLE                     (1UL << 0)


/***** FFT_REAL_STADR *****/
/*! REAL data DMA start address. Requires 16bit alignment.
 */
#define LL_FFT_REAL_STADR(n)                      (((n)&0xFFFFFFFF) << 0) 


/***** FFT_IMAG_STADR *****/
/*! IMAG data DMA start address. Requires 16bit alignment.
 */
#define LL_FFT_IMAG_STADR(n)                      (((n)&0xFFFFFFFF) << 0) 


/***** FFT_WINDOW_STADR *****/
/*! WINDOW data DMA address. Requires 16bit alignment.
 */
#define LL_FFT_WINDOW_STADR(n)                    (((n)&0xFFFFFFFF) << 0)


/***** FFT_REALIMAG_OUT_STADR *****/ 
/*! REALIMAG data output DMA start address. Requires 32bit alignment.
 */
#define LL_FFT_REALIMAG_OUT_STADR(n)              (((n)&0xFFFFFFFF) << 0)


/***** FFT_LEN *****/ 
/*! FFT DMA buffer length
 */
#define LL_FFT_LEN(n)                             (((n)&0xFFFF) << 0) 


/***** FFT_INDEX *****/
/*! FFT_DMA finish once will increase 1. when increase to FFT_LEN, cleared to 0.
 */
#define LL_FFT_INDEX(n)                           (((n)&0xFFFF) << 0) 


/***** FFT_STADR *****/   
/*! FFT input data DMA start address. Requires 64bit alignment. This register does not 
 *  need to be configured and is read only. The hardware automatically loads 
 *  FFT_REALIMAG_OUT_STADR into this register during FFT_KS.
 */
#define LL_FFT_STADR(n)                           (((n)&0xFFFFFFFF) << 0) 


/***** FFT_KS *****/    
/*! FFT kick start, Write 1 to kick off FFT.
 */
#define LL_FFT_KS                                 (1UL << 0)


/***** FFT_CTRL *****/    
/*! FFT enable interrupt
 */
#define LL_FFT_CTRL_IRQ_EN                        (1UL << 7)
/*! Saturate Shift Control:  
 *  0 -> right shift 1 bit at last stage  
 *  1 -> right shift (N-SAT_CNT) bits at last stage  
 */
#define LL_FFT_CTRL_SHIFT_CTRL                    (1UL << 6)
/*! Select the mode of the FFT: 0 -> FFT MODE; 1 -> IFFT MODE.
 */
#define LL_FFT_CTRL_IFFT_MODE                     (1UL << 5)
/*! IFFT DMA sample selection.
 */
#define LL_FFT_CTRL_FFT_POINT_SEL(n)              (((n)&0x7) << 1) 
/*! FFT enable, This bit will be cleared automatically after finish
 */
#define LL_FFT_CTRL_FFT_EN                        (1UL << 0)


/***** FFT_STAT *****/
/*! FFT IRQ pending, writes1 to clear it.
 */
#define LL_FFT_STAT_IRQ_PEDING                    (1UL << 4)
/*! FFT calculate overflow times.
 */
#define LL_FFT_STAT_OVERFLOW_TIMES(n)             (((n)&0xF) << 0) 


/***** FFT_MAXIMUM *****/
/*! The maximum value of the input data of the real or imaginary part that the FFT 
 *  operation module can accept. The default is 11584.
 */
#define LL_FFT_MAXIMUM(n)                         (((n)&0xFFFF) << 0) 


/**
  * @}
  */

/** @defgroup FFT_LL_Exported_Constants FFT LL Exported Constants
  * @ingroup  FFT_LL_Driver
  * @brief    FFT LL external constant definition
  *
@verbatim   
  ===============================================================================
                                Exported Constants
  ===============================================================================  
  
    Exported Constants mainly restricts the partial configuration of the abstraction 
    layer by using the form of enumeration to facilitate the use and understanding of 
    the module configuration. For the specific enumeration meaning, please refer to 
    the annotation of each module.

@endverbatim
  *
  * @{
  */
  
/***** DRIVER API *****/



/***** LL API *****/


  
/***** LL API AND DRIVER API *****/

/**
  * @brief Enumeration constant for FFT downsample select
  */
typedef enum {
    /*! FFT downsample select 1
     */
    LL_FFT_DOWNSAMPLE_1 = 0,
    /*! FFT downsample select 2
     */
    LL_FFT_DOWNSAMPLE_2,
    /*! FFT downsample select 3
     */
    LL_FFT_DOWNSAMPLE_3,
    /*! FFT downsample select 4
     */
    LL_FFT_DOWNSAMPLE_4,
    /*! FFT downsample select 5
     */
    LL_FFT_DOWNSAMPLE_5,
    /*! FFT downsample select 6
     */
    LL_FFT_DOWNSAMPLE_6,
    /*! FFT downsample select 7
     */
    LL_FFT_DOWNSAMPLE_7,
    /*! FFT downsample select 8
     */
    LL_FFT_DOWNSAMPLE_8,
    /*! FFT downsample select 9
     */
    LL_FFT_DOWNSAMPLE_9,
    /*! FFT downsample select 10
     */
    LL_FFT_DOWNSAMPLE_10,
    /*! FFT downsample select 11
     */
    LL_FFT_DOWNSAMPLE_11,
    /*! FFT downsample select 12
     */
    LL_FFT_DOWNSAMPLE_12,
    /*! FFT downsample select 13
     */
    LL_FFT_DOWNSAMPLE_13,
    /*! FFT downsample select 14
     */
    LL_FFT_DOWNSAMPLE_14,
    /*! FFT downsample select 15
     */
    LL_FFT_DOWNSAMPLE_15,
    /*! FFT downsample select 16
     */
    LL_FFT_DOWNSAMPLE_16,
} TYPE_ENUM_LL_FFT_DOWNSAMPLE;

/**
  * @brief Enumeration constant for FFT real and imag exchang select
  */
typedef enum {
    /*! imaginary part of the DMA complex and the real part DMA no exchange
     */
    LL_FFT_R_I_EX_NO = 0,
    /*! imaginary part of the DMA complex and the real part DMA exchange
     */
    LL_FFT_R_I_EX,
} TYPE_ENUM_LL_FFT_R_I_EX;

/**
  * @brief Enumeration constant for FFT windowed data source selection.
  */
typedef enum {
    /*! The FFT's windowed data comes from Memory and is added by the user.
     */
    LL_FFT_WINDOW_SRC_MEM = 0,
    /*! The windowed data of the FFT is derived from the internal hanning table.
     *  For specific coefficients, please refer to array fft_hanning_window.
     */
    LL_FFT_WINDOW_SRC_HANNING,
    /*! The windowed data of the FFT is derived from the internal nuttallwin table. 
     *  For specific coefficients, please refer to array fft_nuttallwin_window.
     */
    LL_FFT_WINDOW_SRC_NUTTALLWIN,
    /*! Set the FFT windowed data to constant 1, that is, the windowing is invalid. 
     */
    LL_FFT_WINDOW_SRC_CONSTANT_1,
} TYPE_ENUM_LL_FFT_WINDOOW;

/**
  * @brief Enumeration constant for FFT imaginary data source processing selection.
  */
typedef enum {
    /*! The FFT imaginary data comes from memory.
     */
    LL_FFT_IMAG_INPUT_DMA = 0,
    /*! The FFT imaginary data is all zero.
     */
    LL_FFT_IMAG_INPUT_ZERO,
    /*! The FFT imaginary data is all inverted.
     */
    LL_FFT_IMAG_INPUT_NEGATE,
} TYPE_ENUM_LL_FFT_IMG_SEL;

/**
  * @brief Enumeration constant for FFT calculation sample selection
  */
typedef enum {
    /*! FFT calculation sample number selection: 32 points.
     */
    LL_FFT_POINT_32 = 0,
    /*! FFT calculation sample number selection: 64 points.
     */
    LL_FFT_POINT_64,
    /*! FFT calculation sample number selection: 128 points.
     */
    LL_FFT_POINT_128,
    /*! FFT calculation sample number selection: 256 points.
     */
    LL_FFT_POINT_256,
    /*! FFT calculation sample number selection: 512 points.
     */
    LL_FFT_POINT_512,
    /*! FFT calculation sample number selection: 1024 points.
     */
    LL_FFT_POINT_1024,
} TYPE_ENUM_LL_FFT_POINT_SEL;

/**
  * @brief Enumeration constant for FFT internal calculation shift selection
  */
typedef enum {
    /*! Indicates that each level is divided by 2, and after the cumulative log(N) 
     *  level, Equivalent to the effect of dividing by N.
     */
    LL_FFT_SHIFT_EVERY_STAGE = 0,
    /*! When it indicates that a certain level is saturated, it divides by 2, when 
     *  it is not saturated, it does not divide by 2, waits for the last stage to 
     *  be added again, and how much compensation is needed is the value of (N - STA_CNT).
     */
    LL_FFT_SHIFT_SATURATION,
} TYPE_ENUM_LL_FFT_SHIFT;

/**
  * @brief Enumeration constant for FFT mode select
  */
typedef enum {
    /*! Select the FFT calculation mode.
     */
    LL_FFT_MODE_FFT = 0,
    /*! Select the IFFT calculation mode.
     */
    LL_FFT_MODE_IFFT,
} TYPE_ENUM_LL_FFT_MODE;

/**
  * @}
  */

/** @defgroup FFT_LL_Exported_Struct FFT LL Exported Struct
  * @ingroup  FFT_LL_Driver
  * @brief    FFT LL external configuration structure definition
  *
@verbatim   
  ===============================================================================
                                Exported Struct
  ===============================================================================  

    Exported Struct mainly extracts the FFT registers from the API, and abstracts 
    the structure. As long as it implements the low coupling between the registers 
    and the registers, the user only needs to configure the structure of the abstraction 
    layer and call hal_fft_init. Function, you can configure the FFT module without 
    involving the configuration of the collective register.

@endverbatim
  *
  * @{
  */
  
/**
  * @brief FFT DMA low layer configuration structure
  */
typedef struct __ll_fft_irq_cfg {
    /*! Interrupt enable control.
     *  @note false represents enable interrupt, true represents disable interrupt.
     */
    bool dma_intr_en;
    /*! Interrupt enable control.
     *  @note false represents enable interrupt, true represents disable interrupt.
     */
    bool intr_en;
} TYPE_LL_FFT_IRQ_CFG;

/**
  * @brief FFT data preprocess low layer configuration structure
  */
typedef struct __ll_fft_preprocess_cfg {
    /*! Configure the value of the FFT downsample. 
     */
    TYPE_ENUM_LL_FFT_DOWNSAMPLE downsample_sel;
    /*! DMA real and imaginary address exchange.For example, the default
     *  real part of FFT0 is placed in SRAM1 (ie, FFT_REAL_STAR is
     *  in SRAM1) and the imaginary part is in SRAM2 (ie, FFT_IMAG_STAR
     *  is in SRAM2). If the DMA exchanges real and imaginary addresses,
     *  then FFT_REAL_STAR will be located in SRAM2 and
     *  FFT_IMAG_STAR will be located in SRAM1.
     */
    TYPE_ENUM_LL_FFT_R_I_EX     real_img_swap;
    /*! Configure the selection of the windowing factor.Please refer to 
     *  TYPE_ENUM_LL_FFT_WINDOOW for the specific type.
     */
    TYPE_ENUM_LL_FFT_WINDOOW    window_src;
    /*! Configure the selection of the FFT imaginary coefficient value.
     *  Please refer to TYPE_ENUM_LL_FFT_IMG_SEL for the specific type.
     */
    TYPE_ENUM_LL_FFT_IMG_SEL    img_val_sel;
    /*! Configure the FFT to calculate the number of samples.
     *  Please refer to TYPE_ENUM_LL_FFT_POINT_SEL for the specific type.
     */
    TYPE_ENUM_LL_FFT_POINT_SEL  point_sel;
    /*! Configure the length of the FFT input buffer.
     */
    u16                         dma_points;
    /*! Index represents the subscript offset of the DMA buffer.
     */
    u16                         index;
    /*! (a+b*j) DMA real part start address of complex number, 2 byte
     *  aligned. The real part data is a 16-bit signed number with 15 bit
     *  fixed point.
     *  @note FFT0's real_dma_src_addr can only be located in SRAM1.  
     *        FFT1's real_dma_src_addr can only be located in SRAM2.  
     *        FFT2's real_dma_src_addr can only be located in SRAM3.  
     *        Statement example:  
     *        static s16 fft0_buf_real[1024] __attribute__((section("SRAM1")));
     */
    u32                         real_dma_src_addr;
    /*! (a+b*j) DMA imaginary part start address of complex number, 2 byte
     *  aligned. The imaginary part data is a 16-bit signed number with 15
     *  bit fixed point.
     *  @note FFT0's img_dma_src_addr can only be located in SRAM2.  
     *        FFT1's img_dma_src_addr can only be located in SRAM3.  
     *        FFT2's img_dma_src_addr can only be located in SRAM1.  
     *        Statement example:  
     *        static s16 fft0_buf_img[1024] __attribute__((section("SRAM2")));
     */
    u32                         img_dma_src_addr;
    /*! The start address of the windowed buffer. 2Byte address alignment.  
     *  @note FFT0's window_dma_src_addr can only be located in SRAM0.  
     *        FFT1's window_dma_src_addr can only be located in SRAM0.  
     *        FFT2's window_dma_src_addr can only be located in SRAM0.  
     *        Statement example:  
     *        static u16 fft_buf_wind[64] __attribute__((section("SRAM0")));
     */
    u32                         window_dma_src_addr;
    /*! Need to use an address to save the output(32bit size) in accordance 
     *  with 32bit aligned variables.  
     *  @note FFT0's dma_dst_addr can only be located in SRAM4 and SRAM5.  
     *        FFT1's dma_dst_addr can only be located in SRAM2 and SRAM5.  
     *        FFT2's dma_dst_addr can only be located in SRAM3 and SRAM5.  
     *        Statement example:  
     *        static u32 fft0_obuf[1024] __attribute__((section("SRAM4_5")));
     */
    u32                         dma_dst_addr;
} TYPE_LL_FFT_PREPROCESS_CFG;

/**
  * @brief FFT low layer configuration structure
  */
typedef struct __ll_fft_cfg {
    /*! Configure the FFT power normalization mode.
     *  Please refer to TYPE_ENUM_LL_FFT_SHIFT for the specific type.
     */    
    TYPE_ENUM_LL_FFT_SHIFT     shift_sel;
    /*! Configure the FFT calculation mode, FFT mode or IFFT mode.
     *  Please refer to TYPE_ENUM_LL_FFT_MODE for the specific type.
     */
    TYPE_ENUM_LL_FFT_MODE      mode;
    /*! Configure the FFT to calculate the number of samples.
     *  Please refer to TYPE_ENUM_LL_FFT_POINT_SEL for the specific type.
     */
    TYPE_ENUM_LL_FFT_POINT_SEL point_sel;
} TYPE_LL_FFT_CFG;

/**
  * @brief FFT low layer Initialization structure
  */
typedef struct __ll_fft_init {
    u8 reserved;
} TYPE_LL_FFT_INIT;

/**
  * @}
  */

/** @defgroup FFT_LL_Interrupt FFT LL Interrupt Handle function
  * @brief   FFT LL Interrupt Handle function
  *
@verbatim   
  ===============================================================================
                        Interrupt Handle function
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the FFT  
    Interrupt Handle function.

    how to use?

    The FFT interrupt handler uses a callback method that reserves the interface 
    to the user in the form of a callback function. The client needs to initialize 
    the callback function when initializing the FFT in order for the interrupt to 
    be processed normally. 
   
@endverbatim
  *
  * @{
  */
  


/**
  * @}
  */
  
/** @defgroup FFT_LL_Inti_Cfg FFT LL Initialization And Configuration
  * @brief    FFT LL Initialization And Configuration
  *
@verbatim   
  ===============================================================================
                        Initialization And Configuration
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the FFT data 
    Initialization and Configuration.
    
    how to use?

@endverbatim
  *
  * @{
  */

/**
  * @brief  Low layer FFT module initialization
  * @param  p_fft: The structure pointer of the FFT group (FFT0, FFT1, FFT2) is selected.
  * @param  p_init: Module configuration structure pointer(TYPE_LL_FFT_INIT)
  * @retval None
  */
void ll_fft_init(FFT_TypeDef *p_fft, TYPE_LL_FFT_INIT *p_init);

/**
  * @brief  Low layer FFT module detele initialization
  * @param  p_fft: The structure pointer of the FFT group (FFT0, FFT1, FFT2) is selected.
  * @retval None
  */
void ll_fft_deinit(FFT_TypeDef *p_fft);

/**
  * @brief  Low layer FFT module interrupt configuration
  * @param  p_fft: The structure pointer of the FFT group (FFT0, FFT1, FFT2) is selected.
  * @param  p_cfg: Module configuration structure pointer(TYPE_LL_FFT_IRQ_CFG)
  * @retval None
  */
void ll_fft_irq_config(FFT_TypeDef *p_fft, TYPE_LL_FFT_IRQ_CFG *p_cfg);
  
/**
  * @brief  Low layer FFT preprocess configuration
  * @param  p_fft: The structure pointer of the FFT group (FFT0, FFT1, FFT2) is selected.
  * @param  p_cfg: Module configuration structure pointer(TYPE_LL_FFT_PREPROCESS_CFG)
  * @retval None
  */
void ll_fft_preprocess_config(FFT_TypeDef *p_fft, TYPE_LL_FFT_PREPROCESS_CFG *p_cfg);
  
/**
  * @brief  Low layer FFT module configuration
  * @param  p_fft: The structure pointer of the FFT group (FFT0, FFT1, FFT2) is selected.
  * @param  p_cfg: Module configuration structure pointer(TYPE_LL_FFT_CFG)
  * @retval None
  */
void ll_fft_config(FFT_TypeDef *p_fft, TYPE_LL_FFT_CFG *p_cfg);

/**
  * @}
  */
  
/** @defgroup FFT_LL_Data_Transfers FFT LL Data transfers functions
  * @brief    FFT LL Data transfers functions 
  *
@verbatim   
  ===============================================================================
                            Data transfers functions
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the FFT data 
    transfers and receive.
  
@endverbatim
  *
  * @{
  */

/**
  * @brief  Low layer FFT preprocess start function
  * @param  p_fft: The structure pointer of the FFT group (FFT0, FFT1, FFT2) is selected.
  * @retval None
  */
void ll_fft_preprocess_start(FFT_TypeDef *p_fft);

/**
  * @brief  Low layer FFT module start function
  * @param  p_fft: The structure pointer of the FFT group (FFT0, FFT1, FFT2) is selected.
  * @retval None
  */
void ll_fft_start(FFT_TypeDef *p_fft);

/**
  * @brief  Low layer FFT preprocess stop function
  * @param  p_fft: The structure pointer of the FFT group (FFT0, FFT1, FFT2) is selected.
  * @retval None
  */
void ll_fft_preprocess_stop(FFT_TypeDef *p_fft);

/**
  * @brief  Low layer FFT module stop function
  * @param  p_fft: The structure pointer of the FFT group (FFT0, FFT1, FFT2) is selected.
  * @retval None
  */
 void ll_fft_stop(FFT_TypeDef *p_fft);
  
/**
  * @brief  FFT preprocess get finished pending
  * @param  p_fft: Select the initialized FFT preprocess group pointer
  * @retval pending value
  */
#define LL_FFT_PREPROCESS_GET_DONE_PENDING(p_fft) ((p_fft)->DMA_CON & LL_FFT_DMA_CON_FFT_DMA_PEND)

/**
  * @brief  FFT get finished pending
  * @param  p_fft: Select the initialized FFT group pointer
  * @retval pending value
  */
#define LL_FFT_GET_DONE_PENDING(p_fft)            ((p_fft)->STAT & LL_FFT_STAT_IRQ_PEDING)

/**
  * @brief  FFT clear preprocess done pending
  * @param  p_fft: Select the initialized FFT group pointer
  * @retval None
  */
__STATIC_INLINE void ll_fft_clear_preprocess_done_pending(FFT_TypeDef *p_fft) {
    p_fft->DMA_CON |= LL_FFT_DMA_CON_FFT_DMA_PEND;
}

/**
  * @brief  FFT clear done pending
  * @param  p_fft: Select the initialized FFT group pointer
  * @retval None
  */
__STATIC_INLINE void ll_fft_clear_done_pending(FFT_TypeDef *p_fft) {
    p_fft->STAT |= LL_FFT_STAT_IRQ_PEDING;
}

/**
  * @brief  FFT preprocess wait for the specified channel to finish computing
  * @param  p_fft: Select the initialized FFT group pointer
  * @retval None
  */
__STATIC_INLINE void ll_fft_wait_preprocess_done_pending(FFT_TypeDef *p_fft) {
    while(!(p_fft->DMA_CON & LL_FFT_DMA_CON_FFT_DMA_PEND));
    p_fft->DMA_CON |= LL_FFT_DMA_CON_FFT_DMA_PEND;
}

/**
  * @brief  FFT Wait for the specified channel to finish computing
  * @param  p_fft: Select the initialized FFT group pointer
  * @retval None
  */
__STATIC_INLINE void ll_fft_wait_done_pending(FFT_TypeDef *p_fft) {
    while(!(p_fft->STAT & LL_FFT_STAT_IRQ_PEDING));
    p_fft->STAT |= LL_FFT_STAT_IRQ_PEDING;
}

/**
  * @brief  FFT enable preprocess interrupt
  * @param  p_fft: Select the initialized FFT group pointer
  * @retval None
  */
__STATIC_INLINE void ll_fft_preprocess_interrupt_enable(FFT_TypeDef *p_fft) {
    p_fft->DMA_CON |= LL_FFT_DMA_CON_FFT_DMA_IE;
}

/**
  * @brief  FFT disable preprocess interrupt
  * @param  p_fft: Select the initialized FFT group pointer
  * @retval None
  */
__STATIC_INLINE void ll_fft_preprocess_interrupt_disable(FFT_TypeDef *p_fft) {
    p_fft->DMA_CON &= ~(LL_FFT_DMA_CON_FFT_DMA_IE);
}

/**
  * @brief  FFT enable interrupt
  * @param  p_fft: Select the initialized FFT group pointer
  * @retval None
  */
__STATIC_INLINE void ll_fft_interrupt_enable(FFT_TypeDef *p_fft) {
    p_fft->CTRL |= LL_FFT_CTRL_IRQ_EN;
}

/**
  * @brief  FFT disable interrupt
  * @param  p_fft: Select the initialized FFT group pointer
  * @retval None
  */
__STATIC_INLINE void ll_fft_interrupt_disable(FFT_TypeDef *p_fft) {
    p_fft->CTRL &= ~(LL_FFT_CTRL_IRQ_EN);
}

/**
  * @brief  FFT index set
  * @param  p_fft: Select the initialized FFT group pointer
  * @param  index: fft index , rang : [0,fft_points)
  * @retval None
  */
__STATIC_INLINE void ll_fft_set_index(FFT_TypeDef *p_fft, u16 index) {
    p_fft->INDEX = LL_FFT_INDEX(index);
}

/**
  * @brief  FFT index Get
  * @param  p_fft: Select the initialized FFT group pointer
  * @retval None
  */
#define LL_FFT_GET_INDEX(p_fft)            ((p_fft)->INDEX & LL_FFT_INDEX(0xFFFF))
  
/**
  * @brief  FFT index set dma_dst_addr
  * @param  p_fft: Select the initialized FFT group pointer
  * @param  addr : fft dma_dst_addr, \ref TYPE_LL_FFT_PREPROCESS_CFG
  * @retval None
  */
__STATIC_INLINE void ll_fft_set_preprocess_dma_dst_addr(FFT_TypeDef *p_fft, u32 addr) {
    p_fft->REALIMAG_OUT_STADR = LL_FFT_REALIMAG_OUT_STADR(addr);
}

/**
  * @brief  FFT index set window dma_dst_addr
  * @param  p_fft: Select the initialized FFT group pointer
  * @param  addr : fft window_dma_dst_addr, \ref TYPE_LL_FFT_PREPROCESS_CFG
  * @retval None
  */
__STATIC_INLINE void ll_fft_set_preprocess_window_dma_dst_addr(FFT_TypeDef *p_fft, u32 addr) {
    p_fft->WINDOW_STADR = LL_FFT_WINDOW_STADR(addr);
}

/**
  * @brief  FFT index set real_dma_src_addr
  * @param  p_fft: Select the initialized FFT group pointer
  * @param  addr : fft real_dma_src_addr, \ref TYPE_LL_FFT_PREPROCESS_CFG
  * @retval None
  */
__STATIC_INLINE void ll_fft_set_preprocess_real_dma_src_addr(FFT_TypeDef *p_fft, u32 addr) {
    p_fft->REAL_STADR = LL_FFT_REAL_STADR(addr);
}

/**
  * @brief  FFT index set imag_dma_src_addr
  * @param  p_fft: Select the initialized FFT group pointer
  * @param  addr : fft imag_dma_src_addr, \ref TYPE_LL_FFT_PREPROCESS_CFG
  * @retval None
  */
__STATIC_INLINE void ll_fft_set_preprocess_imag_dma_src_addr(FFT_TypeDef *p_fft, u32 addr) {
    p_fft->IMAG_STADR = LL_FFT_IMAG_STADR(addr);
}

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

/**
  * @}
  */

/**
  * @}
  */

#endif //__TX_PHE_LL_FFT_H

/*************************** (C) COPYRIGHT 2018 HUGE-IC ***** END OF FILE *****/
