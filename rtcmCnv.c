#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "rtcmCnv.h"


#define SYS_NONE    0x00                /* navigation system: none */
#define SYS_GPS     0x01                /* navigation system: GPS */
#define SYS_SBS     0x02                /* navigation system: SBAS */
#define SYS_GLO     0x04                /* navigation system: GLONASS */
#define SYS_GAL     0x08                /* navigation system: Galileo */
#define SYS_QZS     0x10                /* navigation system: QZSS */
#define SYS_CMP     0x20                /* navigation system: BeiDou */
#define SYS_IRN     0x40                /* navigation system: IRNS */
#define SYS_LEO     0x80                /* navigation system: LEO */
#define SYS_ALL     0xFF                /* navigation system: all */

#define RTCM3PREAMB 0xD3        /* rtcm ver.3 frame preamble */

#define P2_10       0.0009765625          /* 2^-10 */
#define P2_24       5.960464477539063E-08 /* 2^-24 */
#define P2_29       1.862645149230957E-09 /* 2^-29 */
#define CLIGHT      299792458.0         /* speed of light (m/s) */
#define RANGE_MS    (CLIGHT*0.001)      /* range in 1 ms */

#define CODE_NONE   0                   /* obs code: none or unknown */
#define CODE_L1C    1                   /* obs code: L1C/A,G1C/A,E1C (GPS,GLO,GAL,QZS,SBS) */
#define CODE_L1P    2                   /* obs code: L1P,G1P,B1P (GPS,GLO,BDS) */
#define CODE_L1W    3                   /* obs code: L1 Z-track (GPS) */
#define CODE_L1Y    4                   /* obs code: L1Y        (GPS) */
#define CODE_L1M    5                   /* obs code: L1M        (GPS) */
#define CODE_L1N    6                   /* obs code: L1codeless,B1codeless (GPS,BDS) */
#define CODE_L1S    7                   /* obs code: L1C(D)     (GPS,QZS) */
#define CODE_L1L    8                   /* obs code: L1C(P)     (GPS,QZS) */
#define CODE_L1E    9                   /* (not used) */
#define CODE_L1A    10                  /* obs code: E1A,B1A    (GAL,BDS) */
#define CODE_L1B    11                  /* obs code: E1B        (GAL) */
#define CODE_L1X    12                  /* obs code: E1B+C,L1C(D+P),B1D+P (GAL,QZS,BDS) */
#define CODE_L1Z    13                  /* obs code: E1A+B+C,L1S (GAL,QZS) */
#define CODE_L2C    14                  /* obs code: L2C/A,G1C/A (GPS,GLO) */
#define CODE_L2D    15                  /* obs code: L2 L1C/A-(P2-P1) (GPS) */
#define CODE_L2S    16                  /* obs code: L2C(M)     (GPS,QZS) */
#define CODE_L2L    17                  /* obs code: L2C(L)     (GPS,QZS) */
#define CODE_L2X    18                  /* obs code: L2C(M+L),B1_2I+Q (GPS,QZS,BDS) */
#define CODE_L2P    19                  /* obs code: L2P,G2P    (GPS,GLO) */
#define CODE_L2W    20                  /* obs code: L2 Z-track (GPS) */
#define CODE_L2Y    21                  /* obs code: L2Y        (GPS) */
#define CODE_L2M    22                  /* obs code: L2M        (GPS) */
#define CODE_L2N    23                  /* obs code: L2codeless (GPS) */
#define CODE_L5I    24                  /* obs code: L5I,E5aI   (GPS,GAL,QZS,SBS) */
#define CODE_L5Q    25                  /* obs code: L5Q,E5aQ   (GPS,GAL,QZS,SBS) */
#define CODE_L5X    26                  /* obs code: L5I+Q,E5aI+Q,L5B+C,B2aD+P (GPS,GAL,QZS,IRN,SBS,BDS) */
#define CODE_L7I    27                  /* obs code: E5bI,B2bI  (GAL,BDS) */
#define CODE_L7Q    28                  /* obs code: E5bQ,B2bQ  (GAL,BDS) */
#define CODE_L7X    29                  /* obs code: E5bI+Q,B2bI+Q (GAL,BDS) */
#define CODE_L6A    30                  /* obs code: E6A,B3A    (GAL,BDS) */
#define CODE_L6B    31                  /* obs code: E6B        (GAL) */
#define CODE_L6C    32                  /* obs code: E6C        (GAL) */
#define CODE_L6X    33                  /* obs code: E6B+C,LEXS+L,B3I+Q (GAL,QZS,BDS) */
#define CODE_L6Z    34                  /* obs code: E6A+B+C,L6D+E (GAL,QZS) */
#define CODE_L6S    35                  /* obs code: L6S        (QZS) */
#define CODE_L6L    36                  /* obs code: L6L        (QZS) */
#define CODE_L8I    37                  /* obs code: E5abI      (GAL) */
#define CODE_L8Q    38                  /* obs code: E5abQ      (GAL) */
#define CODE_L8X    39                  /* obs code: E5abI+Q,B2abD+P (GAL,BDS) */
#define CODE_L2I    40                  /* obs code: B1_2I      (BDS) */
#define CODE_L2Q    41                  /* obs code: B1_2Q      (BDS) */
#define CODE_L6I    42                  /* obs code: B3I        (BDS) */
#define CODE_L6Q    43                  /* obs code: B3Q        (BDS) */
#define CODE_L3I    44                  /* obs code: G3I        (GLO) */
#define CODE_L3Q    45                  /* obs code: G3Q        (GLO) */
#define CODE_L3X    46                  /* obs code: G3I+Q      (GLO) */
#define CODE_L1I    47                  /* obs code: B1I        (BDS) (obsolute) */
#define CODE_L1Q    48                  /* obs code: B1Q        (BDS) (obsolute) */
#define CODE_L5A    49                  /* obs code: L5A SPS    (IRN) */
#define CODE_L5B    50                  /* obs code: L5B RS(D)  (IRN) */
#define CODE_L5C    51                  /* obs code: L5C RS(P)  (IRN) */
#define CODE_L9A    52                  /* obs code: SA SPS     (IRN) */
#define CODE_L9B    53                  /* obs code: SB RS(D)   (IRN) */
#define CODE_L9C    54                  /* obs code: SC RS(P)   (IRN) */
#define CODE_L9X    55                  /* obs code: SB+C       (IRN) */
#define CODE_L1D    56                  /* obs code: B1D        (BDS) */
#define CODE_L5D    57                  /* obs code: L5D(L5S),B2aD (QZS,BDS) */
#define CODE_L5P    58                  /* obs code: L5P(L5S),B2aP (QZS,BDS) */
#define CODE_L5Z    59                  /* obs code: L5D+P(L5S) (QZS) */
#define CODE_L6E    60                  /* obs code: L6E        (QZS) */
#define CODE_L7D    61                  /* obs code: B2bD       (BDS) */
#define CODE_L7P    62                  /* obs code: B2bP       (BDS) */
#define CODE_L7Z    63                  /* obs code: B2bD+P     (BDS) */
#define CODE_L8D    64                  /* obs code: B2abD      (BDS) */
#define CODE_L8P    65                  /* obs code: B2abP      (BDS) */
#define CODE_L4A    66                  /* obs code: G1aL1OCd   (GLO) */
#define CODE_L4B    67                  /* obs code: G1aL1OCd   (GLO) */
#define CODE_L4X    68                  /* obs code: G1al1OCd+p (GLO) */
#define MAXCODE     68                  /* max number of obs code */

#define MAXFREQ     8                   /* max NFREQ */
#define FREQ1       1.57542E9           /* L1/E1/B1C  frequency (Hz) */
#define FREQ2       1.22760E9           /* L2         frequency (Hz) */
#define FREQ5       1.17645E9           /* L5/E5a/B2a frequency (Hz) */
#define FREQ6       1.27875E9           /* E6/L6  frequency (Hz) */
#define FREQ7       1.20714E9           /* E5b    frequency (Hz) */
#define FREQ1_GLO   1.60200E9           /* GLONASS G1 base frequency (Hz) */
#define DFRQ1_GLO   0.56250E6           /* GLONASS G1 bias frequency (Hz/n) */
#define FREQ2_GLO   1.24600E9           /* GLONASS G2 base frequency (Hz) */
#define DFRQ2_GLO   0.43750E6           /* GLONASS G2 bias frequency (Hz/n) */
#define FREQ3_GLO   1.202025E9          /* GLONASS G3 frequency (Hz) */
#define FREQ1a_GLO  1.600995E9          /* GLONASS G1a frequency (Hz) */
#define FREQ2a_GLO  1.248060E9          /* GLONASS G2a frequency (Hz) */
#define FREQ1_CMP   1.561098E9          /* BDS B1I     frequency (Hz) */
#define FREQ2_CMP   1.20714E9           /* BDS B2I/B2b frequency (Hz) */
#define FREQ3_CMP   1.26852E9           /* BDS B3      frequency (Hz) */
#define FREQ8       1.191795E9          /* E5a+b  frequency (Hz) */
#define FREQ9       2.492028E9          /* S      frequency (Hz) */

#define NFREQ       7
#define NEXOBS      3


#define MINPRNGPS   1                   /* min satellite PRN number of GPS */
#define MAXPRNGPS   32                  /* max satellite PRN number of GPS */
#define NSATGPS     (MAXPRNGPS-MINPRNGPS+1) /* number of GPS satellites */
#define NSYSGPS     1

#ifndef DISGLO
#define MINPRNGLO   1                   /* min satellite slot number of GLONASS */
#define MAXPRNGLO   27                  /* max satellite slot number of GLONASS */
#define NSATGLO     (MAXPRNGLO-MINPRNGLO+1) /* number of GLONASS satellites */
#define NSYSGLO     1
#else
#define MINPRNGLO   0
#define MAXPRNGLO   0
#define NSATGLO     0
#define NSYSGLO     0
#endif
#ifndef  DISGAL
#define MINPRNGAL   1                   /* min satellite PRN number of Galileo */
#define MAXPRNGAL   36                  /* max satellite PRN number of Galileo */
#define NSATGAL    (MAXPRNGAL-MINPRNGAL+1) /* number of Galileo satellites */
#define NSYSGAL     1
#else
#define MINPRNGAL   0
#define MAXPRNGAL   0
#define NSATGAL     0
#define NSYSGAL     0
#endif
#ifndef DISQZS
#define MINPRNQZS   193                 /* min satellite PRN number of QZSS */
#define MAXPRNQZS   202                 /* max satellite PRN number of QZSS */
#define MINPRNQZS_S 183                 /* min satellite PRN number of QZSS L1S */
#define MAXPRNQZS_S 191                 /* max satellite PRN number of QZSS L1S */
#define NSATQZS     (MAXPRNQZS-MINPRNQZS+1) /* number of QZSS satellites */
#define NSYSQZS     1
#else
#define MINPRNQZS   0
#define MAXPRNQZS   0
#define MINPRNQZS_S 0
#define MAXPRNQZS_S 0
#define NSATQZS     0
#define NSYSQZS     0
#endif
#ifndef DISCMP
#define MINPRNCMP   1                   /* min satellite sat number of BeiDou */
#define MAXPRNCMP   63                  /* max satellite sat number of BeiDou */
#define NSATCMP     (MAXPRNCMP-MINPRNCMP+1) /* number of BeiDou satellites */
#define NSYSCMP     1
#else
#define MINPRNCMP   0
#define MAXPRNCMP   0
#define NSATCMP     0
#define NSYSCMP     0
#endif
#ifndef DISIRN
#define MINPRNIRN   1                   /* min satellite sat number of IRNSS */
#define MAXPRNIRN   14                  /* max satellite sat number of IRNSS */
#define NSATIRN     (MAXPRNIRN-MINPRNIRN+1) /* number of IRNSS satellites */
#define NSYSIRN     1
#else
#define MINPRNIRN   0
#define MAXPRNIRN   0
#define NSATIRN     0
#define NSYSIRN     0
#endif
#ifndef DISSBS
#define MINPRNSBS   120                 /* min satellite PRN number of SBAS */
#define MAXPRNSBS   158                 /* max satellite PRN number of SBAS */
#define NSATSBS     (MAXPRNSBS-MINPRNSBS+1) /* number of SBAS satellites */
#define NSYSSBS     1
#else
#define MINPRNSBS   0                   /* min satellite PRN number of SBAS */
#define MAXPRNSBS   0                   /* max satellite PRN number of SBAS */
#define NSATSBS     0                   /* number of SBAS satellites */
#define NSYSSBS     0
#endif

#define MAXSAT      (NSATGPS+NSATGLO+NSATGAL+NSATQZS+NSATCMP+NSATIRN+NSATSBS)

#ifndef MAXOBS
#define MAXOBS      96                  /* max number of obs in an epoch */
#endif

#define SNR_UNIT    0.001               /* SNR unit (dBHz) */

#define ROUND(x)    ((int)floor((x)+0.5))
#define ROUND_U(x)  ((uint32_t)floor((x)+0.5))



static FILE *fp_trace=NULL;     /* file pointer of trace */
static int level_trace=1;       /* level of trace */
static char file_trace[1024];   /* trace file */


typedef unsigned char uint8_t;
typedef unsigned int  uint32_t;
typedef int int32_t;


const int glo_fcn[32]={
                      1,-4,5,6,1,-4,5,6,
                      2,-7,0,-1,-2,-7,0,-1,
                      4,-3,3,2,4,-3,3,2,
                      -5,1,1,1,1,1,1,1  //not R26 R27
                     };

static const uint32_t tbl_CRC24Q[]={
    0x000000,0x864CFB,0x8AD50D,0x0C99F6,0x93E6E1,0x15AA1A,0x1933EC,0x9F7F17,
    0xA18139,0x27CDC2,0x2B5434,0xAD18CF,0x3267D8,0xB42B23,0xB8B2D5,0x3EFE2E,
    0xC54E89,0x430272,0x4F9B84,0xC9D77F,0x56A868,0xD0E493,0xDC7D65,0x5A319E,
    0x64CFB0,0xE2834B,0xEE1ABD,0x685646,0xF72951,0x7165AA,0x7DFC5C,0xFBB0A7,
    0x0CD1E9,0x8A9D12,0x8604E4,0x00481F,0x9F3708,0x197BF3,0x15E205,0x93AEFE,
    0xAD50D0,0x2B1C2B,0x2785DD,0xA1C926,0x3EB631,0xB8FACA,0xB4633C,0x322FC7,
    0xC99F60,0x4FD39B,0x434A6D,0xC50696,0x5A7981,0xDC357A,0xD0AC8C,0x56E077,
    0x681E59,0xEE52A2,0xE2CB54,0x6487AF,0xFBF8B8,0x7DB443,0x712DB5,0xF7614E,
    0x19A3D2,0x9FEF29,0x9376DF,0x153A24,0x8A4533,0x0C09C8,0x00903E,0x86DCC5,
    0xB822EB,0x3E6E10,0x32F7E6,0xB4BB1D,0x2BC40A,0xAD88F1,0xA11107,0x275DFC,
    0xDCED5B,0x5AA1A0,0x563856,0xD074AD,0x4F0BBA,0xC94741,0xC5DEB7,0x43924C,
    0x7D6C62,0xFB2099,0xF7B96F,0x71F594,0xEE8A83,0x68C678,0x645F8E,0xE21375,
    0x15723B,0x933EC0,0x9FA736,0x19EBCD,0x8694DA,0x00D821,0x0C41D7,0x8A0D2C,
    0xB4F302,0x32BFF9,0x3E260F,0xB86AF4,0x2715E3,0xA15918,0xADC0EE,0x2B8C15,
    0xD03CB2,0x567049,0x5AE9BF,0xDCA544,0x43DA53,0xC596A8,0xC90F5E,0x4F43A5,
    0x71BD8B,0xF7F170,0xFB6886,0x7D247D,0xE25B6A,0x641791,0x688E67,0xEEC29C,
    0x3347A4,0xB50B5F,0xB992A9,0x3FDE52,0xA0A145,0x26EDBE,0x2A7448,0xAC38B3,
    0x92C69D,0x148A66,0x181390,0x9E5F6B,0x01207C,0x876C87,0x8BF571,0x0DB98A,
    0xF6092D,0x7045D6,0x7CDC20,0xFA90DB,0x65EFCC,0xE3A337,0xEF3AC1,0x69763A,
    0x578814,0xD1C4EF,0xDD5D19,0x5B11E2,0xC46EF5,0x42220E,0x4EBBF8,0xC8F703,
    0x3F964D,0xB9DAB6,0xB54340,0x330FBB,0xAC70AC,0x2A3C57,0x26A5A1,0xA0E95A,
    0x9E1774,0x185B8F,0x14C279,0x928E82,0x0DF195,0x8BBD6E,0x872498,0x016863,
    0xFAD8C4,0x7C943F,0x700DC9,0xF64132,0x693E25,0xEF72DE,0xE3EB28,0x65A7D3,
    0x5B59FD,0xDD1506,0xD18CF0,0x57C00B,0xC8BF1C,0x4EF3E7,0x426A11,0xC426EA,
    0x2AE476,0xACA88D,0xA0317B,0x267D80,0xB90297,0x3F4E6C,0x33D79A,0xB59B61,
    0x8B654F,0x0D29B4,0x01B042,0x87FCB9,0x1883AE,0x9ECF55,0x9256A3,0x141A58,
    0xEFAAFF,0x69E604,0x657FF2,0xE33309,0x7C4C1E,0xFA00E5,0xF69913,0x70D5E8,
    0x4E2BC6,0xC8673D,0xC4FECB,0x42B230,0xDDCD27,0x5B81DC,0x57182A,0xD154D1,
    0x26359F,0xA07964,0xACE092,0x2AAC69,0xB5D37E,0x339F85,0x3F0673,0xB94A88,
    0x87B4A6,0x01F85D,0x0D61AB,0x8B2D50,0x145247,0x921EBC,0x9E874A,0x18CBB1,
    0xE37B16,0x6537ED,0x69AE1B,0xEFE2E0,0x709DF7,0xF6D10C,0xFA48FA,0x7C0401,
    0x42FA2F,0xC4B6D4,0xC82F22,0x4E63D9,0xD11CCE,0x575035,0x5BC9C3,0xDD8538
};

/* MSM signal ID table -------------------------------------------------------*/

const char *msm_sig_gps[32]={

    ""  ,"1C","1P","1W",""  ,""  ,""  ,"2C","2P","2W",""  ,""  , /*  1-12 */
    ""  ,""  ,"2S","2L","2X",""  ,""  ,""  ,""  ,"5I","5Q","5X", /* 13-24 */
    ""  ,""  ,""  ,""  ,""  ,"1S","1L","1X"                      /* 25-32 */
};

const char *msm_sig_glo[32]={
    ""  ,"1C","1P",""  ,""  ,""  ,""  ,"2C","2P",""  ,""  ,""  ,
    ""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,
    ""  ,""  ,""  ,""  ,""  ,""  ,""  ,""
};

const char *msm_sig_gal[32]={

    ""  ,"1C","1A","1B","1X","1Z",""  ,"6C","6A","6B","6X","6Z",
    ""  ,"7I","7Q","7X",""  ,"8I","8Q","8X",""  ,"5I","5Q","5X",
    ""  ,""  ,""  ,""  ,""  ,""  ,""  ,""
};

const char *msm_sig_cmp[32]={
    ""  ,"2I","2Q","2X",""  ,""  ,""  ,"6I","6Q","6X",""  ,""  ,
    ""  ,"7I","7Q","7X",""  ,""  ,""  ,""  ,""  ,"5D"  ,"5P"  ,"5X"  ,
    "7D"  ,""  ,""  ,""  ,""  ,"1D"  ,"1P"  ,"1X"
};


const char *msm_sig_sbs[32]={
    ""  ,"1C",""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,
    ""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,"5I","5Q","5X",
    ""  ,""  ,""  ,""  ,""  ,""  ,""  ,""
};

const char *msm_sig_qzs[32]={
    ""  ,"1C",""  ,""  ,""  ,""  ,""  ,""  ,"6S","6L","6X",""  ,
    ""  ,""  ,"2S","2L","2X",""  ,""  ,""  ,""  ,"5I","5Q","5X",
    ""  ,""  ,""  ,""  ,""  ,"1S","1L","1X"
};

const char *msm_sig_irn[32]={
    ""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,
    ""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,""  ,"5A",""  ,""  ,
    ""  ,""  ,""  ,""  ,""  ,""  ,""  ,""
};

static char *obscodes[]={       /* observation code strings */

    ""  ,"1C","1P","1W","1Y", "1M","1N","1S","1L","1E", /*  0- 9 */
    "1A","1B","1X","1Z","2C", "2D","2S","2L","2X","2P", /* 10-19 */
    "2W","2Y","2M","2N","5I", "5Q","5X","7I","7Q","7X", /* 20-29 */
    "6A","6B","6C","6X","6Z", "6S","6L","8L","8Q","8X", /* 30-39 */
    "2I","2Q","6I","6Q","3I", "3Q","3X","1I","1Q","5A", /* 40-49 */
    "5B","5C","9A","9B","9C", "9X","1D","5D","5P","5Z", /* 50-59 */
    "6E","7D","7P","7Z","8D", "8P","4A","4B","4X",""    /* 60-69 */
};

static char codepris[7][MAXFREQ][16]={  /* code priority for each freq-index */
   /*    0         1          2          3         4         5       6      7*/
    {"CPYWMNSLX","PYWCMNDLSX","IQX"     ,""        ,""       ,""      ,"",   ""}, /* GPS */
    {"CPABX"    ,"CPABX"     ,"IQX"     ,"CPABX"   ,"CPABX"  ,""      ,"",   ""}, /* GLO */
    {"CABXZ"    ,"IQX"       ,"IQX"     ,"ABCXZ"   ,"IQX"    ,""      ,"",   ""}, /* GAL */
    {"CLSXZ"    ,"LSX"       ,"IQXDPZ"  ,"LSXEZ"   ,""       ,""      ,"",   ""}, /* QZS */
    {"C"        ,"IQX"       ,""        ,""        ,""       ,""      ,"",   ""}, /* SBS */
    {"IQXDPAN"  ,"IQXDPZ"    ,"DPX"     ,"DPX"     ,"DPX"    ,"IQX"   ,"DPZX",""}, /* BDS */
    {"ABCX"     ,"ABCX"      ,""        ,""        ,""       ,""      ,"",   ""}  /* IRN */
};

static char obsfrqstr[8][MAXFREQ][5]={
        {"L1", "L2", "L5", "",   "",    "", "",""},       /*GPS*/
        {"G1", "G2", "G3", "G1a","G2a", "", "",""},       /*GLO*/
        {"E1", "E5b","E5a","E6", "E5ab","", "",""},       /*GAL*/
        {"L1", "L2", "L5", "L6", "",    "", "",""},       /*QZS*/
        {"L1", "L5", "","","",          "", "",""},        /*SBS*/
        {"B1I","B3I","B2a","B1C","B2ab","B2I", "B2b",""}, /*BDS*/
        {"L5", "S",  "",   "",   "",    "", "",""},       /*IRN*/
        {"L1", "L2", "",   "",   "",    "", "",""},      /*LEO*/
};

static int obsfrqidx[8][MAXFREQ][1]={
        {0,1,2,3,4,5,6,7},                               /*GPS*/
        {0,1,2,3,4,5,6,7},                               /*GLO*/
        {0,1,2,3,4,5,6,7},                               /*GAL*/
        {0,1,2,3,4,5,6,7},                               /*QZS*/
        {0,1,2,3,4,5,6,7},                               /*SBS*/
        {0,1,2,3,4,5,6,7},                               /*BDS*/
        {0,1,2,3,4,5,6,7},                               /*IRN*/
        {0,1,2,3,4,5,6,7},                               /*LEO*/
};

typedef struct {        /* observation data record */
//    gtime_t time;       /* receiver sampling time (GPST) */
    uint32_t sat,rcv;    /* satellite/receiver number */
    uint32_t SNR[NFREQ+NEXOBS]; /* signal strength (0.001 dBHz) */
    uint8_t  LLI[NFREQ+NEXOBS]; /* loss of lock indicator */
    uint32_t locktime[NFREQ+NEXOBS]; /* locktime*/
    uint8_t code[NFREQ+NEXOBS]; /* code indicator (CODE_???) */
    double L[NFREQ+NEXOBS]; /* observation data carrier-phase (cycle) */
    double P[NFREQ+NEXOBS]; /* observation data pseudorange (m) */
    float  D[NFREQ+NEXOBS]; /* observation data doppler frequency (Hz) */
} obsd_con;

typedef struct {        /* observation data */
    int n,nmax;         /* number of obervation data/allocated */
    obsd_con *data;       /* observation data records */
} obs_con;

typedef struct {              /* multi-signal-message header type */
//    uint8_t iod;              /* issue of data station */
//    uint8_t time_s;           /* cumulative session transmitting time */
//    uint8_t clk_str;          /* clock steering indicator */
//    uint8_t clk_ext;          /* external clock indicator */
//    uint8_t smooth;           /* divergence free smoothing indicator */
//    uint8_t tint_s;           /* soothing interval */
    uint8_t nsat,nsig;        /* number of satellites/signals */
    uint8_t sats[64];         /* satellites */
    uint8_t sigs[32];         /* signals */
    uint8_t cellmask[64];     /* cell mask */
} msm_h_con;


typedef struct {        /* RTCM control struct type */
//    int staid;          /* station id */
//    int stah;           /* station health */
//    int seqno;          /* sequence number for rtcm 2 or iods msm */
//    int outtype;        /* output message type */
//    gtime_t time;       /* message time */
//    gtime_t time_s;     /* message start time */
    obs_con obs;          /* observation data (uncorrected) */
//    nav_t nav;          /* satellite ephemerides */
//    sta_t sta;          /* station parameters */
//    dgps_t *dgps;       /* output of dgps corrections */
//    ssr_t ssr[MAXSAT];  /* output of ssr corrections */
//    char msg[128];      /* special message */
//    char msgtype[256];  /* last message type */
//    char msmtype[7][128]; /* msm signal types */
//    int obsflag;        /* obs data complete flag (1:ok,0:not complete) */
//    int ephsat;         /* input ephemeris satellite number */
//    int ephset;         /* input ephemeris set (0-1) */
//    double cp[MAXSAT][NFREQ+NEXOBS]; /* carrier-phase measurement */
//    uint32_t lock[MAXSAT][NFREQ+NEXOBS]; /* lock time */ //change ZRZ uint32_t
//    uint16_t loss[MAXSAT][NFREQ+NEXOBS]; /* loss of lock count */
//    gtime_t lltime[MAXSAT][NFREQ+NEXOBS]; /* last lock time */
//    int nbyte;          /* number of bytes in message buffer */
    int nbit;           /* number of bits in word buffer (bits) */
    int len;            /* message length (bytes) */
    int lensd;
    uint8_t buff[1200]; /* message buffer */
    uint8_t buffsd[1200];
//    uint32_t word;      /* word buffer for rtcm 2 */
//    uint32_t nmsg2[100]; /* message count of RTCM 2 (1-99:1-99,0:other) */
//    uint32_t nmsg3[400]; /* message count of RTCM 3 (1-299:1001-1299,300-329:4070-4099,0:ohter) */
//    char opt[256];      /* RTCM dependent options */
} rtcm_con;


int m_gnss_frq_idx[8][MAXFREQ];
int m_gnss_frq_num[8];

static int obsfrqstr2idx(const char* frq_str,int sys_idx)
{
    int i,idx=0;
    for(i=0;i<MAXFREQ;i++){
        if(!strcmp(frq_str,obsfrqstr[sys_idx][i])){
            idx = *obsfrqidx[sys_idx][i];
            break;
        }
    }
    return idx;
}

static int getobsfrqidx(char* frq_str,int sys,int *idxs)
{
    int sys_idx=0,i=0;
    for (int j=0;j<NFREQ;j++){
        idxs[j] = MAXFREQ;
    }
    switch (sys){
        case SYS_GLO: sys_idx = 1;break;
        case SYS_GAL: sys_idx = 2;break;
        case SYS_QZS: sys_idx = 3;break;
        case SYS_SBS: sys_idx = 4;break;
        case SYS_CMP: sys_idx = 5;break;
        case SYS_IRN: sys_idx = 6;break;
//        case SYS_LEO: sys_idx = 7;break;
    }

    char *token;

    token = strtok(frq_str,"+");
    while(token){
        idxs[i++]=obsfrqstr2idx(token,sys_idx)+1;
        token=strtok(NULL,"+");
    }
    return i;

}



static void setfrqpri(char frq_[40],int i)
{
    int sys;
        switch (i){
            case 0: sys=SYS_GPS; m_gnss_frq_num[i]=getobsfrqidx(frq_,sys,m_gnss_frq_idx[0]);break;
            case 1: sys=SYS_GLO; m_gnss_frq_num[i]=getobsfrqidx(frq_,sys,m_gnss_frq_idx[1]);break;
            case 2: sys=SYS_GAL; m_gnss_frq_num[i]=getobsfrqidx(frq_,sys,m_gnss_frq_idx[2]);break;
            case 3: sys=SYS_QZS; m_gnss_frq_num[i]=getobsfrqidx(frq_,sys,m_gnss_frq_idx[3]);break;
            case 4: sys=SYS_SBS; m_gnss_frq_num[i]=getobsfrqidx(frq_,sys,m_gnss_frq_idx[4]);break;
            case 5: sys=SYS_CMP; m_gnss_frq_num[i]=getobsfrqidx(frq_,sys,m_gnss_frq_idx[5]);break;
            case 6: sys=SYS_IRN; m_gnss_frq_num[i]=getobsfrqidx(frq_,sys,m_gnss_frq_idx[6]);break;
        }


}

API_DECLSPEC void rtcmlogopen(const char *file)
{
//        gtime_t time=utc2gpst(timeget());
        char path[1024];

//        reppath(file,path,time,"","");
        if (!*file||!(fp_trace=fopen(file,"w"))) fp_trace=stderr;
        strcpy(file_trace,file);
//        tick_trace=tickget();
//        time_trace=time;
//        initlock(&lock_trace);
}

API_DECLSPEC void rtcmlogclose(void)
{
        if (fp_trace&&fp_trace!=stderr) fclose(fp_trace);
        fp_trace=NULL;
        file_trace[0]='\0';
}
API_DECLSPEC void rtcmloglevel(int level)
{
        level_trace=level;
}


static void trace(int level, const char *format, ...)
{
    va_list ap;

    /* print error message to stderr */
    if (level<=1) {
        va_start(ap,format); vfprintf(stderr,format,ap); va_end(ap);
    }
    if (!fp_trace||level>level_trace) return;
    fprintf(fp_trace,"%d ",level);
    va_start(ap,format); vfprintf(fp_trace,format,ap); va_end(ap);
    fflush(fp_trace);
}

/* crc-24q parity --------------------------------------------------------------
* compute crc-24q parity for sbas, rtcm3
* args   : uint8_t *buff    I   data
*          int    len       I   data length (bytes)
* return : crc-24Q parity
* notes  : see reference [2] A.4.3.3 Parity
*-----------------------------------------------------------------------------*/
static uint32_t rtk_crc24q(const uint8_t *buff, int len)
{
    uint32_t crc=0;
    int i;

    trace(2,"rtk_crc24q: len=%d\n",len);

    for (i=0;i<len;i++) crc=((crc<<8)&0xFFFFFF)^tbl_CRC24Q[(crc>>16)^buff[i]];
    return crc;
}

/* extract unsigned/signed bits ------------------------------------------------
* extract unsigned/signed bits from byte data
* args   : uint8_t *buff    I   byte data
*          int    pos       I   bit position from start of data (bits)
*          int    len       I   bit length (bits) (len<=32)
* return : extracted unsigned/signed bits
*-----------------------------------------------------------------------------*/
static uint32_t getbitu(const uint8_t *buff, int pos, int len)
{
    uint32_t bits=0;
    int i;
    for (i=pos;i<pos+len;i++) bits=(bits<<1)+((buff[i/8]>>(7-i%8))&1u);
    return bits;
}

/* obs code to obs code string -------------------------------------------------
* convert obs code to obs code string
* args   : uint8_t code     I   obs code (CODE_???)
* return : obs code string ("1C","1P","1P",...)
* notes  : obs codes are based on RINEX 3.04
*-----------------------------------------------------------------------------*/
static char *code2obs(uint8_t code)
{
    if (code<=CODE_NONE||MAXCODE<code) return "";
    return obscodes[code];
}


static int32_t getbits(const uint8_t *buff, int pos, int len)
{
    uint32_t bits=getbitu(buff,pos,len);
    if (len<=0||32<=len||!(bits&(1u<<(len-1)))) return (int32_t)bits;
    return (int32_t)(bits|(~0u<<len)); /* extend sign */
}

/* set unsigned/signed bits ----------------------------------------------------
* set unsigned/signed bits to byte data
* args   : uint8_t *buff IO byte data
*          int    pos       I   bit position from start of data (bits)
*          int    len       I   bit length (bits) (len<=32)
*          [u]int32_t data  I   unsigned/signed data
* return : none
*-----------------------------------------------------------------------------*/
static void setbitu(uint8_t *buff, int pos, int len, uint32_t data)
{
    uint32_t mask=1u<<(len-1);
    int i;
    if (len<=0||32<len) return;
    for (i=pos;i<pos+len;i++,mask>>=1) {
        if (data&mask) buff[i/8]|=1u<<(7-i%8); else buff[i/8]&=~(1u<<(7-i%8));
    }
}
static void setbits(uint8_t *buff, int pos, int len, int32_t data)
{
    if (data<0) data|=1<<(len-1); else data&=~(1<<(len-1)); /* set sign bit */
    setbitu(buff,pos,len,(uint32_t)data);
}

/* satellite system+prn/slot number to satellite number ------------------------
* convert satellite system+prn/slot number to satellite number
* args   : int    sys       I   satellite system (SYS_GPS,SYS_GLO,...)
*          int    prn       I   satellite prn/slot number
* return : satellite number (0:error)
*-----------------------------------------------------------------------------*/
static int satno(int sys, int prn)
{
    if (prn<=0) return 0;
    switch (sys) {
        case SYS_GPS:
            if (prn<MINPRNGPS||MAXPRNGPS<prn) return 0;
            return prn-MINPRNGPS+1;
        case SYS_GLO:
            if (prn<MINPRNGLO||MAXPRNGLO<prn) return 0;
            return NSATGPS+prn-MINPRNGLO+1;
        case SYS_GAL:
            if (prn<MINPRNGAL||MAXPRNGAL<prn) return 0;
            return NSATGPS+NSATGLO+prn-MINPRNGAL+1;
        case SYS_QZS:
            if (prn<MINPRNQZS||MAXPRNQZS<prn) return 0;
            return NSATGPS+NSATGLO+NSATGAL+prn-MINPRNQZS+1;
        case SYS_CMP:
            if (prn<MINPRNCMP||MAXPRNCMP<prn) return 0;
            return NSATGPS+NSATGLO+NSATGAL+NSATQZS+prn-MINPRNCMP+1;
        case SYS_IRN:
            if (prn<MINPRNIRN||MAXPRNIRN<prn) return 0;
            return NSATGPS+NSATGLO+NSATGAL+NSATQZS+NSATCMP+prn-MINPRNIRN+1;
//        case SYS_LEO:
//            if (prn<MINPRNLEO||MAXPRNLEO<prn) return 0;
//            return NSATGPS+NSATGLO+NSATGAL+NSATQZS+NSATCMP+NSATIRN+
//                   prn-MINPRNLEO+1;
        case SYS_SBS:
            if (prn<MINPRNSBS||MAXPRNSBS<prn) return 0;
            return NSATGPS+NSATGLO+NSATGAL+NSATQZS+NSATCMP+NSATIRN+prn-MINPRNSBS+1;
    }
    return 0;
}
/* satellite number to satellite system ----------------------------------------
* convert satellite number to satellite system
* args   : int    sat       I   satellite number (1-MAXSAT)
*          int    *prn      IO  satellite prn/slot number (NULL: no output)
* return : satellite system (SYS_GPS,SYS_GLO,...)
*-----------------------------------------------------------------------------*/
extern int satsys(int sat, int *prn)
{
    int sys=SYS_NONE;
    if (sat<=0||MAXSAT<sat) sat=0;
    else if (sat<=NSATGPS) {
        sys=SYS_GPS; sat+=MINPRNGPS-1;
    }
    else if ((sat-=NSATGPS)<=NSATGLO) {
        sys=SYS_GLO; sat+=MINPRNGLO-1;
    }
    else if ((sat-=NSATGLO)<=NSATGAL) {
        sys=SYS_GAL; sat+=MINPRNGAL-1;
    }
    else if ((sat-=NSATGAL)<=NSATQZS) {
        sys=SYS_QZS; sat+=MINPRNQZS-1;
    }
    else if ((sat-=NSATQZS)<=NSATCMP) {
        sys=SYS_CMP; sat+=MINPRNCMP-1;
    }
    else if ((sat-=NSATCMP)<=NSATIRN) {
        sys=SYS_IRN; sat+=MINPRNIRN-1;
    }
    else if ((sat-=NSATIRN)<=NSATSBS) {
        sys=SYS_SBS; sat+=MINPRNSBS-1;
    }
    else sat=0;
    if (prn) *prn=sat;
    return sys;
}

static uint8_t obs2code(const char *obs)
{
    int i;

    for (i=1;*obscodes[i];i++) {
        if (strcmp(obscodes[i],obs)) continue;
        return (uint8_t)i;
    }
    return CODE_NONE;
}
/* satellite no to MSM satellite ID ------------------------------------------*/
static int to_satid(int sys, int sat)
{
    int prn;

    if (satsys(sat,&prn)!=sys) return 0;

    if      (sys==SYS_QZS) prn-=MINPRNQZS-1;
    else if (sys==SYS_SBS) prn-=MINPRNSBS-1;

    return prn;
}
/* observation code to MSM signal ID -----------------------------------------*/
static int to_sigid(int sys, uint8_t code)
{
    const char **msm_sig;
    char *sig;
    int i;

    /* signal conversion for undefined signal by rtcm */
    if (sys==SYS_GPS) {
        if      (code==CODE_L1Y) code=CODE_L1P;
        else if (code==CODE_L1M) code=CODE_L1P;
        else if (code==CODE_L1N) code=CODE_L1P;
        else if (code==CODE_L2D) code=CODE_L2P;
        else if (code==CODE_L2Y) code=CODE_L2P;
        else if (code==CODE_L2M) code=CODE_L2P;
        else if (code==CODE_L2N) code=CODE_L2P;
    }
    if (!*(sig=code2obs(code))) return 0;

    switch (sys) {
        case SYS_GPS: msm_sig=msm_sig_gps; break;
        case SYS_GLO: msm_sig=msm_sig_glo; break;
        case SYS_GAL: msm_sig=msm_sig_gal; break;
        case SYS_QZS: msm_sig=msm_sig_qzs; break;
        case SYS_SBS: msm_sig=msm_sig_sbs; break;
        case SYS_CMP: msm_sig=msm_sig_cmp; break;
        case SYS_IRN: msm_sig=msm_sig_irn; break;
        default: return 0;
    }
    for (i=0;i<32;i++) {
        if (!strcmp(sig,msm_sig[i])) return i+1;
    }
    return 0;
}
/* m_gnss_frq_idx[0] */

/* GPS obs code to frequency -------------------------------------------------*/
static int code2freq_GPS(uint8_t code, double *freq,int *ord)
{
    int idx[NFREQ];
    for (int i= 0;i<NFREQ;i++){
        idx[i] = 2;
        idx[i] = NFREQ;
    }
    for (int i=0;i<m_gnss_frq_num[0];i++){
        idx[m_gnss_frq_idx[0][i]-1] =i;
    }
//    idx[m_gnss_frq_idx[0][0]-1] =0;
//    idx[m_gnss_frq_idx[0][1]-1] =1;

    char *obs=code2obs(code);
    switch (obs[0]) {
        case '1': *freq=FREQ1; *ord=0; return idx[0]; /* L1 */
        case '2': *freq=FREQ2; *ord=1; return idx[1]; /* L2 */
        case '5': *freq=FREQ5; *ord=2; return idx[2]; /* L5 */
    }
    return -1;
}

/* GLONASS obs code to frequency ---------------------------------------------*/
static int code2freq_GLO(uint8_t code, int fcn, double *freq,int *ord)
{
    int idx[NFREQ];
    for (int i= 0;i<NFREQ;i++){
        idx[i] = 4;
        idx[i] = NFREQ;
    }
    for (int i=0;i<m_gnss_frq_num[1];i++){
        idx[m_gnss_frq_idx[1][i]-1] =i;
    }
//    idx[m_gnss_frq_idx[1][0]-1] =0;
//    idx[m_gnss_frq_idx[1][1]-1] =1;
//    idx[m_gnss_frq_idx[1][2]-1] =2;
//    idx[m_gnss_frq_idx[1][3]-1] =3;

    char *obs=code2obs(code);

    if (fcn<-7||fcn>6) return -1;

    switch (obs[0]) {
        case '1': *freq=FREQ1_GLO+DFRQ1_GLO*fcn; *ord=0; return idx[0]; /* G1 */
        case '2': *freq=FREQ2_GLO+DFRQ2_GLO*fcn; *ord=1; return idx[1]; /* G2 */
        case '3': *freq=FREQ3_GLO;               *ord=2; return idx[2]; /* G3 */
        case '4': *freq=FREQ1a_GLO;              *ord=3; return idx[3]; /* G1a */
        case '6': *freq=FREQ2a_GLO;              *ord=4; return idx[4]; /* G2a */
    }
    return -1;
}

/* Galileo obs code to frequency ---------------------------------------------*/
static int code2freq_GAL(uint8_t code, double *freq,int *ord)
{
    int idx[NFREQ];
    for (int i= 0;i<NFREQ;i++){
        idx[i] = 4;
        idx[i] = NFREQ;
    }
    for (int i=0;i<m_gnss_frq_num[2];i++){
        idx[m_gnss_frq_idx[2][i]-1] =i;
    }
//    idx[m_gnss_frq_idx[2][0]-1] =0;
//    idx[m_gnss_frq_idx[2][1]-1] =1;
//    idx[m_gnss_frq_idx[2][2]-1] =2;
//    idx[m_gnss_frq_idx[2][3]-1] =3;

    char *obs=code2obs(code);
    switch (obs[0]) {
        case '1': *freq=FREQ1; *ord=0; return idx[0]; /* E1 */
        case '7': *freq=FREQ7; *ord=1; return idx[1]; /* E5b */
        case '5': *freq=FREQ5; *ord=2; return idx[2]; /* E5a */
        case '6': *freq=FREQ6; *ord=3; return idx[3]; /* E6 */
        case '8': *freq=FREQ8; *ord=4; return idx[4]; /* E5ab */
    }
    return -1;
}

/* QZSS obs code to frequency ------------------------------------------------*/
static int code2freq_QZS(uint8_t code, double *freq,int *ord)
{
    int idx[NFREQ];
    for (int i= 0;i<NFREQ;i++){
        idx[i] = 3;
        idx[i] = NFREQ;
    }
    for (int i=0;i<m_gnss_frq_num[3];i++){
        idx[m_gnss_frq_idx[3][i]-1] =i;
    }
//    idx[m_gnss_frq_idx[3][0]-1] =0;
//    idx[m_gnss_frq_idx[3][1]-1] =1;
//    idx[m_gnss_frq_idx[3][2]-1] =2;

    char *obs=code2obs(code);

    switch (obs[0]) {
        case '1': *freq=FREQ1; *ord=0; return idx[0]; /* L1 */
        case '2': *freq=FREQ2; *ord=1; return idx[1]; /* L2 */
        case '5': *freq=FREQ5; *ord=2; return idx[2]; /* L5 */
        case '6': *freq=FREQ6; *ord=3; return idx[3]; /* L6 */
    }
    return -1;
}

/* SBAS obs code to frequency ------------------------------------------------*/
static int code2freq_SBS(uint8_t code, double *freq,int *ord)
{
    int idx[NFREQ];
    for (int i= 0;i<NFREQ;i++){
        idx[i] = 2;
        idx[i] = NFREQ;
    }
    for (int i=0;i<m_gnss_frq_num[4];i++){
        idx[m_gnss_frq_idx[4][i]-1] =i;
    }
//    idx[m_gnss_frq_idx[4][0]-1] =0;
//    idx[m_gnss_frq_idx[4][1]-1] =1;

    char *obs=code2obs(code);

    switch (obs[0]) {
        case '1': *freq=FREQ1; *ord=0; return idx[0]; /* L1 */
        case '5': *freq=FREQ5; *ord=1; return idx[1]; /* L5 */
    }
    return -1;
}

static int code2freq_BDS(uint8_t code, double *freq,int *ord)
{
    int idx[NFREQ];
    for (int i= 0;i<NFREQ;i++){
        idx[i] = 6;
        idx[i] = NFREQ;
    }
    for (int i=0;i<m_gnss_frq_num[5];i++){
        idx[m_gnss_frq_idx[5][i]-1] =i;
    }
//    idx[m_gnss_frq_idx[5][0]-1] =0;
//    idx[m_gnss_frq_idx[5][1]-1] =1;
//    idx[m_gnss_frq_idx[5][2]-1] =2;
//    idx[m_gnss_frq_idx[5][3]-1] =3;
//    idx[m_gnss_frq_idx[5][4]-1] =4;
//    idx[m_gnss_frq_idx[5][5]-1] =5;
    char *obs=code2obs(code);

    switch (obs[0]) {
        case '2': *freq=FREQ1_CMP; *ord=0; return idx[0]; /* B1I */
        case '6': *freq=FREQ3_CMP; *ord=1; return idx[1]; /* B3 */
        case '5': *freq=FREQ5;     *ord=2; return idx[2]; /* B2a */
        case '1': *freq=FREQ1;     *ord=3; return idx[3]; /* B1C */
        case '8': *freq=FREQ8;     *ord=4; return idx[4]; /* B2ab */
        case '7': {
            if (obs[1]=='I'||obs[1]=='Q'||obs[1]=='X'){
                *freq=FREQ2_CMP; *ord=5; return idx[5]; /* B2I*/
            }
            else {
                *freq=FREQ2_CMP; *ord=6; return idx[6]; /* B2b */
            }
        }
    }
    return -1;
}

/* NavIC obs code to frequency -----------------------------------------------*/
static int code2freq_IRN(uint8_t code, double *freq,int *ord)
{

    int idx[NFREQ];
    for (int i= 0;i<NFREQ;i++){
        idx[i] = 2;
        idx[i] = NFREQ;
    }
    for (int i=0;i<m_gnss_frq_num[6];i++){
        idx[m_gnss_frq_idx[6][i]-1] =i;
    }
//    idx[m_gnss_frq_idx[6][0]-1] =0;
//    idx[m_gnss_frq_idx[6][1]-1] =1;

    char *obs=code2obs(code);

    switch (obs[0]) {
        case '5': *freq=FREQ5; *ord=0; return 0; /* L5 */
        case '9': *freq=FREQ9; *ord=1; return 1; /* S */
    }
    return -1;
}

/* system and obs code to frequency --------------------------------------------
* convert system and obs code to carrier frequency
* args   : int    sys       I   satellite system (SYS_???)
*          uint8_t code     I   obs code (CODE_???)
*          int    fcn       I   frequency channel number for GLONASS
* return : carrier frequency (Hz) (0.0: error)
*-----------------------------------------------------------------------------*/
static double code2freq(int sys, uint8_t code, int fcn)
{
    double freq=0.0;
    int ord;

    switch (sys) {
        case SYS_GPS: (void)code2freq_GPS(code,&freq,&ord); break;
        case SYS_GLO: (void)code2freq_GLO(code,fcn,&freq,&ord); break;
        case SYS_GAL: (void)code2freq_GAL(code,&freq,&ord); break;
        case SYS_QZS: (void)code2freq_QZS(code,&freq,&ord); break;
        case SYS_SBS: (void)code2freq_SBS(code,&freq,&ord); break;
        case SYS_CMP: (void)code2freq_BDS(code,&freq,&ord); break;
        case SYS_IRN: (void)code2freq_IRN(code,&freq,&ord); break;
    }
    return freq;
}
static int code2idx(int sys, uint8_t code)
{
    double freq;
    int ord;

    switch (sys) {
        case SYS_GPS: return code2freq_GPS(code,&freq,&ord);
        case SYS_GLO: return code2freq_GLO(code,0,&freq,&ord);
        case SYS_GAL: return code2freq_GAL(code,&freq,&ord);
        case SYS_QZS: return code2freq_QZS(code,&freq,&ord);
        case SYS_SBS: return code2freq_SBS(code,&freq,&ord);
        case SYS_CMP: return code2freq_BDS(code,&freq,&ord);
        case SYS_IRN: return code2freq_IRN(code,&freq,&ord);
    }
    return -1;
}

static int systbl(int sys){
    int ord;

    switch (sys) {
        case SYS_GPS: ord=0;break;
        case SYS_GLO: ord=1;break;
        case SYS_GAL: ord=2;break;
        case SYS_QZS: ord=3;break;
        case SYS_SBS: ord=4;break;
        case SYS_CMP: ord=5;break;
        case SYS_IRN: ord=6;break;
    }
    return ord;

}

static int codeidxtbl(int sys, uint8_t code){
    double freq;
    int ord;

    switch (sys) {
        case SYS_GPS: code2freq_GPS(code,&freq,&ord);break;
        case SYS_GLO: code2freq_GLO(code,0,&freq,&ord);break;
        case SYS_GAL: code2freq_GAL(code,&freq,&ord);break;
        case SYS_QZS: code2freq_QZS(code,&freq,&ord);break;
        case SYS_SBS: code2freq_SBS(code,&freq,&ord);break;
        case SYS_CMP: code2freq_BDS(code,&freq,&ord);break;
        case SYS_IRN: code2freq_IRN(code,&freq,&ord);break;
    }
    return ord;
}


/* get code priority -----------------------------------------------------------
* get code priority for multiple codes in a frequency
* args   : int    sys       I   system (SYS_???)
*          uint8_t code     I   obs code (CODE_???)
*          char   *opt      I   code options (NULL:no option)
* return : priority (15:highest-1:lowest,0:error)
*-----------------------------------------------------------------------------*/
static int getcodepri(int sys, uint8_t code, const char *opt)
{
    const char *p,*optstr;
    char *obs,str[8]="";
    int i,j;

    switch (sys) {
        case SYS_GPS: i=0; optstr="-GL%2s"; break;
        case SYS_GLO: i=1; optstr="-RL%2s"; break;
        case SYS_GAL: i=2; optstr="-EL%2s"; break;
        case SYS_QZS: i=3; optstr="-JL%2s"; break;
        case SYS_SBS: i=4; optstr="-SL%2s"; break;
        case SYS_CMP: i=5; optstr="-CL%2s"; break;
        case SYS_IRN: i=6; optstr="-IL%2s"; break;
        default: return 0;
    }
//    if ((j=code2idx(sys,code))<0) return 0;codeidxtbl//change
    j=codeidxtbl(sys,code);
    obs=code2obs(code);

    /* parse code options */
    for (p=opt;p&&(p=strchr(p,'-'));p++) {
        if (sscanf(p,optstr,str)<1||str[0]!=obs[0]) continue;
        return str[1]==obs[1]?15:0;
    }
    /* search code priority */
    return (p=strchr(codepris[i][j],obs[1]))?14-(int)(p-codepris[i][j]):0;
}

/* get signal index ----------------------------------------------------------*/
static void sigindex(int sys, const uint8_t *code, int n, const char *opt,
                     int *idx)
{
    int i,nex,pri,pri_h[8]={0},index[8]={0},ex[32]={0};

    /* test code priority */
    for (i=0;i<n;i++) {
        if (!code[i]) continue;

        if (idx[i]>=NFREQ) { /* save as extended signal if idx >= NFREQ */
            ex[i]=1;
            continue;
        }
        /* code priority */
        pri=getcodepri(sys,code[i],opt);

        /* select highest priority signal */
        if (pri>pri_h[idx[i]]) {
            if (index[idx[i]]) ex[index[idx[i]]-1]=1;
            pri_h[idx[i]]=pri;
            index[idx[i]]=i+1;
        }
        else ex[i]=1;
    }
    /* signal index in obs data */
    for (i=nex=0;i<n;i++) {
        if (ex[i]==0) ;
        else if (nex<NEXOBS) idx[i]=NFREQ+nex++;
        else { /* no space in obs data */
            trace(1,"rtcm msm: no space in obs data sys=%d code=%d\n",sys,code[i]);
            idx[i]=-1;
        }
#if 0 /* for debug */
        trace(2,"sig pos: sys=%d code=%d ex=%d idx=%d\n",sys,code[i],ex[i],idx[i]);
#endif
    }
}

/* get observation data index ------------------------------------------------*/
static int obsindex(obs_con *obs, int sat)
{
    int i,j;

    for (i=0;i<obs->n;i++) {
        if (obs->data[i].sat==sat) return i; /* field already exists */
    }
    if (i>=MAXOBS) return -1; /* overflow */

    /* add new field */
//    obs->data[i].time=time;
    obs->data[i].sat=sat;
    for (j=0;j<NFREQ+NEXOBS;j++) {
        obs->data[i].L[j]=obs->data[i].P[j]=0.0;
        obs->data[i].D[j]=0.0;
        obs->data[i].SNR[j]=obs->data[i].LLI[j]=obs->data[i].code[j]=0;
        obs->data[i].locktime[j]=0;
    }
    obs->n++;
    return i;
}

/* decode type MSM message header --------------------------------------------*/
static int decode_msm_head(rtcm_con *rtcm, int sys, int *sync, int *iod,
                           msm_h_con *h, int *hsize)
{
    msm_h_con h0={0};
    double tow,tod;
    char *msg,tstr[64];
    int i=24,j,dow,mask,staid,type,ncell=0;
    int temp;

    type=getbitu(rtcm->buff,i,12); i+=12;
    //i+=12;

    *h=h0;
    if (i+157<=rtcm->len*8) {
        i+=12;
        i+=30;
        *sync     =getbitu(rtcm->buff,i, 1);       i+= 1;
        if (*sync==0){
            trace(1,"sync!\n");
            temp=i;
        }
        i+= 3;
        i+= 7;
        i+= 2;
        i+= 2;
        i+= 1;
        i+= 3;

//        staid     =getbitu(rtcm->buff,i,12);       i+=12;

//        if (sys==SYS_GLO) {
//            dow   =getbitu(rtcm->buff,i, 3);       i+= 3;
//            tod   =getbitu(rtcm->buff,i,27)*0.001; i+=27;
//            adjday_glot(rtcm,tod);
//        }
//        else if (sys==SYS_CMP) {
//            tow   =getbitu(rtcm->buff,i,30)*0.001; i+=30;
//            tow+=14.0; /* BDT -> GPST */
//            adjweek(rtcm,tow);
//        }
//        else {
//            tow   =getbitu(rtcm->buff,i,30)*0.001; i+=30;
//            adjweek(rtcm,tow);
//        }
//        *sync     =getbitu(rtcm->buff,i, 1);       i+= 1;
//        if (*sync==0){
//            msg="";
//        }
//        *iod      =getbitu(rtcm->buff,i, 3);       i+= 3;
//        h->time_s =getbitu(rtcm->buff,i, 7);       i+= 7;
//        h->clk_str=getbitu(rtcm->buff,i, 2);       i+= 2;
//        h->clk_ext=getbitu(rtcm->buff,i, 2);       i+= 2;
//        h->smooth =getbitu(rtcm->buff,i, 1);       i+= 1;
//        h->tint_s =getbitu(rtcm->buff,i, 3);       i+= 3;
        for (j=1;j<=64;j++) {
            mask=getbitu(rtcm->buff,i,1); i+=1;
            if (mask) h->sats[h->nsat++]=j;
        }
        for (j=1;j<=32;j++) {
            mask=getbitu(rtcm->buff,i,1); i+=1;
            if (mask) h->sigs[h->nsig++]=j;
        }
    }
    else {
        trace(1,"rtcm3 %d length error: len=%d\n",type,rtcm->len);
        return -1;
    }
//    /* test station id */
//    if (!test_staid(rtcm,staid)) return -1;

    if (h->nsat*h->nsig>64) {
        trace(1,"rtcm3 %d number of sats and sigs error: nsat=%d nsig=%d\n",
              type,h->nsat,h->nsig);
        return -1;
    }
    if (i+h->nsat*h->nsig>rtcm->len*8) {
        trace(1,"rtcm3 %d length error: len=%d nsat=%d nsig=%d\n",type,
              rtcm->len,h->nsat,h->nsig);
        return -1;
    }
    for (j=0;j<h->nsat*h->nsig;j++) {
        h->cellmask[j]=getbitu(rtcm->buff,i,1); i+=1;
        if (h->cellmask[j]) ncell++;
    }
    *hsize=i;

//    time2str(rtcm->time,tstr,2);
//    trace(4,"decode_head_msm: time=%s sys=%d staid=%d nsat=%d nsig=%d sync=%d iod=%d ncell=%d\n",
//          tstr,sys,staid,h->nsat,h->nsig,*sync,*iod,ncell);

//    if (rtcm->outtype) {
//        msg=rtcm->msgtype+strlen(rtcm->msgtype);
//        sprintf(msg," staid=%4d %s nsat=%2d nsig=%2d iod=%2d ncell=%2d sync=%d",
//                staid,tstr,h->nsat,h->nsig,*iod,ncell,*sync);
//    }
    return ncell;
}

/* loss-of-lock indicator ----------------------------------------------------*/
static int lossoflock(rtcm_con *rtcm, int sat, int idx, int lock)
{
//    int lli=(!lock&&!rtcm->lock[sat-1][idx])||lock<rtcm->lock[sat-1][idx];
//    rtcm->lock[sat-1][idx]=(uint32_t)lock;
    int lli=!lock;
    return lli;
}


/* save obs data in MSM message ----------------------------------------------*/
static void save_msm_obs(rtcm_con *rtcm, int sys, msm_h_con *h, const double *r,
                         const double *pr, const double *cp, const double *rr,
                         const double *rrf, const double *cnr, const int *lock,
                         const int *ex, const int *half)
{
    const char *sig[32];
    double tt,freq;
    uint8_t code[32]={0};
    char *msm_type="",*q=NULL;
    int temp;
    int i,j,k,type,prn,sat,fcn,index=0,idx[32];

    type=getbitu(rtcm->buff,24,12);

//    switch (sys) {
//        case SYS_GPS: msm_type=q=rtcm->msmtype[0]; break;
//        case SYS_GLO: msm_type=q=rtcm->msmtype[1]; break;
//        case SYS_GAL: msm_type=q=rtcm->msmtype[2]; break;
//        case SYS_QZS: msm_type=q=rtcm->msmtype[3]; break;
//        case SYS_SBS: msm_type=q=rtcm->msmtype[4]; break;
//        case SYS_CMP: msm_type=q=rtcm->msmtype[5]; break;
//        case SYS_IRN: msm_type=q=rtcm->msmtype[6]; break;
//    }
    /* id to signal */
    for (i=0;i<h->nsig;i++) {
        switch (sys) {
            case SYS_GPS: sig[i]=msm_sig_gps[h->sigs[i]-1]; break;
            case SYS_GLO: sig[i]=msm_sig_glo[h->sigs[i]-1]; break;
            case SYS_GAL: sig[i]=msm_sig_gal[h->sigs[i]-1]; break;
            case SYS_QZS: sig[i]=msm_sig_qzs[h->sigs[i]-1]; break;
            case SYS_SBS: sig[i]=msm_sig_sbs[h->sigs[i]-1]; break;
            case SYS_CMP: sig[i]=msm_sig_cmp[h->sigs[i]-1]; break;
            case SYS_IRN: sig[i]=msm_sig_irn[h->sigs[i]-1]; break;
            default: sig[i]=""; break;
        }
        /* signal to rinex obs type */
        code[i]=obs2code(sig[i]);
        idx[i]=code2idx(sys,code[i]);

        if (code[i]!=CODE_NONE) {
            if (q) q+=sprintf(q,"L%s%s",sig[i],i<h->nsig-1?",":"");
        }
        else {
            if (q) q+=sprintf(q,"(%d)%s",h->sigs[i],i<h->nsig-1?",":"");

            trace(1,"rtcm3 %d: unknown signal id=%2d\n",type,h->sigs[i]);
        }
    }
    trace(3,"rtcm3 %d: signals=%s\n",type,msm_type);

    /* get signal index */
//    sigindex(sys,code,h->nsig,rtcm->opt,idx);
    sigindex(sys,code,h->nsig,"",idx);

    for (i=j=0;i<h->nsat;i++) {

        prn=h->sats[i];
        if      (sys==SYS_QZS) prn+=MINPRNQZS-1;
        else if (sys==SYS_SBS) prn+=MINPRNSBS-1;

        if ((sat=satno(sys,prn))) {
//            tt=timediff(rtcm->obs.data[0].time,rtcm->time);
//            if (rtcm->obsflag||fabs(tt)>1E-9) {
//                rtcm->obs.n=rtcm->obsflag=0;
//            }
//            index=obsindex(&rtcm->obs,rtcm->time,sat);
            index=obsindex(&rtcm->obs,sat);
        }
        else {
            trace(2,"rtcm3 %d satellite error: prn=%d\n",type,prn);
        }
        fcn=0;
        if (sys==SYS_GLO) {
            fcn=-8; /* no glonass fcn info */
            fcn = glo_fcn[prn-1]; //change
//            if (ex&&ex[i]<=13) {
//                fcn=ex[i]-7;
//                if (!rtcm->nav.glo_fcn[prn-1]) {
//                    rtcm->nav.glo_fcn[prn-1]=fcn+8; /* fcn+8 */
//                }
//            }
//            else if (rtcm->nav.geph[prn-1].sat==sat) {
//                fcn=rtcm->nav.geph[prn-1].frq;
//            }
//            else if (rtcm->nav.glo_fcn[prn-1]>0) {
//                fcn=rtcm->nav.glo_fcn[prn-1]-8;
//            }
        }
        for (k=0;k<h->nsig;k++) {
            if (!h->cellmask[k+i*h->nsig]) continue;

            if (sat&&index>=0&&idx[k]>=0) {
                freq=fcn<-7?0.0:code2freq(sys,code[k],fcn);

                temp = idx[k];
                /* pseudorange (m) */
                if (r[i]!=0.0&&pr[j]>-1E12) {
                    rtcm->obs.data[index].P[idx[k]]=r[i]+pr[j];
                }
                /* carrier-phase (cycle) */
                if (r[i]!=0.0&&cp[j]>-1E12) {
                    rtcm->obs.data[index].L[idx[k]]=(r[i]+cp[j])*freq/CLIGHT;
                }
                /* doppler (hz) */
                if (rr&&rrf&&rrf[j]>-1E12) {
                    rtcm->obs.data[index].D[idx[k]]=
                        (float)(-(rr[i]+rrf[j])*freq/CLIGHT);
                }
                rtcm->obs.data[index].LLI[idx[k]]=
                    lossoflock(rtcm,sat,idx[k],lock[j])+(half[j]?3:0);
                rtcm->obs.data[index].SNR [idx[k]]=(uint32_t)(cnr[j]/SNR_UNIT+0.5);
                rtcm->obs.data[index].code[idx[k]]=code[k];
                rtcm->obs.data[index].locktime[idx[k]]=lock[j];
            }
            j++;
        }
    }
}


/* decode MSM 4: full pseudorange and phaserange plus CNR --------------------*/
static int decode_msm4(rtcm_con *rtcm, int sys)
{
    msm_h_con h={0};
    double r[64],pr[64],cp[64],cnr[64];
    int i,j,type,sync,iod,ncell,rng,rng_m,prv,cpv,lock[64],half[64];

    type=getbitu(rtcm->buff,24,12);

//    /* decode msm header */
    if ((ncell=decode_msm_head(rtcm,sys,&sync,&iod,&h,&i))<0) return -1;

    if (i+h.nsat*18+ncell*48>rtcm->len*8) {
        trace(1,"rtcm3 %d length error: nsat=%d ncell=%d len=%d\n",type,h.nsat,
              ncell,rtcm->len);
        return -1;
    }
    for (j=0;j<h.nsat;j++) r[j]=0.0;
    for (j=0;j<ncell;j++) pr[j]=cp[j]=-1E16;

    /* decode satellite data */
    for (j=0;j<h.nsat;j++) { /* range */
        rng  =getbitu(rtcm->buff,i, 8); i+= 8;
        if (rng!=255) r[j]=rng*RANGE_MS;
    }
    for (j=0;j<h.nsat;j++) {
        rng_m=getbitu(rtcm->buff,i,10); i+=10;
        if (r[j]!=0.0) r[j]+=rng_m*P2_10*RANGE_MS;
    }
    /* decode signal data */
    for (j=0;j<ncell;j++) { /* pseudorange */
        prv=getbits(rtcm->buff,i,15); i+=15;
        if (prv!=-16384) pr[j]=prv*P2_24*RANGE_MS;
    }
    for (j=0;j<ncell;j++) { /* phaserange */
        cpv=getbits(rtcm->buff,i,22); i+=22;
        if (cpv!=-2097152) cp[j]=cpv*P2_29*RANGE_MS;
    }
    for (j=0;j<ncell;j++) { /* lock time */
        lock[j]=getbitu(rtcm->buff,i,4); i+=4;
    }
    for (j=0;j<ncell;j++) { /* half-cycle ambiguity */
        half[j]=getbitu(rtcm->buff,i,1); i+=1;
    }
    for (j=0;j<ncell;j++) { /* cnr */
        cnr[j]=getbitu(rtcm->buff,i,6)*1.0; i+=6;
    }
//    /* save obs data in msm message */
    save_msm_obs(rtcm,sys,&h,r,pr,cp,NULL,NULL,cnr,lock,NULL,half);

//    rtcm->obsflag=!sync;
//    return sync?0:1;
    return 1;
}





static void free_rtcm(rtcm_con *rtcm){
    if(rtcm->obs.data!=NULL){
        trace(2,"free rtcm obs\n");
        free(rtcm->obs.data); rtcm->obs.data=NULL; rtcm->obs.n=0;
    }
}
static int init_rtcm(rtcm_con *rtcm){
    int i;
    obsd_con data0={{0}};
    rtcm->len=0;//rtcm->nbyte=rtcm->nbit=rtcm->len=0;
    rtcm->lensd=0;
    rtcm->obs.data=NULL;
    if (!(rtcm->obs.data=(obsd_con *)malloc(sizeof(obsd_con)*MAXOBS))) {
        trace(1,"init_rtcm: obs or nav malloc fail\n");
        free_rtcm(rtcm);
        return 0;
    }
    rtcm->obs.n=0;
    rtcm->nbit=0;
    memset(rtcm->buffsd,0,1200*sizeof(uint8_t));
//    int lenobs=malloc_usable_size(rtcm->obs.data);
//    int conlen=sizeof(obsd_con);
//    trace(1,"obs len: %d\n",lenobs);


    for (i=0;i<MAXOBS   ;i++) rtcm->obs.data[i]=data0;
    return 1;
}


/* generate MSM satellite, signal and cell index -----------------------------*/
static void gen_msm_index(rtcm_con *rtcm, int sys, int *nsat, int *nsig,
                          int *ncell, uint8_t *sat_ind, uint8_t *sig_ind,
                          uint8_t *cell_ind)
{
    int i,j,sat,sig,cell,sys_idx;

    *nsat=*nsig=*ncell=0;

    /* generate satellite and signal index */
    for (i=0;i<rtcm->obs.n;i++) {
        if (!(sat=to_satid(sys,rtcm->obs.data[i].sat))) continue;
        sys_idx = systbl(sys);

        for (j=0;j<m_gnss_frq_num[sys_idx];j++) {  //change
//        for (j=0;j<NFREQ+NEXOBS;j++) {
            if (!(sig=to_sigid(sys,rtcm->obs.data[i].code[j]))) continue;

            sat_ind[sat-1]=sig_ind[sig-1]=1;
        }
    }
    for (i=0;i<64;i++) {
        if (sat_ind[i]) sat_ind[i]=++(*nsat);
    }
    for (i=0;i<32;i++) {
        if (sig_ind[i]) sig_ind[i]=++(*nsig);
    }
    /* generate cell index */
    for (i=0;i<rtcm->obs.n;i++) {
        if (!(sat=to_satid(sys,rtcm->obs.data[i].sat))) continue;

        for (j=0;j<m_gnss_frq_num[sys_idx];j++) {// change
//        for (j=0;j<NFREQ+NEXOBS;j++) {
            if (!(sig=to_sigid(sys,rtcm->obs.data[i].code[j]))) continue;

            cell=sig_ind[sig-1]-1+(sat_ind[sat-1]-1)*(*nsig);
            cell_ind[cell]=1;
        }
    }
    for (i=0;i<*nsat*(*nsig);i++) {
        if (cell_ind[i]&&*ncell<64) cell_ind[i]=++(*ncell);
    }
}

/* GLONASS frequency channel number in RTCM (FCN+7,-1:error) -----------------*/
static int fcn_glo(int sat, rtcm_con *rtcm)
{
    int prn;

    if (satsys(sat,&prn)!=SYS_GLO) {
        return -1;
    }

    if (glo_fcn[prn-1]>-8) { /* (-8: no data) */
        return glo_fcn[prn-1]+7;
    }
    return -1;
}

/* generate MSM satellite data fields ----------------------------------------*/
static void gen_msm_sat(rtcm_con *rtcm, int sys, int nsat, const uint8_t *sat_ind,
                        double *rrng, double *rrate, uint8_t *info)
{
    obsd_con *data;
    double freq;
    int i,j,k,sat,sig,fcn,sys_idx;

    for (i=0;i<64;i++) rrng[i]=rrate[i]=0.0;

    for (i=0;i<rtcm->obs.n;i++) {
        data=rtcm->obs.data+i;
        fcn=fcn_glo(data->sat,rtcm); /* fcn+7 */
        sys_idx = systbl(sys);

        if (!(sat=to_satid(sys,data->sat))) continue;

        for (j=0;j<m_gnss_frq_num[sys_idx];j++) {  //change
//        for (j=0;j<NFREQ+NEXOBS;j++) {
            if (!(sig=to_sigid(sys,data->code[j]))) continue;
            k=sat_ind[sat-1]-1;
            freq=code2freq(sys,data->code[j],fcn-7);

            /* rough range (ms) and rough phase-range-rate (m/s) */
            if (rrng[k]==0.0&&data->P[j]!=0.0) {
                rrng[k]=ROUND( data->P[j]/RANGE_MS/P2_10)*RANGE_MS*P2_10;
            }
            if (rrate[k]==0.0&&data->D[j]!=0.0&&freq>0.0) {
                rrate[k]=ROUND(-data->D[j]*CLIGHT/freq)*1.0;
            }
            /* extended satellite info */
            if (info) info[k]=sys!=SYS_GLO?0:(fcn<0?15:fcn);
        }
    }
}

/* generate MSM signal data fields -------------------------------------------*/
static void gen_msm_sig(rtcm_con *rtcm, int sys, int nsat, int nsig, int ncell,
                        const uint8_t *sat_ind, const uint8_t *sig_ind,
                        const uint8_t *cell_ind, const double *rrng,
                        const double *rrate, double *psrng, double *phrng,
                        double *rate, double *lock, uint8_t *half, float *cnr)
{
    obsd_con *data;
    double freq,lambda,psrng_s,phrng_s,rate_s,lt;
    int i,j,k,sat,sig,fcn,cell,LLI,sys_idx;

    for (i=0;i<ncell;i++) {
        if (psrng) psrng[i]=0.0;
        if (phrng) phrng[i]=0.0;
        if (rate ) rate [i]=0.0;
    }
    for (i=0;i<rtcm->obs.n;i++) {
        data=rtcm->obs.data+i;
        fcn=fcn_glo(data->sat,rtcm); /* fcn+7 */
        sys_idx = systbl(sys);

        if (!(sat=to_satid(sys,data->sat))) continue;

        for (j=0;j<m_gnss_frq_num[sys_idx];j++) {
            if (!(sig=to_sigid(sys,data->code[j]))) continue;

            k=sat_ind[sat-1]-1;
            if ((cell=cell_ind[sig_ind[sig-1]-1+k*nsig])>=64) continue;

            freq=code2freq(sys,data->code[j],fcn-7);
            lambda=freq==0.0?0.0:CLIGHT/freq;
            psrng_s=data->P[j]==0.0?0.0:data->P[j]-rrng[k];
            phrng_s=data->L[j]==0.0||lambda<=0.0?0.0: data->L[j]*lambda-rrng [k];
            rate_s =data->D[j]==0.0||lambda<=0.0?0.0:-data->D[j]*lambda-rrate[k];

            /* subtract phase - psudorange integer cycle offset */
            LLI=data->LLI[j];
            if ((LLI&1)) {
//            if ((LLI&1)||fabs(phrng_s-rtcm->cp[data->sat-1][j])>1171.0) { //change
//                rtcm->cp[data->sat-1][j]=ROUND(phrng_s/lambda)*lambda;
                LLI|=1;
            }
//            phrng_s-=rtcm->cp[data->sat-1][j];

//            lt=locktime_d(data->time,rtcm->lltime[data->sat-1]+j,LLI);

            if (psrng&&psrng_s!=0.0) psrng[cell-1]=psrng_s;
            if (phrng&&phrng_s!=0.0) phrng[cell-1]=phrng_s;
            if (rate &&rate_s !=0.0) rate [cell-1]=rate_s;
            /*if (lock) lock[cell-1]=lt;*/// change ZRZ
            if (lock) lock[cell-1]=data->locktime[j];
            if (half) half[cell-1]=(data->LLI[j]&2)?1:0;
            if (cnr ) cnr [cell-1]=(float)(data->SNR[j]*SNR_UNIT);
        }
    }
}

/* encode MSM header ---------------------------------------------------------*/
static int encode_msm_head(int type, rtcm_con *rtcm, int sys, int sync, int *nsat,
                           int *ncell, double *rrng, double *rrate,
                           uint8_t *info, double *psrng, double *phrng,
                           double *rate, double *lock, uint8_t *half,
                           float *cnr)
{
    double tow;
    uint8_t sat_ind[64]={0},sig_ind[32]={0},cell_ind[32*64]={0};
    uint32_t dow,epoch;
    int i=24,j,nsig=0;

    switch (sys) {
        case SYS_GPS: type+=1070; break;
        case SYS_GLO: type+=1080; break;
        case SYS_GAL: type+=1090; break;
        case SYS_QZS: type+=1110; break;
        case SYS_SBS: type+=1100; break;
        case SYS_CMP: type+=1120; break;
        case SYS_IRN: type+=1130; break;
        default: return 0;
    }
    /* generate msm satellite, signal and cell index */
    gen_msm_index(rtcm,sys,nsat,&nsig,ncell,sat_ind,sig_ind,cell_ind);

//    if (sys==SYS_GLO) {
//        /* GLONASS time (dow + tod-ms) */
//        tow=time2gpst(timeadd(gpst2utc(rtcm->time),10800.0),NULL);
//        dow=(uint32_t)(tow/86400.0);
//        epoch=(dow<<27)+ROUND_U(fmod(tow,86400.0)*1E3);
//    }
//    else if (sys==SYS_CMP) {
//        /* BDS time (tow-ms) */
//        epoch=ROUND_U(time2gpst(gpst2bdt(rtcm->time),NULL)*1E3);
//    }
//    else {
//        /* GPS, QZSS, Galileo and IRNSS time (tow-ms) */
//        epoch=ROUND_U(time2gpst(rtcm->time,NULL)*1E3);
//    }
    /* encode msm header (ref [15] table 3.5-78) */
//    setbitu(rtcm->buff,i,12,type       ); i+=12; /* message number */
//    setbitu(rtcm->buff,i,12,rtcm->staid); i+=12; /* reference station id */
//    setbitu(rtcm->buff,i,30,epoch      ); i+=30; /* epoch time */
	i += 12;
	i += 12;
	i += 30;
    setbitu(rtcm->buff,i, 1,sync       ); /* multiple message bit */
//    setbitu(rtcm->buff,i, 3,rtcm->seqno); i+= 3; /* issue of data station */
//    setbitu(rtcm->buff,i, 7,0          ); i+= 7; /* reserved */
//    setbitu(rtcm->buff,i, 2,0          ); i+= 2; /* clock streering indicator */
//    setbitu(rtcm->buff,i, 2,0          ); i+= 2; /* external clock indicator */
//    setbitu(rtcm->buff,i, 1,0          ); i+= 1; /* smoothing indicator */
//    setbitu(rtcm->buff,i, 3,0          ); i+= 3; /* smoothing interval */

    
    i+= 1;
    i+= 3;
    i+= 7;
    i+= 2;
    i+= 2;
    i+= 1;
    i+= 3;
    memcpy(rtcm->buffsd,rtcm->buff,13*sizeof(uint8_t));  // 24+12+12+30+1+3+7+2+2+1+3 =97

    /* satellite mask */
    for (j=0;j<64;j++) {
        setbitu(rtcm->buffsd,i,1,sat_ind[j]?1:0); i+=1;
    }
    /* signal mask */
    for (j=0;j<32;j++) {
        setbitu(rtcm->buffsd,i,1,sig_ind[j]?1:0); i+=1;
    }
    /* cell mask */
    for (j=0;j<*nsat*nsig&&j<64;j++) {
        setbitu(rtcm->buffsd,i,1,cell_ind[j]?1:0); i+=1;
    }
    /* generate msm satellite data fields */
    gen_msm_sat(rtcm,sys,*nsat,sat_ind,rrng,rrate,info);

    /* generate msm signal data fields */
    gen_msm_sig(rtcm,sys,*nsat,nsig,*ncell,sat_ind,sig_ind,cell_ind,rrng,rrate,
                psrng,phrng,rate,lock,half,cnr);

    return i;
}

/* encode rough range integer ms ---------------------------------------------*/
static int encode_msm_int_rrng(rtcm_con *rtcm, int i, const double *rrng,
                               int nsat)
{
    uint32_t int_ms;
    int j;

    for (j=0;j<nsat;j++) {
        if (rrng[j]==0.0) {
            int_ms=255;
        }
        else if (rrng[j]<0.0||rrng[j]>RANGE_MS*255.0) {
//            trace(2,"msm rough range overflow %s rrng=%.3f\n",
//                 time_str(rtcm->time,0),rrng[j]);
            int_ms=255;
        }
        else {
            int_ms=ROUND_U(rrng[j]/RANGE_MS/P2_10)>>10;
        }
        setbitu(rtcm->buffsd,i,8,int_ms); i+=8;
    }
    return i;
}

/* encode rough range modulo 1 ms --------------------------------------------*/
static int encode_msm_mod_rrng(rtcm_con *rtcm, int i, const double *rrng,
                               int nsat)
{
    uint32_t mod_ms;
    int j;

    for (j=0;j<nsat;j++) {
        if (rrng[j]<=0.0||rrng[j]>RANGE_MS*255.0) {
            mod_ms=0;
        }
        else {
            mod_ms=ROUND_U(rrng[j]/RANGE_MS/P2_10)&0x3FFu;
        }
        setbitu(rtcm->buffsd,i,10,mod_ms); i+=10;
    }
    return i;
}
/* encode fine pseudorange ---------------------------------------------------*/
static int encode_msm_psrng(rtcm_con *rtcm, int i, const double *psrng, int ncell)
{
    int j,psrng_val;

    for (j=0;j<ncell;j++) {
        if (psrng[j]==0.0) {
            psrng_val=-16384;
        }
        else if (fabs(psrng[j])>292.7) {
//            trace(2,"msm fine pseudorange overflow %s psrng=%.3f\n",
//                 time_str(rtcm->time,0),psrng[j]);
            psrng_val=-16384;
        }
        else {
            psrng_val=ROUND(psrng[j]/RANGE_MS/P2_24);
        }
        setbits(rtcm->buffsd,i,15,psrng_val); i+=15;
    }
    return i;
}
/* encode fine phase-range ---------------------------------------------------*/
static int encode_msm_phrng(rtcm_con *rtcm, int i, const double *phrng, int ncell)
{
    int j,phrng_val;

    for (j=0;j<ncell;j++) {
        if (phrng[j]==0.0) {
            phrng_val=-2097152;
        }
        else if (fabs(phrng[j])>1171.0) {
//            trace(2,"msm fine phase-range overflow %s phrng=%.3f\n",
//                 time_str(rtcm->time,0),phrng[j]);
            phrng_val=-2097152;
        }
        else {
            phrng_val=ROUND(phrng[j]/RANGE_MS/P2_29);
        }
        setbits(rtcm->buffsd,i,22,phrng_val); i+=22;
    }
    return i;
}

/* MSM lock time indicator (ref [17] table 3.5-74) ---------------------------*/
static int to_msm_lock(double lock)
{
    if (lock<0.032  ) return 0;
    if (lock<0.064  ) return 1;
    if (lock<0.128  ) return 2;
    if (lock<0.256  ) return 3;
    if (lock<0.512  ) return 4;
    if (lock<1.024  ) return 5;
    if (lock<2.048  ) return 6;
    if (lock<4.096  ) return 7;
    if (lock<8.192  ) return 8;
    if (lock<16.384 ) return 9;
    if (lock<32.768 ) return 10;
    if (lock<65.536 ) return 11;
    if (lock<131.072) return 12;
    if (lock<262.144) return 13;
    if (lock<524.288) return 14;
    return 15;
}

/* encode lock-time indicator ------------------------------------------------*/
static int encode_msm_lock(rtcm_con *rtcm, int i, const double *lock, int ncell)
{
    int j,lock_val;

    for (j=0;j<ncell;j++) {
//        lock_val=to_msm_lock(lock[j]);//change ZRZ
        lock_val=lock[j];
        setbitu(rtcm->buffsd,i,4,lock_val); i+=4;
    }
    return i;
}

/* encode half-cycle-ambiguity indicator -------------------------------------*/
static int encode_msm_half_amb(rtcm_con *rtcm, int i, const uint8_t *half,
                               int ncell)
{
    int j;

    for (j=0;j<ncell;j++) {
        setbitu(rtcm->buffsd,i,1,half[j]); i+=1;
    }
    return i;
}

/* encode signal CNR ---------------------------------------------------------*/
static int encode_msm_cnr(rtcm_con *rtcm, int i, const float *cnr, int ncell)
{
    int j,cnr_val;

    for (j=0;j<ncell;j++) {
        cnr_val=ROUND(cnr[j]/1.0);
        setbitu(rtcm->buffsd,i,6,cnr_val); i+=6;
    }
    return i;
}

/* encode MSM 4: full pseudorange and phaserange plus CNR --------------------*/
static int encode_msm4(rtcm_con *rtcm, int sys, int sync)
{
    double rrng[64],rrate[64],psrng[64],phrng[64],lock[64];
    float cnr[64];
    uint8_t half[64];
    int i,nsat,ncell;

    trace(3,"encode_msm4: sys=%d sync=%d\n",sys,sync);

    /* encode msm header */
    if (!(i=encode_msm_head(4,rtcm,sys,sync,&nsat,&ncell,rrng,rrate,NULL,psrng,
                            phrng,NULL,lock,half,cnr))) {
        return 0;
    }
    /* encode msm satellite data */
    i=encode_msm_int_rrng(rtcm,i,rrng ,nsat ); /* rough range integer ms */
    i=encode_msm_mod_rrng(rtcm,i,rrng ,nsat ); /* rough range modulo 1 ms */

    /* encode msm signal data */
    i=encode_msm_psrng   (rtcm,i,psrng,ncell); /* fine pseudorange */
    i=encode_msm_phrng   (rtcm,i,phrng,ncell); /* fine phase-range */
    i=encode_msm_lock    (rtcm,i,lock ,ncell); /* lock-time indicator */
    i=encode_msm_half_amb(rtcm,i,half ,ncell); /* half-cycle-amb indicator */
    i=encode_msm_cnr     (rtcm,i,cnr  ,ncell); /* signal cnr */
    rtcm->nbit=i;
    return 1;
}

static int encode_rtcm3(rtcm_con *rtcm, int type, int sync){
    int ret=0;

    trace(3,"encode_rtcm3: type=%d subtype=%d sync=%d\n",type,sync);

    switch (type) {
        case 1074: ret=encode_msm4(rtcm,SYS_GPS,sync); break;
        case 1084: ret=encode_msm4(rtcm,SYS_GLO,sync); break;
        case 1094: ret=encode_msm4(rtcm,SYS_GAL,sync); break;
        case 1104: ret=encode_msm4(rtcm,SYS_SBS,sync); break;
        case 1114: ret=encode_msm4(rtcm,SYS_QZS,sync); break;
        case 1124: ret=encode_msm4(rtcm,SYS_CMP,sync); break;
        case 1134: ret=encode_msm4(rtcm,SYS_IRN,sync); break;
    }
}

static int decode_rtcm3(rtcm_con *rtcm)
{
	//static resnum = 0;
    double tow;
    int ret=-1,type=getbitu(rtcm->buff,24,12),week;

//    trace(3,"decode_rtcm3: len=%3d type=%d\n",rtcm->len,type);

//    if (rtcm->outtype) {
//        sprintf(rtcm->msgtype,"RTCM %4d (%4d):",type,rtcm->len);
//    }
    /* real-time input option */
//    if (strstr(rtcm->opt,"-RT_INP")) {
//        tow=time2gpst(utc2gpst(timeget()),&week);
//        rtcm->time=gpst2time(week,floor(tow));
//    }
//    trace(3,"rtcm type:%d\n",type);
	//trace(4, "resnmum: %d\n", resnum);
    trace(2,"type : %d\n",type);
	/*if (resnum > 86400 * 100) {
		trace(1, "processure time test over!!!");
		return -1;
	}*/
    switch (type) {
		case 1074: ret = decode_msm4(rtcm, SYS_GPS); break;
        case 1084: ret=decode_msm4(rtcm,SYS_GLO); break;
        case 1094: ret=decode_msm4(rtcm,SYS_GAL); break;
        case 1104: ret=decode_msm4(rtcm,SYS_SBS); break;
        case 1114: ret=decode_msm4(rtcm,SYS_QZS); break;
        case 1124: ret=decode_msm4(rtcm,SYS_CMP); break;
        case 1134: ret=decode_msm4(rtcm,SYS_IRN); break;

        default :  trace(1,"unsupposed type : %d\n",type); break;
    }
	
//    if (ret>=0) {
//        if      (1001<=type&&type<=1299) rtcm->nmsg3[type-1000]++; /*   1-299 */
//        else if (4070<=type&&type<=4099) rtcm->nmsg3[type-3770]++; /* 300-329 */
//        else rtcm->nmsg3[0]++; /* other */
//    }

    return ret;
}

/* generate RTCM 3 message -----------------------------------------------------
* generate RTCM 3 message
* args   : rtcm_t *rtcm     IO  rtcm control struct
*          int    type      I   message type
*          int    subtype   I   message subtype
*          int    sync      I   sync flag (1:another message follows)
* return : status (1:ok,0:error)
* notes  : For rtcm 3 msm, the {nsat} x {nsig} in rtcm->obs should not exceed
*          64. If {nsat} x {nsig} of the input obs data exceeds 64, separate
*          them to multiple ones and call gen_rtcm3() multiple times as user
*          responsibility.
*          ({nsat} = number of valid satellites, {nsig} = number of signals in
*          the obs data)
*-----------------------------------------------------------------------------*/
static int gen_rtcm3(rtcm_con *rtcm, int type, int sync)
{
    uint32_t crc;
    int i=0;

    trace(4,"gen_rtcm3: type=%d sync=%d\n",type,sync);

//    rtcm->nbit=rtcm->len=rtcm->nbyte=0;
    rtcm->nbit=0;
	rtcm->lensd = 0;

    /* set preamble and reserved */
    setbitu(rtcm->buffsd,i, 8,RTCM3PREAMB); i+= 8;
    setbitu(rtcm->buffsd,i, 6,0          ); i+= 6;
    setbitu(rtcm->buffsd,i,10,0          ); i+=10;

    /* encode rtcm 3 message body */
    if (!encode_rtcm3(rtcm,type,sync)) return 0;

    /* padding to align 8 bit boundary */
    for (i=rtcm->nbit;i%8;i++) {
        setbitu(rtcm->buffsd,i,1,0);
    }
    /* message length (header+data) (bytes) */
    if ((rtcm->lensd=i/8)>=3+1024) {
        trace(2,"generate rtcm 3 message length error len=%d\n",rtcm->len-3);
        rtcm->nbit=rtcm->lensd=0;
        return 0;
    }
    /* message length without header and parity */
    setbitu(rtcm->buffsd,14,10,rtcm->lensd-3);

    /* crc-24q */
    crc=rtk_crc24q(rtcm->buffsd,rtcm->lensd);
    setbitu(rtcm->buffsd,i,24,crc);

    /* length total (bytes) */
//    rtcm->nbyte=rtcm->len+3;
//    rtcm->nbyte=rtcm->len+3;

    return 1;
}

/* convert RTCM 3 message -----------------------------------------------------
* generate RTCM 3 message
* args   : uint8_t *buff_in I   rtcm binary data
*          int    len       I   length of received rtcm data
*          char  **freq_c   I   sent frequency
*          uint8_t *buff_sd o   converted rtcm data (need to be sent)
*          int    *len_sd   o   results length
* return : status (1:ok,0,-1:error or no rtcm data)
*
* freq_c selection :
*
* "L1", "L2", "L5",                            GPS
* "G1", "G2", "G3", "G1a","G2a",               GLO
* "E1", "E5b","E5a","E6", "E5ab",              GAL
* "L1", "L2", "L5", "L6",                      QZS
* "L1", "L5",                                  SBS
* "B1I","B3I","B2a","B1C","B2ab","B2I", "B2b", BDS
* "L5", "S",                                   IRN
*
* freq_c format: char *freq_c[7]={
*                    "L1",                     //GPS
*                    "",                       //GLONASS
*                    "E1+E5b",                 //Galileo
*                    "L1+L2",                  //QZSS
*                    "L1+L5",                  //SBAS
*                    "B1I+B2I+B3I",            //BDS
*                    "L5+S"                    //IRN
*                   };
* note: Taking the above as an example, the rtcm data will be encoded:
* GPS L1 data will be obtained, while the corresponding L2 and L3 are discarded.
* GLONASS no data will be sent;
* BDS B1I, B2I, B3I will be obtained in new rtcm buff.
* "G3", "G1a","G2a" in rtcm ICD are not provided. therefore, please do not choose the three.
*-----------------------------------------------------------------------------*/


API_DECLSPEC int rtcmCvt(int sync,unsigned char *buff_in,int len,char **freq_c,unsigned char *buff_sd,int *len_sd){
    int ret,stringlen,type;
//    double len_dbe;
    /*char _freq_def[7][20]={
        "L1+L2",
        "G1+G2",
        "E1+E5b",
        "L1+L2",
        "L1+L5",
        "B1I+B3I",
        "L5+S"
    }; */// default frequencies
    char _freq_sel[40],_freq_ext[40];

    rtcm_con rtcm_in;

    for (int i=0;i<7;i++){
        memcpy(_freq_ext,freq_c[i],40*sizeof(char));
        stringlen = strlen(_freq_ext);
        if(strlen(_freq_ext)==0){
//            memcpy(_freq_sel,_freq_def[i],20*sizeof(char)); //if default, please uncomment the line, comment the next line.
            _freq_sel[0]='\0';
        }
        else {
            memcpy(_freq_sel,_freq_ext,40*sizeof(char));
        }

        setfrqpri(_freq_sel,i);
    }

    init_rtcm(&rtcm_in);
    memcpy(rtcm_in.buff,buff_in,len*sizeof(uint8_t));
    rtcm_in.len=len;
	type = getbitu(rtcm_in.buff, 24, 12);
	trace(2, "type : %d\n", type);


    ret = decode_rtcm3(&rtcm_in);

    //type = getbitu(rtcm_in.buff,24,12);
    if (ret<0){

        trace(1,"type error: %d\n",type);
    }
    else {
        ret= gen_rtcm3(&rtcm_in, type, sync);

		if (ret>0) {
			*len_sd = rtcm_in.lensd + 3;
			memcpy(buff_sd, rtcm_in.buffsd, *len_sd * sizeof(uint8_t));
		}
		else {
			*len_sd = 0;
		}
        
    }

    free_rtcm(&rtcm_in);
    return ret;

}








