#pragma once

#ifdef RTCMCNV_EXPORTS
#define API_DECLSPEC _declspec(dllexport)
#else  
#define API_DECLSPEC _declspec(dllimport)
#endif // RTCMCNV_EXPORTS


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
API_DECLSPEC int rtcmCvt(int sync,unsigned char *buff_in,int len,char **freq_c,unsigned char *buff_sd,int *len_sd);

/* RTCM Convert log -----------------------------------------------------
* generate RTCM 3 convert log
* args   : char *file       I   path: generated log file,
*          int  level       I   log level, selection: 1,2,3,4
* Note : 1 (brief) -> 4 (detailed).
* if opened log file, please close file pid when ending
* *-----------------------------------------------------------------------------*/
API_DECLSPEC void rtcmlogopen(const char *file);
API_DECLSPEC void rtcmlogclose(void);
API_DECLSPEC void rtcmloglevel(int level);

