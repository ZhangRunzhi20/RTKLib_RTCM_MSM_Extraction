# RTKLib_RTCM_MSM_Extraction

Too many GNSS observations will increase the pressure on network transmission. It is inconvenient to directly configure the receiver to obtain the required observations. For example, for a set of GNSS observations at GPS L1/L2 frequencies, you only need the L1 frequency, and it is inconvenient to directly configure the tracking mode of the receiver. A can be used to extract frequency-specific observations from RTCM packets and reassemble them into RTCM MSM without affecting the decoding of RTCM MSM.
## Function interface and parameters
``` C
API_DECLSPEC int rtcmCvt(int sync,unsigned char *buff_in,int len,char **freq_c,unsigned char *buff_sd,int *len_sd);
```
### args:
- **I**    `uint8_t *buff_in`       rtcm binary data
- **I**    `int len`                length of received rtcm data
- **I**    `char  **freq_c`         sent frequency
- **O**    `uint8_t *buff_sd`       converted rtcm data (need to be sent)
- **O**    `int    *len_sd`         results length
- return : status                   (1:ok; 0,-1:error or no rtcm data)
### freq_c selection :

- GPS："L1", "L2", "L5",                           
- GLONASS： "G1", "G2", "G3", "G1a","G2a",               
- Galileo： "E1", "E5b","E5a","E6", "E5ab",             
- QZSS： "L1", "L2", "L5", "L6",                      
- SBAS： "L1", "L5",                                  
- BDS： "B1I","B3I","B2a","B1C","B2ab","B2I", "B2b",
- IRNSS： "L5", "S",                                  
### freq_c format 
``` C 
char *freq_c[7]={
                  "L1",                     //GPS
                  "",                       //GLONASS
                  "E1+E5b",                 //Galileo
                  "L1+L2",                  //QZSS
                  "L1+L5",                  //SBAS
                  "B1I+B2I+B3I",            //BDS
                  "L5+S"                    //IRN
                  };
```

### note: 
Taking the above as an example, the rtcm data will be encoded:
* GPS L1 data will be obtained, while the corresponding L2 and L3 are discarded.
* GLONASS no data will be sent;
* BDS B1I, B2I, B3I will be obtained in new rtcm buff.
* "G3", "G1a","G2a" in RTCM ICD are not provided. therefore, please do not choose the three.
