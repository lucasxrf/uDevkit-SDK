/**
 * @file i2c_pic32.h
 * @author Sebastien CAUX (sebcaux)
 * @copyright Robotips 2016-2017
 *
 * @date October 06, 2016, 11:15 PM
 *
 * @brief I2C support for rtprog for PIC32MM, PIC32MK, PIC32MX,
 * PIC32MZDA, PIC32MZEC and PIC32MZEF
 */

#ifndef I2C_PIC32_H
#define I2C_PIC32_H

#if defined(DEVICE_32MX110F016B) || defined(DEVICE_32MX110F016C) || defined(DEVICE_32MX110F016D) \
 || defined(DEVICE_32MX120F032B) || defined(DEVICE_32MX120F032C) || defined(DEVICE_32MX120F032D) \
 || defined(DEVICE_32MX120F064H) || defined(DEVICE_32MX130F064B) || defined(DEVICE_32MX130F064C) \
 || defined(DEVICE_32MX130F064D) || defined(DEVICE_32MX130F128H) || defined(DEVICE_32MX130F128L) \
 || defined(DEVICE_32MX130F256B) || defined(DEVICE_32MX130F256D) || defined(DEVICE_32MX150F128B) \
 || defined(DEVICE_32MX150F128C) || defined(DEVICE_32MX150F128D) || defined(DEVICE_32MX150F256H) \
 || defined(DEVICE_32MX150F256L) || defined(DEVICE_32MX170F256B) || defined(DEVICE_32MX170F256D) \
 || defined(DEVICE_32MX170F512H) || defined(DEVICE_32MX170F512L) || defined(DEVICE_32MX210F016B) \
 || defined(DEVICE_32MX210F016C) || defined(DEVICE_32MX210F016D) || defined(DEVICE_32MX220F032B) \
 || defined(DEVICE_32MX220F032C) || defined(DEVICE_32MX220F032D) || defined(DEVICE_32MX230F064B) \
 || defined(DEVICE_32MX230F064C) || defined(DEVICE_32MX230F064D) || defined(DEVICE_32MX230F128H) \
 || defined(DEVICE_32MX230F128L) || defined(DEVICE_32MX230F256B) || defined(DEVICE_32MX230F256D) \
 || defined(DEVICE_32MX250F128B) || defined(DEVICE_32MX250F128C) || defined(DEVICE_32MX250F128D) \
 || defined(DEVICE_32MX250F256H) || defined(DEVICE_32MX250F256L) || defined(DEVICE_32MX270F256B) \
 || defined(DEVICE_32MX270F256D) || defined(DEVICE_32MX270F256DB) || defined(DEVICE_32MX270F512H) \
 || defined(DEVICE_32MX270F512L) || defined(DEVICE_32MX320F032H) || defined(DEVICE_32MX320F064H) \
 || defined(DEVICE_32MX320F128H) || defined(DEVICE_32MX320F128L) || defined(DEVICE_32MX330F064H) \
 || defined(DEVICE_32MX330F064L) || defined(DEVICE_32MX340F128H) || defined(DEVICE_32MX340F128L) \
 || defined(DEVICE_32MX340F256H) || defined(DEVICE_32MX340F512H) || defined(DEVICE_32MX350F128H) \
 || defined(DEVICE_32MX350F128L) || defined(DEVICE_32MX350F256H) || defined(DEVICE_32MX350F256L) \
 || defined(DEVICE_32MX360F256L) || defined(DEVICE_32MX360F512L) || defined(DEVICE_32MX370F512H) \
 || defined(DEVICE_32MX370F512L) || defined(DEVICE_32MX420F032H) || defined(DEVICE_32MX430F064H) \
 || defined(DEVICE_32MX430F064L) || defined(DEVICE_32MX440F128H) || defined(DEVICE_32MX440F128L) \
 || defined(DEVICE_32MX440F256H) || defined(DEVICE_32MX440F512H) || defined(DEVICE_32MX450F128H) \
 || defined(DEVICE_32MX450F128L) || defined(DEVICE_32MX450F256H) || defined(DEVICE_32MX450F256HB) \
 || defined(DEVICE_32MX450F256L) || defined(DEVICE_32MX460F256L) || defined(DEVICE_32MX460F512L) \
 || defined(DEVICE_32MX470F512H) || defined(DEVICE_32MX470F512L) || defined(DEVICE_32MX470F512LB) \
 || defined(DEVICE_32MX530F128H) || defined(DEVICE_32MX530F128L) || defined(DEVICE_32MX550F256H) \
 || defined(DEVICE_32MX550F256L) || defined(DEVICE_32MX570F512H) || defined(DEVICE_32MX570F512L) \
 || defined(DEVICE_32WK2057GPB132) || defined(DEVICE_32WK2057GPD132)
 #define I2C_COUNT 2
#elif defined(DEVICE_32MM0064GPM028) || defined(DEVICE_32MM0064GPM036) || defined(DEVICE_32MM0064GPM048) \
 || defined(DEVICE_32MM0064GPM064) || defined(DEVICE_32MM0128GPM028) || defined(DEVICE_32MM0128GPM036) \
 || defined(DEVICE_32MM0128GPM048) || defined(DEVICE_32MM0128GPM064) || defined(DEVICE_32MM0256GPM028) \
 || defined(DEVICE_32MM0256GPM036) || defined(DEVICE_32MM0256GPM048) || defined(DEVICE_32MM0256GPM064)
 #define I2C_COUNT 3
#elif defined(DEVICE_32MK1024GPE100) || defined(DEVICE_32MK1024MCF100) || defined(DEVICE_32MX534F064H) \
 || defined(DEVICE_32MX564F064H) || defined(DEVICE_32MX564F128H) || defined(DEVICE_32MX575F256H) \
 || defined(DEVICE_32MX575F512H) || defined(DEVICE_32MX664F064H) || defined(DEVICE_32MX664F128H) \
 || defined(DEVICE_32MX675F256H) || defined(DEVICE_32MX675F512H) || defined(DEVICE_32MX695F512H) \
 || defined(DEVICE_32MX764F128H) || defined(DEVICE_32MX775F256H) || defined(DEVICE_32MX775F512H) \
 || defined(DEVICE_32MX795F512H) || defined(DEVICE_32MZ0512EFE064) || defined(DEVICE_32MZ0512EFF064) \
 || defined(DEVICE_32MZ0512EFK064) || defined(DEVICE_32MZ1024ECG064) || defined(DEVICE_32MZ1024ECH064) \
 || defined(DEVICE_32MZ1024ECM064) || defined(DEVICE_32MZ1024EFE064) || defined(DEVICE_32MZ1024EFF064) \
 || defined(DEVICE_32MZ1024EFG064) || defined(DEVICE_32MZ1024EFH064) || defined(DEVICE_32MZ1024EFK064) \
 || defined(DEVICE_32MZ1024EFM064) || defined(DEVICE_32MZ2048ECG064) || defined(DEVICE_32MZ2048ECH064) \
 || defined(DEVICE_32MZ2048ECM064) || defined(DEVICE_32MZ2048EFG064) || defined(DEVICE_32MZ2048EFH064) \
 || defined(DEVICE_32MZ2048EFM064)
 #define I2C_COUNT 4
#elif defined(DEVICE_32MX534F064L) || defined(DEVICE_32MX564F064L) || defined(DEVICE_32MX564F128L) \
 || defined(DEVICE_32MX575F256L) || defined(DEVICE_32MX575F512L) || defined(DEVICE_32MX664F064L) \
 || defined(DEVICE_32MX664F128L) || defined(DEVICE_32MX675F256L) || defined(DEVICE_32MX675F512L) \
 || defined(DEVICE_32MX695F512L) || defined(DEVICE_32MX764F128L) || defined(DEVICE_32MX775F256L) \
 || defined(DEVICE_32MX775F512L) || defined(DEVICE_32MX795F512L) || defined(DEVICE_32MZ0512EFE100) \
 || defined(DEVICE_32MZ0512EFE124) || defined(DEVICE_32MZ0512EFE144) || defined(DEVICE_32MZ0512EFF100) \
 || defined(DEVICE_32MZ0512EFF124) || defined(DEVICE_32MZ0512EFF144) || defined(DEVICE_32MZ0512EFK100) \
 || defined(DEVICE_32MZ0512EFK124) || defined(DEVICE_32MZ0512EFK144) || defined(DEVICE_32MZ1024ECG100) \
 || defined(DEVICE_32MZ1024ECG124) || defined(DEVICE_32MZ1024ECG144) || defined(DEVICE_32MZ1024ECH100) \
 || defined(DEVICE_32MZ1024ECH124) || defined(DEVICE_32MZ1024ECH144) || defined(DEVICE_32MZ1024ECM100) \
 || defined(DEVICE_32MZ1024ECM124) || defined(DEVICE_32MZ1024ECM144) || defined(DEVICE_32MZ1024EFE100) \
 || defined(DEVICE_32MZ1024EFE124) || defined(DEVICE_32MZ1024EFE144) || defined(DEVICE_32MZ1024EFF100) \
 || defined(DEVICE_32MZ1024EFF124) || defined(DEVICE_32MZ1024EFF144) || defined(DEVICE_32MZ1024EFG100) \
 || defined(DEVICE_32MZ1024EFG124) || defined(DEVICE_32MZ1024EFG144) || defined(DEVICE_32MZ1024EFH100) \
 || defined(DEVICE_32MZ1024EFH124) || defined(DEVICE_32MZ1024EFH144) || defined(DEVICE_32MZ1024EFK100) \
 || defined(DEVICE_32MZ1024EFK124) || defined(DEVICE_32MZ1024EFK144) || defined(DEVICE_32MZ1024EFM100) \
 || defined(DEVICE_32MZ1024EFM124) || defined(DEVICE_32MZ1024EFM144) || defined(DEVICE_32MZ1025DAA169) \
 || defined(DEVICE_32MZ1025DAA176) || defined(DEVICE_32MZ1025DAA288) || defined(DEVICE_32MZ1025DAB169) \
 || defined(DEVICE_32MZ1025DAB176) || defined(DEVICE_32MZ1025DAB288) || defined(DEVICE_32MZ1025DAG169) \
 || defined(DEVICE_32MZ1025DAG176) || defined(DEVICE_32MZ1025DAH169) || defined(DEVICE_32MZ1025DAH176) \
 || defined(DEVICE_32MZ1064DAA169) || defined(DEVICE_32MZ1064DAA176) || defined(DEVICE_32MZ1064DAA288) \
 || defined(DEVICE_32MZ1064DAB169) || defined(DEVICE_32MZ1064DAB176) || defined(DEVICE_32MZ1064DAB288) \
 || defined(DEVICE_32MZ1064DAG169) || defined(DEVICE_32MZ1064DAG176) || defined(DEVICE_32MZ1064DAH169) \
 || defined(DEVICE_32MZ1064DAH176) || defined(DEVICE_32MZ2025DAA169) || defined(DEVICE_32MZ2025DAA176) \
 || defined(DEVICE_32MZ2025DAA288) || defined(DEVICE_32MZ2025DAB169) || defined(DEVICE_32MZ2025DAB176) \
 || defined(DEVICE_32MZ2025DAB288) || defined(DEVICE_32MZ2025DAG169) || defined(DEVICE_32MZ2025DAG176) \
 || defined(DEVICE_32MZ2025DAH169) || defined(DEVICE_32MZ2025DAH176) || defined(DEVICE_32MZ2048ECG100) \
 || defined(DEVICE_32MZ2048ECG124) || defined(DEVICE_32MZ2048ECG144) || defined(DEVICE_32MZ2048ECH100) \
 || defined(DEVICE_32MZ2048ECH124) || defined(DEVICE_32MZ2048ECH144) || defined(DEVICE_32MZ2048ECM100) \
 || defined(DEVICE_32MZ2048ECM124) || defined(DEVICE_32MZ2048ECM144) || defined(DEVICE_32MZ2048EFG100) \
 || defined(DEVICE_32MZ2048EFG124) || defined(DEVICE_32MZ2048EFG144) || defined(DEVICE_32MZ2048EFH100) \
 || defined(DEVICE_32MZ2048EFH124) || defined(DEVICE_32MZ2048EFH144) || defined(DEVICE_32MZ2048EFM100) \
 || defined(DEVICE_32MZ2048EFM124) || defined(DEVICE_32MZ2048EFM144) || defined(DEVICE_32MZ2064DAA169) \
 || defined(DEVICE_32MZ2064DAA176) || defined(DEVICE_32MZ2064DAA288) || defined(DEVICE_32MZ2064DAB169) \
 || defined(DEVICE_32MZ2064DAB176) || defined(DEVICE_32MZ2064DAB288) || defined(DEVICE_32MZ2064DAG169) \
 || defined(DEVICE_32MZ2064DAG176) || defined(DEVICE_32MZ2064DAH169) || defined(DEVICE_32MZ2064DAH176) \
 || defined(DEVICE_32PR0000DAD288) || defined(DEVICE_32PR0000DAD288EMU)
 #define I2C_COUNT 5
#else
 #define I2C_COUNT 0
#endif

#if defined(DEVICE_32MZ0512EFE064) || defined(DEVICE_32MZ0512EFF064) || defined(DEVICE_32MZ2048EFM064) \
 || defined(DEVICE_32MZ0512EFK064) || defined(DEVICE_32MZ1024ECG064) || defined(DEVICE_32MZ1024ECH064) \
 || defined(DEVICE_32MZ1024ECM064) || defined(DEVICE_32MZ1024EFE064) || defined(DEVICE_32MZ1024EFF064) \
 || defined(DEVICE_32MZ1024EFG064) || defined(DEVICE_32MZ1024EFH064) || defined(DEVICE_32MZ1024EFK064) \
 || defined(DEVICE_32MZ1024EFM064) || defined(DEVICE_32MZ2048ECG064) || defined(DEVICE_32MZ2048ECH064) \
 || defined(DEVICE_32MZ2048ECM064) || defined(DEVICE_32MZ2048EFG064) || defined(DEVICE_32MZ2048EFH064)
 #define I2C_NOI2C2 // I2C 2 does not exist on 64 pins MZ device, but they have I2C 5 with I2C_COUNT=4 ...

  #if I2C_COUNT>=1
    #define I2C_NUM1 0
  #endif
  // no I2C2
  #if I2C_COUNT>=2
    #define I2C_NUM3 1
  #endif
  #if I2C_COUNT>=3
    #define I2C_NUM4 2
  #endif
  #if I2C_COUNT>=4
    #define I2C_NUM5 3
  #endif

  rt_dev_t i2c(uint8_t d);

#else // in normal case ...
  #if I2C_COUNT>=1
    #define I2C_NUM1 0
  #endif
  #if I2C_COUNT>=2
    #define I2C_NUM2 1
  #endif
  #if I2C_COUNT>=3
    #define I2C_NUM3 2
  #endif
  #if I2C_COUNT>=4
    #define I2C_NUM4 3
  #endif
  #if I2C_COUNT>=5
    #define I2C_NUM5 4
  #endif

  #define i2c(d) MKDEV(DEV_CLASS_I2C, (d)-1);
#endif

#define I2C_FPGD 9615384 // 104ns

#endif // I2C_PIC32_H
