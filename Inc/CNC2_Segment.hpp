#ifndef CNC2_SEGMENT_H
#define CNC2_SEGMENT_H

#include "SignalList.hpp"
#include "GeneralConfig.h"


#include "RngClass.hpp"
#include "TimeClass.hpp"

#include "CNC2_axe.hpp"
#include "Vector.hpp"

#include "CNC2_dataDef.hpp"

#define QUEUE_TYPE 0

class Segment_c
{

  Segment_c* next;

  #if QUEUE_TYPE == 1 
  static QueueHandle_t queue;
  static QueueHandle_t emptyQueue;
  #elif QUEUE_TYPE == 2
  static int first;
  static int last;
  static int firstFree;
  static int lastFree;
  static int length ;
  static int emptyLength;
  uint16_t idx;
   
  #else
  static uint32_t frontGuard;
  static Segment_c* first;
  static Segment_c* last;
  static Segment_c* emptyList;   
  static uint32_t length ;
  static uint32_t emptyLength;
  static uint32_t backGuard;
  #endif

  static void SanityCheck(void);

  public:

  void AddToEmpty(void);
  void AddToEmptyISR(void);
  static Segment_c* GetEmpty(void);

  bool AddToList(void);
  static Segment_c* GetFromList(void);
  static Segment_c* GetFromListISR(void);
  
  static Segment_c* ReadFirstISR(void);

  static void ResetList(void);

  static void InitList(void);

  static int GetLength(void);
  static int GetEmptyLength(void);
  static int GetLengthISR(void);
  static int GetEmptyLengthISR(void);
 
  Segment_c(void)
  {
    next = nullptr;
  }

  /* vector data */
  uint32_t totalLength;
  uint32_t periodCalib;
  uint32_t pulses[AXE_NOOF];
  uint16_t period[AXE_NOOF];
  uint32_t calibTicks[AXE_NOOF];
  uint16_t  prescaler[AXE_NOOF];
  
  
  int16_t value;
  DIR_et dir[AXE_NOOF];
  uint8_t orderCode;
  

  /*speed data */
  float v0;
  float vM;
  float vE;
  float dV;
  float dt;

  int seqNo;

  int lx,ly,lz;

  /* debug data */
  uint32_t endPos[AXE_NOOF];
  int segmentNr;
  
};


#endif
