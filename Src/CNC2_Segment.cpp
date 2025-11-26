 #include <stdio.h>
#include <stdlib.h>
#include <cstring>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

#include "math.h"


#include "CNC2_Segment.hpp"

Segment_c segmentList[SEGMENT_LIST_LENGTH];

#if QUEUE_TYPE == 1 
QueueHandle_t Segment_c::queue;
QueueHandle_t Segment_c::emptyQueue;
#elif QUEUE_TYPE == 2
int Segment_c::first = -1;
int Segment_c::last = -1;
int Segment_c::firstFree = -1;
int Segment_c::lastFree = -1;
int Segment_c::length = 0;
int Segment_c::emptyLength = 0;

#else
uint32_t Segment_c::frontGuard = 0xDEADBEEF;
Segment_c* Segment_c::first = nullptr;
Segment_c* Segment_c::last = nullptr;
Segment_c* Segment_c::emptyList = nullptr;

uint32_t Segment_c::length = 0;
uint32_t Segment_c::emptyLength = 0;
uint32_t Segment_c::backGuard = 0xDEADBEEF;
#endif

void Segment_c::SanityCheck(void)
{
/*  uint8_t errorMask = 0;
  if(frontGuard != 0xDEADBEEF)
  {
    errorMask |= 0x01;
  }
  if(backGuard != 0xDEADBEEF)
  {
    errorMask |= 0x02;
  }
 
  Segment_c* seg_p = first;
  int cnt1 = 0;
  int prevNr = -1;
*/
/*

  while(seg_p != nullptr)
  {
    if((prevNr != -1) && (prevNr+1 != seg_p->segmentNr))
    {
      errorMask |= 0x04;
    }
    cnt1++;

    if(seg_p->next == nullptr && seg_p != last)
    {
      errorMask |= 0x08;
    }

    seg_p = seg_p->next;
  }

  if(cnt1 != length)
  {
    errorMask |= 0x10;
  }

  seg_p = emptyList;
  int cnt2 = 0;

  while(seg_p != nullptr)
  {
    cnt2++;
    seg_p = seg_p->next;
  }

  if(cnt2 != emptyLength)
  {
    errorMask |= 0x20;
  }
*/
/*
  if(errorMask != 0)
  {
    printf("sanity fail %x\n",errorMask );

  }*/
  

}

void Segment_c::InitList(void)
{
  #if QUEUE_TYPE == 1
  queue = xQueueCreate( SEGMENT_LIST_LENGTH, sizeof( void* ) );
  emptyQueue = xQueueCreate( SEGMENT_LIST_LENGTH, sizeof( void* ) );
  #elif QUEUE_TYPE == 2
  for(int i=0;i< SEGMENT_LIST_LENGTH; i++)
  {
    segmentList[i].idx = i;
  }

  #endif

}

void Segment_c::ResetList(void)
{
  #if QUEUE_TYPE == 1
  xQueueReset(queue);
  xQueueReset(emptyQueue); 
  #elif QUEUE_TYPE == 2
  first = -1;
  last = -1;
  firstFree = -1;
  lastFree = -1;
  length = 0;
  emptyLength = 0;
  #else
  first = nullptr;
  last = nullptr;
  emptyList = nullptr;
   emptyLength = 0;
  length = 0;
  #endif
  for(int i=0;i< SEGMENT_LIST_LENGTH; i++)
  {
    segmentList[i].AddToEmpty();
  }
  
/*
  int queueSize = uxQueueMessagesWaiting(queue);
  int emptyQueueSize = uxQueueMessagesWaiting(emptyQueue);
  printf("ResetList, Q=%d,EQ=%d\n",queueSize,emptyQueueSize);*/
}


void Segment_c::AddToEmpty(void)
{
  #if QUEUE_TYPE == 1
  Segment_c* segPtr = this;
  xQueueSend(emptyQueue,&segPtr,0);
  #elif QUEUE_TYPE == 2
  taskENTER_CRITICAL();
  /* check range */

  if(emptyLength == 0)
  {
    if(first <= last)
    {
      if(idx>= first && idx <= last)
      {
        printf("Queue error empty 1\n");
      }
    }
    else
    {
      if(idx<= first || idx >= last)
      {
        printf("Queue error empty 2\n");
      }

    }
  }
  else
  {
    int wantedIdx = (lastFree+1) & (SEGMENT_LIST_LENGTH-1);
    if(idx != wantedIdx)
    {
      printf("Queue error empty 3\n");
    }
  }

  if(emptyLength == 0)
  {
    firstFree = idx;
  }
  lastFree = idx;
  emptyLength++;

  taskEXIT_CRITICAL();
  #else
  taskENTER_CRITICAL();
  SanityCheck();
  this->next = emptyList;
  emptyList = this;
  emptyLength++;
  taskEXIT_CRITICAL();
  #endif
}

void Segment_c::AddToEmptyISR(void)
{
  #if QUEUE_TYPE == 1
  Segment_c* segPtr = this;
 /* int queueSize = uxQueueMessagesWaitingFromISR(queue);
  int emptyQueueSize = uxQueueMessagesWaitingFromISR(emptyQueue);
  printf("AddToEmptyISR, Q=%d,EQ=%d\n",queueSize,emptyQueueSize);*/
  BaseType_t xTaskWokenByReceive = pdFALSE;
  xQueueSendFromISR( emptyQueue, &( segPtr ),&xTaskWokenByReceive );
  #elif QUEUE_TYPE == 2
  //taskENTER_CRITICAL();

  /* check range */

  if(emptyLength == 0)
  {
    if(first <= last)
    {
      if(idx>= first && idx <= last)
      {
        printf("Queue error empty ISR 1\n");
      }
    }
    else
    {
      if(idx<= first || idx >= last)
      {
        printf("Queue error empty ISR 2\n");
      }

    }
  }
  else
  {
    int wantedIdx = (lastFree+1) & (SEGMENT_LIST_LENGTH-1);
    if(idx != wantedIdx)
    {
      printf("Queue error empty ISR 3\n");
    }
  }

  if(emptyLength == 0)
  {
    firstFree = idx;
  }
  lastFree = idx;
  emptyLength++;

  //taskEXIT_CRITICAL();
  #else

 SanityCheck();
  this->next = emptyList;
  emptyList = this;
  emptyLength++;
  #endif
}


Segment_c* Segment_c::GetEmpty(void)
{
  Segment_c* retPtr = nullptr;
  #if QUEUE_TYPE == 1

   /* int queueSize = uxQueueMessagesWaiting(queue);
    int emptyQueueSize = uxQueueMessagesWaiting(emptyQueue);
    printf("GetEmpty, Q=%d,EQ=%d\n",queueSize,emptyQueueSize);*/

    
    if( xQueueReceive( emptyQueue, &( retPtr ),0 ) != pdPASS )
    {
      retPtr = nullptr;
    }
  #elif QUEUE_TYPE == 2
  taskENTER_CRITICAL();

  if(emptyLength != 0)
  {

    retPtr = &segmentList[firstFree];
    firstFree++;
    firstFree &= SEGMENT_LIST_LENGTH-1;
    emptyLength--;

    if(emptyLength == 0)
    {
      lastFree = -1;
      firstFree = -1;
    }
  }

  taskEXIT_CRITICAL();
  #else
  taskENTER_CRITICAL();
   SanityCheck();
  retPtr = emptyList;
  if(emptyList != nullptr)
  {
    if(retPtr->next == nullptr)
    {
      emptyList = nullptr;
    }
    else
    {
      emptyList = retPtr->next;
    }
    retPtr->next = nullptr;
  }
  emptyLength--;
  taskEXIT_CRITICAL();
  #endif
  return retPtr;
}

uint32_t lastSegNr;
uint32_t lastSegNrOut;

bool Segment_c::AddToList(void)
{
  #if QUEUE_TYPE == 1

/*      int queueSize2 = uxQueueMessagesWaiting(queue);
    int emptyQueueSize = uxQueueMessagesWaiting(emptyQueue);
    printf("AddToList, Q=%d,EQ=%d\n",queueSize2,emptyQueueSize);*/




  bool needStart = false;
  int queueSize = uxQueueMessagesWaiting(queue);
  if(queueSize == 0)
  {
    needStart = true;
  }
  Segment_c* segPtr = this;

  if(lastSegNr+1 != segPtr->segmentNr)
  {
    printf("Mssing seg at input, prev=%d, act=%d\n", lastSegNr,segPtr->segmentNr);

  }
  lastSegNr = segPtr->segmentNr;

  if(xQueueSend(queue,&segPtr,0)!= pdPASS )
  {
    printf("Failed queueing, sgNr=%d\n",segPtr->segmentNr);

  }

  #elif QUEUE_TYPE == 2
  taskENTER_CRITICAL();

  /* check range */

  if(length == 0)
  {
    if(firstFree <= lastFree)
    {
      if(idx>= firstFree && idx <= lastFree)
      {
        printf("Queue error 1 \n");
      }
    }
    else
    {
      if(idx<= firstFree || idx >= lastFree)
      {
        printf("Queue error 2\n");
      }

    }
  }
  else
  {
    int wantedIdx = (last+1) & (SEGMENT_LIST_LENGTH-1);
    if(idx != wantedIdx)
    {
      printf("Queue error 3\n");
    }
  }

  bool needStart = false;

  if(length == 0)
  {
    first = idx;
    needStart = true;
  }
  last = idx;
  length++;

  taskEXIT_CRITICAL();
  #else
  taskENTER_CRITICAL();
   SanityCheck();
  bool needStart = false;
  this->next = nullptr;
  if(last == nullptr)
  {
    last = this;
    first = this;
    needStart = true;
  }
  else
  {

    if(last->segmentNr+1 != this->segmentNr)
    {
      printf("Mssing seg at input, prev=%d, act=%d\n", lastSegNr,this->segmentNr);
    }
    last->next = this;
    last = this;

  }
  length++;
  taskEXIT_CRITICAL();
  #endif
  return needStart;
}
Segment_c* Segment_c::GetFromList(void)
{  
  Segment_c* retPtr = nullptr;
  #if QUEUE_TYPE == 1
/*
        int queueSize = uxQueueMessagesWaiting(queue);
    int emptyQueueSize = uxQueueMessagesWaiting(emptyQueue);
    printf("GetFromList, Q=%d,EQ=%d\n",queueSize,emptyQueueSize);*/

  
  if( xQueueReceive( queue, &( retPtr ),0 ) != pdPASS )
  {
    retPtr = nullptr;
  }
  else
  {
    if(lastSegNrOut+1 != retPtr->segmentNr)
    {
      printf("Missing segments at output, prev=%d, act=%d\n",lastSegNrOut,retPtr->segmentNr);

    }
    lastSegNrOut= retPtr->segmentNr;
  }


  #elif QUEUE_TYPE == 2
  taskENTER_CRITICAL();

  if(length != 0)
  {

    retPtr = &(segmentList[first]);
    first++;
    first &= SEGMENT_LIST_LENGTH-1;
    length--;

    if(length == 0)
    {
      last = -1;
      first = -1;
    }
  }
  taskEXIT_CRITICAL();

  #else
  taskENTER_CRITICAL();
   SanityCheck();
  retPtr = first;
  if(retPtr != nullptr)
  {
    if(lastSegNrOut+1 != retPtr->segmentNr)
    {
      printf("Missing segments at output, prev=%d, act=%d\n",lastSegNrOut,retPtr->segmentNr);

    }
    lastSegNrOut= retPtr->segmentNr;

    if(retPtr->next == nullptr)
    {
      first = nullptr;
      last = nullptr;
    }
    else
    {
      first = retPtr->next;
    }
    retPtr->next = nullptr;
  }
  length--;
  taskEXIT_CRITICAL();
  #endif
   //printf("get seg %d\n",retPtr->segmentNr);
  return retPtr;
}


Segment_c* Segment_c::GetFromListISR(void)
{  
  Segment_c* retPtr = nullptr;
  #if QUEUE_TYPE == 1

 /*       int queueSize = uxQueueMessagesWaitingFromISR(queue);
    int emptyQueueSize = uxQueueMessagesWaitingFromISR(emptyQueue);
    printf("GetFromListISR, Q=%d,EQ=%d\n",queueSize,emptyQueueSize);*/
  
  BaseType_t xTaskWokenByReceive = pdTRUE;
  if( xQueueReceiveFromISR( queue, &( retPtr ),&xTaskWokenByReceive ) != pdPASS )
  {
    retPtr = nullptr;
  }
  else
  {
    if(lastSegNrOut+1 != retPtr->segmentNr)
    {
      printf("Missing segments at output ISR, prev=%d, act=%d\n",lastSegNrOut,retPtr->segmentNr);

    }
    lastSegNrOut= retPtr->segmentNr;
  }
  #elif QUEUE_TYPE == 2
  //taskENTER_CRITICAL();

  if(length != 0)
  {

    retPtr = &(segmentList[first]);
    first++;
    first &= SEGMENT_LIST_LENGTH-1;
    length--;

    if(length == 0)
    {
      last = -1;
      first = -1;
    }
  }
  //taskEXIT_CRITICAL();
  #else
   SanityCheck();
  retPtr = first;
  if(retPtr != nullptr)
  {
    if(lastSegNrOut+1 != retPtr->segmentNr)
    {
      printf("Missing segments at output, prev=%d, act=%d\n",lastSegNrOut,retPtr->segmentNr);

    }
    lastSegNrOut= retPtr->segmentNr;

    if(retPtr->next == nullptr)
    {
      first = nullptr;
      last = nullptr;
    }
    else
    {
      first = retPtr->next;
    }
    retPtr->next = nullptr;
  }
  length--;
  #endif
  //printf("get seg ISR %d\n",retPtr->segmentNr);
  return retPtr;
}


Segment_c* Segment_c::ReadFirstISR(void)
{
  Segment_c* retPtr = nullptr;
  #if QUEUE_TYPE == 1
 /*       int queueSize = uxQueueMessagesWaitingFromISR(queue);
    int emptyQueueSize = uxQueueMessagesWaitingFromISR(emptyQueue);
    printf("ReadFirstISR, Q=%d,EQ=%d\n",queueSize,emptyQueueSize);*/
  
  if( xQueuePeekFromISR( queue, &( retPtr ) ) != pdPASS )
  {
    retPtr = nullptr;
  }

  #elif QUEUE_TYPE == 2
  //taskENTER_CRITICAL();

  if(length != 0)
  {
    retPtr = &(segmentList[first]);    
  }
  //taskEXIT_CRITICAL();

  #else
   SanityCheck();
  retPtr = first;
  #endif
  return retPtr;
}

int Segment_c::GetLength(void)
{
#if QUEUE_TYPE == 1
  return uxQueueMessagesWaiting(queue);
    #elif QUEUE_TYPE == 2
taskENTER_CRITICAL();
    return length;
taskEXIT_CRITICAL();
    #else
    taskENTER_CRITICAL();
  return length;
  taskEXIT_CRITICAL();
  #endif
}
int Segment_c::GetEmptyLength(void)
{
#if QUEUE_TYPE == 1
  return uxQueueMessagesWaiting(emptyQueue);
      #elif QUEUE_TYPE == 2
taskENTER_CRITICAL();
    return emptyLength;
taskEXIT_CRITICAL();
    #else
    taskENTER_CRITICAL();
  return emptyLength;
  taskEXIT_CRITICAL();
  #endif
}
int Segment_c::GetLengthISR(void)
{
#if QUEUE_TYPE == 1
  return uxQueueMessagesWaitingFromISR(queue);
      #elif QUEUE_TYPE == 2
  //taskENTER_CRITICAL();
    return length;
  //taskEXIT_CRITICAL();
    #else
  return length;
  #endif
}
int Segment_c::GetEmptyLengthISR(void)
{
#if QUEUE_TYPE == 1
  return uxQueueMessagesWaitingFromISR(emptyQueue);
        #elif QUEUE_TYPE == 2
//taskENTER_CRITICAL();
    return emptyLength;
//taskEXIT_CRITICAL();
    #else
  return emptyLength;
  #endif
}