#ifndef CNC2_SURFACE_OFFSET_H
#define CNC2_SURFACE_OFFSET_H

#include "SignalList.hpp"
#include "GeneralConfig.h"
#include "CNC2_dataDef.hpp"

#include "CommandHandler.hpp"

class SurfaceOffset_c
{

  int xSize;
  int ySize;
  int xStep;
  int yStep;
  int xStart;
  int yStart;
  bool active;

  int* array;

  

  public:

  int GetPoint(int x, int y);

  bool IsActive(void) { return active; }
  int GetStepX(void) { return xStep; }
  int GetStepY(void) { return yStep; }
  int GetProbesX(void) { return xSize; }
  int GetProbesY(void) { return ySize; }

  static SurfaceOffset_c* ownRef; /* for printouts only */

  void Clear(void);
  SurfaceOffset_c();
  void Init(int xSize, int ySize, int xStep, int yStep, int xStart, int yStart);
  void SetProbe(int x, int y, int val);
  void Activate(void) ;
  void Deactivate(void);

  int AddSurfaceOffset(int x, int y);

  int GetMaxSegLength(void);

};


class Com_surfoffset : public Command_c
{
  public:
  char* GetComString(void) { return (char*)"surfoffset"; }
  void PrintHelp(CommandHandler_c* commandHandler ){}
  comResp_et Handle(CommandData_st* comData_);
};

class CommandSurfaceOffset_c :public CommandGroup_c
{

  Com_surfoffset surfoffset;

  public:



};

#endif
