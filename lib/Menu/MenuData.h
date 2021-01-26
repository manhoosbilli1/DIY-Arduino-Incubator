#ifndef _sampleMenu_
#define _sampleMenu_
#include "MenuManager.h"
#include <avr/pgmspace.h>

/*

Generated using LCD Menu Builder at https://lcd-menu-bulder.cohesivecomputing.co.uk/
For more information, visit https://www.cohesivecomputing.co.uk/hackatronics/arduino-lcd-menu-library/

All our hackatronics projects are free for personal use. If you find them helpful or useful, please consider
making a small donation to our hackatronics fund using the donate buttons on our web pages. Thank you.
		
*/

enum sampleMenuCommandId
{
  mnuCmdBack = 0,
  mnuCmdSetTemp,
  mnuCmdSetHum,
  mnuCmdSetFreq,
  mnuCmdSetTurnDelay,
  mnuCmdSetTime,
  mnuCmdSetHours,
  mnuCmdSetMins,
  mnuCmdSetSeconds,
  mnuCmdSetYear,
  mnuCmdSetMonth,
  mnuCmdSetDay,
  mnuCmdCalcHatchDay,
  mnuCmdSetDays
};

PROGMEM const char sampleMenu_back[] = "Back";
PROGMEM const char sampleMenu_exit[] = "Exit";

PROGMEM const char sampleMenu_5_1[] = "Set Hours";
PROGMEM const char sampleMenu_5_2[] = "Set Minutes";
PROGMEM const char sampleMenu_5_3[] = "Set Seconds";
PROGMEM const char sampleMenu_5_4[] = "Set Year";
PROGMEM const char sampleMenu_5_5[] = "Set Month";
PROGMEM const char sampleMenu_5_6[] = "Set Day";
PROGMEM const MenuItem sampleMenu_List_5[] = {{mnuCmdSetHours, sampleMenu_5_1}, {mnuCmdSetMins, sampleMenu_5_2}, {mnuCmdSetSeconds, sampleMenu_5_3}, {mnuCmdSetYear, sampleMenu_5_4}, {mnuCmdSetMonth, sampleMenu_5_5}, {mnuCmdSetDay, sampleMenu_5_6}, {mnuCmdBack, sampleMenu_back}};

PROGMEM const char sampleMenu_6_1[] = "Incubation Time";
PROGMEM const MenuItem sampleMenu_List_6[] = {{mnuCmdSetDays, sampleMenu_6_1}, {mnuCmdBack, sampleMenu_back}};

PROGMEM const char sampleMenu_1[] = "Temp SetPoint";
PROGMEM const char sampleMenu_2[] = "Humidty SetPoin";
PROGMEM const char sampleMenu_3[] = "Turns per day";
PROGMEM const char sampleMenu_4[] = "Tray Turn Delay";
PROGMEM const char sampleMenu_5[] = "Set Time";
PROGMEM const char sampleMenu_6[] = "Calc Hatch Day";
PROGMEM const MenuItem sampleMenu_Root[] = {{mnuCmdSetTemp, sampleMenu_1}, {mnuCmdSetHum, sampleMenu_2}, {mnuCmdSetFreq, sampleMenu_3}, {mnuCmdSetTurnDelay, sampleMenu_4}, {mnuCmdSetTime, sampleMenu_5, sampleMenu_List_5, menuCount(sampleMenu_List_5)}, {mnuCmdCalcHatchDay, sampleMenu_6, sampleMenu_List_6, menuCount(sampleMenu_List_6)}, {mnuCmdBack, sampleMenu_exit}};

/*
case mnuCmdSetTemp :
	break;
case mnuCmdSetHum :
	break;
case mnuCmdSetFreq :
	break;
case mnuCmdSetTurnDelay :
	break;
case mnuCmdSetHours :
	break;
case mnuCmdSetMins :
	break;
case mnuCmdSetSeconds :
	break;
case mnuCmdSetYear :
	break;
case mnuCmdSetMonth :
	break;
case mnuCmdSetDay :
	break;
case mnuCmdSetDays :
	break;
*/

/*
<?xml version="1.0"?>
<RootMenu xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:xsd="http://www.w3.org/2001/XMLSchema">
    <Config IdPrefix="mnuCmd" VarPrefix="sampleMenu" UseNumbering="false" IncludeNumberHierarchy="false" MaxNameLen="15" MenuBackFirstItem="false" BackText="Back" ExitText="Exit" AvrProgMem="true"/>
    <MenuItems>
        <Item Id="SetTemp" Name="Temp SetPoint"/>
        <Item Id="SetHum" Name="Humidty SetPoint"/>
        <Item Id="SetFreq" Name="Turns per day"/>
        <Item Id="SetTurnDelay" Name="Tray Turn Delay"/>
        <Item Id="SetTime" Name="Set Time">
            <MenuItems>
                <Item Id="SetHours" Name="Set Hours"/>
                <Item Id="SetMins" Name="Set Minutes"/>
                <Item Id="SetSeconds" Name="Set Seconds"/>
                <Item Id="SetYear" Name="Set Year"/>
                <Item Id="SetMonth" Name="Set Month"/>
                <Item Id="SetDay" Name="Set Day"/>
            </MenuItems>
        </Item>
        <Item Id="CalcHatchDay" Name="Calc Hatch Day">
            <MenuItems>
                <Item Id="SetDays" Name="Incubation Time"/>
            </MenuItems>
        </Item>
    </MenuItems>
</RootMenu>
*/
#endif
