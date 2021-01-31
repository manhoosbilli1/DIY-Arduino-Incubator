#ifndef _runtimeMenu_
#define _runtimeMenu_
#include "MenuManager.h"
#include <avr/pgmspace.h>

/*
Generated using LCD Menu Builder at https://lcd-menu-bulder.cohesivecomputing.co.uk/
For more information, visit https://www.cohesivecomputing.co.uk/hackatronics/arduino-lcd-menu-library/
All our hackatronics projects are free for personal use. If you find them helpful or useful, please consider
making a small donation to our hackatronics fund using the donate buttons on our web pages. Thank you.
		
*/

enum runtimeMenuCommandId
{
  runtimeCmdBack = 0,
  runtimeCmdAdvancedSetup,
  runtimeCmdTurnMotorOnce,
  runtimeCmdSetTemp,
  runtimeCmdSetHum,
  runtimeCmdSetFreq,
  runtimeCmdSetTurnDelay,
  runtimeCmdSetTime,
  runtimeCmdSetHours,
  runtimeCmdSetMins,
  runtimeCmdSetSeconds,
  runtimeCmdSetYear,
  runtimeCmdSetMonth,
  runtimeCmdSetDay,
  runtimeCmdSetPID,
  runtimeCmdSetP,
  runtimeCmdSetI,
  runtimeCmdSetD,
  runtimeCmdIncubationTime,
  runtimeCmdSaveProfile,
  runtimeCmdShowHatchDay,
  runtimeCmdCalcHatchDay,
  runtimeCmdToggleLight,
  runtimeCmdToggleCandler,
  runtimeCmdIncubationDone,
  runtimeCmdConfirmation
};

PROGMEM const char runtimeMenu_back[] = "Back";
PROGMEM const char runtimeMenu_exit[] = "Exit";

PROGMEM const char runtimeMenu_1_6_1[] = "Set Hours";
PROGMEM const char runtimeMenu_1_6_2[] = "Set Minutes";
PROGMEM const char runtimeMenu_1_6_3[] = "Set Seconds";
PROGMEM const char runtimeMenu_1_6_4[] = "Set Year";
PROGMEM const char runtimeMenu_1_6_5[] = "Set Month";
PROGMEM const char runtimeMenu_1_6_6[] = "Set Day";
PROGMEM const MenuItem runtimeMenu_List_1_6[] = {{runtimeCmdSetHours, runtimeMenu_1_6_1}, {runtimeCmdSetMins, runtimeMenu_1_6_2}, {runtimeCmdSetSeconds, runtimeMenu_1_6_3}, {runtimeCmdSetYear, runtimeMenu_1_6_4}, {runtimeCmdSetMonth, runtimeMenu_1_6_5}, {runtimeCmdSetDay, runtimeMenu_1_6_6}, {runtimeCmdBack, runtimeMenu_back}};

PROGMEM const char runtimeMenu_1_7_1[] = "Set P constant";
PROGMEM const char runtimeMenu_1_7_2[] = "Set I constant";
PROGMEM const char runtimeMenu_1_7_3[] = "Set D constant";
PROGMEM const MenuItem runtimeMenu_List_1_7[] = {{runtimeCmdSetP, runtimeMenu_1_7_1}, {runtimeCmdSetI, runtimeMenu_1_7_2}, {runtimeCmdSetD, runtimeMenu_1_7_3}, {runtimeCmdBack, runtimeMenu_back}};

PROGMEM const char runtimeMenu_1_1[] = "Turn Motor Once";
PROGMEM const char runtimeMenu_1_2[] = "Temp SetPoint";
PROGMEM const char runtimeMenu_1_3[] = "Hum SetPoint";
PROGMEM const char runtimeMenu_1_4[] = "Turns per day";
PROGMEM const char runtimeMenu_1_5[] = "Tray Turn Delay";
PROGMEM const char runtimeMenu_1_6[] = "Set Time";
PROGMEM const char runtimeMenu_1_7[] = "PID constants";
PROGMEM const char runtimeMenu_1_8[] = "Incubation Time";
PROGMEM const char runtimeMenu_1_9[] = "Save Profile";
PROGMEM const MenuItem runtimeMenu_List_1[] = {{runtimeCmdTurnMotorOnce, runtimeMenu_1_1}, {runtimeCmdSetTemp, runtimeMenu_1_2}, {runtimeCmdSetHum, runtimeMenu_1_3}, {runtimeCmdSetFreq, runtimeMenu_1_4}, {runtimeCmdSetTurnDelay, runtimeMenu_1_5}, {runtimeCmdSetTime, runtimeMenu_1_6, runtimeMenu_List_1_6, menuCount(runtimeMenu_List_1_6)}, {runtimeCmdSetPID, runtimeMenu_1_7, runtimeMenu_List_1_7, menuCount(runtimeMenu_List_1_7)}, {runtimeCmdIncubationTime, runtimeMenu_1_8}, {runtimeCmdSaveProfile, runtimeMenu_1_9}, {runtimeCmdBack, runtimeMenu_back}};

PROGMEM const char runtimeMenu_6_1[] = "Are you sure?";
PROGMEM const MenuItem runtimeMenu_List_6[] = {{runtimeCmdConfirmation, runtimeMenu_6_1}, {runtimeCmdBack, runtimeMenu_back}};

PROGMEM const char runtimeMenu_1[] = "Advanced Setup";
PROGMEM const char runtimeMenu_2[] = "Show Hatch Day";
PROGMEM const char runtimeMenu_3[] = "Find Hatch Day";
PROGMEM const char runtimeMenu_4[] = "Toggle Light";
PROGMEM const char runtimeMenu_5[] = "Toggle Candler";
PROGMEM const char runtimeMenu_6[] = "Incubation Done";
PROGMEM const MenuItem runtimeMenu_Root[] = {{runtimeCmdAdvancedSetup, runtimeMenu_1, runtimeMenu_List_1, menuCount(runtimeMenu_List_1)}, {runtimeCmdShowHatchDay, runtimeMenu_2}, {runtimeCmdCalcHatchDay, runtimeMenu_3}, {runtimeCmdToggleLight, runtimeMenu_4}, {runtimeCmdToggleCandler, runtimeMenu_5}, {runtimeCmdIncubationDone, runtimeMenu_6, runtimeMenu_List_6, menuCount(runtimeMenu_List_6)}, {runtimeCmdBack, runtimeMenu_exit}};

/*
case runtimeCmdTurnMotorOnce :
	break;
case runtimeCmdSetTemp :
	break;
case runtimeCmdSetHum :
	break;
case runtimeCmdSetFreq :
	break;
case runtimeCmdSetTurnDelay :
	break;
case runtimeCmdSetHours :
	break;
case runtimeCmdSetMins :
	break;
case runtimeCmdSetSeconds :
	break;
case runtimeCmdSetYear :
	break;
case runtimeCmdSetMonth :
	break;
case runtimeCmdSetDay :
	break;
case runtimeCmdSetP :
	break;
case runtimeCmdSetI :
	break;
case runtimeCmdSetD :
	break;
case runtimeCmdIncubationTime :
	break;
case runtimeCmdSaveProfile :
	break;
case runtimeCmdShowHatchDay :
	break;
case runtimeCmdCalcHatchDay :
	break;
case runtimeCmdToggleLight :
	break;
case runtimeCmdToggleCandler :
	break;
case runtimeCmdConfirmation :
	break;
*/

/*
<?xml version="1.0"?>
<RootMenu xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:xsd="http://www.w3.org/2001/XMLSchema">
    <Config IdPrefix="runtimeCmd" VarPrefix="runtimeMenu" UseNumbering="false" IncludeNumberHierarchy="false" MaxNameLen="15" MenuBackFirstItem="false" BackText="Back" ExitText="Exit" AvrProgMem="true"/>
    <MenuItems>
        <Item Id="AdvancedSetup" Name="Advanced Setup">
            <MenuItems>
                <Item Id="TurnMotorOnce" Name="Turn Motor Once"/>
                <Item Id="SetTemp" Name="Temp SetPoint"/>
                <Item Id="SetHum" Name="Hum SetPoint"/>
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
                <Item Id="SetPID" Name="PID constants">
                    <MenuItems>
                        <Item Id="SetP" Name="Set P constant"/>
                        <Item Id="SetI" Name="Set I constant"/>
                        <Item Id="SetD" Name="Set D constant"/>
                    </MenuItems>
                </Item>
                <Item Id="IncubationTime" Name="Incubation Time"/>
                <Item Id="SaveProfile" Name="Save Profile"/>
            </MenuItems>
        </Item>
        <Item Id="ShowHatchDay" Name="Show Hatch Day"/>
        <Item Id="CalcHatchDay" Name="Find Hatch Day"/>
        <Item Id="ToggleLight" Name="Toggle Light"/>
        <Item Id="ToggleCandler" Name="Toggle Candler"/>
        <Item Id="IncubationDone" Name="Incubation Done?">
            <MenuItems>
                <Item Id="Confirmation" Name="Are you sure?"/>
            </MenuItems>
        </Item>
    </MenuItems>
</RootMenu>
*/
#endif