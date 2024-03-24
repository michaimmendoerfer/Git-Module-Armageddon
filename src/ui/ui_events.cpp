// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.4
// LVGL version: 8.2.0
// Project name: SquareLine_Project_Jeepify_Large_PDC

#include "ui.h"
#include "Jeepify.h"
#include "PeerClass.h"
#include "pref_manager.h"
#include <nvs_flash.h>

#define SWITCHES_PER_SCREEN 4

extern PeerClass Module;
extern void ToggleSwitch(int SNr);
extern void SetDemoMode(bool Mode);
extern void SetDebugMode(bool Mode);
extern void SetSleepMode(bool Mode);
extern void SaveModule();
extern void SendNameChange(int Pos);
extern void CurrentCalibration();
extern void VoltageCalibration(int SNr, float V);

extern uint32_t TSPair;

int ScrSwitchPage = 0;
int ActiveSensorNr = -1;
int ActiveChangeNameNr = -1;

void SwitchUpdateTimer(lv_timer_t * timer);
void GaugeSingleUpdateTimer(lv_timer_t * timer);
void CalibrationUpdateTimer(lv_timer_t * timer);

lv_timer_t *SwitchTimer;
lv_timer_t *GaugeSingleTimer;
lv_timer_t *CalibTimer;

lv_obj_t *SingleMeter;
lv_meter_indicator_t * SingleIndic;
lv_meter_indicator_t * SingleIndicNeedle;
lv_meter_scale_t * scale;

#pragma region MENU
void Ui_Menu_Loaded(lv_event_t * e)
{
	if (Module.GetPairMode()) lv_obj_add_state(ui_BtnMenu4, LV_STATE_CHECKED);
}

void Ui_MenuBtn4_Click(lv_event_t * e)
{
	Module.SetPairMode(!Module.GetPairMode());
	if (Module.GetPairMode()) 
	{
		lv_obj_add_state(ui_BtnMenu4, LV_STATE_CHECKED);
		TSPair = millis();
		//smartdisplay_led_set_rgb(1,0,0);
	}
	else
	{
		lv_obj_clear_state(ui_BtnMenu4, LV_STATE_CHECKED);
		TSPair = 0;
		//smartdisplay_led_set_rgb(0,0,0);
	}
}
#pragma endregion MENU
#pragma region SWITCH
void Ui_Switch_Loaded(lv_event_t * e)
{
	lv_obj_t *Container;
	int PeriphPos;
	
	for (int Sw=0; Sw<SWITCHES_PER_SCREEN; Sw++)
	{
		Container = lv_obj_get_child(lv_scr_act(), Sw);
		PeriphPos = ScrSwitchPage*SWITCHES_PER_SCREEN+Sw;

		if (!Module.isPeriphEmpty(PeriphPos))
		{
			if (Module.GetPeriphValue(PeriphPos)) 
				lv_imgbtn_set_state(lv_obj_get_child(Container, 0), LV_IMGBTN_STATE_CHECKED_RELEASED);
			lv_label_set_text(lv_obj_get_child(Container, 1), Module.GetName());
			lv_label_set_text(lv_obj_get_child(Container, 2), Module.GetPeriphName(PeriphPos));
		}
		else 
		{
			lv_obj_add_flag(Container, LV_OBJ_FLAG_HIDDEN);
		}
	}

	static uint32_t user_data = 10;
	if (!SwitchTimer) 
	{
		SwitchTimer = lv_timer_create(SwitchUpdateTimer, 500,  &user_data);
		Serial.println("SwitchTimer created");
	}
}

void SwitchUpdateTimer(lv_timer_t * timer)
{
	Serial.println("SinglUpdateTimer");
	lv_obj_t *Container;
	int PeriphPos;
	
	for (int Sw=0; Sw<SWITCHES_PER_SCREEN; Sw++)
	{
		Container = lv_obj_get_child(lv_scr_act(), Sw);
		PeriphPos = ScrSwitchPage*SWITCHES_PER_SCREEN+Sw;

		if (!Module.isPeriphEmpty(PeriphPos))
		{
			Serial.printf("SingleUpdateTimer: Value of %d is %f", PeriphPos, Module.GetPeriphValue(PeriphPos));
			if (Module.GetPeriphValue(PeriphPos) == 1) 
			{
				lv_imgbtn_set_state(lv_obj_get_child(Container, 0), LV_IMGBTN_STATE_CHECKED_RELEASED);
			}
			else
			{
				lv_imgbtn_set_state(lv_obj_get_child(Container, 0), LV_IMGBTN_STATE_RELEASED);
			}
		}
	}	
}

void Ui_SwitchButton_Clicked(lv_event_t * e)
{
	lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    
	if(event_code == LV_EVENT_CLICKED) {
        auto Container 	 = lv_obj_get_parent(target);
		auto SwitchLabel = lv_obj_get_child(Container, 3);
		auto SwitchName  = lv_obj_get_child(Container, 2);
	    int SwitchNr = atoi(lv_label_get_text(SwitchLabel));
		
		//Serial.printf("Button %d pressed from Switch %s\n\r", SwitchNr, lv_label_get_text(SwitchName));
		int ToToggle = ScrSwitchPage*SWITCHES_PER_SCREEN+SwitchNr-1;
		Serial.printf("ToToggle = %d", ToToggle);
		ToggleSwitch(ToToggle);	
	}
}

void Ui_Switch_SwitchButton_Long(lv_event_t * e)
{
	// wird nicht erreicht
	
	lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    
	if(event_code == LV_EVENT_CLICKED) {
        auto Container 	 = lv_obj_get_parent(target);
		auto SwitchLabel = lv_obj_get_child(Container, 3);
		auto SwitchName  = lv_obj_get_child(Container, 2);
	    int SwitchNr = atoi(lv_label_get_text(SwitchLabel));
		ActiveChangeNameNr = SwitchNr;

		_ui_screen_change(&ui_ScrChangeName, LV_SCR_LOAD_ANIM_FADE_ON, 100, 0, &ui_ScrChangeName_screen_init);
	}
}

void Ui_Switch_Next(lv_event_t * e)
{
	if ((ScrSwitchPage == 0) and (Module.GetType() == SWITCH_8_WAY))
		ScrSwitchPage++;
	Ui_Switch_Loaded(e);
}

void UI_Switch_Prev(lv_event_t * e)
{
	if (ScrSwitchPage > 0) ScrSwitchPage--;
	Ui_Switch_Loaded(e);
}

void Ui_Switch_Leave(lv_event_t * e)
{
	lv_timer_del(SwitchTimer);
	SwitchTimer = NULL;

	Serial.println("SwitchTimer deleted");
}
#pragma endregion SWITCH
#pragma region SETTINGS
void Ui_SettingBtn1_Click(lv_event_t * e)
{
	Module.SetDemoMode(!Module.GetDemoMode());
	SetDemoMode(Module.GetDemoMode());
}

void Ui_SettingBtn2_Click(lv_event_t * e)
{
	Module.SetSleepMode(!Module.GetSleepMode());
	SetSleepMode(Module.GetSleepMode());
}

void Ui_SettingBtn3_Click(lv_event_t * e)
{
	Module.SetDebugMode(!Module.GetDebugMode());
	SetDebugMode(Module.GetDebugMode());
}

void Ui_SettingBtn4_Click(lv_event_t * e)
{
	Module.SetPairMode(!Module.GetPairMode());
}

void Ui_Settings_Loaded(lv_event_t * e)
{
	if (Module.GetDemoMode())  lv_obj_add_state(ui_SwSettingsDemo,  LV_STATE_CHECKED);
	if (Module.GetSleepMode()) lv_obj_add_state(ui_SwSettingsSleep, LV_STATE_CHECKED);
	if (Module.GetDebugMode()) lv_obj_add_state(ui_SwSettingsDebug, LV_STATE_CHECKED);
	if (Module.GetPairMode())  lv_obj_add_state(ui_SwSettingsPair,  LV_STATE_CHECKED);

	char ModuleName[100];
	char ModuleType[100];

	sprintf(ModuleName, "%s - %s", Module.GetName(), Module.GetVersion());
	strcpy(ModuleType, TypeInText(Module.GetType()));

	if (Module.GetVoltageMon() >= 0) strcat(ModuleType, " with Voltage-Monitor");

	lv_label_set_text(ui_LblSettingName, ModuleName);
	lv_label_set_text(ui_LblSettingType, ModuleType);
}

void Ui_Settings_Reset(lv_event_t * e)
{
	nvs_flash_erase(); nvs_flash_init();
    ESP.restart();
}

#pragma endregion SETTINGS
#pragma region GAUGESINGLE

static void SingleMeter_cb(lv_event_t * e) {

	lv_obj_draw_part_dsc_t	*dsc  = (lv_obj_draw_part_dsc_t *)lv_event_get_param(e);
	double					value;

	if( dsc->text != NULL ) {		// Filter major ticks...
		value = dsc->value / 10;
		snprintf(dsc->text, sizeof(dsc->text), "%5.1f", value);
	}
}
void Ui_GaugeSingle_Loaded(lv_event_t * e)
{
	if (ActiveSensorNr < 0)
	{
		for (int SNr=0; SNr<MAX_PERIPHERALS; SNr++)
		{
			if (Module.GetPeriphType(SNr) == SENS_TYPE_VOLT)
			{
				ActiveSensorNr = SNr;
				Serial.printf("Sensor gefunden: %d\n\r", ActiveSensorNr);
				break;
			}
		}
	}
	
	if (ActiveSensorNr >= 0)
	{
		lv_label_set_text_fmt(ui_LblGaugeSingleValueDescription, "%10s", Module.GetPeriphName(ActiveSensorNr));
	}
	else
	{
		lv_label_set_text(ui_LblGaugeSingleValueDescription, "n.n.");
		lv_label_set_text(ui_LblGaugeSingleValue, "--.-V");
	}
	
	SingleMeter = lv_meter_create(ui_ImgGaugeSingleGauge);
	lv_obj_center(SingleMeter);
	lv_obj_set_style_bg_color(SingleMeter, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_bg_opa(SingleMeter, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_size(SingleMeter, 180, 180);
	lv_obj_set_pos(SingleMeter, 0,0);
	scale = lv_meter_add_scale(SingleMeter);
	
	lv_obj_move_background(ui_ImgGaugeSingleGauge);
	lv_obj_set_style_text_color(SingleMeter, lv_color_hex(0xdbdbdb), LV_PART_TICKS);
	
	SingleIndicNeedle = lv_meter_add_needle_line(SingleMeter, scale, 4, lv_palette_main(LV_PALETTE_GREY), -10);
	
	if ((ActiveSensorNr>=0) and (Module.GetPeriphType(ActiveSensorNr) == SENS_TYPE_VOLT))
	{
		//lv_meter_set_scale_ticks(SingleMeter, scale, 41, 2, 10, lv_palette_main(LV_PALETTE_GREY));
    	lv_meter_set_scale_major_ticks(SingleMeter, scale, 5, 4, 15, lv_color_black(), 15);
    	lv_meter_set_scale_range(SingleMeter, scale, 0, 400, 90, 270);
	
		//Add a green arc to the start
		SingleIndic = lv_meter_add_scale_lines(SingleMeter, scale, lv_palette_main(LV_PALETTE_GREEN), lv_palette_main(LV_PALETTE_GREEN), false, 0);
    	lv_meter_set_indicator_start_value(SingleMeter, SingleIndic, 0);
    	lv_meter_set_indicator_end_value(SingleMeter, SingleIndic, 250);

		SingleIndic = lv_meter_add_arc(SingleMeter, scale, 3, lv_palette_main(LV_PALETTE_RED), 0);
    	lv_meter_set_indicator_start_value(SingleMeter, SingleIndic, 300);
    	lv_meter_set_indicator_end_value(SingleMeter, SingleIndic, 400);

		//Make the tick lines red at the end of the scale
		SingleIndic = lv_meter_add_scale_lines(SingleMeter, scale, lv_palette_main(LV_PALETTE_RED), lv_palette_main(LV_PALETTE_RED), false, 0);
		lv_meter_set_indicator_start_value(SingleMeter, SingleIndic, 300);
		lv_meter_set_indicator_end_value(SingleMeter, SingleIndic, 400);

		lv_obj_add_event_cb(SingleMeter, SingleMeter_cb, LV_EVENT_DRAW_PART_BEGIN, NULL);
	}

	static uint32_t user_data = 10;
	if (!GaugeSingleTimer) 
	{
		GaugeSingleTimer = lv_timer_create(GaugeSingleUpdateTimer, 500,  &user_data);
		Serial.println("GaugeSingleTimer created");
	}
}
void GaugeSingleUpdateTimer(lv_timer_t * timer)
{
	if ((ActiveSensorNr >= 0) and (Module.GetPeriphChanged(ActiveSensorNr)))
	{	
		char buf[10];
		dtostrf(Module.GetPeriphValue(ActiveSensorNr), 0, 1, buf);
		strcat(buf, "V");
		lv_label_set_text(ui_LblGaugeSingleValue, buf);
		lv_meter_set_indicator_value(SingleMeter, SingleIndicNeedle, Module.GetPeriphValue(ActiveSensorNr)*10);
	}
}

void Ui_GaugeSingle_Leave(lv_event_t * e)
{
	lv_timer_del(GaugeSingleTimer);
	GaugeSingleTimer = NULL;

	lv_obj_del(SingleMeter);
	
	SingleMeter       = NULL;
	scale             = NULL;
	SingleIndicNeedle = NULL;
	
	Serial.println("GaugeSingleTimer deleted");
}

void Ui_GaugeSingle_Next(lv_event_t * e)
{
	// Your code here
}

void Ui_GaugeSingle_Prev(lv_event_t * e)
{
	// Your code here
}
#pragma endregion GAUGESINGLE
#pragma region CHANGENAME
void Ui_ChangeName_Next(lv_event_t * e)
{
	int NewNr = ActiveChangeNameNr++;
	if (NewNr == MAX_PERIPHERALS) NewNr = 0;

	if (Module.isPeriphEmpty(NewNr) == false)
	{
		ActiveChangeNameNr = NewNr;
		lv_textarea_set_text(ui_TxtAreaChangeName, Module.GetPeriphName(ActiveChangeNameNr));
	}
}

void Ui_ChangeName_Prev(lv_event_t * e)
{
	int NewNr = ActiveChangeNameNr;
	
	for (int SNr=0; SNr<MAX_PERIPHERALS; SNr++)
	{
		NewNr--;
		if (NewNr == -1) NewNr = MAX_PERIPHERALS-1;

		if (Module.isPeriphEmpty(NewNr) == false)
		{
			ActiveChangeNameNr = NewNr;
			lv_textarea_set_text(ui_TxtAreaChangeName, Module.GetPeriphName(ActiveChangeNameNr));
		}
	}
}

void Ui_ChangeName_Loaded(lv_event_t * e)
{
	if (ActiveChangeNameNr < 0)
	{
		for (int SNr=0; SNr<MAX_PERIPHERALS; SNr++)
		{
			if (Module.isPeriphEmpty(SNr) == false)
			{
				ActiveChangeNameNr = SNr;
				//Serial.printf("Sensor gefunden: %d\n\r", ActiveChangeNameNr);
				break;
			}
		}
	}
	
	if (ActiveChangeNameNr >= 0)
	{
		lv_textarea_set_text(ui_TxtAreaChangeName, Module.GetPeriphName(ActiveChangeNameNr));
	}
	else
	{
		lv_textarea_set_text(ui_TxtAreaChangeName, "nothing found");
	}
}

void Ui_ChangeName_Ready(lv_event_t * e)
{
	if (ActiveChangeNameNr >= 0)
	{
		Module.SetPeriphName(ActiveChangeNameNr, lv_textarea_get_text(ui_TxtAreaChangeName));
		SaveModule();
		SendNameChange(ActiveChangeNameNr);
	}
	_ui_screen_change(&ui_ScrSwitch, LV_SCR_LOAD_ANIM_FADE_ON, 100, 0, &ui_ScrSwitch_screen_init);
}
#pragma endregion CHANGENAME
#pragma region CALIB
void ui_Calib_Loaded(lv_event_t * e)
{
	static int user_data = 10;

	if (!CalibTimer) 
	{
		CalibTimer = lv_timer_create(CalibrationUpdateTimer, 500,  &user_data);
		Serial.println("CalibTimer created");
	}
}

void Ui_NumKey_Ready(lv_event_t * e)
{
	float NewVoltage = atof(lv_label_get_text(ui_TxtAreaScrNumKeyVoltage));

	if (Module.GetVoltageMon() != -1)
	{
		VoltageCalibration(Module.GetVoltageMon(), NewVoltage) ;
	}
	_ui_screen_change(&ui_ScreenCalib, LV_SCR_LOAD_ANIM_FADE_ON, 100, 0, &ui_ScreenCalib_screen_init);
}

void CalibrationUpdateTimer(lv_timer_t * timer)
{
	Serial.println("CalibUpdateTimer");

	char CurrentLines[300];
	char Buf[40];

	bool FirstRun = true;

	for (int SNr=0; SNr<MAX_PERIPHERALS; SNr++)
	{
		if (Module.GetPeriphType(SNr) == SENS_TYPE_AMP)
		{
			if (FirstRun) 
			{
				FirstRun = false;
			}
			else
			{
				strcat(CurrentLines, "\n\r");
			}

			sprintf(Buf, "(%.2fA) - %s\n\r", Module.GetPeriphValue(SNr), Module.GetPeriphName(SNr));
			strcat(CurrentLines, Buf);
		}
	}
	lv_textarea_set_text(ui_LblCalibCurrent, CurrentLines);

	if (Module.GetVoltageMon() != -1)
	{
		char buf[10];
		dtostrf(Module.GetPeriphValue(Module.GetVoltageMon()), 0, 1, buf);
		strcat(buf, "V");

		lv_label_set_text(ui_LblCalibVoltage, buf);
	}
}

void Ui_Calib_Leave(lv_event_t * e)
{
	lv_timer_del(CalibTimer);
	CalibTimer = NULL;

	Serial.println("CalibTimer deleted");
}

void ui_Calib_Current_Click(lv_event_t * e)
{
	CurrentCalibration();
}
#pragma endregion CALIB
void Ui_Init_Custom(lv_event_t * e)
{
	
}
