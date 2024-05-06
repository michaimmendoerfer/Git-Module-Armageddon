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
#define GAUGES_PER_SCREEN   5

extern PeerClass Module;
extern void ToggleSwitch(int SNr);
extern void ChangeBrightness(int B);
extern void SetDemoMode(bool Mode);
extern void SetDebugMode(bool Mode);
extern void SetSleepMode(bool Mode);
extern void SetPairMode(bool Mode);
extern void SaveModule();
extern void SendNameChange(int Pos);
extern void CurrentCalibration();
extern void VoltageCalibration(int SNr, float V);
extern LinkedList<PeriphClass*> SwitchList;
extern LinkedList<PeriphClass*> SensorList;
extern bool NameChanged;

extern uint32_t TSPair;

int ScrSwitchPage = 0;
int ScrGaugeMultiPage = 0;
//int ActiveSensorNr = -1;
int ActiveChangeNameNr = -1;
PeriphClass *ActiveSensor = NULL;

void SwitchUpdateTimer(lv_timer_t * timer);
void SettingsUpdateTimer(lv_timer_t * timer);
void GaugeSingleUpdateTimer(lv_timer_t * timer);
void GaugeMultiUpdateTimer(lv_timer_t * timer);
void CalibrationUpdateTimer(lv_timer_t * timer);

lv_timer_t *SwitchTimer;
lv_timer_t *GaugeSingleTimer;
lv_timer_t *GaugeMultiTimer;
lv_timer_t *CalibTimer;
lv_timer_t *SettingsTimer;

lv_obj_t *SingleMeter;
lv_meter_indicator_t * SingleIndic;
lv_meter_indicator_t * SingleIndicNeedle;
lv_meter_scale_t * scale;

lv_obj_t *Meter[GAUGES_PER_SCREEN];
lv_obj_t *GaugeList[GAUGES_PER_SCREEN] = {ui_BtnGaugeMultiGauge1, ui_BtnGaugeMultiGauge2, ui_BtnGaugeMultiGauge3, ui_BtnGaugeMultiGauge4, ui_BtnGaugeMultiGauge5};
lv_meter_indicator_t  *Indic[GAUGES_PER_SCREEN];
lv_meter_scale_t *Scale[GAUGES_PER_SCREEN];

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
	char Buf[10];
	PeriphClass *Switch;

	for (int Sw=0; Sw<SWITCHES_PER_SCREEN; Sw++)
	{
		Container = lv_obj_get_child(lv_scr_act(), Sw);
		
		Switch = SwitchList.get(Sw+ScrSwitchPage*SWITCHES_PER_SCREEN);

		if (Switch)
		{
			if (Switch->GetValue() == 1) 
			{
				lv_imgbtn_set_state(lv_obj_get_child(Container, 0), LV_IMGBTN_STATE_CHECKED_RELEASED);
			}

			lv_label_set_text(lv_obj_get_child(Container, 1), Module.GetName());
			lv_label_set_text(lv_obj_get_child(Container, 2), Switch->GetName());
			lv_label_set_text(lv_obj_get_child(Container, 3), itoa(Switch->GetPos(), Buf, 10));
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
		
		PeriphPos = atoi(lv_label_get_text(lv_obj_get_child(Container, 3)));
		
		Serial.printf("SwitchUpdateTimer: Value of SWwitch %d (Pos %d) is %f", Sw, PeriphPos, Module.GetPeriphValue(PeriphPos));
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

void Ui_SwitchButton_Clicked(lv_event_t * e)
{
	lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    
	if(event_code == LV_EVENT_CLICKED) {
        auto Container 	 = lv_obj_get_parent(target);
		auto SwitchLabel = lv_obj_get_child(Container, 3);
		//auto SwitchName  = lv_obj_get_child(Container, 2);
	    int  SwitchPos   = atoi(lv_label_get_text(SwitchLabel));
		
		//Serial.printf("Button %d pressed from Switch %s\n\r", SwitchNr, lv_label_get_text(SwitchName));
		Serial.printf("ToToggle = %d", SwitchPos);
		ToggleSwitch(SwitchPos);	
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
		//auto SwitchName  = lv_obj_get_child(Container, 2);
	    int SwitchNr = atoi(lv_label_get_text(SwitchLabel));
		ActiveChangeNameNr = SwitchNr;

		_ui_screen_change(&ui_ScrChangeName, LV_SCR_LOAD_ANIM_FADE_ON, 100, 0, &ui_ScrChangeName_screen_init);
	}
}

void Ui_Switch_Next(lv_event_t * e)
{
	if (SwitchList.size() > (ScrSwitchPage+1)*SWITCHES_PER_SCREEN)
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
void Ui_Settings_Brightness_Changed(lv_event_t * e)
{
	ChangeBrightness(atoi(lv_label_get_text(ui_SlSettingsBrightnessValue)));
}

void Ui_SettingBtn1_Click(lv_event_t * e)
{
	SetDemoMode(!Module.GetDemoMode());
}

void Ui_SettingBtn2_Click(lv_event_t * e)
{
	SetSleepMode(!Module.GetSleepMode());
}

void Ui_SettingBtn3_Click(lv_event_t * e)
{
	SetDebugMode(!Module.GetDebugMode());
}

void Ui_SettingBtn4_Click(lv_event_t * e)
{
	SetPairMode(!Module.GetPairMode());
}

void Ui_Settings_Loaded(lv_event_t * e)
{
	char ModuleName[100];
	char ModuleType[100];
	char ModuleBrightness[20];

	sprintf(ModuleName, "%s - %s", Module.GetName(), Module.GetVersion());
	strcpy(ModuleType, TypeInText(Module.GetType()));

	if (Module.GetVoltageMon() >= 0) strcat(ModuleType, " with Voltage-Monitor");

	lv_label_set_text(ui_LblSettingName, ModuleName);
	lv_label_set_text(ui_LblSettingType, ModuleType);
	
	sprintf(ModuleBrightness, "%d", Module.GetBrightness());
	lv_label_set_text(ui_SlSettingsBrightnessValue, ModuleBrightness);
	lv_slider_set_value(ui_SlSettingsBrightness, Module.GetBrightness(), LV_ANIM_ON);

	static uint32_t user_data = 10;
	if (!SettingsTimer) 
	{
		SettingsTimer = lv_timer_create(SettingsUpdateTimer, 500,  &user_data);
		Serial.println("SettingsTimer created");
	}
}

void Ui_Settings_Leave(lv_event_t * e)
{
	lv_timer_del(SettingsTimer);
	SettingsTimer = NULL;

	Serial.println("SettingsTimer deleted");
}

void SettingsUpdateTimer(lv_timer_t * timer)
{
	Serial.println("SettingsUpdateTimer");
	
	if (Module.GetDemoMode())  
		lv_obj_add_state(ui_SwSettingsDemo,  LV_STATE_CHECKED);
	else 
		lv_obj_clear_state(ui_SwSettingsDemo,  LV_STATE_CHECKED);
	
	if (Module.GetSleepMode()) 
		lv_obj_add_state(ui_SwSettingsSleep, LV_STATE_CHECKED);
	else
		lv_obj_clear_state(ui_SwSettingsSleep, LV_STATE_CHECKED);
	
	if (Module.GetDebugMode()) 
		lv_obj_add_state(ui_SwSettingsDebug, LV_STATE_CHECKED);
	else
		lv_obj_clear_state(ui_SwSettingsDebug, LV_STATE_CHECKED);
	
	if (Module.GetPairMode())  
		lv_obj_add_state(ui_SwSettingsPair,  LV_STATE_CHECKED);
	else
		lv_obj_clear_state(ui_SwSettingsPair,  LV_STATE_CHECKED);
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
	int G = 0;

	if (!ActiveSensor) ActiveSensor = PeriphList.get(0);
	
	if (ActiveSensor)
	{
		lv_label_set_text_fmt(ui_LblGaugeSingleValueDescription, "%10s", ActiveSensor->GetName());
		
		lv_obj_t *Gauge = ui_ImgGaugeSingleGauge;
		lv_obj_t *GaugeType  = ui_LblGaugeSingleType;
            
		Meter[0] = lv_meter_create(lv_scr_act());
		lv_obj_set_pos(Meter[G], 35, 78);
		lv_obj_set_size(Meter[G], 248, 234);
		
		Scale[G] = lv_meter_add_scale(Meter[G]);
		
		if (ActiveSensor->GetType() == SENS_TYPE_AMP)
		{
			lv_meter_set_scale_ticks(Meter[G], Scale[G], 21, 1, 5, lv_color_hex(0xff926b3f));
			lv_meter_set_scale_major_ticks(Meter[G], Scale[G], 5, 1, 8, lv_color_hex(0xff785212), 8);
			lv_meter_set_scale_range(Meter[G], Scale[G], 0, 400, 90, 225);
			lv_label_set_text(GaugeType, "A");
		
		}
		if (ActiveSensor->GetType() == SENS_TYPE_VOLT)
		{
			lv_meter_set_scale_ticks(Meter[G], Scale[G], 31, 1, 5, lv_color_hex(0xff926b3f));
			lv_meter_set_scale_major_ticks(Meter[G], Scale[G], 5, 1, 8, lv_color_hex(0xff785212), 8);
			lv_meter_set_scale_range(Meter[G], Scale[G], 90, 150, 90, 225);
			lv_label_set_text(GaugeType, "V");
		}	


		lv_obj_set_style_border_width(Meter[G], 0, LV_PART_MAIN | LV_STATE_DEFAULT);
		Indic[G] = lv_meter_add_needle_line(Meter[G], Scale[G], 4, lv_color_hex(0xff8f7a5b), -5);
		
		lv_obj_set_style_bg_opa(Meter[G], 0, LV_PART_MAIN | LV_STATE_DEFAULT);
		lv_obj_set_style_text_font(Meter[G], &lv_font_montserrat_14, LV_PART_MAIN | LV_STATE_DEFAULT);
		lv_obj_set_style_text_color(Meter[G], lv_color_hex(0xff5f430d), LV_PART_MAIN | LV_STATE_DEFAULT);
		lv_obj_clear_flag(Meter[G],LV_OBJ_FLAG_CLICKABLE);

		lv_obj_add_event_cb(Meter[G], SingleMeter_cb, LV_EVENT_DRAW_PART_BEGIN, NULL);
	}
	else
	{
		lv_label_set_text(ui_LblGaugeSingleValueDescription, "n.n.");
		lv_label_set_text(ui_LblGaugeSingleValue, "--.-V");
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
	int G = 0;
	if ((ActiveSensor) and (ActiveSensor->GetChanged()))
	{	
		char buf[10];
		dtostrf(ActiveSensor->GetValue(), 0, 1, buf);
		if (ActiveSensor->GetType() == SENS_TYPE_VOLT) strcat(buf, "V");
		if (ActiveSensor->GetType() == SENS_TYPE_AMP)  strcat(buf, "A");
		
		lv_label_set_text(ui_LblGaugeSingleValue, buf);
		lv_meter_set_indicator_value(Meter[G], Indic[G], ActiveSensor->GetValue()*10);
	}
}

void Ui_GaugeSingle_Leave(lv_event_t * e)
{
	lv_timer_del(GaugeSingleTimer);
	GaugeSingleTimer = NULL;

	int G = 0;
	lv_obj_del(Meter[G]);
	
	Meter[G]  = NULL;
	Scale[G]  = NULL;
	Indic[G]  = NULL;

	Serial.println("GaugeSingleTimer deleted");
}

void Ui_GaugeSingle_Next(lv_event_t * e)
{
	ActiveSensor = FindNextPeriph(NULL, ActiveSensor, SENS_TYPE_SENS, true);
	Ui_GaugeSingle_Leave(e);
	Ui_GaugeSingle_Loaded(e);
}

void Ui_GaugeSingle_Prev(lv_event_t * e)
{
	ActiveSensor = FindPrevPeriph(NULL, ActiveSensor, SENS_TYPE_SENS, true);
	Ui_GaugeSingle_Leave(e);
	Ui_GaugeSingle_Loaded(e);
}
#pragma endregion GAUGESINGLE
#pragma region GAUGEMULTI
void Ui_MultiGauge_Loaded(lv_event_t * e)
{
	lv_obj_t *Gauge;
	lv_obj_t *GaugeName;
	lv_obj_t *GaugeValue;
	lv_obj_t *GaugeType;

	PeriphClass *Sensor;

	for (int G=0; G<GAUGES_PER_SCREEN; G++)
	{
		Gauge = lv_obj_get_child(lv_scr_act(), 3*GAUGES_PER_SCREEN+G);
		GaugeName  = lv_obj_get_child(lv_scr_act(), G);
		GaugeValue = lv_obj_get_child(lv_scr_act(), G+GAUGES_PER_SCREEN);
		GaugeType  = lv_obj_get_child(lv_scr_act(), G+2*GAUGES_PER_SCREEN);
		
		Sensor = SensorList.get(G+ScrGaugeMultiPage*GAUGES_PER_SCREEN);

		if (Sensor)
		{
			lv_label_set_text(GaugeName, Sensor->GetName());
			
			// meter
            Meter[G] = lv_meter_create(lv_scr_act()); 
            
            lv_obj_set_pos(Meter[G], lv_obj_get_x(Gauge)-13, lv_obj_get_y(Gauge)+5);
            lv_obj_set_size(Meter[G], 126, 145);
            
			Scale[G] = lv_meter_add_scale(Meter[G]);
			
			if (Sensor->GetType() == SENS_TYPE_AMP)
			{
				lv_meter_set_scale_ticks(Meter[G], Scale[G], 21, 1, 5, lv_color_hex(0xff926b3f));
				lv_meter_set_scale_major_ticks(Meter[G], Scale[G], 5, 1, 8, lv_color_hex(0xff785212), 5);
				lv_meter_set_scale_range(Meter[G], Scale[G], 0, 40, 90, 225);
				lv_label_set_text(GaugeType, "A");
			
			}
			if (Sensor->GetType() == SENS_TYPE_VOLT)
			{
				lv_meter_set_scale_ticks(Meter[G], Scale[G], 13, 1, 5, lv_color_hex(0xff926b3f));
				lv_meter_set_scale_major_ticks(Meter[G], Scale[G], 4, 1, 8, lv_color_hex(0xff785212), 5);
				lv_meter_set_scale_range(Meter[G], Scale[G], 9, 15, 90, 225);
				lv_label_set_text(GaugeType, "V");
			}	
			
			lv_obj_set_style_border_width(Meter[G], 0, LV_PART_MAIN | LV_STATE_DEFAULT);
			Indic[G] = lv_meter_add_needle_line(Meter[G], Scale[G], 2, lv_color_hex(0xff8f7a5b), -5);
            
            lv_obj_set_style_bg_opa(Meter[G], 0, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_text_font(Meter[G], &lv_font_montserrat_8, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_text_color(Meter[G], lv_color_hex(0xff5f430d), LV_PART_MAIN | LV_STATE_DEFAULT);
			lv_obj_clear_flag(Meter[G],LV_OBJ_FLAG_CLICKABLE);
		}
		else 
		{
			lv_obj_add_flag(Gauge, LV_OBJ_FLAG_HIDDEN);
			lv_obj_add_flag(GaugeName, LV_OBJ_FLAG_HIDDEN);
			lv_obj_add_flag(GaugeValue, LV_OBJ_FLAG_HIDDEN);
		}
	}

	static uint32_t user_data = 10;
	if (!GaugeMultiTimer) 
	{
		GaugeMultiTimer = lv_timer_create(GaugeMultiUpdateTimer, 500,  &user_data);
		Serial.println("GaugeMultiTimer created");
	}
}

void GaugeMultiUpdateTimer(lv_timer_t * timer)
{
	lv_obj_t *Gauge;
	lv_obj_t *GaugeName;
	lv_obj_t *GaugeValue;
	lv_obj_t *GaugeType;

	PeriphClass *Sensor;
	
	Serial.println("GaugeMultiUpdateTimer");
	
	for (int G=0; G<GAUGES_PER_SCREEN; G++)
	{
		Gauge      = lv_obj_get_child(lv_scr_act(), G+3*GAUGES_PER_SCREEN);
		GaugeName  = lv_obj_get_child(lv_scr_act(), G);
		GaugeValue = lv_obj_get_child(lv_scr_act(), G+GAUGES_PER_SCREEN);
		
		Sensor = SensorList.get(G+ScrGaugeMultiPage*GAUGES_PER_SCREEN);

		if (Sensor)
		{
			if (Sensor->hasChanged())
				{	
					char buf[10];
					dtostrf(Sensor->GetValue(), 0, 1, buf);
					if (Sensor->GetType() == SENS_TYPE_VOLT) strcat(buf, "V");
					if (Sensor->GetType() == SENS_TYPE_AMP ) strcat(buf, "A");
					
					lv_label_set_text(GaugeValue, buf);
					lv_meter_set_indicator_value(Meter[G], Indic[G], abs(Sensor->GetValue()));
				}
		}
	}
}
void Ui_GaugeMulti_Leave(lv_event_t * e)
{
	lv_timer_del(GaugeMultiTimer);
	GaugeMultiTimer = NULL;

	for (int G=0; G<GAUGES_PER_SCREEN; G++)
	{
		if (Meter[G])
		{
			lv_obj_del(Meter[G]);
	
			Meter[G]  = NULL;
			Scale[G]  = NULL;
			Indic[G]  = NULL;
		}
	}

	Serial.println("GaugeMultiTimer deleted");
}

void Ui_GaugeMulti_Gauge_Click(lv_event_t * e)
{
	lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
	PeriphClass *Sensor;

	if(event_code == LV_EVENT_CLICKED) {
		if (target == ui_BtnGaugeMultiGauge1)      Sensor = SensorList.get(0+ScrGaugeMultiPage*GAUGES_PER_SCREEN);
		else if (target == ui_BtnGaugeMultiGauge2) Sensor = SensorList.get(1+ScrGaugeMultiPage*GAUGES_PER_SCREEN);
		else if (target == ui_BtnGaugeMultiGauge3) Sensor = SensorList.get(2+ScrGaugeMultiPage*GAUGES_PER_SCREEN);
		else if (target == ui_BtnGaugeMultiGauge4) Sensor = SensorList.get(3+ScrGaugeMultiPage*GAUGES_PER_SCREEN);
		else if (target == ui_BtnGaugeMultiGauge5) Sensor = SensorList.get(4+ScrGaugeMultiPage*GAUGES_PER_SCREEN);
	}
	if (Sensor) 
	{
		ActiveSensor = Sensor;
		_ui_screen_change(&ui_ScrGaugeSingle, LV_SCR_LOAD_ANIM_FADE_ON, 100, 0, &ui_ScrGaugeSingle_screen_init);
	}
	
}

#pragma endregion GAUGEMULTI
#pragma region CHANGENAME
void Ui_ChangeName_Next(lv_event_t * e)
{
	ActiveChangeNameNr++;
	int NewNr = ActiveChangeNameNr;
	
	if (NewNr == MAX_PERIPHERALS) NewNr = 0;

	if (Module.isPeriphEmpty(NewNr) == false)
	{
		ActiveChangeNameNr = NewNr;
		lv_textarea_set_text(ui_TxtAreaChangeName, Module.GetPeriphName(ActiveChangeNameNr));
	}
}

void Ui_ChangeName_Prev(lv_event_t * e)
{
	ActiveChangeNameNr--;
	int NewNr = ActiveChangeNameNr;
	
	if (NewNr == -1) NewNr = MAX_PERIPHERALS-1;

	if (Module.isPeriphEmpty(NewNr) == false)
	{
		ActiveChangeNameNr = NewNr;
		lv_textarea_set_text(ui_TxtAreaChangeName, Module.GetPeriphName(ActiveChangeNameNr));
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
		NameChanged = true;
		//SendNameChange(ActiveChangeNameNr);
	}
	_ui_screen_change(&ui_ScrMenu, LV_SCR_LOAD_ANIM_FADE_ON, 100, 0, &ui_ScrMenu_screen_init);
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

	char LblValues[100];
	char LblNames[110];
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
				strcat(LblValues, "\n\r");
				strcat(LblNames, "\n\r");
			}

			sprintf(Buf, "%.2fA\n\r", Module.GetPeriphValue(SNr));
			strcat(LblValues, Buf);
			sprintf(Buf, "%s\n\r", Module.GetPeriphName(SNr));
			strcat(LblNames, Buf);
		}
	}
	lv_label_set_text(ui_LblCalibCurrentValues, LblValues);
	lv_label_set_text(ui_LblCalibCurrentNames,  LblNames);

	if (Module.GetVoltageMon() != -1)
	{
		char buf[10];
		dtostrf(Module.GetPeriphValue(Module.GetVoltageMon()), 0, 1, buf);
		strcat(buf, "V");

		lv_label_set_text(ui_LblCalibCurrentValues, buf);
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
