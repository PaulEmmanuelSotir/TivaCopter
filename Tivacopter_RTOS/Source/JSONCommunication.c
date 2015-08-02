/*
 * JSONCommunication.c
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

//----------------------------------------
// BIOS header files
//----------------------------------------
#include <xdc/std.h>  						//mandatory - have to include first, for BIOS types
#include <ti/sysbios/BIOS.h> 				//mandatory - if you call APIs like BIOS_start()
#include <xdc/runtime/Log.h>				//needed for any Log_info() call
#include <xdc/cfg/global.h> 				//header file for statically defined objects/handles>
#include <ti/sysbios/knl/Clock.h>
#include <xdc/runtime/Error.h>
#include <ti/sysbios/hal/Hwi.h>
#include <ti/sysbios/knl/Queue.h>

#include "inc/tm4c1294ncpdt.h"
#include "driverlib/debug.h"
#include "driverlib/rom_map.h"
#include "driverlib/uart.h"
#include "driverlib/rom.h"
#include "driverlib/gpio.h"

#include "PinMap.h"
#include "Utils/UARTConsole.h"
#include "Utils/jsmn.h"
#include "JSONCommunication.h"

static bool JSONCommunicationStarted = false;
static bool JSONProgrammaticAccessMode = true;

//----------------------------------------
// UART console from 'main.c'
//----------------------------------------
extern UARTConsole Console;

//----------------------------------------
// Private static JSON data sources array
//----------------------------------------
static struct
{
	JSONDataSource array[MAX_DATASOURCE_COUNT];
	uint32_t used;
	bool IsJSONDatasourcesArrayInitialized;
	const uint32_t capacity;
} JSONDataSources = {.used = 0, .IsJSONDatasourcesArrayInitialized = false, .capacity = MAX_DATASOURCE_COUNT};

static void InitializeJSONDataSourcesArray()
{
	if(!JSONDataSources.IsJSONDatasourcesArrayInitialized)
	{
		memset(JSONDataSources.array, NULL, JSONDataSources.capacity*sizeof(JSONDataSource));
		JSONDataSources.IsJSONDatasourcesArrayInitialized = true;
	}
}

//----------------------------------------
// Private static JSON data inputs array
//----------------------------------------
static struct
{
	JSONDataInput array[MAX_DATAINPUT_COUNT];
	uint32_t used;
	bool IsJSONDatainputsArrayInitialized;
	const uint32_t capacity;
} JSONDataInputs = {.used = 0, .IsJSONDatainputsArrayInitialized = false, .capacity = MAX_DATAINPUT_COUNT};

static void InitializeJSONDataInputsArray()
{
	if(!JSONDataInputs.IsJSONDatainputsArrayInitialized)
	{
		memset(JSONDataInputs.array, NULL, JSONDataInputs.capacity*sizeof(JSONDataInput));
		JSONDataInputs.IsJSONDatainputsArrayInitialized = true;
	}
}

//----------------------------------------
// Raw echo data source:
// Simple data source which sends back
// all received JSON objects.
//----------------------------------------
static JSONDataSource* rawEcho_ds;

//----------------------------------------
// list sources:
// List all available JSON data source.
//----------------------------------------
void JSON_list_sources_cmd(int argc, char *argv[])
{
	if(checkArgCount(&Console, argc, 1))
	{
		UARTwrite(&Console, "AVAILABLE JSON DATA SOURCES:", 28);

		uint32_t i;
		for(i = 0; i < JSONDataSources.capacity; ++i)
		{
			JSONDataSource* ds = &JSONDataSources.array[i];
			if(ds->name != NULL)
				UARTprintf(&Console, "\n - %s		%s %s", ds->name, ds->enabled ? "Enabled" : "Disabled", ds->period > 0 ? "(Periodic)" : "");
		}
	}
}

//----------------------------------------
// enable:
// Enables specified JSON data source.
//----------------------------------------
void JSON_enable_cmd(int argc, char *argv[])
{
	if(argc > 1)
	{
		while(--argc)
		{
			int32_t ds_idx = JSONDataSources.capacity;
			while(ds_idx--)
			{
				JSONDataSource ds = JSONDataSources.array[ds_idx];
				if(ds.name != NULL)
					if(strcmp(ds.name, argv[argc]) == 0)
					{
						ds.enabled = true;
						UARTprintf(&Console, "'%s' JSON data source enabled.\n", argv[argc]);
						break;
					}
			}

			if(ds_idx < 0)
				UARTprintf(&Console, "Wrong JSON data source name ('%s')\n", argv[argc]);
		}

	}
	else
		UARTwrite(&Console, "Too few arguments.", 18);
}

//----------------------------------------
// disable:
// Enables specified JSON data source.
//----------------------------------------
void JSON_disable_cmd(int argc, char *argv[])
{
	if(argc > 1)
	{
		while(--argc)
		{
			int32_t ds_idx = JSONDataSources.capacity;
			while(ds_idx--)
			{
				JSONDataSource ds = JSONDataSources.array[ds_idx];
				if(strcmp(ds.name, argv[argc]) == 0)
				{
					ds.enabled = false;
					UARTprintf(&Console, "'%s' JSON data source disabled.\n", argv[argc]);
					break;
				}
			}

			if(ds_idx < 0)
				UARTprintf(&Console, "Wrong JSON data source name ('%s')\n", argv[argc]);
		}
	}
	else
		UARTwrite(&Console, "Too few arguments.", 18);
}

//----------------------------------------
// start:
// Starts JSON communication.
//----------------------------------------
void JSON_start_cmd(int argc, char *argv[])
{
	if(checkArgCount(&Console, argc, 1))
	{
		DisableCmdLineInterface(&Console);
		JSONCommunicationStarted = true;
	}
}

//----------------------------------------
// programmatic access:
// Enables programmatic access mode. (new
// line means new JSON object)
//----------------------------------------
void JSON_enable_programatic_access_cmd(int argc, char *argv[])
{
	if(checkArgCount(&Console, argc, 1))
	{
		JSONProgrammaticAccessMode = true;
		UARTwrite(&Console, "Programmatic access mode enabled.", 33);
	}
}

//----------------------------------------
// programmatic access:
// Disables programmatic access mode.
//----------------------------------------
void JSON_disable_programatic_access_cmd(int argc, char *argv[])
{
	if(checkArgCount(&Console, argc, 1))
	{
		JSONProgrammaticAccessMode = false;
		UARTwrite(&Console, "Programmatic access mode disabled.", 34);
	}
}

//------------------------------------------
// New JSON object received:
// Called by UART console when a newline
// character is received.
//------------------------------------------
static void NewJSONObjectReceived(char c)
{
	static char buf[INPUT_JSON_BUFFER_SIZE];
	static jsmntok_t tokens[INPUT_JSON_TOKEN_NUM];
	static jsmn_parser parser;
	static char data[MAX_DATA_COUNT][32];
	static char* dataPtrs[MAX_DATA_COUNT];

	uint32_t i, j, tokNum;
	jsmntok_t* tok;
	JSONDataInput* di;
	const char* key;

	//TODO : lock
	memset(buf, 0, INPUT_JSON_BUFFER_SIZE);
	memset(data, 0, 32*MAX_DATA_COUNT);

	UARTgets(&Console, buf, INPUT_JSON_BUFFER_SIZE);

	if(rawEcho_ds != NULL)
		if(rawEcho_ds->enabled)
			SendJSONData(rawEcho_ds, (char**)(&buf));

	jsmn_init(&parser);
	tokNum = jsmn_parse(&parser, buf, INPUT_JSON_BUFFER_SIZE, tokens, INPUT_JSON_TOKEN_NUM);

	for(i = 0; i < JSONDataInputs.capacity; i++)
	{
		di = &JSONDataInputs.array[i];

		if(di->name != NULL)
		{
			if(tokNum != di->dataCount*2+1)
				continue;

			for(j = 0; j < di->dataCount; j++)
			{
				tok = &tokens[1+2*j];
				key = di->keys[j];
				if(!strncmp(buf + tok->start, key, tok->end - tok->start))
				{
					tok = &tokens[2+2*j];

					if(tok->end - tok->start > 32)
						return;

					strncpy(data[j], buf + tok->start, tok->end - tok->start);
					dataPtrs[j] = data[j];
				}
				else
					break;
			}

			if(j == di->dataCount)
			{
				di->dataAccessor((char**)dataPtrs);
				return;
			}
		}
	}
}

//---------------------------------------------
// Subscribe data source:
// Creates a data source and get it from static
// 'JSONDataSources' array.
// The 'keys' array should contain 'DataCount'
// names of the data fields provided by the
// datasource.
//---------------------------------------------
JSONDataSource* SubscribeJSONDataSource(const char* name, const char* keys[], uint32_t dataCount)
{
	return SubscribePeriodicJSONDataSource2(name, keys, dataCount, 0, NULL, true);
}

JSONDataSource* SubscribeJSONDataSource2(const char* name, const char* keys[], uint32_t dataCount, bool enabled)
{
	return SubscribePeriodicJSONDataSource2(name, keys, dataCount, 0, NULL, enabled);
}

//--------------------------------------------
// Subscribe periodic data source:
// Creates a periodic data source and get it
// from static 'JSONDataSources' array.
// The 'keys' array should contain 'DataCount'
// names of the data fields provided by the
// datasource.
// 'period' is the period of sending in RTOS
// clock ticks.
//--------------------------------------------
JSONDataSource* SubscribePeriodicJSONDataSource(const char* name, const char* keys[], uint32_t dataCount, uint32_t period, DataValuesGetAccessor dataAccessor)
{
	return SubscribePeriodicJSONDataSource2(name, keys, dataCount, period, dataAccessor, true);
}

JSONDataSource* SubscribePeriodicJSONDataSource2(const char* name, const char* keys[], uint32_t dataCount, uint32_t period, DataValuesGetAccessor dataAccessor, bool enabled)
{
	if(!JSONDataSources.IsJSONDatasourcesArrayInitialized)
		InitializeJSONDataSourcesArray();
	if(!JSONDataInputs.IsJSONDatainputsArrayInitialized)
		InitializeJSONDataInputsArray();

	if(dataCount > MAX_DATA_COUNT)
	{
		Log_error1("Error: Too much data fields given to create a new datasource (please modify MAX_DATA_COUNT=%u if needed).", MAX_DATA_COUNT);
		ASSERT(FALSE);
		return NULL;
	}

	if (JSONDataSources.used >= JSONDataSources.capacity)
	{
		Log_error1("Error: Maximum JSON datasources count reached (please modify MAX_DATASOURCE_COUNT=%u if needed).", MAX_DATASOURCE_COUNT);
		ASSERT(FALSE);
		return NULL;
	}

	// Find a free instance of JSONDataSource structure
	uint32_t idx = 0;
	if(JSONDataSources.used++ != 0)
		while(JSONDataSources.array[idx].name != NULL)
			if(idx++ > JSONDataSources.capacity)
			{
				Log_error1("Error: JSON datasources counter or datasources array corrrupted (MAX_DATASOURCE_COUNT=%u).", MAX_DATASOURCE_COUNT);
				ASSERT(FALSE);
				return NULL;
			}
	JSONDataSource* newSource = &JSONDataSources.array[idx];

	newSource->name = name;
	newSource->keys = keys;
	newSource->dataCount = dataCount;
	newSource->enabled = enabled;
	newSource->period = period;
	newSource->dataAccessor = dataAccessor;
	newSource->sendNowFlag = false;

	if(period > 0)
	{
		Clock_Params clockParams;
		Error_Block eb;
		Error_init(&eb);
		Clock_Params_init(&clockParams);
		clockParams.period = period;
		clockParams.startFlag = true;
		clockParams.arg = (UArg)newSource;
		newSource->clock = Clock_create(PeriodicJSONDataSendingSwi, period, &clockParams, &eb);
		if(newSource->clock == NULL)
		{
			UnsubscribeJSONDataSource(newSource);
			Log_error0("Error: Periodic data source clock creation failed.");
			return NULL;
		}
	}
	else
		newSource->clock = NULL;

	return newSource;
}

//---------------------------------------------
// Unsubscribe JSON data source:
// Unsubscribes given data source.
// Returns false if the given datasource wasn't
// subscribed, returns true otherwise.
//---------------------------------------------
bool UnsubscribeJSONDataSource(JSONDataSource* datasource)
{
	if(!JSONDataSources.IsJSONDatasourcesArrayInitialized)
		InitializeJSONDataSourcesArray();
	if(!JSONDataInputs.IsJSONDatainputsArrayInitialized)
		InitializeJSONDataInputsArray();

	if(datasource != NULL)
	{
		// Find specified datasource in JSON datasources array
		uint32_t idx = 0;
		while(&JSONDataSources.array[idx] != datasource)
			if(idx++ > JSONDataSources.capacity)
			{
				Log_warning0("Didn't found specified datasource during datasource unsubscription.");
				return false;
			}

		if(datasource->period > 0)
		{
			if(datasource->clock == NULL)
				Log_error0("Error: Periodic datasource corruption detected while unsubcribing it (period greater than 0 but NULL clock handle).");
			else
			{
				Clock_stop(datasource->clock);
				Clock_delete(&(datasource->clock));
			}
		}

		memset(datasource, NULL, sizeof(JSONDataSource));

		JSONDataSources.used--;

		return true;
	}

	return false;
}

//--------------------------------------------
// Send JSON data:
// Send specified data corresponding to given
// datasource ('values' array should have the
// same size as datasource's 'keys' array).
// Returns true if json data have been
// successfully sent, returns false otherwise.
//--------------------------------------------
bool SendJSONData(JSONDataSource* ds, char* values[])
{
	if(!JSONCommunicationStarted)
		return false;

	if(IsAbortRequested(&Console))
	{
		JSONCommunicationStarted = false;
		EnableCmdLineInterface(&Console);
		return false;
	}

	if(JSONDataSources.used > 0 && ds != NULL)
		if(ds->name != NULL)
		{
			uint32_t dsIdx;
			uint32_t valIdx;

			for(dsIdx = 0; dsIdx < JSONDataSources.capacity; ++dsIdx)
			{
				if(ds == &JSONDataSources.array[dsIdx])
				{
					if(ds->enabled)
					{
						UARTwrite(&Console, JSONProgrammaticAccessMode ? "\n{ " : "\n{\n", 3);

						for(valIdx = 0; valIdx < ds->dataCount; ++valIdx)
						{
							char* value = values[valIdx];
							const char* key = ds->keys[valIdx];

							if(value == NULL || key == NULL) {
								Log_error0("Error: JSON datasource provided wrong values or keys.");
								return false;
							}

							// Don't append comma if we are printing the last element
							const char* comma = valIdx == ds->dataCount-1 ? "" : ",";

							UARTprintf(&Console, JSONProgrammaticAccessMode ? " \"%s\": \"%s\"%s " : "\t\"%s\": \"%s\"%s \n", key, value, comma);
						}

						UARTwrite(&Console, JSONProgrammaticAccessMode ? " }" : "\n}", 2);

						// If Tx UART console buffer is near to be full, we wait for UART transmition
						// TODO: trouver mieux !
						uint32_t timeout = 10;
						if(UARTTxBytesFree(&Console) < 128)
							while(UARTTxBytesFree(&Console) < 1024 && --timeout)
								Task_sleep(1);

						return true;
					}
					Log_warning0("Tried to send disabled JSON datasource.");
					return false;
				}
			}
			Log_error0("Error: Invalid JSON datasource.");
			return false;
		}

	Log_error0("Error: there isn't any subscribed JSON datasource.");
	return false;
}

//------------------------------------------
// Periodic data sending task:
// Sends JSON data from periodic datasources
// in a low priority task.
//------------------------------------------
void PeriodicJSONDataSendingTask(void)
{
	// Subscribe UART console commands for JSON communication
	bool sucess = true;
	sucess = sucess && SubscribeCmd(&Console, "listSources", 	JSON_list_sources_cmd, 	"List all available JSON data sources.");
	sucess = sucess && SubscribeCmd(&Console, "enable", 		JSON_enable_cmd, 		"Enables specified JSON data source's stream (only active once \'start\' have been called).");
	sucess = sucess && SubscribeCmd(&Console, "disable", 		JSON_disable_cmd, 		"Disables specified JSON data source's stream.");
	sucess = sucess && SubscribeListeningCmd(&Console, "start", JSON_start_cmd, 		"Starts JSON communication.", "\n", NewJSONObjectReceived);
	sucess = sucess && SubscribeCmd(&Console, "progModeEn", 	JSON_enable_programatic_access_cmd, 	"Enables programmatic access mode. (newline means new JSON object)");
	sucess = sucess && SubscribeCmd(&Console, "progModeDis", 	JSON_disable_programatic_access_cmd, 	"Disables programmatic access mode.");
	if(!sucess)
	{
		Log_error0("Error (re)allocating memory for UART console command (from JSON API).");
		return;
	}

	// Subscribe raw echo from data inputs JSON datasource
	rawEcho_ds = SubscribeJSONDataSource2("rawEcho", (const char*[]) { "rawInput" }, 2, false);

	while(1)
	{
		Semaphore_pend(PeriodicJSON_Sem, BIOS_WAIT_FOREVER);

		if(JSONDataSources.used > 0)
		{
			uint32_t dsIdx;
			uint32_t valIdx;

			// Send data from JSON datasources accessors if they are enabled and if their sendNowFlag is raised
			for(dsIdx = 0; dsIdx < JSONDataSources.capacity; ++dsIdx)
			{
				JSONDataSource* ds = &JSONDataSources.array[dsIdx];
				if(ds->sendNowFlag && ds->enabled && ds->name != NULL)
				{
					UARTwrite(&Console, JSONProgrammaticAccessMode ? "\n{ " : "\n{\n", 3);

					// Get value string pointer array from JSON data source
					char** values = ds->dataAccessor();

					for(valIdx = 0; valIdx < ds->dataCount; ++valIdx)
					{
						char* value = values[valIdx];
						const char* key = ds->keys[valIdx];

						if(value == NULL || key == NULL)
						{
							Log_error0("Error: JSON datasource provided wrong values or keys.");
							break;
						}

						// Don't append comma if we are printing the last element
						const char* comma = valIdx == ds->dataCount-1 ? "" : ",";

						UARTprintf(&Console, JSONProgrammaticAccessMode ? " \"%s\": \"%s\"%s " : "\t\"%s\": \"%s\"%s \n", key, value, comma);
					}

					UARTwrite(&Console, JSONProgrammaticAccessMode ? " }" : "\n}", 2);

					// If Tx UART console buffer is near to be full, we wait for UART transmition
					// TODO: trouver mieux !
					uint32_t timeout = ds->period;
					if(UARTTxBytesFree(&Console) < 128)
						while(UARTTxBytesFree(&Console) < 1024 && --timeout)
							Task_sleep(1);

					ds->sendNowFlag = false;
				}
			}
			continue;
		}
		Log_error0("Error: there isn't any subscribed JSON datasource or data sending have been disabled.");
	}
}

//-----------------------------------------
// Periodic data sending software interrupt
//-----------------------------------------
void PeriodicJSONDataSendingSwi(UArg dataSource)
{
	if(IsAbortRequested(&Console))
	{
		JSONCommunicationStarted = false;
		EnableCmdLineInterface(&Console);
		return;
	}

	// Verify wether if JSON periodic sending task terminated
	Task_Stat PeriodicJSONTaskStat;
	Task_stat(PeriodicJSONDataSending_Task, &PeriodicJSONTaskStat);
	if(PeriodicJSONTaskStat.mode == ti_sysbios_knl_Task_Mode_TERMINATED)
	{
		Log_error0("Error: JSON periodic sending task terminated unexpectedly.");

		// Unsubscribe all data sources
		uint32_t dsIdx;
		for(dsIdx = 0; dsIdx < JSONDataSources.capacity; ++dsIdx)
		{
			JSONDataSource* ds = &JSONDataSources.array[dsIdx];
			if(ds->name != NULL)
				UnsubscribeJSONDataSource(ds);
		}

		JSONCommunicationStarted = false;
		EnableCmdLineInterface(&Console);
		return;
	}

	JSONDataSource* ds = (JSONDataSource*)dataSource;
	if(ds != NULL && JSONCommunicationStarted)
	{
		if(ds->dataAccessor != NULL && ds->name != NULL)
		{
			ds->sendNowFlag = true;

			Semaphore_post(PeriodicJSON_Sem);
		}
	}
}

//--------------------------------------------
// Subscribe JSON data input:
// Subscribe to incomming data corresponding
// to given input name and keys.
// The 'keys' array should contain 'DataCount'
// names of the data fields that will be
// populated as soon as corresponding JSON
// data is received.
//--------------------------------------------
JSONDataInput* SubscribeJSONDataInput(char* name, const char* keys[], uint32_t dataCount, DataValuesSetAccessor dataAccessor)
{
	if(!JSONDataSources.IsJSONDatasourcesArrayInitialized)
		InitializeJSONDataSourcesArray();
	if(!JSONDataInputs.IsJSONDatainputsArrayInitialized)
		InitializeJSONDataInputsArray();

	if(dataCount > MAX_DATA_COUNT)
	{
		Log_error1("Error: Too much data fields given to create a new datainput (please modify MAX_DATA_COUNT=%u if needed).", MAX_DATA_COUNT);
		ASSERT(false);
		return NULL;
	}

	if (JSONDataInputs.used == JSONDataInputs.capacity)
	{
		Log_error1("Error: Maximum JSON datainputs count reached (please modify MAX_DATAINPUT_COUNT=%u if needed).", MAX_DATAINPUT_COUNT);
		ASSERT(false);
		return NULL;
	}

	// Find a free instance of JSONDataInput structure
	uint32_t idx = 0;
	if(JSONDataInputs.used++ != 0)
		while(JSONDataInputs.array[idx].name != NULL)
			if(idx++ > JSONDataInputs.capacity)
			{
				Log_error1("Error: JSON datainputs counter or datainputs array corrrupted (MAX_DATAINPUT_COUNT=%u).", MAX_DATAINPUT_COUNT);
				ASSERT(FALSE);
				return NULL;
			}
	JSONDataInput* newInput = &JSONDataInputs.array[idx];

	newInput->name = name;
	newInput->keys = keys;
	newInput->dataCount = dataCount;
	newInput->dataAccessor = dataAccessor;

	return newInput;
}

//--------------------------------------------
// Unsubscribe JSON data input:
// Unsubscribes given data input.
// Returns false if the given data input wasn't
// subscribed, returns true otherwise.
//--------------------------------------------
bool UnsubscribeJSONDataInput(JSONDataInput* datainput)
{
	if(!JSONDataSources.IsJSONDatasourcesArrayInitialized)
		InitializeJSONDataSourcesArray();
	if(!JSONDataInputs.IsJSONDatainputsArrayInitialized)
		InitializeJSONDataInputsArray();

	if(datainput != NULL)
	{
		// Find specified datasource in JSON datasources array
		uint32_t idx = 0;
		while(&JSONDataInputs.array[idx] != datainput)
			if(idx++ > JSONDataInputs.capacity)
				return false;

		memset(datainput, NULL, sizeof(JSONDataInput));

		JSONDataInputs.used--;

		return true;
	}

	return false;
}
