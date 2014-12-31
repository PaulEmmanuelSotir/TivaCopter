/*
 * JSONCommunication.c
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

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
#include "driverlib/rom_map.h"
#include "driverlib/uart.h"
#include "driverlib/rom.h"
#include "driverlib/gpio.h"

#include "PinMap.h"
#include "Utils/UARTConsole.h"
#include "Utils/jsmn.h"
#include "JSONCommunication.h"

static bool JSONCommunicationStarted = false;
static bool JSONProgrammaticAccesMode = false;

//----------------------------------------
// UART console from 'main.c'
//----------------------------------------
extern UARTConsole Console;

//----------------------------------------
// Private dynamic JSON data sources array
//----------------------------------------
static struct
{
	JSONDataSource *array;
	uint32_t used;
	uint32_t size;
} JSONDataSources = {NULL, 0, 0};

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
		for(i = 0; i < JSONDataSources.used; ++i)
		{
			JSONDataSource* ds = &JSONDataSources.array[i];
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
	if(checkArgCount(&Console, argc, 2))
	{
		int i = JSONDataSources.used;
		while(i--)
		{
			if(strcmp(JSONDataSources.array[i].name, argv[1]) == 0)
			{
				JSONDataSources.array[i].enabled = true;
				UARTwrite(&Console, "JSON data source enabled.", 25);
				return;
			}
		}

		UARTwrite(&Console, "Wrong JSON data source name.", 28);
	}
}

//----------------------------------------
// disable:
// Enables specified JSON data source.
//----------------------------------------
void JSON_disable_cmd(int argc, char *argv[])
{
	if(checkArgCount(&Console, argc, 2))
	{
		int i = JSONDataSources.used;
		while(i--)
		{
			if(strcmp(JSONDataSources.array[i].name, argv[1]) == 0)
			{
				JSONDataSources.array[i].enabled = false;
				UARTwrite(&Console, "JSON data source disabled.", 26);
				return;
			}
		}

		UARTwrite(&Console, "Wrong JSON data source name.", 28);
	}
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
		JSONProgrammaticAccesMode = true;
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
		JSONProgrammaticAccesMode = false;
		UARTwrite(&Console, "Programmatic access mode disabled.", 34);
	}
}

//--------------------------------------------
// Suscribe data source:
// Creates a data source and add it to dynamic
// 'JSONDataSources' array.
// The 'keys' array should contain 'DataCount'
// names of the data fields provided by the
// datasource.
//--------------------------------------------
JSONDataSource* SuscribeJSONDataSource(char* name, const char* keys[], uint32_t dataCount)
{
	return SuscribePeriodicJSONDataSource(name, keys, dataCount, 0, NULL);
}

//--------------------------------------------
// Suscribe periodic data source:
// Creates a periodic data source and add it
// to dynamic 'JSONDataSources' array.
// The 'keys' array should contain 'DataCount'
// names of the data fields provided by the
// datasource.
// 'period' is the period of sending in RTOS
// clock ticks.
//--------------------------------------------
JSONDataSource* SuscribePeriodicJSONDataSource(char* name, const char* keys[], uint32_t dataCount, uint32_t period, DataValuesAccessor dataAccessor)
{
	// If there is no more available space in JSONDataSources array, we allocate more memory.
	if (JSONDataSources.used == JSONDataSources.size)
	{
		JSONDataSources.size += 4;
		JSONDataSources.array = (JSONDataSource *)realloc(JSONDataSources.array, JSONDataSources.size * sizeof(JSONDataSource));

		// Verify wether if any error occured during memory allocation.
		if (JSONDataSources.array == NULL)
		{
			JSONDataSources.size = 0;
			JSONDataSources.used = 0;
			free(JSONDataSources.array);
			Log_error0("Error (re)allocating memory for JSON datasources.");
			return NULL;
		}
	}

	JSONDataSource* newSource = &JSONDataSources.array[JSONDataSources.used++];
	newSource->name = name;
	newSource->keys = keys;
	newSource->dataCount = dataCount;
	newSource->enabled = false;
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
		clockParams.startFlag = TRUE;
		clockParams.arg = (UArg)newSource;
		if(Clock_create(PeriodicJSONDataSendingSwi, period, &clockParams, &eb) == NULL)
		{
			JSONDataSources.size = 0;
			JSONDataSources.used = 0;
			free(JSONDataSources.array);
			Log_error0("Error: Periodic data source clock creation failed.");
			return NULL;
		}
	}

	return newSource;
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

	if(JSONDataSources.used > 0)
	{
		uint32_t dsIdx;
		uint32_t valIdx;

		for(dsIdx = 0; dsIdx < JSONDataSources.used; ++dsIdx)
		{
			if(ds == &JSONDataSources.array[dsIdx])
			{
				if(ds->enabled)
				{
					UARTwrite(&Console, JSONProgrammaticAccesMode ? "\n{ " : "\n{\n", 3);

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

						UARTprintf(&Console, JSONProgrammaticAccesMode ? " \"%s\": \"%s\"%s " : "\t\"%s\": \"%s\"%s \n", key, value, comma);
					}

					UARTwrite(&Console, JSONProgrammaticAccesMode ? " }" : "\n}", 2);

					// Send data now to avoid data losses due to limited buffer size
					UARTFlushTx(&Console, false);

					return true;
				}
				// Datasource disabled (not an error)
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
	// Subsribe UART console commands for JSON communication
	bool sucess = true;
	sucess = sucess && SubscribeCmd(&Console, "listSources", 	JSON_list_sources_cmd, 	"List all available JSON data sources.");
	sucess = sucess && SubscribeCmd(&Console, "enable", 		JSON_enable_cmd, 		"Enables specified JSON data source's stream (only active once \'start\' have been called).");
	sucess = sucess && SubscribeCmd(&Console, "disable", 		JSON_disable_cmd, 		"Disables specified JSON data source's stream.");
	sucess = sucess && SubscribeCmd(&Console, "start", 			JSON_start_cmd, 		"Starts JSON communication.");
	sucess = sucess && SubscribeCmd(&Console, "progModeEn", 	JSON_enable_programatic_access_cmd, 	"Enables programmatic access mode. (newline means new JSON object)");
	sucess = sucess && SubscribeCmd(&Console, "progModeDis", 	JSON_disable_programatic_access_cmd, 	"Disables programmatic access mode.");
	if(!sucess)
	{
		Log_error0("Error (re)allocating memory for UART console command (from JSON API).");
		return;
	}

	while(1)
	{
		Semaphore_pend(PeriodicJSON_Sem, BIOS_WAIT_FOREVER);

		if(JSONDataSources.used > 0)
		{
			uint32_t dsIdx;
			uint32_t valIdx;

			// Send data from JSON datasources accessors if they are enabled and if their sendNowFlag is raised
			for(dsIdx = 0; dsIdx < JSONDataSources.used; ++dsIdx)
			{
				JSONDataSource* ds = &JSONDataSources.array[dsIdx];
				if(ds->sendNowFlag && ds->enabled)
				{
					UARTwrite(&Console, JSONProgrammaticAccesMode ? "\n{ " : "\n{\n", 3);

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

						UARTprintf(&Console, JSONProgrammaticAccesMode ? " \"%s\": \"%s\"%s " : "\t\"%s\": \"%s\"%s \n", key, value, comma);
					}

					UARTwrite(&Console, JSONProgrammaticAccesMode ? " }" : "\n}", 2);

					// Send data now to avoid data losses due to limited buffer size
					UARTFlushTx(&Console, false);

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
static void PeriodicJSONDataSendingSwi(UArg dataSource)
{
	if(IsAbortRequested(&Console))
	{
		JSONCommunicationStarted = false;
		EnableCmdLineInterface(&Console);
		return;
	}

	JSONDataSource* ds = (JSONDataSource*)dataSource;
	if(ds != NULL && JSONCommunicationStarted)
	{
		if(ds->dataAccessor != NULL)
		{
			ds->sendNowFlag = true;
			Semaphore_post(PeriodicJSON_Sem);
		}
	}
}

